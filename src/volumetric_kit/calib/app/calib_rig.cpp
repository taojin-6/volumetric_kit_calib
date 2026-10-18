// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/calib_rig.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#define GLFW_INCLUDE_NONE  // window and input only: gfx_glfw makes the surface
#include <GLFW/glfw3.h>

#include <glm/vec2.hpp>

#include "imgui.h"
#include "volumetric_kit/calib/app/capture_worker.hpp"
#include "volumetric_kit/calib/app/cloud_worker.hpp"
#include "volumetric_kit/calib/app/keys.hpp"
#include "volumetric_kit/calib/app/live_view.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/depth/depth_checks.hpp"
#include "volumetric_kit/calib/viz/board_editor.hpp"
#include "volumetric_kit/calib/viz/mosaic.hpp"
#include "volumetric_kit/calib/viz/overlay.hpp"
#include "volumetric_kit/calib/viz/panels.hpp"
#include "volumetric_kit/calib/viz/point_cloud.hpp"
#include "volumetric_kit/calib/viz/scene_view.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/shared_device.hpp"
#include "volumetric_kit/gfx/app/shared_device_config.hpp"
#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/core/device_requirements.hpp"
#include "volumetric_kit/gfx/core/retire_queue.hpp"
#include "volumetric_kit/gfx/core/texture_upload.hpp"
#include "volumetric_kit/gfx/glfw/imgui_glfw.hpp"
#include "volumetric_kit/gfx/glfw/shared_device.hpp"
#include "volumetric_kit/gfx/glfw/window.hpp"
#include "volumetric_kit/gfx/pipelines/depth_cloud_pipeline.hpp"
#include "volumetric_kit/gfx/pipelines/primitive_pipeline.hpp"
#include "volumetric_kit/gfx/ui/imgui_overlay.hpp"
#include "volumetric_kit/gfx/ui/metrics_panel.hpp"
#include "volumetric_kit/gfx/windowing/swapchain.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sensor.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"

namespace volumetric_kit::calib::app {

namespace {

namespace sensor = recon::sensor;
namespace glfw = gfx::glfw;
using Clock = std::chrono::steady_clock;

// Ctrl+C closes the window rather than ending the process, so the workers stop,
// the captures held are written and the cameras are released. The handler
// restores the default, so a second Ctrl+C ends a run that hangs.
// TODO: give close-on-interrupt one home (the core's base tier, or gfx_glfw)
// and port recon's rig_viewer, whose handler this repeats.
std::atomic<bool> g_interrupted{false};
static_assert(std::atomic<bool>::is_always_lock_free,
              "the interrupt flag must be safe to set from a signal handler");
extern "C" void on_interrupt(int signal) {
  g_interrupted.store(true);
  std::signal(signal, SIG_DFL);
}

// on_interrupt handles SIGINT and SIGTERM while it lives; the handlers it
// replaced are put back on every way out.
class InterruptHandlers {
 public:
  InterruptHandlers() noexcept {
    g_interrupted.store(false);
    previous_int_ = std::signal(SIGINT, on_interrupt);
    previous_term_ = std::signal(SIGTERM, on_interrupt);
  }
  ~InterruptHandlers() {
    if (previous_int_ != SIG_ERR) std::signal(SIGINT, previous_int_);
    if (previous_term_ != SIG_ERR) std::signal(SIGTERM, previous_term_);
  }
  InterruptHandlers(const InterruptHandlers&) = delete;
  InterruptHandlers& operator=(const InterruptHandlers&) = delete;

 private:
  void (*previous_int_)(int) = SIG_ERR;
  void (*previous_term_)(int) = SIG_ERR;
};

// How long a capture's flash stays, and how many messages the window keeps.
constexpr auto kFlash = std::chrono::milliseconds(250);
constexpr std::size_t kMessages = 8;

// What GLFW's callbacks hand the window's loop, all on the window's thread.
struct Input {
  KeyQueue keys;  // mapped once the frame knows whether ImGui takes text
  double scroll = 0.0;
};

void on_key(GLFWwindow* window, int key, int /*scancode*/, int action,
            int /*mods*/) {
  auto* input = static_cast<Input*>(glfwGetWindowUserPointer(window));
  if (input != nullptr) (void)input->keys.push(key, action);  // OOM drops it
}

void on_scroll(GLFWwindow* window, double /*dx*/, double dy) {
  auto* input = static_cast<Input*>(glfwGetWindowUserPointer(window));
  if (input != nullptr) input->scroll += dy;
}

// TODO: InputBinding, GlfwRun and WindowDeleter, the shared-device bring-up in
// run_window and its wheel-and-drag input for the mosaic and the 3D view
// (steer) are generic GLFW glue; move them to gfx_glfw in a gfx PR, then port
// gfx's examples, recon's viewers and calib_rig onto it.

// Points the window's callbacks at an Input for as long as it lives.
class InputBinding {
 public:
  InputBinding(GLFWwindow* window, Input& input) noexcept : window_(window) {
    glfwSetWindowUserPointer(window_, &input);
  }
  ~InputBinding() { glfwSetWindowUserPointer(window_, nullptr); }
  InputBinding(const InputBinding&) = delete;
  InputBinding& operator=(const InputBinding&) = delete;

 private:
  GLFWwindow* window_;
};

// GLFW, initialized for the run.
class GlfwRun {
 public:
  GlfwRun() : ok_(glfwInit() == GLFW_TRUE) {}
  ~GlfwRun() {
    if (ok_) glfwTerminate();
  }
  GlfwRun(const GlfwRun&) = delete;
  GlfwRun& operator=(const GlfwRun&) = delete;
  bool ok() const noexcept { return ok_; }

 private:
  bool ok_;
};

struct WindowDeleter {
  void operator()(GLFWwindow* window) const noexcept {
    glfwDestroyWindow(window);
  }
};

// --- The 3D view's settings, until the scenario has them.
// TODO: take the stride, colour mode, point size and the live sets' sampling
// from the core's calib_rig view settings once the scenario has them (CO5).
constexpr viz::ScenePlacement kPlacement = viz::ScenePlacement::Beside;
constexpr int kMaxStride = 8;
// The pointer's feel: radians of orbit per pixel dragged, and each wheel
// step's zoom, in the 3D view and the mosaic.
constexpr float kOrbitPerPixel = 0.005f;
constexpr float kSceneZoomStep = 0.9f;
constexpr float kMosaicZoomStep = 1.1f;

// What the window holds of the 3D view.
struct View3d {
  viz::ScenePlacement placement = kPlacement;
  bool frozen = false;
  // Its mode, stride and point size; the rest is set each frame.
  viz::CloudStyle style;
  viz::SceneOptions options;
  std::vector<viz::SceneCamera> cameras;  // in the rig's order
  viz::SceneView view;
  std::shared_ptr<const CloudSnapshot> latest;  // released before the worker
  std::shared_ptr<core::Buffer> lines;          // the scene's lines, uploaded
  std::uint32_t line_vertices = 0;
  bool lines_stale = true;
  float max_point_size = 1.0f;
  std::string last_error;  // the last failure said, said once

  // The placed cameras' poses.
  std::vector<recon::camera::Mat4d> placed() const {
    std::vector<recon::camera::Mat4d> poses;
    for (const viz::SceneCamera& camera : cameras) {
      if (camera.color_to_world) poses.push_back(*camera.color_to_world);
    }
    return poses;
  }

  // A snapshot frozen without colour -- prepared in the solid mode -- has none
  // to show, so the colour modes wait until the view is live again.
  bool colour_modes_open() const {
    return !frozen || latest == nullptr || latest->color;
  }
};

// Which view a drag began in, and how it moves it.
enum class Drag : std::uint8_t { None, Elsewhere, Mosaic, Orbit, Pan };

struct Pointer {
  Drag drag = Drag::None;
  glm::vec2 from{0.0f};
};

bool inside(const VkRect2D& rect, glm::vec2 point) {
  return point.x >= static_cast<float>(rect.offset.x) &&
         point.y >= static_cast<float>(rect.offset.y) &&
         point.x < static_cast<float>(rect.offset.x) +
                       static_cast<float>(rect.extent.width) &&
         point.y < static_cast<float>(rect.offset.y) +
                       static_cast<float>(rect.extent.height);
}

std::uint32_t tint_of(const float rgb[3]) {
  const auto byte = [](float v) {
    return static_cast<std::uint8_t>(
        std::lround(255.0f * std::clamp(v, 0.0f, 1.0f)));
  };
  return gfx::pipelines::pack_rgba8(byte(rgb[0]), byte(rgb[1]), byte(rgb[2]));
}

// What the window shows of the captures and the workers.
struct Notices {
  std::deque<std::string> messages;  // newest last
  viz::CaptureStatus capture;
  Clock::time_point flash_until{};
  int flashed = -1;

  void note(const std::string& message) {
    std::fprintf(stderr, "calib_rig: %s\n", message.c_str());
    messages.push_back(message);
    while (messages.size() > kMessages) messages.pop_front();
  }
};

// A 3D-view failure, which repeats each frame until its cause goes: said
// once, until another is.
void report_view(View3d& v, const core::Status& status, Notices& notices) {
  if (status.ok() || status.message() == v.last_error) return;
  v.last_error = status.message();
  notices.note("3D view: " + v.last_error);
}

// A press of Capture: refused, and said, once the live view has stopped.
void arm(LiveWorker& live, Notices& notices) {
  if (!live.arm()) notices.note("capture: the live view has stopped");
}

void drop_last(CaptureWorker& captures, Notices& notices) {
  const core::Status dropped = captures.drop_last();
  if (!dropped.ok()) notices.note(dropped.message());
}

// The keys polled before this frame, mapped with its WantTextInput.
void act_on_keys(KeyQueue& keys, bool want_text_input, LiveWorker& live,
                 CaptureWorker& captures, Notices& notices) {
  const core::Result<std::vector<KeyRequest>> requests =
      keys.take(want_text_input);
  if (!requests.ok()) {
    notices.note(requests.status().message());
    return;
  }
  for (const KeyRequest request : requests.value()) {
    if (request == KeyRequest::Capture) arm(live, notices);
    if (request == KeyRequest::DropLast) drop_last(captures, notices);
  }
}

// Everything a frame's UI reads.
struct FrameInputs {
  const std::vector<std::string>* ids = nullptr;
  std::optional<quality::SessionSummary> summary;
  viz::SaverQueue saver;
  std::string directory;
  core::StageMetrics stages;
};

// The capture's window: its buttons, then where it stands.
void capture_window(LiveWorker& live, CaptureWorker& captures, Notices& notices,
                    const FrameInputs& in) {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
  ImGui::Begin("Capture");
  if (ImGui::Button("Capture")) arm(live, notices);
  ImGui::SameLine();
  if (ImGui::Button("Drop last")) drop_last(captures, notices);
  notices.capture.decision = live.decision();
  const core::Result<std::vector<viz::PanelLine>> lines =
      viz::capture_panel(notices.capture);
  if (lines.ok()) viz::draw_panel_lines("capture", lines.value());
  ImGui::TextUnformatted(in.directory.empty() ? "session: none yet"
                                              : in.directory.c_str());
  ImGui::End();
}

void rig_windows(const LiveSnapshot& snapshot, const viz::BoardEditor& editor,
                 const FrameInputs& in) {
  const std::vector<std::string>& ids = *in.ids;
  ImGui::SetNextWindowPos(ImVec2(10, 200), ImGuiCond_FirstUseEver);
  ImGui::Begin("Rig");
  const core::Result<std::vector<viz::PanelLine>> set_lines =
      viz::set_panel(*snapshot.set, snapshot.stats, ids, in.saver);
  if (set_lines.ok()) viz::draw_panel_lines("set", set_lines.value());
  ImGui::End();

  ImGui::SetNextWindowPos(ImVec2(10, 420), ImGuiCond_FirstUseEver);
  ImGui::Begin("Cameras");
  const core::Result<std::vector<viz::CameraLive>> cameras =
      camera_lives(snapshot);
  for (std::size_t i = 0; cameras.ok() && i < ids.size(); ++i) {
    if (!ImGui::CollapsingHeader(ids[i].c_str(),
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
      continue;
    }
    const quality::CameraSummary none;
    const quality::CameraSummary& session =
        in.summary ? in.summary->cameras[i] : none;
    const core::Result<std::vector<viz::PanelLine>> lines =
        viz::camera_panel(cameras.value()[i], session, editor.board());
    ImGui::PushID(static_cast<int>(i));
    if (lines.ok()) viz::draw_panel_lines("camera", lines.value());
    if (!snapshot.errors[i].ok()) {
      ImGui::TextWrapped("%s", snapshot.errors[i].message().c_str());
    }
    ImGui::PopID();
  }
  ImGui::End();

  if (in.summary) {
    const core::Result<viz::SessionPanel> panel =
        viz::session_panel(*in.summary, ids);
    if (panel.ok()) {
      ImGui::Begin("Session");
      viz::draw_session_panel(panel.value());
      ImGui::End();
    }
  }

  const core::Result<gfx::FrameMetrics> metrics =
      viz::stage_frame_metrics(in.stages, snapshot.sets_per_second);
  if (metrics.ok()) gfx::ui::draw_metrics_panel(metrics.value(), "Stages");
}

// Each tile's marks, behind every ImGui window.
void draw_marks(const viz::RigMosaic& mosaic, const LiveSnapshot& snapshot,
                const FrameInputs& in) {
  const quality::SessionSummary* session = in.summary ? &*in.summary : nullptr;
  const core::Result<std::vector<viz::TileOverlay>> tiles =
      tile_overlays(snapshot, *in.ids, session);
  if (!tiles.ok()) return;
  for (std::size_t i = 0; i < tiles.value().size(); ++i) {
    const core::Result<viz::TileMarks> marks =
        viz::tile_marks(tiles.value()[i]);
    if (marks.ok()) {
      viz::draw_tile_marks(mosaic.view(), mosaic.tile_region(i), marks.value());
    }
  }
}

void flash(const Notices& notices) {
  const Clock::time_point now = Clock::now();
  if (now >= notices.flash_until) return;
  const float left =
      std::chrono::duration<float>(notices.flash_until - now).count() /
      std::chrono::duration<float>(kFlash).count();
  const ImVec2 size = ImGui::GetIO().DisplaySize;
  ImDrawList* list = ImGui::GetForegroundDrawList();
  list->AddRectFilled(ImVec2(0, 0), size,
                      IM_COL32(255, 255, 255, static_cast<int>(96 * left)));
  const std::string text = "capture " + std::to_string(notices.flashed);
  list->AddText(ImGui::GetFont(), 48.0f, ImVec2(24, size.y - 72),
                IM_COL32(0, 200, 0, 255), text.c_str());
}

// What the workers reported since the last frame.
void take_reports(LiveWorker& live, CloudWorker& cloud, CaptureWorker& captures,
                  Notices& notices) {
  for (const core::Status& error : live.take_errors()) {
    notices.note(error.message());
  }
  for (const core::Status& error : cloud.take_errors()) {
    notices.note(error.message());
  }
  for (const core::Status& error : captures.take_errors()) {
    notices.note(error.message());
  }
  for (const CaptureEvent& event : captures.take_events()) {
    switch (event.kind) {
      case CaptureEvent::Kind::Saved:
        notices.capture.last = event.meta;
        notices.capture.last_usable_views = event.usable_views;
        notices.capture.last_flagged = event.flagged;
        notices.capture.error.clear();
        notices.flash_until = Clock::now() + kFlash;
        notices.flashed = event.meta.index;
        break;
      case CaptureEvent::Kind::Dropped:
        notices.note("capture " + std::to_string(event.meta.index) +
                     " dropped");
        break;
      case CaptureEvent::Kind::Failed:
        notices.capture.error = event.status.message();
        notices.note(event.status.message());
        break;
    }
  }
}

// The 3D view's window: where it sits, freeze, colouring, and each camera's
// toggle and tint; then the worker told what the view needs.
void view_window(View3d& v, CloudWorker& cloud,
                 const std::vector<std::string>& ids, Notices& notices) {
  ImGui::SetNextWindowPos(ImVec2(10, 760), ImGuiCond_FirstUseEver);
  ImGui::Begin("3D view");
  int placement = static_cast<int>(v.placement);
  ImGui::RadioButton("hidden", &placement,
                     static_cast<int>(viz::ScenePlacement::Hidden));
  ImGui::SameLine();
  ImGui::RadioButton("beside", &placement,
                     static_cast<int>(viz::ScenePlacement::Beside));
  ImGui::SameLine();
  ImGui::RadioButton("full window", &placement,
                     static_cast<int>(viz::ScenePlacement::FullWindow));
  v.placement = static_cast<viz::ScenePlacement>(placement);
  ImGui::Checkbox("freeze", &v.frozen);

  using gfx::pipelines::CloudColor;
  const bool open = v.colour_modes_open();
  if (!open) v.style.mode = CloudColor::Solid;
  constexpr const char* kModes[] = {"solid", "tinted luma", "true colour"};
  if (ImGui::BeginCombo("colour", kModes[static_cast<int>(v.style.mode)])) {
    for (int m = 0; m < 3; ++m) {
      ImGui::BeginDisabled(m != 0 && !open);
      if (ImGui::Selectable(kModes[m], m == static_cast<int>(v.style.mode))) {
        v.style.mode = static_cast<CloudColor>(m);
      }
      ImGui::EndDisabled();
    }
    ImGui::EndCombo();
  }
  int stride = static_cast<int>(v.style.stride);
  ImGui::SliderInt("stride", &stride, 1, kMaxStride);
  v.style.stride =
      static_cast<std::uint32_t>(std::clamp(stride, 1, kMaxStride));
  ImGui::SliderFloat("point size", &v.style.point_size, 1.0f,
                     std::max(1.0f, v.max_point_size), "%.0f px");
  if (ImGui::Checkbox("depth frusta", &v.options.depth_frusta)) {
    v.lines_stale = true;
  }
  if (ImGui::Button("reset view")) {
    const core::Status framed = v.view.frame(v.placed());
    if (!framed.ok()) notices.note(framed.message());
  }
  // TODO: name each placed camera over the 3D view once gfx has a
  // world-anchored overlay (a world point to ImGui display coordinates
  // through view_proj and the viewport, framebuffer scale included), the 3D
  // counterpart of ImageOverlay; until then this list names them.
  for (std::size_t i = 0; i < v.cameras.size() && i < ids.size(); ++i) {
    viz::SceneCamera& camera = v.cameras[i];
    ImGui::PushID(static_cast<int>(i));
    if (ImGui::Checkbox(ids[i].c_str(), &camera.shown)) v.lines_stale = true;
    ImGui::SameLine();
    const std::uint32_t t = camera.tint;
    float rgb[3] = {static_cast<float>(t & 0xFFu) / 255.0f,
                    static_cast<float>((t >> 8) & 0xFFu) / 255.0f,
                    static_cast<float>((t >> 16) & 0xFFu) / 255.0f};
    if (ImGui::ColorEdit3(
            "tint", rgb,
            ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel)) {
      camera.tint = tint_of(rgb);
      v.lines_stale = true;
    }
    if (!camera.color_to_world) {
      ImGui::SameLine();
      ImGui::TextDisabled("not placed until the rig is solved");
    }
    ImGui::PopID();
  }
  ImGui::End();
  cloud.set_live(cloud_live(v.frozen, v.placement, false));
  cloud.set_color(v.style.mode != CloudColor::Solid);
}

// The wheel and drags of this frame, to the view under the pointer: the
// mosaic pans and zooms; the 3D view orbits (left), pans (right) and dollies.
void steer(GLFWwindow* window, Input& input, Pointer& pointer, bool imgui_mouse,
           const viz::WindowSplit& split, gfx::camera::ImageView2D& mosaic,
           viz::SceneView& scene) {
  const glm::vec2 cursor = glfw::cursor_in_framebuffer(window);
  const auto scroll = static_cast<float>(std::exchange(input.scroll, 0.0));
  if (imgui_mouse) {
    pointer.drag = Drag::None;
    return;
  }
  const bool over_scene = inside(split.scene, cursor);
  const bool over_mosaic = inside(split.mosaic, cursor);
  if (scroll != 0.0f && over_scene) {
    scene.orbit().zoom(std::pow(kSceneZoomStep, scroll));
  } else if (scroll != 0.0f && over_mosaic) {
    mosaic.zoom_about(cursor, std::pow(kMosaicZoomStep, scroll));
  }
  const bool left =
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  const bool right =
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  if (!left && !right) {
    pointer.drag = Drag::None;
    return;
  }
  if (pointer.drag == Drag::None) {
    pointer.drag = over_scene            ? (left ? Drag::Orbit : Drag::Pan)
                   : over_mosaic && left ? Drag::Mosaic
                                         : Drag::Elsewhere;
    pointer.from = cursor;
    return;
  }
  const glm::vec2 delta = cursor - pointer.from;
  pointer.from = cursor;
  gfx::camera::OrbitCamera& orbit = scene.orbit();
  switch (pointer.drag) {
    case Drag::Mosaic:
      mosaic.pan(delta);
      break;
    case Drag::Orbit:
      orbit.orbit(-kOrbitPerPixel * delta.x, kOrbitPerPixel * delta.y);
      break;
    case Drag::Pan: {
      // A pixel's width at the target's distance, so the scene follows the
      // pointer.
      const float per_pixel =
          2.0f * orbit.distance() * std::tan(0.5f * viz::SceneView::kFovY) /
          std::max(1.0f, static_cast<float>(split.scene.extent.height));
      orbit.pan(-delta.x * per_pixel, delta.y * per_pixel);
      break;
    }
    case Drag::None:
    case Drag::Elsewhere:
      break;
  }
}

// The scene's lines uploaded again after a change: the old buffer is freed
// once frame `frame_number`, which no longer draws it, retires.
core::Status refresh_lines(View3d& v, gfx::app::WindowedApp& app,
                           gfx::RetireQueue& retire,
                           std::uint64_t frame_number) {
  if (!v.lines_stale) return {};
  // TODO: CA12b passes the solved boards, and their board, once Calibrate
  // places them.
  VKC_ASSIGN(const std::vector<gfx::pipelines::ColorVertex> vertices,
             viz::scene_lines(v.cameras, nullptr, {}, v.options));
  std::shared_ptr<core::Buffer> fresh;
  if (!vertices.empty()) {
    gfx::BufferUploadDesc upload;
    upload.data = vertices.data();
    upload.size = vertices.size() * sizeof(gfx::pipelines::ColorVertex);
    upload.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    VKC_ASSIGN(core::Buffer buffer,
               gfx::upload_buffer(app.device(), app.allocator(), upload));
    fresh = std::make_shared<core::Buffer>(std::move(buffer));
  }
  if (v.lines) retire.push(frame_number, [old = v.lines] { (void)old; });
  v.lines = std::move(fresh);
  v.line_vertices = static_cast<std::uint32_t>(vertices.size());
  v.lines_stale = false;
  return {};
}

// The 3D view's points and lines, inside the rendering scope; the frame
// waits for the points' preparation at the vertex stage.
core::Status draw_scene(View3d& v, gfx::pipelines::DepthCloudPipeline& clouds,
                        const gfx::pipelines::PrimitivePipeline& primitives,
                        gfx::RetireQueue& retire, gfx::windowing::Frame& frame,
                        VkExtent2D extent, VkRect2D viewport,
                        const core::SharedDevice& shared) {
  const glm::mat4 view_proj = v.view.view_proj(viewport);
  viz::CloudStyle style = v.style;
  style.viewport = viewport;
  style.view_proj = view_proj;
  style.producer_family = shared.compute_family();
  style.renderer_family = shared.graphics_family();
  VKC_ASSIGN(const std::optional<core::TimelinePoint> wait,
             record_cloud(clouds, retire, frame.cmd, extent, frame.number,
                          v.latest, v.cameras, style));
  if (wait) frame.waits.push_back({*wait, VK_PIPELINE_STAGE_VERTEX_SHADER_BIT});
  if (v.lines != nullptr && v.line_vertices > 0) {
    gfx::pipelines::PrimitiveDraw lines;
    lines.topology = gfx::pipelines::PrimitiveTopology::Lines;
    lines.vertices = v.lines.get();
    lines.vertex_count = v.line_vertices;
    lines.viewport = viewport;
    lines.view_proj = view_proj;
    VKC_TRY(primitives.submit(frame.cmd, {extent, &lines, 1}));
  }
  return {};
}

// Stops the workers, waits for the renderer, and frees what the frames held,
// on every way out of the loop.
class Teardown {
 public:
  Teardown(LiveWorker& live, CloudWorker& cloud, gfx::app::WindowedApp& app,
           gfx::RetireQueue& retire) noexcept
      : live_(live), cloud_(cloud), app_(app), retire_(retire) {}
  ~Teardown() {
    live_.stop();
    cloud_.stop();
    const core::Status idle = app_.wait_idle();
    if (!idle.ok()) {
      std::fprintf(stderr, "calib_rig: %s\n", idle.message().c_str());
    }
    retire_.reclaim();  // idle, or lost: either way nothing reads them
  }
  Teardown(const Teardown&) = delete;
  Teardown& operator=(const Teardown&) = delete;

 private:
  LiveWorker& live_;
  CloudWorker& cloud_;
  gfx::app::WindowedApp& app_;
  gfx::RetireQueue& retire_;
};

core::Status run_window(GLFWwindow* window, const core::config::Config& config,
                        const RigScenario& scenario) {
  const RigPreset& preset = scenario.preset;
  Notices notices;
  // --- One VkDevice: gfx draws on its graphics side, recon decodes, reads
  // back and prepares frames on its compute side. Wide points where the GPU
  // has them, one-pixel points where it does not.
  gfx::app::WindowedAppConfig app_config;
  app_config.app_name = "calib_rig";
  app_config.swapchain.extent = glfw::framebuffer_extent(window);
  app_config.swapchain.depth_format = VK_FORMAT_D32_SFLOAT;  // the 3D view's
  app_config.frames_in_flight = 2;
  gfx::app::WindowDeviceConfig device_config;
  device_config.instance.app_name = app_config.app_name;
  device_config.instance.enable_validation = config.window.validation;
  VKC_ASSIGN(RigRequirements needs, rig_requirements(true));
  device_config.graphics = needs.graphics;
  core::Result<std::unique_ptr<core::SharedDevice>> made =
      glfw::make_shared_device(window, needs.compute, device_config);
  if (!made.ok() && made.status().domain() == core::Status::Code::Unsupported) {
    VKC_ASSIGN(needs, rig_requirements(false));
    device_config.graphics = needs.graphics;
    made = glfw::make_shared_device(window, needs.compute, device_config);
    if (made.ok()) {
      notices.note(
          "3D view: points are one pixel wide: the GPU has no "
          "largePoints");
    }
  }
  if (!made.ok()) return made.status();
  const std::unique_ptr<core::SharedDevice> shared = std::move(made).value();
  app_config.device = needs.graphics;
  VKC_ASSIGN(gfx::app::WindowedApp app,
             gfx::app::WindowedApp::adopt(
                 shared->graphics_payload(), app_config,
                 [&shared](VkInstance) -> core::Result<VkSurfaceKHR> {
                   return shared->release_surface();
                 }));
  VKC_ASSIGN(core::Device recon_device,
             core::Device::adopt(shared->compute_payload(), needs.compute));
  VKC_ASSIGN(
      core::Allocator recon_allocator,
      core::Allocator::create(shared->instance().handle(), recon_device));

  // --- The rig, at the preset.
  // TODO: open the rig from scenario.sync once recon's open_orbbec_sensors
  // takes a parsed sync configuration, so the file is read once.
  VKC_ASSIGN(auto sensors,
             sensor::open_orbbec_sensors(
                 config.source.sync_config,
                 orbbec_options(
                     preset, recon_device, recon_allocator,
                     {shared->compute_family(), shared->graphics_family()})));
  std::vector<sensor::SensorInfo> infos;
  for (const auto& camera : sensors) infos.push_back(camera->info());
  VKC_TRY(check_preset(infos, preset));
  VKC_ASSIGN(sensor::SensorArray array,
             sensor::SensorArray::open(std::move(sensors), {}));

  CaptureSetup setup;
  setup.root = config.output.path;
  setup.software = app_software();
  setup.sync_path = config.source.sync_config;
  setup.reference = array.primary();
  setup.depth_mode = preset.depth_mode;
  std::vector<recon::camera::ImageSize> sizes;
  std::vector<viz::SceneCamera> scene_cameras;
  for (std::size_t i = 0; i < array.size(); ++i) {
    VKC_ASSIGN(capture::SessionCamera camera,
               session_camera(array.sensor(i), scenario.sync));
    sizes.push_back(camera.info.color->size);
    // Before the rig is solved only its reference, the world's origin, is
    // placed.
    // TODO: CA12b places every camera at Calibrate's solved poses.
    viz::SceneCamera scene_camera;
    scene_camera.color = *camera.info.color;
    scene_camera.depth = camera.info.depth;
    scene_camera.depth_to_color = camera.info.depth_to_color;
    if (i == array.primary()) {
      scene_camera.color_to_world = recon::camera::Mat4d(1.0);
    }
    // TODO: give each camera its own default tint from gfx's categorical
    // palette once gfx has one; until then every tint starts white.
    scene_cameras.push_back(std::move(scene_camera));
    setup.cameras.push_back(std::move(camera));
  }
  // TODO: cap cv::setNumThreads once H4 measures the contention between
  // OpenCV, the SDK and the decode threads.
  VKC_ASSIGN(std::unique_ptr<CaptureWorker> captures,
             CaptureWorker::create(std::move(setup)));
  VKC_ASSIGN(viz::BoardEditor editor,
             viz::BoardEditor::create(legacy_femto_board()));
  // The 3D view's worker, prepared frames shared with the renderer's family.
  VKC_ASSIGN(std::unique_ptr<CloudWorker> cloud,
             CloudWorker::create(
                 recon_device, recon_allocator, array.size(),
                 {shared->compute_family(), shared->graphics_family()}));

  // --- The window's side.
  VKC_ASSIGN(std::unique_ptr<viz::RigMosaic> mosaic,
             viz::RigMosaic::create(app.device(), app.allocator(),
                                    app.swapchain().layout(), sizes));
  gfx::pipelines::DepthCloudPipelineDesc cloud_desc;
  cloud_desc.max_draws = static_cast<std::uint32_t>(scene_cameras.size());
  VKC_ASSIGN(std::unique_ptr<gfx::pipelines::DepthCloudPipeline> clouds,
             gfx::pipelines::DepthCloudPipeline::create(
                 app.device(), app.allocator(), app.swapchain().layout(),
                 app.frame_loop().timeline(), cloud_desc));
  VKC_ASSIGN(std::unique_ptr<gfx::pipelines::PrimitivePipeline> primitives,
             gfx::pipelines::PrimitivePipeline::create(
                 app.device(), app.swapchain().layout()));
  gfx::RetireQueue retire(app.frame_loop().timeline());
  View3d view3d;  // its snapshot released before the retire queue and cloud
  view3d.cameras = std::move(scene_cameras);
  view3d.max_point_size = clouds->max_point_size();
  VKC_TRY(view3d.view.frame(view3d.placed()));

  // GLFW's callbacks before ImGui's backend, which chains to them, so the keys
  // reach the app first.
  Input input;
  const InputBinding binding(window, input);
  glfwSetKeyCallback(window, on_key);
  glfwSetScrollCallback(window, on_scroll);
  gfx::ui::ImGuiOverlayConfig overlay_config;
  overlay_config.layout = app.swapchain().layout();
  overlay_config.min_image_count = app.swapchain().image_count();
  overlay_config.image_count = app.swapchain().image_count();
  VKC_ASSIGN(gfx::ui::ImGuiOverlay overlay,
             gfx::ui::ImGuiOverlay::create(app.device(), app.instance_handle(),
                                           overlay_config));
  ImGui::SetCurrentContext(overlay.context());
  VKC_ASSIGN(std::unique_ptr<glfw::ImGuiGlfw> platform,
             glfw::ImGuiGlfw::create(window, overlay.context()));

  // --- The live worker last, as its thread starts the cameras: from here
  // every way out stops it, then the 3D view's worker (Teardown), so no set
  // is held when the array closes.
  VKC_ASSIGN(std::unique_ptr<LiveWorker> live,
             LiveWorker::create(std::move(array), recon_device, recon_allocator,
                                editor.board(), editor.square_measured(),
                                captures.get(), cloud.get()));
  const std::vector<std::string>& ids = live->ids();
  std::shared_ptr<const LiveSnapshot> latest;  // released before live
  const Teardown teardown(*live, *cloud, app, retire);
  bool fresh = false;
  bool fitted = false;
  Pointer pointer;
  while (glfwWindowShouldClose(window) == GLFW_FALSE && !g_interrupted.load()) {
    glfwPollEvents();
    if (std::shared_ptr<const LiveSnapshot> s = live->take_snapshot()) {
      latest = std::move(s);
      fresh = true;
    }
    if (std::shared_ptr<const CloudSnapshot> s = cloud->take_snapshot()) {
      view3d.latest = std::move(s);
    }
    take_reports(*live, *cloud, *captures, notices);
    if (live->captures() > 0 && !editor.locked()) {
      (void)editor.edit(live->board(), live->square_measured());
      editor.lock();
    }

    VKC_ASSIGN(std::optional<gfx::windowing::Frame> next,
               app.begin_frame(glfw::framebuffer_size(window)));
    if (!next) {  // minimized: nothing is drawn, so nothing is prepared
      cloud->set_live(cloud_live(view3d.frozen, view3d.placement, true));
      glfwWaitEventsTimeout(0.05);
      continue;
    }
    gfx::windowing::Frame& frame = *next;
    retire.poll();
    const VkExtent2D extent = frame.target->extent();
    const viz::WindowSplit split = viz::window_split(view3d.placement, extent);
    gfx::camera::ImageView2D& view = mosaic->view();
    view.set_viewport(
        glm::vec2(static_cast<float>(split.mosaic.offset.x),
                  static_cast<float>(split.mosaic.offset.y)),
        glm::vec2(static_cast<float>(split.mosaic.extent.width),
                  static_cast<float>(split.mosaic.extent.height)));
    if (!fitted) {
      view.fit();
      fitted = true;
    }

    FrameInputs in;
    in.ids = &ids;
    in.summary = captures->summary();
    in.saver = {captures->pending(), captures->capacity(), captures->full()};
    in.directory = captures->directory();
    if (latest) in.stages = latest->stages;
    if (view3d.latest) in.stages.merge(view3d.latest->stages);
    if (fresh && latest) {
      core::StageScope scope(in.stages, "tiles");
      VKC_TRY(
          record_snapshot(*mosaic, retire, frame.cmd, latest, frame.number));
      for (const viz::TileRefusal& refusal : mosaic->refused()) {
        notices.note("camera " + ids[refusal.camera] + ": " +
                     refusal.status.message());
      }
      fresh = false;
    }

    platform->new_frame();
    overlay.new_frame();
    const ImGuiIO& io = ImGui::GetIO();
    act_on_keys(input.keys, io.WantTextInput, *live, *captures, notices);
    steer(window, input, pointer, io.WantCaptureMouse, split, view,
          view3d.view);

    capture_window(*live, *captures, notices, in);
    view_window(view3d, *cloud, ids, notices);
    ImGui::SetNextWindowPos(ImVec2(10, 640), ImGuiCond_FirstUseEver);
    ImGui::Begin("Board");
    const Target before = editor.board();
    const bool measured_before = editor.square_measured();
    const core::Status edited = viz::draw_board_editor(editor);
    if (!edited.ok()) notices.note(edited.message());
    if (editor.board() != before ||
        editor.square_measured() != measured_before) {
      const core::Status taken =
          live->set_board(editor.board(), editor.square_measured());
      if (!taken.ok()) notices.note(taken.message());
    }
    ImGui::End();
    const bool mosaic_shown = split.mosaic.extent.width > 0;
    const bool scene_shown = split.scene.extent.width > 0;
    if (scene_shown) {
      report_view(view3d, refresh_lines(view3d, app, retire, frame.number),
                  notices);
    }
    if (latest && mosaic_shown) draw_marks(*mosaic, *latest, in);

    gfx::RenderTargetBeginInfo begin;
    frame.target->begin(frame.cmd, begin);
    if (mosaic_shown) mosaic->draw(frame.cmd, extent);
    if (scene_shown) {
      core::StageScope scope(in.stages, "point cloud");
      report_view(view3d,
                  draw_scene(view3d, *clouds, *primitives, retire, frame,
                             extent, split.scene, *shared),
                  notices);
    }
    if (latest) rig_windows(*latest, editor, in);
    if (!notices.messages.empty()) {
      ImGui::Begin("Messages");
      for (const std::string& message : notices.messages) {
        ImGui::TextWrapped("%s", message.c_str());
      }
      ImGui::End();
    }
    flash(notices);
    overlay.render(frame.cmd);
    frame.target->end(frame.cmd);
    const core::Status ended = app.end_frame(frame);
    if (!ended.ok() && !gfx::windowing::swapchain_stale(ended)) {
      mosaic->discard();
      return ended;
    }
  }
  return {};
}

}  // namespace

core::Result<RigRequirements> rig_requirements(bool large_points) {
  return compute::guarded(
      "rig_requirements", [&]() -> core::Result<RigRequirements> {
        RigRequirements needs;
        VKC_ASSIGN(needs.compute,
                   core::merge(recon::device_requirements(),
                               depth::DepthChecks::device_requirements()));
        needs.graphics = gfx::device_requirements();
        needs.graphics.features.largePoints = large_points ? VK_TRUE : VK_FALSE;
        return needs;
      });
}

core::Result<RigScenario> check_scenario(const core::config::Config& config) {
  return compute::guarded("check_scenario", [&]() -> core::Result<RigScenario> {
    RigScenario scenario;
    VKC_ASSIGN(scenario.preset, rig_preset(config.source));
    std::error_code ec;
    if (!std::filesystem::is_directory(config.output.path, ec)) {
      return core::Status::not_found("output.path: " + config.output.path +
                                     " is not an existing directory");
    }
    // recon refuses a file that lists no camera.
    core::Result<sensor::OrbbecRigSyncConfig> sync =
        sensor::read_orbbec_sync_config(config.source.sync_config);
    if (!sync.ok()) return sync.status().with_context("source.sync_config");
    scenario.sync = std::move(sync).value();
    return scenario;
  });
}

core::Status run_calib_rig(const core::config::Config& config) {
  return compute::guarded("calib_rig", [&]() -> core::Status {
    VKC_ASSIGN(const RigScenario scenario, check_scenario(config));
    const InterruptHandlers interrupts;
    const GlfwRun glfw_run;
    if (!glfw_run.ok()) {
      return core::Status::unsupported("calib_rig: GLFW will not initialize");
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    const std::unique_ptr<GLFWwindow, WindowDeleter> window(
        glfwCreateWindow(config.window.width, config.window.height, "calib_rig",
                         nullptr, nullptr));
    if (window == nullptr) {
      return core::Status::unsupported(
          "calib_rig: GLFW will not open a window");
    }
    return run_window(window.get(), config, scenario);
  });
}

}  // namespace volumetric_kit::calib::app
