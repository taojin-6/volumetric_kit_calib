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
#include "volumetric_kit/calib/app/keys.hpp"
#include "volumetric_kit/calib/app/live_view.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/viz/board_editor.hpp"
#include "volumetric_kit/calib/viz/mosaic.hpp"
#include "volumetric_kit/calib/viz/overlay.hpp"
#include "volumetric_kit/calib/viz/panels.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/shared_device.hpp"
#include "volumetric_kit/gfx/app/shared_device_config.hpp"
#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/core/retire_queue.hpp"
#include "volumetric_kit/gfx/glfw/imgui_glfw.hpp"
#include "volumetric_kit/gfx/glfw/shared_device.hpp"
#include "volumetric_kit/gfx/glfw/window.hpp"
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
std::atomic<bool> g_interrupted{false};
static_assert(std::atomic<bool>::is_always_lock_free,
              "the interrupt flag must be safe to set from a signal handler");
extern "C" void on_interrupt(int signal) {
  g_interrupted.store(true);
  std::signal(signal, SIG_DFL);
}

// How long a capture's flash stays, and how many messages the window keeps.
constexpr auto kFlash = std::chrono::milliseconds(250);
constexpr std::size_t kMessages = 8;

// What GLFW's callbacks hand the window's loop, all on the window's thread.
struct Input {
  bool want_text_input = false;  // ImGui's, as of the last frame
  std::vector<KeyRequest> requests;
  double scroll = 0.0;
};

void on_key(GLFWwindow* window, int key, int /*scancode*/, int action,
            int /*mods*/) {
  auto* input = static_cast<Input*>(glfwGetWindowUserPointer(window));
  if (input == nullptr) return;
  const KeyRequest request = key_request(key, action, input->want_text_input);
  if (request != KeyRequest::None) input->requests.push_back(request);
}

void on_scroll(GLFWwindow* window, double /*dx*/, double dy) {
  auto* input = static_cast<Input*>(glfwGetWindowUserPointer(window));
  if (input != nullptr) input->scroll += dy;
}

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

bool same_board(const Target& a, const Target& b) {
  return a.kind == b.kind && a.rows == b.rows && a.cols == b.cols &&
         a.square_size_m == b.square_size_m &&
         a.marker_size_m == b.marker_size_m && a.dictionary == b.dictionary &&
         a.first_marker_id == b.first_marker_id &&
         a.legacy_pattern == b.legacy_pattern &&
         a.square_size_tolerance_m == b.square_size_tolerance_m &&
         a.planarity_bow_m == b.planarity_bow_m;
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
  if (ImGui::Button("Capture")) live.arm();
  ImGui::SameLine();
  if (ImGui::Button("Drop last")) {
    const core::Status dropped = captures.drop_last();
    if (!dropped.ok()) notices.note(dropped.message());
  }
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
void take_reports(LiveWorker& live, CaptureWorker& captures, Notices& notices) {
  for (const core::Status& error : live.take_errors()) {
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

// Stops the live worker, waits for the renderer, and frees what the frames
// held, on every way out of the loop.
class Teardown {
 public:
  Teardown(LiveWorker& live, gfx::app::WindowedApp& app,
           gfx::RetireQueue& retire) noexcept
      : live_(live), app_(app), retire_(retire) {}
  ~Teardown() {
    live_.stop();
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
  gfx::app::WindowedApp& app_;
  gfx::RetireQueue& retire_;
};

core::Status run_window(GLFWwindow* window, const core::config::Config& config,
                        const RigPreset& preset) {
  // --- One VkDevice: gfx draws on its graphics side, recon decodes and reads
  // back on its compute side.
  gfx::app::WindowedAppConfig app_config;
  app_config.app_name = "calib_rig";
  app_config.swapchain.extent = glfw::framebuffer_extent(window);
  app_config.frames_in_flight = 2;
  gfx::app::WindowDeviceConfig device_config;
  device_config.instance.app_name = app_config.app_name;
  device_config.instance.enable_validation = config.window.validation;
  device_config.graphics = app_config.device;
  VKC_ASSIGN(std::unique_ptr<core::SharedDevice> shared,
             glfw::make_shared_device(window, recon::device_requirements(),
                                      device_config));
  VKC_ASSIGN(gfx::app::WindowedApp app,
             gfx::app::WindowedApp::adopt(
                 shared->graphics_payload(), app_config,
                 [&shared](VkInstance) -> core::Result<VkSurfaceKHR> {
                   return shared->release_surface();
                 }));
  VKC_ASSIGN(core::Device recon_device,
             core::Device::adopt(shared->compute_payload(),
                                 recon::device_requirements()));
  VKC_ASSIGN(
      core::Allocator recon_allocator,
      core::Allocator::create(shared->instance().handle(), recon_device));

  // --- The rig, at the preset.
  VKC_ASSIGN(auto sensors,
             sensor::open_orbbec_sensors(
                 config.source.sync_config,
                 orbbec_options(
                     preset, recon_device, recon_allocator,
                     {shared->compute_family(), shared->graphics_family()})));
  std::vector<sensor::SensorInfo> infos;
  for (const auto& camera : sensors) infos.push_back(camera->info());
  VKC_TRY(check_preset(infos, preset));
  VKC_ASSIGN(const sensor::OrbbecRigSyncConfig sync,
             sensor::read_orbbec_sync_config(config.source.sync_config));
  VKC_ASSIGN(sensor::SensorArray array,
             sensor::SensorArray::open(std::move(sensors), {}));

  CaptureSetup setup;
  setup.root = config.output.path;
  setup.software = app_software();
  setup.sync_path = config.source.sync_config;
  setup.reference = array.primary();
  setup.depth_mode = preset.depth_mode;
  std::vector<recon::camera::ImageSize> sizes;
  for (std::size_t i = 0; i < array.size(); ++i) {
    VKC_ASSIGN(capture::SessionCamera camera,
               session_camera(array.sensor(i), sync));
    sizes.push_back(camera.info.color->size);
    setup.cameras.push_back(std::move(camera));
  }
  VKC_ASSIGN(std::unique_ptr<CaptureWorker> captures,
             CaptureWorker::create(std::move(setup)));
  VKC_ASSIGN(viz::BoardEditor editor,
             viz::BoardEditor::create(legacy_femto_board()));
  VKC_ASSIGN(std::unique_ptr<LiveWorker> live,
             LiveWorker::create(std::move(array), recon_device, recon_allocator,
                                editor.board(), editor.square_measured(),
                                captures.get()));
  const std::vector<std::string>& ids = live->ids();

  // --- The window's side.
  VKC_ASSIGN(std::unique_ptr<viz::RigMosaic> mosaic,
             viz::RigMosaic::create(app.device(), app.allocator(),
                                    app.swapchain().layout(), sizes));
  gfx::RetireQueue retire(app.frame_loop().timeline());
  std::shared_ptr<const LiveSnapshot> latest;  // released before the worker

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
  ImGui::GetIO().IniFilename = nullptr;  // nothing written outside a session
  VKC_ASSIGN(std::unique_ptr<glfw::ImGuiGlfw> platform,
             glfw::ImGuiGlfw::create(window, overlay.context()));

  const Teardown teardown(*live, app, retire);
  Notices notices;
  bool fresh = false;
  bool fitted = false;
  std::optional<glm::vec2> dragged_from;
  while (glfwWindowShouldClose(window) == GLFW_FALSE && !g_interrupted.load()) {
    glfwPollEvents();
    for (const KeyRequest request : input.requests) {
      if (request == KeyRequest::Capture) {
        live->arm();
      } else if (request == KeyRequest::DropLast) {
        const core::Status dropped = captures->drop_last();
        if (!dropped.ok()) notices.note(dropped.message());
      }
    }
    input.requests.clear();
    if (std::shared_ptr<const LiveSnapshot> s = live->take_snapshot()) {
      latest = std::move(s);
      fresh = true;
    }
    take_reports(*live, *captures, notices);
    if (live->captures() > 0 && !editor.locked()) {
      (void)editor.edit(live->board(), live->square_measured());
      editor.lock();
    }

    VKC_ASSIGN(std::optional<gfx::windowing::Frame> next,
               app.begin_frame(glfw::framebuffer_size(window)));
    if (!next) {  // minimized
      glfwWaitEventsTimeout(0.05);
      continue;
    }
    gfx::windowing::Frame& frame = *next;
    retire.poll();
    const VkExtent2D extent = frame.target->extent();
    gfx::camera::ImageView2D& view = mosaic->view();
    view.set_viewport({0.0f, 0.0f},
                      glm::vec2(static_cast<float>(extent.width),
                                static_cast<float>(extent.height)));
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
    const glm::vec2 cursor = glfw::cursor_in_framebuffer(window);
    if (!io.WantCaptureMouse) {
      if (input.scroll != 0.0) {
        view.zoom_about(cursor,
                        std::pow(1.1f, static_cast<float>(input.scroll)));
      }
      if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        if (dragged_from) view.pan(cursor - *dragged_from);
        dragged_from = cursor;
      } else {
        dragged_from.reset();
      }
    } else {
      dragged_from.reset();
    }
    input.scroll = 0.0;

    capture_window(*live, *captures, notices, in);
    ImGui::SetNextWindowPos(ImVec2(10, 640), ImGuiCond_FirstUseEver);
    ImGui::Begin("Board");
    const Target before = editor.board();
    const bool measured_before = editor.square_measured();
    const core::Status edited = viz::draw_board_editor(editor);
    if (!edited.ok()) notices.note(edited.message());
    if (!same_board(editor.board(), before) ||
        editor.square_measured() != measured_before) {
      const core::Status taken =
          live->set_board(editor.board(), editor.square_measured());
      if (!taken.ok()) notices.note(taken.message());
    }
    ImGui::End();
    if (latest) {
      draw_marks(*mosaic, *latest, in);
      rig_windows(*latest, editor, in);
    }
    if (!notices.messages.empty()) {
      ImGui::Begin("Messages");
      for (const std::string& message : notices.messages) {
        ImGui::TextWrapped("%s", message.c_str());
      }
      ImGui::End();
    }
    flash(notices);
    input.want_text_input = io.WantTextInput;

    gfx::RenderTargetBeginInfo begin;
    frame.target->begin(frame.cmd, begin);
    mosaic->draw(frame.cmd, extent);
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

core::Result<RigPreset> check_scenario(const core::config::Config& config) {
  VKC_ASSIGN(RigPreset preset, rig_preset(config.source));
  std::error_code ec;
  if (!std::filesystem::is_directory(config.output.path, ec)) {
    return core::Status::not_found("output.path: " + config.output.path +
                                   " is not an existing directory");
  }
  return preset;
}

core::Status run_calib_rig(const core::config::Config& config) {
  VKC_ASSIGN(const RigPreset preset, check_scenario(config));
  g_interrupted.store(false);
  std::signal(SIGINT, on_interrupt);
  std::signal(SIGTERM, on_interrupt);
  const GlfwRun glfw_run;
  if (!glfw_run.ok()) {
    return core::Status::unsupported("calib_rig: GLFW will not initialize");
  }
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  const std::unique_ptr<GLFWwindow, WindowDeleter> window(
      glfwCreateWindow(config.window.width, config.window.height, "calib_rig",
                       nullptr, nullptr));
  if (window == nullptr) {
    return core::Status::unsupported("calib_rig: GLFW will not open a window");
  }
  const core::Status ran = run_window(window.get(), config, preset);
  std::signal(SIGINT, SIG_DFL);
  std::signal(SIGTERM, SIG_DFL);
  return ran;
}

}  // namespace volumetric_kit::calib::app
