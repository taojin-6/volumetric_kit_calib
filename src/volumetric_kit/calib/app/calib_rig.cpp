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
#include "volumetric_kit/calib/compute/guarded.hpp"
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
// run_window and its wheel-and-drag view input are generic GLFW glue; move them
// to gfx_glfw in a gfx PR, then port gfx's examples, recon's viewers and
// calib_rig onto it.

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
                        const RigScenario& scenario) {
  const RigPreset& preset = scenario.preset;
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
  for (std::size_t i = 0; i < array.size(); ++i) {
    VKC_ASSIGN(capture::SessionCamera camera,
               session_camera(array.sensor(i), scenario.sync));
    sizes.push_back(camera.info.color->size);
    setup.cameras.push_back(std::move(camera));
  }
  // TODO: cap cv::setNumThreads once H4 measures the contention between
  // OpenCV, the SDK and the decode threads.
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
  VKC_ASSIGN(std::unique_ptr<glfw::ImGuiGlfw> platform,
             glfw::ImGuiGlfw::create(window, overlay.context()));

  const Teardown teardown(*live, app, retire);
  Notices notices;
  bool fresh = false;
  bool fitted = false;
  std::optional<glm::vec2> dragged_from;
  while (glfwWindowShouldClose(window) == GLFW_FALSE && !g_interrupted.load()) {
    glfwPollEvents();
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
    act_on_keys(input.keys, io.WantTextInput, *live, *captures, notices);
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
    if (editor.board() != before ||
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
