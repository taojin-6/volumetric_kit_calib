// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/calib_rig.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#define GLFW_INCLUDE_NONE  // window and input only: gfx_glfw makes the surface
#include <GLFW/glfw3.h>

#include <glm/vec2.hpp>

#include "imgui.h"
#include "volumetric_kit/calib/app/calib_mode.hpp"
#include "volumetric_kit/calib/app/calibrate_worker.hpp"
#include "volumetric_kit/calib/app/capture_worker.hpp"
#include "volumetric_kit/calib/app/cloud_worker.hpp"
#include "volumetric_kit/calib/app/keys.hpp"
#include "volumetric_kit/calib/app/live_view.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/app/rig_mode.hpp"
#include "volumetric_kit/calib/app/rig_setup.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
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
#include "volumetric_kit/sensor/frame/device_requirements.hpp"

namespace volumetric_kit::calib::app {

namespace {

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
constexpr int kMaxStride = 8;
// The pointer's feel: radians of orbit per pixel dragged, and each wheel
// step's zoom, in the 3D view and the mosaic.
constexpr float kOrbitPerPixel = 0.005f;
constexpr float kSceneZoomStep = 0.9f;
constexpr float kMosaicZoomStep = 1.1f;

// What the window holds of the 3D view.
struct View3d {
  viz::ScenePlacement placement = viz::ScenePlacement::Hidden;  // the mode's
  bool frozen = false;
  // Its mode, stride and point size; the rest is set each frame.
  viz::CloudStyle style;
  viz::SceneOptions options;
  std::vector<viz::SceneCamera> cameras;      // in the rig's order
  std::optional<compute::Board> board;        // the solved boards'
  std::vector<sensor::camera::Mat4d> boards;  // each solved board's T_wb
  std::optional<sensor::camera::Vec3d> up;    // the world's (world_up)
  viz::SceneView view;
  std::shared_ptr<const CloudSnapshot> latest;  // released before the worker
  std::shared_ptr<core::Buffer> lines;          // the scene's lines, uploaded
  std::uint32_t line_vertices = 0;
  bool lines_stale = true;
  float max_point_size = 1.0f;
  std::string last_error;  // the last failure said, said once

  // The placed cameras' poses.
  std::vector<sensor::camera::Mat4d> placed() const {
    std::vector<sensor::camera::Mat4d> poses;
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

// Each camera in the 3D view, through its lens, unplaced.
std::vector<viz::SceneCamera> scene_cameras(const CaptureSetup& setup) {
  std::vector<viz::SceneCamera> cameras;
  for (std::size_t i = 0; i < setup.cameras.size(); ++i) {
    cameras.push_back(scene_camera(setup.cameras[i], setup.lenses[i], i,
                                   i == setup.reference));
  }
  return cameras;
}

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

// The session the window runs, as the Setup panel last chose it: what a
// switch starts from (switch_session), and what the panels say of each lens.
struct RigNow {
  RigScenario scenario;                   // its mode and lens file
  CaptureSetup setup;                     // the capture worker's, as last given
  LiveOptions live;                       // the live worker's, as last given
  std::vector<LensStatus> statuses;       // each camera's lens
  std::vector<std::string> origins;       // each lens's, for the Lenses panel
  std::vector<std::string> lens_lines;    // each camera panel's
  std::vector<viz::PanelLine> lens_file;  // the Setup panel's lens lines

  // The panels' views of each lens, after the setup changed.
  core::Status refresh() {
    const capture::SessionMode mode = setup.mode;
    const sensor::camera::ArrayCalibration* file = setup.calibration.get();
    VKC_ASSIGN(statuses, lens_statuses(mode, setup.cameras, file));
    origins.clear();
    lens_lines.clear();
    for (std::size_t i = 0; i < setup.cameras.size(); ++i) {
      origins.emplace_back(to_string(statuses[i]));
      VKC_ASSIGN(std::string line, lens_line(mode, setup.cameras[i], file));
      lens_lines.push_back(std::move(line));
    }
    VKC_ASSIGN(lens_file, setup_lines(mode, setup.cameras, file,
                                      scenario.calibration_path));
    return {};
  }
};

// A press of Capture: refused, and said, while the mode refuses a capture
// (capture_refusal) or once the live view has stopped.
void arm(LiveWorker& live, const RigNow& now, Notices& notices) {
  const core::Result<std::string> refused =
      capture_refusal(now.setup.mode, now.setup.cameras, now.statuses);
  if (!refused.ok()) {
    notices.note(refused.status().message());
  } else if (!refused->empty()) {
    notices.note("capture: " + *refused);
  } else if (!live.arm()) {
    notices.note("capture: the live view has stopped");
  }
}

void drop_last(CaptureWorker& captures, Notices& notices) {
  const core::Status dropped = captures.drop_last();
  if (!dropped.ok()) notices.note(dropped.message());
}

// What the Calibration and Lenses windows show of Calibrate: of the last run
// started, its result, or how it ended without one.
struct CalibrationShown {
  std::vector<viz::PanelLine> result;       // the result's panel
  std::vector<viz::PanelLine> lens_checks;  // a pose result's lens checks
  // A lens run that wrote a lens file, for "Use these lenses".
  std::shared_ptr<const CalibrateResult> lens_run;
  bool explain = false;   // a press found Calibrate held back: say why
  bool awaiting = false;  // a run started, its outcome not yet shown
  std::string run_error;  // the run's latest failure posted
  std::optional<viz::PanelLine> ended;  // a run ended with no result: why
};

// Everything a frame's UI reads.
struct FrameInputs {
  const std::vector<std::string>* ids = nullptr;
  const RigNow* now = nullptr;
  viz::PanelOptions panel;  // the rule's thresholds as the source sets them
  std::optional<quality::SessionSummary> summary;
  std::vector<quality::LensReadiness> lenses;  // a lens session's
  std::shared_ptr<const KeptCaptures> kept;
  std::size_t unresolved = 0;  // captures, drops, switches; read first
  viz::SaverQueue saver;
  std::string directory;
  core::StageMetrics stages;

  capture::SessionMode session() const { return now->setup.mode; }
  bool ready() const {
    return session_ready(session(), summary, lenses, now->statuses);
  }
};

// What the window's requests reach.
struct Workers {
  LiveWorker& live;
  CaptureWorker& captures;
  CalibrateWorker& calibrate;
};

// A press of Calibrate, its key's or a button's: the captures kept go to T4
// once the gate lets them; otherwise why not is said, and the Calibration
// window says it per camera. What the gate takes is read at the press, the
// captures, drops and switches unresolved first, so a drop pressed earlier
// in the frame holds it back, and the snapshot sent reflects every capture
// and drop made before.
void press_calibrate(const Workers& workers, const RigNow& now, RigMode& mode,
                     CalibrationShown& shown, Notices& notices) {
  const CaptureWorker& captures = workers.captures;
  const std::size_t unresolved = captures.unresolved();
  const std::optional<quality::SessionSummary> summary = captures.summary();
  const std::vector<quality::LensReadiness> lenses = captures.lenses();
  std::shared_ptr<const KeptCaptures> kept = captures.kept();
  const CalibrateGate gate = calibrate_gate(
      mode, session_ready(now.setup.mode, summary, lenses, now.statuses),
      unresolved);
  if (!gate.enabled) {
    notices.note("calibrate: " + std::string(gate.reason));
    shown.explain = true;
    return;
  }
  // The live search out of the way first, its set in flight waited out: a
  // lens run's fits then run at once, each on one thread (calibrate).
  workers.live.pause_search(true);
  const core::Result<bool> started = workers.calibrate.request(std::move(kept));
  if (!started.ok()) {
    notices.note(started.status().message());
  } else if (!started.value()) {
    notices.note("calibrate: already running");
  } else {
    mode.calibrating = true;
    // The last run's result is not this one's.
    shown = CalibrationShown{};
    shown.awaiting = true;
  }
}

// A press of Cancel, Escape's or the button's.
void cancel_calibrate(CalibrateWorker& calibrate, Notices& notices) {
  notices.note(calibrate.cancel()
                   ? "calibrate: cancelled"
                   : "calibrate: too late to cancel: writing its results");
}

// Back to the camera images: the 3D view's snapshot let go -- each frame that
// drew it holds it until it retires -- and its freeze undone, so the view is
// live when it is shown again.
void show_images(RigMode& mode, View3d& v) {
  mode.view = RigView::Images;
  v.latest.reset();
  v.frozen = false;
}

// The keys polled before this frame, mapped with its WantTextInput, each
// done as the window's mode has it (rig_action).
void act_on_keys(KeyQueue& keys, bool want_text_input, const Workers& workers,
                 const RigNow& now, RigMode& mode, CalibrationShown& shown,
                 View3d& v, Notices& notices) {
  const core::Result<std::vector<KeyRequest>> requests =
      keys.take(want_text_input);
  if (!requests.ok()) {
    notices.note(requests.status().message());
    return;
  }
  for (const KeyRequest request : requests.value()) {
    const RigStep step = rig_action(mode, request);
    if (!step.hint.empty()) notices.note(std::string(step.hint));
    switch (step.action) {
      case RigAction::Arm:
        arm(workers.live, now, notices);
        break;
      case RigAction::DropLast:
        drop_last(workers.captures, notices);
        break;
      case RigAction::Calibrate:
        press_calibrate(workers, now, mode, shown, notices);
        break;
      case RigAction::Cancel:
        cancel_calibrate(workers.calibrate, notices);
        break;
      case RigAction::ShowImages:
        show_images(mode, v);
        break;
      case RigAction::ShowScene:
        mode.view = RigView::Scene;
        break;
      case RigAction::Ignore:
        break;
    }
  }
}

// Calibrate's controls: its button, the gate's reason beside it while it is
// held back (a press then says why, per camera, in the Calibration window);
// or, while it runs, its progress and Cancel.
void calibrate_controls(const Workers& workers, RigMode& mode,
                        CalibrationShown& shown, const FrameInputs& in,
                        Notices& notices) {
  CalibrateWorker& calibrate = workers.calibrate;
  if (const std::optional<CalibrateProgress> at = calibrate.progress()) {
    const core::Result<viz::PanelLine> line = viz::calibrate_progress_line(
        to_string(at->stage), at->done, at->total, at->seconds);
    if (line.ok()) viz::draw_panel_lines("calibrating", {line.value()});
    if (ImGui::Button("Cancel")) cancel_calibrate(calibrate, notices);
    return;
  }
  const CalibrateGate gate = calibrate_gate(mode, in.ready(), in.unresolved);
  if (ImGui::Button("Calibrate")) {
    press_calibrate(workers, *in.now, mode, shown, notices);
  }
  if (!gate.enabled) {
    ImGui::SameLine();
    ImGui::TextDisabled("%.*s", static_cast<int>(gate.reason.size()),
                        gate.reason.data());
  }
}

// The capture's window: its buttons and Calibrate's, then where it stands.
void capture_window(const Workers& workers, RigMode& mode,
                    CalibrationShown& shown, Notices& notices,
                    const FrameInputs& in) {
  LiveWorker& live = workers.live;
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
  ImGui::Begin("Capture");
  if (ImGui::Button("Capture")) arm(live, *in.now, notices);
  ImGui::SameLine();
  if (ImGui::Button("Drop last")) drop_last(workers.captures, notices);
  calibrate_controls(workers, mode, shown, in, notices);
  notices.capture.decision = live.decision();
  const core::Result<std::vector<viz::PanelLine>> lines =
      viz::capture_panel(notices.capture);
  if (lines.ok()) viz::draw_panel_lines("capture", lines.value());
  ImGui::TextUnformatted(in.directory.empty() ? "session: none yet"
                                              : in.directory.c_str());
  ImGui::End();
}

void rig_windows(const LiveSnapshot& snapshot, const viz::BoardEditor& editor,
                 const CalibrationShown& shown, const FrameInputs& in) {
  const std::vector<std::string>& ids = *in.ids;
  ImGui::SetNextWindowPos(ImVec2(10, 200), ImGuiCond_FirstUseEver);
  ImGui::Begin("Rig");
  const core::Result<viz::SetLive> set = set_live(*snapshot.set);
  const core::Result<std::vector<viz::PanelLine>> set_lines =
      set.ok()
          ? viz::set_panel(set.value(), snapshot.stats, ids, in.saver, in.panel)
          : core::Result<std::vector<viz::PanelLine>>(set.status());
  if (set_lines.ok()) viz::draw_panel_lines("set", set_lines.value());
  ImGui::End();

  ImGui::SetNextWindowPos(ImVec2(10, 420), ImGuiCond_FirstUseEver);
  ImGui::Begin("Cameras");
  core::Result<std::vector<viz::CameraLive>> cameras = camera_lives(snapshot);
  for (std::size_t i = 0; cameras.ok() && i < ids.size(); ++i) {
    if (!ImGui::CollapsingHeader(ids[i].c_str(),
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
      continue;
    }
    const quality::CameraSummary none;
    const quality::CameraSummary& session =
        in.summary ? in.summary->cameras[i] : none;
    viz::CameraLive& camera = (*cameras)[i];
    camera.lens = in.now->lens_lines[i];
    const core::Result<std::vector<viz::PanelLine>> lines =
        viz::camera_panel(camera, session, editor.board(), in.panel);
    ImGui::PushID(static_cast<int>(i));
    if (lines.ok()) viz::draw_panel_lines("camera", lines.value());
    if (!snapshot.errors[i].ok()) {
      ImGui::TextWrapped("%s", snapshot.errors[i].message().c_str());
    }
    ImGui::PopID();
  }
  ImGui::End();

  // A lens session's panel: how far each camera's views are; a pose
  // session's, once calibrated: each lens checked on the solve.
  if (in.session() == capture::SessionMode::Intrinsics) {
    const core::Result<std::vector<viz::PanelLine>> lenses =
        viz::lens_panel(ids, in.lenses, in.now->origins);
    ImGui::Begin("Lenses");
    if (lenses.ok()) viz::draw_panel_lines("lenses", lenses.value());
    ImGui::End();
  } else if (!shown.lens_checks.empty()) {
    ImGui::Begin("Lenses");
    viz::draw_panel_lines("lens checks", shown.lens_checks);
    ImGui::End();
  }
  if (in.session() == capture::SessionMode::Extrinsics && in.summary) {
    core::Result<viz::SessionPanel> panel =
        viz::session_panel(*in.summary, ids);
    if (panel.ok()) {
      if (in.kept) {
        core::Result<viz::PanelLine> tag =
            viz::origin_tag_line(tag_coverage(*in.kept));
        if (tag.ok()) panel->lines.push_back(std::move(tag).value());
      }
      ImGui::Begin("Session");
      viz::draw_session_panel(panel.value());
      ImGui::End();
    }
  }
}

// The board's editor; an edit goes to the live worker.
void board_window(viz::BoardEditor& editor, LiveWorker& live,
                  Notices& notices) {
  ImGui::SetNextWindowPos(ImVec2(10, 640), ImGuiCond_FirstUseEver);
  ImGui::Begin("Board");
  const Target before = editor.board();
  const bool measured_before = editor.square_measured();
  const core::Status edited = viz::draw_board_editor(editor);
  if (!edited.ok()) notices.note(edited.message());
  if (editor.board() != before || editor.square_measured() != measured_before) {
    const core::Status taken =
        live.set_board(editor.board(), editor.square_measured());
    if (!taken.ok()) notices.note(taken.message());
  }
  ImGui::End();
}

void stages_window(const LiveSnapshot& snapshot, const FrameInputs& in) {
  const core::Result<gfx::FrameMetrics> metrics =
      viz::stage_frame_metrics(in.stages, snapshot.sets_per_second);
  if (metrics.ok()) gfx::ui::draw_metrics_panel(metrics.value(), "Stages");
}

// Why Calibrate is held back, per camera, in plain words: what the
// Calibration window says after a press found it so; or that it may run.
std::vector<viz::PanelLine> held_back(const RigMode& mode,
                                      const FrameInputs& in) {
  const CalibrateGate gate = calibrate_gate(mode, in.ready(), in.unresolved);
  if (gate.enabled) return {{"calibrate", "ready: press C", viz::Tone::Good}};
  std::vector<viz::PanelLine> lines{
      {"not calibrated", std::string(gate.reason), viz::Tone::Warn}};
  const RigNow& now = *in.now;
  const core::Result<std::vector<NotReady>> why = calibrate_blockers(
      now.setup.mode, now.setup.cameras, now.statuses,
      now.scenario.calibration_path, in.summary, in.lenses, now.setup.session);
  if (!why.ok()) {
    lines.push_back({"why", why.status().message(), viz::Tone::Bad});
    return lines;
  }
  for (const NotReady& camera : why.value()) {
    lines.push_back(
        {camera.camera.empty() ? std::string("session") : camera.camera,
         camera.why, viz::Tone::Warn});
  }
  return lines;
}

// The Calibration window, while Calibrate runs, once a press found it held
// back, and once a run ended: Calibrate's controls, why it is held back, and
// what the result is worth, with "Use these lenses" under a lens run's, or
// why the run ended without one; collapsed as the images come back, open as
// the 3D view does. Returns whether "Use these lenses" was pressed.
bool calibration_window(const Workers& workers, RigMode& mode,
                        bool view_changed, CalibrationShown& shown,
                        const FrameInputs& in, Notices& notices) {
  if (!mode.calibrating && !shown.explain && shown.result.empty() &&
      !shown.ended) {
    return false;
  }
  if (view_changed) {
    ImGui::SetNextWindowCollapsed(mode.view == RigView::Images,
                                  ImGuiCond_Always);
  }
  bool use = false;
  ImGui::SetNextWindowPos(ImVec2(430, 10), ImGuiCond_FirstUseEver);
  if (ImGui::Begin("Calibration")) {
    calibrate_controls(workers, mode, shown, in, notices);
    if (shown.explain && !mode.calibrating) {
      viz::draw_panel_lines("held back", held_back(mode, in));
    }
    if (shown.ended) viz::draw_panel_lines("ended", {*shown.ended});
    viz::draw_panel_lines("calibration", shown.result);
    if (shown.lens_run != nullptr) {
      ImGui::BeginDisabled(mode.calibrating);
      use = ImGui::Button("Use these lenses");
      ImGui::EndDisabled();
      ImGui::SameLine();
      ImGui::TextDisabled("for the next session, but those that failed");
    }
  }
  ImGui::End();
  return use;
}

// A change the Setup panel or "Use these lenses" asks for: the mode, or a
// lens file.
struct SetupRequest {
  std::optional<capture::SessionMode> mode;
  std::optional<LensFile> lenses;
  bool confirmed = false;  // closing a session with captures, confirmed
};

// What the Setup panel keeps between frames: the lens runs listed when its
// list opened, the path typed, and a switch waiting for its confirm.
struct SetupPanel {
  std::vector<LensResult> listed;
  bool list_open = false;
  std::array<char, 1024> typed{};
  std::optional<SetupRequest> confirm;
};

// A lens file for the rig, or why not, said.
std::optional<LensFile> or_note(core::Result<LensFile> file, Notices& notices) {
  if (!file.ok()) {
    notices.note(file.status().message());
    return std::nullopt;
  }
  return std::move(file).value();
}

// "Use these lenses": the lens run shown, as the Setup list takes a run.
std::optional<LensFile> use_lens_run(const CalibrateResult& run,
                                     const RigNow& now, Notices& notices) {
  core::Result<LensResult> read = read_lens_run(run.directory);
  if (!read.ok()) return or_note(read.status(), notices);
  return or_note(lens_run_file(*read, now.setup.cameras), notices);
}

// The Setup window: the mode, and where each camera's lens comes from, with
// a lens run's file to pick or a path to type; held while Calibrate runs. A
// switch that would close a session with captures asks first.
SetupRequest setup_window(SetupPanel& panel, const RigNow& now,
                          const RigMode& mode, const FrameInputs& in,
                          const std::string& root, Notices& notices) {
  SetupRequest request;
  ImGui::SetNextWindowPos(ImVec2(10, 900), ImGuiCond_FirstUseEver);
  ImGui::Begin("Setup");
  ImGui::BeginDisabled(mode.calibrating);
  if (panel.confirm) {
    ImGui::TextWrapped(
        "Start a new session? This one's %zu captures stay in %s, but "
        "Calibrate can no longer take them.",
        in.kept != nullptr ? in.kept->captures.size() : 0,
        in.directory.c_str());
    if (ImGui::Button("Start it")) {
      request = *std::move(panel.confirm);
      request.confirmed = true;
      panel.confirm.reset();
    }
    ImGui::SameLine();
    if (ImGui::Button("Keep this one")) panel.confirm.reset();
  }
  const bool lenses = now.setup.mode == capture::SessionMode::Intrinsics;
  if (ImGui::RadioButton("Lenses", lenses) && !lenses) {
    request.mode = capture::SessionMode::Intrinsics;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Poses", !lenses) && lenses) {
    request.mode = capture::SessionMode::Extrinsics;
  }
  if (mode.calibrating) {
    ImGui::SameLine();
    ImGui::TextDisabled("Calibrate is running");
  }
  viz::draw_panel_lines("setup", now.lens_file);
  const bool open = ImGui::BeginCombo("lens runs", "pick a lens run's file");
  if (open) {
    if (!panel.list_open) {
      core::Result<std::vector<LensResult>> found = find_lens_results(root);
      panel.listed =
          found.ok() ? std::move(found).value() : std::vector<LensResult>{};
      if (!found.ok()) notices.note(found.status().message());
    }
    for (std::size_t i = 0; i < panel.listed.size(); ++i) {
      const core::Result<std::string> label =
          lens_result_label(panel.listed[i]);
      ImGui::PushID(static_cast<int>(i));
      if (label.ok() && ImGui::Selectable(label.value().c_str())) {
        request.lenses =
            or_note(lens_run_file(panel.listed[i], now.setup.cameras), notices);
      }
      ImGui::PopID();
    }
    if (panel.listed.empty())
      ImGui::TextDisabled("none under %s", root.c_str());
    ImGui::EndCombo();
  }
  panel.list_open = open;
  ImGui::InputText("path", panel.typed.data(), panel.typed.size());
  ImGui::SameLine();
  if (ImGui::Button("Load") && panel.typed[0] != '\0') {
    request.lenses =
        or_note(load_lens_file(panel.typed.data(), now.setup.cameras), notices);
  }
  ImGui::EndDisabled();
  ImGui::End();
  return request;
}

// A switch asked for this frame, once confirmed where it would close a
// session with captures: else held for the Setup window's confirm.
SetupRequest confirmed(SetupRequest request, SetupPanel& panel,
                       const FrameInputs& in, Notices& notices) {
  if ((!request.mode && !request.lenses) || request.confirmed ||
      in.kept == nullptr || in.kept->captures.empty()) {
    return request;
  }
  notices.note("switch: confirm it in the Setup window");
  panel.confirm = std::move(request);
  return {};
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

// What the workers but Calibrate's (take_calibrate) reported since the last
// frame.
void take_reports(const Workers& workers, CloudWorker& cloud,
                  Notices& notices) {
  for (const core::Status& error : workers.live.take_errors()) {
    notices.note(error.message());
  }
  for (const core::Status& error : cloud.take_errors()) {
    notices.note(error.message());
  }
  CaptureWorker& captures = workers.captures;
  for (const core::Status& error : captures.take_errors()) {
    notices.note(error.message());
  }
  for (const CaptureEvent& event : captures.take_events()) {
    switch (event.kind) {
      case CaptureEvent::Kind::Saved:
        notices.capture.last = event.meta;
        notices.capture.last_usable_views = event.usable_views;
        notices.capture.last_tag_cameras = event.tag_cameras;
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

// The 3D view's window: freeze, colouring, and each camera's toggle and tint.
void view_window(View3d& v, const std::vector<std::string>& ids,
                 Notices& notices) {
  ImGui::SetNextWindowPos(ImVec2(10, 760), ImGuiCond_FirstUseEver);
  ImGui::Begin("3D view");
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
    const core::Status framed = v.view.frame(v.placed(), v.up);
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
    ImGui::PopID();
  }
  ImGui::End();
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
  VKC_ASSIGN(const std::vector<gfx::pipelines::ColorVertex> vertices,
             viz::scene_lines(v.cameras, v.board ? &*v.board : nullptr,
                              v.boards, v.options));
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

// A Calibrate result in the 3D view: every camera at its pose in the result's
// world, the solved boards and the origin tag drawn, the view framed on the
// world's origin when it is new (frame_on_result), up as world_up has it, its
// panel's lines and its lens checks', its notices (calibration_notices), and
// the 3D view in the window. Nothing changes when the result cannot be shown.
core::Status show_result(const std::shared_ptr<const CalibrateResult>& run,
                         const KeptCaptures& kept, RigMode& mode, View3d& v,
                         std::optional<RigOrigin>& framed,
                         CalibrationShown& shown, Notices& notices) {
  const CalibrateResult& result = *run;
  std::vector<viz::PanelLine>& panel = shown.result;
  // A lens run: its panel and where it was written; no 3D view.
  if (result.mode == capture::SessionMode::Intrinsics) {
    std::vector<std::pair<std::string, std::string>> unsolved;
    for (const UnsolvedLens& lens : result.unsolved) {
      unsolved.emplace_back(lens.id, lens.reason);
    }
    VKC_ASSIGN(std::vector<viz::PanelLine> lines,
               viz::lens_results_panel(
                   result.lenses, result.report_only, unsolved, result.number,
                   result.directory, result.captures, result.seconds));
    VKC_ASSIGN(const std::vector<std::string> said,
               viz::calibration_notices(result.number, result.directory, {}));
    panel = std::move(lines);
    // "Use these lenses" once there is a file to take: a run of factory
    // lenses alone writes none.
    shown.lens_run = result.files.array_calibration.empty() ? nullptr : run;
    mode = on_result(mode);
    for (const std::string& notice : said) notices.note(notice);
    return {};
  }
  VKC_ASSIGN(compute::Board board, compute::Board::create(kept.board));
  VKC_ASSIGN(std::vector<viz::PanelLine> lines,
             viz::calibration_panel(result.ids, result.metrics, result.number,
                                    result.directory, result.captures,
                                    result.seconds));
  VKC_ASSIGN(
      const std::vector<viz::PanelLine> depth,
      viz::depth_panel(result.ids, result.depth ? &*result.depth : nullptr,
                       result.depth_failure));
  VKC_ASSIGN(std::vector<viz::PanelLine> world,
             viz::world_panel(result.ids, result.reference, result.world));
  VKC_ASSIGN(const std::vector<viz::PanelLine> rows,
             viz::pair_rows_panel(result.pairs));
  VKC_ASSIGN(std::vector<viz::PanelLine> lens_checks,
             viz::lens_check_panel(result.lens_checks));
  world.insert(world.end(), rows.begin(), rows.end());
  lines.insert(lines.end(), depth.begin(), depth.end());
  lines.insert(lines.end(), world.begin(), world.end());
  // The notices: the world's, the pairs' and the lens checks' warnings.
  std::vector<viz::PanelLine> warned = world;
  warned.insert(warned.end(), lens_checks.begin(), lens_checks.end());
  VKC_ASSIGN(const std::vector<std::string> said,
             viz::calibration_notices(result.number, result.directory, warned));
  std::vector<sensor::camera::Mat4d> boards = result.boards_to_world;
  VKC_TRY(placed_cameras(v.cameras, result.color_to_world));
  const std::optional<solve::TagWorld>& tag = result.world.origin.tag;
  const RigOrigin origin{result.reference, tag.has_value()};
  v.board = std::move(board);
  v.boards = std::move(boards);
  v.up = world_up(origin);
  v.options.tag_edge_m = tag ? tag->edge_m : 0.0;
  v.lines_stale = true;
  panel = std::move(lines);
  shown.lens_checks = std::move(lens_checks);
  shown.lens_run = nullptr;
  if (frame_on_result(framed, origin)) {
    const core::Status aimed = v.view.frame(v.placed(), v.up);
    if (aimed.ok()) {
      framed = origin;
    } else {
      report_view(v, aimed, notices);
    }
  }
  mode = on_result(mode);
  for (const std::string& notice : said) notices.note(notice);
  return {};
}

// Calibrate's outcome since the last frame: each failure said; a result
// shown; or, once the run started ended without one, why kept for the
// Calibration window: its failure, else its cancel. busy() is read first, as
// a run's result and failure are posted before it ends.
void take_calibrate(CalibrateWorker& calibrate, const CaptureWorker& captures,
                    RigMode& mode, View3d& v, std::optional<RigOrigin>& framed,
                    CalibrationShown& shown, Notices& notices) {
  const bool running = calibrate.busy();
  for (const core::Status& error : calibrate.take_errors()) {
    notices.note(error.message());
    shown.run_error = error.message();
  }
  if (std::shared_ptr<const CalibrateResult> result = calibrate.take_result()) {
    shown.awaiting = false;
    const std::shared_ptr<const KeptCaptures> kept = captures.kept();
    const core::Status placed =
        kept != nullptr
            ? show_result(result, *kept, mode, v, framed, shown, notices)
            : core::Status::not_found("no captures kept");
    if (!placed.ok()) {
      notices.note("calibrate: " + placed.message());
      shown.ended =
          viz::PanelLine{"calibrate", "written, not shown: " + placed.message(),
                         viz::Tone::Bad};
    }
  } else if (shown.awaiting && !running) {
    shown.awaiting = false;
    shown.ended =
        shown.run_error.empty()
            ? viz::PanelLine{"calibrate", "cancelled", viz::Tone::Muted}
            : viz::PanelLine{"calibrate", "failed: " + shown.run_error,
                             viz::Tone::Bad};
  }
  mode.calibrating = running;
}

// Stops the workers, waits for the renderer, and frees what the frames held,
// on every way out of the loop: Calibrate first, whose run stops at its next
// step, then the live view, which feeds the 3D view's worker.
class Teardown {
 public:
  Teardown(CalibrateWorker& calibrate, LiveWorker& live, CloudWorker& cloud,
           gfx::app::WindowedApp& app, gfx::RetireQueue& retire) noexcept
      : calibrate_(calibrate),
        live_(live),
        cloud_(cloud),
        app_(app),
        retire_(retire) {}
  ~Teardown() {
    calibrate_.stop();
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
  CalibrateWorker& calibrate_;
  LiveWorker& live_;
  CloudWorker& cloud_;
  gfx::app::WindowedApp& app_;
  gfx::RetireQueue& retire_;
};

core::Status run_window(GLFWwindow* window, const core::config::Config& config,
                        const RigScenario& scenario) {
  Notices notices;
  // --- One VkDevice: gfx draws on its graphics side, sensor decodes, reads
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
  VKC_ASSIGN(core::Device sensor_device,
             core::Device::adopt(shared->compute_payload(), needs.compute));
  VKC_ASSIGN(
      core::Allocator sensor_allocator,
      core::Allocator::create(shared->instance().handle(), sensor_device));

  // --- The rig's cameras, as one source: the opener's alone names a driver.
  VKC_ASSIGN(
      OpenedSource opened,
      open_source(config, scenario, sensor_device, sensor_allocator,
                  {shared->compute_family(), shared->graphics_family()}));
  const std::size_t camera_count = opened.cameras.size();

  CaptureSetup setup;
  setup.root = config.output.path;
  setup.software = app_software();
  setup.driver = opened.driver;
  setup.sync_path = opened.sync_path;
  setup.reference = opened.reference;
  setup.depth_mode = opened.depth_mode;
  setup.color_codec = opened.color_codec;
  setup.lenses = opened.lenses;
  std::vector<sensor::camera::ImageSize> sizes;
  for (const capture::SessionCamera& camera : opened.cameras) {
    sizes.push_back(camera.size);
  }
  setup.cameras = opened.cameras;
  // The mode's parts: the session's mode and file, the capture rule, the
  // window's mode, and the Board window's board and the boards it takes;
  // kept, as a switch starts from them (switch_session).
  RigNow now;
  now.scenario = scenario;
  RigMode mode;
  VKC_ASSIGN(viz::BoardEditor editor,
             apply_mode(scenario, setup, now.live, mode));
  now.setup = std::move(setup);
  VKC_TRY(now.refresh());
  // OpenCV's threads are not capped: a cap made neither the fits nor the
  // live search faster, and one thread halved the search's rate
  // (DECISIONS.md, 2026-11-03).
  VKC_ASSIGN(std::unique_ptr<CaptureWorker> captures,
             CaptureWorker::create(now.setup));
  // The 3D view's worker, prepared frames shared with the renderer's family.
  VKC_ASSIGN(std::unique_ptr<CloudWorker> cloud,
             CloudWorker::create(
                 sensor_device, sensor_allocator, camera_count,
                 {shared->compute_family(), shared->graphics_family()}));
  // Calibrate's worker (T4), after the 3D view's, so it goes first, with its
  // depth checks on sensor's device, built now so a failure shows here.
  VKC_ASSIGN(
      std::unique_ptr<CalibrateDepth> depth,
      CalibrateDepth::create(sensor_device, sensor_allocator, camera_count));
  VKC_ASSIGN(std::unique_ptr<CalibrateWorker> calibrate,
             CalibrateWorker::create(std::move(depth)));

  // --- The window's side.
  VKC_ASSIGN(std::unique_ptr<viz::RigMosaic> mosaic,
             viz::RigMosaic::create(app.device(), app.allocator(),
                                    app.swapchain().layout(), sizes));
  gfx::pipelines::DepthCloudPipelineDesc cloud_desc;
  cloud_desc.max_draws = static_cast<std::uint32_t>(camera_count);
  VKC_ASSIGN(std::unique_ptr<gfx::pipelines::DepthCloudPipeline> clouds,
             gfx::pipelines::DepthCloudPipeline::create(
                 app.device(), app.allocator(), app.swapchain().layout(),
                 app.frame_loop().timeline(), cloud_desc));
  VKC_ASSIGN(std::unique_ptr<gfx::pipelines::PrimitivePipeline> primitives,
             gfx::pipelines::PrimitivePipeline::create(
                 app.device(), app.swapchain().layout()));
  gfx::RetireQueue retire(app.frame_loop().timeline());
  View3d view3d;  // its snapshot released before the retire queue and cloud
  view3d.cameras = scene_cameras(now.setup);
  view3d.max_point_size = clouds->max_point_size();
  VKC_TRY(view3d.view.frame(view3d.placed()));

  // GLFW's callbacks before ImGui's backend, which chains to them, so the keys
  // reach the app first.
  Input input;
  const InputBinding binding(window, input);
  glfwSetKeyCallback(window, on_key);
  glfwSetScrollCallback(window, on_scroll);
  VKC_ASSIGN(
      gfx::ui::ImGuiOverlay overlay,
      gfx::ui::ImGuiOverlay::create(app.device(), app.instance_handle(),
                                    gfx::ui::overlay_config(app.swapchain())));
  ImGui::SetCurrentContext(overlay.context());
  VKC_ASSIGN(std::unique_ptr<glfw::ImGuiGlfw> platform,
             glfw::ImGuiGlfw::create(window, overlay.context()));

  // --- The live worker last, as its thread starts the cameras: from here
  // every way out stops Calibrate's worker, then it, then the 3D view's
  // (Teardown), so no set is held when the source closes.
  VKC_ASSIGN(std::unique_ptr<LiveWorker> live,
             LiveWorker::create(std::move(opened.source), now.setup.lenses,
                                editor.board(), editor.square_measured(),
                                captures.get(), cloud.get(), now.live));
  const std::vector<std::string>& ids = live->ids();
  std::shared_ptr<const LiveSnapshot> latest;  // released before live
  const Teardown teardown(*calibrate, *live, *cloud, app, retire);
  const Workers workers{*live, *captures, *calibrate};
  RigView shown = mode.view;        // the last frame's
  std::optional<RigOrigin> framed;  // the 3D view's, once framed on a result
  CalibrationShown calibration;     // the Calibration and Lenses windows'
  SetupPanel setup_panel;
  // Each mode's board as it last had it (by capture::SessionMode), so a
  // switch back finds the board entered there.
  std::array<std::optional<std::pair<Target, bool>>, 2> boards;
  // A new session of `next`'s mode and lens file, the cameras left open; said
  // when refused.
  const auto start_session = [&](const RigScenario& next) {
    if (calibrate->busy()) {
      notices.note(
          "switch: Calibrate is running: wait for it, or cancel it (Escape)");
      return;
    }
    const std::shared_ptr<const KeptCaptures> closing = captures->kept();
    const std::string directory = captures->directory();
    boards[static_cast<std::size_t>(now.setup.mode)] =
        std::make_pair(editor.board(), editor.square_measured());
    core::Result<viz::BoardEditor> made =
        switch_session(next, boards[static_cast<std::size_t>(next.mode)],
                       now.setup, now.live, mode, *live, *captures);
    if (!made.ok()) {
      notices.note("switch: " + made.status().message());
      return;
    }
    editor = std::move(made).value();
    now.scenario = next;
    const core::Status refreshed = now.refresh();
    if (!refreshed.ok()) notices.note(refreshed.message());
    calibration = CalibrationShown{};
    view3d.cameras = scene_cameras(now.setup);
    view3d.lines_stale = true;
    framed.reset();
    if (closing != nullptr && !closing->captures.empty()) {
      notices.note("session " + directory + " closed with " +
                   std::to_string(closing->captures.size()) +
                   " captures, kept on disk");
    }
    notices.note(std::string(next.mode == capture::SessionMode::Intrinsics
                                 ? "Lenses"
                                 : "Poses") +
                 ": a new session starts at the next capture");
  };
  // A lens file for the next session, its notes said.
  const auto take_lenses = [&](const LensFile& file) {
    for (const std::string& note : file.notes) notices.note(note);
    RigScenario next = now.scenario;
    next.calibration = file.calibration;
    next.calibration_path = file.path;
    start_session(next);
  };
  bool fresh = false;
  core::StageMetrics tiles;  // the last recording's, held between live sets
  std::vector<std::string> refusals_said;  // each tile's, said once
  bool fitted = false;
  Pointer pointer;
  while (glfwWindowShouldClose(window) == GLFW_FALSE && !g_interrupted.load()) {
    glfwPollEvents();
    if (std::shared_ptr<const LiveSnapshot> s = live->take_snapshot()) {
      latest = std::move(s);
      fresh = true;
    }
    if (std::shared_ptr<const CloudSnapshot> s = cloud->take_snapshot()) {
      // Held only while the 3D view is shown (show_images).
      if (mode.view == RigView::Scene) view3d.latest = std::move(s);
    }
    take_reports(workers, *cloud, notices);
    take_calibrate(*calibrate, *captures, mode, view3d, framed, calibration,
                   notices);
    // Calibrate has the CPU while it runs; the pictures keep showing.
    live->pause_search(search_paused(mode));
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

    // The captures, drops and switches unresolved first, so the snapshot
    // read after reflects every one made before.
    FrameInputs in;
    in.ids = &ids;
    in.now = &now;
    in.panel.rule = live->rule_options();
    in.unresolved = captures->unresolved();
    in.saver = {captures->pending(), captures->capacity(), captures->full()};
    in.summary = captures->summary();
    in.lenses = captures->lenses();
    in.kept = captures->kept();
    in.directory = captures->directory();
    if (latest) in.stages = latest->stages;
    if (view3d.latest) in.stages.merge(view3d.latest->stages);

    // The keys before the layout, so a switch of view shows in this frame.
    platform->new_frame();
    overlay.new_frame();
    const ImGuiIO& io = ImGui::GetIO();
    act_on_keys(input.keys, io.WantTextInput, workers, now, mode, calibration,
                view3d, notices);
    const bool view_changed = mode.view != shown;
    shown = mode.view;
    view3d.placement = scene_placement(mode);
    // The board is searched for only where the images show it.
    live->set_detecting(mode.view == RigView::Images);

    const VkExtent2D extent = frame.target->extent();
    const viz::WindowSplit split = viz::window_split(view3d.placement, extent);
    const bool mosaic_shown = split.mosaic.extent.width > 0;
    const bool scene_shown = split.scene.extent.width > 0;
    gfx::camera::ImageView2D& view = mosaic->view();
    view.set_viewport(
        glm::vec2(static_cast<float>(split.mosaic.offset.x),
                  static_cast<float>(split.mosaic.offset.y)),
        glm::vec2(static_cast<float>(split.mosaic.extent.width),
                  static_cast<float>(split.mosaic.extent.height)));
    if (!fitted && mosaic_shown) {
      view.fit();
      fitted = true;
    }
    // The tiles only while the mosaic is shown: a snapshot taken meanwhile
    // stays fresh for the images' return.
    core::StageMetrics recorded;
    if (fresh && latest && mosaic_shown) {
      core::StageScope scope(recorded, "tiles");
      VKC_TRY(
          record_snapshot(*mosaic, retire, frame.cmd, latest, frame.number));
      VKC_ASSIGN(const std::vector<viz::TileRefusal> unsaid,
                 refusals_to_say(mosaic->refused(), refusals_said));
      for (const viz::TileRefusal& refusal : unsaid) {
        notices.note("camera " + ids[refusal.camera] + ": " +
                     refusal.status.message());
      }
      fresh = false;
    }
    VKC_TRY(viz::hold_stages(recorded, mosaic_shown, tiles, in.stages));
    steer(window, input, pointer, io.WantCaptureMouse, split, view,
          view3d.view);

    // A switch the Setup panel or "Use these lenses" asks for takes effect
    // at the next frame's inputs.
    SetupRequest request;
    if (mode.view == RigView::Images) {
      capture_window(workers, mode, calibration, notices, in);
      board_window(editor, *live, notices);
      request =
          setup_window(setup_panel, now, mode, in, config.output.path, notices);
    } else {
      view_window(view3d, ids, notices);
    }
    if (calibration_window(workers, mode, view_changed, calibration, in,
                           notices)) {
      request = SetupRequest{};
      request.lenses = use_lens_run(*calibration.lens_run, now, notices);
    }
    request = confirmed(std::move(request), setup_panel, in, notices);
    cloud->set_live(cloud_live(view3d.frozen, view3d.placement, false));
    cloud->set_color(view3d.style.mode != gfx::pipelines::CloudColor::Solid);
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
    if (latest && mode.view == RigView::Images) {
      rig_windows(*latest, editor, calibration, in);
    }
    if (latest) stages_window(*latest, in);
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
    // After the frame, which drew from the session it ends.
    if (request.mode) {
      RigScenario next = now.scenario;
      next.mode = *request.mode;
      start_session(next);
    } else if (request.lenses) {
      take_lenses(*request.lenses);
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
                   core::merge(sensor::device_requirements(),
                               depth::DepthChecks::device_requirements()));
        needs.graphics = gfx::device_requirements();
        needs.graphics.features.largePoints = large_points ? VK_TRUE : VK_FALSE;
        return needs;
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
