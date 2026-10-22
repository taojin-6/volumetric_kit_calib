// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file rig_mode.hpp
/// @brief What calib_rig's window shows, and what each key does there: the
///        camera images until Calibrate gives a result, then the 3D view,
///        and back.
///
/// The window (T0) keeps a @ref RigMode and asks these pure functions what
/// to do: @ref rig_action for each key, @ref calibrate_gate whether
/// Calibrate may run, @ref scene_placement where the 3D view sits,
/// @ref frame_on_result whether a result's view is framed afresh, and
/// @ref world_up which way is up in it.
///
/// - **Images**, at the start and after V, B or `.`: the rig mosaic fills
///   the window, with the board found in each tile; the 3D view is hidden,
///   so its worker prepares nothing (`cloud_live`).
/// - **Scene**, at each result (@ref on_result): the 3D view fills the
///   window, every camera's live points at the solved poses; the mosaic is
///   neither drawn nor recorded.
///
/// A capture or a drop pressed in the 3D view only goes back to the images,
/// so nothing is captured or dropped unseen.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "volumetric_kit/calib/app/keys.hpp"
#include "volumetric_kit/calib/viz/scene_view.hpp"

namespace volumetric_kit::calib::app {

/// @brief What the window shows.
enum class RigView : std::uint8_t {
  Images,  ///< The camera images: the rig mosaic, the board found in each.
  Scene,   ///< The 3D view, in the whole window.
};

/// @brief Where the window stands.
///
/// @code
/// RigMode mode;  // the images, not calibrated
/// // Each frame:
/// mode.calibrating = calibrate->busy();
/// if (auto result = calibrate->take_result()) mode = on_result(mode);
/// @endcode
struct RigMode {
  RigView view = RigView::Images;  ///< What it shows.
  bool calibrated = false;         ///< A Calibrate result has been shown.
  /// A Calibrate is running (`CalibrateWorker::busy`).
  bool calibrating = false;
};

/// @brief What the window does for a key's request.
enum class RigAction : std::uint8_t {
  Ignore,      ///< Nothing, but its hint.
  Arm,         ///< Arm a capture (`LiveWorker::arm`).
  DropLast,    ///< Drop the last capture (`CaptureWorker::drop_last`).
  Calibrate,   ///< Run Calibrate, once @ref calibrate_gate lets it.
  Cancel,      ///< Cancel the running Calibrate (`CalibrateWorker::cancel`).
  ShowImages,  ///< Show the camera images.
  ShowScene,   ///< Show the 3D view.
};

/// @brief A request's action, and what the window says of it.
///
/// @code
/// const RigStep step = rig_action(mode, KeyRequest::Capture);
/// if (!step.hint.empty()) note(step.hint);
/// @endcode
struct RigStep {
  RigAction action = RigAction::Ignore;  ///< What to do.
  /// What the window says, a message of static storage; empty for nothing.
  std::string_view hint;
};

/// @brief What @p request does in @p mode.
///
/// - `Capture`: `Arm` in the images; `ShowImages` in the 3D view, with
///   "press again to capture".
/// - `DropLast`: `DropLast` in the images; `ShowImages` in the 3D view, with
///   "press again to drop the last capture".
/// - `Calibrate`: `Calibrate` in both.
/// - `SwitchView`: `ShowScene` in the images once calibrated, else `Ignore`
///   with "calibrate first"; `ShowImages` in the 3D view.
/// - `Cancel`: `Cancel` while calibrating, else `Ignore`, in both.
/// - `None`: `Ignore`.
///
/// Capturing and dropping go on while Calibrate runs: what is captured then
/// goes into the next Calibrate.
/// @param mode     Where the window stands.
/// @param request  The key's request (@ref key_request).
/// @return The action and its hint.
///
/// @code
/// for (const KeyRequest request : requests) {
///   const RigStep step = rig_action(mode, request);
///   if (step.action == RigAction::Arm) live->arm();
/// }
/// @endcode
RigStep rig_action(const RigMode& mode, KeyRequest request) noexcept;

/// @brief Whether Calibrate may run now, and why not.
///
/// @code
/// const CalibrateGate gate = calibrate_gate(mode, ready, unresolved);
/// ImGui::BeginDisabled(!gate.enabled);
/// @endcode
struct CalibrateGate {
  bool enabled = false;  ///< It may run.
  /// Why it may not, a message of static storage; empty when it may.
  std::string_view reason;
};

/// @brief Whether Calibrate may run: the session is ready, no capture or
///        drop is unresolved, and none is running. Its key and its buttons
///        share it.
/// @param mode        Where the window stands: `calibrating` holds it back.
/// @param ready       Whether the session's checks let Calibrate run
///                    (`quality::SessionSummary::ready`); false before the
///                    first capture is written.
/// @param unresolved  The captures and drops queued or in progress
///                    (`CaptureWorker::unresolved`), which a snapshot taken
///                    now would not reflect.
/// @return Enabled; or not, the reason the first of "already running", "a
///         capture or a drop is still being written" and "the session is
///         not ready (see the Session panel)" that holds.
///
/// @code
/// const std::size_t unresolved = captures->unresolved();  // first
/// const auto summary = captures->summary();
/// const CalibrateGate gate =
///     calibrate_gate(mode, summary && summary->ready, unresolved);
/// if (!gate.enabled) note(gate.reason);
/// @endcode
CalibrateGate calibrate_gate(const RigMode& mode, bool ready,
                             std::size_t unresolved) noexcept;

/// @brief Where the 3D view sits in @p mode.
/// @param mode  Where the window stands.
/// @return `Hidden` with the images; `FullWindow` in the 3D view.
///
/// @code
/// view3d.placement = scene_placement(mode);
/// cloud->set_live(cloud_live(frozen, view3d.placement, minimized));
/// @endcode
viz::ScenePlacement scene_placement(const RigMode& mode) noexcept;

/// @brief The mode once a Calibrate result is shown: the 3D view,
///        calibrated. A refusal, a cancel or a failure has no such step: the
///        view and the result shown stay.
/// @param mode  Where the window stood.
/// @return @p mode in the 3D view, calibrated.
///
/// @code
/// if (auto result = calibrate->take_result()) mode = on_result(mode);
/// @endcode
RigMode on_result(RigMode mode) noexcept;

/// @brief Where a result's world is: what the 3D view is framed on.
///
/// @code
/// const RigOrigin origin{result->reference,
///                        result->world.origin.tag.has_value()};
/// @endcode
struct RigOrigin {
  /// The reference camera, by rig index (`CalibrateResult::reference`),
  /// whose colour frame is the world without the tag.
  std::size_t reference = 0;
  /// The world is the origin tag's, +z up from the floor, whichever camera
  /// is the reference.
  bool tag = false;
};

/// @return Whether @p a and @p b are the same world: both the tag's, or the
///         same reference camera's.
bool operator==(const RigOrigin& a, const RigOrigin& b) noexcept;
/// @return Whether @p a and @p b are different worlds.
bool operator!=(const RigOrigin& a, const RigOrigin& b) noexcept;

/// @brief Whether a result's 3D view is framed afresh (`SceneView::frame`):
///        at the first result, and when its world's origin is not the one
///        last framed; otherwise the view's orbit is kept.
/// @param framed  The origin the view was last framed on; empty before the
///                first result.
/// @param origin  The result's.
/// @return Whether to frame the view.
///
/// @code
/// if (frame_on_result(framed, origin)) {
///   VKC_TRY(view.frame(poses, world_up(origin)));
///   framed = origin;
/// }
/// @endcode
bool frame_on_result(const std::optional<RigOrigin>& framed,
                     const RigOrigin& origin) noexcept;

/// @brief The up the 3D view takes for @p origin's world (`SceneView::frame`).
/// @param origin  The result's world.
/// @return The tag's +z, so its floor lies level, in a tag world; empty in a
///         reference camera's, for the cameras' mean up.
///
/// @code
/// VKC_TRY(view.frame(poses, world_up(origin)));
/// @endcode
std::optional<recon::camera::Vec3d> world_up(const RigOrigin& origin) noexcept;

}  // namespace volumetric_kit::calib::app
