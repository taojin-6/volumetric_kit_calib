// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file live_checks.hpp
/// @brief What the live view measures in each camera's view of the board, set
///        by set: what the view is worth (@ref check_view), and how far its
///        corners moved (@ref corner_shift), which gates a capture on
///        stillness and flags one whose live corners disagree with its full
///        ones (@ref live_vs_full_flagged).
///
/// Pure and geometric (DECISIONS.md, 2026-10-14); every function may run on
/// several threads at once.

#include <cstdint>
#include <optional>
#include <vector>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/core/view_check.hpp"
#include "volumetric_kit/calib/quality/export.hpp"
#include "volumetric_kit/calib/solve/view_pose.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::quality {

/// @brief The thresholds behind a view's verdict, and what counts as
///        frontal.
///
/// @code
/// ViewCheckOptions options;
/// options.far_m = 4.0;  // flag boards beyond 4 m
/// @endcode
struct ViewCheckOptions {
  /// A view tilted more than this (degrees) is `Oblique` (amber); one tilted
  /// more than the solve's `InitialiseOptions::oblique_deg` is flagged by the
  /// solve too (red).
  double oblique_deg = 60.0;
  double far_m = 4.5;  ///< A board farther than this (metres) is `Far`.
  /// A view tilted less than this (degrees) is frontal (DECISIONS.md,
  /// 2026-10-14).
  double frontal_deg = 20.0;
};

/// @brief One camera's view of the board, checked: the record a capture
///        stores, and the view's own poses, which the session's checks
///        compare and judge as the solve does.
///
/// @code
/// VKC_ASSIGN(const CheckedView view, check_view(cam, board, detection));
/// if (view.frontal) hint("tilt the board toward or away from this camera");
/// @endcode
struct CheckedView {
  ViewCheck check;  ///< What the view is worth.
  /// Its own poses (@ref solve::view_pose; `pose->best.board_to_camera` is
  /// T_cb): empty when the view has none.
  std::optional<solve::ViewPose> pose;
  /// Posed, and tilted less than `ViewCheckOptions::frontal_deg`.
  bool frontal = false;
};

/// @brief What one camera's view of @p board is worth.
///
/// - **Counts.** The corners and markers found.
/// - **Pose.** A view @ref compute::usable_for_pose accepts is posed by
///   @ref solve::view_pose; its better fit gives `pnp_rms_px`, the distance
///   from the camera to the board's centre (@ref compute::Board::centre),
///   the tilt (@ref solve::tilt_deg), and the cell of a 3 x 3 grid over the
///   image holding the centre's projection, empty when it falls outside.
/// - **Verdict**, the first that applies: `Collinear` when the corners, or
///   all but one, lie on one line; `None` for any other view with no pose;
///   `Oblique` above `oblique_deg`; `Far` beyond `far_m`; `Partial` with some
///   of the board's corners; `Good` with all of them.
/// @param camera     The camera, at the size of the image searched.
/// @param board      The board.
/// @param detection  What the detector found (live or full).
/// @param options    The thresholds.
/// @return The checked view; `InvalidArgument` for a detection of an image
///         of another size, or an option out of range (naming it);
///         @ref solve::view_pose's `Backend` error when IPPE fails; or what
///         OpenCV threw (@ref compute::guarded).
///
/// @code
/// VKC_ASSIGN(const CheckedView view, check_view(cam, board, detection));
/// draw_verdict(to_string(view.check.verdict));  // a caller's own drawing
/// @endcode
VC_QUALITY_API core::Result<CheckedView> check_view(
    const compute::CvCamera& camera, const compute::Board& board,
    const Detection& detection, const ViewCheckOptions& options = {});

/// @brief How @ref corner_shift pairs a corner of one detection with one of
///        another of the same camera.
enum class CornerMatch : std::uint8_t {
  /// By id: a ChArUco id names one physical corner in every frame.
  ById,
  /// By position, ids ignored: each corner pairs with the other detection's
  /// nearest, within a third of the distance to its own nearest neighbour.
  /// A checkerboard's ids name its corners in one view only, so a still
  /// board cut by the image's edge can number its corners differently from
  /// frame to frame.
  Nearest,
};

/// @brief How @p board's corners pair across detections.
/// @param board  The board.
/// @return `ById` for a ChArUco board; `Nearest` for a checkerboard.
///
/// @code
/// VKC_ASSIGN(const CornerShift moved,
///            corner_shift(previous, current, corner_match(board)));
/// @endcode
VC_QUALITY_API CornerMatch corner_match(const compute::Board& board) noexcept;

/// @brief How far each corner moved from one set of detections to another,
///        each paired with one in the same camera (@ref CornerMatch).
///
/// @code
/// VKC_ASSIGN(const CornerShift shift, corner_shift(previous, current));
/// if (shift.cameras[i]) show_stillness(i, *shift.cameras[i]);
/// @endcode
struct CornerShift {
  /// For each camera, the median distance its corners moved (pixels); empty
  /// when no corner is in both of its detections.
  std::vector<std::optional<double>> cameras;
  /// The median over every corner in both, all cameras together (pixels);
  /// empty when there is none.
  std::optional<double> median_px;
  std::optional<double> max_px;  ///< The largest (pixels); empty with none.
};

/// @brief How far the corners moved from @p before to @p after: from one set
///        to the next (stillness), or from a capture's live detections to its
///        full ones (@ref live_vs_full_flagged).
/// @param before  One detection per camera, in the rig's order; a camera
///                with no frame or no board has an empty one.
/// @param after   The same cameras' detections, in the same order.
/// @param match   How corners pair (@ref corner_match gives a board's).
/// @return The shift; `InvalidArgument` for lists of different lengths, a
///         corner not finite, an id repeated in one detection (`ById`), two
///         detections of one camera, both with corners, in images of
///         different sizes (each naming the camera's index), or a @p match
///         that is not a @ref CornerMatch; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const CornerShift moved,
///            corner_shift(previous, current, corner_match(board)));
/// set.motion_px = moved.median_px;  // the capture rule's stillness
/// @endcode
VC_QUALITY_API core::Result<CornerShift> corner_shift(
    const std::vector<Detection>& before, const std::vector<Detection>& after,
    CornerMatch match = CornerMatch::ById);

/// @brief When a capture's live corners and its full ones disagree.
///
/// @code
/// LiveVsFullOptions options;
/// options.max_px = 1.5;
/// @endcode
struct LiveVsFullOptions {
  double median_px = 0.3;  ///< Flagged when the median shift is above this.
  double max_px = 1.0;     ///< Flagged when any corner's is above this.
};

/// @brief Whether a capture is flagged because its live (hybrid) corners
///        disagree with its full ones: flagged, never refused, since the
///        solver takes only the full ones.
/// @param shift    `corner_shift(live, full)` of the capture.
/// @param options  The limits.
/// @return `true` when the median or the largest shift is above its limit;
///         `false` when no corner is in both.
///
/// @code
/// VKC_ASSIGN(const CornerShift shift, corner_shift(live, full));
/// meta.live_vs_full_px = shift.median_px;
/// if (live_vs_full_flagged(shift)) warn("live and full corners disagree");
/// @endcode
VC_QUALITY_API bool live_vs_full_flagged(
    const CornerShift& shift, const LiveVsFullOptions& options = {}) noexcept;

}  // namespace volumetric_kit::calib::quality
