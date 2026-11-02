// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file lens.hpp
/// @brief One camera's lens from its views of a board: sensor's eight
///        coefficient rational lens by OpenCV's `calibrateCamera`, with views
///        held out to judge it by (`quality/lens_gate.hpp`).
///
/// Each view has its own pose, so a board may be cut by the image's edge and a
/// checkerboard's ids need hold in that view alone.

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/export.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"

namespace volumetric_kit::calib::solve {

/// How far toward an image corner or side (@ref LensReach) a view reaches to
/// count: about 70 degrees off axis toward the corners, 61 toward the sides,
/// on the lab's fisheye. An R8 fit with corners that far does not fold inside
/// the frame.
inline constexpr double kLensReach = 0.9;

/// @brief How far a pixel, or the farthest of several, gets from the image's
///        centre toward each image corner (top-left, top-right, bottom-left,
///        bottom-right) and side (left, right), in that order, as a fraction
///        of the way: its offset along the line to that corner or side, over
///        the line's length. 0 or more; lens-free, so it holds for any camera.
using LensReach = std::array<double, 6>;

/// @brief How far one pixel reaches.
/// @param size   The image.
/// @param pixel  The pixel, OpenCV's convention (centres at integers).
/// @return Its reach, 1 at the corner or side itself, 0 away from it.
///
/// @code
/// const LensReach reach = pixel_reach({3840, 2160}, {0.0, 0.0});
/// // reach[0] is almost 1: the top-left corner.
/// @endcode
VC_SOLVE_API LensReach pixel_reach(sensor::camera::ImageSize size,
                                   const sensor::camera::Vec2d& pixel) noexcept;

/// @brief What @ref solve_lens takes and holds out.
///
/// @code
/// LensOptions options;
/// options.outlier_rounds = 0;  // keep every corner
/// @endcode
struct LensOptions {
  /// Fitted views left after the outlier rounds, at least; at least 1.
  int min_views = 20;
  /// A view with fewer corners is not used; at least 4.
  int min_corners = 12;
  /// Held out, never fitted: within each cell of a 3 x 3 grid over the image
  /// that views centre in, one view of every this many in capture order, the
  /// middle one, so a cell of 3 gives one; never a view that is the last
  /// fitted one reaching @ref kLensReach toward a corner or side. At least 2.
  int holdout_every = 5;
  /// Solves after outliers are dropped, at most, by the bundle adjustment's
  /// rule and bounds (@ref BundleAdjustOptions); 0 drops none.
  int outlier_rounds = 3;
};

/// @brief What @ref solve_lens did with one view.
///
/// @code
/// for (const LensView& view : fit.views) {
///   if (view.use == LensView::Use::Dropped) recapture(view.frame_id);
/// }
/// @endcode
struct LensView {
  /// What a view is good for.
  enum class Use : std::uint8_t {
    Fitted,   ///< The lens was fitted to it.
    HeldOut,  ///< Scored through the lens, never fitted.
    Dropped,  ///< Left out by the outlier rule.
    /// Not used: fewer than `min_corners`, or they fix no pose.
    TooFew,
  };
  std::int64_t frame_id = 0;  ///< Its capture, as `Observation::frame_id`.
  Use use{};                  ///< What the solve did with it.
  int corners = 0;            ///< The corners it scored; found, if none did.
  /// Its RMS through the lens over those corners, at the round that dropped
  /// it for a view dropped; infinite for a held-out view with no pose; 0 for
  /// a view not used.
  double rms_px = 0;
};

/// @brief The name calib's results files give @p use.
/// @param use  A use.
/// @return `"fitted"`, `"held_out"`, `"dropped"` or `"too_few"`; `"unknown"`
///         for a value @ref LensView::Use does not name.
///
/// @code
/// for (const LensView& view : fit.views) log(to_string(view.use));
/// @endcode
VC_SOLVE_API const char* to_string(LensView::Use use) noexcept;

/// @brief One camera's solved lens and what it was solved from.
///
/// @code
/// VKC_ASSIGN(const LensFit fit, solve_lens("bm-1", size, board, views));
/// log(fit.camera.model.intrinsics.fx, fit.sigma.fx, fit.holdout_rms_px);
/// @endcode
struct LensFit {
  /// Its id and solved lens at the views' size; source `Calibrated`.
  RigCamera camera;
  /// One sigma of fx, fy, cx, cy. Reported, not gated: an R8 lens's focal
  /// length trades against its coefficients, so only the curve is known well.
  sensor::camera::PinholeIntrinsics sigma;
  double rms_px = 0;  ///< OpenCV's 2D RMS over the fitted corners.
  /// The same over every corner of the held-out views: each posed on its
  /// own, by @ref view_pose then refined through the lens, and nothing
  /// dropped, as the rule would hide what they are held out to show. NaN
  /// with none held out; infinite when one has no pose.
  double holdout_rms_px = 0;
  /// The kept fit began at the given start, not OpenCV's own.
  bool from_start = false;
  std::vector<LensView> views;  ///< Every view of the camera, in capture order.
  /// Every corner of the fitted and held-out views, in that order, its
  /// @ref CornerResidual::camera 0; `dropped` for one the rule left out.
  std::vector<CornerResidual> residuals;
};

/// @brief What @ref solve_lens does with a camera's views before it fits:
///        which it can use, and which it holds out. Readiness counts by it.
/// @param camera_id  The camera; views of others are ignored.
/// @param size       The image the views were detected in.
/// @param board      The board seen.
/// @param views      Any cameras' views, at most one per camera and capture.
/// @param options    What it takes and holds out.
/// @return The camera's views in capture order, whatever the order of
///         @p views, each `Fitted`, `HeldOut` or `TooFew` with its corners
///         found; `InvalidArgument` for an option out of range (naming it), an
///         empty id, an image of no area, two views of one capture, or a
///         corner not finite (naming its capture); or what OpenCV threw
///         (@ref compute::guarded). Each message starts `plan_lens_views: `.
///
/// @code
/// VKC_ASSIGN(const std::vector<LensView> plan,
///            plan_lens_views("bm-1", size, board, views));
/// @endcode
VC_SOLVE_API core::Result<std::vector<LensView>> plan_lens_views(
    std::string_view camera_id, sensor::camera::ImageSize size,
    const compute::Board& board, const std::vector<Observation>& views,
    const LensOptions& options = {});

/// @brief One camera's R8 lens from its board views.
///
/// 1. **Views.** @ref plan_lens_views picks those fitted and those held out.
/// 2. **Fit.** `cv::calibrateCamera` with `CALIB_RATIONAL_MODEL`, from OpenCV's
///    own start and, when given, from @p start with
///    `CALIB_USE_INTRINSIC_GUESS`, on the same corners; the fit with the lower
///    RMS is kept. A start may land in a wrong minimum, or OpenCV may throw on
///    its own; only a fit of higher RMS, or none, costs anything.
/// 3. **Outlier rounds.** After each fit, up to `outlier_rounds` times, the
///    bundle adjustment's bounds drop corners, and a view with none to drop
///    if its RMS is above them; a view whose kept corners fall below
///    `min_corners` or fix no pose goes whole. Step 2 then runs again. The
///    rounds stop when none is dropped.
/// 4. **Hold-out.** Each view held out is posed through the lens and scored.
///
/// The result depends on the views' captures, not on their order in
/// @p views, and is the same on every run but for rounding (about 1e-8 px):
/// OpenCV's threads sum the fit's normal equations in any order.
/// @param camera_id  The camera; views of others are ignored.
/// @param size       The image the views were detected in.
/// @param board      The board seen: its ids need hold in one view only.
/// @param views      Any cameras' views, at most one per camera and capture.
/// @param start      A real lens to start from, at @p size: never a stand-in.
/// @param options    What it takes and holds out.
/// @return The fit; `InvalidArgument` as @ref plan_lens_views, or for a
///         @p start of another size or one sensor refuses, or fewer than
///         `min_views` views to fit, at first or after the outlier rounds
///         (saying how many); `Numerical` when no start gives a finite fit;
///         or what OpenCV threw (@ref compute::guarded). Each message starts
///         `solve_lens: `.
///
/// @code
/// VKC_ASSIGN(const LensFit fit, solve_lens("bm-1", size, board, views, m0));
/// @endcode
VC_SOLVE_API core::Result<LensFit> solve_lens(
    std::string_view camera_id, sensor::camera::ImageSize size,
    const compute::Board& board, const std::vector<Observation>& views,
    const std::optional<sensor::camera::CameraModel>& start = std::nullopt,
    const LensOptions& options = {});

}  // namespace volumetric_kit::calib::solve
