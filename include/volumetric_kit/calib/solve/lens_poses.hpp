// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file lens_poses.hpp
/// @brief A camera's views through a lens held fixed: each posed on its own,
///        as @ref solve_lens poses a held-out view, and each corner's
///        residual. What `quality::check_lens` judges; outside README's
///        stable table.

#include <string_view>
#include <vector>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/export.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"

namespace volumetric_kit::calib::solve {

/// @brief A camera's views through a lens held fixed (@ref pose_lens_views).
///
/// @code
/// VKC_ASSIGN(const PosedLensViews posed,
///            pose_lens_views("bm-1", lens, board, views));
/// for (const CornerResidual& r : posed.residuals) {
///   if (!r.dropped) plot(r.pixel, r.residual);
/// }
/// @endcode
struct PosedLensViews {
  /// Every view of the camera, in capture order: `HeldOut` for one posed and
  /// scored; `Dropped` for one left with no pose, its corners past the bound
  /// leaving fewer than `min_corners` or none fixing a pose, its RMS
  /// infinite; `TooFew` as @ref plan_lens_views says.
  std::vector<LensView> views;
  /// Every corner of the views `HeldOut` and `Dropped`, in capture order, its
  /// @ref CornerResidual::camera 0: `dropped` for one past the bound, and for
  /// every corner of a view `Dropped`, whose residual is not a number.
  std::vector<CornerResidual> residuals;
  /// The corner bound: the adjustment's rule (@ref BundleAdjustOptions's
  /// `outlier_px` and `outlier_mads`) over the corners at each view's pose on
  /// all of them; 0 with no view posed.
  double bound_px = 0;
};

/// @brief A camera's views through @p lens held fixed, each posed on its own
///        as @ref solve_lens poses a held-out view: none is fitted.
///
/// 1. **Views.** Those @ref plan_lens_views would fit or hold out.
/// 2. **Bound.** Each view posed on all its corners through the lens (IPPE's
///    two, @ref view_pose, each refined, the better kept); the corner bound is
///    the adjustment's rule over their residuals, as @ref solve_lens's is over
///    the fitted views'.
/// 3. **Poses.** Each view posed as a held-out one: its corners past the bound
///    marked dropped and the view posed again on the rest, up to
///    `outlier_rounds` times; then scored.
///
/// @param camera_id  The camera; views of others are ignored.
/// @param lens       The lens, held fixed, at the views' size.
/// @param board      The board seen: its ids need hold in one view only.
/// @param views      Any cameras' views, at most one per camera and capture.
/// @param options    `min_corners` and `outlier_rounds` as @ref solve_lens
///                   takes them; the rest is checked, not used.
/// @return The views; `InvalidArgument` as @ref plan_lens_views, or for a
///         lens sensor refuses; or what OpenCV threw (@ref compute::guarded).
///         Each message starts `pose_lens_views: `.
VC_SOLVE_API core::Result<PosedLensViews> pose_lens_views(
    std::string_view camera_id, const sensor::camera::CameraModel& lens,
    const compute::Board& board, const std::vector<Observation>& views,
    const LensOptions& options = {});

}  // namespace volumetric_kit::calib::solve
