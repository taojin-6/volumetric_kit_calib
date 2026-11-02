// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file lens_readiness.hpp
/// @brief Whether a camera's views are enough to fit its lens: what intrinsic
///        mode shows while the board is moved about, and what enables its
///        Calibrate. Counted on the views the solve will fit.

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/quality/export.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"

namespace volumetric_kit::calib::quality {

/// @brief What a camera's views must reach to be ready.
///
/// @code
/// LensReadyOptions options;
/// options.min_views = 40;
/// @endcode
struct LensReadyOptions {
  int min_views = 30;  ///< Views to fit, at least; at least 1.
  /// Cells of the 8 x 6 grid holding a corner, as a share, at least.
  double min_cells = 0.75;
  /// Reach (@ref solve::LensReach) toward each image corner and side, at least.
  double min_reach = solve::kLensReach;
  /// Frontal views (tilted under 20 degrees), as a share, at most.
  double max_frontal = 0.5;
  /// Which views the solve will fit (@ref solve::plan_lens_views).
  solve::LensOptions lens;
};

/// @brief How far along a camera's views are.
///
/// @code
/// VKC_ASSIGN(const LensReadiness state,
///            lens_readiness("bm-1", size, board, views, &frontal));
/// show(state.views, state.cells, state.reach, state.ready);
/// @endcode
struct LensReadiness {
  int views = 0;  ///< The views the solve will fit.
  /// The share of 8 x 6 cells holding one of their corners.
  double cells = 0.0;
  solve::LensReach reach{};  ///< How far their corners get.
  /// The share of them frontal; empty without a real lens to tell by.
  std::optional<double> frontal;
  bool ready = false;  ///< All of it enough.
};

/// @brief How far along @p camera_id's views are.
/// @param camera_id  The camera; views of others are ignored.
/// @param size       The image the views were detected in.
/// @param board      The board seen.
/// @param views      Any cameras' views, as @ref solve::solve_lens takes them.
/// @param frontal    The captures whose view of this camera is frontal
///                   (`CheckedView::frontal`); null with no real lens: a
///                   stand-in's tilt is wrong at a fisheye's edge, so frontal
///                   views are not judged.
/// @param options    What ready means.
/// @return The state; `InvalidArgument` for an option out of range (naming it)
///         or what @ref solve::plan_lens_views refuses; or what OpenCV threw
///         (@ref compute::guarded). Each message starts `lens_readiness: `.
VC_QUALITY_API core::Result<LensReadiness> lens_readiness(
    std::string_view camera_id, sensor::camera::ImageSize size,
    const compute::Board& board, const std::vector<Observation>& views,
    const std::vector<std::int64_t>* frontal,
    const LensReadyOptions& options = {});

}  // namespace volumetric_kit::calib::quality
