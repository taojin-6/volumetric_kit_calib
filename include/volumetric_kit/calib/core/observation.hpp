// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file observation.hpp
/// @brief A single camera's detected target points in one frame.
///
/// The atomic unit the solve consumes: which target feature (`id`) was seen at
/// which subpixel location (`pixel`), with what confidence (`weight`, the
/// inverse-variance from subpixel refinement). Identified features (ChArUco /
/// AprilGrid) carry stable ids; for a plain checkerboard the id is the
/// row-major corner index.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace volumetric_kit::calib {

/// @brief One detected target feature in image space.
struct DetectedPoint {
  int id = -1;                  ///< Stable target-feature id (corner/marker).
  Eigen::Vector2d pixel{0, 0};  ///< Subpixel location (pixels).
  double weight = 1.0;          ///< Inverse-variance weight from refinement.
};

/// @brief All features one camera detected in one frame.
struct Observation {
  std::string camera_id;      ///< Stable camera identifier (e.g. serial).
  std::int64_t frame_id = 0;  ///< Frame/timestamp index for multi-cam sync.
  std::vector<DetectedPoint> points;  ///< Detected features in this frame.

  /// @return The number of detected features.
  std::size_t size() const noexcept { return points.size(); }
  /// @return `true` if no features were detected.
  bool empty() const noexcept { return points.empty(); }
};

}  // namespace volumetric_kit::calib
