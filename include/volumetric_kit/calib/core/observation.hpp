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
///
/// @code
/// DetectedPoint corner;
/// corner.id = 17;
/// corner.pixel = {812.4, 390.1};
/// corner.weight = 1.0 / (0.05 * 0.05);  // refined to sigma = 0.05 px
/// @endcode
struct DetectedPoint {
  int id = -1;                  ///< Stable target-feature id (corner/marker).
  Eigen::Vector2d pixel{0, 0};  ///< Subpixel location (pixels).
  /// Inverse-variance weight from refinement (1/px^2).
  double weight = 1.0;
};

/// @brief All features one camera detected in one frame.
///
/// @code
/// Observation obs;
/// obs.camera_id = "bm-1";
/// obs.frame_id = 42;  // the 43rd synchronized capture set
/// obs.points.push_back(corner);
/// @endcode
struct Observation {
  std::string camera_id;  ///< Stable camera identifier (e.g. serial).
  /// Index of the capture set this frame belongs to; frames from different
  /// cameras with the same index were captured together. An index, not a
  /// timestamp.
  std::int64_t frame_id = 0;
  std::vector<DetectedPoint> points;  ///< Detected features in this frame.

  /// @return The number of detected features.
  std::size_t size() const noexcept { return points.size(); }
  /// @return `true` if no features were detected.
  bool empty() const noexcept { return points.empty(); }
};

}  // namespace volumetric_kit::calib
