// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file detection.hpp
/// @brief What a board detector found in one image: the record the live view
///        draws, the live checks read and a capture stores.
///
/// A plain record in the `core` tier, so the tiers that fill, show and store
/// it need not depend on each other. Pixels are OpenCV's and recon's: pixel
/// centres sit at integer coordinates.

#include <array>
#include <vector>

#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib {

/// @brief One marker found: a board's, or the origin tag.
///
/// @code
/// for (const DetectedMarker& marker : detection.markers) {
///   draw_outline(marker.corners);  // a caller's own drawing
/// }
/// @endcode
struct DetectedMarker {
  int id = -1;  ///< Its id in its dictionary.
  /// Its corners, clockwise from its top-left as printed: OpenCV's order.
  std::array<recon::camera::Vec2d, 4> corners{};
};

/// @brief What a detector found of a board in one image.
///
/// @code
/// Detection detection;
/// detection.image_size = {3840, 2160};
/// detection.observation.camera_id = serial;
/// detection.observation.frame_id = capture_index;
/// detection.observation.points.push_back({0, {812.4, 390.1}, 1.0});
/// @endcode
struct Detection {
  recon::camera::ImageSize image_size;  ///< The image searched.
  /// The board's corners found, by id.
  Observation observation;
  std::vector<DetectedMarker> markers;  ///< The board's markers found.
};

}  // namespace volumetric_kit::calib
