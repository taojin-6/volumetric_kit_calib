// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file detection.hpp
/// @brief What a detector found of a marker board in one image: its corners by
///        id, and its markers.
///
/// A plain record in the `core` tier, so the tier that detects and the one
/// that stores a capture need not depend on each other. Pixels are OpenCV's
/// and recon's: pixel centres sit at integer coordinates.

#include <array>
#include <vector>

#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib {

/// @brief One marker found.
///
/// @code
/// DetectedMarker marker;
/// marker.id = 3;
/// marker.corners = {{{100.0, 50.0}, {140.0, 50.0}, {140.0, 90.0},
///                    {100.0, 90.0}}};
/// @endcode
struct DetectedMarker {
  int id = -1;  ///< Its id in the board's dictionary.
  /// Its corners, clockwise from its top-left as printed: OpenCV's order.
  std::array<recon::camera::Vec2d, 4> corners{};
};

/// @brief What a detector found of a board in one image.
///
/// @code
/// Detection detection;
/// detection.image_size = {3840, 2160};
/// detection.observation.points.push_back({0, {812.4, 390.1}, 1.0});
/// detection.markers.push_back(marker);
/// @endcode
struct Detection {
  /// The image searched, whose pixels the corners are in.
  recon::camera::ImageSize image_size;
  /// The board's corners, by id, in ascending order.
  Observation observation;
  /// The markers found, in ascending order of id.
  std::vector<DetectedMarker> markers;
};

}  // namespace volumetric_kit::calib
