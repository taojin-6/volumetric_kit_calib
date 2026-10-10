// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file luma_image.hpp
/// @brief A camera's luma on the host: what calibration detects on, and what a
///        capture session stores.

#include <cstdint>
#include <vector>

#include "volumetric_kit/calib/core/camera_model.hpp"

namespace volumetric_kit::calib::capture {

/// @brief An 8-bit single-channel image the caller owns: a camera's luma as
///        captured, before undistortion.
///
/// @code
/// const std::uint8_t centre =
///     image.pixels[(image.size.height / 2) * image.size.width +
///                  image.size.width / 2];
/// @endcode
struct LumaImage {
  ImageSize size;  ///< Width and height (pixels).
  /// Row-major, `size.width * size.height` bytes, rows tightly packed.
  std::vector<std::uint8_t> pixels;

  /// @return `true` when the image has no pixels.
  bool empty() const noexcept { return pixels.empty(); }
};

}  // namespace volumetric_kit::calib::capture
