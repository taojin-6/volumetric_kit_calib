// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file luma.hpp
/// @brief The 8-bit luma image the detectors read.
///
/// Pixels are OpenCV's and sensor's: pixel centres sit at integer coordinates,
/// so a W-pixel-wide image spans `[-0.5, W - 0.5]`.

#include <cstddef>
#include <cstdint>

namespace volumetric_kit::calib::detect {

/// @brief An 8-bit luma image the caller owns: @ref height rows of
///        @ref width bytes, @ref stride bytes apart.
///
/// sensor's `PictureView` maps onto it field for field.
///
/// @code
/// const LumaView luma{view.width, view.height, view.stride, view.luma};
/// @endcode
struct LumaView {
  std::uint32_t width = 0;             ///< Pixels per row.
  std::uint32_t height = 0;            ///< Rows.
  std::size_t stride = 0;              ///< Bytes between rows; >= @ref width.
  const std::uint8_t* data = nullptr;  ///< The first row's first pixel.
};

}  // namespace volumetric_kit::calib::detect
