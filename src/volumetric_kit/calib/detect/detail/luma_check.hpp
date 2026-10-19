// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The image check every detector runs before OpenCV reads a LumaView.

#include <cstdint>
#include <limits>
#include <string>
#include <string_view>

#include "volumetric_kit/calib/detect/luma.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::detect::detail {

// OK when OpenCV can read `image` as a CV_8UC1 Mat: at least one pixel,
// sides within OpenCV's int, data set, and rows at least `width` bytes apart.
// Otherwise InvalidArgument, its message starting "<caller>: ". It builds the
// message, which can throw bad_alloc, so it runs inside compute::guarded.
inline core::Status check_luma(const LumaView& image, std::string_view caller) {
  const auto refused = [caller](const char* what) {
    return core::Status::invalid_argument(std::string(caller) + ": " + what);
  };
  if (image.width < 1 || image.height < 1) {
    return refused("the image has no pixels");
  }
  constexpr std::uint32_t kMaxSide = std::numeric_limits<int>::max();
  if (image.width > kMaxSide || image.height > kMaxSide) {
    return refused("the image is larger than OpenCV's int sizes hold");
  }
  if (image.data == nullptr) return refused("image.data is null");
  if (image.stride < image.width) {
    return refused("image.stride is below image.width");
  }
  return {};
}

}  // namespace volumetric_kit::calib::detect::detail
