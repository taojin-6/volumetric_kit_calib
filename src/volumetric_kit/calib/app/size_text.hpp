// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The app tier's own: a picture's size as its messages write it.

#include <string>

#include "volumetric_kit/sensor/camera/camera_model.hpp"

namespace volumetric_kit::calib::app {

// `3840x2160`.
inline std::string size_text(sensor::camera::ImageSize size) {
  return std::to_string(size.width) + "x" + std::to_string(size.height);
}

}  // namespace volumetric_kit::calib::app
