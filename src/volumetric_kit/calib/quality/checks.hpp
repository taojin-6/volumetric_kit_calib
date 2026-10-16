// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// What the quality tier's checks share: private to the tier.

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "volumetric_kit/calib/quality/live_checks.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::quality::detail {

// The cell of a cols x rows grid over an image of `size` that holds the pixel
// (u, v), row-major from the top left; empty outside the image, whose edge is
// at -0.5 and size - 0.5 (OpenCV puts pixel centres at integers).
inline std::optional<int> grid_cell(double u, double v, const cv::Size& size,
                                    int cols, int rows) noexcept {
  const double x = (u + 0.5) / size.width;
  const double y = (v + 0.5) / size.height;
  // Written so a NaN fails it.
  if (!(x >= 0.0 && x < 1.0 && y >= 0.0 && y < 1.0)) return std::nullopt;
  // The products stay below cols and rows but for rounding.
  const int col = std::min(static_cast<int>(x * cols), cols - 1);
  const int row = std::min(static_cast<int>(y * rows), rows - 1);
  return row * cols + col;
}

// Refuses options out of range, each message `prefix` and the field's name.
inline core::Status check_view_options(const ViewCheckOptions& options,
                                       const std::string& prefix) {
  if (!(options.oblique_deg > 0.0 && options.oblique_deg <= 180.0)) {
    return core::Status::invalid_argument(prefix +
                                          "oblique_deg must be in (0, 180]");
  }
  if (!(options.far_m > 0.0) || !std::isfinite(options.far_m)) {
    return core::Status::invalid_argument(prefix +
                                          "far_m must be positive and finite");
  }
  if (!(options.frontal_deg >= 0.0 && options.frontal_deg <= 180.0)) {
    return core::Status::invalid_argument(prefix +
                                          "frontal_deg must be in [0, 180]");
  }
  return {};
}

}  // namespace volumetric_kit::calib::quality::detail
