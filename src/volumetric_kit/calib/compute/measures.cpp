// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/compute/measures.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include <glm/trigonometric.hpp>

namespace volumetric_kit::calib::compute {

double median(std::vector<double> values) noexcept {
  const std::size_t n = values.size();
  if (n == 0) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  return n % 2 == 1 ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

double quantile(std::vector<double> values, double q) noexcept {
  if (values.empty() || !(q >= 0.0 && q <= 1.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // Ascending, a NaN after every number, so the order is strict and weak.
  std::sort(values.begin(), values.end(), [](double a, double b) {
    return a < b || (!std::isnan(a) && std::isnan(b));
  });
  const double at = q * static_cast<double>(values.size() - 1);
  const auto lo = static_cast<std::size_t>(at);
  const std::size_t hi = std::min(lo + 1, values.size() - 1);
  const double fraction = at - static_cast<double>(lo);
  const double below = values[lo];
  // Infinite below, the next is the same infinity or a NaN: never inf - inf.
  if (fraction == 0.0 || std::isinf(below)) return below;
  return below + fraction * (values[hi] - below);
}

double angle_deg(const sensor::camera::Vec3d& a,
                 const sensor::camera::Vec3d& b) noexcept {
  return glm::degrees(
      std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b)));
}

double rotation_angle_deg(const sensor::camera::Mat3d& a,
                          const sensor::camera::Mat3d& b) noexcept {
  return glm::degrees(glm::length(
      sensor::camera::rodrigues_from_rotation(glm::transpose(a) * b)));
}

bool finite(const sensor::camera::RodriguesTransform& transform) noexcept {
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(transform.rvec[i]) ||
        !std::isfinite(transform.tvec[i])) {
      return false;
    }
  }
  return true;
}

std::optional<int> grid_cell(double u, double v, int width, int height,
                             int cols, int rows) noexcept {
  const double x = (u + 0.5) / width;
  const double y = (v + 0.5) / height;
  // Written so a NaN fails it.
  if (!(x >= 0.0 && x < 1.0 && y >= 0.0 && y < 1.0)) return std::nullopt;
  // The products stay below cols and rows but for rounding.
  const int col = std::min(static_cast<int>(x * cols), cols - 1);
  const int row = std::min(static_cast<int>(y * rows), rows - 1);
  return row * cols + col;
}

}  // namespace volumetric_kit::calib::compute
