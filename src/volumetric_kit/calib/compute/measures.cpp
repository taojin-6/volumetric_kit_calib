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

double angle_deg(const recon::camera::Vec3d& a,
                 const recon::camera::Vec3d& b) noexcept {
  return glm::degrees(
      std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b)));
}

double rotation_angle_deg(const recon::camera::Mat3d& a,
                          const recon::camera::Mat3d& b) noexcept {
  return glm::degrees(glm::length(
      recon::camera::rodrigues_from_rotation(glm::transpose(a) * b)));
}

}  // namespace volumetric_kit::calib::compute
