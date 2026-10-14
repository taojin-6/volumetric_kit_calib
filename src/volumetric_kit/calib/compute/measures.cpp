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
