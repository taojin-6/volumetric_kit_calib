// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/core/camera_model.hpp"

#include <cmath>

#include "volumetric_kit/calib/core/check.hpp"

namespace volumetric_kit::calib {

Result<PinholeBrownConrady> PinholeBrownConrady::create(
    const Intrinsics& intrinsics, const Distortion& distortion) {
  if (!std::isfinite(intrinsics.fx) || !std::isfinite(intrinsics.fy) ||
      !std::isfinite(intrinsics.cx) || !std::isfinite(intrinsics.cy)) {
    return Status::invalid_argument("intrinsics must be finite");
  }
  if (intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
    return Status::invalid_argument("focal lengths fx, fy must be positive");
  }
  return PinholeBrownConrady{intrinsics, distortion};
}

Result<PinholeBrownConrady> PinholeBrownConrady::create(
    const Intrinsics& intrinsics) {
  return create(intrinsics, Distortion{});
}

Eigen::Vector2d PinholeBrownConrady::project(
    const Eigen::Vector3d& point_camera) const {
  VC_CHECK(point_camera.z() > 0.0, "project() requires a point in front (z>0)");

  const double x = point_camera.x() / point_camera.z();
  const double y = point_camera.y() / point_camera.z();

  const double r2 = x * x + y * y;
  const double radial = 1.0 + distortion_.k1 * r2 + distortion_.k2 * r2 * r2 +
                        distortion_.k3 * r2 * r2 * r2;

  const double x_d = x * radial + 2.0 * distortion_.p1 * x * y +
                     distortion_.p2 * (r2 + 2.0 * x * x);
  const double y_d = y * radial + distortion_.p1 * (r2 + 2.0 * y * y) +
                     2.0 * distortion_.p2 * x * y;

  return {intrinsics_.fx * x_d + intrinsics_.cx,
          intrinsics_.fy * y_d + intrinsics_.cy};
}

Eigen::Vector2d PinholeBrownConrady::unproject(const Eigen::Vector2d& pixel,
                                               int iterations) const {
  // Distorted normalized coordinates.
  const double x_distorted = (pixel.x() - intrinsics_.cx) / intrinsics_.fx;
  const double y_distorted = (pixel.y() - intrinsics_.cy) / intrinsics_.fy;

  // Fixed-point iteration to remove distortion (OpenCV's scheme): start from
  // the distorted point and converge to the undistorted normalized point.
  double x = x_distorted;
  double y = y_distorted;
  for (int i = 0; i < iterations; ++i) {
    const double r2 = x * x + y * y;
    const double radial = 1.0 + distortion_.k1 * r2 + distortion_.k2 * r2 * r2 +
                          distortion_.k3 * r2 * r2 * r2;
    const double dx =
        2.0 * distortion_.p1 * x * y + distortion_.p2 * (r2 + 2.0 * x * x);
    const double dy =
        distortion_.p1 * (r2 + 2.0 * y * y) + 2.0 * distortion_.p2 * x * y;
    x = (x_distorted - dx) / radial;
    y = (y_distorted - dy) / radial;
  }
  return {x, y};
}

}  // namespace volumetric_kit::calib
