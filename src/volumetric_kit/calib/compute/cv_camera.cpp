// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/compute/cv_camera.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

#include <opencv2/geometry.hpp>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/recon/camera/projection.hpp"

namespace volumetric_kit::calib::compute {

core::Result<CvCamera> cv_camera(const recon::camera::CameraModel& model) {
  return guarded("cv_camera", [&]() -> core::Result<CvCamera> {
    VKC_TRY(recon::camera::check_camera_model(model).with_context("cv_camera"));
    constexpr std::uint32_t kMaxSide = std::numeric_limits<int>::max();
    if (model.size.width > kMaxSide || model.size.height > kMaxSide) {
      return core::Status::invalid_argument(
          "cv_camera: the image is larger than OpenCV's int sizes hold");
    }
    const recon::camera::PinholeIntrinsics& k = model.intrinsics;
    const recon::camera::RationalDistortion& d = model.distortion;
    CvCamera camera;
    camera.size = cv::Size(static_cast<int>(model.size.width),
                           static_cast<int>(model.size.height));
    camera.camera_matrix =
        cv::Matx33d(k.fx, 0.0, k.cx, 0.0, k.fy, k.cy, 0.0, 0.0, 1.0);
    camera.distortion =
        cv::Vec<double, 8>(d.k1, d.k2, d.p1, d.p2, d.k3, d.k4, d.k5, d.k6);
    return camera;
  });
}

std::optional<recon::camera::Vec2d> project(
    const CvCamera& camera, const recon::camera::Vec3d& point) noexcept {
  if (!(point.z > 0.0)) return std::nullopt;
  double xd = 0.0;
  double yd = 0.0;
  recon::camera::distort_rational(camera.distortion.val, point.x / point.z,
                                  point.y / point.z, &xd, &yd);
  const cv::Matx33d& k = camera.camera_matrix;
  const recon::camera::Vec2d pixel(k(0, 0) * xd + k(0, 2),
                                   k(1, 1) * yd + k(1, 2));
  if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y)) return std::nullopt;
  return pixel;
}

core::Result<std::vector<cv::Point2d>> undistort_points(
    const CvCamera& camera, const std::vector<cv::Point2d>& pixels) {
  return guarded(
      "undistort_points", [&]() -> core::Result<std::vector<cv::Point2d>> {
        std::vector<cv::Point2d> rays;
        if (pixels.empty()) return rays;
        cv::undistortPoints(
            pixels, rays, camera.camera_matrix, camera.distortion,
            cv::noArray(), cv::noArray(),
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                             100, 1e-10));
        return rays;
      });
}

}  // namespace volumetric_kit::calib::compute
