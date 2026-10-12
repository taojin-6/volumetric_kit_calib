// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/compute/cv_camera.hpp"

#include <cstdint>
#include <limits>

namespace volumetric_kit::calib::compute {

core::Result<CvCamera> cv_camera(const recon::camera::CameraModel& model) {
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
}

}  // namespace volumetric_kit::calib::compute
