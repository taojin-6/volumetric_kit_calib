// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/compute/cv_pose.hpp"

#include <opencv2/geometry.hpp>

#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::compute {
namespace {

recon::camera::Vec3d recon_vec(const cv::Vec3d& v) noexcept {
  return {v[0], v[1], v[2]};
}

}  // namespace

cv::Matx44d cv_matrix(const recon::camera::Mat4d& m) noexcept {
  cv::Matx44d out;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      out(row, column) = m[column][row];
    }
  }
  return out;
}

recon::camera::Mat4d matrix_from_cv(const cv::Matx44d& m) noexcept {
  recon::camera::Mat4d out(1.0);
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      out[column][row] = m(row, column);
    }
  }
  return out;
}

recon::camera::Mat4d matrix_from_cv_pose(const CvPose& pose) noexcept {
  return recon::camera::matrix_from_rodrigues(
      {recon_vec(pose.rvec), recon_vec(pose.tvec)});
}

core::Result<CvPose> cv_pose(const recon::camera::Mat4d& transform) {
  return guarded("cv_pose", [&]() -> core::Result<CvPose> {
    VKC_TRY(recon::camera::check_rigid(transform).with_context("cv_pose"));
    const cv::Matx44d m = cv_matrix(transform);
    CvPose pose;
    // TODO: take recon's rodrigues_from_matrix, accurate near 0 and pi, once
    // calib's recon pin carries it (recon #228), and drop opencv_geometry
    // from calib_compute's links.
    cv::Rodrigues(m.get_minor<3, 3>(0, 0), pose.rvec);
    pose.tvec = cv::Vec3d(m(0, 3), m(1, 3), m(2, 3));
    return pose;
  });
}

}  // namespace volumetric_kit::calib::compute
