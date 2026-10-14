// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/compute/cv_pose.hpp"

#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::compute {
namespace {

recon::camera::Vec3d recon_vec(const cv::Vec3d& v) noexcept {
  return {v[0], v[1], v[2]};
}

cv::Vec3d cv_vec(const recon::camera::Vec3d& v) noexcept {
  return {v.x, v.y, v.z};
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
    const recon::camera::RodriguesTransform pose =
        recon::camera::rodrigues_from_matrix(transform);
    return CvPose{cv_vec(pose.rvec), cv_vec(pose.tvec)};
  });
}

}  // namespace volumetric_kit::calib::compute
