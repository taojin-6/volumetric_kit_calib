// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file cv_pose.hpp
/// @brief recon's rigid transforms in OpenCV's types.
///
/// recon's `Mat4d` is GLM's, column-major (`m[column][row]`); OpenCV's
/// `cv::Matx` is row-major (`m(row, column)`). OpenCV writes a rigid
/// transform as `rvec` and `tvec`, the form recon's `RodriguesTransform`
/// holds: `x' = R(rvec) x + tvec`.

#include <opencv2/core.hpp>

#include "volumetric_kit/calib/compute/export.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::compute {

/// @brief A rigid transform as OpenCV's pose functions take and return one:
///        `x' = R(rvec) x + tvec`.
///
/// @code
/// CvPose pose;
/// cv::solvePnP(object, image, cam.camera_matrix, cam.distortion, pose.rvec,
///              pose.tvec);
/// const recon::camera::Mat4d board_to_camera = matrix_from_cv_pose(pose);
/// @endcode
struct CvPose {
  cv::Vec3d rvec;  ///< The rotation, as a Rodrigues vector (radians).
  cv::Vec3d tvec;  ///< The translation, applied after it (metres).
};

/// @brief @p m as a `cv::Matx44d`, element for element.
/// @param m  A matrix, column-major.
/// @return The same matrix, row-major: `cv_matrix(m)(row, column)` is
///         `m[column][row]`.
VC_COMPUTE_API cv::Matx44d cv_matrix(const recon::camera::Mat4d& m) noexcept;

/// @brief @p m as recon's `Mat4d`, element for element; the inverse of
///        @ref cv_matrix.
/// @param m  A matrix, row-major.
/// @return The same matrix, column-major.
VC_COMPUTE_API recon::camera::Mat4d matrix_from_cv(
    const cv::Matx44d& m) noexcept;

/// @brief The matrix of a transform OpenCV wrote, through recon's
///        `matrix_from_rodrigues`.
/// @param pose  The transform.
/// @return `[R(rvec) | tvec]`.
VC_COMPUTE_API recon::camera::Mat4d matrix_from_cv_pose(
    const CvPose& pose) noexcept;

/// @brief A rigid transform in OpenCV's form.
///
/// The rotation goes through `cv::Rodrigues`, which returns exactly zero for
/// a rotation below about 1e-5 rad: fine for an initial value, not for a
/// solver's state.
/// @param transform  The transform.
/// @return The transform; or `InvalidArgument` for one recon's `check_rigid`
///         refuses.
///
/// @code
/// VKC_ASSIGN(const CvPose pose, cv_pose(world_to_camera));
/// @endcode
VC_COMPUTE_API core::Result<CvPose> cv_pose(
    const recon::camera::Mat4d& transform);

}  // namespace volumetric_kit::calib::compute
