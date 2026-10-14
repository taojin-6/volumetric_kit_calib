// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file cv_camera.hpp
/// @brief recon's camera model in OpenCV's types: the image size, the camera
///        matrix and the eight distortion coefficients; where that lens images
///        a point; and each pixel's ray back through it.
///
/// recon's `RationalDistortion` is OpenCV's rational lens in OpenCV's order,
/// so the conversion is exact: `cv::projectPoints` with these values computes
/// recon's `distort_rational`.

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include "volumetric_kit/calib/compute/export.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::compute {

/// @brief A camera as OpenCV's functions take it.
///
/// @code
/// if (info.color) {  // a recon SensorInfo
///   VKC_ASSIGN(const CvCamera cam, cv_camera(*info.color));
///   std::vector<cv::Point2d> pixels;
///   cv::projectPoints(points, rvec, tvec, cam.camera_matrix, cam.distortion,
///                     pixels);
/// }
/// @endcode
struct CvCamera {
  cv::Size size;  ///< The image the intrinsics describe (pixels).
  /// `[fx 0 cx; 0 fy cy; 0 0 1]`, in pixels of @ref size.
  cv::Matx33d camera_matrix;
  /// `k1, k2, p1, p2, k3, k4, k5, k6`: OpenCV's rational order.
  cv::Vec<double, 8> distortion;
};

/// @brief @p model in OpenCV's types.
/// @param model  The camera.
/// @return The camera; or `InvalidArgument`, its message starting
///         `cv_camera: `, for a model recon's `check_camera_model` refuses or
///         a side longer than OpenCV's `int` holds.
VC_COMPUTE_API core::Result<CvCamera> cv_camera(
    const recon::camera::CameraModel& model);

/// @brief Where @p camera images @p point: recon's `distort_rational`, then
///        the camera matrix's focal lengths and centre, as `cv::projectPoints`
///        computes for a point in the camera's frame.
/// @param camera  The camera.
/// @param point   A point in the camera's frame (+z forward).
/// @return Its pixel, OpenCV's convention (centres at integers); empty when
///         the point is not in front of the camera (z not above 0) or its
///         pixel is not finite (far enough off-axis, the lens overflows).
///
/// @code
/// if (const auto pixel = project(cam, centre_in(board, board_to_camera))) {
///   mark(*pixel);  // a caller's own drawing
/// }
/// @endcode
VC_COMPUTE_API std::optional<recon::camera::Vec2d> project(
    const CvCamera& camera, const recon::camera::Vec3d& point) noexcept;

/// @brief Each pixel's ray as normalized coordinates `(X / Z, Y / Z)`:
///        `cv::undistortPoints`, iterated until the lens takes each ray back
///        to its pixel within 1e-10 px, or 100 times.
///
/// `cv::undistortPoints`' default criteria stop after 5 iterations, which can
/// leave a pixel or more near the corners of a strongly distorted image.
/// @param camera  The camera.
/// @param pixels  Pixels in its image, OpenCV's convention (centres at
///                integers).
/// @return The rays, in order; or what OpenCV threw (@ref guarded).
///
/// @code
/// VKC_ASSIGN(const std::vector<cv::Point2d> rays,
///            undistort_points(cam, corners));
/// @endcode
VC_COMPUTE_API core::Result<std::vector<cv::Point2d>> undistort_points(
    const CvCamera& camera, const std::vector<cv::Point2d>& pixels);

}  // namespace volumetric_kit::calib::compute
