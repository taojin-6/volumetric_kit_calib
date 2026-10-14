// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "reprojection.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

#include <opencv2/geometry.hpp>

namespace volumetric_kit::calib::solve::detail {

std::vector<double> squared_errors(
    const compute::CvCamera& camera, const compute::Board& board,
    const Observation& view, const recon::camera::Mat4d& board_to_camera) {
  std::vector<cv::Point3d> in_camera;
  in_camera.reserve(view.points.size());
  for (const DetectedPoint& point : view.points) {
    const cv::Point3d& x =
        board.object_points()[static_cast<std::size_t>(point.id)];
    const glm::dvec4 q = board_to_camera * glm::dvec4(x.x, x.y, x.z, 1.0);
    in_camera.emplace_back(q.x, q.y, q.z);
  }
  std::vector<double> errors(view.points.size(),
                             std::numeric_limits<double>::infinity());
  if (in_camera.empty()) return errors;
  std::vector<cv::Point2d> pixels;
  cv::projectPoints(in_camera, cv::Vec3d(0.0, 0.0, 0.0),
                    cv::Vec3d(0.0, 0.0, 0.0), camera.camera_matrix,
                    camera.distortion, pixels);
  for (std::size_t i = 0; i < errors.size(); ++i) {
    if (!(in_camera[i].z > 0.0)) continue;
    const recon::camera::Vec2d& seen = view.points[i].pixel;
    const double dx = pixels[i].x - seen.x;
    const double dy = pixels[i].y - seen.y;
    const double e2 = dx * dx + dy * dy;
    if (!std::isnan(e2)) errors[i] = e2;
  }
  return errors;
}

}  // namespace volumetric_kit::calib::solve::detail
