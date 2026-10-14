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

void project(const compute::CvCamera& camera, const Pose& world_to_camera,
             const Pose& board_to_world, const std::vector<cv::Point3d>& object,
             Derivatives derivatives, Projection& out) {
  const std::size_t n = object.size();
  out.pixels.clear();
  out.depths.assign(n, 0.0);
  out.by_camera.clear();
  out.by_board.clear();
  out.by_focal.clear();
  out.by_centre.clear();
  if (n == 0) return;
  const cv::Vec3d r_cw(world_to_camera[0], world_to_camera[1],
                       world_to_camera[2]);
  const cv::Vec3d t_cw(world_to_camera[3], world_to_camera[4],
                       world_to_camera[5]);
  const cv::Vec3d r_wb(board_to_world[0], board_to_world[1], board_to_world[2]);
  const cv::Vec3d t_wb(board_to_world[3], board_to_world[4], board_to_world[5]);
  cv::Matx33d rotation_wb;
  cv::Matx<double, 3, 9> by_r_wb;  // d R_wb (row-major) / d r_wb
  if (derivatives == Derivatives::None) {
    cv::Rodrigues(r_wb, rotation_wb);
  } else {
    cv::Rodrigues(r_wb, rotation_wb, by_r_wb);
  }
  cv::Matx33d rotation_cw;
  cv::Rodrigues(r_cw, rotation_cw);

  std::vector<cv::Point3d> world(n);
  for (std::size_t i = 0; i < n; ++i) {
    const cv::Vec3d x =
        rotation_wb * cv::Vec3d(object[i].x, object[i].y, object[i].z) + t_wb;
    world[i] = cv::Point3d(x[0], x[1], x[2]);
    out.depths[i] = (rotation_cw * x + t_cw)[2];
  }
  if (derivatives == Derivatives::None) {
    cv::projectPoints(world, r_cw, t_cw, camera.camera_matrix,
                      camera.distortion, out.pixels);
    return;
  }
  cv::Mat dpdr;
  cv::Mat dpdt;
  cv::Mat dpdf;
  cv::Mat dpdc;
  if (derivatives == Derivatives::All) {
    cv::projectPoints(world, r_cw, t_cw, camera.camera_matrix,
                      camera.distortion, out.pixels, dpdr, dpdt, dpdf, dpdc);
  } else {
    cv::projectPoints(world, r_cw, t_cw, camera.camera_matrix,
                      camera.distortion, out.pixels, dpdr, dpdt);
  }
  out.by_camera.resize(n);
  out.by_board.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    const int row = 2 * static_cast<int>(i);
    cv::Matx23d by_x_camera;  // dp/dtvec, which is dp/dX_c
    CornerJacobian& by_camera = out.by_camera[i];
    for (int a = 0; a < 2; ++a) {
      for (int k = 0; k < 3; ++k) {
        by_camera(a, k) = dpdr.at<double>(row + a, k);
        by_camera(a, k + 3) = dpdt.at<double>(row + a, k);
        by_x_camera(a, k) = dpdt.at<double>(row + a, k);
      }
    }
    const cv::Matx23d by_x_world = by_x_camera * rotation_cw;
    const double x[3] = {object[i].x, object[i].y, object[i].z};
    CornerJacobian& by_board = out.by_board[i];
    for (int k = 0; k < 3; ++k) {
      // dX_w / dr_wb[k] = (dR_wb / dr_wb[k]) X.
      cv::Vec3d moved;
      for (int a = 0; a < 3; ++a) {
        moved[a] = by_r_wb(k, 3 * a) * x[0] + by_r_wb(k, 3 * a + 1) * x[1] +
                   by_r_wb(k, 3 * a + 2) * x[2];
      }
      const cv::Vec2d by_r = by_x_world * moved;
      by_board(0, k) = by_r[0];
      by_board(1, k) = by_r[1];
      by_board(0, k + 3) = by_x_world(0, k);
      by_board(1, k + 3) = by_x_world(1, k);
    }
  }
  if (derivatives != Derivatives::All) return;
  out.by_focal.resize(n);
  out.by_centre.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    const int row = 2 * static_cast<int>(i);
    for (int a = 0; a < 2; ++a) {
      for (int k = 0; k < 2; ++k) {
        out.by_focal[i](a, k) = dpdf.at<double>(row + a, k);
        out.by_centre[i](a, k) = dpdc.at<double>(row + a, k);
      }
    }
  }
}

}  // namespace volumetric_kit::calib::solve::detail
