// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "reprojection.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include <opencv2/geometry.hpp>

namespace volumetric_kit::calib::solve::detail {
namespace {

// `points`, in the world, through `camera` at `world_to_camera`: their
// pixels, depths and residuals against `detected` (unless null), and the
// derivatives asked for, but for the board block, which `project` adds.
void project_points(const compute::CvCamera& camera,
                    const Pose& world_to_camera,
                    const std::vector<cv::Point3d>& points,
                    const std::vector<cv::Point2d>* detected,
                    Derivatives derivatives, Projection& out) {
  const std::size_t n = points.size();
  out.pixels.clear();
  out.residuals.clear();
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
  cv::Matx33d rotation_cw;
  cv::Rodrigues(r_cw, rotation_cw);
  for (std::size_t i = 0; i < n; ++i) {
    const cv::Vec3d x(points[i].x, points[i].y, points[i].z);
    out.depths[i] = (rotation_cw * x + t_cw)[2];
  }
  // OpenCV sizes the pixels in its own, uninstrumented, code: a fresh vector
  // each time, as AddressSanitizer's container checks need.
  std::vector<cv::Point2d> pixels;
  cv::Mat dpdr;
  cv::Mat dpdt;
  cv::Mat dpdf;
  cv::Mat dpdc;
  if (derivatives == Derivatives::None) {
    cv::projectPoints(points, r_cw, t_cw, camera.camera_matrix,
                      camera.distortion, pixels);
  } else if (derivatives == Derivatives::All) {
    cv::projectPoints(points, r_cw, t_cw, camera.camera_matrix,
                      camera.distortion, pixels, dpdr, dpdt, dpdf, dpdc);
  } else {
    cv::projectPoints(points, r_cw, t_cw, camera.camera_matrix,
                      camera.distortion, pixels, dpdr, dpdt);
  }
  out.pixels = std::move(pixels);
  if (detected != nullptr) {
    out.residuals.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      out.residuals.emplace_back(out.pixels[i].x - (*detected)[i].x,
                                 out.pixels[i].y - (*detected)[i].y);
    }
  }
  if (derivatives == Derivatives::None) return;
  out.by_camera.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    const int row = 2 * static_cast<int>(i);
    for (int a = 0; a < 2; ++a) {
      for (int k = 0; k < 3; ++k) {
        out.by_camera[i](a, k) = dpdr.at<double>(row + a, k);
        out.by_camera[i](a, k + 3) = dpdt.at<double>(row + a, k);
      }
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

}  // namespace

void project(const compute::CvCamera& camera, const Pose& world_to_camera,
             const Pose& board_to_world, const std::vector<cv::Point3d>& object,
             const std::vector<cv::Point2d>* detected, Derivatives derivatives,
             Projection& out) {
  const std::size_t n = object.size();
  const cv::Vec3d r_wb(board_to_world[0], board_to_world[1], board_to_world[2]);
  const cv::Vec3d t_wb(board_to_world[3], board_to_world[4], board_to_world[5]);
  cv::Matx33d rotation_wb;
  cv::Matx<double, 3, 9> by_r_wb;  // d R_wb (row-major) / d r_wb
  if (derivatives == Derivatives::None) {
    cv::Rodrigues(r_wb, rotation_wb);
  } else {
    cv::Rodrigues(r_wb, rotation_wb, by_r_wb);
  }
  std::vector<cv::Point3d> world(n);
  for (std::size_t i = 0; i < n; ++i) {
    const cv::Vec3d x =
        rotation_wb * cv::Vec3d(object[i].x, object[i].y, object[i].z) + t_wb;
    world[i] = cv::Point3d(x[0], x[1], x[2]);
  }
  project_points(camera, world_to_camera, world, detected, derivatives, out);
  if (derivatives == Derivatives::None || n == 0) return;

  cv::Matx33d rotation_cw;
  cv::Rodrigues(
      cv::Vec3d(world_to_camera[0], world_to_camera[1], world_to_camera[2]),
      rotation_cw);
  out.by_board.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    // dp/dX_w = (dp/dX_c) R_cw, where dp/dX_c is dp/dtvec.
    const cv::Matx23d by_x_world =
        out.by_camera[i].get_minor<2, 3>(0, 3) * rotation_cw;
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
}

std::vector<double> squared_errors(
    const compute::CvCamera& camera, const compute::Board& board,
    const Observation& view, const recon::camera::Mat4d& board_to_camera) {
  const std::size_t n = view.points.size();
  std::vector<cv::Point3d> in_camera;
  std::vector<cv::Point2d> detected;
  in_camera.reserve(n);
  detected.reserve(n);
  for (const DetectedPoint& point : view.points) {
    const cv::Point3d& x =
        board.object_points()[static_cast<std::size_t>(point.id)];
    const glm::dvec4 q = board_to_camera * glm::dvec4(x.x, x.y, x.z, 1.0);
    in_camera.emplace_back(q.x, q.y, q.z);
    detected.emplace_back(point.pixel.x, point.pixel.y);
  }
  std::vector<double> errors(n, std::numeric_limits<double>::infinity());
  Projection at;
  project_points(camera, Pose::all(0.0), in_camera, &detected,
                 Derivatives::None, at);
  for (std::size_t i = 0; i < n; ++i) {
    if (!(at.depths[i] > 0.0)) continue;
    const double e2 = at.residuals[i].dot(at.residuals[i]);
    if (!std::isnan(e2)) errors[i] = e2;
  }
  return errors;
}

}  // namespace volumetric_kit::calib::solve::detail
