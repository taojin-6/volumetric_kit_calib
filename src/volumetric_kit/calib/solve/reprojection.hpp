// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The solve's one projection of a view's corners: private to the tier.

#include <vector>

#include <opencv2/core.hpp>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::solve::detail {

// A transform as the adjustment holds it: the Rodrigues vector, then the
// translation.
using Pose = cv::Vec6d;

// `t` as a Pose.
inline Pose pose_of(const recon::camera::RodriguesTransform& t) noexcept {
  return {t.rvec.x, t.rvec.y, t.rvec.z, t.tvec.x, t.tvec.y, t.tvec.z};
}

// What `project` differentiates the residuals by.
enum class Derivatives {
  None,
  Extrinsics,  // the camera's and the board's transforms
  All,         // and the focal lengths and the principal point
};

// A view's corners through its camera.
struct Projection {
  std::vector<cv::Point2d> pixels;
  std::vector<cv::Vec2d> residuals;  // pixels minus those detected, if given
  std::vector<double> depths;        // z in the camera
  std::vector<CornerJacobian> by_camera;
  std::vector<CornerJacobian> by_board;
  std::vector<cv::Matx22d> by_focal;   // d/d(fx, fy)
  std::vector<cv::Matx22d> by_centre;  // d/d(cx, cy)
};

// `object`, in the board's frame, through `camera` with the board at
// `board_to_world` and the camera at `world_to_camera`: cv::projectPoints of
// the corners in the world, and the residuals against `detected` (in
// `object`'s order) unless it is null. The camera block is its dp/drvec and
// dp/dtvec, the board block dp/dX_w = (dp/dtvec) R_cw through the
// vector-to-matrix derivative of cv::Rodrigues(r_wb), and `All` adds its
// dp/df and dp/dc. No matrix is turned back into a vector. Throws what OpenCV
// throws.
void project(const compute::CvCamera& camera, const Pose& world_to_camera,
             const Pose& board_to_world, const std::vector<cv::Point3d>& object,
             const std::vector<cv::Point2d>* detected, Derivatives derivatives,
             Projection& out);

// Each of `view`'s corners' squared reprojection error (pixels^2) with the
// board at `board_to_camera`: moved into the camera in double, then through
// the lens, as `project` projects. A corner not in front of the camera
// (z <= 0, or not finite) is infinity. `view`'s ids must be the board's
// (compute::usable_for_pose). Throws what OpenCV throws.
std::vector<double> squared_errors(const compute::CvCamera& camera,
                                   const compute::Board& board,
                                   const Observation& view,
                                   const recon::camera::Mat4d& board_to_camera);

}  // namespace volumetric_kit::calib::solve::detail
