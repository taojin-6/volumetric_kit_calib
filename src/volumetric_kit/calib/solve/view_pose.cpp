// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/solve/view_pose.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/geometry.hpp>

#include "reprojection.hpp"
#include "volumetric_kit/calib/compute/cv_pose.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/compute/usable_for_pose.hpp"

namespace volumetric_kit::calib::solve {
namespace {

namespace camera = recon::camera;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// OpenCV's 3-vector, whatever its shape and depth, in double; NaN unless it
// holds three values.
cv::Vec3d read_vec3(const cv::Mat& m) {
  if (m.total() * static_cast<std::size_t>(m.channels()) != 3) {
    return cv::Vec3d::all(std::numeric_limits<double>::quiet_NaN());
  }
  cv::Mat d;
  m.convertTo(d, CV_64F);
  const cv::Mat flat = d.reshape(1, 3);
  return {flat.at<double>(0), flat.at<double>(1), flat.at<double>(2)};
}

bool finite(const cv::Vec3d& v) noexcept {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

// IPPE's two solutions in one frame of the board.
using Solutions = std::array<PoseCandidate, 2>;

// The board turned a quarter turn about its normal: (x, y, z) to (-y, x, z).
cv::Point3d quarter_turned(const cv::Point3d& p) noexcept {
  return {-p.y, p.x, p.z};
}

// A pose of the quarter-turned board as one of the board: T' Q, Q the turn,
// which moves T''s columns exactly.
camera::Mat4d unturned(camera::Mat4d board_to_camera) noexcept {
  const glm::dvec4 x = board_to_camera[0];
  board_to_camera[0] = board_to_camera[1];
  board_to_camera[1] = -x;
  return board_to_camera;
}

// How far apart two poses put the board's origin (metres).
double apart(const PoseCandidate& a, const PoseCandidate& b) noexcept {
  return glm::length(
      camera::Vec3d(a.board_to_camera[3] - b.board_to_camera[3]));
}

}  // namespace

double ViewPose::ambiguity() const noexcept {
  if (!alternative) return std::numeric_limits<double>::infinity();
  if (best.rms_px > 0.0) return alternative->rms_px / best.rms_px;
  return alternative->rms_px > 0.0 ? std::numeric_limits<double>::infinity()
                                   : 1.0;
}

core::Result<ViewPose> view_pose(const compute::CvCamera& camera,
                                 const compute::Board& board,
                                 const Observation& view) {
  return compute::guarded("view_pose", [&]() -> core::Result<ViewPose> {
    VKC_ASSIGN(const compute::ViewUse use,
               compute::usable_for_pose(board, view, camera.size));
    if (use != compute::ViewUse::Usable) {
      return core::Status::invalid_argument(
          std::string("view_pose: the view fixes no pose: ") +
          compute::describe(use));
    }
    std::vector<cv::Point3d> object;
    std::vector<cv::Point2d> image;
    object.reserve(view.points.size());
    image.reserve(view.points.size());
    for (const DetectedPoint& point : view.points) {
      object.push_back(
          board.object_points()[static_cast<std::size_t>(point.id)]);
      image.emplace_back(point.pixel.x, point.pixel.y);
    }
    // Works around OpenCV 5.0.0's solvePnPGeneric (geometry/src/solvepnp.cpp),
    // whose IPPE undistorts with undistortPoints' default 5 iterations: IPPE
    // gets converged rays and no lens. The fits below use the pixels.
    VKC_ASSIGN(const std::vector<cv::Point2d> rays,
               compute::undistort_points(camera, image));
    // IPPE's two solutions for `points`, the board's corners in one frame, as
    // poses of the board. Each fit is infinite for a solution not finite, a
    // corner behind the camera, or a projection that overflowed.
    const auto solve = [&](const std::vector<cv::Point3d>& points,
                           bool quarter_turn) -> core::Result<Solutions> {
      std::vector<cv::Mat> rvecs;
      std::vector<cv::Mat> tvecs;
      cv::solvePnPGeneric(points, rays, cv::Matx33d::eye(), cv::noArray(),
                          rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
      // IPPE gives two solutions of a usable view. solvePnPGeneric swallows
      // whatever IPPE throws (running out of memory included) and returns the
      // solutions it had kept by then: none, or half of them, which its own
      // assertion then reports. Either is OpenCV failing, not the view.
      if (rvecs.size() != 2 || tvecs.size() != 2) {
        return core::Status::backend_error(
            core::Status::Backend::Other, cv::Error::StsError,
            "view_pose: IPPE failed, solvePnPGeneric giving " +
                std::to_string(rvecs.size()) + " solutions of 2");
      }
      Solutions solutions;
      for (std::size_t i = 0; i < solutions.size(); ++i) {
        const compute::CvPose pose{read_vec3(rvecs[i]), read_vec3(tvecs[i])};
        PoseCandidate& candidate = solutions[i];
        candidate.board_to_camera = compute::matrix_from_cv_pose(pose);
        if (quarter_turn) {
          candidate.board_to_camera = unturned(candidate.board_to_camera);
        }
        candidate.rms_px = kInfinity;
        if (!finite(pose.rvec) || !finite(pose.tvec)) continue;
        double sum = 0.0;
        for (const double e2 : detail::squared_errors(
                 camera, board, view, candidate.board_to_camera)) {
          sum += e2;
        }
        candidate.rms_px = std::sqrt(sum / static_cast<double>(points.size()));
      }
      return solutions;
    };
    // Works around OpenCV 5.0.0's IPPE::PoseSolver::rot2vec
    // (geometry/src/ippe.cpp), which loses the angle near a half turn: IPPE
    // also solves the board turned a quarter turn about its normal. A pose a
    // half turn in both frames points the printed face along the camera's z,
    // away from it. Each solution, paired across the frames by translation
    // (exact in both; one not finite keeps IPPE's order), keeps the frame
    // that fits it better.
    std::vector<cv::Point3d> turned;
    turned.reserve(object.size());
    for (const cv::Point3d& x : object) turned.push_back(quarter_turned(x));
    VKC_ASSIGN(const Solutions plain, solve(object, false));
    VKC_ASSIGN(Solutions other, solve(turned, true));
    if (apart(plain[0], other[1]) + apart(plain[1], other[0]) <
        apart(plain[0], other[0]) + apart(plain[1], other[1])) {
      std::swap(other[0], other[1]);
    }
    std::vector<PoseCandidate> candidates;
    for (std::size_t i = 0; i < plain.size(); ++i) {
      const PoseCandidate& better =
          other[i].rms_px < plain[i].rms_px ? other[i] : plain[i];
      if (std::isfinite(better.rms_px)) candidates.push_back(better);
    }
    // TODO: a board exactly face-on and centred on the optical axis can still
    // get no pose: IPPE's PoseSolver::computeRotations (OpenCV 5.0.0,
    // geometry/src/ippe.cpp) takes the square root of a 1 - |column|^2 that
    // rounds below zero, in both frames -- 422 of 3600 in-plane angles 0.1
    // degrees apart, 2 m in front of view_pose_test's pinhole; none once
    // tilted 1e-4 rad.
    if (candidates.empty()) {
      return core::Status::numerical(
          "view_pose: IPPE gave no finite pose with every corner in front of "
          "the camera");
    }
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const PoseCandidate& a, const PoseCandidate& b) {
                       return a.rms_px < b.rms_px;
                     });
    ViewPose pose;
    pose.best = candidates[0];
    if (candidates.size() > 1) pose.alternative = candidates[1];
    return pose;
  });
}

double tilt_deg(const compute::Board& board,
                const camera::Mat4d& board_to_camera) noexcept {
  const cv::Point3d middle = board.centre();
  const camera::Vec3d centre(board_to_camera *
                             glm::dvec4(middle.x, middle.y, middle.z, 1.0));
  // The board's z points into it, away from the printed face.
  return compute::angle_deg(-camera::Vec3d(board_to_camera[2]), -centre);
}

}  // namespace volumetric_kit::calib::solve
