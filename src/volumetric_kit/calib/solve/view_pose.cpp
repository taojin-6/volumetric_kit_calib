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
constexpr double kHalfSqrt2 = 0.70710678118654752440;

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
camera::Mat4d quarter_unturned(camera::Mat4d board_to_camera) noexcept {
  const glm::dvec4 x = board_to_camera[0];
  board_to_camera[0] = board_to_camera[1];
  board_to_camera[1] = -x;
  return board_to_camera;
}

// The board turned an eighth turn about its normal.
cv::Point3d eighth_turned(const cv::Point3d& p) noexcept {
  return {kHalfSqrt2 * (p.x - p.y), kHalfSqrt2 * (p.x + p.y), p.z};
}

// A pose of the eighth-turned board as one of the board: T' E, E the turn.
camera::Mat4d eighth_unturned(camera::Mat4d board_to_camera) noexcept {
  const glm::dvec4 x = board_to_camera[0];
  const glm::dvec4 y = board_to_camera[1];
  board_to_camera[0] = kHalfSqrt2 * (x + y);
  board_to_camera[1] = kHalfSqrt2 * (y - x);
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
    // IPPE's two solutions for the board's corners turned about its normal
    // by `turn` (none if null), as poses of the board (`unturn`). Each fit is
    // infinite for a solution not finite, a corner behind the camera, or a
    // projection that overflowed.
    using Turn = cv::Point3d (*)(const cv::Point3d&) noexcept;
    using Unturn = camera::Mat4d (*)(camera::Mat4d) noexcept;
    const auto solve = [&](Turn turn,
                           Unturn unturn) -> core::Result<Solutions> {
      std::vector<cv::Point3d> points;
      points.reserve(object.size());
      for (const cv::Point3d& x : object) {
        points.push_back(turn != nullptr ? turn(x) : x);
      }
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
        if (unturn != nullptr) {
          candidate.board_to_camera = unturn(candidate.board_to_camera);
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
    VKC_ASSIGN(const Solutions plain, solve(nullptr, nullptr));
    VKC_ASSIGN(Solutions other, solve(quarter_turned, quarter_unturned));
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
    // Works around OpenCV 5.0.0's IPPE::PoseSolver::computeRotations
    // (geometry/src/ippe.cpp), which takes the square root of 1 - |c|^2 for
    // each of the rotation's first two columns c: zero for a board whose x or
    // y axis is square to the line of sight to it, as when tilted exactly
    // about that axis, it rounds below zero on some builds (GCC's on Linux)
    // and leaves no pose. The quarter turn swaps the axes, so it fails too;
    // turned an eighth turn, neither axis is a column.
    if (candidates.empty()) {
      VKC_ASSIGN(const Solutions eighth, solve(eighth_turned, eighth_unturned));
      for (const PoseCandidate& candidate : eighth) {
        if (std::isfinite(candidate.rms_px)) candidates.push_back(candidate);
      }
    }
    // TODO: a board exactly face-on and centred on the optical axis can still
    // get no pose: both its axes are square to the line of sight, in every
    // frame, and the square roots above round below zero -- 422 of 3600
    // in-plane angles 0.1 degrees apart, 2 m in front of view_pose_test's
    // pinhole, with Homebrew's OpenCV on macOS; none once tilted 1e-4 rad.
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
  return compute::angle_deg(compute::face_normal(board_to_camera),
                            -compute::centre_in(board, board_to_camera));
}

}  // namespace volumetric_kit::calib::solve
