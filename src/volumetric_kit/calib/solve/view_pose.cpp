// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/solve/view_pose.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/geometry.hpp>

#include "reprojection.hpp"
#include "volumetric_kit/calib/compute/cv_pose.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/usable_for_pose.hpp"

namespace volumetric_kit::calib::solve {
namespace {

namespace camera = recon::camera;

// OpenCV's 3-vector, whatever its shape and depth, in double; false unless
// it holds three values.
bool read_vec3(const cv::Mat& m, cv::Vec3d& v) {
  if (m.total() * static_cast<std::size_t>(m.channels()) != 3) return false;
  cv::Mat d;
  m.convertTo(d, CV_64F);
  const cv::Mat flat = d.reshape(1, 3);
  v = cv::Vec3d(flat.at<double>(0), flat.at<double>(1), flat.at<double>(2));
  return true;
}

bool finite(const cv::Vec3d& v) noexcept {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
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
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::solvePnPGeneric(object, image, camera.camera_matrix, camera.distortion,
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

    std::vector<PoseCandidate> candidates;
    for (std::size_t i = 0; i < rvecs.size(); ++i) {
      compute::CvPose pose;
      if (!read_vec3(rvecs[i], pose.rvec) || !read_vec3(tvecs[i], pose.tvec) ||
          !finite(pose.rvec) || !finite(pose.tvec)) {
        continue;
      }
      PoseCandidate candidate;
      candidate.board_to_camera = compute::matrix_from_cv_pose(pose);
      double sum = 0.0;
      for (const double e2 : detail::squared_errors(
               camera, board, view, candidate.board_to_camera)) {
        sum += e2;
      }
      // A corner behind the camera, or a projection that overflowed.
      candidate.rms_px = std::sqrt(sum / static_cast<double>(object.size()));
      if (!std::isfinite(candidate.rms_px)) continue;
      candidates.push_back(candidate);
    }
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
  const Target& target = board.target();
  const double s = target.square_size_m;
  // The chessboard's middle: OpenCV's board has one square more than corners
  // on each axis.
  const camera::Vec3d centre(board_to_camera *
                             glm::dvec4(0.5 * s * (target.cols + 1),
                                        0.5 * s * (target.rows + 1), 0.0, 1.0));
  // The board's z points into it, away from the printed face.
  const camera::Vec3d face = -camera::Vec3d(board_to_camera[2]);
  const camera::Vec3d to_camera = -centre;
  return glm::degrees(std::atan2(glm::length(glm::cross(face, to_camera)),
                                 glm::dot(face, to_camera)));
}

}  // namespace volumetric_kit::calib::solve
