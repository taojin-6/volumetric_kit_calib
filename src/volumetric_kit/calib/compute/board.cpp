// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/compute/board.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <utility>

#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::compute {

std::optional<cv::aruco::PredefinedDictionaryType> opencv_dictionary(
    MarkerDictionary dictionary) noexcept {
  switch (dictionary) {
    case MarkerDictionary::None:
      return std::nullopt;
    case MarkerDictionary::ArUco4x4_50:
      return cv::aruco::DICT_4X4_50;
    case MarkerDictionary::ArUco4x4_100:
      return cv::aruco::DICT_4X4_100;
    case MarkerDictionary::ArUco4x4_250:
      return cv::aruco::DICT_4X4_250;
    case MarkerDictionary::ArUco4x4_1000:
      return cv::aruco::DICT_4X4_1000;
    case MarkerDictionary::ArUco5x5_50:
      return cv::aruco::DICT_5X5_50;
    case MarkerDictionary::ArUco5x5_100:
      return cv::aruco::DICT_5X5_100;
    case MarkerDictionary::ArUco5x5_250:
      return cv::aruco::DICT_5X5_250;
    case MarkerDictionary::ArUco5x5_1000:
      return cv::aruco::DICT_5X5_1000;
    case MarkerDictionary::ArUco6x6_50:
      return cv::aruco::DICT_6X6_50;
    case MarkerDictionary::ArUco6x6_100:
      return cv::aruco::DICT_6X6_100;
    case MarkerDictionary::ArUco6x6_250:
      return cv::aruco::DICT_6X6_250;
    case MarkerDictionary::ArUco6x6_1000:
      return cv::aruco::DICT_6X6_1000;
    case MarkerDictionary::ArUco7x7_50:
      return cv::aruco::DICT_7X7_50;
    case MarkerDictionary::ArUco7x7_100:
      return cv::aruco::DICT_7X7_100;
    case MarkerDictionary::ArUco7x7_250:
      return cv::aruco::DICT_7X7_250;
    case MarkerDictionary::ArUco7x7_1000:
      return cv::aruco::DICT_7X7_1000;
    case MarkerDictionary::ArUcoOriginal:
      return cv::aruco::DICT_ARUCO_ORIGINAL;
    case MarkerDictionary::AprilTag16h5:
      return cv::aruco::DICT_APRILTAG_16h5;
    case MarkerDictionary::AprilTag25h9:
      return cv::aruco::DICT_APRILTAG_25h9;
    case MarkerDictionary::AprilTag36h10:
      return cv::aruco::DICT_APRILTAG_36h10;
    case MarkerDictionary::AprilTag36h11:
      return cv::aruco::DICT_APRILTAG_36h11;
    case MarkerDictionary::ArUcoMip36h12:
      return cv::aruco::DICT_ARUCO_MIP_36h12;
  }
  return std::nullopt;
}

Board::Board(const Target& target, const cv::aruco::CharucoBoard& charuco,
             std::vector<cv::Point3d> object_points)
    : target_(target),
      charuco_(charuco),
      object_points_(std::move(object_points)) {}

cv::Point3d Board::centre() const noexcept {
  // OpenCV's board has one square more than corners on each axis.
  const double s = target_.square_size_m;
  return {0.5 * s * (target_.cols + 1), 0.5 * s * (target_.rows + 1), 0.0};
}

sensor::camera::Vec3d centre_in(
    const Board& board, const sensor::camera::Mat4d& board_to_frame) noexcept {
  const cv::Point3d c = board.centre();
  return sensor::camera::Vec3d(board_to_frame * glm::dvec4(c.x, c.y, c.z, 1.0));
}

sensor::camera::Vec3d face_normal(
    const sensor::camera::Mat4d& board_to_frame) noexcept {
  return -sensor::camera::Vec3d(board_to_frame[2]);
}

core::Result<Board> Board::create(const Target& target) {
  return guarded("Board::create", [&]() -> core::Result<Board> {
    VKC_TRY(check_target(target));
    if (target.kind != TargetKind::ChArUco) {
      return core::Status::unsupported(
          "target.kind must be ChArUco for a Board");
    }
    // OpenCV's board keeps its lengths and corners as float, its far corner at
    // squares x square_size_m: each must be a normal float (a double beyond
    // float's range does not even convert).
    constexpr float kMin = std::numeric_limits<float>::min();
    if (!(target.square_size_m >= kMin &&
          target.square_size_m <= std::numeric_limits<float>::max())) {
      return core::Status::invalid_argument(
          "target.square_size_m must be a normal float");
    }
    const float square = static_cast<float>(target.square_size_m);
    const float max_corners =
        static_cast<float>(std::max(target.rows, target.cols));
    if (!std::isfinite(square * (max_corners + 1.0f))) {
      return core::Status::invalid_argument(
          "target.square_size_m must keep the board's extent a float");
    }
    const float marker = static_cast<float>(target.marker_size_m);
    if (!(marker >= kMin && marker < square)) {
      return core::Status::invalid_argument(
          "target.marker_size_m must stay a normal float below square_size_m");
    }
    const std::optional<cv::aruco::PredefinedDictionaryType> dictionary =
        opencv_dictionary(target.dictionary);
    // check_target refuses a ChArUco board without a dictionary.
    if (!dictionary) {
      return core::Status::invalid_argument(
          "target.dictionary must name the markers' dictionary");
    }

    const cv::Size squares(target.cols + 1, target.rows + 1);
    // One marker in every other square; check_target holds them inside the
    // dictionary.
    std::vector<int> ids(static_cast<std::size_t>(squares.area() / 2));
    std::iota(ids.begin(), ids.end(), target.first_marker_id);
    cv::aruco::CharucoBoard charuco(
        squares, square, marker,
        cv::aruco::getPredefinedDictionary(*dictionary), ids);
    charuco.setLegacyPattern(target.legacy_pattern);
    const std::vector<cv::Point3f> corners = charuco.getChessboardCorners();
    return Board(target, charuco,
                 std::vector<cv::Point3d>(corners.begin(), corners.end()));
  });
}

}  // namespace volumetric_kit::calib::compute
