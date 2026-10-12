// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/compute/board.hpp"

#include <limits>
#include <numeric>
#include <optional>
#include <utility>

#include <opencv2/objdetect/aruco_dictionary.hpp>

#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::compute {
namespace {

// OpenCV's predefined dictionary for each MarkerDictionary, mapped by name:
// the two enums' values differ. Empty for None and for a value that is not a
// MarkerDictionary.
std::optional<cv::aruco::PredefinedDictionaryType> opencv_dictionary(
    MarkerDictionary dictionary) {
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

}  // namespace

Board::Board(const Target& target, const cv::aruco::CharucoBoard& charuco,
             std::vector<cv::Point3d> object_points)
    : target_(target),
      charuco_(charuco),
      object_points_(std::move(object_points)) {}

core::Result<Board> Board::create(const Target& target) {
  VKC_TRY(check_target(target));
  if (target.kind != TargetKind::ChArUco) {
    return core::Status::unsupported("target.kind must be ChArUco for a Board");
  }
  // OpenCV's board keeps its lengths as float; a double beyond float's range
  // does not convert.
  if (target.square_size_m > std::numeric_limits<float>::max()) {
    return core::Status::invalid_argument(
        "target.square_size_m must fit a float");
  }
  const float square = static_cast<float>(target.square_size_m);
  const float marker = static_cast<float>(target.marker_size_m);
  if (!(marker > 0.0f && marker < square)) {
    return core::Status::invalid_argument(
        "target.marker_size_m must stay positive and below square_size_m as a "
        "float");
  }
  const std::optional<cv::aruco::PredefinedDictionaryType> dictionary =
      opencv_dictionary(target.dictionary);
  // check_target refuses a ChArUco board without a dictionary.
  if (!dictionary) {
    return core::Status::invalid_argument(
        "target.dictionary must name the markers' dictionary");
  }

  return guarded("Board::create", [&]() -> core::Result<Board> {
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
