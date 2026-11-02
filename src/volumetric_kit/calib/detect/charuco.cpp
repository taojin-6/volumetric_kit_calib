// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/detect/charuco.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

#include "detail/luma_check.hpp"
#include "detail/search.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::detect {
namespace {

namespace camera = sensor::camera;

camera::Vec2d pixel(const cv::Point2f& p) { return {p.x, p.y}; }

}  // namespace

namespace detail {

Detection find_charuco(const cv::Mat& full, const cv::Mat* half,
                       const compute::Board& board) {
  const cv::aruco::CharucoBoard& charuco = *board.charuco();
  const cv::aruco::CharucoDetector detector(charuco);
  std::vector<cv::Point2f> corners;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> markers;
  std::vector<int> marker_ids;
  if (half == nullptr) {
    detector.detectBoard(full, corners, ids, markers, marker_ids);
  } else {
    detector.detectBoard(*half, corners, ids, markers, marker_ids);
    for (std::vector<cv::Point2f>& marker : markers) {
      for (cv::Point2f& p : marker) p = full_resolution(p);
    }
    for (cv::Point2f& p : corners) p = full_resolution(p);
    // Every marker found bounds a corner's window, the board's or not: each
    // is an edge that would pull the corner.
    refine_corners(full, nearest_of(corners, markers), corners, ids);
  }

  Detection detection;
  std::vector<DetectedPoint>& points = detection.observation.points;
  points.reserve(corners.size());
  for (std::size_t i = 0; i < corners.size(); ++i) {
    points.push_back({ids[i], pixel(corners[i]), 1.0});
  }
  std::sort(points.begin(), points.end(),
            [](const DetectedPoint& a, const DetectedPoint& b) {
              return a.id < b.id;
            });
  // The detector reports every marker of the board's dictionary it finds,
  // another board's too.
  const std::vector<int>& board_ids = charuco.getIds();
  detection.markers.reserve(markers.size());
  for (std::size_t i = 0; i < markers.size(); ++i) {
    if (std::find(board_ids.begin(), board_ids.end(), marker_ids[i]) ==
        board_ids.end()) {
      continue;
    }
    DetectedMarker marker;
    marker.id = marker_ids[i];
    for (std::size_t k = 0; k < marker.corners.size(); ++k) {
      marker.corners[k] = pixel(markers[i][k]);
    }
    detection.markers.push_back(marker);
  }
  std::sort(detection.markers.begin(), detection.markers.end(),
            [](const DetectedMarker& a, const DetectedMarker& b) {
              return a.id < b.id;
            });
  return detection;
}

}  // namespace detail

core::Result<Detection> detect_charuco(const LumaView& image,
                                       const compute::Board& board) {
  return compute::guarded("detect_charuco", [&]() -> core::Result<Detection> {
    VKC_TRY(detail::check_luma(image, "detect_charuco"));
    if (board.charuco() == nullptr) {
      return core::Status::invalid_argument(
          "detect_charuco: the board is not ChArUco (detect_board finds a "
          "checkerboard)");
    }
    // OpenCV reads the image and never writes it.
    const cv::Mat full(static_cast<int>(image.height),
                       static_cast<int>(image.width), CV_8UC1,
                       const_cast<std::uint8_t*>(image.data), image.stride);
    Detection detection = detail::find_charuco(full, nullptr, board);
    detection.image_size = {image.width, image.height};
    return detection;
  });
}

}  // namespace volumetric_kit::calib::detect
