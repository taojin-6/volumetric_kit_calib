// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/detect/charuco.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

#include "detail/luma_check.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::detect {
namespace {

namespace camera = recon::camera;

// The hybrid's refinement (charuco.hpp, detect_charuco).
constexpr double kMinHalfWindow = 2.0;
constexpr double kMaxHalfWindow = 10.0;
constexpr int kRefineIterations = 100;
constexpr double kRefineEpsilon = 0.01;
constexpr double kMaxRefineShift = 2.0;

core::Status check_image(const LumaView& image, Mode mode) {
  if (mode != Mode::Full && mode != Mode::Hybrid) {
    return core::Status::invalid_argument("detect_charuco: mode is not a Mode");
  }
  if (mode == Mode::Hybrid && (image.width < 2 || image.height < 2)) {
    return core::Status::invalid_argument(
        "detect_charuco: a Hybrid image is at least 2 x 2 pixels");
  }
  return detail::check_luma(image, "detect_charuco");
}

// The image's even part halved: INTER_AREA at exactly half the size averages
// each 2 x 2 block, which it does not when a side is odd.
cv::Mat halved(const cv::Mat& full) {
  const cv::Mat even = full(cv::Rect(0, 0, full.cols & ~1, full.rows & ~1));
  cv::Mat half;
  cv::resize(even, half, cv::Size(even.cols / 2, even.rows / 2), 0.0, 0.0,
             cv::INTER_AREA);
  return half;
}

// A pixel of the image halved, in the image: the centre of the 2 x 2 pixels
// INTER_AREA averaged into it.
cv::Point2f full_resolution(const cv::Point2f& half) {
  return {2.0f * half.x + 0.5f, 2.0f * half.y + 0.5f};
}

// Each corner refined in `image` from its seed, dropped (with its id) when it
// moves more than kMaxRefineShift. Every marker found bounds the window, the
// board's or not: each is an edge that would pull the corner.
void refine_corners(const cv::Mat& image,
                    const std::vector<std::vector<cv::Point2f>>& markers,
                    std::vector<cv::Point2f>& corners, std::vector<int>& ids) {
  const cv::TermCriteria criteria(
      cv::TermCriteria::COUNT | cv::TermCriteria::EPS, kRefineIterations,
      kRefineEpsilon);
  std::size_t kept = 0;
  std::vector<cv::Point2f> point(1);
  for (std::size_t i = 0; i < corners.size(); ++i) {
    const cv::Point2f seed = corners[i];
    double nearest = std::numeric_limits<double>::infinity();
    for (const std::vector<cv::Point2f>& marker : markers) {
      for (const cv::Point2f& p : marker) {
        nearest = std::min(nearest, cv::norm(p - seed));
      }
    }
    const int half = static_cast<int>(
        std::clamp(std::floor(0.5 * nearest), kMinHalfWindow, kMaxHalfWindow));
    point[0] = seed;
    cv::cornerSubPix(image, point, cv::Size(half, half), cv::Size(-1, -1),
                     criteria);
    if (cv::norm(point[0] - seed) > kMaxRefineShift) continue;
    corners[kept] = point[0];
    ids[kept] = ids[i];
    ++kept;
  }
  corners.resize(kept);
  ids.resize(kept);
}

camera::Vec2d pixel(const cv::Point2f& p) { return {p.x, p.y}; }

}  // namespace

core::Result<Detection> detect_charuco(const LumaView& image,
                                       const compute::Board& board, Mode mode) {
  return compute::guarded("detect_charuco", [&]() -> core::Result<Detection> {
    VKC_TRY(check_image(image, mode));
    // OpenCV reads the image and never writes it.
    const cv::Mat full(static_cast<int>(image.height),
                       static_cast<int>(image.width), CV_8UC1,
                       const_cast<std::uint8_t*>(image.data), image.stride);
    const cv::aruco::CharucoDetector detector(board.charuco());
    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> markers;
    std::vector<int> marker_ids;
    if (mode == Mode::Full) {
      detector.detectBoard(full, corners, ids, markers, marker_ids);
    } else {
      detector.detectBoard(halved(full), corners, ids, markers, marker_ids);
      for (std::vector<cv::Point2f>& marker : markers) {
        for (cv::Point2f& p : marker) p = full_resolution(p);
      }
      for (cv::Point2f& p : corners) p = full_resolution(p);
      refine_corners(full, markers, corners, ids);
    }

    Detection detection;
    detection.image_size = {image.width, image.height};
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
    const std::vector<int>& board_ids = board.charuco().getIds();
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
  });
}

}  // namespace volumetric_kit::calib::detect
