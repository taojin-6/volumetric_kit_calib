// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/detect/charuco.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

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

// usable_for_pose's bounds.
constexpr std::size_t kMinCorners = 4;
constexpr double kEdgeMargin = 2.0;

core::Status check_image(const LumaView& image, Mode mode) {
  if (mode != Mode::Full && mode != Mode::Hybrid) {
    return core::Status::invalid_argument("detect_charuco: mode is not a Mode");
  }
  const std::uint32_t min_side = mode == Mode::Hybrid ? 2 : 1;
  if (image.width < min_side || image.height < min_side) {
    return core::Status::invalid_argument(
        mode == Mode::Hybrid
            ? "detect_charuco: a Hybrid image is at least 2 x 2 pixels"
            : "detect_charuco: the image has no pixels");
  }
  constexpr std::uint32_t kMaxSide = std::numeric_limits<int>::max();
  if (image.width > kMaxSide || image.height > kMaxSide) {
    return core::Status::invalid_argument(
        "detect_charuco: the image is larger than OpenCV's int sizes hold");
  }
  if (image.data == nullptr) {
    return core::Status::invalid_argument("detect_charuco: image.data is null");
  }
  if (image.stride < image.width) {
    return core::Status::invalid_argument(
        "detect_charuco: image.stride is below image.width");
  }
  return {};
}

// A pixel of the image halved, in the image: the centre of the 2 x 2 pixels
// INTER_AREA averaged into it.
cv::Point2f full_resolution(const cv::Point2f& half) {
  return {2.0f * half.x + 0.5f, 2.0f * half.y + 0.5f};
}

// Each corner refined in `image` from its seed, dropped (with its id) when it
// moves more than kMaxRefineShift.
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
      cv::Mat half;
      cv::resize(full, half, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
      detector.detectBoard(half, corners, ids, markers, marker_ids);
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
    detection.markers.reserve(markers.size());
    for (std::size_t i = 0; i < markers.size(); ++i) {
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

core::Result<Usability> usable_for_pose(const Detection& detection,
                                        const compute::Board& board) {
  return compute::guarded("usable_for_pose", [&]() -> core::Result<Usability> {
    const std::size_t count = board.object_points().size();
    std::vector<bool> seen(count, false);
    std::vector<int> ids;
    ids.reserve(detection.observation.points.size());
    for (const DetectedPoint& point : detection.observation.points) {
      if (point.id < 0 || static_cast<std::size_t>(point.id) >= count) {
        return core::Status::invalid_argument("usable_for_pose: corner id " +
                                              std::to_string(point.id) +
                                              " is not the board's");
      }
      if (seen[static_cast<std::size_t>(point.id)]) {
        return core::Status::invalid_argument("usable_for_pose: corner id " +
                                              std::to_string(point.id) +
                                              " appears twice");
      }
      seen[static_cast<std::size_t>(point.id)] = true;
      ids.push_back(point.id);
    }
    if (ids.size() < kMinCorners) return Usability::TooFewCorners;
    if (board.charuco().checkCharucoCornersCollinear(ids)) {
      return Usability::Collinear;
    }
    const double right =
        static_cast<double>(detection.image_size.width) - 1.0 - kEdgeMargin;
    const double bottom =
        static_cast<double>(detection.image_size.height) - 1.0 - kEdgeMargin;
    for (const DetectedPoint& point : detection.observation.points) {
      // A NaN fails every comparison, so it is near the edge too.
      if (!(point.pixel.x >= kEdgeMargin && point.pixel.x <= right &&
            point.pixel.y >= kEdgeMargin && point.pixel.y <= bottom)) {
        return Usability::NearEdge;
      }
    }
    return Usability::Usable;
  });
}

}  // namespace volumetric_kit::calib::detect
