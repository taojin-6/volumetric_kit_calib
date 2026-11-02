// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "detail/search.hpp"

namespace volumetric_kit::calib::detect::detail {
namespace {

// The refinement (search.hpp, refine_corners).
constexpr double kMinHalfWindow = 2.0;
constexpr double kMaxHalfWindow = 10.0;
constexpr int kRefineIterations = 100;
constexpr double kRefineEpsilon = 0.01;
constexpr double kMaxRefineShift = 2.0;

}  // namespace

std::vector<double> nearest_of(
    const std::vector<cv::Point2f>& corners,
    const std::vector<std::vector<cv::Point2f>>& points) {
  std::vector<double> nearest(corners.size(),
                              std::numeric_limits<double>::infinity());
  for (std::size_t i = 0; i < corners.size(); ++i) {
    for (const std::vector<cv::Point2f>& group : points) {
      for (const cv::Point2f& p : group) {
        nearest[i] = std::min(nearest[i], cv::norm(p - corners[i]));
      }
    }
  }
  return nearest;
}

std::vector<double> nearest_other(const std::vector<cv::Point2f>& corners) {
  std::vector<double> nearest(corners.size(),
                              std::numeric_limits<double>::infinity());
  for (std::size_t i = 0; i < corners.size(); ++i) {
    for (std::size_t j = 0; j < corners.size(); ++j) {
      if (j != i) {
        nearest[i] = std::min(nearest[i], cv::norm(corners[j] - corners[i]));
      }
    }
  }
  return nearest;
}

void refine_corners(const cv::Mat& image, const std::vector<double>& nearest,
                    std::vector<cv::Point2f>& corners, std::vector<int>& ids) {
  const cv::TermCriteria criteria(
      cv::TermCriteria::COUNT | cv::TermCriteria::EPS, kRefineIterations,
      kRefineEpsilon);
  std::size_t kept = 0;
  std::vector<cv::Point2f> point(1);
  for (std::size_t i = 0; i < corners.size(); ++i) {
    const cv::Point2f seed = corners[i];
    const int half = static_cast<int>(std::clamp(
        std::floor(0.5 * nearest[i]), kMinHalfWindow, kMaxHalfWindow));
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

}  // namespace volumetric_kit::calib::detect::detail
