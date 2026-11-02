// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/compute/usable_for_pose.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::compute {
namespace {

constexpr std::size_t kMinCorners = 4;
constexpr double kInsidePx = 2.0;  // from the image's edge, at -0.5 and size

// Whether the corners `ids` names lie on one line on the board: each on the
// line through the first two. The corners are on a grid of `square`-sided
// cells, so three not on one line make a triangle of at least half a cell:
// twice its area, the cross product below, is at least square^2, where three
// on a line give 0 but for rounding. The ids are the board's, each once.
bool on_one_line(const std::vector<cv::Point3d>& corners,
                 const std::vector<int>& ids, double square) {
  const cv::Point3d& a = corners[static_cast<std::size_t>(ids[0])];
  const cv::Point3d& b = corners[static_cast<std::size_t>(ids[1])];
  const double tolerance = 0.5 * square * square;
  for (std::size_t i = 2; i < ids.size(); ++i) {
    const cv::Point3d& p = corners[static_cast<std::size_t>(ids[i])];
    const double twice_area =
        (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    if (std::abs(twice_area) > tolerance) return false;
  }
  return true;
}

}  // namespace

core::Result<ViewUse> usable_for_pose(const Board& board,
                                      const Observation& view,
                                      const cv::Size& image) {
  return guarded("usable_for_pose", [&]() -> core::Result<ViewUse> {
    if (image.width <= 0 || image.height <= 0) {
      return core::Status::invalid_argument(
          "usable_for_pose: the image has no area");
    }
    if (view.points.size() < kMinCorners) return ViewUse::TooFewCorners;

    const std::size_t corners = board.object_points().size();
    std::vector<bool> seen(corners, false);
    std::vector<int> ids;
    ids.reserve(view.points.size());
    for (const DetectedPoint& point : view.points) {
      if (point.id < 0 || static_cast<std::size_t>(point.id) >= corners ||
          seen[static_cast<std::size_t>(point.id)]) {
        return ViewUse::UnknownCorner;
      }
      seen[static_cast<std::size_t>(point.id)] = true;
      ids.push_back(point.id);
    }

    const double low = kInsidePx - 0.5;
    const double right = image.width - 0.5 - kInsidePx;
    const double bottom = image.height - 0.5 - kInsidePx;
    for (const DetectedPoint& point : view.points) {
      // Written so a NaN fails it.
      if (!(point.pixel.x >= low && point.pixel.x <= right &&
            point.pixel.y >= low && point.pixel.y <= bottom)) {
        return ViewUse::OutsideImage;
      }
    }

    // Every corner but one on a line, which every corner on it is too. The
    // ids index the board's corners: they are checked above.
    std::vector<int> others(ids.size() - 1);
    for (std::size_t skip = 0; skip < ids.size(); ++skip) {
      for (std::size_t i = 0, j = 0; i < ids.size(); ++i) {
        if (i != skip) others[j++] = ids[i];
      }
      if (on_one_line(board.object_points(), others,
                      board.target().square_size_m)) {
        return ViewUse::Collinear;
      }
    }
    return ViewUse::Usable;
  });
}

const char* describe(ViewUse use) noexcept {
  switch (use) {
    case ViewUse::Usable:
      return "usable";
    case ViewUse::TooFewCorners:
      return "fewer than 4 corners";
    case ViewUse::UnknownCorner:
      return "a corner the board does not have, or one repeated";
    case ViewUse::OutsideImage:
      return "a corner not finite or not 2 px inside the image";
    case ViewUse::Collinear:
      return "its corners, or all but one, lie on one line";
  }
  return "an unknown use";
}

const char* to_string(ViewUse use) noexcept {
  switch (use) {
    case ViewUse::Usable:
      return "usable";
    case ViewUse::TooFewCorners:
      return "too_few_corners";
    case ViewUse::UnknownCorner:
      return "unknown_corner";
    case ViewUse::OutsideImage:
      return "outside_image";
    case ViewUse::Collinear:
      return "collinear";
  }
  return "unknown";
}

}  // namespace volumetric_kit::calib::compute
