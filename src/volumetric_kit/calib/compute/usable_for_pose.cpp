// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/compute/usable_for_pose.hpp"

#include <cstddef>
#include <vector>

#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::compute {
namespace {

constexpr std::size_t kMinCorners = 4;
constexpr double kInsidePx = 2.0;  // from the image's edge, at -0.5 and size

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

    // OpenCV indexes the board's corners by these ids unchecked: they are
    // checked above.
    if (board.charuco().checkCharucoCornersCollinear(ids)) {
      return ViewUse::Collinear;
    }
    return ViewUse::Usable;
  });
}

}  // namespace volumetric_kit::calib::compute
