// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/live_checks.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "checks.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/compute/usable_for_pose.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;

constexpr int kCellsPerSide = 3;

// A detection's corners by id, sorted; InvalidArgument for an id repeated or
// a pixel not finite.
core::Result<std::vector<std::pair<int, camera::Vec2d>>> corners_by_id(
    const Detection& detection, const char* which, std::size_t camera) {
  std::vector<std::pair<int, camera::Vec2d>> corners;
  corners.reserve(detection.observation.points.size());
  for (const DetectedPoint& point : detection.observation.points) {
    if (!std::isfinite(point.pixel.x) || !std::isfinite(point.pixel.y)) {
      return core::Status::invalid_argument(
          std::string("corner_shift: camera ") + std::to_string(camera) +
          "'s " + which + " corner " + std::to_string(point.id) +
          " is not finite");
    }
    corners.emplace_back(point.id, point.pixel);
  }
  std::sort(corners.begin(), corners.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  for (std::size_t i = 1; i < corners.size(); ++i) {
    if (corners[i].first == corners[i - 1].first) {
      return core::Status::invalid_argument(
          std::string("corner_shift: camera ") + std::to_string(camera) +
          "'s " + which + " detection repeats corner " +
          std::to_string(corners[i].first));
    }
  }
  return corners;
}

}  // namespace

core::Result<CheckedView> check_view(const compute::CvCamera& camera,
                                     const compute::Board& board,
                                     const Detection& detection,
                                     const ViewCheckOptions& options) {
  return compute::guarded("check_view", [&]() -> core::Result<CheckedView> {
    VKC_TRY(detail::check_view_options(options, "check_view: options."));
    const camera::ImageSize& size = detection.image_size;
    if (static_cast<long long>(size.width) != camera.size.width ||
        static_cast<long long>(size.height) != camera.size.height) {
      return core::Status::invalid_argument(
          "check_view: the detection is of a " + std::to_string(size.width) +
          "x" + std::to_string(size.height) + " image, the camera's is " +
          std::to_string(camera.size.width) + "x" +
          std::to_string(camera.size.height));
    }
    CheckedView view;
    ViewCheck& check = view.check;
    check.corners = static_cast<int>(detection.observation.points.size());
    check.markers = static_cast<int>(detection.markers.size());

    VKC_ASSIGN(
        const compute::ViewUse use,
        compute::usable_for_pose(board, detection.observation, camera.size));
    if (use == compute::ViewUse::Collinear) {
      check.collinear = true;
      check.verdict = ViewVerdict::Collinear;
      return view;
    }
    if (use != compute::ViewUse::Usable) return view;
    core::Result<solve::ViewPose> pose =
        solve::view_pose(camera, board, detection.observation);
    if (!pose.ok()) {
      // No pose in front of the camera: a view of no use, not a failure.
      if (pose.status().domain() == core::Status::Code::Numerical) return view;
      return pose.status();
    }

    const camera::Mat4d& board_to_camera = pose->best.board_to_camera;
    const camera::Vec3d centre = compute::centre_in(board, board_to_camera);
    check.distance_m = glm::length(centre);
    check.tilt_deg = solve::tilt_deg(board, board_to_camera);
    check.pnp_rms_px = pose->best.rms_px;
    view.pose = *pose;
    if (const std::optional<camera::Vec2d> pixel =
            compute::project(camera, centre)) {
      check.cell = detail::grid_cell(pixel->x, pixel->y, camera.size,
                                     kCellsPerSide, kCellsPerSide);
    }
    view.frontal = *check.tilt_deg < options.frontal_deg;

    const auto all_corners = static_cast<int>(board.object_points().size());
    if (*check.tilt_deg > options.oblique_deg) {
      check.verdict = ViewVerdict::Oblique;
    } else if (*check.distance_m > options.far_m) {
      check.verdict = ViewVerdict::Far;
    } else if (check.corners < all_corners) {
      check.verdict = ViewVerdict::Partial;
    } else {
      check.verdict = ViewVerdict::Good;
    }
    return view;
  });
}

core::Result<CornerShift> corner_shift(const std::vector<Detection>& before,
                                       const std::vector<Detection>& after) {
  return compute::guarded("corner_shift", [&]() -> core::Result<CornerShift> {
    if (before.size() != after.size()) {
      return core::Status::invalid_argument(
          "corner_shift: " + std::to_string(before.size()) +
          " detections before, " + std::to_string(after.size()) + " after");
    }
    CornerShift shift;
    shift.cameras.resize(before.size());
    std::vector<double> all;
    for (std::size_t c = 0; c < before.size(); ++c) {
      VKC_ASSIGN(const auto from, corners_by_id(before[c], "earlier", c));
      VKC_ASSIGN(const auto to, corners_by_id(after[c], "later", c));
      if (from.empty() || to.empty()) continue;
      if (before[c].image_size.width != after[c].image_size.width ||
          before[c].image_size.height != after[c].image_size.height) {
        return core::Status::invalid_argument(
            "corner_shift: camera " + std::to_string(c) +
            "'s detections are of images of different sizes");
      }
      std::vector<double> moved;
      for (std::size_t i = 0, j = 0; i < from.size() && j < to.size();) {
        if (from[i].first < to[j].first) {
          ++i;
        } else if (to[j].first < from[i].first) {
          ++j;
        } else {
          moved.push_back(glm::length(to[j].second - from[i].second));
          ++i;
          ++j;
        }
      }
      if (moved.empty()) continue;
      all.insert(all.end(), moved.begin(), moved.end());
      shift.cameras[c] = compute::median(std::move(moved));
    }
    if (!all.empty()) {
      shift.max_px = *std::max_element(all.begin(), all.end());
      shift.median_px = compute::median(std::move(all));
    }
    return shift;
  });
}

bool live_vs_full_flagged(const CornerShift& shift,
                          const LiveVsFullOptions& options) noexcept {
  return (shift.median_px && *shift.median_px > options.median_px) ||
         (shift.max_px && *shift.max_px > options.max_px);
}

}  // namespace volumetric_kit::calib::quality
