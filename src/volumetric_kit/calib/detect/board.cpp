// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/detect/board.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include "detail/luma_check.hpp"
#include "detail/search.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::detect {
namespace {

// The grid of inner corners the checkerboard's search asks for, across x
// down: with CALIB_CB_LARGER, the smallest it finds, either way round (and
// the smallest board check_target takes). A smaller one, 4 x 3, comes back
// as a different grid from frame to frame of a still board cut by the
// picture's edge.
const cv::Size kSmallestGrid(5, 4);

// OpenCV reads the picture and never writes it.
cv::Mat mat_of(const LumaView& image) {
  return cv::Mat(static_cast<int>(image.height), static_cast<int>(image.width),
                 CV_8UC1, const_cast<std::uint8_t*>(image.data), image.stride);
}

std::string size_text(int across, int down) {
  return std::to_string(across) + " x " + std::to_string(down);
}

core::Status check_half(const LumaView& image, const LumaView& half) {
  VKC_TRY(detail::check_luma(half, "detect_board: options.half"));
  if (half.width != image.width / 2 || half.height != image.height / 2) {
    return core::Status::invalid_argument(
        "detect_board: options.half is " + std::to_string(half.width) + " x " +
        std::to_string(half.height) + ", not the picture halved (" +
        std::to_string(image.width / 2) + " x " +
        std::to_string(image.height / 2) + ")");
  }
  return {};
}

// The mean of the 5 x 5 pixels around `at` in `image`; empty when they are
// not all in it.
std::optional<double> patch_mean(const cv::Mat& image, const cv::Point2f& at) {
  constexpr float kHalf = 2.0f;
  if (!(at.x >= kHalf && at.y >= kHalf && at.x <= image.cols - 1 - kHalf &&
        at.y <= image.rows - 1 - kHalf)) {
    return std::nullopt;
  }
  cv::Mat patch;
  cv::getRectSubPix(image, cv::Size(5, 5), at, patch, CV_32F);
  return cv::mean(patch)[0];
}

// Whether each corner of the grid found (`across` x `down`, row by row, in
// `image`'s pixels) is a checkerboard's corner:
// - **in step:** along its row and its column, the step from the corner
//   before differs from the step to the corner after by under a quarter of
//   their mean (a lens bends a grid far less from one square to the next);
// - **dark and light by turns:** its four squares, seen at their centres
//   (half a step along the grid each way) and all in the image, have each
//   diagonal pair within half the contrast between the pairs, and that
//   contrast at least half the grid's median.
// Grown past the size asked for (CALIB_CB_LARGER), SB at times adds corners
// past the board -- a row of the T-junctions where the board meets its white
// margin, or a corner off the board altogether -- or returns a grid that is
// no grid, in the fisheye's corners. Such corners fail; and when half or more
// of the grid's fail, it is no board's, and every corner is dropped.
std::vector<bool> board_corners(const cv::Mat& image,
                                const std::vector<cv::Point2f>& corners,
                                int across, int down) {
  const auto at = [&](int r, int c) {
    return corners[static_cast<std::size_t>(r) * across + c];
  };
  // Whether the steps before and after `here` agree; true at an end.
  const auto in_step = [](const cv::Point2f& before, const cv::Point2f& here,
                          const cv::Point2f& after) {
    const cv::Point2f in = here - before;
    const cv::Point2f out = after - here;
    return cv::norm(out - in) < 0.125 * (cv::norm(in) + cv::norm(out));
  };
  // Half the step to the next corner along an axis, from the neighbours.
  const auto half_step = [](const cv::Point2f& before, const cv::Point2f& here,
                            const cv::Point2f& after, bool has_before,
                            bool has_after) {
    if (has_before && has_after) return 0.25f * (after - before);
    return has_after ? 0.5f * (after - here) : 0.5f * (here - before);
  };
  // Each corner: whether it passes all but the contrast's floor, and its
  // contrast.
  std::vector<bool> kept(corners.size(), false);
  std::vector<double> contrast(corners.size(), 0.0);
  std::vector<double> contrasts;
  for (int r = 0; r < down; ++r) {
    for (int c = 0; c < across; ++c) {
      const std::size_t k = static_cast<std::size_t>(r) * across + c;
      const cv::Point2f p = at(r, c);
      const bool left = c > 0, right = c + 1 < across;
      const bool up = r > 0, below = r + 1 < down;
      if ((left && right && !in_step(at(r, c - 1), p, at(r, c + 1))) ||
          (up && below && !in_step(at(r - 1, c), p, at(r + 1, c)))) {
        continue;
      }
      const cv::Point2f a = half_step(left ? at(r, c - 1) : p, p,
                                      right ? at(r, c + 1) : p, left, right);
      const cv::Point2f b = half_step(up ? at(r - 1, c) : p, p,
                                      below ? at(r + 1, c) : p, up, below);
      const std::optional<double> m1 = patch_mean(image, p + a + b);
      const std::optional<double> m2 = patch_mean(image, p - a - b);
      const std::optional<double> m3 = patch_mean(image, p + a - b);
      const std::optional<double> m4 = patch_mean(image, p - a + b);
      if (!m1 || !m2 || !m3 || !m4) continue;
      contrast[k] = std::abs((*m1 + *m2) - (*m3 + *m4)) / 2.0;
      contrasts.push_back(contrast[k]);
      kept[k] = std::abs(*m1 - *m2) < 0.5 * contrast[k] &&
                std::abs(*m3 - *m4) < 0.5 * contrast[k];
    }
  }
  if (contrasts.empty()) return kept;
  const auto middle =
      contrasts.begin() + static_cast<std::ptrdiff_t>(contrasts.size() / 2);
  std::nth_element(contrasts.begin(), middle, contrasts.end());
  std::size_t passed = 0;
  for (std::size_t k = 0; k < corners.size(); ++k) {
    kept[k] = kept[k] && contrast[k] >= 0.5 * *middle;
    if (kept[k]) ++passed;
  }
  if (2 * passed <= corners.size()) kept.assign(corners.size(), false);
  return kept;
}

// The checkerboard in `full`, searched in `half` when given (search.hpp).
core::Result<Detection> find_checkerboard(const cv::Mat& full,
                                          const cv::Mat* half,
                                          const compute::Board& board) {
  int flags = cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_LARGER;
  if (half == nullptr) flags |= cv::CALIB_CB_EXHAUSTIVE;
  std::vector<cv::Point2f> found;
  cv::Mat meta;
  Detection detection;
  if (!cv::findChessboardCornersSB(half != nullptr ? *half : full,
                                   kSmallestGrid, found, flags, meta)) {
    return detection;
  }
  // The grid found: `across` corners in each of its `down` rows, row by row.
  const int across = meta.empty() ? kSmallestGrid.width : meta.cols;
  const int down = meta.empty() ? kSmallestGrid.height : meta.rows;
  if (found.size() != static_cast<std::size_t>(across) * down) {
    return core::Status::backend_error(
        core::Status::Backend::Other, cv::Error::StsError,
        "detect_board: findChessboardCornersSB found a " +
            size_text(across, down) + " grid of " +
            std::to_string(found.size()) + " corners");
  }
  if (half != nullptr) {
    for (cv::Point2f& p : found) p = detail::full_resolution(p);
  }
  // The board's corners among them, and the grid they span.
  const std::vector<bool> kept = board_corners(full, found, across, down);
  int r0 = down, r1 = -1, c0 = across, c1 = -1;
  for (int r = 0; r < down; ++r) {
    for (int c = 0; c < across; ++c) {
      if (!kept[static_cast<std::size_t>(r) * across + c]) continue;
      r0 = std::min(r0, r);
      r1 = std::max(r1, r);
      c0 = std::min(c0, c);
      c1 = std::max(c1, c);
    }
  }
  if (r1 < 0) return detection;
  const int grid_across = c1 - c0 + 1;
  const int grid_down = r1 - r0 + 1;
  const Target& target = board.target();
  const bool as_entered =
      grid_across <= target.cols && grid_down <= target.rows;
  const bool turned = grid_across <= target.rows && grid_down <= target.cols;
  if (!as_entered && !turned) {
    return core::Status::invalid_argument(
        "detect_board: found a checkerboard of " +
        size_text(grid_across, grid_down) +
        " inner corners, larger than the board's " +
        size_text(target.cols, target.rows) +
        " either way round: is the board entered right?");
  }
  // Row-major from the grid's first corner; turned, its rows are the
  // board's columns, read from the last, which keeps the grid's handedness.
  std::vector<cv::Point2f> corners;
  std::vector<int> ids;
  for (int r = r0; r <= r1; ++r) {
    for (int c = c0; c <= c1; ++c) {
      const std::size_t k = static_cast<std::size_t>(r) * across + c;
      if (!kept[k]) continue;
      corners.push_back(found[k]);
      ids.push_back(as_entered
                        ? (r - r0) * target.cols + (c - c0)
                        : (c - c0) * target.cols + (grid_down - 1 - (r - r0)));
    }
  }
  if (half != nullptr) {
    detail::refine_corners(full, detail::nearest_other(corners), corners, ids);
  }

  std::vector<DetectedPoint>& points = detection.observation.points;
  points.reserve(corners.size());
  for (std::size_t i = 0; i < corners.size(); ++i) {
    points.push_back({ids[i], {corners[i].x, corners[i].y}, 1.0});
  }
  std::sort(points.begin(), points.end(),
            [](const DetectedPoint& a, const DetectedPoint& b) {
              return a.id < b.id;
            });
  return detection;
}

}  // namespace

core::Result<Detection> detect_board(const LumaView& image,
                                     const compute::Board& board,
                                     const DetectOptions& options) {
  return compute::guarded("detect_board", [&]() -> core::Result<Detection> {
    VKC_TRY(detail::check_luma(image, "detect_board"));
    if (options.half != nullptr) VKC_TRY(check_half(image, *options.half));
    const cv::Mat full = mat_of(image);
    cv::Mat half;
    if (options.half != nullptr) half = mat_of(*options.half);
    const cv::Mat* searched = options.half != nullptr ? &half : nullptr;
    Detection detection;
    if (board.charuco() != nullptr) {
      detection = detail::find_charuco(full, searched, board);
    } else {
      VKC_ASSIGN(detection, find_checkerboard(full, searched, board));
    }
    detection.image_size = {image.width, image.height};
    return detection;
  });
}

}  // namespace volumetric_kit::calib::detect
