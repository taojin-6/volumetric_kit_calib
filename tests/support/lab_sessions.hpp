// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file lab_sessions.hpp
/// @brief A lab pair's two sessions as a person shows the boards, at a
///        quarter of 4K through the lab's fisheye (`quarter_fisheye`): a lens
///        session's checkerboard pictures and a pose session's ChArUco pairs.
///        Its users link calib_detect and vc_synthetic_rig.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <optional>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "detect_helpers.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/detect/board.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"

namespace volumetric_kit::calib::test {

/// The pictures' size: `quarter_fisheye()`'s.
inline constexpr sensor::camera::ImageSize kLabQuarter{960, 540};

/// @brief The lab's checkerboard with 40 mm squares: at a quarter of 4K,
///        half a 4K picture's pixels a square, as near as the 20 mm board at
///        half the distance.
inline Target lab_lens_board() {
  Target board = legacy_checkerboard();
  board.square_size_m = 0.04;
  return board;
}

/// @brief The rig's ChArUco board at twice its size: at a quarter of 4K, its
///        markers as many pixels as the lab's board's at 4K from twice as far.
inline Target lab_pose_board() {
  Target board = legacy_femto_board();
  board.square_size_m *= 2.0;
  board.marker_size_m *= 2.0;
  return board;
}

/// @brief The board at a pose, through a truth, when the live view finds it
///        in the picture's half and the capture gives a view to fit: the
///        picture, and the capture's corners.
struct PlacedBoard {
  std::vector<std::uint8_t> picture;  ///< 8-bit, rows packed.
  Observation full;                   ///< The capture's corners.
};

/// @brief The board at @p pose through @p truth, rendered, when the live view
///        finds at least @p min_live corners in the half and the capture
///        @p min_full in the picture.
inline std::optional<PlacedBoard> place_board(const FisheyeTruth& truth,
                                              const compute::Board& board,
                                              const sensor::camera::Mat4d& pose,
                                              std::size_t min_live,
                                              std::size_t min_full) {
  RenderOptions options;
  options.ray = truth.pixel_to_ray();
  const cv::Mat picture = render_board(truth.frame(), pose, board, options);
  const cv::Mat half = halved(picture);
  const detect::LumaView half_view = luma(half);
  const core::Result<Detection> live =
      detect::detect_board(luma(picture), board, {&half_view});
  const core::Result<Detection> full =
      detect::detect_board(luma(picture), board);
  if (!live.ok() || !full.ok() || live->observation.size() < min_live ||
      full->observation.size() < min_full) {
    return std::nullopt;
  }
  return PlacedBoard{{picture.data, picture.data + picture.total()},
                     full->observation};
}

/// @brief A lens session's pictures, as a person makes them by the runbook:
///        the board tilted 20-35 degrees near the axis and further out, its
///        tilt's axis turning; then pushed toward each image corner and side,
///        turned along the line to it, and held where the live view finds it
///        and its corners reach 0.9 of the way there, as the Lenses panel
///        asks, four times each.
/// @param truth  The lens.
/// @param board  The checkerboard (@ref lab_lens_board).
inline std::vector<std::vector<std::uint8_t>> lens_session_pictures(
    const FisheyeTruth& truth, const compute::Board& board) {
  const auto min_corners =
      static_cast<std::size_t>(solve::LensOptions{}.min_corners);
  std::vector<std::vector<std::uint8_t>> pictures;
  const auto keep = [&](const sensor::camera::Mat4d& pose) {
    std::optional<PlacedBoard> placed =
        place_board(truth, board, pose, 1, min_corners);
    if (placed) pictures.push_back(std::move(placed->picture));
  };
  for (int i = 0; i < 18; ++i) {
    keep(facing(board, 8.0 + 6.0 * (i % 3), 20.0 * i, 0.65 + 0.05 * (i % 2),
                40.0 * i, 25.0 + 5.0 * (i % 3)));
  }
  for (int i = 0; i < 18; ++i) {
    keep(facing(board, 38.0 + 4.0 * (i % 2), 20.0 * i + 10.0, 0.55,
                40.0 * i + 15.0, 20.0 + 5.0 * (i % 3)));
  }
  // Toward each image corner and side, in solve::LensReach's order.
  const struct {
    double azimuth_deg;
    double roll_deg;
  } goals[] = {{-150.6, -17.0}, {-29.4, 17.0}, {150.6, 17.0},
               {29.4, -17.0},   {180.0, -5.0}, {0.0, 5.0}};
  constexpr int kEach = 4;
  for (std::size_t g = 0; g < std::size(goals); ++g) {
    int kept = 0;
    for (double off = 58.0; off <= 74.0 && kept < kEach; off += 2.0) {
      for (const double tilt : {0.0, 10.0, 20.0}) {
        for (const double turn : {0.0, -10.0, 10.0}) {
          if (kept == kEach) break;
          std::optional<PlacedBoard> placed =
              place_board(truth, board,
                          facing(board, off, goals[g].azimuth_deg, 0.5,
                                 goals[g].roll_deg + turn, tilt),
                          1, min_corners);
          if (!placed) continue;
          double reach = 0.0;
          for (const DetectedPoint& point : placed->full.points) {
            reach = std::max(reach,
                             solve::pixel_reach(kLabQuarter, point.pixel)[g]);
          }
          if (reach < solve::kLensReach) continue;
          pictures.push_back(std::move(placed->picture));
          ++kept;
        }
      }
    }
  }
  return pictures;
}

/// @brief One placement of the board as both cameras of a pair picture it.
struct ShownPair {
  std::vector<std::uint8_t> left;   ///< The left camera's picture.
  std::vector<std::uint8_t> right;  ///< The right camera's.
};

/// @brief A pose session's pictures, as a person covers the pair's shared
///        view by the runbook: the board's centre toward the middle of each
///        of the 9 parts of the left camera's view the rows are judged in --
///        the centre first, then 35 degrees across and 22 up and down -- at
///        2.0 and 2.6 m, tilted 15 degrees one way and the other about its x
///        axis; kept where the live view finds the board in both cameras'
///        halves, and the capture in both full pictures.
/// @param truth  Both cameras' lens.
/// @param pair   The pair (`lab_pair`).
/// @param board  The ChArUco board (@ref lab_pose_board).
inline std::vector<ShownPair> pair_session_pictures(
    const FisheyeTruth& truth, const std::vector<SyntheticCamera>& pair,
    const compute::Board& board) {
  constexpr double kDeg = 3.14159265358979323846 / 180.0;
  const sensor::camera::Mat4d left_to_world =
      sensor::camera::rigid_inverse(pair[0].world_to_camera);
  std::vector<ShownPair> shown;
  for (const double down : {0.0, -22.0, 22.0}) {
    for (const double across : {0.0, -35.0, 35.0}) {
      for (const double distance : {2.0, 2.6}) {
        for (const double tilt : {-15.0, 15.0}) {
          const double tx = std::tan(across * kDeg);
          const double ty = std::tan(down * kDeg);
          const sensor::camera::Mat4d board_to_world =
              left_to_world *
              facing(board, std::atan(std::hypot(tx, ty)) / kDeg,
                     std::atan2(ty, tx) / kDeg, distance, 0.0, tilt);
          ShownPair placed;
          bool seen = true;
          for (std::size_t c = 0; c < 2 && seen; ++c) {
            std::optional<PlacedBoard> one = place_board(
                truth, board, pair[c].world_to_camera * board_to_world, 6, 6);
            seen = one.has_value();
            if (seen) (c == 0 ? placed.left : placed.right) = one->picture;
          }
          if (seen) shown.push_back(std::move(placed));
        }
      }
    }
  }
  return shown;
}

}  // namespace volumetric_kit::calib::test
