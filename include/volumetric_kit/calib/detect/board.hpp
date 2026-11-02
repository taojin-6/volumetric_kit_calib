// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file board.hpp
/// @brief The board in an 8-bit luma image (luma.hpp), either kind: one
///        detector for a ChArUco board and a plain checkerboard.

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/detect/export.hpp"
#include "volumetric_kit/calib/detect/luma.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::detect {

/// @brief How @ref detect_board searches.
///
/// @code
/// DetectOptions options;
/// options.half = &half;  // the picture halved on the GPU
/// @endcode
struct DetectOptions {
  /// The picture halved, as sensor makes it on the GPU: `width / 2` x
  /// `height / 2`, each pixel the rounded mean of 2 x 2 of the picture's (an
  /// odd last row or column left out), so its pixel u is 2u + 0.5 in the
  /// picture. The board is searched there and each corner refined in the
  /// picture: the live view's faster search. Null: searched in the picture.
  const LumaView* half = nullptr;
};

/// @brief The board's corners in one 8-bit picture, either kind.
///
/// - **ChArUco:** OpenCV's `CharucoDetector`, corners by ChArUco id and the
///   board's markers, as `detect_charuco` finds them in Full mode, or with
///   @ref DetectOptions::half as in Hybrid mode, from that half.
/// - **Checkerboard:** `findChessboardCornersSB` asked for 5 x 4 inner
///   corners with `CALIB_CB_LARGER`, so any larger grid counts, a board cut
///   by the picture's edge included. In the picture it runs with
///   `CALIB_CB_NORMALIZE_IMAGE` and `CALIB_CB_EXHAUSTIVE`; in the half with
///   `CALIB_CB_NORMALIZE_IMAGE`. Its grid depends on OpenCV's random numbers:
///   each search starts them from one seed and puts them back after, so a
///   picture always gives the same detection, on any number of OpenCV
///   threads. A corner out of step with its neighbours, or whose four
///   squares, seen at their centres in the picture, are not dark and light by
///   turns, is dropped: so grown, SB at times adds corners past the board
///   (where it meets its white margin, or off it) or skips squares; a grid
///   with half its corners dropped is dropped whole. Each corner is then
///   refined in the picture with `cornerSubPix`, its half-window half the
///   distance to its nearest neighbour, rounded down and held to 2-10 px; a
///   corner that moves more than 2 px is dropped. The grid the corners kept
///   span is numbered row-major from its own first corner into the board's
///   ids, turned a quarter when it fits the board only that way round; so an
///   id names a corner in this view only (`Observation`). No markers.
///
/// No board is not a failure: the detection is then empty. Calls may run on
/// several threads at once, sharing @p board.
/// @param image    The picture, at least 1 x 1 (2 x 2 with a half).
/// @param board    The board sought.
/// @param options  The half, if any.
/// @return The detection: the picture's size; the board's corners by id (the
///         index into `compute::Board::object_points`), in ascending order,
///         each of weight 1, with the camera and frame left for the caller to
///         fill; and a ChArUco board's markers, in ascending order of id. Or
///         `InvalidArgument` for a picture or half `detect_charuco` would
///         refuse (no pixels, a null `data`, a `stride` below `width`, a side
///         longer than OpenCV's `int` holds), a half of another size than the
///         picture halved, or a checkerboard grid larger than @p board either
///         way round (naming both sizes); or what OpenCV threw
///         (`compute::guarded`).
///
/// @code
/// VKC_ASSIGN(Detection found, detect_board(luma, board));
/// found.observation.camera_id = "bm-1";
/// found.observation.frame_id = capture_index;
/// @endcode
VC_DETECT_API core::Result<Detection> detect_board(
    const LumaView& image, const compute::Board& board,
    const DetectOptions& options = {});

}  // namespace volumetric_kit::calib::detect
