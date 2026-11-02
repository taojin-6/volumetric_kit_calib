// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file charuco.hpp
/// @brief The ChArUco board in an 8-bit luma image (luma.hpp): its corners by
///        id and its markers.
///
/// The detector is OpenCV's `CharucoDetector` with its default parameters and
/// no camera model. Internal: callers find either kind of board with
/// `detect_board` (board.hpp), which also searches a picture's half, as the
/// GPU makes it, for the live view. Nothing halves a picture on the CPU.

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/detect/export.hpp"
#include "volumetric_kit/calib/detect/luma.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::detect {

/// @brief Find @p board in @p image at its resolution: OpenCV's
///        `detectBoard`, whose corners OpenCV refines with `cornerSubPix` in
///        the image.
///
/// No board is not a failure: the detection is then empty. Calls may run on
/// several threads at once, sharing @p board.
/// @param image  The luma image, at least 1 x 1.
/// @param board  The board sought.
/// @return The detection: the image's size; the board's corners by ChArUco
///         id (the index into `compute::Board::object_points`), in ascending
///         order, each of weight 1, with the camera and frame left for the
///         caller to fill; and the board's markers, in ascending order of id
///         (another board's markers of the same dictionary are left out).
///         Or `InvalidArgument` for an image with no pixels, a null `data`, a
///         `stride` below `width`, a side longer than OpenCV's `int` holds,
///         or a board that is not ChArUco; or what OpenCV threw
///         (`compute::guarded`).
///
/// @code
/// VKC_ASSIGN(Detection detection, detect_charuco(luma, board));
/// detection.observation.camera_id = serial;
/// detection.observation.frame_id = capture_index;
/// @endcode
// TODO: fold into detect_board, its only caller's path, and remove this
// header (the source tier's PR left the tracked file in place).
VC_DETECT_API core::Result<Detection> detect_charuco(
    const LumaView& image, const compute::Board& board);

}  // namespace volumetric_kit::calib::detect
