// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file charuco.hpp
/// @brief The ChArUco board in an 8-bit luma image (luma.hpp): its corners by
///        id and its markers.
///
/// The detector is OpenCV's `CharucoDetector` with its default parameters and
/// no camera model.

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/detect/export.hpp"
#include "volumetric_kit/calib/detect/luma.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::detect {

/// @brief How @ref detect_charuco finds the board.
enum class Mode {
  /// Markers and corners at the image's resolution. What a capture stores
  /// and the solver takes.
  Full,
  /// Markers and first corners in the image halved (`INTER_AREA`), each
  /// corner then refined in the image itself. For live display and checks;
  /// captures use Full.
  Hybrid,
};

/// @brief Find @p board in @p image.
///
/// - **Full:** OpenCV's `detectBoard`, whose corners OpenCV refines with
///   `cornerSubPix` in the image.
/// - **Hybrid:** `detectBoard` in the image halved; an odd last column or
///   row is left out, so each pixel there averages 2 x 2 pixels of the image.
///   A pixel `u` there is `2u + 0.5` in the image, the centre of those
///   2 x 2 pixels; the markers are reported there. Each corner is refined
///   from there with `cornerSubPix` (at most 100 iterations, to 0.01 px), its
///   half-window half its distance to the nearest marker corner found (the
///   board's or not), rounded down and held to 2-10 px; a corner that moves
///   more than 2 px is dropped.
///
/// No board is not a failure: the detection is then empty. Calls may run on
/// several threads at once, sharing @p board.
/// @param image  The luma image, at least 1 x 1 (Full) or 2 x 2 (Hybrid).
/// @param board  The board sought.
/// @param mode   Full or Hybrid.
/// @return The detection: the image's size; the board's corners by ChArUco
///         id (the index into `compute::Board::object_points`), in ascending
///         order, each of weight 1, with the camera and frame left for the
///         caller to fill; and the board's markers, in ascending order of id
///         (another board's markers of the same dictionary are left out).
///         Or `InvalidArgument` for an image with no pixels (or too few for
///         @p mode), a null `data`, a `stride` below `width`, a side longer
///         than OpenCV's `int` holds, or a @p mode that is not a @ref Mode;
///         or what OpenCV threw (`compute::guarded`).
///
/// @code
/// VKC_ASSIGN(Detection detection,
///            detect_charuco(luma, board, Mode::Full));
/// detection.observation.camera_id = serial;
/// detection.observation.frame_id = capture_index;
/// @endcode
VC_DETECT_API core::Result<Detection> detect_charuco(
    const LumaView& image, const compute::Board& board, Mode mode);

}  // namespace volumetric_kit::calib::detect
