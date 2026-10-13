// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file charuco.hpp
/// @brief The ChArUco board in an 8-bit luma image: its corners by id and its
///        markers, and whether a view of them fixes the board's pose.
///
/// Pixels are OpenCV's and recon's: pixel centres sit at integer coordinates,
/// so a W-pixel-wide image spans `[-0.5, W - 0.5]`. The detector is OpenCV's
/// `CharucoDetector` with its default parameters and no camera model.

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/detect/export.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::detect {

/// @brief An 8-bit luma image the caller owns: @ref height rows of
///        @ref width bytes, @ref stride bytes apart.
///
/// recon's `PictureView` maps onto it field for field.
///
/// @code
/// const LumaView luma{view.width, view.height, view.stride, view.luma};
/// @endcode
struct LumaView {
  std::uint32_t width = 0;             ///< Pixels per row.
  std::uint32_t height = 0;            ///< Rows.
  std::size_t stride = 0;              ///< Bytes between rows; >= @ref width.
  const std::uint8_t* data = nullptr;  ///< The first row's first pixel.
};

/// @brief How @ref detect_charuco finds the board.
enum class Mode {
  /// Markers and corners at the image's resolution. What a capture stores
  /// and the solver takes.
  Full,
  /// Markers and first corners in the image halved (`INTER_AREA`), each
  /// corner then refined in the image itself. Under a third of Full's time
  /// at 4K, for live display and checks only.
  Hybrid,
};

/// @brief One marker found.
///
/// @code
/// for (const DetectedMarker& marker : detection.markers) {
///   draw_outline(marker.corners);  // a caller's own drawing
/// }
/// @endcode
struct DetectedMarker {
  int id = -1;  ///< Its id in the board's dictionary.
  /// Its corners, clockwise from its top-left as printed: OpenCV's order.
  std::array<recon::camera::Vec2d, 4> corners{};
};

/// @brief What @ref detect_charuco found in one image.
///
/// @code
/// VKC_ASSIGN(Detection detection,
///            detect_charuco(luma, board, Mode::Full));
/// detection.observation.camera_id = serial;
/// detection.observation.frame_id = capture_index;
/// @endcode
struct Detection {
  /// The image searched, which @ref usable_for_pose checks corners against.
  recon::camera::ImageSize image_size;
  /// The board's corners, by ChArUco id (the index into
  /// `compute::Board::object_points`), in ascending order, each of weight 1.
  /// The camera and frame are the caller's to fill.
  Observation observation;
  /// The markers found, in ascending order of id.
  std::vector<DetectedMarker> markers;
};

/// @brief Find @p board in @p image.
///
/// - **Full:** OpenCV's `detectBoard`, whose corners OpenCV refines with
///   `cornerSubPix` in the image.
/// - **Hybrid:** `detectBoard` in the image halved. A pixel `u` there is
///   `2u + 0.5` in the image, the centre of the 2 x 2 pixels it averages;
///   the markers are reported there. Each corner is refined from there with
///   `cornerSubPix` (at most 100 iterations, to 0.01 px), its half-window
///   half its distance to the nearest marker corner, rounded down and held
///   to 2-10 px; a corner that moves more than 2 px is dropped.
///
/// No board is not a failure: the detection is then empty. Calls may run on
/// several threads at once, sharing @p board.
/// @param image  The luma image, at least 1 x 1 (Full) or 2 x 2 (Hybrid).
/// @param board  The board sought.
/// @param mode   Full or Hybrid.
/// @return The detection; `InvalidArgument` for an image with no pixels
///         (or too few for @p mode), a null `data`, a `stride` below
///         `width`, a side longer than OpenCV's `int` holds, or a @p mode
///         that is not a @ref Mode; or what OpenCV threw
///         (`compute::guarded`).
VC_DETECT_API core::Result<Detection> detect_charuco(
    const LumaView& image, const compute::Board& board, Mode mode);

/// @brief What a view is worth for the board's pose (@ref usable_for_pose).
enum class Usability {
  Usable,         ///< Enough corners, off one line, inside the image.
  TooFewCorners,  ///< Fewer than 4 corners.
  Collinear,      ///< The corners lie on one line of the board.
  /// A corner is not finite, or not at least 2 px inside the image's outer
  /// pixel centres.
  NearEdge,
};

/// @brief Whether @p detection fixes @p board's pose: 4 corners or more, not
///        on one line (OpenCV's `checkCharucoCornersCollinear`), and each
///        finite with `2 <= x <= width - 3` and `2 <= y <= height - 3`.
///
/// The checks run in that order, and the first that fails is the answer.
/// @param detection  A detection of @p board.
/// @param board      The board.
/// @return The answer; `InvalidArgument` for a corner id that is not
///         @p board's or that appears twice; or what OpenCV threw
///         (`compute::guarded`).
///
/// @code
/// VKC_ASSIGN(const Usability use, usable_for_pose(detection, board));
/// if (use == Usability::Usable) views.push_back(detection.observation);
/// @endcode
VC_DETECT_API core::Result<Usability> usable_for_pose(
    const Detection& detection, const compute::Board& board);

}  // namespace volumetric_kit::calib::detect
