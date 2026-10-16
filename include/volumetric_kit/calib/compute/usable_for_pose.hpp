// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file usable_for_pose.hpp
/// @brief Whether a view of a board fixes the board's pose: the one check the
///        detector applies to what it finds and the solver repeats on what it
///        is given.

#include <opencv2/core.hpp>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/export.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::compute {

/// @brief What a view of a board is good for: a pose, or why not.
enum class ViewUse {
  Usable,         ///< The view fixes the board's pose.
  TooFewCorners,  ///< Fewer than 4 corners.
  UnknownCorner,  ///< A corner id the board does not have, or one repeated.
  /// A corner not finite, or less than 2 px inside the image.
  OutsideImage,
  /// The corners, or all but one of them, lie on one line (OpenCV's check),
  /// so no 4 of them fix the board's plane.
  Collinear,
};

/// @brief Whether @p view fixes the pose of @p board: at least 4 corners,
///        each a corner of the board once, each finite and at least 2 px
///        inside the image, and 4 of them with no 3 on one line.
///
/// The checks run in the order @ref ViewUse lists them, and the first that
/// fails is the answer. The image's edge is at -0.5 and `size - 0.5` (OpenCV
/// puts pixel centres at integers), so a usable corner lies in
/// `[1.5, width - 2.5] x [1.5, height - 2.5]`. A planar pose needs a
/// homography, and so 4 corners with no 3 on one line; there are 4 such unless
/// every corner, or every corner but one, lies on one line.
/// @param board  The board seen.
/// @param view   The corners seen, by id (`DetectedPoint::id`).
/// @param image  The size of the image the corners are in.
/// @return The view's use; `InvalidArgument` for an image of no area, or
///         what OpenCV threw (@ref guarded).
///
/// @code
/// VKC_ASSIGN(const ViewUse use, usable_for_pose(board, view, cam.size));
/// if (use != ViewUse::Usable) return;  // nothing to pose
/// @endcode
VC_COMPUTE_API core::Result<ViewUse> usable_for_pose(const Board& board,
                                                     const Observation& view,
                                                     const cv::Size& image);

/// @brief Why a view is, or is not, usable, as a phrase for a message.
/// @param use  @ref usable_for_pose's answer.
/// @return A phrase such as "fewer than 4 corners"; "an unknown use" for a
///         value @ref ViewUse does not name.
///
/// @code
/// if (use != ViewUse::Usable) {
///   log(std::string("view refused: ") + describe(use));
/// }
/// @endcode
VC_COMPUTE_API const char* describe(ViewUse use) noexcept;

/// @brief The name calib's results files give @p use.
/// @param use  @ref usable_for_pose's answer.
/// @return `"usable"`, `"too_few_corners"`, `"unknown_corner"`,
///         `"outside_image"` or `"collinear"`; `"unknown"` for a value
///         @ref ViewUse does not name.
///
/// @code
/// log(to_string(use));
/// @endcode
VC_COMPUTE_API const char* to_string(ViewUse use) noexcept;

}  // namespace volumetric_kit::calib::compute
