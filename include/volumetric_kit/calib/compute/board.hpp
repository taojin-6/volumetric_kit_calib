// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file board.hpp
/// @brief A ChArUco @ref volumetric_kit::calib::Target as OpenCV's board, and
///        its corners in the board's frame.
///
/// The board frame is OpenCV's: the origin at the top-left outer corner of
/// the chessboard, x right and y down across the printed face, and z into the
/// board, so the printed face looks along -z. The top-left square is black,
/// except in @ref volumetric_kit::calib::Target::legacy_pattern's layout with
/// an even number of square rows, where it is white and holds the first
/// marker. The corners come from OpenCV's `getChessboardCorners`; calib keeps
/// no corner formula.

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_board.hpp>

#include "volumetric_kit/calib/compute/export.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::compute {

/// @brief A ChArUco board: its @ref Target, OpenCV's `CharucoBoard`, and each
///        corner's position by its id.
///
/// A value: copies share OpenCV's board, which this class only reads.
/// OpenCV's board copies share state too: never call a setter on a copy of
/// @ref charuco, which would change every copy's board under its
/// @ref object_points and @ref target.
///
/// @code
/// VKC_ASSIGN(const Board board, Board::create(legacy_femto_board()));
/// cv::aruco::CharucoDetector detector(board.charuco());
/// const cv::Point3d corner = board.object_points()[id];  // id from detection
/// @endcode
class VC_COMPUTE_API Board {
 public:
  /// @brief Build the board @p target describes.
  ///
  /// OpenCV's board counts squares, one more than @p target's corners on each
  /// axis, and takes its markers from @ref Target::first_marker_id on, in
  /// @ref Target::legacy_pattern's layout.
  /// @param target  A ChArUco board.
  /// @return The board; `InvalidArgument` for a target @ref check_target
  ///         refuses, or whose lengths OpenCV's `float`s cannot hold (a
  ///         square or marker below the normal range, a board beyond the
  ///         range, or a marker that rounds to the square); `Unsupported`
  ///         for another @ref TargetKind; or what OpenCV threw
  ///         (@ref guarded).
  static core::Result<Board> create(const Target& target);

  /// @return The board's description.
  const Target& target() const noexcept { return target_; }
  /// @return OpenCV's board, for its detector and drawing.
  const cv::aruco::CharucoBoard& charuco() const noexcept { return charuco_; }
  /// @return Each corner's position in the board frame (metres), indexed by
  ///         its ChArUco id: `target().rows * target().cols` of them, on
  ///         z = 0.
  const std::vector<cv::Point3d>& object_points() const noexcept {
    return object_points_;
  }

 private:
  Board(const Target& target, const cv::aruco::CharucoBoard& charuco,
        std::vector<cv::Point3d> object_points);

  Target target_;
  cv::aruco::CharucoBoard charuco_;
  std::vector<cv::Point3d> object_points_;
};

}  // namespace volumetric_kit::calib::compute
