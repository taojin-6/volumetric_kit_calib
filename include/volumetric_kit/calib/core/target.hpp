// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file target.hpp
/// @brief Calibration target (board) description.
///
/// Carries the geometry the detector and solver need, plus the physical
/// tolerances the quality tier propagates: the measured square size and its
/// tolerance scale every metric extrinsic, and board non-planarity (bow) leaks
/// straight into the intrinsics (see DESIGN.md "Target fidelity").

#include <optional>

namespace volumetric_kit::calib {

/// @brief The pattern family of a calibration target.
enum class TargetKind {
  Checkerboard,          ///< Symmetric checkerboard; best corner accuracy,
                         ///< full board must be visible, 180-degree
                         ///< orientation ambiguity.
  ChArUco,               ///< Checkerboard + ArUco markers; robust to
                         ///< occlusion/partial views, unambiguous orientation.
  SymmetricCircleGrid,   ///< Circles on a square grid; blur-robust, but the
                         ///< projected centroid is perspective/distortion-
                         ///< biased.
  AsymmetricCircleGrid,  ///< Circles with alternate rows offset by half a
                         ///< pitch (OpenCV's asymmetric grid); same bias.
  AprilGrid,             ///< Grid of AprilTags.
  Screen,                ///< Pattern shown on a calibrated display: flatness
                         ///< solved, geometry known to the pixel pitch.
};

/// @brief A calibration board: pattern, layout, and physical tolerances.
///
/// `rows` and `cols` count the grid of features the detector reports, which is
/// what `DetectedPoint::id` indexes: inner corners for a checkerboard or
/// ChArUco board, circles for a circle grid, tags for an AprilGrid. OpenCV's
/// `CharucoBoard`, and the legacy stereo and bundle-adjustment scripts, count
/// squares instead, one more on each axis; `findChessboardCorners` and the
/// legacy intrinsics script count inner corners, as here.
///
/// @code
/// // The legacy scripts' 5 x 3-square ChArUco board: 4 x 2 inner corners.
/// Target board;
/// board.kind = TargetKind::ChArUco;
/// board.rows = 2;
/// board.cols = 4;
/// board.square_size_m = 0.15;
/// board.marker_size_m = 0.112;
/// board.planarity_bow_m = 0.0005;  // measured; tolerance left unknown
/// @endcode
struct Target {
  TargetKind kind = TargetKind::ChArUco;
  int rows = 0;  ///< Feature rows (see above; inner corners for ChArUco).
  int cols = 0;  ///< Feature columns (see above; inner corners for ChArUco).
  double square_size_m = 0.0;  ///< Measured square/spacing size (metres).
  /// ArUco/AprilTag marker size (metres); 0 for a pattern without markers.
  double marker_size_m = 0.0;
  /// 1-sigma tolerance on @ref square_size_m (metres) -- print/thermal error;
  /// propagated to rig-scale uncertainty. Empty when unknown.
  std::optional<double> square_size_tolerance_m;
  /// Peak out-of-plane deviation of the physical board (metres); 0 when
  /// measured flat, empty when not measured.
  std::optional<double> planarity_bow_m;
};

}  // namespace volumetric_kit::calib
