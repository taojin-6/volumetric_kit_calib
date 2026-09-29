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

namespace volumetric_kit::calib {

/// @brief The pattern family of a calibration target.
enum class TargetKind {
  Checkerboard,  ///< Symmetric checkerboard; best corner accuracy, full board
                 ///< must be visible, 180-degree orientation ambiguity.
  ChArUco,       ///< Checkerboard + ArUco markers; robust to occlusion/partial
                 ///< views, unambiguous orientation.
  CircleGrid,    ///< Symmetric/asymmetric circle grid; blur-robust but the
                 ///< projected centroid is perspective/distortion-biased.
  AprilGrid,     ///< Grid of AprilTags.
  Screen,        ///< Pattern shown on a calibrated display: flatness solved,
                 ///< geometry known to the pixel pitch.
};

/// @brief A calibration board: pattern, layout, and physical tolerances.
struct Target {
  TargetKind kind = TargetKind::ChArUco;
  int rows = 0;  ///< Inner corners (checker/ChArUco) or features per column.
  int cols = 0;  ///< Inner corners (checker/ChArUco) or features per row.
  double square_size_m = 0.0;  ///< Measured square/spacing size (metres).
  double marker_size_m = 0.0;  ///< ArUco/AprilTag marker size (metres), if any.
  /// 1-sigma tolerance on @ref square_size_m (metres) — print/thermal error;
  /// propagated to rig-scale uncertainty.
  double square_size_tolerance_m = 0.0;
  /// Peak out-of-plane deviation of the physical board (metres); 0 = assumed
  /// perfectly flat (rigid substrate).
  double planarity_bow_m = 0.0;
};

}  // namespace volumetric_kit::calib
