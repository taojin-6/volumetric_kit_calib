// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file view_check.hpp
/// @brief What one camera's view of the board is worth: the record the live
///        checks fill, the panels show and a capture stores.
///
/// A plain record in the `core` tier, so the tiers that fill, show and store
/// it need not depend on each other. The thresholds behind each verdict belong
/// to the checks that fill it.

#include <optional>

#include "volumetric_kit/calib/core/export.hpp"

namespace volumetric_kit::calib {

/// @brief What a view is good for.
enum class ViewVerdict {
  None,       ///< No usable view: no board, or too few corners.
  Good,       ///< Usable, with the whole board in view.
  Partial,    ///< Usable, with part of the board in view.
  Collinear,  ///< The corners seen lie on one line, so they fix no pose.
  Oblique,    ///< The board is turned too far from the camera.
  Far,        ///< The board is too far from the camera.
};

/// @return A stable lowercase name for @p verdict (`"good"`, `"partial"`,
///         ...), as calib's files write it; `"unknown"` for a value that is
///         not a @ref ViewVerdict.
VC_CORE_API const char* to_string(ViewVerdict verdict) noexcept;

/// @brief One camera's view of the board in one set: what was detected, the
///        view's own pose, and the verdict.
///
/// Every measure from the view's own pose is empty when the view has none.
///
/// @code
/// ViewCheck check;
/// check.corners = 8;
/// check.markers = 7;
/// check.distance_m = 2.8;
/// check.tilt_deg = 12.0;
/// check.pnp_rms_px = 0.21;
/// check.cell = 4;  // the board's centre is in the middle of the image
/// check.verdict = ViewVerdict::Good;
/// @endcode
struct ViewCheck {
  int corners = 0;         ///< Board corners detected.
  int markers = 0;         ///< Markers detected.
  bool collinear = false;  ///< The corners detected lie on one line.
  /// The board's distance from the camera (metres).
  std::optional<double> distance_m;
  /// How far the board is turned from facing the camera (degrees; 0 is
  /// face-on).
  std::optional<double> tilt_deg;
  /// The pose's reprojection RMS over the view's corners (pixels).
  std::optional<double> pnp_rms_px;
  /// The cell of a 3 x 3 grid over the image that holds the board's centre,
  /// numbered row-major from the top left (0 to 8).
  std::optional<int> cell;
  ViewVerdict verdict = ViewVerdict::None;  ///< What the view is good for.
};

}  // namespace volumetric_kit::calib
