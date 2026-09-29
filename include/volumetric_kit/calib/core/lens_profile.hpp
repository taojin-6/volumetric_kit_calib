// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file lens_profile.hpp
/// @brief Physical lens parameters and capture-hygiene flags.
///
/// Feeds the optics analyses (depth-of-field slab, diffraction ceiling, optimal
/// aperture) and the requirements solver. The `has_*` flags drive the
/// capture-hygiene checks: image stabilization moves the optical center
/// frame-to-frame and autofocus breathes the focal length, so both must be off
/// during calibration (see DESIGN.md).

#include <string>

namespace volumetric_kit::calib {

/// @brief Physical parameters of a lens. Zero fields mean "unknown".
struct LensProfile {
  double focal_length_mm = 0.0;          ///< Focal length (millimetres).
  double f_number = 0.0;                 ///< Aperture f-number (N), e.g. 2.8.
  double focus_distance_m = 0.0;         ///< Focus distance (metres).
  bool has_image_stabilization = false;  ///< OIS/IS present (must be disabled).
  bool has_autofocus = false;            ///< AF present (must be locked off).
  std::string mount;  ///< Lens mount / identifier (free-form; optional).
};

}  // namespace volumetric_kit::calib
