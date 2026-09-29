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

#include <optional>
#include <string>

namespace volumetric_kit::calib {

/// @brief Physical parameters of a lens.
///
/// Every field is optional, and an empty one means "unknown", as in
/// `SensorProfile`. For the hygiene flags this is the difference between "not
/// checked" and "the lens has none": a check can pass a known `false` but must
/// still ask about an unknown.
///
/// @code
/// LensProfile lens;
/// lens.focal_length_mm = 35.0;
/// lens.f_number = 5.6;
/// lens.has_autofocus = false;  // manual cine glass
/// lens.mount = "PL";
/// // has_image_stabilization stays unknown until someone checks the lens
/// @endcode
struct LensProfile {
  std::optional<double> focal_length_mm;   ///< Focal length (millimetres).
  std::optional<double> f_number;          ///< Aperture f-number (N), e.g. 2.8.
  std::optional<double> focus_distance_m;  ///< Focus distance (metres).
  /// OIS/IS present (it must be disabled for calibration).
  std::optional<bool> has_image_stabilization;
  /// AF present (it must be locked off for calibration).
  std::optional<bool> has_autofocus;
  std::string mount;  ///< Lens mount, e.g. "PL" or "EF"; empty when unknown.
};

}  // namespace volumetric_kit::calib
