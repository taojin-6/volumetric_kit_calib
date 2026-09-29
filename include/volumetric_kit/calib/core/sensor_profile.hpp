// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sensor_profile.hpp
/// @brief Physical image-sensor parameters.
///
/// The `quality` tier is physical, not heuristic: predicting the reprojection
/// floor from noise, flagging diffraction vs. pixel pitch, and answering "is
/// this error good?" all require real sensor numbers. This struct is where they
/// live, attached per camera alongside a @ref lens_profile.hpp "LensProfile".

#include <optional>

namespace volumetric_kit::calib {

/// @brief Physical parameters of an image sensor.
///
/// Every field is optional, and an empty one means "unknown": an analysis that
/// needs it reports it as missing rather than assuming a value. A present zero
/// is a real zero -- a global shutter's line time, or a negligible dark
/// current -- never a stand-in for "unknown".
///
/// @code
/// SensorProfile sensor;
/// sensor.pixel_pitch_um = 3.45;
/// sensor.width_px = 2448;
/// sensor.height_px = 2048;
/// sensor.rolling_shutter_line_time_us = 0.0;  // known: a global shutter
/// if (!sensor.read_noise_e) {
///   // the noise-floor verdict reports read noise as missing
/// }
/// @endcode
struct SensorProfile {
  std::optional<double> pixel_pitch_um;  ///< Pixel pitch (micrometres).
  std::optional<int> width_px;           ///< Active sensor width (pixels).
  std::optional<int> height_px;          ///< Active sensor height (pixels).
  std::optional<double> read_noise_e;    ///< Read noise (electrons RMS).
  std::optional<double> full_well_e;     ///< Full-well capacity (electrons).
  std::optional<double> gain_e_per_dn;  ///< Conversion gain (electrons per DN).
  /// QE at the working wavelength (0..1).
  std::optional<double> quantum_efficiency;
  /// Dark current (electrons/pixel/s).
  std::optional<double> dark_current_e_per_s;
  /// Rolling-shutter readout time per line (microseconds); 0 for a global
  /// shutter. A non-zero value means a moving target is geometrically sheared
  /// (see DESIGN.md image-quality gate).
  std::optional<double> rolling_shutter_line_time_us;
};

}  // namespace volumetric_kit::calib
