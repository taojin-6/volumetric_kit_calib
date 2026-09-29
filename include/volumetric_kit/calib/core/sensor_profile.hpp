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

namespace volumetric_kit::calib {

/// @brief Physical parameters of an image sensor.
///
/// Defaults are intentionally zero/unset; a value of 0 means "unknown" and the
/// quality analyses that need a given field report it as missing rather than
/// silently assuming one.
struct SensorProfile {
  double pixel_pitch_um = 0.0;        ///< Pixel pitch (micrometres).
  int width_px = 0;                   ///< Active sensor width (pixels).
  int height_px = 0;                  ///< Active sensor height (pixels).
  double read_noise_e = 0.0;          ///< Read noise (electrons RMS).
  double full_well_e = 0.0;           ///< Full-well capacity (electrons).
  double gain_e_per_dn = 0.0;         ///< Conversion gain (electrons per DN).
  double quantum_efficiency = 0.0;    ///< QE at the working wavelength (0..1).
  double dark_current_e_per_s = 0.0;  ///< Dark current (electrons/pixel/s).
  /// Rolling-shutter readout time per line (microseconds); 0 for global
  /// shutter. A non-zero value means a moving target is geometrically sheared
  /// (see DESIGN.md image-quality gate).
  double rolling_shutter_line_time_us = 0.0;
};

}  // namespace volumetric_kit::calib
