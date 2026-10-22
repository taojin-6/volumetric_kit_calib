// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tone.hpp
/// @brief How the live view colours what it shows: a tone for each value, and
///        a view's verdict as one.

#include <cstdint>

#include "imgui.h"
#include "volumetric_kit/calib/core/view_check.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"

namespace volumetric_kit::calib::viz {

/// @brief What a value shown says about the calibration.
enum class Tone : std::uint8_t {
  Normal,  ///< A plain value.
  Muted,   ///< Nothing to show yet, or not in play.
  Good,    ///< As it should be (green).
  Warn,    ///< Usable, but worth acting on (amber).
  Bad,     ///< Not usable as it is (red).
};

/// @brief The colour a tone is drawn in.
/// @param tone  The tone.
/// @return Its colour (`IM_COL32`); `Normal`'s for a value that is not a
///         @ref Tone.
///
/// @code
/// ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(tone_color(tone)), "%s",
///                    value.c_str());
/// @endcode
ImU32 tone_color(Tone tone) noexcept;

/// @brief A view's verdict as a tone, as the live view shows it.
///
/// `Good` and `Partial` are `Good`; `Far` is `Warn`; `Oblique` is `Warn`,
/// or `Bad` once its tilt passes the solve's `oblique_deg`, which flags it
/// too; `Collinear` is `Bad`; `None` is `Muted`.
/// @param check       The view's check.
/// @param thresholds  The session's thresholds: `solve.oblique_deg`.
/// @return The tone.
///
/// @code
/// const Tone tone = verdict_tone(view.check, session_options);
/// @endcode
Tone verdict_tone(const ViewCheck& check,
                  const quality::SessionOptions& thresholds) noexcept;

}  // namespace volumetric_kit::calib::viz
