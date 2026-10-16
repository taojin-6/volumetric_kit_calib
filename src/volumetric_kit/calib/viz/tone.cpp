// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/viz/tone.hpp"

namespace volumetric_kit::calib::viz {

ImU32 tone_color(Tone tone) noexcept {
  switch (tone) {
    case Tone::Normal:
      break;
    case Tone::Muted:
      return IM_COL32(140, 140, 140, 255);
    case Tone::Good:
      return IM_COL32(80, 210, 120, 255);
    case Tone::Warn:
      return IM_COL32(255, 176, 0, 255);
    case Tone::Bad:
      return IM_COL32(255, 80, 80, 255);
  }
  return IM_COL32(230, 230, 230, 255);
}

Tone verdict_tone(const ViewCheck& check,
                  const quality::SessionOptions& thresholds) noexcept {
  switch (check.verdict) {
    case ViewVerdict::Good:
    case ViewVerdict::Partial:
      return Tone::Good;
    case ViewVerdict::Far:
      return Tone::Warn;
    case ViewVerdict::Oblique:
      return check.tilt_deg && *check.tilt_deg > thresholds.solve.oblique_deg
                 ? Tone::Bad
                 : Tone::Warn;
    case ViewVerdict::Collinear:
      return Tone::Bad;
    case ViewVerdict::None:
      break;
  }
  return Tone::Muted;
}

}  // namespace volumetric_kit::calib::viz
