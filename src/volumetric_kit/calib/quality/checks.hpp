// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// What the quality tier's checks share: private to the tier.

#include <cmath>
#include <cstdio>
#include <string>

#include "volumetric_kit/calib/quality/live_checks.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::quality::detail {

// `v` in a gate's message or a verdict's reason: "%.3g".
inline std::string number(double v) {
  char text[32];
  std::snprintf(text, sizeof text, "%.3g", v);
  return text;
}

// Refuses options out of range, each message `prefix` and the field's name.
inline core::Status check_view_options(const ViewCheckOptions& options,
                                       const std::string& prefix) {
  if (!(options.oblique_deg > 0.0 && options.oblique_deg <= 180.0)) {
    return core::Status::invalid_argument(prefix +
                                          "oblique_deg must be in (0, 180]");
  }
  if (!(options.far_m > 0.0) || !std::isfinite(options.far_m)) {
    return core::Status::invalid_argument(prefix +
                                          "far_m must be positive and finite");
  }
  if (!(options.frontal_deg >= 0.0 && options.frontal_deg <= 180.0)) {
    return core::Status::invalid_argument(prefix +
                                          "frontal_deg must be in [0, 180]");
  }
  return {};
}

}  // namespace volumetric_kit::calib::quality::detail
