// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// What the quality tier's checks share: private to the tier.

#include <cmath>
#include <cstddef>
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

// Each direction of a solve::LensReach, as a message names it.
inline constexpr const char* kReachToward[] = {
    "top-left corner",     "top-right corner", "bottom-left corner",
    "bottom-right corner", "left side",        "right side"};

// The standard errors in a cell's limit for its `n` corners, at least 2:
// Student's t for n - 1 degrees of freedom at the two-sided tail a normal's 3
// leaves, 0.27%. Its distribution for whole degrees of freedom (Abramowitz and
// Stegun 26.7.3 and 26.7.4), solved by bisection.
inline double student_t(std::size_t n) {
  constexpr double kPi = 3.14159265358979323846;
  const std::size_t dof = n - 1;
  const double inside = std::erf(3.0 / std::sqrt(2.0));
  // P(|T| <= sqrt(dof) tan(theta)).
  const auto within = [dof](double theta) {
    const double s = std::sin(theta);
    const double c = std::cos(theta);
    double term = dof % 2 == 0 ? 1.0 : c;
    double sum = term;
    for (std::size_t k = dof % 2 == 0 ? 2 : 3; k + 2 <= dof; k += 2) {
      term *= (k - 1.0) / static_cast<double>(k) * c * c;
      sum += term;
    }
    if (dof % 2 == 0) return s * sum;
    return 2.0 / kPi * (theta + (dof > 1 ? s * sum : 0.0));
  };
  double lo = 0.0;
  double hi = kPi / 2.0;
  for (int i = 0; i < 60; ++i) {
    const double mid = 0.5 * (lo + hi);
    (within(mid) < inside ? lo : hi) = mid;
  }
  return std::sqrt(static_cast<double>(dof)) * std::tan(0.5 * (lo + hi));
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
