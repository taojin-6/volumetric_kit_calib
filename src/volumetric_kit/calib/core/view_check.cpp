// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/core/view_check.hpp"

namespace volumetric_kit::calib {

const char* to_string(ViewVerdict verdict) noexcept {
  switch (verdict) {
    case ViewVerdict::None:
      return "none";
    case ViewVerdict::Good:
      return "good";
    case ViewVerdict::Partial:
      return "partial";
    case ViewVerdict::Collinear:
      return "collinear";
    case ViewVerdict::Oblique:
      return "oblique";
    case ViewVerdict::Far:
      return "far";
  }
  return "unknown";
}

}  // namespace volumetric_kit::calib
