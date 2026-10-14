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

bool is_usable(ViewVerdict verdict) noexcept {
  switch (verdict) {
    case ViewVerdict::Good:
    case ViewVerdict::Partial:
    case ViewVerdict::Oblique:
    case ViewVerdict::Far:
      return true;
    case ViewVerdict::None:
    case ViewVerdict::Collinear:
      return false;
  }
  return false;
}

}  // namespace volumetric_kit::calib
