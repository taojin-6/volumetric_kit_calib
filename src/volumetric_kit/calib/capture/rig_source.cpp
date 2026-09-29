// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/capture/rig_source.hpp"

namespace volumetric_kit::calib::capture {

const char* to_string(SyncRole role) noexcept {
  switch (role) {
    case SyncRole::Standalone:
      return "standalone";
    case SyncRole::Primary:
      return "primary";
    case SyncRole::Secondary:
      return "secondary";
  }
  return "standalone";
}

std::optional<std::size_t> find_primary(
    const std::vector<CameraInfo>& cameras) noexcept {
  std::optional<std::size_t> found;
  for (std::size_t i = 0; i < cameras.size(); ++i) {
    if (cameras[i].role != SyncRole::Primary) continue;
    if (found) return std::nullopt;  // two primaries name no reference
    found = i;
  }
  return found;
}

std::size_t SetInfo::count() const noexcept {
  std::size_t n = 0;
  for (const bool here : present) n += here ? 1 : 0;
  return n;
}

bool SetInfo::complete() const noexcept {
  return !present.empty() && count() == present.size();
}

}  // namespace volumetric_kit::calib::capture
