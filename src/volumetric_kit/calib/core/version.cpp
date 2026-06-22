// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/version.hpp"

namespace volumetric_kit::calib {

const char* version_string() noexcept { return VC_VERSION_STRING; }

int version_major() noexcept { return VC_VERSION_MAJOR; }

int version_minor() noexcept { return VC_VERSION_MINOR; }

int version_patch() noexcept { return VC_VERSION_PATCH; }

}  // namespace volumetric_kit::calib
