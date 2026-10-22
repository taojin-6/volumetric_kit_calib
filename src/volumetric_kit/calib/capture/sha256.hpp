// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Internal: SHA-256 (FIPS 180-4), which session.json records of the rig's sync
// file. The family has no other; it moves to the core when a second repository
// needs one.

#include <string>
#include <string_view>

namespace volumetric_kit::calib::capture::detail {

// The SHA-256 digest of `bytes` as 64 lowercase hexadecimal digits.
std::string sha256_hex(std::string_view bytes);

}  // namespace volumetric_kit::calib::capture::detail
