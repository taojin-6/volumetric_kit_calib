// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Internal to calib_capture: 8-bit grey PNG files through stb. stb's
// functions are compiled into png_io.cpp alone, with internal linkage, so a
// program that also links another copy of stb -- gfx's glTF loader carries
// stb_image -- gets no duplicate symbols.

#include <string>

#include "volumetric_kit/calib/capture/rig_source.hpp"
#include "volumetric_kit/calib/core/result.hpp"

namespace volumetric_kit::calib::capture::detail {

// Write @p image to @p path as an 8-bit grey PNG; IoError if it cannot be.
Status write_png(const std::string& path, const LumaImage& image);

// Read the PNG at @p path as one 8-bit channel, converting colour or 16-bit
// images; IoError if it does not decode.
Result<LumaImage> read_png(const std::string& path);

}  // namespace volumetric_kit::calib::capture::detail
