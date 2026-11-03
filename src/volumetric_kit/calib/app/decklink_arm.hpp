// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The opener's DeckLink arm (open_source.hpp), internal to calib_app and built
// where sensor builds its DeckLink driver (VS_WITH_DECKLINK, Linux x86-64).

#include <cstdint>
#include <vector>

#include "volumetric_kit/calib/app/open_source.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/sensor/decklink/decklink_stereo.hpp"

namespace volumetric_kit::core {
class Allocator;
class Device;
}  // namespace volumetric_kit::core

namespace volumetric_kit::calib::app {

// What the scenario's pair is opened with: its one pair (source.pairs[0]),
// left then right, each camera's id and input from source.cameras, the mode
// (source.color_width, color_height, fps), and no calibration: calib makes
// the lenses. The skew bound and the lock's wait are the driver's defaults.
// InvalidArgument, naming the field, for no pair or more than one, or a pair
// naming a camera source.cameras lacks; the rest is the driver's to refuse
// when it opens.
core::Result<sensor::DecklinkStereo::Options> decklink_options(
    const core::config::Source& source);

// open_source's DeckLink arm: the pair opened with decklink_options, the left
// camera the reference, each camera's record add_stereo_camera's.
core::Result<OpenedSource> open_decklink(
    const core::config::Config& config, const RigScenario& scenario,
    const core::Device& device, core::Allocator& allocator,
    const std::vector<std::uint32_t>& families);

}  // namespace volumetric_kit::calib::app
