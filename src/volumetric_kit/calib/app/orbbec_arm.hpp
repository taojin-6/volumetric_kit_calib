// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The opener's Orbbec arm (open_source.hpp), internal to calib_app and built
// where sensor builds its Orbbec driver (VS_WITH_ORBBEC): the only code of the
// app that names an Orbbec type, so calib_app links sensor_orbbec privately.

#include <cstdint>
#include <vector>

#include "volumetric_kit/calib/app/open_source.hpp"
#include "volumetric_kit/calib/app/preset.hpp"
#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/sensor/orbbec/orbbec_sensor.hpp"
#include "volumetric_kit/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace volumetric_kit::core {
class Allocator;
class Device;
}  // namespace volumetric_kit::core

namespace volumetric_kit::calib::app {

// The arm's checks of a scenario (check_scenario's step 5): its preset
// (rig_preset's refusals) and its sync file, read (sensor's
// read_orbbec_sync_config's refusals, in context source.sync_config).
core::Status check_orbbec(const core::config::Config& config);

// What each Orbbec camera of the rig is opened with: `preset`'s modes and
// colour coding, decoded onto `device` and readable from each of `families`
// (at most sensor::kMaxColorQueueFamilies, the count checked when a camera
// opens), its range `preset`'s depth mode's. No serial, sync settings or
// pose, which the rig's sync file and calibration give, and no colour
// controls, so each camera opens at auto exposure with auto white balance
// on, written where it differs.
sensor::OrbbecSensor::Options orbbec_options(
    const RigPreset& preset, const core::Device& device,
    core::Allocator& allocator, const std::vector<std::uint32_t>& families);

// A Femto's record in its session: its sensor's facts (`info`, with its
// colour model), its device's own, and its depth delay in the rig's sync
// file (0 for a camera it does not list); its lens the factory's.
core::Result<capture::SessionCamera> orbbec_camera(
    const sensor::SensorInfo& info, const sensor::OrbbecDeviceInfo& device,
    const sensor::OrbbecRigSyncConfig& sync);

// open_source's Orbbec arm: the rig's sync file's cameras at the preset,
// checked (check_preset), each camera's lens rig_lens's.
core::Result<OpenedSource> open_orbbec(
    const core::config::Config& config, const RigScenario& scenario,
    const core::Device& device, core::Allocator& allocator,
    const std::vector<std::uint32_t>& families);

}  // namespace volumetric_kit::calib::app
