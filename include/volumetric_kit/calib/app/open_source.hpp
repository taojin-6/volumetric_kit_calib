// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file open_source.hpp
/// @brief calib_rig's opener, the one code that names a driver: its scenario's
///        source read and checked before a window opens (@ref check_scenario),
///        then its cameras opened as one `source::RigSource`, with what the
///        session records of each (@ref open_source).
///
/// One arm today, `orbbec`: the rig's sync file, the Femto preset and its
/// check, and each camera's Orbbec facts. Everything after it -- the
/// workers, the session, the views -- reads the source and the records alone.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/app/preset.hpp"
#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/calib/source/rig_source.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/orbbec/orbbec_stream.hpp"
#include "volumetric_kit/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace volumetric_kit::core {
class Allocator;
class Device;
}  // namespace volumetric_kit::core

namespace volumetric_kit::calib::app {

/// @brief What calib_rig takes from its scenario before it opens a window or
///        a camera.
///
/// @code
/// VKC_ASSIGN(const RigScenario scenario, check_scenario(config));
/// const std::size_t cameras = scenario.sync.devices.size();
/// @endcode
struct RigScenario {
  RigPreset preset;  ///< The modes every camera streams at.
  /// The rig's sync file (`source.sync_config`), read: its cameras, each
  /// one's sync settings.
  sensor::OrbbecRigSyncConfig sync;
};

/// @brief What calib_rig checks of its scenario before it opens a window or
///        a camera.
/// @param config  The scenario, as `core::config::load` resolved it for
///                `Application::CalibRig`.
/// @return The scenario's preset and sync file; @ref rig_preset's refusals;
///         `NotFound` when `output.path` is not an existing directory;
///         `sensor::read_orbbec_sync_config`'s refusals of the sync
///         file (missing, unreadable, malformed or listing no camera); or
///         `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const RigScenario scenario, check_scenario(config));
/// @endcode
core::Result<RigScenario> check_scenario(const core::config::Config& config);

/// @brief A Femto's record in its session: its sensor's facts, its device's
///        own and its depth delay in the rig's sync file.
/// @param info    The camera's `SensorInfo`, with its colour model.
/// @param device  What the camera said of itself when it opened.
/// @param sync    The rig's sync configuration: the camera's depth delay, by
///                serial (0 for a camera it does not list).
/// @return The record, its lens the factory's; or `OutOfMemory`.
/// @pre `info.color` holds a model.
///
/// @code
/// VKC_ASSIGN(const capture::SessionCamera camera,
///            orbbec_camera(sensor.info(), sensor.device_info(), sync));
/// @endcode
core::Result<capture::SessionCamera> orbbec_camera(
    const sensor::SensorInfo& info, const sensor::OrbbecDeviceInfo& device,
    const sensor::OrbbecRigSyncConfig& sync);

/// @brief The rig's cameras, open, and what the session records of them.
///
/// @code
/// VKC_ASSIGN(OpenedSource opened,
///            open_source(config, scenario, device, allocator, families));
/// setup.cameras = opened.cameras;
/// @endcode
struct OpenedSource {
  /// Every camera, as one source; opened, not started.
  std::unique_ptr<source::RigSource> source;
  /// Each camera's record, in the source's order.
  std::vector<capture::SessionCamera> cameras;
  /// Each camera's lens, in the source's order: what its views are checked
  /// through and the solve takes.
  std::vector<sensor::camera::CameraModel> lenses;
  std::size_t reference = 0;  ///< The reference camera: the rig's primary.
  std::string driver;         ///< What opened the cameras: `"orbbec"`.
  std::string sync_path;      ///< The rig's sync file; empty for none.
  /// The cameras' depth mode; empty when none has depth.
  std::optional<capture::DepthMode> depth_mode;
  std::string color_codec;  ///< How the cameras code colour: `"mjpeg"`.
};

/// @brief Open the scenario's cameras: the rig's sync file's, each at the
///        preset, decoded onto @p device and readable from each of
///        @p families, checked (@ref check_preset), as one source.
/// @param config     The scenario.
/// @param scenario   What @ref check_scenario read of it.
/// @param device     The compute library's device, which the cameras decode
///                   onto; it must outlive the source and every set.
/// @param allocator  Its allocator.
/// @param families   The queue families that read the pictures: the
///                   compute library's and the renderer's.
/// @return The source and its records; what sensor's opener or
///         `SensorArray::open` refuses (a camera that does not answer
///         among them); @ref check_preset's refusals; what
///         `source::RigSource::open` refuses; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(OpenedSource opened,
///            open_source(config, scenario, device, allocator,
///                        {compute_family, graphics_family}));
/// @endcode
core::Result<OpenedSource> open_source(
    const core::config::Config& config, const RigScenario& scenario,
    const core::Device& device, core::Allocator& allocator,
    const std::vector<std::uint32_t>& families);

}  // namespace volumetric_kit::calib::app
