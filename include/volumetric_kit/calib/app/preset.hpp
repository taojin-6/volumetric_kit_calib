// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file preset.hpp
/// @brief The modes calib_rig streams every camera at, from its scenario, and
///        the check that each camera opened at them.
///
/// The scenario (`core::config::Application::CalibRig`) names the colour and
/// depth sizes and the rate; the core checks only that they are positive.
/// Whether a camera has the mode is the driver's to say when it opens, and
/// whether every camera opened at it is @ref check_preset's, before the app
/// draws a frame. A rig of any number of cameras.

#include <cstdint>
#include <string>
#include <vector>

#include "volumetric_kit/calib/app/export.hpp"
#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sensor.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"

namespace volumetric_kit::core {
class Allocator;
class Device;
}  // namespace volumetric_kit::core

namespace volumetric_kit::calib::app {

/// @brief The modes every camera of the rig streams at: what calib_rig asks
///        each camera for, and holds each to.
///
/// @code
/// VKC_ASSIGN(const RigPreset preset, rig_preset(config.source));
/// // The defaults: 3840 x 2160 MJPEG colour, 640 x 576 depth, 25 fps.
/// @endcode
struct RigPreset {
  recon::camera::ImageSize color{3840, 2160};  ///< The colour image's size.
  recon::camera::ImageSize depth{640, 576};    ///< The depth image's size.
  std::uint32_t fps = 25;                      ///< Frames a second, both.
  /// The depth mode those sizes are: its name and range (@ref depth_mode).
  capture::DepthMode depth_mode;
};

/// @brief The depth mode a Femto Mega streams at @p depth: its name and the
///        range its maker states.
///
/// - 640 x 576: `nfov_unbinned`, 0.5-3.86 m;
/// - 320 x 288: `nfov_binned`, 0.5-5.46 m;
/// - 1024 x 1024: `wfov_unbinned`, 0.25-2.21 m;
/// - 512 x 512: `wfov_binned`, 0.25-2.88 m.
/// @param depth  The depth image's size.
/// @return The mode, in millimetre units (`metres_per_unit` 0.001);
///         `InvalidArgument` naming any other size; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const capture::DepthMode mode, depth_mode({640, 576}));
/// @endcode
VC_APP_API core::Result<capture::DepthMode> depth_mode(
    recon::camera::ImageSize depth);

/// @brief The preset a CalibRig scenario's source asks for.
/// @param source  The scenario's source, as `core::config::load` resolved it
///                for `Application::CalibRig`.
/// @return Its sizes, rate and depth mode; `InvalidArgument` for a colour
///         codec other than MJPEG (a capture saves each camera's JPEG), or a
///         depth size @ref depth_mode does not name; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const auto config, core::config::load(
///                                   path,
///                                   core::config::Application::CalibRig));
/// VKC_ASSIGN(const RigPreset preset, rig_preset(config.source));
/// @endcode
VC_APP_API core::Result<RigPreset> rig_preset(
    const core::config::Source& source);

/// @brief Refuse a rig whose cameras did not all open at @p preset.
///
/// Each camera must report a colour model of the preset's colour size, a
/// depth model of its depth size, and its rate.
/// @param cameras  Each camera's `SensorInfo`, in the rig's order; at least
///                 one.
/// @param preset   The modes.
/// @return OK; `InvalidArgument` for no camera; `Unsupported` naming the first
///         camera and what it reports instead (its colour or depth size, a
///         missing model, or its rate); or `OutOfMemory`.
///
/// @code
/// VKC_TRY(check_preset(infos, preset));  // before the window draws a frame
/// @endcode
VC_APP_API core::Status check_preset(
    const std::vector<recon::sensor::SensorInfo>& cameras,
    const RigPreset& preset);

/// @brief What each Orbbec camera of the rig is opened with: @p preset's
///        modes in MJPEG, decoded onto @p device and readable from each of
///        @p families, its range @p preset's depth mode's.
/// @param preset     The modes.
/// @param device     The compute library's device, which the cameras decode
///                   onto; it must outlive them and every frame.
/// @param allocator  Its allocator, as `OrbbecStreamOptions::allocator`.
/// @param families   The queue families that read the decoded pictures: the
///                   readback's and the renderer's; at most
///                   `recon::sensor::kMaxColorQueueFamilies`, and the count
///                   is checked when a camera opens.
/// @return The options, for `recon::sensor::open_orbbec_sensors`; no serial,
///         sync settings or pose, which the rig's sync file and calibration
///         give.
///
/// @code
/// const auto options = orbbec_options(preset, device, allocator,
///                                     {compute_family, graphics_family});
/// VKC_ASSIGN(auto sensors, recon::sensor::open_orbbec_sensors(
///                              config.source.sync_config, options));
/// @endcode
VC_APP_API recon::sensor::OrbbecSensor::Options orbbec_options(
    const RigPreset& preset, const core::Device& device,
    core::Allocator& allocator, const std::vector<std::uint32_t>& families);

}  // namespace volumetric_kit::calib::app
