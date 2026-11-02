// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file open_source.hpp
/// @brief calib_rig's opener, the one code that names a driver: its scenario
///        read and checked before a window opens (@ref check_scenario), then
///        its cameras opened as one `source::RigSource`, with what the
///        session records of each (@ref open_source).
///
/// Two arms, each built where sensor builds its driver:
/// - `orbbec` (`VS_WITH_ORBBEC`): the rig's sync file, the Femto preset and
///   its check, and each camera's Orbbec facts;
/// - `decklink` (`VS_WITH_DECKLINK`, Linux x86-64): the scenario's one pair
///   of SDI inputs as sensor's `DecklinkStereo`, opened with no calibration,
///   for its lenses (intrinsic mode).
///
/// Each camera's lens is the mode's (`rig_lens`, calib_mode.hpp). Everything
/// after the opener -- the workers, the session, the views -- reads the
/// source and the records alone, and no public header of the app names a
/// driver's type.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/calib/source/rig_source.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"

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
/// if (scenario.mode == capture::SessionMode::Intrinsics) seed_checkerboard();
/// @endcode
struct RigScenario {
  /// What it calibrates, by `operation`: `calibrate_intrinsics` each
  /// camera's lens, `calibrate` the rig's poses.
  capture::SessionMode mode = capture::SessionMode::Extrinsics;
  std::string driver;  ///< `source.driver`: `"orbbec"` or `"decklink"`.
  /// `source.calibration`, read: the lenses the cameras start from; null
  /// for none.
  std::shared_ptr<const sensor::camera::ArrayCalibration> calibration;
};

/// @brief What calib_rig checks of its scenario before it opens a window or
///        a camera: that it runs what calib supports, so no scenario runs
///        another solve than it asks for.
///
/// 1. `operation` is `calibrate` or `calibrate_intrinsics`.
/// 2. `source.driver` is `orbbec`, or `decklink` with `calibrate_intrinsics`
///    (a DeckLink pair's poses are not solved yet), and this build has the
///    driver's arm.
/// 3. `output.path` is an existing directory.
/// 4. `source.calibration`, when given, reads as sensor's calibration file.
/// 5. The driver's own: for `orbbec`, the preset (`rig_preset`) and the
///    sync file (`source.sync_config`); for `decklink`, the pair and its
///    cameras (`source.pairs`, `source.cameras`) as its driver takes them.
/// @param config  The scenario, as `core::config::load` resolved it for
///                `Application::CalibRig`.
/// @return The mode, the driver and the calibration file; `InvalidArgument`
///         naming the field and its value for an operation or a driver
///         calib_rig does not run, or a driver this build lacks, saying how
///         to build it; `NotFound` when `output.path` is not an existing
///         directory; `read_array_calibration`'s refusals of
///         `source.calibration`; the driver's refusals, each in its field's
///         context (`source.sync_config`, ...); or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const RigScenario scenario, check_scenario(config));
/// @endcode
core::Result<RigScenario> check_scenario(const core::config::Config& config);

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
  /// Each camera's record, in the source's order, its lens's origin among
  /// it.
  std::vector<capture::SessionCamera> cameras;
  /// Each camera's lens, in the source's order (`rig_lens`): what its views
  /// are checked through and the solve takes.
  std::vector<sensor::camera::CameraModel> lenses;
  std::size_t reference = 0;  ///< The reference camera: the rig's primary.
  /// What opened the cameras: `"orbbec"` or `"decklink"`.
  std::string driver;
  std::string sync_path;  ///< The rig's sync file; empty for none.
  /// The cameras' depth mode; empty when none has depth.
  std::optional<capture::DepthMode> depth_mode;
  /// How the cameras code colour (`"mjpeg"`); empty when not coded.
  std::string color_codec;
};

/// @brief Open the scenario's cameras by its driver's arm, decoded onto
///        @p device and readable from each of @p families, as one source.
///
/// - `orbbec`: the rig's sync file's cameras at the preset, each checked
///   (`check_preset`), its factory lens the driver's.
/// - `decklink`: the pair's two inputs at the scenario's mode, with no
///   calibration (`DecklinkStereo::Options::calibration` empty), the left
///   camera the reference.
///
/// Each camera's lens is `rig_lens`'s for the mode, from
/// `scenario.calibration`, and its record says where it came from.
/// @param config     The scenario.
/// @param scenario   What @ref check_scenario read of it.
/// @param device     The compute library's device, which the cameras decode
///                   onto; it must outlive the source and every set.
/// @param allocator  Its allocator.
/// @param families   The queue families that read the pictures: the
///                   compute library's and the renderer's.
/// @return The source and its records; what the driver refuses (a camera
///         that does not answer among them); `check_preset`'s refusals;
///         `rig_lens`'s; what `source::RigSource::open` refuses; or
///         `OutOfMemory`.
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
