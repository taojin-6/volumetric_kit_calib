// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file calib_rig.hpp
/// @brief calib_rig, the live rig calibration application: a window showing
///        every camera of the rig live, with the board found in each,
///        captures saved to a session on disk, and Calibrate, whose result
///        shows the live points at the solved poses.
///
/// Five threads. The window's (T0) draws the rig mosaic and its marks and
/// panels, or the 3D view (@ref RigMode), and turns the keys and buttons into
/// requests; it never waits on a camera, a detector, the solve or the disk.
/// The live worker (T1, @ref LiveWorker) polls the rig, reads each set's luma
/// back, finds the board and drives the capture rule. The 3D-view worker (T2,
/// @ref CloudWorker) prepares each sampled set's depth and colour on the
/// GPU. The capture worker (T3, @ref CaptureWorker) detects each capture at
/// full resolution and queues it on the session writer. Calibrate's worker
/// (T4, @ref CalibrateWorker) solves the captures kept and writes the results
/// directory. The rig's cameras decode onto the compute library's side of one
/// `VkDevice` that gfx shares, so the window copies each decoded picture into
/// its tile and draws each prepared depth as points on the GPU, and only the
/// luma the detector reads crosses to the host.

#include "volumetric_kit/calib/app/preset.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/core/vulkan/device_requirements.hpp"
#include "volumetric_kit/sensor/orbbec/orbbec_sync_config.hpp"

namespace volumetric_kit::calib::app {

/// @brief What calib_rig's one device must give each side that adopts it.
///
/// @code
/// VKC_ASSIGN(const RigRequirements needs, rig_requirements());
/// config.graphics = needs.graphics;
/// VKC_ASSIGN(auto shared,
///            gfx::glfw::make_shared_device(window, needs.compute, config));
/// @endcode
struct RigRequirements {
  /// The compute library's side: sensor's decoders, readback and frame
  /// preparation, merged with calib_depth's checks.
  core::DeviceRequirements compute;
  /// The renderer's: gfx's, with `largePoints` for points wider than a pixel
  /// when asked.
  core::DeviceRequirements graphics;
};

/// @brief The device calib_rig asks for.
/// @param large_points  Whether the renderer enables `largePoints`. Without
///                      it gfx draws every point one pixel wide, which a
///                      device that lacks the feature is left with.
/// @return Both sides' requirements; or `InvalidArgument` if sensor's and
///         calib_depth's cannot merge (`core::merge`).
///
/// @code
/// VKC_ASSIGN(const RigRequirements wide, rig_requirements(true));
/// @endcode
core::Result<RigRequirements> rig_requirements(bool large_points = true);

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

/// @brief Run calib_rig until its window closes, or Ctrl+C in the terminal.
///
/// After @ref check_scenario it opens a window of `config.window`'s size,
/// with the validation layer when `window.validation`, the one device gfx and
/// sensor's tiers share, and every camera of the rig's sync file
/// (`source.sync_config`) at the preset (@ref check_preset). Ctrl+C or
/// SIGTERM closes the window while it runs; the handlers it replaced are put
/// back when it returns. The camera images come first: there the keys
/// (@ref key_request) or the buttons capture, drop the last capture and run
/// Calibrate, dragging pans the mosaic and the wheel zooms it. Each result
/// fills the window with the 3D view (@ref rig_action has what each key does
/// there), where a left drag orbits, a right drag pans and the wheel
/// dollies. On the way out it stops Calibrate and the other workers, lets the
/// capture worker write what it holds, and waits for the GPU.
/// @param config  The scenario.
/// @return OK once the window closes; or what failed: the scenario, the
///         window or device, a camera's open or preset, a frame, or what
///         anything it called threw.
///
/// @code
/// VKC_ASSIGN(const auto config, core::config::load(
///                                   path,
///                                   core::config::Application::CalibRig));
/// VKC_TRY(run_calib_rig(config));
/// @endcode
core::Status run_calib_rig(const core::config::Config& config);

}  // namespace volumetric_kit::calib::app
