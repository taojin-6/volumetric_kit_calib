// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file calib_rig.hpp
/// @brief calib_rig, the live rig calibration application: a window showing
///        every camera of the rig live, with the board found in each,
///        captures saved to a session on disk, and Calibrate, whose result
///        is each camera's lens (intrinsic mode) or the rig's poses, the live
///        points shown at them (extrinsic mode); or, in Scan mode, every
///        camera's picture saved as a still at each press.
///
/// Five threads. The window's (T0) draws the rig mosaic and its marks and
/// panels, or the 3D view (@ref RigMode), and turns the keys and buttons into
/// requests; it never waits on a camera, a detector, the solve or the disk.
/// The live worker (T1, @ref LiveWorker) takes each set from the rig's
/// source, its luma and half made on the GPU and read back, finds the board
/// and drives the capture rule. The 3D-view worker (T2, @ref CloudWorker)
/// prepares each sampled RGB-D set's depth and colour on the GPU. The capture
/// worker (T3, @ref CaptureWorker) detects each capture at full resolution,
/// or makes a scan's stills, and queues it on the session writer.
/// Calibrate's worker (T4,
/// @ref CalibrateWorker) solves the captures kept and writes the results
/// directory. The rig's cameras decode onto the compute library's side of one
/// `VkDevice` that gfx shares, so the window copies each picture into its
/// tile and draws each prepared depth as points on the GPU, and only the luma
/// the detector reads crosses to the host. Only the opener
/// (@ref open_source) names a driver.

#include "volumetric_kit/calib/app/open_source.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/core/vulkan/device_requirements.hpp"

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

/// @brief Run calib_rig until its window closes, or Ctrl+C in the terminal.
///
/// After @ref check_scenario it opens a window of `config.window`'s size,
/// with the validation layer when `window.validation`, the one device gfx and
/// sensor's tiers share, and the scenario's cameras by its driver
/// (@ref open_source), starting in its mode (calib_mode.hpp): intrinsic mode
/// (Lenses) fits each camera's lens and has no 3D view; extrinsic mode
/// (Poses) solves the rig; Scan saves every camera's picture at a press, in
/// colour, with no board, no Calibrate and no 3D view. The Setup window
/// switches the mode and the lens
/// file, the cameras left open (@ref switch_session). Ctrl+C or
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
///         window or device, a camera's open, preset or lens, a frame, or
///         what anything it called threw.
///
/// @code
/// VKC_ASSIGN(const auto config, core::config::load(
///                                   path,
///                                   core::config::Application::CalibRig));
/// VKC_TRY(run_calib_rig(config));
/// @endcode
core::Status run_calib_rig(const core::config::Config& config);

}  // namespace volumetric_kit::calib::app
