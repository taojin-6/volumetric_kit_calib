// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file calib_rig.hpp
/// @brief calib_rig, the live rig calibration application: a window showing
///        every camera of the rig live, with the board found in each, and
///        captures saved to a session on disk.
///
/// Three threads. The window's (T0) draws the rig mosaic and its marks and
/// panels, and turns the keys and buttons into requests; it never waits on a
/// camera, a detector or the disk. The live worker (T1, @ref LiveWorker)
/// polls the rig, reads each set's luma back, finds the board and drives the
/// capture rule. The capture worker (T3, @ref CaptureWorker) detects each
/// capture at full resolution and queues it on the session writer. The rig's
/// cameras decode onto the compute library's side of one `VkDevice` that gfx
/// shares, so the window copies each decoded picture into its tile on the
/// GPU, and only the luma the detector reads crosses to the host.

#include "volumetric_kit/calib/app/export.hpp"
#include "volumetric_kit/calib/app/preset.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"

namespace volumetric_kit::calib::app {

/// @brief What calib_rig checks of its scenario before it opens a window or
///        a camera.
/// @param config  The scenario, as `core::config::load` resolved it for
///                `Application::CalibRig`.
/// @return The rig's preset (@ref rig_preset's refusals); or `NotFound` when
///         `output.path` is not an existing directory.
///
/// @code
/// VKC_ASSIGN(const RigPreset preset, check_scenario(config));
/// @endcode
VC_APP_API core::Result<RigPreset> check_scenario(
    const core::config::Config& config);

/// @brief Run calib_rig until its window closes, or Ctrl+C in the terminal.
///
/// After @ref check_scenario it opens a window of `config.window`'s size,
/// with the validation layer when `window.validation`, the one device gfx and
/// recon share, and every camera of the rig's sync file
/// (`source.sync_config`) at the preset (@ref check_preset). Space, Right,
/// Page Down or S (or the Capture button) captures; Backspace, Left or Page Up
/// (or Drop last) drops the last capture (@ref key_request). Dragging pans
/// the mosaic and the wheel zooms it. On the way out it stops the live worker,
/// lets the capture worker write what it holds, and waits for the GPU.
/// @param config  The scenario.
/// @return OK once the window closes; or what failed: the scenario, the
///         window or device, a camera's open or preset, or a frame.
///
/// @code
/// VKC_ASSIGN(const auto config, core::config::load(
///                                   path,
///                                   core::config::Application::CalibRig));
/// VKC_TRY(run_calib_rig(config));
/// @endcode
VC_APP_API core::Status run_calib_rig(const core::config::Config& config);

}  // namespace volumetric_kit::calib::app
