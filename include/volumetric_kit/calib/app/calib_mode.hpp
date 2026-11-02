// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file calib_mode.hpp
/// @brief What calib_rig does in each mode, the scenario's `operation`:
///        intrinsic (`calibrate_intrinsics`), each camera's lens; or
///        extrinsic (`calibrate`), the rig's poses. The workers read the
///        mode, never the driver.
///
/// | | Intrinsic | Extrinsic |
/// | --- | --- | --- |
/// | Board | the checkerboard first; either kind, with a lens view's corners |
/// the rig's ChArUco board | | Lens | the file's, the factory's, else the
/// stand-in | the file's, the factory's, else none: the run stops | | Capture |
/// one view is enough | two views | | Calibrate | once a camera is ready (@ref
/// lens_states) | once the session's links are | | Its result | a lens file |
/// the rig's poses, in the 3D view |

#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/app/capture_worker.hpp"
#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/quality/capture_rule.hpp"
#include "volumetric_kit/calib/quality/lens_readiness.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"
#include "volumetric_kit/calib/viz/board_editor.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"

namespace volumetric_kit::calib::app {

/// @brief The mode @p operation asks for.
/// @param operation  The scenario's operation.
/// @return `Intrinsics` for `calibrate_intrinsics`, `Extrinsics` for
///         `calibrate`; `InvalidArgument` naming any other ("operation:
///         fuse is not ...").
///
/// @code
/// VKC_ASSIGN(const capture::SessionMode mode, session_mode(config.operation));
/// @endcode
core::Result<capture::SessionMode> session_mode(
    core::config::Operation operation);

/// @brief The board a session of @p mode starts from, as the Board window
///        shows it: `legacy_checkerboard()` for lenses, `legacy_femto_board()`
///        for poses.
/// @param mode  The mode.
/// @return The board.
///
/// @code
/// VKC_ASSIGN(viz::BoardEditor editor,
///            viz::BoardEditor::create(board_seed(mode), false,
///                                     board_rule(mode)));
/// @endcode
Target board_seed(capture::SessionMode mode) noexcept;

/// @brief Which boards a session of @p mode takes: for lenses, either kind
///        with at least `solve::LensOptions::min_corners` inner corners, so
///        a whole board gives a lens view (the rig's 5 x 3 ChArUco board has
///        8, and never would); for poses, ChArUco alone, whose ids hold
///        across cameras.
/// @param mode  The mode.
/// @return The rule.
viz::BoardRule board_rule(capture::SessionMode mode) noexcept;

/// @brief The capture rule's thresholds in @p mode: for lenses one view is
///        a capture (`min_views` 1), as each camera is fitted alone; for
///        poses the rule's defaults.
/// @param mode  The mode.
/// @return The thresholds.
///
/// @code
/// LiveOptions options;
/// options.rule = capture_rule(mode);
/// @endcode
quality::CaptureRuleOptions capture_rule(capture::SessionMode mode) noexcept;

/// @brief A camera's lens for the live view and the session's checks, and
///        where it came from.
///
/// @code
/// VKC_ASSIGN(const RigLens lens, rig_lens(mode, id, size, factory, file,
///                                         path));
/// camera.lens = lens.origin;
/// @endcode
struct RigLens {
  sensor::camera::CameraModel model;  ///< The lens, at the pictures' size.
  capture::LensOrigin origin = capture::LensOrigin::Factory;  ///< Its origin.
};

/// @brief The stand-in lens: `fx = fy = width / 2`, centred, no distortion.
///        Distance and tilt through it are rough, and wrong at a fisheye's
///        edge; corners, cells and reach do not depend on it. Never a lens
///        fit's start (@ref lens_start).
/// @param size  The pictures' size.
/// @return The lens.
sensor::camera::CameraModel stand_in_lens(
    sensor::camera::ImageSize size) noexcept;

/// @brief A camera's lens, the first of:
///        1. @p file's colour lens for @p id, at exactly @p size;
///        2. @p factory, the driver's own (a Femto's colour lens);
///        3. for lenses, @ref stand_in_lens.
///
/// For poses there is no stand-in, and a factory camera with a calibrated
/// lens in @p file is refused: the runtime would read its factory lens and
/// leave the file's unused.
/// @param mode       The mode.
/// @param id         The camera's id, its key in @p file.
/// @param size       Its pictures' size.
/// @param factory    Its factory lens, at @p size; empty for none.
/// @param file       The scenario's calibration file (`source.calibration`);
///                   null for none.
/// @param file_path  Its path, for the messages.
/// @return The lens and its origin; `InvalidArgument`, for poses, for a
///         camera with no lens ("bm-2 has no lens at 3840x2160 in <path>:
///         run intrinsic mode") or a factory camera with a calibrated one
///         in @p file; or `OutOfMemory`.
core::Result<RigLens> rig_lens(
    capture::SessionMode mode, const std::string& id,
    sensor::camera::ImageSize size,
    const std::optional<sensor::camera::CameraModel>& factory,
    const sensor::camera::ArrayCalibration* file, const std::string& file_path);

/// @brief Where a camera's lens fit starts besides OpenCV's own
///        (`solve::solve_lens`'s `start`): a real lens at @p size, the first
///        of @p file's lens for @p id, another camera's lens in @p file (as
///        bm-1's for bm-3), and @p factory. Never the stand-in: from a poor
///        start the fit lands in wrong minima.
/// @param id       The camera's id.
/// @param size     Its pictures' size.
/// @param factory  Its factory lens; empty for none.
/// @param file     The scenario's calibration file; null for none.
/// @return The start; empty for none.
///
/// @code
/// const auto start = lens_start(id, size, camera.color,
/// kept.calibration.get()); VKC_ASSIGN(solve::LensFit fit,
///            solve::solve_lens(id, size, board, views, start));
/// @endcode
std::optional<sensor::camera::CameraModel> lens_start(
    const std::string& id, sensor::camera::ImageSize size,
    const std::optional<sensor::camera::CameraModel>& factory,
    const sensor::camera::ArrayCalibration* file);

/// @brief How far along each camera's views are toward its lens fit
///        (`quality::lens_readiness`, its defaults), over every kept
///        capture: the Lenses panel, and what Calibrate fits. Frontal views
///        are not judged through a stand-in.
/// @param kept  The captures kept and what they were taken with.
/// @return One per camera, in rig order; `InvalidArgument` for a capture
///         whose views or frontal flags are not one per camera; what
///         `lens_readiness` refuses; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const std::vector<quality::LensReadiness> states,
///            lens_states(*kept));
/// @endcode
core::Result<std::vector<quality::LensReadiness>> lens_states(
    const KeptCaptures& kept);

/// @brief Whether Calibrate may run in @p mode: for lenses, once a camera is
///        ready; for poses, once the session is (`SessionSummary::ready`).
/// @param mode     The mode.
/// @param summary  The session's checks; empty before the first capture.
/// @param lenses   Each camera's lens readiness (@ref lens_states).
/// @return Whether it may run.
///
/// @code
/// const bool ready = session_ready(mode, captures->summary(),
///                                  captures->lenses());
/// @endcode
bool session_ready(capture::SessionMode mode,
                   const std::optional<quality::SessionSummary>& summary,
                   const std::vector<quality::LensReadiness>& lenses) noexcept;

}  // namespace volumetric_kit::calib::app
