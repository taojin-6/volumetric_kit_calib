// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file calib_mode.hpp
/// @brief What calib_rig does in each mode, the scenario's `operation`:
///        intrinsic (`calibrate_intrinsics`), each camera's lens; or
///        extrinsic (`calibrate`), the rig's poses. The workers read the
///        mode, never the driver.
///
/// README.md's "Lenses and Poses" table says how the modes differ; each
/// difference has its one home here. The Setup panel switches them
/// (`rig_setup.hpp`).

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/app/capture_worker.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/app/open_source.hpp"
#include "volumetric_kit/calib/app/rig_mode.hpp"
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
///         `calibrate`; `InvalidArgument` for any other, by its number
///         ("operation: 0 is not ..."): core's `load` refuses them for
///         calib_rig, so only a scenario made in code gets one.
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
///        3. @ref stand_in_lens: for lenses, a fit's views go through it; for
///           poses, the live view alone does, and Calibrate waits for a real
///           lens (@ref lens_status: `Missing`).
///
/// For poses a factory camera with a calibrated lens in @p file, at any
/// size, is refused: the runtime would read its factory lens and leave the
/// file's unused.
/// @param mode       The mode.
/// @param id         The camera's id, its key in @p file.
/// @param size       Its pictures' size.
/// @param factory    Its factory lens, at @p size; empty for none.
/// @param file       The lens file (`source.calibration`, or the one the
///                   Setup panel loaded); null for none.
/// @param file_path  Its path, for the messages.
/// @return The lens and its origin; `InvalidArgument`, for poses, for a
///         factory camera with a calibrated one in @p file; or
///         `OutOfMemory`.
core::Result<RigLens> rig_lens(
    capture::SessionMode mode, const std::string& id,
    sensor::camera::ImageSize size,
    const std::optional<sensor::camera::CameraModel>& factory,
    const sensor::camera::ArrayCalibration* file, const std::string& file_path);

/// @brief A camera's lens by @ref rig_lens, its record told where it came
///        from: `lens` its origin, `lens_file` @p path for a file's lens,
///        empty otherwise. Its factory lens is the record's `color`.
/// @param mode    The mode.
/// @param file    The lens file; null for none.
/// @param path    Its path.
/// @param camera  The camera's record: its id, size and factory lens are
///                read, its lens origin and file set.
/// @return The lens; or what @ref rig_lens refuses, @p camera then unchanged.
///
/// @code
/// VKC_ASSIGN(opened.lenses.emplace_back(),
///            take_lens(mode, file.get(), path, camera));
/// @endcode
core::Result<sensor::camera::CameraModel> take_lens(
    capture::SessionMode mode, const sensor::camera::ArrayCalibration* file,
    const std::string& path, capture::SessionCamera& camera);

/// @brief Where a camera's lens comes from, as the Setup panel says it.
enum class LensStatus : std::uint8_t {
  Calibrated,  ///< A lens file's, marked calibrated.
  Factory,     ///< The driver's, or a lens file's marked factory.
  StandIn,     ///< None, for lenses: the fit's views go through a stand-in.
  Missing,     ///< None, for poses: Calibrate waits for one.
};

/// @return @p status's name: `"calibrated"`, `"factory"`, `"stand-in"` or
///         `"missing"`; `"unknown"` for a value @ref LensStatus does not name.
const char* to_string(LensStatus status) noexcept;

/// @brief Where @p camera's lens comes from in @p mode: a file's keeps its
///        mark (@ref lens_source), the driver's is `Factory`, and the
///        stand-in is `StandIn` for lenses, `Missing` for poses.
/// @param mode    The mode.
/// @param camera  The camera's record (@ref take_lens).
/// @param file    The lens file its lens came from; null for none.
/// @return The status.
///
/// @code
/// if (lens_status(mode, camera, file) == LensStatus::Missing) warn(id);
/// @endcode
LensStatus lens_status(capture::SessionMode mode,
                       const capture::SessionCamera& camera,
                       const sensor::camera::ArrayCalibration* file);

/// @brief Each camera's @ref lens_status, in rig order.
/// @param mode     The mode.
/// @param cameras  The cameras' records.
/// @param file     The lens file; null for none.
/// @return The statuses; or `OutOfMemory`.
core::Result<std::vector<LensStatus>> lens_statuses(
    capture::SessionMode mode,
    const std::vector<capture::SessionCamera>& cameras,
    const sensor::camera::ArrayCalibration* file);

/// @brief How a calibration file marks a camera's lens, from where it came
///        from: what the solve carries into the file it writes
///        (`solve::RigCamera::source`). A lens from @p file keeps @p file's
///        mark (a pair's lenses from intrinsic mode stay `Calibrated`); any
///        other is `Factory` (no pose session takes a stand-in).
/// @param camera  The camera's record: its id, size and lens origin.
/// @param file    The scenario's calibration file, which `rig_lens` took
///                the lens from; null for none.
/// @return The mark.
///
/// @code
/// const auto mark = lens_source(camera, setup.calibration.get());
/// rig.push_back({camera.id, lens, mark});
/// @endcode
sensor::camera::IntrinsicsSource lens_source(
    const capture::SessionCamera& camera,
    const sensor::camera::ArrayCalibration* file);

/// @brief A camera's `lens` line in its panel (`viz::CameraLive::lens`): for
///        poses, its @ref lens_status, then ` (file)` when the lens is
///        @p file's, as `calibrated (file)` or `missing`; for lenses none, as
///        the Lenses panel says where each came from.
/// @param mode    The mode.
/// @param camera  The camera's record: its id, size and lens origin.
/// @param file    The scenario's calibration file; null for none.
/// @return The line's text, empty for none; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(cameras[i].lens,
///            lens_line(mode, setup.cameras[i], setup.calibration.get()));
/// @endcode
core::Result<std::string> lens_line(
    capture::SessionMode mode, const capture::SessionCamera& camera,
    const sensor::camera::ArrayCalibration* file);

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
/// const auto start =
///     lens_start(id, size, camera.color, kept.calibration.get());
/// VKC_ASSIGN(solve::LensFit fit,
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
///        ready; for poses, once the session is (`SessionSummary::ready`)
///        and every camera has a real lens, calibrated or factory.
/// @param mode      The mode.
/// @param summary   The session's checks; empty before the first capture.
/// @param lenses    Each camera's lens readiness (@ref lens_states).
/// @param statuses  Each camera's @ref lens_status.
/// @return Whether it may run.
///
/// @code
/// const bool ready = session_ready(mode, captures->summary(),
///                                  captures->lenses(), statuses);
/// @endcode
bool session_ready(capture::SessionMode mode,
                   const std::optional<quality::SessionSummary>& summary,
                   const std::vector<quality::LensReadiness>& lenses,
                   const std::vector<LensStatus>& statuses) noexcept;

/// @brief Why a capture is refused in @p mode, in plain words: for poses,
///        while a camera's lens is missing, as the session's captures could
///        never be calibrated (a lens file loaded later starts a new
///        session): `bm-6 has no lens: load a lens file in Setup first`.
///        The live view still shows each camera through the stand-in.
/// @param mode      The mode.
/// @param cameras   Each camera's record, in rig order.
/// @param statuses  Each camera's @ref lens_status, in rig order.
/// @return The reason; empty when a capture may be taken. `InvalidArgument`
///         for @p statuses of another length than @p cameras; or
///         `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const std::string refused,
///            capture_refusal(mode, cameras, statuses));
/// if (refused.empty()) live->arm();
/// @endcode
core::Result<std::string> capture_refusal(
    capture::SessionMode mode,
    const std::vector<capture::SessionCamera>& cameras,
    const std::vector<LensStatus>& statuses);

/// @brief One camera that holds Calibrate back, and why, in plain words.
struct NotReady {
  std::string camera;  ///< Its id; empty for the session as a whole.
  std::string why;     ///< What it lacks, and what to do.
};

/// @brief Why Calibrate may not run yet, per camera, in plain words: what the
///        Calibration window says when it is pressed with nothing ready.
///
/// - Lenses, unless a camera is ready: each camera's shortfall
///   (`LensReadiness::unmet`), as `reach 0.83 toward the top-left corner,
///   needs 0.9: capture there`; before the first capture, `no captures yet`
///   for the session.
/// - Poses: each camera without a real lens (`Missing`), `no lens at
///   3840x2160 in <path>: load a lens file in Setup, or fit it in Lenses
///   mode`; then, before the first capture, `no captures yet` for the
///   session; a rig of one camera; and each camera not linked to the
///   reference (`no captures it shares with another camera: capture boards
///   it and another see`) or linked through fewer than `ready_from` shared
///   captures (`shares 3 captures with bm-1, needs 5: capture boards both
///   see`).
/// @param mode      The mode.
/// @param cameras   Each camera's record, in rig order.
/// @param statuses  Each camera's @ref lens_status, in rig order.
/// @param path      The lens file's path; empty for none.
/// @param summary   The session's checks; empty before the first capture.
/// @param lenses    Each camera's lens readiness; empty before the first
///                  capture and for poses.
/// @param options   The session's thresholds: `ready_from`.
/// @return What holds it back; empty once it may run. `InvalidArgument` for
///         lists of other lengths than @p cameras; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const std::vector<NotReady> why,
///            calibrate_blockers(mode, cameras, statuses, path, summary,
///                               lenses, setup.session));
/// @endcode
core::Result<std::vector<NotReady>> calibrate_blockers(
    capture::SessionMode mode,
    const std::vector<capture::SessionCamera>& cameras,
    const std::vector<LensStatus>& statuses, const std::string& path,
    const std::optional<quality::SessionSummary>& summary,
    const std::vector<quality::LensReadiness>& lenses,
    const quality::SessionOptions& options);

/// @brief Sets calib_rig's parts to @p scenario's mode, the one place the
///        window does, at the start and at each switch (`switch_session`):
///        the capture worker's session (`setup.mode`, the lens file as
///        `setup.calibration`, and the pairs and how their rows are checked,
///        `setup.pairs` and `setup.pair_rows`), the live view's capture rule
///        (@ref capture_rule), the window's mode (`RigMode::intrinsic`), and
///        the Board window's board and the boards it takes (@ref board_seed,
///        @ref board_rule). Each camera's lens is @ref take_lens's.
/// @param scenario  The scenario, checked (`check_scenario`).
/// @param setup     The capture worker's setup: its mode, file and pairs are
///                  set.
/// @param live      The live worker's options: its rule is set.
/// @param window    The window's mode: `intrinsic` is set.
/// @return The Board window's editor; or `OutOfMemory`.
///
/// @code
/// LiveOptions live;
/// RigMode mode;
/// VKC_ASSIGN(viz::BoardEditor editor,
///            apply_mode(scenario, setup, live, mode));
/// @endcode
core::Result<viz::BoardEditor> apply_mode(const RigScenario& scenario,
                                          CaptureSetup& setup,
                                          LiveOptions& live, RigMode& window);

}  // namespace volumetric_kit::calib::app
