// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file rig_setup.hpp
/// @brief calib_rig's Setup panel: the mode, Lenses or Poses, and the lens
///        file each camera's lens comes from, chosen in the app; a change
///        starts a new session with the cameras left open
///        (@ref switch_session).
///
/// The scenario's `operation` and `source.calibration` are where it starts.
/// A lens file comes from a lens run's results under `output.path`
/// (@ref find_lens_results), a typed path (@ref load_lens_file), or the lens
/// run just shown (@ref passed_lenses, "Use these lenses").

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/calib/app/calib_mode.hpp"
#include "volumetric_kit/calib/app/calibrate_worker.hpp"
#include "volumetric_kit/calib/app/capture_worker.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/app/open_source.hpp"
#include "volumetric_kit/calib/app/rig_mode.hpp"
#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/viz/board_editor.hpp"
#include "volumetric_kit/calib/viz/panels.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"

namespace volumetric_kit::calib::app {

/// @brief A lens run's results directory under the session root: its lens
///        file, and what its report says of each camera.
///
/// @code
/// for (const LensResult& found : find_lens_results(root).value()) {
///   list(lens_result_label(found), found.path);
/// }
/// @endcode
struct LensResult {
  std::string path;     ///< Its `array_calibration.json`.
  std::string session;  ///< Its session's directory name.
  /// Its results directory's name, `<RRR>_<YYYY-MM-DD_HHMMSS>`: the number
  /// and when Calibrate was pressed.
  std::string results;
  /// Each camera the run fitted and wrote, and whether its lens passed, in
  /// the report's order.
  std::vector<std::pair<std::string, bool>> cameras;
};

/// @brief Every lens run's results under @p root: each
///        `<root>/<session>/results/<RRR>_<stamp>/` holding
///        `array_calibration.json` and a `report.json` with a `lenses`
///        section, newest first (by its stamp, then its session, then its
///        number). A directory that cannot be read, or whose report does not
///        parse, is left out.
/// @param root  The session root (`output.path`).
/// @return The runs; `NotFound` when @p root is not a directory; or
///         `OutOfMemory`.
core::Result<std::vector<LensResult>> find_lens_results(
    const std::string& root);

/// @brief A lens run as the picker lists it: when, each camera's verdict,
///        and where, as `2026-11-02 23:57:12  bm-5 pass, bm-6 fail
///        (2026-11-02_215407/000)`.
/// @param result  The run.
/// @return The line; or `OutOfMemory`.
core::Result<std::string> lens_result_label(const LensResult& result);

/// @brief A lens file read and checked against the rig, and what to say of
///        it.
///
/// @code
/// VKC_ASSIGN(const LensFile file, load_lens_file(path, setup.cameras));
/// for (const std::string& note : file.notes) say(note);
/// next.calibration = file.calibration;
/// next.calibration_path = file.path;
/// @endcode
struct LensFile {
  /// The file's lenses: those the rig takes.
  std::shared_ptr<const sensor::camera::ArrayCalibration> calibration;
  std::string path;  ///< Where it was read.
  /// What the window says beside it: each camera it gives no lens, and,
  /// for @ref passed_lenses, each camera whose lens was not taken, and why.
  std::vector<std::string> notes;
};

/// @brief Read @p path as a lens file for the rig's @p cameras, checking its
///        cameras and sizes.
/// @param path     The file: sensor's calibration file.
/// @param cameras  The rig's cameras' records, in rig order.
/// @return The file, its notes naming each camera it has no lens for;
///         `InvalidArgument` saying what is wrong, each message
///         `lens file <path>: ...`: a lens of one of @p cameras at another
///         size than its pictures ("bm-5's lens is 1920x1080, its pictures
///         3840x2160"), or no lens for any of them ("no lens for bm-5 or
///         bm-6: it holds bm-1, bm-2"); `read_array_calibration`'s refusal;
///         or `OutOfMemory`.
core::Result<LensFile> load_lens_file(
    const std::string& path,
    const std::vector<capture::SessionCamera>& cameras);

/// @brief "Use these lenses": the lens file a lens run wrote, as
///        @ref load_lens_file checks it, with the lenses of the cameras that
///        passed alone; the notes say which others were not taken, and why.
/// @param result   The lens run, written (@ref CalibrateResult::directory).
/// @param cameras  The rig's cameras' records, in rig order.
/// @return The lenses; `InvalidArgument` for a result that is not a written
///         lens run, a run that wrote no lens file, or no camera that passed
///         (each saying so); what @ref load_lens_file refuses; or
///         `OutOfMemory`.
core::Result<LensFile> passed_lenses(
    const CalibrateResult& result,
    const std::vector<capture::SessionCamera>& cameras);

/// @brief The Setup panel's lens lines: `lens file`, the path or `none`;
///        then `lens <id>` for each camera, its @ref lens_status, with
///        ` (<file>)` for a file's: `calibrated` green, `factory` and
///        `stand-in` plain, `missing` red with `load a lens file`.
/// @param mode     The mode.
/// @param cameras  The cameras' records, in rig order.
/// @param file     The lens file; null for none.
/// @param path     Its path; empty for none.
/// @return The lines; or `OutOfMemory`.
core::Result<std::vector<viz::PanelLine>> setup_lines(
    capture::SessionMode mode,
    const std::vector<capture::SessionCamera>& cameras,
    const sensor::camera::ArrayCalibration* file, const std::string& path);

/// @brief Start a new session of @p next's mode and lens file, the cameras
///        left open: the switch the Setup panel makes.
///
/// 1. Each camera's lens by @ref take_lens for @p next, and the window's
///    parts by @ref apply_mode; the window back to the images, uncalibrated.
///    The Board window's board is @p board, or the mode's seed.
/// 2. The live worker restarts first (`LiveWorker::restart`): from then on
///    it hands no capture on until it takes the new board, lenses and rule.
/// 3. Then the capture worker (`CaptureWorker::restart`): what was queued
///    before goes to the session it closes, whose captures stay on disk; the
///    next capture opens the new session.
///
/// Nothing changes when step 1 or 2 is refused.
/// @param next          The mode, the lens file (`calibration`,
///                      `calibration_path`) and the pairs.
/// @param board         The board to search for, as the mode last had it,
///                      and whether its square was measured; empty for the
///                      mode's seed.
/// @param setup         The capture worker's setup, updated.
/// @param live_options  The live worker's options, updated.
/// @param window        The window's mode, updated.
/// @param live          The live worker.
/// @param captures      The capture worker.
/// @return The Board window's editor, unlocked; what @ref take_lens,
///         `viz::BoardEditor::edit`, `LiveWorker::restart` or
///         `CaptureWorker::restart` refuses; or `OutOfMemory`.
///
/// @code
/// RigScenario next = now;
/// next.mode = capture::SessionMode::Extrinsics;
/// VKC_ASSIGN(editor, switch_session(next, std::nullopt, setup, live_options,
///                                   mode, *live, *captures));
/// @endcode
core::Result<viz::BoardEditor> switch_session(
    const RigScenario& next,
    const std::optional<std::pair<Target, bool>>& board, CaptureSetup& setup,
    LiveOptions& live_options, RigMode& window, LiveWorker& live,
    CaptureWorker& captures);

}  // namespace volumetric_kit::calib::app
