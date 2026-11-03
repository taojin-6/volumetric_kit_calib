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
/// A lens file comes from a lens run, one listed under `output.path`
/// (@ref find_lens_results) or the one just shown ("Use these lenses"),
/// through @ref lens_run_file alike; or from a typed path
/// (@ref load_lens_file).

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

/// @brief A lens run's results directory: its lens file, and what its report
///        says of each camera.
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
  /// and when Calibrate was pressed (`results_name`).
  std::string results;
  /// Each camera the run fitted and wrote, and whether its lens passed, in
  /// the report's order.
  std::vector<std::pair<std::string, bool>> cameras;
};

/// @brief Read a lens run's results directory, as Calibrate wrote it.
/// @param directory  `<session>/results/<RRR>_<stamp>`
///                   (`CalibrateResult::directory`).
/// @return The run; `NotFound` without its `array_calibration.json` or
///         `report.json`; `InvalidArgument` for a name that is not a
///         results directory's, or a report that is not a lens run's
///         (calib's `format`, a `lenses` section, each camera's `written`
///         and `verdict`), naming the file; or what reading it fails with.
core::Result<LensResult> read_lens_run(const std::string& directory);

/// @brief Every lens run's results under @p root: each
///        `<root>/<session>/results/<RRR>_<stamp>/` that @ref read_lens_run
///        reads, newest first (by its stamp, then its session, then its
///        number). A directory it refuses is left out.
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
  /// What the window says beside it: each camera it gives no lens, and why,
  /// in rig order.
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

/// @brief The lenses a lens run gives the next session, picked in the Setup
///        list or by "Use these lenses" alike: its lens file, as
///        @ref load_lens_file checks it, without the lenses that failed in
///        the run. So each camera takes its lens that passed, or the lens
///        the run carried from the file its session started from when it did
///        not fit the camera (`quality::format_lens_results`); a camera
///        whose lens failed takes none from it (`bm-6's lens failed its gates
///        in that run: not taken`).
/// @param run      The run (@ref read_lens_run).
/// @param cameras  The rig's cameras' records, in rig order.
/// @return The lenses; `InvalidArgument`, as `lens file <path>: ...`, when no
///         rig camera's lens passed in it and none is carried; what
///         @ref load_lens_file refuses; or `OutOfMemory`.
core::Result<LensFile> lens_run_file(
    const LensResult& run, const std::vector<capture::SessionCamera>& cameras);

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
///    next capture opens the new session. It is queued before the caller's
///    next `LiveWorker::arm`, which a capture of the new session needs.
///
/// Nothing changes when step 1, the capture worker's check of the next
/// setup (`CaptureWorker::accepts`) or step 2 is refused. Step 3 then fails
/// only once the capture worker's thread has ended, the live worker on the
/// next session.
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
///         `viz::BoardEditor::edit`, `CaptureWorker::accepts`,
///         `LiveWorker::restart` or `CaptureWorker::restart` refuses; or
///         `OutOfMemory`.
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
