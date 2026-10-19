// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file calibrate_worker.hpp
/// @brief calib_rig's Calibrate (its T4 thread): the rig solved from the
///        captures kept, measured, and its results directory written, off the
///        window's thread.
///
/// Each Calibrate works from scratch on one snapshot of the kept captures
/// (@ref CaptureWorker::kept), never from an earlier result: @ref calibrate
/// runs `solve::initialise`, `solve::bundle_adjust` and
/// `quality::calibration_metrics`, all in the reference camera's gauge, and
/// formats the results files (`quality::format_results`); @ref write_results
/// writes them to the session's `results/<RRR>_<YYYY-MM-DD_HHMMSS>/`.
/// @ref CalibrateWorker runs the two on a thread of its own and publishes
/// the result only once the files are on disk.
// TODO: CA12b's depth checks of the kept raw depth (CB6), on recon's device,
// and the world at the origin tag (CB8, CB9) join calibrate() here.

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "volumetric_kit/calib/app/capture_worker.hpp"
#include "volumetric_kit/calib/app/export.hpp"
#include "volumetric_kit/calib/quality/calibration_metrics.hpp"
#include "volumetric_kit/calib/quality/results.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::app {

/// @brief Where a Calibrate is.
enum class CalibrateStage {
  Solving,    ///< `initialise` and `bundle_adjust`.
  Measuring,  ///< `calibration_metrics`: leave one out, then the A/B test.
  Writing,    ///< The results directory; a cancel no longer stops it.
};

/// @brief The name the window gives @p stage.
/// @param stage  A stage.
/// @return `"solving"`, `"measuring"` or `"writing"`; `"unknown"` for a value
///         @ref CalibrateStage does not name.
///
/// @code
/// if (auto at = calibrate->progress()) show(to_string(at->stage));
/// @endcode
VC_APP_API const char* to_string(CalibrateStage stage) noexcept;

/// @brief How far a Calibrate has got.
///
/// @code
/// const CalibrateProgress at = *calibrate->progress();
/// if (at.total > 0) show(at.done, at.total);  // leave-one-out k of K
/// @endcode
struct CalibrateProgress {
  CalibrateStage stage = CalibrateStage::Solving;  ///< Where it is.
  /// Measuring: the leave-one-out re-solves done; otherwise 0.
  std::size_t done = 0;
  /// Measuring: the re-solves in all, once the first is done; otherwise 0.
  std::size_t total = 0;
  double seconds = 0.0;  ///< Since it began.
};

/// @brief Called on @ref calibrate's thread as it moves on: as it begins, as
///        it starts measuring and after each leave-one-out re-solve. A throw
///        stops the run (@ref calibrate).
using CalibrateCallback = std::function<void(const CalibrateProgress&)>;

/// @brief A solved and measured rig, and its results files.
///
/// Poses are in the reference camera's gauge: the world is its colour frame,
/// and its `color_to_world` is exactly the identity.
///
/// @code
/// VKC_ASSIGN(CalibrateResult result, calibrate(*kept));
/// for (std::size_t c = 0; c < result.ids.size(); ++c) {
///   place(result.ids[c], result.color_to_world[c]);
/// }
/// @endcode
struct CalibrateResult {
  /// The results directory's number, `RRR`; -1 until written
  /// (@ref write_results).
  int number = -1;
  std::string directory;  ///< The results directory; empty until written.
  std::vector<std::string> ids;  ///< Each camera's id, in rig order.
  std::size_t reference = 0;     ///< The reference camera, by index.
  std::size_t captures = 0;      ///< The kept captures it was solved from.
  solve::RigSolution solution;   ///< The adjustment's solution.
  quality::CalibrationMetrics metrics;  ///< Its metrics.
  /// Each camera's colour frame to the world, the inverse of its T_cw, in
  /// rig order: as recon and the 3D view take a pose.
  std::vector<recon::camera::Mat4d> color_to_world;
  /// Each solved board's T_wb, in @ref solve::RigSolution::boards' order.
  std::vector<recon::camera::Mat4d> boards_to_world;
  quality::ResultFiles files;  ///< The results files' text.
  double seconds = 0.0;        ///< How long the solve and metrics took.
};

/// @brief Solve and measure the rig from @p kept, from scratch, and format
///        its results files; writes nothing.
///
/// Every camera's view with corners in every kept capture goes to
/// `solve::initialise` (with `kept.solve`, from `kept.reference`), then
/// `solve::bundle_adjust` (its defaults) and `quality::calibration_metrics`
/// (both options, so the report records them), and
/// `quality::format_results`. Thread-free: it runs on the calling thread and
/// holds no state between calls, so the same snapshot gives the same result,
/// but for the times.
/// @param kept      The captures and what they were taken with.
/// @param progress  Called as the run moves on (@ref CalibrateCallback); may
///                  be empty. To stop the run it throws: the run then returns
///                  what it threw, as a `Status` (`compute::guarded`), and
///                  the caller tells its own stop from a failure.
/// @return The result, its @ref CalibrateResult::number -1;
///         `InvalidArgument` for no capture, more than `bundle_adjust` takes
///         (150), a null capture, one whose views are not one per camera or
///         name another camera or capture, or a refusal of the solve's (a
///         camera outside the tree, a link short of `min_shared`, naming them);
///         `Numerical` for a solve that fails; or another error of the
///         solve's, the metrics', the formatter's or @p progress's. Each
///         message starts `calibrate: `.
///
/// @code
/// VKC_ASSIGN(const CalibrateResult result, calibrate(*captures->kept()));
/// @endcode
VC_APP_API core::Result<CalibrateResult> calibrate(
    const KeptCaptures& kept, const CalibrateCallback& progress = {});

/// The session's subdirectory each Calibrate's results directory is in.
inline constexpr const char* kResultsDirectory = "results";

/// @brief A results directory's path:
///        `<session>/results/<RRR>_<YYYY-MM-DD_HHMMSS>`.
/// @param session  The session's directory.
/// @param number   `RRR`, from 0 to 999, zero-padded.
/// @param stamp    When Calibrate was pressed, in `capture::session_stamp`'s
///                 format.
/// @return The path; `InvalidArgument` for a number out of range, or what
///         `session_stamp` refuses; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const std::string dir,
///            results_directory(session, 0, std::chrono::system_clock::now()));
/// // <session>/results/000_2026-10-18_143000
/// @endcode
VC_APP_API core::Result<std::string> results_directory(
    const std::string& session, int number,
    std::chrono::system_clock::time_point stamp);

/// @brief Writes one whole results file; by default the core's
///        `write_file(path, bytes, WriteMode::CreateNew)`.
using ResultSink = std::function<core::Status(const std::string& path,
                                              std::string_view bytes)>;

/// @brief Where @ref write_results wrote.
struct WrittenResults {
  int number = -1;        ///< The directory's `RRR`.
  std::string directory;  ///< The directory (@ref results_directory).
};

/// @brief Write a Calibrate's results files to a new directory of the
///        session's.
///
/// 1. `<session>/results/` is created unless it is a directory already.
/// 2. The run's directory takes the first number from @p first_number that
///    no entry of `results/` starts with (`RRR_`), so no number is used
///    twice, and is created exclusively; a name taken meanwhile moves to the
///    next number.
/// 3. `residuals.csv`, then `report.json`, then `array_calibration.json`
///    last, each through @p sink: a directory left by a failure never holds a
///    calibration file.
/// @param session       The session's directory; it must exist.
/// @param first_number  The lowest number to take, from 0 to 999.
/// @param stamp         When Calibrate was pressed (@ref results_directory).
/// @param files         The files' text (@ref CalibrateResult::files).
/// @param sink          Writes each file; empty for the core's
///                      `write_file(CreateNew)`.
/// @return Where it wrote; `NotFound` when @p session does not exist;
///         `InvalidArgument` for an empty @p session, when `results/` is not
///         a directory, or for a number out of range, or none left below
///         1000; or the failure of the directory's creation or of a file's
///         write, naming it, with the files after it unwritten. Each message
///         starts `write_results: `.
///
/// @code
/// VKC_ASSIGN(const WrittenResults written,
///            write_results(kept->directory, 0, pressed, result.files));
/// @endcode
VC_APP_API core::Result<WrittenResults> write_results(
    const std::string& session, int first_number,
    std::chrono::system_clock::time_point stamp,
    const quality::ResultFiles& files, const ResultSink& sink = {});

/// @brief The Calibrate worker: a thread that runs @ref calibrate and
///        @ref write_results on each snapshot it is given, one at a time.
///
/// Every member may be called from any thread. A run's outcome is a result
/// (@ref take_result), published only once its files are on disk, or a
/// failure (@ref take_errors), which leaves the results written before and
/// writes nothing, or nothing at all when it was cancelled. The numbers go up
/// from 0, one per directory written.
///
/// @code
/// VKC_ASSIGN(std::unique_ptr<CalibrateWorker> calibrate,
///            CalibrateWorker::create());
/// // The Calibrate button:
/// VKC_ASSIGN(const bool started, calibrate->request(captures->kept()));
/// // Each frame of the window:
/// if (auto at = calibrate->progress()) draw_progress(*at);
/// if (auto result = calibrate->take_result()) place_cameras(*result);
/// for (const core::Status& e : calibrate->take_errors()) show(e);
/// @endcode
// TODO: CB6 gives the worker recon's device for the depth checks; calib_rig
// then stops it before that device goes.
class VC_APP_API CalibrateWorker final {
 public:
  /// @brief Start the thread, waiting for a request.
  /// @param sink  Writes each results file (@ref write_results); empty for
  ///              the core's `write_file(CreateNew)`.
  /// @return The worker; `IoError` if the thread cannot start; or
  ///         `OutOfMemory`.
  static core::Result<std::unique_ptr<CalibrateWorker>> create(
      ResultSink sink = {});

  /// @brief As @ref stop.
  ~CalibrateWorker();
  CalibrateWorker(const CalibrateWorker&) = delete;
  CalibrateWorker& operator=(const CalibrateWorker&) = delete;
  CalibrateWorker(CalibrateWorker&&) = delete;
  CalibrateWorker& operator=(CalibrateWorker&&) = delete;

  /// @brief Calibrate from @p kept: the press's time names its results
  ///        directory.
  /// @param kept     The snapshot (@ref CaptureWorker::kept), held until the
  ///                 run ends.
  /// @param pressed  When Calibrate was pressed (@ref results_directory); by
  ///                 default, now.
  /// @return `true` once the run is started; `false`, starting nothing,
  ///         while a run is @ref busy or the worker is stopped;
  ///         `InvalidArgument` for a null snapshot; or `OutOfMemory`.
  core::Result<bool> request(std::shared_ptr<const KeptCaptures> kept,
                             std::chrono::system_clock::time_point pressed =
                                 std::chrono::system_clock::now());

  /// @return Whether a run is under way; once it is not, its outcome is
  ///         there to take.
  bool busy() const;

  /// @return How far the run has got, its seconds counted to now; empty
  ///         when none is @ref busy.
  std::optional<CalibrateProgress> progress() const;

  /// @brief Cancel the run, unless it is writing: it stops at its next step
  ///        (@ref CalibrateCallback) and publishes, writes and reports
  ///        nothing.
  /// @return Whether a run was cancelled; `false` when none is busy or it is
  ///         already writing.
  bool cancel();

  /// @brief Take the newest result not yet taken.
  /// @return It, with its files on disk; null when none was published since
  ///         the last call.
  std::shared_ptr<const CalibrateResult> take_result();

  /// @brief Take what went wrong since the last call: each run's refusal or
  ///        failure, naming its cause, and why the thread ended.
  /// @return The failures, oldest first, each in context `Calibrate`.
  std::vector<core::Status> take_errors();

  /// @brief Cancel the run (@ref cancel), wait for it, and stop the thread.
  ///        A run already writing finishes first, its outcome there to
  ///        take. Idempotent.
  void stop() noexcept;

 private:
  struct State;
  explicit CalibrateWorker(std::unique_ptr<State> state);
  void run() noexcept;

  std::unique_ptr<State> state_;
};

}  // namespace volumetric_kit::calib::app
