// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file panels.hpp
/// @brief The live view's panels: what each camera sees, how the latest set
///        arrived, what the captures kept add up to and how much of the
///        origin tag they hold, where a capture stands, and the stage
///        timings; and Calibrate's: how far it has got, and what its result
///        is worth.
///
/// Each panel is a model -- lines of text, each in a @ref Tone -- built from
/// host data alone (the quality tier's checks and metrics, recon's counters,
/// the capture records), and drawn with ImGui by @ref draw_panel_lines and
/// @ref draw_session_panel. The thresholds behind each tone are the quality
/// tier's own (@ref PanelOptions).

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "volumetric_kit/calib/core/capture_meta.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/core/view_check.hpp"
#include "volumetric_kit/calib/quality/calibration_metrics.hpp"
#include "volumetric_kit/calib/quality/capture_rule.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"
#include "volumetric_kit/calib/solve/spanning_tree.hpp"
#include "volumetric_kit/calib/viz/export.hpp"
#include "volumetric_kit/calib/viz/tone.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/gfx/core/frame_metrics.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"

namespace volumetric_kit::calib::viz {

/// @brief One line of a panel: a label, and a value in its tone.
///
/// @code
/// const PanelLine line{"tilt", "64.2 deg", Tone::Warn};
/// @endcode
struct PanelLine {
  std::string label;         ///< What the value is.
  std::string value;         ///< The value, as shown.
  Tone tone = Tone::Normal;  ///< How it is coloured.
};

/// @brief The thresholds the panels' tones follow: the quality tier's, so the
///        panels judge as the checks and the capture rule do.
///
/// @code
/// PanelOptions options;
/// options.session = session_options;  // as SessionChecks::create took
/// options.rule = rule_options;        // as CaptureRule::create took
/// @endcode
struct PanelOptions {
  /// The view, solve and session thresholds: tilt (`view.oblique_deg`,
  /// `solve.oblique_deg`), the tree's link strengths and `ready_from`.
  quality::SessionOptions session;
  /// The capture rule's: `still_px` and `spread_warn_ms`.
  quality::CaptureRuleOptions rule;
};

/// @brief What the live view knows of one camera in the latest set.
///
/// @code
/// CameraLive camera;
/// camera.present = set.frames[i].has_value();
/// camera.check = views[i].check;           // quality::check_view's
/// camera.stillness_px = shift.cameras[i];  // quality::corner_shift's
/// camera.frames = array_stats.sensors[i];
/// camera.lost = orbbec_stats.lost;         // where the driver counts it
/// @endcode
struct CameraLive {
  bool present = false;  ///< The set has its frame.
  ViewCheck check;       ///< Its view's live check.
  /// How far its corners moved since the set before (pixels); empty when no
  /// corner is in both.
  std::optional<double> stillness_px;
  recon::sensor::SensorStats frames;  ///< Its frame counters.
  /// The frames its driver lost; empty where the driver does not count them.
  std::optional<std::uint64_t> lost;
};

/// @brief One camera's panel.
///
/// Lines, in order: `verdict` (or `no frame`, red); `corners` and `markers`
/// found of the board's; `distance`, `tilt` (amber above
/// `session.view.oblique_deg`, red above `session.solve.oblique_deg`) and
/// `fit` from the view's own pose; `cell`, where the board's centre is in a
/// 3 x 3 grid; `stillness` (green below `rule.still_px`); `views` usable and
/// used by the solve; `coverage` filled; and `frames`, amber once any failed
/// or was lost.
/// @param camera   The camera in the latest set.
/// @param session  Its part of the session.
/// @param board    The board, for its corners and markers.
/// @param options  The thresholds.
/// @return The lines; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const std::vector<PanelLine> lines,
///            camera_panel(camera, summary.cameras[i], board, options));
/// @endcode
VC_VIZ_API core::Result<std::vector<PanelLine>> camera_panel(
    const CameraLive& camera, const quality::CameraSummary& session,
    const Target& board, const PanelOptions& options = {});

/// @brief How full the session writer is.
///
/// @code
/// const std::size_t pending = writer->pending();
/// const bool full = pending >= writer_options.queue_capacity;
/// live_set.saver_full = full;  // the capture rule judges the same
/// const SaverQueue saver{pending, writer_options.queue_capacity, full};
/// @endcode
struct SaverQueue {
  std::size_t pending = 0;   ///< Captures queued or being written.
  std::size_t capacity = 0;  ///< The most it holds.
  // TODO: the writer's own full() once the capture tier gives SessionWriter
  // one (quality::LiveSet::saver_full's TODO).
  /// It holds as many as it may: what the caller gives the capture rule as
  /// @ref quality::LiveSet::saver_full.
  bool full = false;
};

/// @brief The latest set's panel.
///
/// Lines, in order: `set`, the frames it holds (amber when incomplete);
/// `spread` (amber above `rule.spread_warn_ms`); `skew <id>` for each camera,
/// its frame's time less the trigger's; `sets`, the array's counters; and
/// `saver`, the writer's queue (amber when `saver.full`).
/// @param set      The latest set; one with no frames before the first.
/// @param stats    The array's counters.
/// @param ids      Each camera's id, in the set's order.
/// @param saver    The writer's queue.
/// @param options  The thresholds.
/// @return The lines; `InvalidArgument` when @p ids and the set's frames
///         differ in number; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const std::vector<PanelLine> lines,
///            set_panel(*snapshot.set, array_stats, serials, saver));
/// @endcode
VC_VIZ_API core::Result<std::vector<PanelLine>> set_panel(
    const recon::sensor::FrameSet& set,
    const recon::sensor::SensorArrayStats& stats,
    const std::vector<std::string>& ids, const SaverQueue& saver,
    const PanelOptions& options = {});

/// @brief The session's panel: lines, and the captures each pair of cameras
///        shares as a matrix.
///
/// @code
/// VKC_ASSIGN(const SessionPanel panel, session_panel(summary, serials));
/// calibrate_button.enabled = panel.calibrate;
/// draw_session_panel(panel);
/// @endcode
struct SessionPanel {
  /// `captures` kept and dropped; then each camera's place in the tree
  /// preview: `reference`, its link `from <parent>` with the captures shared
  /// (green when strong, amber when fair, red when weak), or `not linked`
  /// (red); then a `warning` for each repeated capture, floor-only camera and
  /// mostly frontal camera; and `calibrate`.
  std::vector<PanelLine> lines;
  std::vector<std::string> ids;  ///< The matrix's rows and columns.
  /// The captures each pair shares with a view the solve uses in both
  /// (@ref quality::SessionSummary::covisibility).
  solve::Covisibility shared;
  /// Each count's tone: a tree link's strength, `Muted` on the diagonal and
  /// for none, `Normal` otherwise.
  std::vector<std::vector<Tone>> shared_tones;
  bool calibrate = false;  ///< Calibrate may run.
};

/// @brief The session's panel.
/// @param summary  What the captures kept add up to.
/// @param ids      Each camera's id, in the summary's order.
/// @param options  The thresholds: `session.ready_from` names Calibrate's
///                 rule.
/// @return The panel; `InvalidArgument` for ids, cameras and a matrix that
///         do not agree in number, or a link to a camera outside the rig; or
///         `OutOfMemory`.
VC_VIZ_API core::Result<SessionPanel> session_panel(
    const quality::SessionSummary& summary, const std::vector<std::string>& ids,
    const PanelOptions& options = {});

/// @brief The origin tag in the captures kept: the cameras that saw it, and
///        how often.
///
/// @code
/// TagCoverage coverage;
/// coverage.cameras = 4;  // the rig's
/// coverage.seen_by = 2;  // cameras that saw it in a capture kept
/// coverage.views = 9;    // their captures kept that saw it
/// @endcode
struct TagCoverage {
  std::size_t cameras = 0;  ///< The rig's cameras.
  std::size_t seen_by = 0;  ///< Those that saw it in a capture kept.
  /// Its views: for each camera, the captures kept that saw it, summed.
  std::size_t views = 0;
};

/// @brief The session panel's line for the origin tag.
///
/// `origin tag`: `seen by <k> of <N> cameras, <V> views`, green at 2
/// cameras or more; amber at 1, adding that the reference camera will be the
/// origin unless a second camera sees it; `seen by 0 of <N> cameras`, muted,
/// before any.
/// @param coverage  The tag in the captures kept.
/// @return The line; `InvalidArgument` for more cameras seeing it than the
///         rig has, or fewer views than cameras seeing it; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const PanelLine line, origin_tag_line(coverage));
/// panel.lines.push_back(line);
/// @endcode
VC_VIZ_API core::Result<PanelLine> origin_tag_line(const TagCoverage& coverage);

/// @brief Where a capture stands: the rule's latest answer and the last
///        capture written.
///
/// @code
/// CaptureStatus status;
/// status.decision = decision;  // the rule's latest answer that is not Idle
/// status.last = meta;          // the last capture written (CA9's record)
/// status.last_usable_views = usable;
/// status.last_tag_cameras = tag_cameras;
/// status.last_flagged = quality::live_vs_full_flagged(shift);
/// @endcode
struct CaptureStatus {
  quality::CaptureDecision decision;  ///< The capture rule's latest answer.
  std::optional<CaptureMeta> last;    ///< The last capture written.
  int last_usable_views = 0;          ///< Its usable views.
  int last_tag_cameras = 0;  ///< Its cameras that found the origin tag.
  /// Its live corners disagree with its full ones
  /// (@ref quality::live_vs_full_flagged).
  bool last_flagged = false;
  std::string error;  ///< Why the last write failed; empty when none did.
};

/// @brief The capture's panel.
///
/// Lines, in order: `capture`, the rule's state -- `idle`, `waiting: <why>`
/// (amber), `taken` (green), with a `warning` line for its warnings, or
/// `failed: <why>` (red); `last`, the last capture written, with its usable
/// views and the cameras that found the origin tag, and its `live vs full`
/// (amber when flagged); and `save`, a write's failure (red).
/// @param status  Where the capture stands.
/// @return The lines; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const std::vector<PanelLine> lines, capture_panel(status));
/// @endcode
VC_VIZ_API core::Result<std::vector<PanelLine>> capture_panel(
    const CaptureStatus& status);

/// @brief The line that stands for the Calibrate button while Calibrate
///        runs.
///
/// `calibrating`: `<stage> <done>/<total>, <seconds> s`, the count left out
/// while @p total is 0.
/// @param stage    Where it is, as the worker names it (`"measuring"`).
/// @param done     The steps of the stage done, such as the leave-one-out
///                 re-solves.
/// @param total    The stage's steps in all; 0 while unknown or none.
/// @param seconds  Since it began.
/// @return The line; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const PanelLine line,
///            calibrate_progress_line(to_string(at->stage), at->done,
///                                    at->total, at->seconds));
/// @endcode
VC_VIZ_API core::Result<PanelLine> calibrate_progress_line(
    std::string_view stage, std::size_t done, std::size_t total,
    double seconds);

/// @brief The Calibration window's panel: what a Calibrate's result is
///        worth.
///
/// Lines, in order; a figure that is not a number shows as `-`:
/// - `calibration`: its results number, the captures solved and the
///   seconds the solve and metrics took;
/// - `RMS`: every corner's reprojection RMS at the first rig and solved
///   (`initial -> solved`), and their legacy RMSE;
/// - `RMS <id>` for each camera: its own, initial and solved;
/// - `transfer`: the cross-camera transfer's median and 90th percentile;
/// - `moves <id>` for each camera: the most its centre and its rotation
///   moved with one capture left out, each with that capture; `-`, muted,
///   when none moved it;
/// - `principal point`: the A/B verdict (`as given` green, `shifted` amber,
///   `inconclusive`), the gap (`quality::principal_point_gap`) and each
///   side's RMS; `refused: <why>`, muted, when the shifted rig could not be
///   solved;
/// - `centre <id>` for each camera: its centre (metres) in the world, the
///   reference camera's colour frame;
/// - `results`: the results directory.
/// @param ids        Each camera's id, in the rig's order.
/// @param metrics    The result's metrics (`quality::calibration_metrics`).
/// @param number     Its results directory's number, `RRR`.
/// @param directory  Its results directory.
/// @param captures   The kept captures it was solved from.
/// @param seconds    How long the solve and metrics took.
/// @return The lines; `InvalidArgument` when the metrics' cameras, per
///         camera in the reprojection, the leave-one-out or the geometry,
///         are not as many as @p ids; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const std::vector<PanelLine> lines,
///            calibration_panel(result.ids, result.metrics, result.number,
///                              result.directory, result.captures,
///                              result.seconds));
/// draw_panel_lines("calibration", lines);
/// @endcode
// TODO: CB6 adds the depth checks' lines, and CB9 the world's line and each
// camera's height in the origin tag's world.
VC_VIZ_API core::Result<std::vector<PanelLine>> calibration_panel(
    const std::vector<std::string>& ids,
    const quality::CalibrationMetrics& metrics, int number,
    const std::string& directory, std::size_t captures, double seconds);

/// @brief The live stages' timings as gfx's metrics panel takes them.
/// @param stages  The stages timed (poll, readback, detect, tiles, ...).
/// @param fps     The live rate, in sets per second.
/// @return The metrics: the stages' rows, their host total less breakdowns
///         (`StageMetrics::total_cpu_ms`) as the frame's time, and @p fps;
///         or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const gfx::FrameMetrics metrics,
///            stage_frame_metrics(stages, rate));
/// gfx::ui::draw_metrics_panel(metrics, "Stages");
/// @endcode
VC_VIZ_API core::Result<gfx::FrameMetrics> stage_frame_metrics(
    const core::StageMetrics& stages, double fps);

/// @brief Draw lines as a two-column ImGui table, each value in its tone.
/// @param id     The table's ImGui id, unique in its window.
/// @param lines  The lines.
/// @pre An ImGui frame is active, with its context current.
///
/// @code
/// ImGui::Begin(serial.c_str());
/// draw_panel_lines("camera", lines);
/// ImGui::End();
/// @endcode
VC_VIZ_API void draw_panel_lines(const char* id,
                                 const std::vector<PanelLine>& lines);

/// @brief Draw the session's panel: its lines, then the shared captures as a
///        matrix.
/// @param panel  The panel.
/// @pre An ImGui frame is active, with its context current.
VC_VIZ_API void draw_session_panel(const SessionPanel& panel);

}  // namespace volumetric_kit::calib::viz
