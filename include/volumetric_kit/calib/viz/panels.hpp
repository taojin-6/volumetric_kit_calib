// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file panels.hpp
/// @brief The live view's panels: what each camera sees, how the latest set
///        arrived, what the captures kept add up to, where a capture stands,
///        and the stage timings.
///
/// Each panel is a model -- lines of text, each in a @ref Tone -- built from
/// host data alone (the quality tier's checks, recon's counters, the capture
/// records), and drawn with ImGui by @ref draw_panel_lines and
/// @ref draw_session_panel. The thresholds behind each tone are the quality
/// tier's own (@ref PanelOptions).

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/core/capture_meta.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/core/view_check.hpp"
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

/// @brief Where a capture stands: the rule's latest answer and the last
///        capture written.
///
/// @code
/// CaptureStatus status;
/// status.decision = decision;  // the rule's latest answer that is not Idle
/// status.last = meta;          // the last capture written (CA9's record)
/// status.last_usable_views = usable;
/// status.last_flagged = quality::live_vs_full_flagged(shift);
/// @endcode
struct CaptureStatus {
  quality::CaptureDecision decision;  ///< The capture rule's latest answer.
  std::optional<CaptureMeta> last;    ///< The last capture written.
  int last_usable_views = 0;          ///< Its usable views.
  /// Its live corners disagree with its full ones
  /// (@ref quality::live_vs_full_flagged).
  bool last_flagged = false;
  std::string error;  ///< Why the last write failed; empty when none did.
};

/// @brief The capture's panel.
///
/// Lines, in order: `capture`, the rule's state -- `idle`, `waiting: <why>`
/// (amber), `taken` (green), with a `warning` line for its warnings, or
/// `failed: <why>` (red); `last`, the last capture written, and its
/// `live vs full` (amber when flagged); and `save`, a write's failure (red).
/// @param status  Where the capture stands.
/// @return The lines; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const std::vector<PanelLine> lines, capture_panel(status));
/// @endcode
VC_VIZ_API core::Result<std::vector<PanelLine>> capture_panel(
    const CaptureStatus& status);

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
