// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/viz/panels.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <utility>

#include "imgui.h"
#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::viz {
namespace {

// The 3 x 3 cells of ViewCheck::cell, row-major from the top left.
constexpr std::array<const char*, 9> kCells = {
    "top left",    "top",    "top right",  //
    "left",        "centre", "right",      //
    "bottom left", "bottom", "bottom right"};

// `value` with `decimals` places, `+` signed when `sign`.
std::string fixed(double value, int decimals, bool sign = false) {
  char buffer[64];
  const int n = std::snprintf(buffer, sizeof buffer, sign ? "%+.*f" : "%.*f",
                              decimals, value);
  return std::string(
      buffer,
      n < 0 ? 0 : std::min(static_cast<std::size_t>(n), sizeof buffer - 1));
}

// A measure from a view's pose: its value and unit, or `-` (muted) when the
// view has none.
PanelLine measure(const char* label, const std::optional<double>& value,
                  int decimals, const char* unit) {
  if (!value) return {label, "-", Tone::Muted};
  return {label, fixed(*value, decimals) + unit, Tone::Normal};
}

Tone count_tone(std::int64_t count) noexcept {
  return count > 0 ? Tone::Normal : Tone::Muted;
}

Tone strength_tone(quality::LinkStrength strength) noexcept {
  switch (strength) {
    case quality::LinkStrength::Strong:
      return Tone::Good;
    case quality::LinkStrength::Fair:
      return Tone::Warn;
    case quality::LinkStrength::Weak:
      break;
  }
  return Tone::Bad;
}

}  // namespace

core::Result<std::vector<PanelLine>> camera_panel(
    const CameraLive& camera, const quality::CameraSummary& session,
    const Target& board, const PanelOptions& options) {
  return compute::guarded(
      "camera_panel", [&]() -> core::Result<std::vector<PanelLine>> {
        const ViewCheck& check = camera.check;
        std::vector<PanelLine> lines;
        if (camera.present) {
          lines.push_back({"verdict", to_string(check.verdict),
                           verdict_tone(check, options.session)});
        } else {
          lines.push_back({"verdict", "no frame", Tone::Bad});
        }
        lines.push_back(
            {"corners",
             std::to_string(check.corners) + "/" +
                 std::to_string(std::int64_t{board.rows} * board.cols),
             count_tone(check.corners)});
        lines.push_back({"markers",
                         std::to_string(check.markers) + "/" +
                             std::to_string(marker_count(board)),
                         count_tone(check.markers)});
        lines.push_back(measure("distance", check.distance_m, 2, " m"));
        PanelLine tilt = measure("tilt", check.tilt_deg, 1, " deg");
        if (check.tilt_deg &&
            *check.tilt_deg > options.session.solve.oblique_deg) {
          tilt.tone = Tone::Bad;
        } else if (check.tilt_deg &&
                   *check.tilt_deg > options.session.view.oblique_deg) {
          tilt.tone = Tone::Warn;
        }
        lines.push_back(std::move(tilt));
        lines.push_back(measure("fit", check.pnp_rms_px, 2, " px"));
        if (check.cell && *check.cell >= 0 &&
            *check.cell < static_cast<int>(kCells.size())) {
          lines.push_back({"cell",
                           kCells[static_cast<std::size_t>(*check.cell)],
                           Tone::Normal});
        } else {
          lines.push_back({"cell", "-", Tone::Muted});
        }
        PanelLine still = measure("stillness", camera.stillness_px, 2, " px");
        if (camera.stillness_px) {
          still.tone = *camera.stillness_px < options.rule.still_px
                           ? Tone::Good
                           : Tone::Warn;
        }
        lines.push_back(std::move(still));
        lines.push_back({"views",
                         std::to_string(session.usable_views) + " usable, " +
                             std::to_string(session.used_views) + " used",
                         count_tone(session.usable_views)});
        lines.push_back(
            {"coverage", fixed(100.0 * session.coverage_filled, 0) + " %",
             session.coverage_filled > 0.0 ? Tone::Normal : Tone::Muted});
        const recon::sensor::SensorStats& frames = camera.frames;
        std::string counts = std::to_string(frames.received) + " received, " +
                             std::to_string(frames.delivered) + " delivered, " +
                             std::to_string(frames.dropped) + " dropped, " +
                             std::to_string(frames.failed) + " failed";
        if (camera.lost)
          counts += ", " + std::to_string(*camera.lost) + " lost";
        const bool lossy =
            frames.failed > 0 || (camera.lost && *camera.lost > 0);
        lines.push_back(
            {"frames", std::move(counts), lossy ? Tone::Warn : Tone::Normal});
        return lines;
      });
}

core::Result<std::vector<PanelLine>> set_panel(
    const recon::sensor::FrameSet& set,
    const recon::sensor::SensorArrayStats& stats,
    const std::vector<std::string>& ids, const SaverQueue& saver,
    const PanelOptions& options) {
  return compute::guarded(
      "set_panel", [&]() -> core::Result<std::vector<PanelLine>> {
        if (ids.size() != set.frames.size()) {
          return core::Status::invalid_argument(
              "set_panel: " + std::to_string(ids.size()) +
              " ids for a set of " + std::to_string(set.frames.size()) +
              " cameras");
        }
        std::vector<PanelLine> lines;
        if (set.frames.empty()) {
          lines.push_back({"set", "none yet", Tone::Muted});
          lines.push_back({"spread", "-", Tone::Muted});
        } else {
          lines.push_back({"set",
                           std::to_string(set.count()) + "/" +
                               std::to_string(set.frames.size()) + " frames",
                           set.complete() ? Tone::Good : Tone::Warn});
          const double spread_ms = static_cast<double>(set.spread_ns()) / 1e6;
          lines.push_back({"spread", fixed(spread_ms, 2) + " ms",
                           spread_ms > options.rule.spread_warn_ms
                               ? Tone::Warn
                               : Tone::Normal});
        }
        for (std::size_t i = 0; i < ids.size(); ++i) {
          const std::optional<std::int64_t> skew = set.skew_ns(i);
          if (skew) {
            lines.push_back(
                {"skew " + ids[i],
                 fixed(static_cast<double>(*skew) / 1e6, 2, true) + " ms",
                 Tone::Normal});
          } else {
            lines.push_back({"skew " + ids[i], "missing", Tone::Muted});
          }
        }
        lines.push_back({"sets",
                         std::to_string(stats.sets) + " (" +
                             std::to_string(stats.incomplete) +
                             " incomplete, " + std::to_string(stats.unmatched) +
                             " unmatched)",
                         Tone::Normal});
        lines.push_back({"saver",
                         std::to_string(saver.pending) + "/" +
                             std::to_string(saver.capacity) + " queued",
                         saver.full ? Tone::Warn : Tone::Normal});
        return lines;
      });
}

core::Result<SessionPanel> session_panel(const quality::SessionSummary& summary,
                                         const std::vector<std::string>& ids,
                                         const PanelOptions& options) {
  return compute::guarded("session_panel", [&]() -> core::Result<SessionPanel> {
    const std::size_t k = ids.size();
    bool square = summary.covisibility.size() == k;
    for (const std::vector<int>& row : summary.covisibility) {
      square = square && row.size() == k;
    }
    if (summary.cameras.size() != k || !square) {
      return core::Status::invalid_argument(
          "session_panel: " + std::to_string(k) + " ids for " +
          std::to_string(summary.cameras.size()) + " cameras and a matrix of " +
          std::to_string(summary.covisibility.size()) + " rows");
    }
    for (std::size_t i = 0; i < k; ++i) {
      const auto& link = summary.cameras[i].link;
      if (link && link->parent >= k) {
        return core::Status::invalid_argument(
            "session_panel: camera " + std::to_string(i) +
            " is linked from camera " + std::to_string(link->parent) + " of " +
            std::to_string(k));
      }
    }

    SessionPanel panel;
    panel.lines.push_back({"captures",
                           std::to_string(summary.kept) + " kept, " +
                               std::to_string(summary.dropped) + " dropped",
                           count_tone(summary.kept)});
    for (std::size_t i = 0; i < k; ++i) {
      const quality::CameraSummary& cam = summary.cameras[i];
      if (cam.link) {
        panel.lines.push_back({ids[i],
                               "from " + ids[cam.link->parent] + ", " +
                                   std::to_string(cam.link->shared) + " shared",
                               strength_tone(cam.link->strength)});
      } else if (cam.linked) {
        panel.lines.push_back({ids[i], "reference", Tone::Muted});
      } else {
        panel.lines.push_back({ids[i], "not linked", Tone::Bad});
      }
    }
    for (const quality::Duplicate& d : summary.duplicates) {
      panel.lines.push_back({"warning",
                             "capture " + std::to_string(d.capture) +
                                 " repeats capture " +
                                 std::to_string(d.earlier),
                             Tone::Warn});
    }
    for (std::size_t i = 0; i < k; ++i) {
      if (summary.cameras[i].floor_only) {
        panel.lines.push_back({"warning",
                               ids[i] + ": its boards nearly all face alike",
                               Tone::Warn});
      }
    }
    for (std::size_t i = 0; i < k; ++i) {
      if (summary.cameras[i].mostly_frontal) {
        panel.lines.push_back(
            {"warning", ids[i] + ": its views mostly face it; tilt the board",
             Tone::Warn});
      }
    }
    if (summary.ready) {
      panel.lines.push_back({"calibrate", "ready", Tone::Good});
    } else {
      panel.lines.push_back({"calibrate",
                             "not yet: every camera needs links of " +
                                 std::to_string(options.session.ready_from) +
                                 " shared captures",
                             Tone::Muted});
    }

    panel.ids = ids;
    panel.shared = summary.covisibility;
    panel.shared_tones.assign(k, std::vector<Tone>(k, Tone::Normal));
    for (std::size_t a = 0; a < k; ++a) {
      for (std::size_t b = 0; b < k; ++b) {
        if (a == b || panel.shared[a][b] <= 0) {
          panel.shared_tones[a][b] = Tone::Muted;
        }
      }
    }
    for (std::size_t i = 0; i < k; ++i) {
      const auto& link = summary.cameras[i].link;
      if (!link) continue;
      const Tone tone = strength_tone(link->strength);
      panel.shared_tones[i][link->parent] = tone;
      panel.shared_tones[link->parent][i] = tone;
    }
    panel.calibrate = summary.ready;
    return panel;
  });
}

core::Result<std::vector<PanelLine>> capture_panel(
    const CaptureStatus& status) {
  return compute::guarded(
      "capture_panel", [&]() -> core::Result<std::vector<PanelLine>> {
        const quality::CaptureDecision& decision = status.decision;
        std::vector<PanelLine> lines;
        switch (decision.state) {
          case quality::CaptureState::Waiting:
            lines.push_back(
                {"capture", "waiting: " + decision.message, Tone::Warn});
            break;
          case quality::CaptureState::Take:
            lines.push_back({"capture", "taken", Tone::Good});
            if (!decision.message.empty()) {
              lines.push_back({"warning", decision.message, Tone::Warn});
            }
            break;
          case quality::CaptureState::Failed:
            lines.push_back(
                {"capture", "failed: " + decision.message, Tone::Bad});
            break;
          case quality::CaptureState::Idle:
          default:
            lines.push_back({"capture", "idle", Tone::Muted});
            break;
        }
        if (status.last) {
          const CaptureMeta& last = *status.last;
          lines.push_back({"last",
                           "#" + std::to_string(last.index) + ": " +
                               std::to_string(status.last_usable_views) +
                               " usable views, still " +
                               fixed(last.stillness_px, 2) + " px, spread " +
                               fixed(last.spread_ms, 2) + " ms",
                           Tone::Normal});
          lines.push_back({"live vs full",
                           last.live_vs_full_px
                               ? fixed(*last.live_vs_full_px, 2) + " px"
                               : std::string("-"),
                           status.last_flagged ? Tone::Warn : Tone::Normal});
        }
        if (!status.error.empty()) {
          lines.push_back({"save", "failed: " + status.error, Tone::Bad});
        }
        return lines;
      });
}

core::Result<gfx::FrameMetrics> stage_frame_metrics(
    const core::StageMetrics& stages, double fps) {
  return compute::guarded("stage_frame_metrics",
                          [&]() -> core::Result<gfx::FrameMetrics> {
                            gfx::FrameMetrics metrics;
                            metrics.sections = stages.rows();
                            metrics.cpu_frame_ms = stages.total_cpu_ms();
                            metrics.fps = fps;
                            return metrics;
                          });
}

void draw_panel_lines(const char* id, const std::vector<PanelLine>& lines) {
  if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingFixedFit)) return;
  for (const PanelLine& line : lines) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted(line.label.c_str());
    ImGui::TableSetColumnIndex(1);
    if (line.tone == Tone::Normal) {
      ImGui::TextUnformatted(line.value.c_str());
    } else {
      ImGui::PushStyleColor(ImGuiCol_Text, tone_color(line.tone));
      ImGui::TextUnformatted(line.value.c_str());
      ImGui::PopStyleColor();
    }
  }
  ImGui::EndTable();
}

void draw_session_panel(const SessionPanel& panel) {
  draw_panel_lines("session", panel.lines);
  const std::size_t k = panel.ids.size();
  if (k == 0 || panel.shared.size() != k || panel.shared_tones.size() != k) {
    return;
  }
  if (!ImGui::BeginTable(
          "shared", static_cast<int>(k) + 1,
          ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit)) {
    return;
  }
  ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
  for (std::size_t c = 0; c < k; ++c) {
    ImGui::TableSetColumnIndex(static_cast<int>(c) + 1);
    ImGui::TextUnformatted(panel.ids[c].c_str());
  }
  for (std::size_t r = 0; r < k; ++r) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted(panel.ids[r].c_str());
    const std::size_t columns =
        std::min({k, panel.shared[r].size(), panel.shared_tones[r].size()});
    for (std::size_t c = 0; c < columns; ++c) {
      ImGui::TableSetColumnIndex(static_cast<int>(c) + 1);
      char count[16];
      std::snprintf(count, sizeof count, "%d", panel.shared[r][c]);
      ImGui::PushStyleColor(ImGuiCol_Text,
                            tone_color(panel.shared_tones[r][c]));
      ImGui::TextUnformatted(count);
      ImGui::PopStyleColor();
    }
  }
  ImGui::EndTable();
}

}  // namespace volumetric_kit::calib::viz
