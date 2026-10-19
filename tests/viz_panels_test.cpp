// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The panels' models: each line's text and tone, from the live checks, the
// array's counters, the session's summary and origin tag, the capture rule
// and the capture records, judged by the quality tier's thresholds;
// Calibrate's progress, its result's metrics and depth checks; and the stage
// timings as gfx's metrics panel takes them. No device or ImGui frame needed
// (viz_render_test draws them).

#include "volumetric_kit/calib/viz/panels.hpp"

#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace volumetric_kit::calib::viz {
namespace {

using Code = core::Status::Code;

// The line labelled `label`, or a failure naming the lines there are.
const PanelLine& line(const std::vector<PanelLine>& lines,
                      const std::string& label) {
  for (const PanelLine& l : lines) {
    if (l.label == label) return l;
  }
  ADD_FAILURE() << "no line " << label;
  static const PanelLine none;
  return none;
}

std::vector<PanelLine> labelled(const std::vector<PanelLine>& lines,
                                const std::string& label) {
  std::vector<PanelLine> out;
  for (const PanelLine& l : lines) {
    if (l.label == label) out.push_back(l);
  }
  return out;
}

TEST(Tone, EachHasItsOwnColour) {
  const Tone tones[] = {Tone::Normal, Tone::Muted, Tone::Good, Tone::Warn,
                        Tone::Bad};
  for (const Tone a : tones) {
    for (const Tone b : tones) {
      if (a != b) {
        EXPECT_NE(tone_color(a), tone_color(b));
      }
    }
  }
  EXPECT_EQ(tone_color(static_cast<Tone>(99)), tone_color(Tone::Normal));
}

TEST(VerdictTone, RedPastTheSolvesTilt) {
  quality::SessionOptions options;
  options.solve.oblique_deg = 75.0;
  ViewCheck oblique;
  oblique.verdict = ViewVerdict::Oblique;
  oblique.tilt_deg = 72.0;
  EXPECT_EQ(verdict_tone(oblique, options), Tone::Warn);
  oblique.tilt_deg = 75.5;
  EXPECT_EQ(verdict_tone(oblique, options), Tone::Bad);
  oblique.tilt_deg.reset();
  EXPECT_EQ(verdict_tone(oblique, options), Tone::Warn);
  ViewCheck unknown;
  unknown.verdict = static_cast<ViewVerdict>(99);
  EXPECT_EQ(verdict_tone(unknown, options), Tone::Muted);
}

CameraLive good_camera() {
  CameraLive camera;
  camera.present = true;
  camera.check.corners = 8;
  camera.check.markers = 7;
  camera.check.distance_m = 2.8146;
  camera.check.tilt_deg = 12.04;
  camera.check.pnp_rms_px = 0.214;
  camera.check.cell = 4;
  camera.check.verdict = ViewVerdict::Good;
  camera.stillness_px = 0.081;
  camera.frames = {1234, 1200, 30, 0};
  return camera;
}

quality::CameraSummary summary_of(int usable, int used, double filled) {
  quality::CameraSummary session;
  session.usable_views = usable;
  session.used_views = used;
  session.coverage_filled = filled;
  return session;
}

TEST(CameraPanel, ShowsWhatTheCameraSees) {
  const core::Result<std::vector<PanelLine>> lines = camera_panel(
      good_camera(), summary_of(12, 10, 0.39), legacy_femto_board());
  ASSERT_TRUE(lines.ok()) << lines.status().message();
  const std::vector<PanelLine>& l = lines.value();
  const std::vector<std::string> order = {
      "verdict", "corners",   "markers", "distance", "tilt",  "fit",
      "cell",    "stillness", "views",   "coverage", "frames"};
  ASSERT_EQ(l.size(), order.size());
  for (std::size_t i = 0; i < order.size(); ++i)
    EXPECT_EQ(l[i].label, order[i]);

  EXPECT_EQ(line(l, "verdict").value, "good");
  EXPECT_EQ(line(l, "verdict").tone, Tone::Good);
  EXPECT_EQ(line(l, "corners").value, "8/8");
  EXPECT_EQ(line(l, "markers").value, "7/7");
  EXPECT_EQ(line(l, "distance").value, "2.81 m");
  EXPECT_EQ(line(l, "tilt").value, "12.0 deg");
  EXPECT_EQ(line(l, "tilt").tone, Tone::Normal);
  EXPECT_EQ(line(l, "fit").value, "0.21 px");
  EXPECT_EQ(line(l, "cell").value, "centre");
  EXPECT_EQ(line(l, "stillness").value, "0.08 px");
  EXPECT_EQ(line(l, "stillness").tone, Tone::Good);
  EXPECT_EQ(line(l, "views").value, "12 usable, 10 used");
  EXPECT_EQ(line(l, "coverage").value, "39 %");
  EXPECT_EQ(line(l, "frames").value,
            "1234 received, 1200 delivered, 30 dropped, 0 failed");
  EXPECT_EQ(line(l, "frames").tone, Tone::Normal);
}

// Tilt is amber past the view check's limit and red past the solve's; the
// limits are the quality tier's options.
TEST(CameraPanel, TonesTiltStillnessAndLossByTheQualityThresholds) {
  PanelOptions options;
  options.session.view.oblique_deg = 50.0;
  options.session.solve.oblique_deg = 65.0;
  options.rule.still_px = 0.5;
  CameraLive camera = good_camera();
  camera.check.tilt_deg = 55.0;
  camera.stillness_px = 0.45;
  camera.lost = 3;
  const quality::CameraSummary session;
  std::vector<PanelLine> l =
      camera_panel(camera, session, legacy_femto_board(), options).value();
  EXPECT_EQ(line(l, "tilt").tone, Tone::Warn);
  EXPECT_EQ(line(l, "stillness").tone, Tone::Good);
  EXPECT_EQ(line(l, "frames").value,
            "1234 received, 1200 delivered, 30 dropped, 0 failed, 3 lost");
  EXPECT_EQ(line(l, "frames").tone, Tone::Warn);
  EXPECT_EQ(line(l, "views").tone, Tone::Muted);
  EXPECT_EQ(line(l, "coverage").tone, Tone::Muted);

  camera.check.tilt_deg = 66.0;
  camera.stillness_px = 0.5;
  camera.lost = 0;
  camera.frames.failed = 1;
  l = camera_panel(camera, session, legacy_femto_board(), options).value();
  EXPECT_EQ(line(l, "tilt").tone, Tone::Bad);
  EXPECT_EQ(line(l, "stillness").tone, Tone::Warn);
  EXPECT_EQ(line(l, "frames").tone, Tone::Warn);

  // At a limit, the tilt is not yet past it, as check_view judges.
  camera.check.tilt_deg = 50.0;
  l = camera_panel(camera, session, legacy_femto_board(), options).value();
  EXPECT_EQ(line(l, "tilt").tone, Tone::Normal);
  camera.check.tilt_deg = 65.0;
  l = camera_panel(camera, session, legacy_femto_board(), options).value();
  EXPECT_EQ(line(l, "tilt").tone, Tone::Warn);
}

// No frame in the set is red; a view with no pose shows no measures.
TEST(CameraPanel, ShowsAMissingFrameAndAViewWithoutAPose) {
  CameraLive camera;
  const std::vector<PanelLine> l =
      camera_panel(camera, {}, legacy_femto_board()).value();
  EXPECT_EQ(line(l, "verdict").value, "no frame");
  EXPECT_EQ(line(l, "verdict").tone, Tone::Bad);
  EXPECT_EQ(line(l, "corners").value, "0/8");
  EXPECT_EQ(line(l, "corners").tone, Tone::Muted);
  for (const char* label : {"distance", "tilt", "fit", "cell", "stillness"}) {
    EXPECT_EQ(line(l, label).value, "-") << label;
    EXPECT_EQ(line(l, label).tone, Tone::Muted) << label;
  }
  camera.present = true;
  EXPECT_EQ(
      line(camera_panel(camera, {}, legacy_femto_board()).value(), "verdict")
          .value,
      "none");
}

recon::sensor::FrameSet set_of(
    const std::vector<std::optional<std::int64_t>>& at) {
  recon::sensor::FrameSet set;
  set.timestamp_ns = 1'000'000'000;
  for (const std::optional<std::int64_t>& offset_ns : at) {
    set.frames.emplace_back();
    if (!offset_ns) continue;
    set.frames.back().emplace();
    set.frames.back()->timestamp_ns =
        static_cast<std::uint64_t>(1'000'000'000 + *offset_ns);
  }
  return set;
}

const std::vector<std::string> kIds = {"N", "G", "6G"};

TEST(SetPanel, ShowsTheSetsFramesSpreadAndSkew) {
  recon::sensor::SensorArrayStats stats;
  stats.sets = 900;
  stats.incomplete = 3;
  stats.unmatched = 12;
  const std::vector<PanelLine> l =
      set_panel(set_of({0, -1'200'000, 2'500'000}), stats, kIds, {1, 4, false})
          .value();
  EXPECT_EQ(line(l, "set").value, "3/3 frames");
  EXPECT_EQ(line(l, "set").tone, Tone::Good);
  EXPECT_EQ(line(l, "spread").value, "3.70 ms");
  EXPECT_EQ(line(l, "spread").tone, Tone::Normal);
  EXPECT_EQ(line(l, "skew N").value, "+0.00 ms");
  EXPECT_EQ(line(l, "skew G").value, "-1.20 ms");
  EXPECT_EQ(line(l, "skew 6G").value, "+2.50 ms");
  EXPECT_EQ(line(l, "sets").value, "900 (3 incomplete, 12 unmatched)");
  EXPECT_EQ(line(l, "saver").value, "1/4 queued");
  EXPECT_EQ(line(l, "saver").tone, Tone::Normal);
}

TEST(SetPanel, WarnsOfAMissingFrameAWideSpreadAndAFullWriter) {
  PanelOptions options;
  options.rule.spread_warn_ms = 1.0;
  const std::vector<PanelLine> l =
      set_panel(set_of({0, std::nullopt, 1'500'000}), {}, kIds, {4, 4, true},
                options)
          .value();
  EXPECT_EQ(line(l, "set").value, "2/3 frames");
  EXPECT_EQ(line(l, "set").tone, Tone::Warn);
  EXPECT_EQ(line(l, "spread").tone, Tone::Warn);
  EXPECT_EQ(line(l, "skew G").value, "missing");
  EXPECT_EQ(line(l, "skew G").tone, Tone::Muted);
  EXPECT_EQ(line(l, "saver").tone, Tone::Warn);
}

// The writer is full when the caller says so, as it tells the capture rule:
// the panel keeps no rule of its own.
TEST(SetPanel, TonesTheWriterAsTheCallerJudgesIt) {
  const recon::sensor::FrameSet set = set_of({0, 0, 0});
  EXPECT_EQ(line(set_panel(set, {}, kIds, {4, 4, false}).value(), "saver").tone,
            Tone::Normal);
  EXPECT_EQ(line(set_panel(set, {}, kIds, {1, 4, true}).value(), "saver").tone,
            Tone::Warn);
}

TEST(SetPanel, WaitsForTheFirstSetAndRefusesIdsThatDoNotMatch) {
  const std::vector<PanelLine> l =
      set_panel(recon::sensor::FrameSet{}, {}, {}, {}).value();
  EXPECT_EQ(line(l, "set").value, "none yet");
  EXPECT_EQ(line(l, "spread").tone, Tone::Muted);
  EXPECT_EQ(line(l, "saver").value, "0/0 queued");
  EXPECT_EQ(line(l, "saver").tone, Tone::Normal);
  EXPECT_EQ(set_panel(set_of({0, 0}), {}, kIds, {}).status().domain(),
            Code::InvalidArgument);
}

// Three cameras: G linked from N strongly, 6G from G weakly; N the
// reference.
quality::SessionSummary three_camera_summary() {
  quality::SessionSummary summary;
  summary.kept = 14;
  summary.dropped = 1;
  summary.cameras.resize(3);
  summary.cameras[0].linked = true;
  summary.cameras[1].linked = true;
  summary.cameras[1].link =
      quality::PreviewLink{0, 12, quality::LinkStrength::Strong};
  summary.cameras[2].linked = true;
  summary.cameras[2].link =
      quality::PreviewLink{1, 4, quality::LinkStrength::Weak};
  summary.covisibility = {{14, 12, 0}, {12, 13, 4}, {0, 4, 5}};
  summary.ready = true;
  return summary;
}

TEST(SessionPanel, ShowsTheCapturesAndTheTreePreview) {
  const core::Result<SessionPanel> panel =
      session_panel(three_camera_summary(), kIds);
  ASSERT_TRUE(panel.ok()) << panel.status().message();
  const std::vector<PanelLine>& l = panel.value().lines;
  EXPECT_EQ(line(l, "captures").value, "14 kept, 1 dropped");
  EXPECT_EQ(line(l, "N").value, "reference");
  EXPECT_EQ(line(l, "N").tone, Tone::Muted);
  EXPECT_EQ(line(l, "G").value, "from N, 12 shared");
  EXPECT_EQ(line(l, "G").tone, Tone::Good);
  EXPECT_EQ(line(l, "6G").value, "from G, 4 shared");
  EXPECT_EQ(line(l, "6G").tone, Tone::Bad);
  EXPECT_EQ(line(l, "calibrate").value, "ready");
  EXPECT_EQ(line(l, "calibrate").tone, Tone::Good);
  EXPECT_TRUE(labelled(l, "warning").empty());
}

// The matrix: each tree link in its strength's tone, both ways; the
// diagonal and pairs that share nothing muted.
TEST(SessionPanel, TonesTheSharedCapturesByTheTreesLinks) {
  quality::SessionSummary summary = three_camera_summary();
  summary.cameras[2].link->strength = quality::LinkStrength::Fair;
  const SessionPanel panel = session_panel(summary, kIds).value();
  EXPECT_EQ(panel.ids, kIds);
  EXPECT_EQ(panel.shared, summary.covisibility);
  const std::vector<std::vector<Tone>> expected = {
      {Tone::Muted, Tone::Good, Tone::Muted},
      {Tone::Good, Tone::Muted, Tone::Warn},
      {Tone::Muted, Tone::Warn, Tone::Muted}};
  EXPECT_EQ(panel.shared_tones, expected);
}

TEST(SessionPanel, WarnsAndHoldsCalibrateBack) {
  quality::SessionSummary summary = three_camera_summary();
  summary.cameras[2].linked = false;
  summary.cameras[2].link.reset();
  summary.cameras[1].floor_only = true;
  summary.cameras[0].mostly_frontal = true;
  summary.duplicates.push_back({7, 3});
  summary.ready = false;
  PanelOptions options;
  options.session.ready_from = 4;
  const SessionPanel panel = session_panel(summary, kIds, options).value();
  const std::vector<PanelLine>& l = panel.lines;
  EXPECT_EQ(line(l, "6G").value, "not linked");
  EXPECT_EQ(line(l, "6G").tone, Tone::Bad);
  const std::vector<PanelLine> warnings = labelled(l, "warning");
  ASSERT_EQ(warnings.size(), 3u);
  EXPECT_EQ(warnings[0].value, "capture 7 repeats capture 3");
  EXPECT_EQ(warnings[1].value, "G: its boards nearly all face alike");
  EXPECT_EQ(warnings[2].value, "N: its views mostly face it; tilt the board");
  for (const PanelLine& w : warnings) EXPECT_EQ(w.tone, Tone::Warn);
  EXPECT_EQ(line(l, "calibrate").value,
            "not yet: every camera needs links of 4 shared captures");
  EXPECT_EQ(line(l, "calibrate").tone, Tone::Muted);
}

TEST(SessionPanel, RefusesASummaryThatDoesNotMatchItsIds) {
  EXPECT_EQ(session_panel(three_camera_summary(), {"N", "G"}).status().domain(),
            Code::InvalidArgument);
  quality::SessionSummary ragged = three_camera_summary();
  ragged.covisibility[1].pop_back();
  EXPECT_EQ(session_panel(ragged, kIds).status().domain(),
            Code::InvalidArgument);
  quality::SessionSummary far_link = three_camera_summary();
  far_link.cameras[2].link->parent = 3;
  EXPECT_EQ(session_panel(far_link, kIds).status().domain(),
            Code::InvalidArgument);
}

// Green once 2 cameras see the tag, amber at 1, muted before any.
TEST(OriginTagLine, TonesTheCamerasThatSawTheTag) {
  PanelLine l = origin_tag_line({4, 0, 0}).value();
  EXPECT_EQ(l.label, "origin tag");
  EXPECT_EQ(l.value, "seen by 0 of 4 cameras");
  EXPECT_EQ(l.tone, Tone::Muted);

  l = origin_tag_line({4, 1, 1}).value();
  EXPECT_EQ(l.value,
            "seen by 1 of 4 cameras, 1 view: the reference camera will be the "
            "origin unless a second camera sees it");
  EXPECT_EQ(l.tone, Tone::Warn);

  l = origin_tag_line({4, 2, 9}).value();
  EXPECT_EQ(l.value, "seen by 2 of 4 cameras, 9 views");
  EXPECT_EQ(l.tone, Tone::Good);
  l = origin_tag_line({4, 4, 31}).value();
  EXPECT_EQ(l.value, "seen by 4 of 4 cameras, 31 views");
  EXPECT_EQ(l.tone, Tone::Good);

  // A rig of one camera never sees it in two.
  l = origin_tag_line({1, 1, 3}).value();
  EXPECT_EQ(l.value.rfind("seen by 1 of 1 camera, 3 views: ", 0), 0u)
      << l.value;
  EXPECT_EQ(l.tone, Tone::Warn);
}

TEST(OriginTagLine, RefusesACoverageNoRigHas) {
  const core::Result<PanelLine> more = origin_tag_line({2, 3, 3});
  ASSERT_FALSE(more.ok());
  EXPECT_EQ(more.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(more.status().message(),
            "origin_tag_line: seen by 3 of 2 cameras in 3 views");
  EXPECT_EQ(origin_tag_line({4, 2, 1}).status().domain(),
            Code::InvalidArgument);
}

TEST(CapturePanel, ShowsWhereTheRuleStands) {
  CaptureStatus status;
  EXPECT_EQ(line(capture_panel(status).value(), "capture").value, "idle");

  status.decision.state = quality::CaptureState::Waiting;
  status.decision.message = "board moving (0.82 px)";
  std::vector<PanelLine> l = capture_panel(status).value();
  EXPECT_EQ(line(l, "capture").value, "waiting: board moving (0.82 px)");
  EXPECT_EQ(line(l, "capture").tone, Tone::Warn);

  status.decision.state = quality::CaptureState::Failed;
  status.decision.message = "CL2A14100A4 missing";
  l = capture_panel(status).value();
  EXPECT_EQ(line(l, "capture").value, "failed: CL2A14100A4 missing");
  EXPECT_EQ(line(l, "capture").tone, Tone::Bad);

  status.decision.state = quality::CaptureState::Take;
  status.decision.message = "frames spread over 6.10 ms";
  l = capture_panel(status).value();
  EXPECT_EQ(line(l, "capture").value, "taken");
  EXPECT_EQ(line(l, "capture").tone, Tone::Good);
  EXPECT_EQ(line(l, "warning").value, "frames spread over 6.10 ms");
  status.decision.message.clear();
  EXPECT_TRUE(labelled(capture_panel(status).value(), "warning").empty());
}

TEST(CapturePanel, ShowsTheLastCaptureAndAFailedWrite) {
  CaptureStatus status;
  CaptureMeta meta;
  meta.index = 12;
  meta.stillness_px = 0.08;
  meta.spread_ms = 1.4;
  meta.live_vs_full_px = 0.42;
  status.last = meta;
  status.last_usable_views = 3;
  status.last_tag_cameras = 2;
  status.last_flagged = true;
  status.error = "disk full";
  const std::vector<PanelLine> l = capture_panel(status).value();
  EXPECT_EQ(line(l, "last").value,
            "#12: 3 usable views, the origin tag in 2, still 0.08 px, spread "
            "1.40 ms");
  EXPECT_EQ(line(l, "live vs full").value, "0.42 px");
  EXPECT_EQ(line(l, "live vs full").tone, Tone::Warn);
  EXPECT_EQ(line(l, "save").value, "failed: disk full");
  EXPECT_EQ(line(l, "save").tone, Tone::Bad);

  status.last->live_vs_full_px.reset();
  status.last_flagged = false;
  status.error.clear();
  const std::vector<PanelLine> unflagged = capture_panel(status).value();
  EXPECT_EQ(line(unflagged, "live vs full").value, "-");
  EXPECT_EQ(line(unflagged, "live vs full").tone, Tone::Normal);
  EXPECT_TRUE(labelled(unflagged, "save").empty());
}

TEST(CalibrateProgressLine, ShowsTheStageItsCountAndTheSeconds) {
  PanelLine l = calibrate_progress_line("solving", 0, 0, 1.26).value();
  EXPECT_EQ(l.label, "calibrating");
  EXPECT_EQ(l.value, "solving, 1.3 s");
  EXPECT_EQ(l.tone, Tone::Normal);
  l = calibrate_progress_line("measuring", 4, 14, 12.34).value();
  EXPECT_EQ(l.value, "measuring 4/14, 12.3 s");
  l = calibrate_progress_line("measuring", 0, 14, 0.0).value();
  EXPECT_EQ(l.value, "measuring 0/14, 0.0 s");
}

quality::ErrorStats rms_of(double rms_px, std::size_t count = 100) {
  quality::ErrorStats stats;
  stats.count = count;
  stats.rms_px = rms_px;
  return stats;
}

// A solved rig of N, G and 6G: N the reference, which never moves.
quality::CalibrationMetrics three_camera_metrics() {
  quality::CalibrationMetrics m;
  quality::Reprojection& r = m.reprojection;
  r.initial.overall = rms_of(0.8412);
  r.initial.legacy_rmse_px = 0.5948;
  r.initial.cameras = {rms_of(0.70), rms_of(0.92), rms_of(0.88)};
  r.solved.overall = rms_of(0.3134);
  r.solved.legacy_rmse_px = 0.2216;
  r.solved.cameras = {rms_of(0.29), rms_of(0.33), rms_of(0.315)};
  m.transfer.median_px = 0.624;
  m.transfer.p90_px = 1.396;
  m.leave_one_out.cameras.resize(3);
  m.leave_one_out.cameras[1] = {1.234, 7, 0.0456, 3};
  m.leave_one_out.cameras[2] = {0.5, 9, 0.0, std::nullopt};
  m.principal_point.shift_px = 2.5;
  m.principal_point.as_given.rms_px = 0.312;
  m.principal_point.shifted.rms_px = 0.3023;
  m.principal_point.verdict = quality::PrincipalPointVerdict::Shifted;
  m.geometry.centres_m = {
      {0.0, 0.0, 0.0}, {1.2344, -0.0504, 0.3}, {-0.8, 0.0126, 2.25}};
  return m;
}

TEST(CalibrationPanel, ShowsWhatTheResultIsWorth) {
  const core::Result<std::vector<PanelLine>> lines =
      calibration_panel(kIds, three_camera_metrics(), 2,
                        "/s/results/002_2026-10-19_101500", 14, 3.21);
  ASSERT_TRUE(lines.ok()) << lines.status().message();
  const std::vector<PanelLine>& l = lines.value();
  const std::vector<std::string> order = {
      "calibration", "RMS",      "RMS N",     "RMS G",    "RMS 6G",
      "transfer",    "moves N",  "moves G",   "moves 6G", "principal point",
      "centre N",    "centre G", "centre 6G", "results"};
  ASSERT_EQ(l.size(), order.size());
  for (std::size_t i = 0; i < order.size(); ++i) {
    EXPECT_EQ(l[i].label, order[i]);
  }
  EXPECT_EQ(line(l, "calibration").value, "002, 14 captures, 3.2 s");
  EXPECT_EQ(line(l, "RMS").value,
            "0.84 -> 0.31 px, legacy RMSE 0.59 -> 0.22 px");
  EXPECT_EQ(line(l, "RMS G").value, "0.92 -> 0.33 px");
  EXPECT_EQ(line(l, "RMS G").tone, Tone::Normal);
  EXPECT_EQ(line(l, "transfer").value, "median 0.62 px, p90 1.40 px");
  EXPECT_EQ(line(l, "transfer").tone, Tone::Normal);
  EXPECT_EQ(line(l, "moves N").value, "-");
  EXPECT_EQ(line(l, "moves N").tone, Tone::Muted);
  EXPECT_EQ(line(l, "moves G").value,
            "1.23 mm (capture 7), 0.046 deg (capture 3)");
  EXPECT_EQ(line(l, "moves G").tone, Tone::Normal);
  EXPECT_EQ(line(l, "moves 6G").value, "0.50 mm (capture 9)");
  EXPECT_EQ(line(l, "principal point").value,
            "shifted (gap 3.1 %): 0.312 px as given, 0.302 px shifted");
  EXPECT_EQ(line(l, "principal point").tone, Tone::Warn);
  EXPECT_EQ(line(l, "centre N").value, "(+0.000, +0.000, +0.000) m");
  EXPECT_EQ(line(l, "centre G").value, "(+1.234, -0.050, +0.300) m");
  EXPECT_EQ(line(l, "centre 6G").value, "(-0.800, +0.013, +2.250) m");
  EXPECT_EQ(line(l, "results").value, "/s/results/002_2026-10-19_101500");
}

// The A/B verdict's three answers, each in its tone, and a refused re-solve.
TEST(CalibrationPanel, TonesThePrincipalPointsVerdict) {
  quality::CalibrationMetrics m = three_camera_metrics();
  const auto principal = [&] {
    return line(calibration_panel(kIds, m, 0, "d", 1, 0.0).value(),
                "principal point");
  };
  m.principal_point.shifted.rms_px = 0.3083;
  m.principal_point.verdict = quality::PrincipalPointVerdict::Inconclusive;
  PanelLine l = principal();
  EXPECT_EQ(l.value,
            "inconclusive (gap 1.2 %): 0.312 px as given, 0.308 px shifted");
  EXPECT_EQ(l.tone, Tone::Normal);

  m.principal_point.shifted.rms_px = 0.33;
  m.principal_point.verdict = quality::PrincipalPointVerdict::AsGiven;
  l = principal();
  EXPECT_EQ(l.value,
            "as given (gap -5.5 %): 0.312 px as given, 0.330 px shifted");
  EXPECT_EQ(l.tone, Tone::Good);

  m.principal_point.refused = "a link is left short of 3 shared captures";
  m.principal_point.as_given = {};
  m.principal_point.shifted = {};
  m.principal_point.verdict = quality::PrincipalPointVerdict::Inconclusive;
  l = principal();
  EXPECT_EQ(l.value, "refused: a link is left short of 3 shared captures");
  EXPECT_EQ(l.tone, Tone::Muted);
}

// A figure that is not a number shows as `-`: a camera with no corner kept,
// a transfer with no pairs, a gap of sides not measured.
TEST(CalibrationPanel, ShowsAFigureNotMeasuredAsADash) {
  quality::CalibrationMetrics m = three_camera_metrics();
  m.reprojection.initial.cameras[2] = {};
  m.reprojection.solved.cameras[2] = {};
  m.transfer = {};
  m.principal_point.as_given = {};
  m.principal_point.verdict = quality::PrincipalPointVerdict::Inconclusive;
  const std::vector<PanelLine> l =
      calibration_panel(kIds, m, 1, "d", 1, 0.5).value();
  EXPECT_EQ(line(l, "calibration").value, "001, 1 capture, 0.5 s");
  EXPECT_EQ(line(l, "RMS 6G").value, "- -> -");
  EXPECT_EQ(line(l, "RMS 6G").tone, Tone::Muted);
  EXPECT_EQ(line(l, "transfer").value, "median -, p90 -");
  EXPECT_EQ(line(l, "transfer").tone, Tone::Muted);
  EXPECT_EQ(line(l, "principal point").value,
            "inconclusive (gap -): - as given, 0.302 px shifted");
}

TEST(CalibrationPanel, RefusesMetricsOfAnotherRig) {
  const quality::CalibrationMetrics m = three_camera_metrics();
  const core::Result<std::vector<PanelLine>> two =
      calibration_panel({"N", "G"}, m, 0, "d", 1, 0.0);
  ASSERT_FALSE(two.ok());
  EXPECT_EQ(two.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(two.status().message(),
            "calibration_panel: 2 ids for metrics of 3 cameras");
  for (int part = 0; part < 4; ++part) {
    quality::CalibrationMetrics short_one = m;
    switch (part) {
      case 0:
        short_one.reprojection.initial.cameras.pop_back();
        break;
      case 1:
        short_one.reprojection.solved.cameras.pop_back();
        break;
      case 2:
        short_one.leave_one_out.cameras.pop_back();
        break;
      default:
        short_one.geometry.centres_m.pop_back();
        break;
    }
    EXPECT_EQ(
        calibration_panel(kIds, short_one, 0, "d", 1, 0.0).status().domain(),
        Code::InvalidArgument)
        << part;
  }
}

// --- Depth ------------------------------------------------------------------

quality::PairCheck pair_check(std::size_t from, std::size_t to,
                              quality::DepthVerdict verdict,
                              double signed_median_mm = 0.0,
                              double median_abs_mm = 1.0,
                              std::size_t overlap = 5000,
                              std::size_t in_window = 4800) {
  quality::PairCheck p;
  p.agreement.from = from;
  p.agreement.to = to;
  p.agreement.overlap = overlap;
  p.agreement.in_window = in_window;
  p.agreement.signed_median_mm = signed_median_mm;
  p.agreement.median_abs_mm = median_abs_mm;
  p.verdict = verdict;
  return p;
}

// N and G pass, 6G fails its plane; of the six pairs, G -> 6G fails and
// 6G -> N has no overlap; on their own, G -> 6G fails in captures 5 and 7,
// with N -> G in 5, and none in 9.
quality::DepthSummary three_camera_depth() {
  using V = quality::DepthVerdict;
  quality::DepthSummary d;
  d.ratio = {412, 1.00084, 0.00113};
  d.scale = V::Ok;
  d.cameras.resize(3);
  d.cameras[0] = {14, {140, 1.0003, 0.001}, V::Ok, 0.42, 0.214, V::Ok, V::Ok};
  d.cameras[1] = {1, {12, 0.99951, 0.002}, V::Ok, -1.24, 0.5, V::Ok, V::Ok};
  d.cameras[2] = {13, {130, 1.0019, 0.001}, V::Ok, 12.5, 0.38, V::Off, V::Off};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  d.pairs = {pair_check(0, 1, V::Ok, 0.12),
             pair_check(0, 2, V::Ok, -1.4),
             pair_check(1, 0, V::Ok, -0.3),
             pair_check(1, 2, V::Off, 6.24, 7.04, 5000, 3000),
             pair_check(2, 0, V::Insufficient, nan, nan, 0, 0),
             pair_check(2, 1, V::Ok, 2.0)};
  quality::CapturePairs five{5, d.pairs};
  five.pairs[0].verdict = V::Off;
  quality::CapturePairs seven{7, d.pairs};
  quality::CapturePairs nine{9, d.pairs};
  nine.pairs[3].verdict = V::Ok;
  d.captures = {five, seven, nine};
  return d;
}

// Each camera's verdict and figures, the board's scale, the pairs counted
// with the largest signed median, each failing pair and each capture with a
// pair failing on its own, each in its tone.
TEST(DepthPanel, ShowsEachCameraAndThePairsThatFail) {
  const quality::DepthSummary d = three_camera_depth();
  const core::Result<std::vector<PanelLine>> lines = depth_panel(kIds, &d);
  ASSERT_TRUE(lines.ok()) << lines.status().message();
  const std::vector<PanelLine>& l = lines.value();
  const std::vector<std::string> order = {
      "depth scale", "depth N",      "depth G",   "depth 6G",
      "depth pairs", "pair G -> 6G", "capture 5", "capture 7"};
  ASSERT_EQ(l.size(), order.size());
  for (std::size_t i = 0; i < order.size(); ++i) {
    EXPECT_EQ(l[i].label, order[i]);
  }
  EXPECT_EQ(line(l, "depth scale").value,
            "pass: ratio 1.0008 (sd 0.0011, 412 ratios)");
  EXPECT_EQ(line(l, "depth scale").tone, Tone::Good);
  EXPECT_EQ(line(l, "depth N").value,
            "pass, 14 views: ratio 1.0003, offset +0.4 mm, angle 0.21 deg");
  EXPECT_EQ(line(l, "depth N").tone, Tone::Good);
  EXPECT_EQ(line(l, "depth G").value,
            "pass, 1 view: ratio 0.9995, offset -1.2 mm, angle 0.50 deg");
  EXPECT_EQ(line(l, "depth 6G").value,
            "fail, 13 views: ratio 1.0019, offset +12.5 mm, angle 0.38 deg");
  EXPECT_EQ(line(l, "depth 6G").tone, Tone::Bad);
  EXPECT_EQ(line(l, "depth pairs").value,
            "4 of 6 pass, 1 fail, 1 no overlap; largest signed median "
            "+6.2 mm (G -> 6G)");
  EXPECT_EQ(line(l, "depth pairs").tone, Tone::Bad);
  EXPECT_EQ(line(l, "pair G -> 6G").value,
            "fail: signed median +6.2 mm, median |d| 7.0 mm, 60 % in the "
            "window");
  EXPECT_EQ(line(l, "pair G -> 6G").tone, Tone::Bad);
  EXPECT_EQ(line(l, "capture 5").value, "fail: N -> G, G -> 6G");
  EXPECT_EQ(line(l, "capture 5").tone, Tone::Warn);
  EXPECT_EQ(line(l, "capture 7").value, "fail: G -> 6G");
}

// Every pair passing is green; with one short of overlap and none failing,
// plain. A camera with no view is insufficient, muted, its figures dashes.
TEST(DepthPanel, TonesThePairsAndAnInsufficientCamera) {
  using V = quality::DepthVerdict;
  quality::DepthSummary d = three_camera_depth();
  d.captures.clear();
  d.pairs[3].verdict = V::Ok;
  d.pairs[4] = pair_check(2, 0, V::Ok, -0.5);
  d.cameras[2] = {};
  d.scale = V::Insufficient;
  d.ratio = {};
  std::vector<PanelLine> l = depth_panel(kIds, &d).value();
  EXPECT_EQ(line(l, "depth pairs").value,
            "6 of 6 pass, 0 fail, 0 no overlap; largest signed median "
            "+6.2 mm (G -> 6G)");
  EXPECT_EQ(line(l, "depth pairs").tone, Tone::Good);
  EXPECT_EQ(line(l, "depth 6G").value,
            "insufficient, 0 views: ratio -, offset -, angle -");
  EXPECT_EQ(line(l, "depth 6G").tone, Tone::Muted);
  EXPECT_EQ(line(l, "depth scale").value,
            "insufficient: ratio - (sd -, 0 ratios)");
  EXPECT_EQ(line(l, "depth scale").tone, Tone::Muted);
  EXPECT_TRUE(labelled(l, "pair G -> 6G").empty());

  d.pairs[4] = pair_check(2, 0, V::Insufficient, 0.0, 1.0, 0, 0);
  for (quality::PairCheck& p : d.pairs) p.agreement.signed_median_mm = 0.0;
  l = depth_panel(kIds, &d).value();
  EXPECT_EQ(line(l, "depth pairs").value,
            "5 of 6 pass, 0 fail, 1 no overlap; largest signed median "
            "+0.0 mm (N -> G)");
  EXPECT_EQ(line(l, "depth pairs").tone, Tone::Normal);
}

// Not checked: muted when not asked for, red with the failure's reason.
TEST(DepthPanel, SaysWhyTheDepthWasNotChecked) {
  std::vector<PanelLine> l = depth_panel(kIds, nullptr).value();
  ASSERT_EQ(l.size(), 1u);
  EXPECT_EQ(l[0].label, "depth");
  EXPECT_EQ(l[0].value, "not checked");
  EXPECT_EQ(l[0].tone, Tone::Muted);
  l = depth_panel(kIds, nullptr,
                  core::Status::io_error("depth checks: the device was lost"))
          .value();
  ASSERT_EQ(l.size(), 1u);
  EXPECT_EQ(l[0].value, "not checked: depth checks: the device was lost");
  EXPECT_EQ(l[0].tone, Tone::Bad);
}

TEST(DepthPanel, RefusesChecksOfAnotherRig) {
  const quality::DepthSummary d = three_camera_depth();
  const core::Result<std::vector<PanelLine>> two = depth_panel({"N", "G"}, &d);
  ASSERT_FALSE(two.ok());
  EXPECT_EQ(two.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(two.status().message(),
            "depth_panel: depth checks of 3 cameras, or a pair outside them, "
            "for 2 ids");
  quality::DepthSummary stranger = d;
  stranger.captures[1].pairs[2].agreement.to = 3;
  EXPECT_EQ(depth_panel(kIds, &stranger).status().domain(),
            Code::InvalidArgument);
  stranger = d;
  stranger.pairs[0].agreement.from = 4;
  EXPECT_EQ(depth_panel(kIds, &stranger).status().domain(),
            Code::InvalidArgument);
}

// The stages' rows go to gfx's panel as they are; the frame's time is their
// host total without breakdowns.
TEST(StageFrameMetrics, CarriesTheStagesToGfxsPanel) {
  core::StageMetrics stages;
  stages.add_cpu("poll", 1.0);
  stages.add_cpu("readback", 2.0);
  stages.add_gpu("readback", 1.5);
  stages.add_cpu("  ..copy", 1.25);  // a breakdown of readback
  stages.add_cpu("detect", 9.0);
  const gfx::FrameMetrics metrics = stage_frame_metrics(stages, 24.5).value();
  ASSERT_EQ(metrics.sections.size(), 4u);
  EXPECT_STREQ(metrics.sections[1].name, "readback");
  EXPECT_TRUE(metrics.sections[1].has_gpu);
  EXPECT_EQ(metrics.sections[1].gpu_ms, 1.5);
  EXPECT_EQ(metrics.cpu_frame_ms, 12.0);
  EXPECT_EQ(metrics.fps, 24.5);
}

}  // namespace
}  // namespace volumetric_kit::calib::viz
