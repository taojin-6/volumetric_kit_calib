// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's per-mode table: the mode an operation asks for; each camera's
// live lens (the file's at exactly its size, the factory's, the stand-in) and
// what extrinsic mode refuses; where each lens comes from, as the Setup panel
// says it; the mark a lens carries into the file the solve writes; where a
// lens fit starts, never the stand-in; the board each mode seeds and takes, a
// board too small for a lens view refused, saying why; the capture rule; when
// Calibrate may run, and why not, per camera; a stereo pair's camera records
// and lenses; and every part of the window a mode sets, set from one place.
// Host-only.

#include "volumetric_kit/calib/app/calib_mode.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "result_checks.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = sensor::camera;
using capture::LensOrigin;
using capture::SessionMode;
using test::expect_refused;
using Code = core::Status::Code;

constexpr camera::ImageSize k4K{3840, 2160};

// A lens of `size`, its focal length `f`, told apart by it.
camera::CameraModel lens(camera::ImageSize size, double f) {
  camera::CameraModel m;
  m.size = size;
  m.intrinsics = {f, f, 0.5 * size.width, 0.5 * size.height};
  return m;
}

// A calibration file with each entry's colour lens.
camera::ArrayCalibration file_of(
    const std::vector<std::pair<std::string, camera::CameraCalibration>>&
        entries) {
  camera::ArrayCalibration file;
  for (const auto& [id, color] : entries) {
    camera::SensorCalibration& sensor = file.sensors.emplace_back();
    sensor.id = id;
    sensor.color = color;
  }
  return file;
}

camera::CameraCalibration calibrated(camera::ImageSize size, double f) {
  return {lens(size, f), camera::IntrinsicsSource::Calibrated};
}

TEST(AppCalibMode, TheOperationIsTheMode) {
  using Operation = core::config::Operation;
  EXPECT_EQ(session_mode(Operation::CalibrateIntrinsics).value(),
            SessionMode::Intrinsics);
  EXPECT_EQ(session_mode(Operation::Calibrate).value(),
            SessionMode::Extrinsics);
  expect_refused(
      session_mode(Operation::Fuse).status(), Code::InvalidArgument,
      "operation: " + std::to_string(static_cast<int>(Operation::Fuse)) +
          " is not one calib_rig runs (calibrate, calibrate_intrinsics)");
  expect_refused(session_mode(static_cast<Operation>(99)).status(),
                 Code::InvalidArgument,
                 "operation: 99 is not one calib_rig runs");
}

// The live lens, by mode: the file's at exactly the pictures' size first,
// then the factory's, then, for lenses alone, the stand-in.
TEST(AppCalibMode, EachCamerasLensComesFromTheFileTheFactoryOrTheStandIn) {
  const std::string path = "bm_pair/array_calibration.json";
  const camera::ArrayCalibration file =
      file_of({{"bm-1", calibrated(k4K, 1702.0)},
               {"bm-2", calibrated({1920, 1080}, 851.0)}});
  const std::optional<camera::CameraModel> factory = lens(k4K, 2200.0);
  for (const SessionMode mode :
       {SessionMode::Intrinsics, SessionMode::Extrinsics}) {
    // The file's lens at the pictures' size.
    const RigLens filed =
        rig_lens(mode, "bm-1", k4K, std::nullopt, &file, path).value();
    EXPECT_EQ(filed.origin, LensOrigin::File);
    EXPECT_EQ(filed.model.intrinsics.fx, 1702.0);
    // No lens of the camera in the file: the factory's.
    const RigLens own =
        rig_lens(mode, "bm-3", k4K, factory, &file, path).value();
    EXPECT_EQ(own.origin, LensOrigin::Factory);
    EXPECT_EQ(own.model.intrinsics.fx, 2200.0);
    EXPECT_EQ(rig_lens(mode, "bm-3", k4K, factory, nullptr, "").value().origin,
              LensOrigin::Factory);
  }
  // A lens at another size is not the camera's: it falls to the stand-in, in
  // either mode (for poses Calibrate then waits for a real one: lens_status).
  for (const SessionMode mode :
       {SessionMode::Intrinsics, SessionMode::Extrinsics}) {
    const RigLens other =
        rig_lens(mode, "bm-2", k4K, std::nullopt, &file, path).value();
    EXPECT_EQ(other.origin, LensOrigin::StandIn);
    EXPECT_EQ(other.model.size.width, 3840u);
    EXPECT_EQ(other.model.intrinsics.fx, 1920.0);
    EXPECT_EQ(other.model.intrinsics.fy, 1920.0);
    EXPECT_EQ(other.model.intrinsics.cx, 1919.5);
    EXPECT_EQ(other.model.intrinsics.cy, 1079.5);
    EXPECT_EQ(other.model.distortion.k1, 0.0);
    EXPECT_EQ(
        rig_lens(mode, "bm-2", k4K, std::nullopt, nullptr, "").value().origin,
        LensOrigin::StandIn);
  }
  // A factory camera with a calibrated lens in the file, for poses: the
  // runtime would read the factory's. A factory-sourced one is the same lens.
  expect_refused(
      rig_lens(SessionMode::Extrinsics, "bm-1", k4K, factory, &file, path)
          .status(),
      Code::InvalidArgument,
      "bm-1 has a factory lens and a calibrated one in " + path);
  EXPECT_EQ(rig_lens(SessionMode::Intrinsics, "bm-1", k4K, factory, &file, path)
                .value()
                .origin,
            LensOrigin::File);
  const camera::ArrayCalibration factory_file = file_of(
      {{"bm-1", {lens(k4K, 2200.0), camera::IntrinsicsSource::Factory}}});
  EXPECT_EQ(rig_lens(SessionMode::Extrinsics, "bm-1", k4K, factory,
                     &factory_file, path)
                .value()
                .origin,
            LensOrigin::File);
  // The runtime reads the factory lens at any size, so a calibrated one at
  // another size is refused for poses too; for lenses it is not the
  // camera's, which keeps its factory lens.
  expect_refused(
      rig_lens(SessionMode::Extrinsics, "bm-2", k4K, factory, &file, path)
          .status(),
      Code::InvalidArgument,
      "bm-2 has a factory lens and a calibrated one in " + path);
  EXPECT_EQ(rig_lens(SessionMode::Intrinsics, "bm-2", k4K, factory, &file, path)
                .value()
                .origin,
            LensOrigin::Factory);
}

// The mark a lens carries into the file the solve writes: a file lens keeps
// the file's, calibrated or factory; a factory lens, or a stand-in, is
// factory. A pose session's camera panel shows it.
TEST(AppCalibMode, ALensKeepsTheMarkOfWhereItCameFrom) {
  const camera::ArrayCalibration file = file_of(
      {{"bm-1", calibrated(k4K, 1702.0)},
       {"CL2A141000N", {lens(k4K, 2200.0), camera::IntrinsicsSource::Factory}},
       {"bm-2", calibrated({1920, 1080}, 851.0)}});
  const auto record = [](const std::string& id, LensOrigin origin) {
    capture::SessionCamera camera;
    camera.id = id;
    camera.size = k4K;
    camera.lens = origin;
    return camera;
  };
  using Source = camera::IntrinsicsSource;
  EXPECT_EQ(lens_source(record("bm-1", LensOrigin::File), &file),
            Source::Calibrated);
  EXPECT_EQ(lens_source(record("CL2A141000N", LensOrigin::File), &file),
            Source::Factory);
  EXPECT_EQ(lens_source(record("CL2A141000N", LensOrigin::Factory), &file),
            Source::Factory);
  EXPECT_EQ(lens_source(record("bm-2", LensOrigin::StandIn), &file),
            Source::Factory);
  EXPECT_EQ(lens_source(record("bm-1", LensOrigin::Factory), &file),
            Source::Factory);
  EXPECT_EQ(lens_source(record("bm-1", LensOrigin::File), nullptr),
            Source::Factory);

  // The camera panel's lens line says the mark, and a file's lens as such;
  // in a lens session the Lenses panel says it instead.
  const auto line = [&](SessionMode mode, const char* id, LensOrigin origin) {
    return lens_line(mode, record(id, origin), &file).value();
  };
  EXPECT_EQ(line(SessionMode::Extrinsics, "bm-1", LensOrigin::File),
            "calibrated (file)");
  EXPECT_EQ(line(SessionMode::Extrinsics, "CL2A141000N", LensOrigin::File),
            "factory (file)");
  EXPECT_EQ(line(SessionMode::Extrinsics, "CL2A141000N", LensOrigin::Factory),
            "factory");
  EXPECT_EQ(line(SessionMode::Extrinsics, "bm-2", LensOrigin::StandIn),
            "missing");
  EXPECT_EQ(line(SessionMode::Intrinsics, "bm-1", LensOrigin::File), "");
}

// Where each camera's lens comes from, as the Setup panel says it: a file's
// keeps its mark, the driver's is factory, and none is a stand-in for lenses
// and missing for poses. take_lens records it.
TEST(AppCalibMode, EachLensSaysWhereItComesFrom) {
  const std::string path = "lenses.json";
  const camera::ArrayCalibration file =
      file_of({{"bm-1", calibrated(k4K, 1702.0)},
               {"CL2A141000N",
                {lens(k4K, 2200.0), camera::IntrinsicsSource::Factory}}});
  const auto record = [](const std::string& id, bool factory) {
    capture::SessionCamera camera;
    camera.id = id;
    camera.size = k4K;
    if (factory) camera.color = lens(k4K, 2100.0);
    camera.lens_file = "stale.json";
    return camera;
  };
  struct Case {
    SessionMode mode;
    const char* id;
    bool factory;
    LensOrigin origin;
    LensStatus status;
    double fx;
  };
  const Case cases[] = {
      {SessionMode::Intrinsics, "bm-1", false, LensOrigin::File,
       LensStatus::Calibrated, 1702.0},
      {SessionMode::Extrinsics, "bm-1", false, LensOrigin::File,
       LensStatus::Calibrated, 1702.0},
      {SessionMode::Extrinsics, "CL2A141000N", true, LensOrigin::File,
       LensStatus::Factory, 2200.0},
      {SessionMode::Extrinsics, "CL3", true, LensOrigin::Factory,
       LensStatus::Factory, 2100.0},
      {SessionMode::Intrinsics, "bm-2", false, LensOrigin::StandIn,
       LensStatus::StandIn, 1920.0},
      {SessionMode::Extrinsics, "bm-2", false, LensOrigin::StandIn,
       LensStatus::Missing, 1920.0},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(std::string(c.id) +
                 (c.mode == SessionMode::Intrinsics ? " lenses" : " poses"));
    capture::SessionCamera camera = record(c.id, c.factory);
    const camera::CameraModel taken =
        take_lens(c.mode, &file, path, camera).value();
    EXPECT_EQ(taken.intrinsics.fx, c.fx);
    EXPECT_EQ(camera.lens, c.origin);
    EXPECT_EQ(camera.lens_file,
              c.origin == LensOrigin::File ? path : std::string());
    EXPECT_EQ(lens_status(c.mode, camera, &file), c.status);
  }
  EXPECT_STREQ(to_string(LensStatus::Calibrated), "calibrated");
  EXPECT_STREQ(to_string(LensStatus::Factory), "factory");
  EXPECT_STREQ(to_string(LensStatus::StandIn), "stand-in");
  EXPECT_STREQ(to_string(LensStatus::Missing), "missing");
  EXPECT_STREQ(to_string(static_cast<LensStatus>(9)), "unknown");
  // Refused, the record is unchanged.
  capture::SessionCamera refused = record("bm-1", true);
  expect_refused(
      take_lens(SessionMode::Extrinsics, &file, path, refused).status(),
      Code::InvalidArgument,
      "bm-1 has a factory lens and a calibrated one in " + path);
  EXPECT_EQ(refused.lens, LensOrigin::Factory);
  EXPECT_EQ(refused.lens_file, "stale.json");
}

// A lens fit starts from a real lens or OpenCV's own: the camera's file
// lens, another camera's at the same size, its factory lens; never the
// stand-in.
TEST(AppCalibMode, AFitStartsFromARealLensNeverTheStandIn) {
  const camera::ArrayCalibration file =
      file_of({{"bm-1", calibrated(k4K, 1702.0)},
               {"bm-2", calibrated({1920, 1080}, 851.0)}});
  const std::optional<camera::CameraModel> factory = lens(k4K, 2200.0);
  EXPECT_EQ(lens_start("bm-1", k4K, factory, &file)->intrinsics.fx, 1702.0);
  // Its own lens at another size: another camera's at this size.
  EXPECT_EQ(lens_start("bm-2", k4K, std::nullopt, &file)->intrinsics.fx,
            1702.0);
  EXPECT_EQ(lens_start("bm-3", k4K, factory, &file)->intrinsics.fx, 1702.0);
  EXPECT_EQ(
      lens_start("bm-2", {1920, 1080}, std::nullopt, &file)->intrinsics.fx,
      851.0);
  // No file: the factory's; nothing real: none, OpenCV's own alone.
  EXPECT_EQ(lens_start("bm-1", k4K, factory, nullptr)->intrinsics.fx, 2200.0);
  EXPECT_FALSE(lens_start("bm-1", k4K, std::nullopt, nullptr).has_value());
  EXPECT_FALSE(lens_start("bm-1", {960, 540}, std::nullopt, &file).has_value());
  EXPECT_FALSE(
      lens_start("bm-1", {960, 540}, lens(k4K, 2200.0), nullptr).has_value());
}

// Each mode's board and the boards it takes: a lens session seeds the
// checkerboard and takes either kind with a lens view's corners, which the
// rig's 5 x 3 ChArUco board, 8 corners, never gives; a pose session seeds and
// takes ChArUco.
TEST(AppCalibMode, EachModeSeedsAndTakesItsBoards) {
  EXPECT_EQ(board_seed(SessionMode::Intrinsics), legacy_checkerboard());
  EXPECT_EQ(board_seed(SessionMode::Extrinsics), legacy_femto_board());
  const viz::BoardRule lenses = board_rule(SessionMode::Intrinsics);
  EXPECT_TRUE(lenses.checkerboard);
  EXPECT_EQ(lenses.min_corners, solve::LensOptions{}.min_corners);
  const viz::BoardRule poses = board_rule(SessionMode::Extrinsics);
  EXPECT_FALSE(poses.checkerboard);
  EXPECT_EQ(poses.min_corners, 0);

  EXPECT_TRUE(
      viz::BoardEditor::create(legacy_checkerboard(), false, lenses).ok());
  expect_refused(
      viz::BoardEditor::create(legacy_femto_board(), false, lenses).status(),
      Code::InvalidArgument,
      "a lens view needs 12 corners, and this board has 8 (4 x 2 inner "
      "corners), so it would never give one: take the checkerboard");
  // Either kind, large enough.
  Target charuco = legacy_femto_board();
  charuco.cols = 6;  // 7 x 5 squares
  charuco.rows = 4;
  ASSERT_TRUE(check_target(charuco).ok());
  EXPECT_TRUE(viz::BoardEditor::create(charuco, false, lenses).ok());
  EXPECT_TRUE(
      viz::BoardEditor::create(legacy_femto_board(), false, poses).ok());
  expect_refused(
      viz::BoardEditor::create(legacy_checkerboard(), false, poses).status(),
      Code::Unsupported, "the board must be ChArUco, not checkerboard");
}

// One view is a lens capture; poses keep the rule's defaults.
TEST(AppCalibMode, OneViewIsALensCapture) {
  const quality::CaptureRuleOptions lenses =
      capture_rule(SessionMode::Intrinsics);
  const quality::CaptureRuleOptions poses =
      capture_rule(SessionMode::Extrinsics);
  const quality::CaptureRuleOptions defaults;
  EXPECT_EQ(lenses.min_views, 1);
  EXPECT_EQ(poses.min_views, defaults.min_views);
  EXPECT_EQ(poses.min_views, 2);
  for (const quality::CaptureRuleOptions& rule : {lenses, poses}) {
    EXPECT_EQ(rule.still_px, defaults.still_px);
    EXPECT_EQ(rule.timeout, defaults.timeout);
    EXPECT_EQ(rule.spread_warn_ms, defaults.spread_warn_ms);
  }
}

// Calibrate may run once a camera's lens is ready, or once the session's
// links are and every camera has a real lens.
TEST(AppCalibMode, CalibrateWaitsForItsModesReadiness) {
  quality::LensReadiness ready;
  ready.ready = true;
  const quality::LensReadiness waiting;
  quality::SessionSummary linked;
  linked.ready = true;
  using S = LensStatus;
  const std::vector<S> stand_ins{S::StandIn, S::StandIn};
  EXPECT_FALSE(session_ready(SessionMode::Intrinsics, std::nullopt, {}, {}));
  EXPECT_FALSE(session_ready(SessionMode::Intrinsics, linked,
                             {waiting, waiting}, stand_ins));
  EXPECT_TRUE(session_ready(SessionMode::Intrinsics, std::nullopt,
                            {waiting, ready}, stand_ins));
  const std::vector<S> real{S::Calibrated, S::Factory};
  EXPECT_FALSE(
      session_ready(SessionMode::Extrinsics, std::nullopt, {ready}, real));
  EXPECT_FALSE(session_ready(SessionMode::Extrinsics, quality::SessionSummary{},
                             {ready}, real));
  EXPECT_TRUE(session_ready(SessionMode::Extrinsics, linked, {}, real));
  // A camera without a real lens holds poses back, however linked.
  EXPECT_FALSE(session_ready(SessionMode::Extrinsics, linked, {},
                             {S::Calibrated, S::Missing}));
  EXPECT_FALSE(session_ready(SessionMode::Extrinsics, linked, {},
                             {S::StandIn, S::Calibrated}));
}

// Calibrate is never silent: pressed with nothing ready, each camera says
// what it lacks and what to do, in either mode.
TEST(AppCalibMode, SaysPerCameraWhyCalibrateIsHeldBack) {
  const auto record = [](const std::string& id) {
    capture::SessionCamera camera;
    camera.id = id;
    camera.size = k4K;
    return camera;
  };
  const std::vector<capture::SessionCamera> pair{record("bm-5"),
                                                 record("bm-6")};
  const quality::SessionOptions options;
  using S = LensStatus;
  const auto said = [](const std::vector<NotReady>& why) {
    std::vector<std::string> lines;
    for (const NotReady& one : why)
      lines.push_back(one.camera + ": " + one.why);
    return lines;
  };

  // Lenses: before the first capture, then each camera's shortfall, as the
  // owner met it; nothing once a camera is ready.
  const std::vector<S> stand_ins{S::StandIn, S::StandIn};
  EXPECT_EQ(said(calibrate_blockers(SessionMode::Intrinsics, pair, stand_ins,
                                    "", std::nullopt, {}, options)
                     .value()),
            (std::vector<std::string>{": no captures yet"}));
  quality::LensReadiness bm5;
  bm5.unmet = "reach 0.83 toward the top-left corner, needs 0.9: capture there";
  quality::LensReadiness bm6;
  bm6.unmet = "22 views of 30";
  EXPECT_EQ(said(calibrate_blockers(SessionMode::Intrinsics, pair, stand_ins,
                                    "", std::nullopt, {bm5, bm6}, options)
                     .value()),
            (std::vector<std::string>{
                "bm-5: reach 0.83 toward the top-left corner, needs 0.9: "
                "capture there",
                "bm-6: 22 views of 30"}));
  quality::LensReadiness ready;
  ready.ready = true;
  EXPECT_TRUE(calibrate_blockers(SessionMode::Intrinsics, pair, stand_ins, "",
                                 std::nullopt, {bm5, ready}, options)
                  ->empty());

  // Poses: each camera without a lens, naming the file and the fix; no
  // captures; then each link short of the solve's.
  const std::string path = "bm_pair/lenses.json";
  EXPECT_EQ(said(calibrate_blockers(SessionMode::Extrinsics, pair,
                                    {S::Calibrated, S::Missing}, path,
                                    std::nullopt, {}, options)
                     .value()),
            (std::vector<std::string>{
                "bm-6: no lens at 3840x2160 in bm_pair/lenses.json: load a "
                "lens file in Setup, or fit it in Lenses mode",
                ": no captures yet"}));
  quality::SessionSummary summary;
  summary.cameras.resize(2);
  summary.cameras[0].linked = true;  // the reference
  summary.cameras[1].linked = true;
  summary.cameras[1].link = quality::PreviewLink{0, options.ready_from - 1, {}};
  const std::vector<S> real{S::Calibrated, S::Calibrated};
  EXPECT_EQ(
      said(calibrate_blockers(SessionMode::Extrinsics, pair, real, path,
                              summary, {}, options)
               .value()),
      (std::vector<std::string>{
          "bm-6: shares " + std::to_string(options.ready_from - 1) +
          " captures with bm-5, needs " + std::to_string(options.ready_from) +
          ": capture boards both see"}));
  summary.cameras[1].linked = false;
  summary.cameras[1].link.reset();
  EXPECT_EQ(said(calibrate_blockers(SessionMode::Extrinsics, pair, real, "",
                                    summary, {}, options)
                     .value()),
            (std::vector<std::string>{"bm-6: no captures it shares with "
                                      "another camera: capture boards it and "
                                      "another see"}));
  EXPECT_EQ(said(calibrate_blockers(SessionMode::Extrinsics, pair,
                                    {S::Missing, S::Calibrated}, "", summary,
                                    {}, options)
                     .value())
                .front(),
            "bm-5: no lens at 3840x2160 (no lens file): load a lens file in "
            "Setup, or fit it in Lenses mode");
  summary.ready = true;
  EXPECT_TRUE(calibrate_blockers(SessionMode::Extrinsics, pair, real, path,
                                 summary, {}, options)
                  ->empty());
  expect_refused(calibrate_blockers(SessionMode::Extrinsics, pair, {S::Missing},
                                    path, summary, {}, options)
                     .status(),
                 Code::InvalidArgument,
                 "calibrate_blockers: 2 cameras, 1 lens");
}

// A pair's camera, opened with no calibration, takes its lens from the
// scenario's file and records the file; one not there takes the stand-in, a
// pose session's lens missing.
TEST(AppCalibMode, RecordsEachStereoCameraWithItsLens) {
  const std::string path = "bm_pair/array_calibration.json";
  RigScenario scenario;
  scenario.mode = SessionMode::Intrinsics;
  scenario.driver = "decklink";
  scenario.calibration = std::make_shared<const camera::ArrayCalibration>(
      file_of({{"bm-1", calibrated(k4K, 1702.0)}}));
  OpenedSource opened;
  for (const char* id : {"bm-1", "bm-2"}) {
    capture::SessionCamera camera;
    camera.id = id;
    camera.size = k4K;
    camera.input = "SDI 2";
    ASSERT_TRUE(add_stereo_camera(camera, scenario, path, opened).ok());
  }
  ASSERT_EQ(opened.cameras.size(), 2u);
  ASSERT_EQ(opened.lenses.size(), 2u);
  EXPECT_EQ(opened.cameras[0].lens, LensOrigin::File);
  EXPECT_EQ(opened.cameras[0].lens_file, path);
  EXPECT_EQ(opened.cameras[0].input, "SDI 2");
  EXPECT_EQ(opened.lenses[0].intrinsics.fx, 1702.0);
  EXPECT_EQ(opened.cameras[1].lens, LensOrigin::StandIn);
  EXPECT_EQ(opened.cameras[1].lens_file, "");
  EXPECT_EQ(opened.lenses[1].intrinsics.fx, stand_in_lens(k4K).intrinsics.fx);

  scenario.mode = SessionMode::Extrinsics;
  capture::SessionCamera bm2;
  bm2.id = "bm-2";
  bm2.size = k4K;
  ASSERT_TRUE(add_stereo_camera(bm2, scenario, path, opened).ok());
  ASSERT_EQ(opened.cameras.size(), 3u);
  EXPECT_EQ(opened.cameras[2].lens, LensOrigin::StandIn);
  EXPECT_EQ(lens_status(SessionMode::Extrinsics, opened.cameras[2],
                        scenario.calibration.get()),
            LensStatus::Missing);
}

// The window takes every part a mode sets from one place: the session's mode
// and file, the capture rule, its own mode, and the Board window's board and
// the boards it takes. Each part starts at the other mode's.
TEST(AppCalibMode, SetsEveryPartToTheScenariosMode) {
  for (const SessionMode mode :
       {SessionMode::Intrinsics, SessionMode::Extrinsics}) {
    const bool lenses = mode == SessionMode::Intrinsics;
    RigScenario scenario;
    scenario.mode = mode;
    scenario.driver = "decklink";
    scenario.calibration = std::make_shared<const camera::ArrayCalibration>(
        file_of({{"bm-1", calibrated(k4K, 1702.0)}}));
    scenario.pairs = {{"bm-1", "bm-2"}};
    scenario.pair_rows.rectified = {960, 544};
    CaptureSetup setup;
    setup.mode = lenses ? SessionMode::Extrinsics : SessionMode::Intrinsics;
    LiveOptions live;
    live.rule.min_views = 7;
    RigMode window;
    window.intrinsic = !lenses;
    const viz::BoardEditor editor =
        apply_mode(scenario, setup, live, window).value();
    EXPECT_EQ(setup.mode, mode);
    EXPECT_EQ(setup.calibration, scenario.calibration);
    ASSERT_EQ(setup.pairs.size(), 1u);
    EXPECT_EQ(setup.pairs[0].left, "bm-1");
    EXPECT_EQ(setup.pairs[0].right, "bm-2");
    EXPECT_EQ(setup.pair_rows.rectified.width, 960u);
    EXPECT_EQ(setup.pair_rows.rectified.height, 544u);
    EXPECT_EQ(live.rule.min_views, capture_rule(mode).min_views);
    EXPECT_EQ(window.intrinsic, lenses);
    EXPECT_EQ(editor.board(), board_seed(mode));
    EXPECT_EQ(editor.rule().checkerboard, lenses);
    EXPECT_EQ(editor.rule().min_corners, board_rule(mode).min_corners);
    EXPECT_FALSE(editor.square_measured());
    EXPECT_FALSE(editor.locked());
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::app
