// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's per-mode table: the mode an operation asks for; each camera's
// live lens (the file's at exactly its size, the factory's, the stand-in for
// lenses alone) and what extrinsic mode refuses; the mark a lens carries into
// the file the solve writes; where a lens fit starts,
// never the stand-in; the board each mode seeds and takes, a board too small
// for a lens view refused, saying why; the capture rule; and when Calibrate
// may run; a stereo pair's camera records and lenses; and every part of the
// window a mode sets, set from one place. Host-only.

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
  // A lens at another size is not the camera's: lenses fall to the
  // stand-in, poses stop, naming the file.
  const RigLens other =
      rig_lens(SessionMode::Intrinsics, "bm-2", k4K, std::nullopt, &file, path)
          .value();
  EXPECT_EQ(other.origin, LensOrigin::StandIn);
  EXPECT_EQ(other.model.size.width, 3840u);
  EXPECT_EQ(other.model.intrinsics.fx, 1920.0);
  EXPECT_EQ(other.model.intrinsics.fy, 1920.0);
  EXPECT_EQ(other.model.intrinsics.cx, 1919.5);
  EXPECT_EQ(other.model.intrinsics.cy, 1079.5);
  EXPECT_EQ(other.model.distortion.k1, 0.0);
  expect_refused(
      rig_lens(SessionMode::Extrinsics, "bm-2", k4K, std::nullopt, &file, path)
          .status(),
      Code::InvalidArgument,
      "bm-2 has no lens at 3840x2160 in bm_pair/array_calibration.json: "
      "run intrinsic mode");
  expect_refused(
      rig_lens(SessionMode::Extrinsics, "bm-2", k4K, std::nullopt, nullptr, "")
          .status(),
      Code::InvalidArgument,
      "bm-2 has no lens at 3840x2160 (no source.calibration): run intrinsic "
      "mode");
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
  EXPECT_EQ(line(SessionMode::Intrinsics, "bm-1", LensOrigin::File), "");
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
// links are.
TEST(AppCalibMode, CalibrateWaitsForItsModesReadiness) {
  quality::LensReadiness ready;
  ready.ready = true;
  const quality::LensReadiness waiting;
  quality::SessionSummary linked;
  linked.ready = true;
  EXPECT_FALSE(session_ready(SessionMode::Intrinsics, std::nullopt, {}));
  EXPECT_FALSE(
      session_ready(SessionMode::Intrinsics, linked, {waiting, waiting}));
  EXPECT_TRUE(
      session_ready(SessionMode::Intrinsics, std::nullopt, {waiting, ready}));
  EXPECT_FALSE(session_ready(SessionMode::Extrinsics, std::nullopt, {ready}));
  EXPECT_FALSE(session_ready(SessionMode::Extrinsics, quality::SessionSummary{},
                             {ready}));
  EXPECT_TRUE(session_ready(SessionMode::Extrinsics, linked, {}));
}

// A pair's camera, opened with no calibration, takes its lens from the
// scenario's file and records the file; one not there takes the stand-in for
// lenses, and stops a pose session.
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
  expect_refused(add_stereo_camera(bm2, scenario, path, opened),
                 Code::InvalidArgument,
                 "bm-2 has no lens at 3840x2160 in " + path);
  EXPECT_EQ(opened.cameras.size(), 2u);
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
