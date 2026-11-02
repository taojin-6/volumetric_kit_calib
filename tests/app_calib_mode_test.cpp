// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's per-mode table: the mode an operation asks for; each camera's
// live lens (the file's at exactly its size, the factory's, the stand-in for
// lenses alone) and what extrinsic mode refuses; where a lens fit starts,
// never the stand-in; the board each mode seeds and takes, a board too small
// for a lens view refused, saying why; the capture rule; and when Calibrate
// may run. Host-only.

#include "volumetric_kit/calib/app/calib_mode.hpp"

#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "volumetric_kit/calib/solve/lens.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = sensor::camera;
using capture::LensOrigin;
using capture::SessionMode;
using Code = core::Status::Code;

constexpr camera::ImageSize k4K{3840, 2160};

void refused(const core::Status& status, Code code, const std::string& text) {
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.domain(), code) << status.message();
  EXPECT_NE(status.message().find(text), std::string::npos)
      << status.message() << " lacks " << text;
}

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
  refused(session_mode(Operation::Fuse).status(), Code::InvalidArgument,
          "operation: fuse is not one calib_rig runs");
  refused(session_mode(static_cast<Operation>(99)).status(),
          Code::InvalidArgument, "operation: 99 is not one calib_rig runs");
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
  refused(
      rig_lens(SessionMode::Extrinsics, "bm-2", k4K, std::nullopt, &file, path)
          .status(),
      Code::InvalidArgument,
      "bm-2 has no lens at 3840x2160 in bm_pair/array_calibration.json: "
      "run intrinsic mode");
  refused(
      rig_lens(SessionMode::Extrinsics, "bm-2", k4K, std::nullopt, nullptr, "")
          .status(),
      Code::InvalidArgument, "bm-2 has no lens at 3840x2160 (no file)");
  // A factory camera with a calibrated lens in the file, for poses: the
  // runtime would read the factory's. A factory-sourced one is the same lens.
  refused(rig_lens(SessionMode::Extrinsics, "bm-1", k4K, factory, &file, path)
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
  refused(
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
  refused(
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

}  // namespace
}  // namespace volumetric_kit::calib::app
