// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The stable API (README.md's table) as a program built on calib uses it: the
// sample mains, including only the table's headers (and the core's), run on
// synthetic data: their pictures and views are the test's own, the rest of
// main 2 is examples/02_lens_from_images. A break in a call they make fails
// this build; the rest of the table is held in review (DECISIONS.md,
// 2026-11-01).

// First, before anything that could stand in for an include one of them lacks.
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/detect/board.hpp"
#include "volumetric_kit/calib/quality/calibration_metrics.hpp"
#include "volumetric_kit/calib/quality/lens_gate.hpp"
#include "volumetric_kit/calib/quality/pair_row_check.hpp"
#include "volumetric_kit/calib/quality/results.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"
#include "volumetric_kit/calib/solve/world_tag.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

// The test's own pictures, as views: no part of the sample mains.
#include "synthetic_rig.hpp"

namespace calib = volumetric_kit::calib;
namespace core = volumetric_kit::core;
namespace camera = volumetric_kit::sensor::camera;

namespace {

// Main 1, entering a board: what the Board window does with the user's
// fields.
core::Status enter_board(std::size_t& corners) {
  calib::Target board = calib::legacy_checkerboard();  // 13 x 8 inner corners
  board.square_size_m = 0.01994;                       // measured
  VKC_TRY(calib::check_target(board));
  VKC_ASSIGN(const calib::compute::Board b,
             calib::compute::Board::create(board));
  corners = b.object_points().size();
  std::printf("%zu corners\n", corners);  // 104
  return {};
}

TEST(Api, EntersABoard) {
  std::size_t corners = 0;
  const core::Status entered = enter_board(corners);
  ASSERT_TRUE(entered.ok()) << entered.message();
  EXPECT_EQ(corners, 104u);
}

// Main 2, a lens from saved pictures, but for the pictures: a blank one goes
// through the detector, and `views` is what the others would have given.
core::Result<calib::quality::ResultFiles> lens_from_views(
    const char* id, const calib::compute::Board& board, camera::ImageSize size,
    const std::vector<std::uint8_t>& blank,
    std::vector<calib::Observation> views) {
  VKC_ASSIGN(calib::Detection found,
             calib::detect::detect_board(
                 {size.width, 1, size.width, blank.data()}, board));
  found.observation.camera_id = id;
  found.observation.frame_id = 1000;
  views.push_back(std::move(found.observation));
  VKC_ASSIGN(calib::solve::LensFit fit,
             calib::solve::solve_lens(id, size, board, views));
  VKC_ASSIGN(calib::quality::LensVerdict verdict,
             calib::quality::judge_lens(std::move(fit)));
  std::printf("%s: hold-out %.2f px, %s\n", id, verdict.fit.holdout_rms_px,
              verdict.pass ? "pass" : "fail");
  for (const std::string& why : verdict.failed) {
    std::printf("  %s\n", why.c_str());
  }
  return calib::quality::format_lens_results({verdict}, {}, nullptr);
}

TEST(Api, FitsALens) {
  const calib::test::FisheyeTruth truth;
  calib::test::FisheyeCapture capture;
  capture.noise_px = 0.15;
  const calib::compute::Board board = calib::test::lab_checkerboard();
  const core::Result<calib::quality::ResultFiles> files = lens_from_views(
      "bm-1", board, truth.size, std::vector<std::uint8_t>(truth.size.width),
      calib::test::fisheye_views(board, truth, capture));
  ASSERT_TRUE(files.ok()) << files.status().message();
  EXPECT_FALSE(files->array_calibration.empty());
}

// Main 3, a pair's poses: what extrinsic mode's Calibrate does for two
// cameras.
core::Result<calib::quality::ResultFiles> pose_pair(
    const calib::compute::Board& board,            // the 5 x 3 ChArUco
    const std::vector<calib::Observation>& views,  // both, frame_id = capture
    const std::vector<calib::solve::TagView>& tags,
    const camera::ArrayCalibration& lenses,  // intrinsic mode's file
    camera::ImageSize ffs_size) {            // the runtime's, from its config
  namespace solve = calib::solve;
  namespace quality = calib::quality;
  std::vector<solve::RigCamera> cameras;
  for (const char* id : {"bm-1", "bm-2"}) {
    const camera::SensorCalibration* entry = camera::find_sensor(lenses, id);
    if (entry == nullptr || !entry->color) {
      return core::Status::invalid_argument(std::string(id) +
                                            " has no lens: run intrinsic mode");
    }
    cameras.push_back(
        {id, entry->color->model, camera::IntrinsicsSource::Calibrated});
  }
  VKC_ASSIGN(const solve::InitialRig start,
             solve::initialise(cameras, board, views, 0));
  VKC_ASSIGN(const solve::RigSolution solution,
             solve::bundle_adjust(cameras, board, views, start));
  VKC_ASSIGN(
      const quality::CalibrationMetrics metrics,
      quality::calibration_metrics(cameras, board, views, start, solution));
  quality::ResultWorld world;
  VKC_ASSIGN(world.origin, solve::origin_tag_frame(cameras, solution, tags,
                                                   calib::kOriginTag));
  VKC_ASSIGN(const solve::AnchoredRig rig,
             solve::anchor_rig(solution, world.origin));
  quality::PairRowCheckOptions options;
  options.rectified = ffs_size;
  VKC_ASSIGN(const quality::PairRowCheck rows,
             quality::check_pair_rows(rig.calibration, "bm-1", "bm-2", views,
                                      options));
  std::printf("bm-1 | bm-2: %s, rows p95 %.2f px %s\n",
              quality::to_string(rows.verdict), rows.dy_p95_px,
              rows.reason.c_str());
  return quality::format_results(cameras, solution, metrics, nullptr, {}, world,
                                 {rows});
}

TEST(Api, PosesAPair) {
  const std::vector<calib::test::SyntheticCamera> pair =
      calib::test::lab_pair();
  const calib::compute::Board board = calib::test::femto_board();
  calib::test::ObserveOptions noise;
  noise.noise_px = 0.15;
  noise.portable = true;
  const std::vector<calib::Observation> views = calib::test::observe(
      pair, board, calib::test::view_plan(pair, 0, board), noise);
  std::vector<calib::solve::TagView> tags;
  for (const calib::test::MarkerSighting& seen : calib::test::observe_marker(
           pair, calib::kOriginTag.id, calib::test::floor_tag_corners(), 3,
           noise)) {
    const calib::test::SyntheticCamera& cam = pair[seen.camera];
    tags.push_back({cam.id, seen.capture, cam.model.size, seen.marker});
  }
  camera::ArrayCalibration lenses;  // the true lenses, as intrinsic mode's
  for (const calib::test::SyntheticCamera& cam : pair) {
    camera::SensorCalibration& sensor = lenses.sensors.emplace_back();
    sensor.id = cam.id;
    sensor.color = camera::CameraCalibration{
        cam.model, camera::IntrinsicsSource::Calibrated};
  }
  const core::Result<calib::quality::ResultFiles> files =
      pose_pair(board, views, tags, lenses, {960, 544});
  ASSERT_TRUE(files.ok()) << files.status().message();
  const std::string pairs =
      files->report.substr(files->report.find("\"pairs\": ["));
  EXPECT_NE(pairs.find("\"verdict\": \"pass\""), std::string::npos) << pairs;
}

}  // namespace
