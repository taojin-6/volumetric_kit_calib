// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The stable API (README.md's table) as a program built on calib uses it: the
// sample mains, including only the table's headers (and the core's), run on
// synthetic data: their pictures are the test's own, the rest of main 2 is
// examples/02_lens_from_images. A break in a call they make fails this build;
// the rest of the table is held in review (DECISIONS.md, 2026-11-01).

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/detect/board.hpp"
#include "volumetric_kit/calib/quality/lens_gate.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/base/result.hpp"

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
              verdict.pass ? "pass" : verdict.failed.front().c_str());
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

}  // namespace
