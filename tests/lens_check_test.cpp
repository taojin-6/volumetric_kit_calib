// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A lens checked on views of either board, each posed through it held fixed:
// the true lens over views spread across the image is good, on the lab's
// checkerboard and on the rig's 5 x 3 ChArUco board of 8 corners alike; a lens
// bent at one edge reads suspect there and nowhere else, its RMS within the
// limit; a lens off its views reads suspect where its refit reads good; views
// only in the centre are insufficient, naming the parts to capture, and a
// limit passed comes first; noise alone does not read suspect; and what it
// poses, drops and refuses.

#include "volumetric_kit/calib/quality/lens_check.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "result_checks.hpp"
#include "synthetic_rig.hpp"
#include "synthetic_scene.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = sensor::camera;
using Verdict = LensCheck::Verdict;
using test::expect_refused;
using test::lab_lens;
using test::value_of;

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr camera::ImageSize kSize{3840, 2160};

const compute::Board& checkerboard() {
  static const compute::Board once = test::lab_checkerboard();
  return once;
}

const compute::Board& charuco() {
  static const compute::Board once = test::femto_board();
  return once;
}

// A part of the image, as fractions of its width and height.
struct Area {
  double left = 0.0;
  double right = 1.0;
  double top = 0.0;
  double bottom = 1.0;
};

// Placements of `board` before a camera with `lens`, as a person spreads views
// over `area`: its centre aimed at each point of a `cols` x `rows` grid over
// it, at each distance (near enough to fill a cell, far enough to fit in a
// corner), tilted 25 degrees one way and then the other and rolled 15 on every
// other pair; one with a corner within 3 px of the image's edge, or behind the
// camera, is left out.
std::vector<test::Placement> spread(const compute::Board& board,
                                    const camera::CameraModel& lens, int cols,
                                    int rows, const Area& area = {}) {
  const std::vector<double> distances = &board == &charuco()
                                            ? std::vector<double>{1.5, 2.2, 3.0}
                                            : std::vector<double>{0.6, 0.9};
  const compute::CvCamera cam = value_of(compute::cv_camera(lens));
  std::vector<test::Placement> plan;
  int turn = 0;
  for (int j = 0; j < rows; ++j) {
    for (int i = 0; i < cols; ++i) {
      const double x = area.left + (i + 0.5) / cols * (area.right - area.left);
      const double y = area.top + (j + 0.5) / rows * (area.bottom - area.top);
      const cv::Point2d ray = value_of(compute::undistort_points(
          cam, {cv::Point2d(x * lens.size.width - 0.5,
                            y * lens.size.height - 0.5)}))[0];
      const double off_axis = std::atan(std::hypot(ray.x, ray.y)) / kDeg;
      const double azimuth = std::atan2(ray.y, ray.x) / kDeg;
      for (const double distance : distances) {
        const double tilt = turn % 2 == 0 ? 25.0 : -25.0;
        const double roll = turn % 4 < 2 ? 0.0 : 15.0;
        ++turn;
        const camera::Mat4d to_camera =
            test::facing(board, off_axis, azimuth, distance, roll, tilt);
        bool inside = true;
        for (const cv::Point3d& p : board.object_points()) {
          inside =
              inside && (to_camera * glm::dvec4(p.x, p.y, p.z, 1.0)).z > 0.1;
        }
        for (const cv::Point2d& p :
             test::project(lens, to_camera, board.object_points())) {
          inside = inside && p.x >= 3.0 && p.y >= 3.0 &&
                   p.x <= lens.size.width - 4.0 &&
                   p.y <= lens.size.height - 4.0;
        }
        if (inside) plan.push_back({to_camera, {0}});
      }
    }
  }
  return plan;
}

// What camera "bm-1" with `lens` detects of `plan`, with noise the same on
// every standard library.
std::vector<Observation> observe(const compute::Board& board,
                                 const camera::CameraModel& lens,
                                 const std::vector<test::Placement>& plan,
                                 double noise_px, std::uint32_t seed = 1) {
  test::ObserveOptions options;
  options.noise_px = noise_px;
  options.seed = seed;
  options.portable = true;
  return test::observe({{"bm-1", lens, camera::Mat4d(1.0)}}, board, plan,
                       options);
}

// Placements of `board` over the whole image of `lens`: 110 of the
// checkerboard, 377 of the ChArUco board through the lab's lens.
std::vector<test::Placement> spread_plan(const compute::Board& board,
                                         const camera::CameraModel& lens) {
  return &board == &charuco() ? spread(board, lens, 16, 9)
                              : spread(board, lens, 12, 7);
}

// Views of `board` over the whole image, through the lab's lens.
std::vector<Observation> spread_views(const compute::Board& board,
                                      double noise_px = 0.15,
                                      std::uint32_t seed = 1) {
  return observe(board, lab_lens(), spread_plan(board, lab_lens()), noise_px,
                 seed);
}

// `views` as a camera whose lens bends at its right: corners past three
// quarters of the way across image `px` further right than the lens says,
// rising smoothly over 100 px.
std::vector<Observation> bent_right(std::vector<Observation> views, double px) {
  const double from = 0.75 * kSize.width;
  for (Observation& view : views) {
    for (DetectedPoint& point : view.points) {
      const double t = std::clamp((point.pixel.x - from) / 100.0, 0.0, 1.0);
      point.pixel.x += px * t * t * (3.0 - 2.0 * t);
    }
  }
  return views;
}

bool says(const LensCheck& check, const std::string& text) {
  return check.reason.find(text) != std::string::npos;
}

// The farthest any judged cell's mean is past its limit, as their ratio.
double worst_cell(const LensCheck& check) {
  double worst = 0.0;
  for (const LensCell& cell : check.cells) {
    if (std::isnan(cell.limit_px)) continue;
    worst = std::max(worst, glm::length(cell.mean_px) / cell.limit_px);
  }
  return worst;
}

// Every corner of the views posed is kept or dropped, and every part of the
// image judged: the defaults' 9.
TEST(CheckLens, TheTrueLensIsGoodOnEitherBoard) {
  for (const compute::Board* board : {&checkerboard(), &charuco()}) {
    const std::vector<Observation> views = spread_views(*board);
    const LensCheck check =
        value_of(check_lens("bm-1", lab_lens(), *board, views));
    const std::size_t per_view = board->object_points().size();
    EXPECT_EQ(check.verdict, Verdict::Good) << check.reason;
    EXPECT_TRUE(check.reason.empty());
    EXPECT_EQ(check.camera, "bm-1");
    EXPECT_EQ(check.views, views.size());
    EXPECT_EQ(check.corners, views.size() * per_view);
    EXPECT_EQ(check.dropped, 0u);
    EXPECT_DOUBLE_EQ(check.bound_px, 3.0);  // the adjustment's floor
    EXPECT_GT(check.rms_px, 0.1);
    EXPECT_LT(check.rms_px, 0.25);
    EXPECT_GT(check.p95_px, check.rms_px);
    EXPECT_LT(worst_cell(check), 0.5);
    std::size_t in_cells = 0;
    for (const LensCell& cell : check.cells) {
      in_cells += static_cast<std::size_t>(cell.corners);
      EXPECT_EQ(std::isnan(cell.limit_px),
                cell.corners < check.options.min_cell_corners);
      EXPECT_EQ(std::isnan(cell.mean_px.x), std::isnan(cell.limit_px));
      if (!std::isnan(cell.limit_px)) {
        EXPECT_GE(cell.limit_px, check.options.max_cell_bias_px);
      }
    }
    EXPECT_EQ(in_cells, check.corners);
  }
}

// 3 px over 100 px at three quarters across: the cells there catch it, each
// view's pose taking up too little of it, while the RMS passes. Only the
// image's right is named.
TEST(CheckLens, ALensBentAtOneEdgeIsSuspectThereAlone) {
  for (const std::uint32_t seed : {1u, 2u, 3u}) {
    const LensCheck check = value_of(
        check_lens("bm-1", lab_lens(), checkerboard(),
                   bent_right(spread_views(checkerboard(), 0.15, seed), 3.0)));
    EXPECT_EQ(check.verdict, Verdict::Suspect) << seed;
    EXPECT_LE(check.rms_px, 0.5) << seed;
    EXPECT_EQ(check.dropped, 0u) << seed;
    EXPECT_TRUE(says(check, "the mean residual is")) << check.reason;
    EXPECT_FALSE(says(check, "RMS")) << check.reason;
    EXPECT_TRUE(says(check, "right (cell")) << check.reason;
    EXPECT_FALSE(says(check, "left")) << check.reason;
    for (std::size_t c = 0; c < check.cells.size(); ++c) {
      const LensCell& cell = check.cells[c];
      if (!(glm::length(cell.mean_px) > cell.limit_px)) continue;
      EXPECT_GE(c % kCoverageCols, 5u) << c;  // the right third
    }
  }
}

// k1 3% high: the views' poses cannot take it up, so held fixed the lens is
// suspect, by its RMS and its cells; fitted to the same views, the lens is
// good: they were never at fault.
TEST(CheckLens, ALensOffItsViewsIsSuspectWhereItsRefitIsGood) {
  const std::vector<Observation> views = spread_views(checkerboard());
  camera::CameraModel off = lab_lens();
  off.distortion.k1 *= 1.03;
  const LensCheck check =
      value_of(check_lens("bm-1", off, checkerboard(), views));
  EXPECT_EQ(check.verdict, Verdict::Suspect);
  EXPECT_TRUE(says(check, "the RMS through the lens is")) << check.reason;
  EXPECT_TRUE(says(check, "the mean residual is")) << check.reason;

  const solve::LensFit refit =
      value_of(solve::solve_lens("bm-1", kSize, checkerboard(), views, off));
  const LensCheck again =
      value_of(check_lens("bm-1", refit.camera.model, checkerboard(), views));
  EXPECT_EQ(again.verdict, Verdict::Good) << again.reason;
  EXPECT_LT(again.rms_px, 0.25);
}

// The rig's board, 8 corners a view, too few to fit a lens: each view is
// posed and judged. A lens pixels off past 56 degrees reads suspect on them
// too, by the corners it sends past the bound; a corner 8 px off costs its
// view that corner alone.
TEST(CheckLens, FiveByThreeCharucoViewsAreJudged) {
  ASSERT_EQ(charuco().object_points().size(), 8u);
  std::vector<Observation> views = spread_views(charuco());
  const LensCheck good =
      value_of(check_lens("bm-1", lab_lens(), charuco(), views));
  EXPECT_EQ(good.verdict, Verdict::Good) << good.reason;
  EXPECT_EQ(good.views, views.size());
  EXPECT_EQ(good.corners, 8 * views.size());
  for (const LensCell& cell : good.cells) {
    if (std::isnan(cell.limit_px)) continue;
    EXPECT_LE(glm::length(cell.mean_px), cell.limit_px);
  }

  camera::CameraModel off = lab_lens();
  off.distortion.k3 += 1e-3;  // 0.7 px at 45 degrees, 5 at 56, 23 at 65
  const LensCheck suspect = value_of(check_lens("bm-1", off, charuco(), views));
  EXPECT_EQ(suspect.verdict, Verdict::Suspect);
  EXPECT_TRUE(says(suspect, "corners are past the bound (3 px)"))
      << suspect.reason;
  EXPECT_GT(suspect.dropped, suspect.corners / 100);

  views[0].points[4].pixel.x += 8.0;
  const LensCheck glitch =
      value_of(check_lens("bm-1", lab_lens(), charuco(), views));
  EXPECT_EQ(glitch.verdict, Verdict::Good) << glitch.reason;
  EXPECT_EQ(glitch.views, views.size());
  EXPECT_EQ(glitch.dropped, 1u);
  EXPECT_EQ(glitch.corners, 8 * views.size() - 1);

  // 25 px pulls the pose of so few corners that its neighbours go past the
  // bound too, and the view with them: 8 corners of 3016.
  views[0].points[4].pixel.x += 17.0;
  const LensCheck lost =
      value_of(check_lens("bm-1", lab_lens(), charuco(), views));
  EXPECT_EQ(lost.verdict, Verdict::Good) << lost.reason;
  EXPECT_EQ(lost.views, views.size() - 1);
  EXPECT_EQ(lost.dropped, 8u);
}

// Boards aimed within the middle fifth: the image's sides and corners hold no
// judged cell, and are named with the most corners in one of their cells.
TEST(CheckLens, CentralViewsAreInsufficientNamingWhereToCapture) {
  const std::vector<Observation> views =
      observe(charuco(), lab_lens(),
              spread(charuco(), lab_lens(), 6, 4, {0.4, 0.6, 0.4, 0.6}), 0.15);
  const LensCheck check =
      value_of(check_lens("bm-1", lab_lens(), charuco(), views));
  EXPECT_EQ(check.verdict, Verdict::Insufficient);
  EXPECT_EQ(check.views, views.size());
  for (const char* part :
       {"top-left (", "top (", "top-right (", "left (", "right (",
        "bottom-left (", "bottom (", "bottom-right ("}) {
    EXPECT_TRUE(says(check, part)) << part << ": " << check.reason;
  }
  EXPECT_FALSE(says(check, "centre")) << check.reason;
  EXPECT_TRUE(says(check, "each of its 9 parts needs 30 in a cell"))
      << check.reason;
  EXPECT_TRUE(says(check, "capture the board there")) << check.reason;
}

// A limit passed is suspect wherever the views are: no capture elsewhere
// mends it.
TEST(CheckLens, ALimitPassedComesBeforeTooFewCorners) {
  const std::vector<Observation> views =
      observe(charuco(), lab_lens(),
              spread(charuco(), lab_lens(), 6, 4, {0.4, 0.6, 0.4, 0.6}), 0.15);
  LensCheckOptions options;
  options.max_rms_px = 0.05;
  const LensCheck check =
      value_of(check_lens("bm-1", lab_lens(), charuco(), views, options));
  EXPECT_EQ(check.verdict, Verdict::Suspect);
  EXPECT_TRUE(says(check, "the RMS through the lens is")) << check.reason;
  EXPECT_FALSE(says(check, "capture")) << check.reason;
}

// At half the width, half the pixels of RMS.
TEST(CheckLens, HoldsTheRmsToItsLimitScaledByWidth) {
  camera::CameraModel half = lab_lens();
  half.size = {1920, 1080};
  half.intrinsics = {half.intrinsics.fx / 2.0, half.intrinsics.fy / 2.0,
                     (half.intrinsics.cx + 0.5) / 2.0 - 0.5,
                     (half.intrinsics.cy + 0.5) / 2.0 - 0.5};
  const std::vector<Observation> views =
      observe(checkerboard(), half, spread_plan(checkerboard(), half), 0.1);
  const double rms =
      value_of(check_lens("bm-1", half, checkerboard(), views)).rms_px;
  LensCheckOptions options;
  options.max_rms_px = 2.0 * rms;
  const LensCheck at =
      value_of(check_lens("bm-1", half, checkerboard(), views, options));
  EXPECT_EQ(at.verdict, Verdict::Good) << at.reason;
  options.max_rms_px = std::nextafter(2.0 * rms, 0.0);
  const LensCheck past =
      value_of(check_lens("bm-1", half, checkerboard(), views, options));
  EXPECT_EQ(past.verdict, Verdict::Suspect);
  EXPECT_TRUE(says(past, "the RMS through the lens is")) << past.reason;
}

// Other cameras' views, and one of too few corners, are not posed; a view
// spoiled whole is left with no pose, its corners dropped; more than 1% of
// the corners dropped is suspect.
TEST(CheckLens, CountsWhatItPosesAndDrops) {
  std::vector<Observation> views = spread_views(charuco());
  const std::size_t posed = views.size();
  Observation other = views[0];
  other.camera_id = "bm-2";
  views.push_back(other);
  Observation few = views[1];
  few.frame_id = 10000;
  few.points.resize(5);
  views.push_back(few);
  for (std::size_t i = 0; i < views[2].points.size(); ++i) {
    views[2].points[i].pixel.y += i % 2 == 0 ? 20.0 : -20.0;
  }
  LensCheck check = value_of(check_lens("bm-1", lab_lens(), charuco(), views));
  EXPECT_EQ(check.verdict, Verdict::Good) << check.reason;
  EXPECT_EQ(check.views, posed - 1);
  EXPECT_EQ(check.dropped, 8u);
  EXPECT_EQ(check.corners, 8 * (posed - 1));

  // 1% of 3016 corners is 30.16: 3 views spoiled pass, 4 do not.
  for (std::size_t v = 3; v < 5; ++v) {
    for (std::size_t i = 0; i < views[v].points.size(); ++i) {
      views[v].points[i].pixel.y += i % 2 == 0 ? 20.0 : -20.0;
    }
  }
  ASSERT_EQ(8 * posed, 3016u);
  check = value_of(check_lens("bm-1", lab_lens(), charuco(), views));
  EXPECT_EQ(check.dropped, 24u);
  EXPECT_EQ(check.verdict, Verdict::Good) << check.reason;
  for (std::size_t i = 0; i < views[5].points.size(); ++i) {
    views[5].points[i].pixel.y += i % 2 == 0 ? 20.0 : -20.0;
  }
  check = value_of(check_lens("bm-1", lab_lens(), charuco(), views));
  EXPECT_EQ(check.dropped, 32u);
  EXPECT_EQ(check.verdict, Verdict::Suspect);
  EXPECT_TRUE(says(check, "32 of 3016 corners are past the bound"))
      << check.reason;
}

TEST(CheckLens, Refuses) {
  const std::vector<Observation> views = spread_views(charuco());
  const auto refused = [&](const LensCheckOptions& options) {
    return check_lens("bm-1", lab_lens(), charuco(), views, options).status();
  };
  LensCheckOptions options;
  options.max_rms_px = 0.0;
  expect_refused(refused(options), "check_lens", "options.max_rms_px");
  options = {};
  options.max_rms_px = std::numeric_limits<double>::infinity();
  expect_refused(refused(options), "check_lens", "options.max_rms_px");
  options = {};
  options.max_dropped = 1.5;
  expect_refused(refused(options), "check_lens", "options.max_dropped");
  options = {};
  options.max_cell_bias_px = std::numeric_limits<double>::quiet_NaN();
  expect_refused(refused(options), "check_lens", "options.max_cell_bias_px");
  options = {};
  options.min_cell_corners = 1;
  expect_refused(refused(options), "check_lens", "options.min_cell_corners");
  options = {};
  options.min_corners = 3;
  expect_refused(refused(options), "check_lens", "options.min_corners");
  options = {};
  options.outlier_rounds = -1;
  expect_refused(refused(options), "check_lens", "options.outlier_rounds");

  expect_refused(check_lens("", lab_lens(), charuco(), views).status(),
                 "check_lens", "no id");
  camera::CameraModel bad = lab_lens();
  bad.intrinsics.fx = 0.0;
  EXPECT_EQ(check_lens("bm-1", bad, charuco(), views).status().domain(),
            core::Status::Code::InvalidArgument);
  std::vector<Observation> twice = views;
  twice.push_back(views[0]);
  expect_refused(check_lens("bm-1", lab_lens(), charuco(), twice).status(),
                 "check_lens", "capture 0");
  std::vector<Observation> lost = views;
  lost[3].points[0].pixel.x = std::numeric_limits<double>::quiet_NaN();
  expect_refused(check_lens("bm-1", lab_lens(), charuco(), lost).status(),
                 "check_lens", "not finite");
}

TEST(CheckLens, NamesItsVerdicts) {
  EXPECT_STREQ(to_string(Verdict::Good), "good");
  EXPECT_STREQ(to_string(Verdict::Suspect), "suspect");
  EXPECT_STREQ(to_string(Verdict::Insufficient), "insufficient");
  EXPECT_STREQ(to_string(static_cast<Verdict>(9)), "unknown");
}

// The true lens at 0.1, 0.2 and 0.3 px of corner noise, on either board: no
// check of 300 read suspect in the sweep (DECISIONS.md), the worst cell's mean
// at 0.63 of its limit; the default suite draws 3 seeds.
TEST(CheckLensSweep, NoiseAloneDoesNotReadSuspect) {
  const std::uint32_t seeds = test::sweep_size(3, 50);
  int checks = 0;
  double worst = 0.0;
  for (const compute::Board* board : {&checkerboard(), &charuco()}) {
    const std::vector<test::Placement> plan = spread_plan(*board, lab_lens());
    for (const double noise_px : {0.1, 0.2, 0.3}) {
      for (std::uint32_t seed = 1; seed <= seeds; ++seed) {
        const LensCheck check = value_of(
            check_lens("bm-1", lab_lens(), *board,
                       observe(*board, lab_lens(), plan, noise_px, seed)));
        ++checks;
        EXPECT_EQ(check.verdict, Verdict::Good)
            << noise_px << ' ' << seed << ": " << check.reason;
        worst = std::max(worst, worst_cell(check));
      }
    }
  }
  RecordProperty("checks", checks);
  RecordProperty("worst_cell", std::to_string(worst));
  EXPECT_LT(worst, 0.8);
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
