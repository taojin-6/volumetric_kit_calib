// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A lens checked on views of either board, each posed through it held fixed:
// the true lens over views spread across the image is good, on the lab's
// checkerboard and on the rig's 5 x 3 ChArUco board of 8 corners alike; a lens
// bent at one edge reads suspect there and nowhere else, its RMS within the
// limit; a lens off its views reads suspect where its refit reads good; views
// only in the centre are insufficient, naming the parts to capture, and a
// limit passed comes first; noise alone does not read suspect; and what it
// poses, drops and refuses. A lens checked on a solve's residuals: each cell
// held to Student's t, the corners dropped counted, placed and held to their
// share from 2 views on; and on the lab-like pair's rig solve, the true pair
// good as far as its corners reach, a lens off or bent at one edge suspect
// where the check of its views reads good, a camera sharing boards with a
// suspect one suspect too, boards one camera saw alone left out, each camera
// at its own size, a session along the bottom insufficient, naming where to
// capture, and noise alone good on gridded and random sessions.

#include "volumetric_kit/calib/quality/lens_check.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "result_checks.hpp"
#include "synthetic_rig.hpp"
#include "synthetic_scene.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
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

// `views` as if camera bm-1's lens bent at its right: its corners past three
// quarters of the way across image `px` further right than the lens says,
// rising smoothly over 100 px.
std::vector<Observation> bent_right(std::vector<Observation> views, double px) {
  const double from = 0.75 * kSize.width;
  for (Observation& view : views) {
    if (view.camera_id != "bm-1") continue;
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

// The judged cell whose mean is farthest past its limit, as a ratio; -1 with
// none judged.
int worst_cell_at(const LensCheck& check) {
  int at = -1;
  double worst = -1.0;
  for (std::size_t c = 0; c < check.cells.size(); ++c) {
    const LensCell& cell = check.cells[c];
    if (std::isnan(cell.limit_px)) continue;
    const double ratio = glm::length(cell.mean_px) / cell.limit_px;
    if (ratio > worst) {
      worst = ratio;
      at = static_cast<int>(c);
    }
  }
  return at;
}

// How far `views`' corners of camera `id` reach (@ref solve::pixel_reach).
solve::LensReach reach_of(const std::vector<Observation>& views,
                          const std::string& id) {
  solve::LensReach reach{};
  for (const Observation& view : views) {
    if (view.camera_id != id) continue;
    for (const DetectedPoint& point : view.points) {
      const solve::LensReach here = solve::pixel_reach(kSize, point.pixel);
      for (std::size_t d = 0; d < reach.size(); ++d) {
        reach[d] = std::max(reach[d], here[d]);
      }
    }
  }
  return reach;
}

// A pose session of the lab-like pair: the rig's ChArUco board over each
// camera's view in turn (`test::view_plan`), 60 captures.
std::vector<test::Placement> pose_plan() {
  const std::vector<test::SyntheticCamera> pair = test::lab_pair();
  std::vector<test::Placement> plan = test::view_plan(pair, 0, charuco());
  for (test::Placement& placement : test::view_plan(pair, 1, charuco())) {
    plan.push_back(std::move(placement));
  }
  return plan;
}

// What the pair detects of `plan`, with noise the same on every standard
// library.
std::vector<Observation> pair_views(const std::vector<test::Placement>& plan,
                                    double noise_px, std::uint32_t seed = 1) {
  test::ObserveOptions options;
  options.noise_px = noise_px;
  options.seed = seed;
  options.portable = true;
  return test::observe(test::lab_pair(), charuco(), plan, options);
}

// The pair's rig solve of `views`, bm-1's lens with its k1 times `k1_scale`.
struct PairSolve {
  std::vector<solve::RigCamera> cameras;
  solve::RigSolution solution;
};

PairSolve solve_pair(const std::vector<Observation>& views,
                     double k1_scale = 1.0) {
  PairSolve out;
  for (const test::SyntheticCamera& cam : test::lab_pair()) {
    out.cameras.push_back(
        {cam.id, cam.model, camera::IntrinsicsSource::Calibrated});
  }
  out.cameras[0].model.distortion.k1 *= k1_scale;
  const solve::InitialRig start =
      value_of(solve::initialise(out.cameras, charuco(), views, 0));
  out.solution =
      value_of(solve::bundle_adjust(out.cameras, charuco(), views, start));
  return out;
}

// Placements as a person makes them over the lab-like pair's shared view: the
// board's centre aimed anywhere in bm-1's image but its outer 5%, 1.4 to 3 m
// away, tilted up to 30 degrees and rolled up to 20, kept when both cameras
// see it whole 10 px inside their images; drawn the same with every standard
// library.
std::vector<test::Placement> random_plan(std::size_t count,
                                         std::uint32_t seed) {
  const std::vector<test::SyntheticCamera> pair = test::lab_pair();
  std::mt19937 rng(seed);
  const auto uniform = [&rng](double from, double to) {
    constexpr double kToUnit = 1.0 / 4294967296.0;  // 2^-32
    return from + (to - from) * (static_cast<double>(rng()) + 0.5) * kToUnit;
  };
  const camera::CameraModel& lens = pair[0].model;
  const compute::CvCamera cam = value_of(compute::cv_camera(lens));
  const camera::Mat4d camera_to_world =
      camera::rigid_inverse(pair[0].world_to_camera);
  std::vector<test::Placement> plan;
  for (int tries = 0; plan.size() < count && tries < 100000; ++tries) {
    const double x = uniform(0.05, 0.95) * lens.size.width;
    const double y = uniform(0.05, 0.95) * lens.size.height;
    const double distance = uniform(1.4, 3.0);
    const double roll = uniform(-20.0, 20.0);
    const double tilt = uniform(-30.0, 30.0);
    const cv::Point2d ray =
        value_of(compute::undistort_points(cam, {cv::Point2d(x, y)}))[0];
    test::Placement p{
        camera_to_world *
            test::facing(charuco(), std::atan(std::hypot(ray.x, ray.y)) / kDeg,
                         std::atan2(ray.y, ray.x) / kDeg, distance, roll, tilt),
        {0, 1}};
    bool both = true;
    for (const test::SyntheticCamera& c : pair) {
      const camera::Mat4d to_camera = c.world_to_camera * p.board_to_world;
      for (const cv::Point3d& q : charuco().object_points()) {
        both = both && (to_camera * glm::dvec4(q.x, q.y, q.z, 1.0)).z >= 0.3;
      }
      for (const cv::Point2d& px :
           test::project(c.model, to_camera, charuco().object_points())) {
        both = both && px.x >= 10.0 && px.y >= 10.0 &&
               px.x <= c.model.size.width - 11.0 &&
               px.y <= c.model.size.height - 11.0;
      }
    }
    if (both) plan.push_back(std::move(p));
  }
  return plan;
}

// A corner of capture `capture` at `pixel` with `residual`.
solve::CornerResidual corner(std::int64_t capture, double u, double v,
                             double du = 0.0, double dv = 0.0,
                             bool dropped = false) {
  solve::CornerResidual r;
  r.capture = capture;
  r.pixel = {u, v};
  r.residual = {du, dv};
  r.dropped = dropped;
  return r;
}

// 6 corners with no residual in the middle of a cell of each part of the
// image, so every part is judged: captures `first` to `first` + 8.
std::vector<solve::CornerResidual> every_part(std::int64_t first = 100) {
  std::vector<solve::CornerResidual> out;
  const int cols[] = {1, 3, 6};
  const int rows[] = {0, 2, 4};
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 3; ++i) {
      for (int k = 0; k < 6; ++k) {
        out.push_back(corner(first + 3 * j + i, 480.0 * cols[i] + 200.0 + k,
                             360.0 * rows[j] + 180.0));
      }
    }
  }
  return out;
}

// `residuals` as camera `camera`'s of a rig solve, added to `solution`.
void add_as(std::size_t camera, std::vector<solve::CornerResidual> residuals,
            solve::RigSolution& solution) {
  for (solve::CornerResidual& r : residuals) {
    r.camera = camera;
    solution.residuals.push_back(r);
  }
}

// Every corner of the views posed is kept or dropped, and every part of the
// image judged: the defaults' 9; the corners reach as far as the views do.
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
    EXPECT_EQ(check.unshared, 0u);
    EXPECT_EQ(check.reach, reach_of(views, "bm-1"));
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

// Boards aimed within the middle fifth: the image's top and bottom hold no
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
       {"top-left (0)", "top (0)", "top-right (0)", "bottom-left (0)",
        "bottom (0)", "bottom-right (0)"}) {
    EXPECT_TRUE(says(check, part)) << part << ": " << check.reason;
  }
  EXPECT_FALSE(says(check, "centre")) << check.reason;
  EXPECT_TRUE(says(check, "each of its 9 parts needs 6 in a cell"))
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
  options.min_dropped_views = 0;
  expect_refused(refused(options), "check_lens", "options.min_dropped_views");
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

// 6 corners in cell 0, 0, their mean (a, 0) and their scatter about it 0.2 px
// a component: the standard error is 0.2 / sqrt(5) px, and the limit 5.51 of
// them (Student's t for 6 corners), above the 0.1 px floor. A mean just
// within it is good, just past it suspect there alone.
TEST(CheckLensResiduals, HoldsEachCellToStudentsT) {
  const double se = 0.2 / std::sqrt(5.0);
  const double limit = 5.50708 * se;
  const auto with_mean = [](double a) {
    std::vector<solve::CornerResidual> residuals = every_part();
    const double d[6][2] = {{0.2, 0},  {-0.2, 0}, {0.2, 0},
                            {-0.2, 0}, {0, 0.2},  {0, -0.2}};
    for (int k = 0; k < 6; ++k) {
      residuals.push_back(
          corner(k, 100.0 + 50.0 * k, 100.0, a + d[k][0], d[k][1]));
    }
    return residuals;
  };
  const LensCheck within =
      value_of(check_lens_residuals("bm-1", kSize, with_mean(0.99 * limit)));
  EXPECT_EQ(within.verdict, Verdict::Good) << within.reason;
  EXPECT_EQ(within.cells[0].corners, 6);
  EXPECT_NEAR(within.cells[0].mean_px.x, 0.99 * limit, 1e-12);
  EXPECT_NEAR(within.cells[0].limit_px, limit, 1e-5);
  EXPECT_EQ(within.views, 15u);  // captures 0-5 and 100-108
  EXPECT_EQ(within.corners, 60u);
  EXPECT_EQ(within.dropped, 0u);
  EXPECT_TRUE(std::isnan(within.bound_px));
  EXPECT_DOUBLE_EQ(within.cells[1].limit_px, 0.1);  // no scatter: the floor

  const LensCheck past =
      value_of(check_lens_residuals("bm-1", kSize, with_mean(1.01 * limit)));
  EXPECT_EQ(past.verdict, Verdict::Suspect);
  EXPECT_TRUE(says(past, "the mean residual is")) << past.reason;
  EXPECT_TRUE(says(past, "in the image's top-left (cell 0, 0), above 0.493"))
      << past.reason;
  EXPECT_FALSE(says(past, "RMS")) << past.reason;
}

// Dropped corners, and those behind the camera, are held to 1% of all and
// named by where they lie: past it from 2 views or more, suspect; from one, a
// glitch as like as the lens, insufficient. The corners kept alone make the
// RMS and cells.
TEST(CheckLensResiduals, CountsWhatTheSolveDropped) {
  std::vector<solve::CornerResidual> residuals = every_part();  // 54
  residuals.push_back(corner(7, 3700.0, 1000.0, 9.0, 0.0, true));
  residuals.push_back(corner(7, 3700.0, 1100.0, 9.0, 0.0, true));
  LensCheck check = value_of(check_lens_residuals("bm-1", kSize, residuals));
  EXPECT_EQ(check.verdict, Verdict::Insufficient);
  EXPECT_EQ(check.reason,
            "2 of 56 corners were dropped by the solve or behind the camera, "
            "above 1%, from 1 view, in the image's right (2): too few views "
            "to tell the lens from a board's detection; capture the board "
            "there again");
  EXPECT_EQ(check.corners, 54u);
  EXPECT_EQ(check.dropped, 2u);
  EXPECT_DOUBLE_EQ(check.rms_px, 0.0);
  LensCheckOptions any;
  any.min_dropped_views = 1;
  check = value_of(check_lens_residuals("bm-1", kSize, residuals, any));
  EXPECT_EQ(check.verdict, Verdict::Suspect);
  EXPECT_EQ(check.reason,
            "2 of 56 corners were dropped by the solve or behind the camera, "
            "above 1%, from 1 view, in the image's right (2)");

  residuals.back().capture = 8;
  residuals.back().dropped = false;
  residuals.back().residual.x = std::numeric_limits<double>::quiet_NaN();
  check = value_of(check_lens_residuals("bm-1", kSize, residuals));
  EXPECT_EQ(check.dropped, 2u);
  EXPECT_EQ(check.verdict, Verdict::Suspect);
  EXPECT_TRUE(says(check,
                   "2 of 56 corners were dropped by the solve or "
                   "behind the camera, above 1%, from 2 views"))
      << check.reason;

  // 2 of 200 is within 1%.
  for (int k = 0; k < 144; ++k) residuals.push_back(corner(9, 2000.0, 1000.0));
  check = value_of(check_lens_residuals("bm-1", kSize, residuals));
  EXPECT_EQ(check.verdict, Verdict::Good) << check.reason;
  EXPECT_EQ(check.views, 12u);
}

// The parts of the image with no cell of 6 corners are named.
TEST(CheckLensResiduals, TooFewCornersAreInsufficientNamingWhereToCapture) {
  std::vector<solve::CornerResidual> residuals = every_part();
  residuals.erase(residuals.begin(), residuals.begin() + 7);  // top-left
  const LensCheck check =
      value_of(check_lens_residuals("bm-1", kSize, residuals));
  EXPECT_EQ(check.verdict, Verdict::Insufficient);
  EXPECT_EQ(check.reason,
            "too few corners in the image's top-left (0), top (5), the most "
            "in one of its cells: each of its 9 parts needs 6 in a cell of "
            "the 8 x 6 grid; capture the board there");
}

TEST(CheckLensResiduals, Refuses) {
  const std::vector<solve::CornerResidual> residuals = every_part();
  expect_refused(check_lens_residuals("", kSize, residuals).status(),
                 "check_lens_residuals", "no id");
  expect_refused(check_lens_residuals("bm-1", {3840, 0}, residuals).status(),
                 "check_lens_residuals", "3840 x 0 has a side 0");
  LensCheckOptions options;
  options.min_cell_corners = 1;
  expect_refused(
      check_lens_residuals("bm-1", kSize, residuals, options).status(),
      "check_lens_residuals", "options.min_cell_corners");
  std::vector<solve::CornerResidual> outside = residuals;
  outside[3].pixel.x = 3839.5;
  expect_refused(check_lens_residuals("bm-1", kSize, outside).status(),
                 "check_lens_residuals",
                 "a corner of bm-1 in capture 100 at (3.84e+03, 180) is not "
                 "finite or outside the 3840 x 2160 image");
  outside[3].pixel.x = std::numeric_limits<double>::quiet_NaN();
  expect_refused(check_lens_residuals("bm-1", kSize, outside).status(),
                 "check_lens_residuals", "is not finite or outside");
}

// The true pair over its view, both cameras sharing each capture's board:
// good, each camera's cells holding its own corners; good as far out as they
// reach, short of what a lens fit's gates ask.
TEST(CheckRigLens, TheTruePairIsGood) {
  const std::vector<test::Placement> plan = pose_plan();
  for (const std::uint32_t seed : {1u, 2u, 3u}) {
    const std::vector<Observation> views = pair_views(plan, 0.15, seed);
    const PairSolve rig = solve_pair(views);
    for (const solve::RigCamera& cam : rig.cameras) {
      const LensCheck check =
          value_of(check_rig_lens(cam.id, rig.cameras, rig.solution));
      EXPECT_EQ(check.verdict, Verdict::Good) << cam.id << ": " << check.reason;
      EXPECT_EQ(check.camera, cam.id);
      EXPECT_EQ(check.views, plan.size());
      EXPECT_EQ(check.dropped, 0u);
      EXPECT_EQ(check.unshared, 0u);
      EXPECT_EQ(check.reach, reach_of(views, cam.id));
      EXPECT_LT(*std::min_element(check.reach.begin(), check.reach.end()),
                solve::kLensReach);
      EXPECT_GT(check.rms_px, 0.1);
      EXPECT_LT(check.rms_px, 0.25);
      EXPECT_TRUE(std::isnan(check.bound_px));
      std::array<int, kLensCells> in_cells{};
      for (const Observation& view : views) {
        if (view.camera_id != cam.id) continue;
        for (const DetectedPoint& p : view.points) {
          ++in_cells[static_cast<std::size_t>(*compute::grid_cell(
              p.pixel.x, p.pixel.y, 3840, 2160, kCoverageCols, kCoverageRows))];
        }
      }
      for (std::size_t c = 0; c < in_cells.size(); ++c) {
        EXPECT_EQ(check.cells[c].corners, in_cells[c]) << cam.id << ' ' << c;
      }
    }
  }
}

// bm-1's k1 1% high: each view posed on its own takes it up, so the check of
// its views reads good; the rig solve cannot, and reads suspect by the RMS
// and the cells, naming them. The boards it shares show it in bm-2 too.
TEST(CheckRigLens, ALensOffIsSuspectWhereItsViewsPosesHideIt) {
  const std::vector<Observation> views = pair_views(pose_plan(), 0.15);
  const PairSolve rig = solve_pair(views, 1.01);
  const LensCheck check =
      value_of(check_rig_lens("bm-1", rig.cameras, rig.solution));
  EXPECT_EQ(check.verdict, Verdict::Suspect);
  EXPECT_GT(check.rms_px, 0.6);
  EXPECT_TRUE(says(check, "the RMS through the lens is")) << check.reason;
  for (const char* part : {"top-left (cell", "top-right (cell"}) {
    EXPECT_TRUE(says(check, part)) << part << ": " << check.reason;
  }
  const LensCheck shared =
      value_of(check_rig_lens("bm-2", rig.cameras, rig.solution));
  EXPECT_EQ(shared.verdict, Verdict::Suspect);

  const LensCheck own =
      value_of(check_lens("bm-1", rig.cameras[0].model, charuco(), views));
  EXPECT_EQ(own.verdict, Verdict::Good) << own.reason;
  EXPECT_LT(own.rms_px, 0.2);
}

// 3 px over 100 px at three quarters across, in bm-1 alone: hidden from the
// check of its views, suspect in the rig solve, its worst cell in the image's
// right third.
TEST(CheckRigLens, ALensBentAtOneEdgeIsSuspectThere) {
  const std::vector<test::Placement> plan = pose_plan();
  for (const std::uint32_t seed : {1u, 2u, 3u}) {
    const std::vector<Observation> views =
        bent_right(pair_views(plan, 0.15, seed), 3.0);
    const PairSolve rig = solve_pair(views);
    const LensCheck check =
        value_of(check_rig_lens("bm-1", rig.cameras, rig.solution));
    EXPECT_EQ(check.verdict, Verdict::Suspect) << seed;
    EXPECT_TRUE(says(check, "the mean residual is")) << check.reason;
    EXPECT_TRUE(says(check, "right (cell")) << check.reason;
    EXPECT_GE(worst_cell_at(check) % kCoverageCols, 5) << check.reason;

    const LensCheck own =
        value_of(check_lens("bm-1", lab_lens(), charuco(), views));
    EXPECT_EQ(own.verdict, Verdict::Good) << seed << ": " << own.reason;
  }
}

// 10 captures along the bottom of bm-1's view, as the lab's first pose
// session: the image's top and middle are named, where to capture.
TEST(CheckRigLens, TooFewCapturesAreInsufficientNamingWhereToCapture) {
  const std::vector<test::Placement> all =
      test::view_plan(test::lab_pair(), 0, charuco());
  std::vector<test::Placement> bottom;
  for (std::size_t i = 0; i < all.size(); ++i) {
    if (i % 15 >= 10) bottom.push_back(all[i]);  // the lowest row
  }
  const PairSolve rig = solve_pair(pair_views(bottom, 0.15));
  for (const solve::RigCamera& cam : rig.cameras) {
    const LensCheck check =
        value_of(check_rig_lens(cam.id, rig.cameras, rig.solution));
    EXPECT_EQ(check.verdict, Verdict::Insufficient) << cam.id;
    EXPECT_EQ(check.views, 10u);
    for (const char* part : {"top-left (0)", "top (0)", "top-right (0)",
                             "left (0)", "centre (0)", "right (0)"}) {
      EXPECT_TRUE(says(check, part)) << part << ": " << check.reason;
    }
    EXPECT_TRUE(
        says(check, "capture the board there, where another camera sees it"))
        << check.reason;
  }
}

// bm-1's k1 0.3% high, the board over bm-1's view (seed 1): bm-2's true lens
// reads a cell past its limit and bm-1's own residuals none, yet bm-1 reads
// suspect too, naming bm-2. The boards they share say their lenses disagree,
// not which is off.
TEST(CheckRigLens, ACameraSharingBoardsWithASuspectOneIsSuspect) {
  const PairSolve rig = solve_pair(
      pair_views(test::view_plan(test::lab_pair(), 0, charuco()), 0.15), 1.003);
  std::vector<solve::CornerResidual> own;
  for (const solve::CornerResidual& r : rig.solution.residuals) {
    if (r.camera == 0) own.push_back(r);
  }
  const LensCheck alone = value_of(check_lens_residuals("bm-1", kSize, own));
  EXPECT_EQ(alone.verdict, Verdict::Good) << alone.reason;
  const LensCheck partner =
      value_of(check_rig_lens("bm-2", rig.cameras, rig.solution));
  EXPECT_EQ(partner.verdict, Verdict::Suspect);
  EXPECT_TRUE(says(partner, "the mean residual is")) << partner.reason;
  const LensCheck off =
      value_of(check_rig_lens("bm-1", rig.cameras, rig.solution));
  EXPECT_EQ(off.verdict, Verdict::Suspect);
  EXPECT_EQ(off.reason,
            "bm-2 reads suspect on the boards they share: their lenses "
            "disagree");
}

// Three cameras, crafted: bm-2 shares captures 100 to 108 with bm-1, bm-3
// captures 200 to 208. bm-2's mean in one cell is past its limit, so bm-1,
// which shares its boards, reads suspect, naming it; bm-3 shares none of
// them, and bm-1's own residuals are good, so bm-3 reads good.
TEST(CheckRigLens, SuspectReachesOnlyTheCamerasSharingItsBoards) {
  const std::vector<solve::RigCamera> cameras = {
      {"bm-1", lab_lens()}, {"bm-2", lab_lens()}, {"bm-3", lab_lens()}};
  solve::RigSolution solution;
  solution.world_to_camera.resize(3);
  add_as(0, every_part(100), solution);
  add_as(0, every_part(200), solution);
  std::vector<solve::CornerResidual> off = every_part(100);
  for (std::size_t k = 0; k < 6; ++k) off[k].residual = {0.5, 0.0};
  add_as(1, off, solution);
  add_as(2, every_part(200), solution);
  const auto check = [&](const char* id) {
    return value_of(check_rig_lens(id, cameras, solution));
  };
  const LensCheck bm2 = check("bm-2");
  EXPECT_EQ(bm2.verdict, Verdict::Suspect);
  EXPECT_TRUE(says(bm2, "0.5 px in the image's top-left (cell 1, 0)"))
      << bm2.reason;
  EXPECT_FALSE(says(bm2, "disagree")) << bm2.reason;
  const LensCheck bm1 = check("bm-1");
  EXPECT_EQ(bm1.verdict, Verdict::Suspect);
  EXPECT_EQ(bm1.reason,
            "bm-2 reads suspect on the boards they share: their lenses "
            "disagree");
  EXPECT_EQ(bm1.corners, 108u);
  const LensCheck bm3 = check("bm-3");
  EXPECT_EQ(bm3.verdict, Verdict::Good) << bm3.reason;
}

// bm-1's view covered by boards bm-1 alone sees (its view plan), and 10 boards
// both see (the first of bm-2's): the lone boards' poses are bm-1's own and
// take up k1 1% high, so their corners are left out. The true lens is then
// insufficient, the shared boards short of its view; k1 1% high reads suspect
// by bm-1's own RMS, not only beside bm-2.
TEST(CheckRigLens, BoardsOneCameraSawAloneAreLeftOut) {
  const std::vector<test::SyntheticCamera> pair = test::lab_pair();
  std::vector<test::Placement> alone = test::view_plan(pair, 0, charuco());
  for (test::Placement& p : alone) p.cameras = {0};
  const std::vector<test::Placement> plan = test::view_plan(pair, 1, charuco());
  const std::vector<test::Placement> both(plan.begin(), plan.begin() + 10);
  std::vector<Observation> views = pair_views(alone, 0.15);
  for (Observation& view : pair_views(both, 0.15, 2)) {
    view.frame_id += 1000;
    views.push_back(std::move(view));
  }
  for (const bool off : {false, true}) {
    const PairSolve rig = solve_pair(views, off ? 1.01 : 1.0);
    const LensCheck check =
        value_of(check_rig_lens("bm-1", rig.cameras, rig.solution));
    EXPECT_EQ(check.unshared, 8 * alone.size()) << off;
    EXPECT_EQ(check.views, 10u) << off;
    EXPECT_EQ(check.corners + check.dropped, 80u) << off;
    if (!off) {
      EXPECT_EQ(check.verdict, Verdict::Insufficient) << check.reason;
      EXPECT_TRUE(says(check, "where another camera sees it too"))
          << check.reason;
    } else {
      EXPECT_EQ(check.verdict, Verdict::Suspect);
      EXPECT_TRUE(says(check, "the RMS through the lens is")) << check.reason;
    }
  }
}

// Each camera judged at its own image's size: bm-2's half-size image bins its
// corners on its own 8 x 6 grid and refuses one outside it.
TEST(CheckRigLens, JudgesEachCameraAtItsOwnSize) {
  std::vector<solve::RigCamera> cameras = {{"bm-1", lab_lens()},
                                           {"bm-2", lab_lens()}};
  cameras[1].model.size = {1920, 1080};
  solve::RigSolution solution;
  solution.world_to_camera.resize(2);
  add_as(0, every_part(), solution);
  std::vector<solve::CornerResidual> half = every_part();
  for (solve::CornerResidual& r : half) r.pixel *= 0.5;
  add_as(1, half, solution);
  const LensCheck full = value_of(check_rig_lens("bm-1", cameras, solution));
  const LensCheck small = value_of(check_rig_lens("bm-2", cameras, solution));
  EXPECT_EQ(small.verdict, Verdict::Good) << small.reason;
  for (std::size_t c = 0; c < small.cells.size(); ++c) {
    EXPECT_EQ(small.cells[c].corners, full.cells[c].corners) << c;
  }
  solution.residuals.back().pixel = {1919.0, 100.0};
  EXPECT_TRUE(check_rig_lens("bm-2", cameras, solution).ok());
  solution.residuals.back().pixel = {3000.0, 100.0};
  expect_refused(check_rig_lens("bm-2", cameras, solution).status(),
                 "check_rig_lens",
                 "a corner of bm-2 in capture 108 at (3e+03, 100) is not "
                 "finite or outside the 1920 x 1080 image");
}

TEST(CheckRigLens, Refuses) {
  const PairSolve rig = solve_pair(pair_views(pose_plan(), 0.15));
  expect_refused(check_rig_lens("bm-3", rig.cameras, rig.solution).status(),
                 "check_rig_lens", "the rig has no camera \"bm-3\"");
  std::vector<solve::RigCamera> more = rig.cameras;
  more.push_back({"bm-3", lab_lens()});
  expect_refused(check_rig_lens("bm-1", more, rig.solution).status(),
                 "check_rig_lens", "the solution has 2 cameras, the rig 3");
  solve::RigSolution odd = rig.solution;
  odd.residuals[5].camera = 7;
  expect_refused(check_rig_lens("bm-1", rig.cameras, odd).status(),
                 "check_rig_lens",
                 "a residual of camera 7, which the rig does not have");
  odd = rig.solution;
  ASSERT_EQ(odd.residuals[0].camera, 0u);  // bm-1's
  odd.residuals[0].pixel.y = -3.0;
  expect_refused(check_rig_lens("bm-1", rig.cameras, odd).status(),
                 "check_rig_lens", "outside the 3840 x 2160 image");
  LensCheckOptions options;
  options.max_dropped = -0.1;
  expect_refused(
      check_rig_lens("bm-1", rig.cameras, rig.solution, options).status(),
      "check_rig_lens", "options.max_dropped");
}

// The true lens at 0.1, 0.2 and 0.3 px of corner noise, on either board: no
// check of 300 read suspect in the sweep (DECISIONS.md), the worst cell's mean
// at 0.61 of its limit; the default suite draws 3 seeds.
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

// The true pair's rig solve at 0.1, 0.2 and 0.3 px of corner noise: no check
// of 300 read other than good in the sweep (DECISIONS.md), the worst cell's
// mean at 0.78 of its limit; the default suite draws 3 seeds.
TEST(CheckRigLensSweep, NoiseAloneDoesNotReadSuspect) {
  const std::uint32_t seeds = test::sweep_size(3, 50);
  const std::vector<test::Placement> plan = pose_plan();
  int checks = 0;
  double worst = 0.0;
  for (const double noise_px : {0.1, 0.2, 0.3}) {
    for (std::uint32_t seed = 1; seed <= seeds; ++seed) {
      const PairSolve rig = solve_pair(pair_views(plan, noise_px, seed));
      for (const solve::RigCamera& cam : rig.cameras) {
        const LensCheck check =
            value_of(check_rig_lens(cam.id, rig.cameras, rig.solution));
        ++checks;
        EXPECT_EQ(check.verdict, Verdict::Good)
            << cam.id << ' ' << noise_px << ' ' << seed << ": " << check.reason;
        worst = std::max(worst, worst_cell(check));
      }
    }
  }
  RecordProperty("checks", checks);
  RecordProperty("worst_cell", std::to_string(worst));
  EXPECT_LT(worst, 0.9);
}

// The true pair over random sessions (random_plan) of 15, 20, 30 and 40
// captures at 0.15 and 0.3 px of corner noise, 200 sessions each in the sweep
// (DECISIONS.md): the share of cameras read suspect, at most the design's 2%
// at 0.15 px, and the share read good, the rest insufficient. The default
// suite draws 2 sessions of each.
TEST(CheckRigLensSweep, RandomSessionsOfNoiseAlone) {
  const std::uint32_t sessions = test::sweep_size(2, 200);
  for (const std::size_t captures : {15u, 20u, 30u, 40u}) {
    for (const char* noise : {"0.15", "0.3"}) {
      const double noise_px = std::stod(noise);
      int checks = 0;
      int suspect = 0;
      int good = 0;
      for (std::uint32_t s = 1; s <= sessions; ++s) {
        const std::vector<test::Placement> plan =
            random_plan(captures, 1000 + s);
        ASSERT_EQ(plan.size(), captures);
        const PairSolve rig = solve_pair(pair_views(plan, noise_px, s));
        for (const solve::RigCamera& cam : rig.cameras) {
          const LensCheck check =
              value_of(check_rig_lens(cam.id, rig.cameras, rig.solution));
          ++checks;
          suspect += check.verdict == Verdict::Suspect ? 1 : 0;
          good += check.verdict == Verdict::Good ? 1 : 0;
        }
      }
      const std::string at = std::to_string(captures) + "_" + noise;
      RecordProperty("suspect_" + at, suspect);
      RecordProperty("good_" + at, good);
      RecordProperty("checks_" + at, checks);
      if (noise_px < 0.2) {
        EXPECT_LE(suspect, 0.02 * checks) << captures << " captures";
      }
    }
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
