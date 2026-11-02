// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A solved lens's gates, on fits of a fisheye truth and on fits made here: a
// right lens passes, however noisy; a wrong minimum and a curve bent past 45
// degrees, which the RMS passes, fail the bias gate, above its noise floor;
// the fold gate, in normalized radius through sensor, fails the 2024 bm-2 lens
// and a pole inside the frame, and passes the lab's current lens; boards that
// stop short fail the reach gate; and the lens-only results files, read back
// through sensor and nlohmann.

#include "volumetric_kit/calib/quality/lens_gate.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/calib.hpp>
#include <opencv2/geometry.hpp>

#include "result_checks.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = sensor::camera;
using json = nlohmann::json;
using Use = solve::LensView::Use;
using test::expect_refused;
using test::value_of;

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr camera::ImageSize kSize{3840, 2160};

// The lab's lens with its measured centre, off the image's.
test::FisheyeTruth truth() {
  test::FisheyeTruth lens;
  lens.cx = 1917.4;
  lens.cy = 1108.3;
  return lens;
}

const compute::Board& board() {
  static const compute::Board once = test::lab_checkerboard();
  return once;
}

std::vector<Observation> session(std::uint32_t seed, double noise_px = 0.15) {
  test::FisheyeCapture capture;
  capture.seed = seed;
  capture.noise_px = noise_px;
  return test::fisheye_views(board(), truth(), capture);
}

// `views` as if no board had gone past `limit` of the way toward any image
// corner or side.
std::vector<Observation> stop_short(const std::vector<Observation>& views,
                                    double limit) {
  std::vector<Observation> kept;
  for (const Observation& view : views) {
    Observation cut = view;
    cut.points.clear();
    for (const DetectedPoint& point : view.points) {
      const solve::LensReach reach = solve::pixel_reach(kSize, point.pixel);
      if (*std::max_element(reach.begin(), reach.end()) < limit) {
        cut.points.push_back(point);
      }
    }
    if (cut.points.size() >= 12) kept.push_back(std::move(cut));
  }
  return kept;
}

// `view` numbered as the detector numbers a board the frame cuts: from its own
// first corner, half turned if `turned`.
void renumber(Observation& view, bool turned) {
  const int cols = board().target().cols;
  int first_col = cols;
  int first_row = board().target().rows;
  int last_col = 0;
  int last_row = 0;
  for (const DetectedPoint& point : view.points) {
    first_col = std::min(first_col, point.id % cols);
    first_row = std::min(first_row, point.id / cols);
    last_col = std::max(last_col, point.id % cols);
    last_row = std::max(last_row, point.id / cols);
  }
  for (DetectedPoint& point : view.points) {
    const int col = point.id % cols;
    const int row = point.id / cols;
    point.id = turned ? (last_row - row) * cols + (last_col - col)
                      : (row - first_row) * cols + (col - first_col);
  }
}

const solve::LensFit& right_fit() {
  static const solve::LensFit once =
      value_of(solve::solve_lens("bm-1", kSize, board(), session(1)));
  return once;
}

// The 2024 fit of bm-2: its corners folded 74.8 degrees off axis.
camera::CameraModel lens_2024_bm2() {
  return {kSize,
          {1697.7532870795567, 1698.1446781751088, 1905.8014588758742,
           1090.725252968039},
          {0.43858459246177117, -0.008806271952411146, 0.0001088146725150302,
           -0.00011317892942162059, -0.0010377761285881846, 0.8032619209291925,
           0.06157382758196919, -0.005978906562434272}};
}

// The lab's current lens, which never folds.
camera::CameraModel lab_lens() {
  return {kSize,
          {1702.410047823579, 1702.1183368985696, 1917.379808460028,
           1108.2673697604462},
          {0.7635753673346596, 0.1339021352077378, -7.126331072400135e-05,
           4.5662431317182075e-05, 0.0032974798211048597, 1.1293583015706812,
           0.3232813210032048, 0.02266887261491428}};
}

// A fit of one lens with no corners: only its RMS values and lens to judge.
solve::LensFit bare_fit(const camera::CameraModel& model,
                        double holdout_rms_px = 0.2, double rms_px = 0.2) {
  solve::LensFit fit;
  fit.camera.id = "bm-1";
  fit.camera.model = model;
  fit.camera.source = camera::IntrinsicsSource::Calibrated;
  fit.rms_px = rms_px;
  fit.holdout_rms_px = holdout_rms_px;
  return fit;
}

// The gates on a fit made here, whose few corners reach nowhere in particular:
// all but reach.
core::Result<LensVerdict> judge_made(solve::LensFit fit) {
  LensGateOptions options;
  options.min_reach = 0.0;
  return judge_lens(std::move(fit), options);
}

// Adds a view of `n` corners at `pixel`, residuals alternating about `mean`
// by `spread` on each axis.
void add_corners(solve::LensFit& fit, std::int64_t capture, Use use, int n,
                 const camera::Vec2d& pixel, const camera::Vec2d& mean,
                 double spread) {
  fit.views.push_back({capture, use, n, 0.0});
  for (int i = 0; i < n; ++i) {
    solve::CornerResidual r;
    r.capture = capture;
    r.corner = i;
    r.pixel = pixel;
    const double sign = i % 2 == 0 ? 1.0 : -1.0;
    r.residual = mean + camera::Vec2d(sign * spread);
    fit.residuals.push_back(r);
  }
}

bool mentions(const LensVerdict& verdict, const std::string& text) {
  return std::any_of(verdict.failed.begin(), verdict.failed.end(),
                     [&](const std::string& why) {
                       return why.find(text) != std::string::npos;
                     });
}

// Every gate missed, for a failure's message: none is not an error here.
std::string reasons(const LensVerdict& verdict) {
  std::string all = verdict.failed.empty() ? "no gate missed" : "";
  for (const std::string& why : verdict.failed) all += why + "; ";
  return all;
}

constexpr const char* kBias = "mean residual";

// Down at the detector's noise too, where the ratio of two small numbers would
// fail a right lens; and with each view numbered from its own first corner, as
// the detector numbers a board the frame cuts.
TEST(JudgeLens, PassesARightLens) {
  for (const double noise_px : {0.15, 0.01, 0.0}) {
    for (const std::uint32_t seed : {1u, 2u, 3u}) {
      const solve::LensFit fit = value_of(
          solve::solve_lens("bm-1", kSize, board(), session(seed, noise_px)));
      const LensVerdict verdict = value_of(judge_lens(fit));
      ASSERT_TRUE(verdict.pass) << noise_px << ' ' << reasons(verdict);
      EXPECT_TRUE(verdict.failed.empty());
      EXPECT_GT(verdict.worst_cell_bias_px, 0.0);
      EXPECT_LT(verdict.worst_cell_bias_px, 0.1);
      EXPECT_EQ(verdict.fold_deg, 90.0);
      EXPECT_FALSE(verdict.frame_folds);
      EXPECT_EQ(verdict.fit.camera.model.intrinsics.fx,
                fit.camera.model.intrinsics.fx);
    }
  }

  std::vector<Observation> views = session(1);
  for (std::size_t v = 0; v < views.size(); ++v) {
    renumber(views[v], v % 2 == 1);
  }
  const LensVerdict renumbered = value_of(
      judge_lens(value_of(solve::solve_lens("bm-1", kSize, board(), views))));
  EXPECT_TRUE(renumbered.pass) << reasons(renumbered);
  const LensVerdict plain = value_of(judge_lens(right_fit()));
  EXPECT_EQ(renumbered.worst_cell, plain.worst_cell);
  EXPECT_NEAR(renumbered.worst_cell_bias_px, plain.worst_cell_bias_px, 1e-3);
}

// Boards that stop short of the reach look right where they went, and say
// nothing of the curve past it, which the runtime still reads.
TEST(JudgeLens, FailsBoardsThatStopShort) {
  const solve::LensFit fit = value_of(
      solve::solve_lens("bm-1", kSize, board(), stop_short(session(1), 0.8)));
  const LensVerdict verdict = value_of(judge_lens(fit));
  EXPECT_FALSE(verdict.pass);
  EXPECT_TRUE(mentions(verdict, "toward the top-left corner, short of 0.9"));
  for (const double reach : verdict.reach) EXPECT_LT(reach, 0.8);
}

// Each direction by name, at its limit, over the fitted corners kept.
TEST(JudgeLens, HoldsEachDirectionToItsReach) {
  const camera::Vec2d none(0.0);
  solve::LensFit fit = bare_fit(lab_lens());
  add_corners(fit, 0, Use::Fitted, 2, {20.0, 20.0}, none, 0.0);
  add_corners(fit, 1, Use::Fitted, 2, {3820.0, 20.0}, none, 0.0);
  add_corners(fit, 2, Use::Fitted, 2, {20.0, 2140.0}, none, 0.0);
  add_corners(fit, 3, Use::Fitted, 2, {3820.0, 2140.0}, none, 0.0);
  LensVerdict verdict = value_of(judge_lens(fit));
  EXPECT_TRUE(verdict.pass) << reasons(verdict);
  const solve::LensReach corner = solve::pixel_reach(kSize, {3820.0, 2140.0});
  EXPECT_DOUBLE_EQ(verdict.reach[3], corner[3]);
  EXPECT_DOUBLE_EQ(verdict.reach[5], corner[5]);

  LensGateOptions options;
  options.min_reach = *std::min_element(verdict.reach.begin(),  //
                                        verdict.reach.end());
  EXPECT_TRUE(value_of(judge_lens(fit, options)).pass);
  options.min_reach = std::nextafter(options.min_reach, 1.0);
  EXPECT_FALSE(value_of(judge_lens(fit, options)).pass);

  // The bottom right's only corners dropped, or held out: it is not reached.
  for (solve::CornerResidual& r : fit.residuals) r.dropped = r.capture == 3;
  add_corners(fit, 4, Use::HeldOut, 2, {3820.0, 2140.0}, none, 0.0);
  verdict = value_of(judge_lens(fit));
  ASSERT_EQ(verdict.failed.size(), 1u) << reasons(verdict);
  EXPECT_TRUE(mentions(verdict, "toward the bottom-right corner"));

  // A side alone: these reach 0.94 toward the right corners, 0.927 toward it.
  fit = bare_fit(lab_lens());
  add_corners(fit, 0, Use::Fitted, 2, {20.0, 20.0}, none, 0.0);
  add_corners(fit, 1, Use::Fitted, 2, {3700.0, 20.0}, none, 0.0);
  add_corners(fit, 2, Use::Fitted, 2, {20.0, 2140.0}, none, 0.0);
  add_corners(fit, 3, Use::Fitted, 2, {3700.0, 2140.0}, none, 0.0);
  options.min_reach = 0.93;
  verdict = value_of(judge_lens(fit, options));
  ASSERT_EQ(verdict.failed.size(), 1u);
  EXPECT_TRUE(mentions(verdict, "toward the right side"));
}

// A wrong first fit can condemn good views, one of them perhaps the only one
// to reach a corner: nothing here is bad, so nothing stays dropped, and the
// session keeps its reach. OpenCV's path differs in its last bits between
// machines, so a seed may not land wrong anywhere else.
TEST(JudgeLens, AWrongFirstFitCostsASessionNothing) {
  for (const std::uint32_t seed : {18u, 26u}) {
    const LensVerdict verdict = value_of(judge_lens(value_of(solve::solve_lens(
        "bm-1", kSize, board(), stop_short(session(seed), 0.92)))));
    EXPECT_TRUE(verdict.pass) << seed << ' ' << reasons(verdict);
    for (const solve::CornerResidual& r : verdict.fit.residuals) {
      ASSERT_FALSE(r.dropped) << seed;
    }
  }
}

// The reach, through the fit, is the angle the truth gives that corner.
TEST(JudgeLens, MeasuresHowFarOffAxisTheCornersReach) {
  const LensVerdict verdict = value_of(judge_lens(right_fit()));
  const test::FisheyeTruth lens = truth();
  solve::LensReach reach{};
  std::vector<camera::Vec2d> farthest(reach.size());
  std::vector<std::int64_t> fitted;
  for (const solve::LensView& view : right_fit().views) {
    if (view.use == Use::Fitted) fitted.push_back(view.frame_id);
  }
  for (const solve::CornerResidual& r : right_fit().residuals) {
    if (std::find(fitted.begin(), fitted.end(), r.capture) == fitted.end()) {
      continue;
    }
    const solve::LensReach here = solve::pixel_reach(kSize, r.pixel);
    for (std::size_t d = 0; d < reach.size(); ++d) {
      if (here[d] > reach[d]) {
        reach[d] = here[d];
        farthest[d] = r.pixel;
      }
    }
  }
  const auto off_axis = [&](std::size_t d) {
    return std::acos(lens.ray({farthest[d].x, farthest[d].y}).z) / kDeg;
  };
  for (std::size_t d = 0; d < 4; ++d) {
    EXPECT_GT(verdict.corner_reach_deg[d], 70.0) << d;
    EXPECT_NEAR(verdict.corner_reach_deg[d], off_axis(d), 0.1) << d;
  }
  for (std::size_t d = 0; d < 2; ++d) {
    EXPECT_GT(verdict.side_reach_deg[d], 61.0) << d;
    EXPECT_NEAR(verdict.side_reach_deg[d], off_axis(4 + d), 0.1) << d;
  }
}

TEST(JudgeLens, HoldsTheHeldOutRmsToItsLimitScaledByWidth) {
  EXPECT_TRUE(value_of(judge_made(bare_fit(lab_lens(), 0.5, 0.45))).pass);
  LensVerdict verdict =
      value_of(judge_made(bare_fit(lab_lens(), 0.5001, 0.45)));
  EXPECT_FALSE(verdict.pass);
  ASSERT_EQ(verdict.failed.size(), 1u);
  EXPECT_NE(verdict.failed[0].find("0.5 px"), std::string::npos)
      << verdict.failed[0];

  // At half the width, half the pixels of error.
  camera::CameraModel half = lab_lens();
  half.size = {1920, 1080};
  half.intrinsics = {851.0, 851.0, 959.5, 539.5};
  EXPECT_TRUE(value_of(judge_made(bare_fit(half, 0.25, 0.25))).pass);
  EXPECT_FALSE(value_of(judge_made(bare_fit(half, 0.2501, 0.25))).pass);
}

TEST(JudgeLens, HoldsTheHeldOutRmsToItsRatioToTheFits) {
  EXPECT_TRUE(value_of(judge_made(bare_fit(lab_lens(), 0.26, 0.2))).pass);
  const LensVerdict verdict =
      value_of(judge_made(bare_fit(lab_lens(), 0.2601, 0.2)));
  EXPECT_FALSE(verdict.pass);
  EXPECT_TRUE(mentions(verdict, "1.3")) << reasons(verdict);
}

// 3.3 times the fit's, down at the detector's noise, says nothing; the floor
// scales with the image.
TEST(JudgeLens, JudgesTheRatioOnlyAboveItsFloor) {
  EXPECT_TRUE(value_of(judge_made(bare_fit(lab_lens(), 0.1, 0.03))).pass);
  EXPECT_FALSE(value_of(judge_made(bare_fit(lab_lens(), 0.1001, 0.03))).pass);

  camera::CameraModel half = lab_lens();
  half.size = {1920, 1080};
  half.intrinsics = {851.0, 851.0, 959.5, 539.5};
  EXPECT_TRUE(value_of(judge_made(bare_fit(half, 0.05, 0.015))).pass);
  EXPECT_FALSE(value_of(judge_made(bare_fit(half, 0.0501, 0.015))).pass);
}

// A glitch costs a held-out view its corner and no more; a lens wrong enough
// to send more than 1% of them past the bound does not hide there.
TEST(JudgeLens, HoldsTheHeldOutCornersDroppedToTheirShare) {
  const camera::Vec2d at(100.0, 100.0);
  const camera::Vec2d none(0.0);
  solve::LensFit fit = bare_fit(lab_lens());
  add_corners(fit, 0, Use::HeldOut, 200, at, none, 0.1);
  add_corners(fit, 1, Use::Fitted, 20, at, none, 0.1);
  fit.residuals[0].dropped = true;
  fit.residuals[1].dropped = true;
  fit.residuals[200].dropped = true;  // fitted: not a held-out corner
  fit.residuals[201].dropped = true;
  LensVerdict verdict = value_of(judge_made(fit));
  EXPECT_TRUE(verdict.pass) << reasons(verdict);
  EXPECT_EQ(verdict.holdout_corners, 200);
  EXPECT_EQ(verdict.holdout_dropped, 2);

  fit.residuals[2].dropped = true;
  verdict = value_of(judge_made(fit));
  EXPECT_FALSE(verdict.pass);
  EXPECT_EQ(verdict.holdout_dropped, 3);
  EXPECT_TRUE(mentions(verdict, "3 of 200 held-out corners"))
      << reasons(verdict);
}

// One corner off in a held-out view, by more than the 0.5 px gate and past the
// corner bound, passes as it does in a fitted view. Several in every one fail,
// lens or no lens to blame: the gate cannot tell them from a wrong one.
TEST(JudgeLens, ABadHeldOutCornerPassesAndManyFail) {
  for (const double px : {6.0, 25.0}) {
    std::vector<Observation> views = session(1);
    const std::vector<solve::LensView> plan =
        value_of(solve::plan_lens_views("bm-1", kSize, board(), views));
    int spoiled = 0;
    for (std::size_t v = 0; v < plan.size() && spoiled < 1; ++v) {
      if (plan[v].use != Use::HeldOut) continue;
      views[v].points[views[v].points.size() / 2].pixel.x += px;
      ++spoiled;
    }
    ASSERT_EQ(spoiled, 1);
    const LensVerdict one = value_of(
        judge_lens(value_of(solve::solve_lens("bm-1", kSize, board(), views))));
    EXPECT_TRUE(one.pass) << px << ' ' << reasons(one);
    EXPECT_EQ(one.holdout_dropped, 1) << px;

    for (std::size_t v = 0; v < plan.size(); ++v) {
      if (plan[v].use != Use::HeldOut) continue;
      for (std::size_t i = 0; i < 10; i += 2) {
        views[v].points[i].pixel.y += px;
      }
    }
    const LensVerdict many = value_of(
        judge_lens(value_of(solve::solve_lens("bm-1", kSize, board(), views))));
    EXPECT_FALSE(many.pass) << px;
    EXPECT_TRUE(mentions(many, "held-out corners")) << px;
  }
}

// With no view held out nothing vouches for the lens; a view with no pose
// reads infinite, and misses both RMS gates.
TEST(JudgeLens, FailsAFitWithNothingHeldOut) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  LensVerdict verdict = value_of(judge_made(bare_fit(lab_lens(), nan)));
  EXPECT_FALSE(verdict.pass);
  ASSERT_EQ(verdict.failed.size(), 1u);
  EXPECT_TRUE(mentions(verdict, "no view was held out"));

  const double inf = std::numeric_limits<double>::infinity();
  verdict = value_of(judge_made(bare_fit(lab_lens(), inf)));
  EXPECT_FALSE(verdict.pass);
  EXPECT_EQ(verdict.failed.size(), 2u);
}

// 0.1 px, or 3 standard errors of the mean if larger: noise alone cannot fail
// a right lens.
TEST(JudgeLens, HoldsEachCellsBiasToItsNoiseFloor) {
  const camera::Vec2d at(100.0, 100.0);
  // 40 corners of spread 0.5 on each axis: 3 standard errors are 0.335 px.
  solve::LensFit noisy = bare_fit(lab_lens());
  add_corners(noisy, 0, Use::Fitted, 40, at, {0.3, 0.0}, 0.5);
  LensVerdict verdict = value_of(judge_made(noisy));
  EXPECT_TRUE(verdict.pass) << reasons(verdict);
  EXPECT_NEAR(verdict.worst_cell_bias_px, 0.3, 1e-12);
  noisy = bare_fit(lab_lens());
  add_corners(noisy, 0, Use::Fitted, 40, at, {0.34, 0.0}, 0.5);
  EXPECT_TRUE(mentions(value_of(judge_made(noisy)), kBias));

  // Tight corners: the 0.1 px bound itself.
  solve::LensFit tight = bare_fit(lab_lens());
  add_corners(tight, 0, Use::Fitted, 40, at, {0.0999, 0.0}, 0.01);
  EXPECT_TRUE(value_of(judge_made(tight)).pass);
  tight = bare_fit(lab_lens());
  add_corners(tight, 0, Use::Fitted, 40, at, {0.1001, 0.0}, 0.01);
  EXPECT_TRUE(mentions(value_of(judge_made(tight)), kBias));
}

// Too few corners in a cell, those not fitted and those dropped say nothing.
TEST(JudgeLens, JudgesOnlyCellsOfEnoughFittedCorners) {
  const camera::Vec2d at(100.0, 100.0);
  const camera::Vec2d bias(1.0, 0.0);
  solve::LensFit fit = bare_fit(lab_lens());
  add_corners(fit, 0, Use::Fitted, 29, at, bias, 0.01);
  EXPECT_TRUE(value_of(judge_made(fit)).pass);
  add_corners(fit, 1, Use::Fitted, 1, at, bias, 0.01);
  EXPECT_TRUE(mentions(value_of(judge_made(fit)), kBias));

  fit = bare_fit(lab_lens());
  add_corners(fit, 0, Use::HeldOut, 40, at, bias, 0.01);
  add_corners(fit, 1, Use::Fitted, 40, at, bias, 0.01);
  for (solve::CornerResidual& r : fit.residuals) {
    r.dropped = r.capture == 1;
  }
  EXPECT_TRUE(value_of(judge_made(fit)).pass);

  // Cells are the grid's: the same corners in one cell each are judged apart.
  fit = bare_fit(lab_lens());
  add_corners(fit, 0, Use::Fitted, 20, {100.0, 100.0}, bias, 0.01);
  add_corners(fit, 1, Use::Fitted, 20, {700.0, 100.0}, bias, 0.01);
  EXPECT_TRUE(value_of(judge_made(fit)).pass);
  add_corners(fit, 2, Use::Fitted, 20, {110.0, 110.0}, bias, 0.01);
  add_corners(fit, 3, Use::Fitted, 10, {3700.0, 2000.0}, -bias, 0.01);
  const LensVerdict verdict = value_of(judge_made(fit));
  EXPECT_TRUE(mentions(verdict, "cell 0, 0"));
}

// The worst cell is named by column and row; with none judged, by neither.
TEST(JudgeLens, NamesTheWorstCell) {
  EXPECT_EQ(value_of(judge_made(bare_fit(lab_lens()))).worst_cell,
            (std::array<int, 2>{-1, -1}));
  solve::LensFit fit = bare_fit(lab_lens());
  add_corners(fit, 0, Use::Fitted, 40, {100.0, 100.0}, {0.02, 0.0}, 0.01);
  add_corners(fit, 1, Use::Fitted, 40, {1000.0, 400.0}, {0.05, 0.0}, 0.01);
  add_corners(fit, 2, Use::Fitted, 40, {3700.0, 100.0}, {0.03, 0.0}, 0.01);
  const LensVerdict verdict = value_of(judge_made(fit));
  EXPECT_EQ(verdict.worst_cell, (std::array<int, 2>{2, 1}));
  EXPECT_NEAR(verdict.worst_cell_bias_px, 0.05, 1e-12);
}

// The fit from `start` alone on the views `like` fitted: calibrateCamera as
// the solve calls it, each corner's residual as the solve gives it, `like`'s
// held-out figures kept, so the gate on the RMS passes what it passed.
solve::LensFit fit_from_alone(const solve::LensFit& like,
                              const std::vector<Observation>& views,
                              const camera::CameraModel& start) {
  std::vector<std::vector<cv::Point3f>> object;
  std::vector<std::vector<cv::Point2f>> pixels;
  std::vector<const Observation*> fitted;
  for (const solve::LensView& use : like.views) {
    if (use.use != Use::Fitted) continue;
    for (const Observation& view : views) {
      if (view.frame_id != use.frame_id) continue;
      fitted.push_back(&view);
      object.emplace_back();
      pixels.emplace_back();
      for (const DetectedPoint& point : view.points) {
        object.back().emplace_back(
            board().object_points()[static_cast<std::size_t>(point.id)]);
        pixels.back().emplace_back(static_cast<float>(point.pixel.x),
                                   static_cast<float>(point.pixel.y));
      }
    }
  }
  const compute::CvCamera cam = value_of(compute::cv_camera(start));
  cv::Mat matrix = cv::Mat(cam.camera_matrix).clone();
  cv::Mat distortion = cv::Mat(cam.distortion).clone();
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  const double rms = cv::calibrateCamera(
      object, pixels, cam.size, matrix, distortion, rvecs, tvecs,
      cv::CALIB_RATIONAL_MODEL | cv::CALIB_USE_INTRINSIC_GUESS,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500,
                       1e-14));
  solve::LensFit fit = like;
  fit.rms_px = rms;
  fit.camera.model.intrinsics = {
      matrix.at<double>(0, 0), matrix.at<double>(1, 1), matrix.at<double>(0, 2),
      matrix.at<double>(1, 2)};
  const double* d = distortion.ptr<double>();
  fit.camera.model.distortion = {d[0], d[1], d[2], d[3],
                                 d[4], d[5], d[6], d[7]};
  fit.residuals.clear();
  for (std::size_t v = 0; v < fitted.size(); ++v) {
    std::vector<cv::Point3d> corners;
    for (const DetectedPoint& point : fitted[v]->points) {
      corners.push_back(
          board().object_points()[static_cast<std::size_t>(point.id)]);
    }
    std::vector<cv::Point2d> projected;
    cv::projectPoints(corners, rvecs[v], tvecs[v], matrix, distortion,
                      projected);
    for (std::size_t i = 0; i < corners.size(); ++i) {
      solve::CornerResidual r;
      r.capture = fitted[v]->frame_id;
      r.corner = fitted[v]->points[i].id;
      r.pixel = fitted[v]->points[i].pixel;
      r.residual = {projected[i].x - r.pixel.x, projected[i].y - r.pixel.y};
      fit.residuals.push_back(r);
    }
  }
  return fit;
}

// A start as a poor habit gives one: the lens of the views within 45 degrees
// alone, right there and wild past it.
camera::CameraModel central_lens(const std::vector<Observation>& views) {
  const test::FisheyeTruth lens = truth();
  std::vector<Observation> central;
  for (const Observation& view : views) {
    if (std::all_of(view.points.begin(), view.points.end(),
                    [&](const DetectedPoint& p) {
                      return lens.ray({p.pixel.x, p.pixel.y}).z >
                             std::cos(45.0 * kDeg);
                    })) {
      central.push_back(view);
    }
  }
  solve::LensOptions options;
  options.min_views = 5;
  return value_of(solve::solve_lens("bm-1", kSize, board(), central,
                                    std::nullopt, options))
      .camera.model;
}

// A poor start alone lands in a wrong minimum, as a fit of these views often
// does; the views' poses hide it from an RMS. With OpenCV's own start also
// tried, the lower fit is the right one. OpenCV's path differs in its last
// bits between machines, so a seed may land right, but not most.
TEST(JudgeLens, AWrongMinimumFailsTheBiasGateAndTheLowerFitPasses) {
  int wrong = 0;
  for (const std::uint32_t seed : {2u, 3u, 4u, 5u}) {
    const std::vector<Observation> views = session(seed);
    const solve::LensFit right =
        value_of(solve::solve_lens("bm-1", kSize, board(), views));
    const camera::CameraModel poor = central_lens(views);
    const solve::LensFit alone = fit_from_alone(right, views, poor);
    if (alone.rms_px > 1.5 * right.rms_px) {
      ++wrong;
      const LensVerdict verdict = value_of(judge_lens(alone));
      EXPECT_FALSE(verdict.pass) << seed;
      EXPECT_TRUE(mentions(verdict, kBias)) << seed;
    }
    const solve::LensFit both =
        value_of(solve::solve_lens("bm-1", kSize, board(), views, poor));
    EXPECT_NEAR(both.rms_px, right.rms_px, 1e-3) << seed;
    EXPECT_TRUE(value_of(judge_lens(both)).pass) << seed;
    if (alone.rms_px > 1.5 * right.rms_px) {
      EXPECT_FALSE(both.from_start);
    }
  }
  EXPECT_GE(wrong, 3);
}

// A real lens bent past 45 degrees, where an R8 curve cannot follow: 3 px by
// 48 degrees. The RMS gates pass it on most seeds.
TEST(JudgeLens, ALensBentPast45DegreesFailsTheBiasGate) {
  const test::FisheyeTruth lens = truth();
  int fired = 0;
  for (const std::uint32_t seed : {1u, 2u, 3u, 4u, 5u}) {
    std::vector<Observation> views = session(seed);
    for (Observation& view : views) {
      for (DetectedPoint& point : view.points) {
        const double off =
            std::acos(lens.ray({point.pixel.x, point.pixel.y}).z) / kDeg;
        if (off <= 45.0) continue;
        const double t = std::min((off - 45.0) / 3.0, 1.0);
        const camera::Vec2d out =
            glm::normalize(point.pixel - camera::Vec2d(lens.cx, lens.cy));
        point.pixel += 3.0 * t * t * (3.0 - 2.0 * t) * out;
      }
    }
    const solve::LensFit fit =
        value_of(solve::solve_lens("bm-1", kSize, board(), views));
    EXPECT_LE(fit.holdout_rms_px, 0.5) << seed;
    if (mentions(value_of(judge_lens(fit)), kBias)) ++fired;
  }
  EXPECT_GE(fired, 3);
}

TEST(JudgeLens, TheFoldGateFailsThe2024LensAndPassesTheLabs) {
  const LensVerdict bad = value_of(judge_made(bare_fit(lens_2024_bm2())));
  EXPECT_TRUE(bad.frame_folds);
  EXPECT_FALSE(bad.pass);
  EXPECT_NEAR(bad.fold_deg, 74.8, 0.1);
  EXPECT_TRUE(mentions(bad, "folds"));

  const LensVerdict good = value_of(judge_made(bare_fit(lab_lens())));
  EXPECT_FALSE(good.frame_folds);
  EXPECT_TRUE(good.pass);
  EXPECT_EQ(good.fold_deg, 90.0);
}

// k1 < 0 folds at r^2 = -1 / (3 k1), its largest radius (2/3) r there: here
// 1.826 and 1.217. An image corner in pixels is far past that number; in
// normalized radius it is not.
TEST(JudgeLens, TheFoldGateComparesNormalizedRadii) {
  camera::CameraModel model{kSize, {2000.0, 2000.0, 1919.5, 1079.5}, {}};
  model.distortion.k1 = -0.1;
  LensVerdict verdict = value_of(judge_made(bare_fit(model)));
  EXPECT_NEAR(verdict.fold_deg, 61.29, 0.01);
  EXPECT_FALSE(verdict.frame_folds);  // the corners at 1.10
  EXPECT_TRUE(verdict.pass);

  model.intrinsics.fx = model.intrinsics.fy = 1500.0;  // at 1.47
  verdict = value_of(judge_made(bare_fit(model)));
  EXPECT_TRUE(verdict.frame_folds);
  EXPECT_FALSE(verdict.pass);

  // Each axis by its own focal length: at 0.96 and 1.08, so at 1.44.
  model.intrinsics.fx = 2000.0;
  model.intrinsics.fy = 1000.0;
  EXPECT_TRUE(value_of(judge_made(bare_fit(model))).frame_folds);

  // Every image corner counts: with the axis at the right edge, those at the
  // left are at 2.0 and those at the right at 0.6.
  model.intrinsics = {2000.0, 2000.0, 3839.0, 1079.5};
  EXPECT_TRUE(value_of(judge_made(bare_fit(model))).frame_folds);
}

// A pole the frame lies well inside bounds nothing: the model's image runs out
// to infinity before it, and every image corner reads nearer the axis.
TEST(JudgeLens, TheFoldGatePassesAPoleOutsideTheFrame) {
  camera::CameraModel model{kSize, {2000.0, 2000.0, 1919.5, 1079.5}, {}};
  model.distortion.k4 = -0.2;  // the denominator's zero at r^2 = 5
  const LensVerdict verdict = value_of(judge_made(bare_fit(model)));
  EXPECT_NEAR(verdict.fold_deg, std::atan(std::sqrt(5.0)) / kDeg, 0.01);
  EXPECT_FALSE(verdict.frame_folds);
}

// A fit that never saw the corners: its numerator all but cancels a pole 55.8
// degrees off axis, so the curve runs on past a spike, and the image's corners
// read 79 degrees through it. The runtime cuts every ray at that number.
TEST(JudgeLens, TheFoldGateFailsAPoleInsideTheFrame) {
  const camera::CameraModel model{
      kSize,
      {1701.4449259028816, 1701.4119732999638, 1917.2643614302251,
       1108.9347303641816},
      {0.043876244510601972, -0.21896430916385881, 2.7458829657533664e-05,
       -4.400396484662529e-05, -0.0068247300392786274, 0.40966669851916004,
       -0.29805089860040429, -0.048390877236838249}};
  const LensVerdict verdict = value_of(judge_made(bare_fit(model)));
  EXPECT_NEAR(verdict.fold_deg, 55.8, 0.1);
  EXPECT_TRUE(verdict.frame_folds);
  EXPECT_FALSE(verdict.pass);
  EXPECT_TRUE(mentions(verdict, "folds")) << reasons(verdict);
}

TEST(JudgeLens, Refuses) {
  const solve::LensFit fit = bare_fit(lab_lens());
  LensGateOptions options;
  options.max_holdout_rms_px = 0.0;
  expect_refused(judge_lens(fit, options).status(), "judge_lens",
                 "options.max_holdout_rms_px");
  options = {};
  options.max_holdout_ratio = std::numeric_limits<double>::quiet_NaN();
  expect_refused(judge_lens(fit, options).status(), "judge_lens",
                 "options.max_holdout_ratio");
  options = {};
  options.max_cell_bias_px = -1.0;
  expect_refused(judge_lens(fit, options).status(), "judge_lens",
                 "options.max_cell_bias_px");
  options = {};
  options.min_cell_corners = 1;
  expect_refused(judge_lens(fit, options).status(), "judge_lens",
                 "options.min_cell_corners");
  options = {};
  options.min_reach = 1.5;
  expect_refused(judge_lens(fit, options).status(), "judge_lens",
                 "options.min_reach");
  options = {};
  options.min_ratio_holdout_rms_px = -0.1;
  expect_refused(judge_lens(fit, options).status(), "judge_lens",
                 "options.min_ratio_holdout_rms_px");
  options = {};
  options.max_holdout_dropped = 2.0;
  expect_refused(judge_lens(fit, options).status(), "judge_lens",
                 "options.max_holdout_dropped");

  solve::LensFit bad = fit;
  bad.camera.model.intrinsics.fx = 0.0;
  EXPECT_EQ(judge_lens(bad).status().domain(),
            core::Status::Code::InvalidArgument);

  bad = fit;
  bad.residuals.emplace_back().capture = 9;
  expect_refused(judge_lens(bad).status(), "judge_lens", "capture 9");
}

// A file of three other kinds of entry, which the lens-only file treats as it
// should: `bm-1`'s old lens, posed; `bm-3`'s calibrated one, posed, with a
// `bm-2` beside it; and a Femto's factory lens with its depth.
camera::ArrayCalibration base_file() {
  camera::ArrayCalibration file;
  file.world.apriltag = camera::AprilTagWorld{"tag36h11", 0, 0.15};
  file.color_controls = camera::ColorControls{167, 60};
  const auto add = [&](const char* id, camera::IntrinsicsSource source) {
    camera::SensorCalibration& sensor = file.sensors.emplace_back();
    sensor.id = id;
    sensor.color_to_world = camera::Mat4d(1.0);
    sensor.color = camera::CameraCalibration{lens_2024_bm2(), source};
    return &sensor;
  };
  add("bm-1", camera::IntrinsicsSource::Calibrated);
  add("bm-2", camera::IntrinsicsSource::Calibrated);
  add("bm-3", camera::IntrinsicsSource::Calibrated);
  camera::SensorCalibration* femto =
      add("femto", camera::IntrinsicsSource::Factory);
  femto->depth =
      camera::CameraCalibration{lab_lens(), camera::IntrinsicsSource::Factory};
  femto->depth_to_color = camera::Mat4d(1.0);
  camera::SensorCalibration& unlensed = file.sensors.emplace_back();
  unlensed.id = "posed-only";
  unlensed.color_to_world = camera::Mat4d(1.0);
  return file;
}

LensVerdict verdict_of(const std::string& id) {
  solve::LensFit fit = right_fit();
  fit.camera.id = id;
  return value_of(judge_lens(std::move(fit)));
}

TEST(FormatLensResults, TheLensFileReadsBackThroughSensor) {
  const camera::ArrayCalibration base = base_file();
  LensVerdict failed = verdict_of("bm-1");
  failed.pass = false;  // written anyway, for a trial pose
  failed.failed = {"a gate"};
  const LensVerdict factory = verdict_of("bm-2");
  const ResultFiles files = value_of(
      format_lens_results({failed, factory}, {"bm-4"}, &base, {"bm-2"}));

  const json doc = json::parse(files.array_calibration);
  EXPECT_EQ(doc["format"], "volumetric_kit.array_calibration");
  EXPECT_EQ(doc["version"], 2);
  EXPECT_FALSE(doc.contains("color_controls"));
  EXPECT_FALSE(doc.contains("world"));

  const camera::ArrayCalibration back =
      value_of(camera::parse_array_calibration(files.array_calibration));
  EXPECT_TRUE(back.world.sensor.empty());
  EXPECT_FALSE(back.world.apriltag);
  EXPECT_FALSE(back.color_controls);
  ASSERT_EQ(back.sensors.size(), 3u);  // bm-1, bm-3, femto

  const camera::SensorCalibration* solved = camera::find_sensor(back, "bm-1");
  ASSERT_NE(solved, nullptr);
  ASSERT_TRUE(solved->color);
  EXPECT_EQ(solved->color->source, camera::IntrinsicsSource::Calibrated);
  const camera::CameraModel& model = right_fit().camera.model;
  EXPECT_EQ(solved->color->model.size.width, model.size.width);
  EXPECT_EQ(solved->color->model.size.height, model.size.height);
  EXPECT_EQ(solved->color->model.intrinsics.fx, model.intrinsics.fx);
  EXPECT_EQ(solved->color->model.distortion.k6, model.distortion.k6);
  EXPECT_FALSE(solved->color_to_world);
  EXPECT_FALSE(solved->depth);

  const camera::SensorCalibration* carried = camera::find_sensor(back, "bm-3");
  ASSERT_NE(carried, nullptr);
  ASSERT_TRUE(carried->color);
  EXPECT_EQ(carried->color->source, camera::IntrinsicsSource::Calibrated);
  EXPECT_EQ(carried->color->model.intrinsics.cx, lens_2024_bm2().intrinsics.cx);
  EXPECT_FALSE(carried->color_to_world);

  const camera::SensorCalibration* femto = camera::find_sensor(back, "femto");
  ASSERT_NE(femto, nullptr);
  ASSERT_TRUE(femto->color);
  EXPECT_EQ(femto->color->source, camera::IntrinsicsSource::Factory);
  EXPECT_FALSE(femto->depth);
  EXPECT_FALSE(femto->depth_to_color);

  // A report-only camera is not written, nor carried from the base; a
  // camera with no lens has nothing to carry.
  EXPECT_EQ(camera::find_sensor(back, "bm-2"), nullptr);
  EXPECT_EQ(camera::find_sensor(back, "posed-only"), nullptr);
}

TEST(FormatLensResults, TheReportHoldsWhatTheGatesFound) {
  const LensVerdict solved = verdict_of("bm-1");
  LensVerdict factory = verdict_of("femto");
  factory.fit.from_start = true;
  factory.fit.views[3].use = Use::Dropped;
  const ResultFiles files = value_of(
      format_lens_results({solved, factory}, {"bm-4"}, nullptr, {"femto"}));
  const json doc = json::parse(files.report);
  EXPECT_EQ(doc["format"], "volumetric_kit_calib.report");
  EXPECT_EQ(doc["version"], 2);
  EXPECT_EQ(doc.size(), 3u);  // format, version and lenses
  const json& lenses = doc["lenses"];
  EXPECT_EQ(lenses["unsolved"], json::array({"bm-4"}));
  ASSERT_EQ(lenses["cameras"].size(), 2u);

  const json& cam = lenses["cameras"]["bm-1"];
  const solve::LensFit& fit = solved.fit;
  EXPECT_EQ(cam["verdict"], "pass");
  EXPECT_TRUE(cam["failed"].empty());
  EXPECT_EQ(cam["written"], true);
  EXPECT_EQ(cam["start"], "opencv");
  EXPECT_EQ(cam["rms_px"], fit.rms_px);
  EXPECT_EQ(cam["holdout_rms_px"], fit.holdout_rms_px);
  EXPECT_EQ(cam["holdout_corners"], solved.holdout_corners);
  EXPECT_GT(solved.holdout_corners, 0);
  EXPECT_EQ(cam["holdout_dropped"], 0);
  EXPECT_EQ(cam["worst_cell_bias_px"], solved.worst_cell_bias_px);
  EXPECT_EQ(cam["worst_cell"]["column"], solved.worst_cell[0]);
  EXPECT_EQ(cam["worst_cell"]["row"], solved.worst_cell[1]);
  EXPECT_EQ(cam["fold_deg"], 90.0);
  EXPECT_EQ(cam["frame_folds"], false);
  EXPECT_EQ(cam["reach"]["top_left"], solved.reach[0]);
  EXPECT_EQ(cam["reach"]["top_right"], solved.reach[1]);
  EXPECT_EQ(cam["reach"]["bottom_left"], solved.reach[2]);
  EXPECT_EQ(cam["reach"]["bottom_right"], solved.reach[3]);
  EXPECT_EQ(cam["reach"]["left"], solved.reach[4]);
  EXPECT_EQ(cam["reach"]["right"], solved.reach[5]);
  EXPECT_EQ(cam["corner_reach_deg"]["top_left"], solved.corner_reach_deg[0]);
  EXPECT_EQ(cam["corner_reach_deg"]["top_right"], solved.corner_reach_deg[1]);
  EXPECT_EQ(cam["corner_reach_deg"]["bottom_left"], solved.corner_reach_deg[2]);
  EXPECT_EQ(cam["corner_reach_deg"]["bottom_right"],
            solved.corner_reach_deg[3]);
  EXPECT_EQ(cam["side_reach_deg"]["left"], solved.side_reach_deg[0]);
  EXPECT_EQ(cam["side_reach_deg"]["right"], solved.side_reach_deg[1]);
  EXPECT_EQ(cam["sigma"]["cy"], fit.sigma.cy);
  EXPECT_EQ(cam["lens"]["width"], 3840);
  EXPECT_EQ(cam["lens"]["intrinsics"]["fx"], fit.camera.model.intrinsics.fx);
  EXPECT_EQ(cam["lens"]["distortion"]["model"], "rational");
  EXPECT_EQ(cam["lens"]["distortion"]["k4"], fit.camera.model.distortion.k4);

  int fitted = 0;
  int held_out = 0;
  for (const solve::LensView& view : fit.views) {
    fitted += view.use == Use::Fitted;
    held_out += view.use == Use::HeldOut;
  }
  EXPECT_EQ(cam["view_counts"]["fitted"], fitted);
  EXPECT_EQ(cam["view_counts"]["held_out"], held_out);
  EXPECT_EQ(cam["view_counts"]["dropped"], 0);
  EXPECT_EQ(cam["view_counts"]["too_few"], 0);
  ASSERT_EQ(cam["views"].size(), fit.views.size());
  EXPECT_EQ(cam["views"][1]["capture"], fit.views[1].frame_id);
  EXPECT_EQ(cam["views"][1]["use"], solve::to_string(fit.views[1].use));
  EXPECT_EQ(cam["views"][1]["corners"], fit.views[1].corners);
  EXPECT_EQ(cam["views"][1]["rms_px"], fit.views[1].rms_px);

  const json& other = lenses["cameras"]["femto"];
  EXPECT_EQ(other["written"], false);
  EXPECT_EQ(other["start"], "given");
  EXPECT_EQ(other["view_counts"]["dropped"], 1);
  EXPECT_EQ(other["views"][3]["use"], "dropped");
}

// A gate failed is named; a figure not finite is null, as in every report.
TEST(FormatLensResults, TheReportNamesAGateMissed) {
  LensVerdict verdict = verdict_of("bm-1");
  verdict.pass = false;
  verdict.failed = {"the lens folds", "a pattern"};
  verdict.fit.holdout_rms_px = std::numeric_limits<double>::quiet_NaN();
  const json cam =
      json::parse(value_of(format_lens_results({verdict}, {}, nullptr))
                      .report)["lenses"]["cameras"]["bm-1"];
  EXPECT_EQ(cam["verdict"], "fail");
  EXPECT_EQ(cam["failed"], json::array({"the lens folds", "a pattern"}));
  EXPECT_TRUE(cam["holdout_rms_px"].is_null());
}

TEST(FormatLensResults, TheResidualsAreEveryCornerByCamera) {
  LensVerdict one = verdict_of("bm-1");
  LensVerdict two = verdict_of("bm-2");
  one.fit.residuals.resize(3);
  one.fit.residuals[1].dropped = true;
  two.fit.residuals.resize(2);
  const solve::CornerResidual& r = two.fit.residuals[1];
  const std::string text =
      value_of(format_lens_results({one, two}, {}, nullptr)).residuals;
  std::istringstream lines(text);
  std::vector<std::string> rows;
  for (std::string line; std::getline(lines, line);) rows.push_back(line);
  ASSERT_EQ(rows.size(), 6u);
  EXPECT_EQ(rows[0], "capture,serial,corner_id,u,v,du,dv,weight,dropped");
  EXPECT_EQ(
      rows[1].find(std::to_string(one.fit.residuals[0].capture) + ",bm-1,"),
      0u);
  EXPECT_EQ(rows[2].back(), '1');
  EXPECT_EQ(rows[3].back(), '0');
  EXPECT_EQ(rows[5].find(std::to_string(r.capture) + ",bm-2," +
                         std::to_string(r.corner) + ","),
            0u);
}

// No lens to write: the report still says why, and there is no file.
TEST(FormatLensResults, HasNoFileWithNoLensToWrite) {
  ResultFiles files = value_of(
      format_lens_results({verdict_of("femto")}, {}, nullptr, {"femto"}));
  EXPECT_TRUE(files.array_calibration.empty());
  EXPECT_FALSE(files.report.empty());
  files = value_of(format_lens_results({}, {"bm-1"}, nullptr));
  EXPECT_TRUE(files.array_calibration.empty());
  EXPECT_EQ(json::parse(files.report)["lenses"]["unsolved"],
            json::array({"bm-1"}));
}

TEST(FormatLensResults, Refuses) {
  const LensVerdict lens = verdict_of("bm-1");
  expect_refused(format_lens_results({lens, lens}, {}, nullptr).status(),
                 "format_lens_results", "\"bm-1\" is named twice");
  expect_refused(format_lens_results({lens}, {"bm-1"}, nullptr).status(),
                 "format_lens_results", "\"bm-1\" is named twice");
  expect_refused(format_lens_results({}, {""}, nullptr).status(),
                 "format_lens_results", "no id");
  expect_refused(
      format_lens_results({lens}, {"bm-2"}, nullptr, {"bm-2"}).status(),
      "format_lens_results", "report-only camera \"bm-2\" has no lens");
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
