// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// One camera's lens from its views, on a fisheye truth an R8 lens can only
// approach (cv::fisheye's model, the lab's lens): the curve within 0.3 px to 68
// degrees, whatever turn of the camera's frame the views' poses absorbed; the
// true centre within 3 sigma; views held out in every region of any capture
// rhythm, and never a corner's only reaching one; the outlier rule's drops,
// named; and the plan and the fit independent of the views' order.

#include "volumetric_kit/calib/solve/lens.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/geometry.hpp>

#include "result_checks.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/cv_pose.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/solve/view_pose.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"

namespace volumetric_kit::calib::solve {
namespace {

namespace camera = sensor::camera;
using Code = core::Status::Code;
using Use = LensView::Use;
using test::curve_error_px;
using test::expect_refused;
using test::value_of;

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

// The session of `seed` at 0.15 px of corner noise, and its fit: made once,
// for the cases that read them as they are.
const std::vector<Observation>& noisy_views(std::uint32_t seed = 1) {
  static std::map<std::uint32_t, std::vector<Observation>> made;
  const auto at = made.find(seed);
  if (at != made.end()) return at->second;
  test::FisheyeCapture capture;
  capture.seed = seed;
  capture.noise_px = 0.15;
  return made[seed] = test::fisheye_views(board(), truth(), capture);
}

const LensFit& noisy_fit(std::uint32_t seed = 1) {
  static std::map<std::uint32_t, LensFit> made;
  const auto at = made.find(seed);
  if (at != made.end()) return at->second;
  return made[seed] =
             value_of(solve_lens("bm-1", kSize, board(), noisy_views(seed)));
}

// Every other view of seed 1's session: as widely spread with half the views,
// fitted with LensOptions::min_views at 10.
std::vector<Observation> half_session() {
  std::vector<Observation> views;
  for (std::size_t v = 0; v < noisy_views().size(); v += 2) {
    views.push_back(noisy_views()[v]);
  }
  return views;
}

// A 4 x 4 patch of the board's corners, 40 px apart, from (u, v).
Observation patch(std::int64_t frame, double u, double v) {
  Observation view;
  view.camera_id = "bm-1";
  view.frame_id = frame;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      view.points.push_back(
          {row * 13 + col, {u + 40.0 * col, v + 40.0 * row}, 1.0});
    }
  }
  return view;
}

std::map<std::int64_t, Use> uses_by_capture(
    const std::vector<LensView>& views) {
  std::map<std::int64_t, Use> uses;
  for (const LensView& view : views) uses[view.frame_id] = view.use;
  return uses;
}

int count(const std::vector<LensView>& views, Use use) {
  return static_cast<int>(
      std::count_if(views.begin(), views.end(),
                    [&](const LensView& v) { return v.use == use; }));
}

TEST(PixelReach, IsTheFractionOfTheWayTowardEachCornerAndSide) {
  const LensReach at_centre = pixel_reach(kSize, {1919.5, 1079.5});
  for (const double r : at_centre) EXPECT_EQ(r, 0.0);

  const LensReach top_left = pixel_reach(kSize, {-0.5, -0.5});
  EXPECT_DOUBLE_EQ(top_left[0], 1.0);  // the corner
  EXPECT_EQ(top_left[3], 0.0);         // the far corner
  EXPECT_DOUBLE_EQ(top_left[4], 1.0);  // the left side
  EXPECT_EQ(top_left[5], 0.0);

  const LensReach bottom_right = pixel_reach(kSize, {3839.5, 2159.5});
  EXPECT_DOUBLE_EQ(bottom_right[3], 1.0);
  EXPECT_DOUBLE_EQ(bottom_right[5], 1.0);

  // Nine tenths of the way along the top-right diagonal.
  const LensReach along =
      pixel_reach(kSize, {1919.5 + 0.9 * 1920.0, 1079.5 - 0.9 * 1080.0});
  EXPECT_NEAR(along[1], 0.9, 1e-12);
  EXPECT_NEAR(along[5], 0.9, 1e-12);

  // The middle of a side is not an image corner.
  const LensReach right = pixel_reach(kSize, {3839.5, 1079.5});
  EXPECT_DOUBLE_EQ(right[5], 1.0);
  EXPECT_NEAR(right[1], 1920.0 * 1920.0 / (1920.0 * 1920.0 + 1080.0 * 1080.0),
              1e-12);
}

// Five positions in turn: every 5th view overall would hold out one position
// alone, all its views, and none of the rest.
TEST(PlanLensViews, HoldsOutViewsInEveryRegionOfAFivePositionOrder) {
  const std::vector<cv::Point2d> positions = {
      {300, 200}, {3200, 200}, {1800, 900}, {300, 1800}, {3200, 1800}};
  std::vector<Observation> views;
  for (std::int64_t frame = 0; frame < 40; ++frame) {
    const cv::Point2d& at = positions[static_cast<std::size_t>(frame) % 5];
    views.push_back(patch(frame, at.x, at.y));
  }
  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  ASSERT_EQ(plan.size(), 40u);
  std::set<std::size_t> held;
  for (const LensView& view : plan) {
    if (view.use == Use::HeldOut) {
      held.insert(static_cast<std::size_t>(view.frame_id) % 5);
    }
  }
  EXPECT_EQ(held.size(), 5u);
  EXPECT_EQ(count(plan, Use::HeldOut), 10);  // of 8 views in each: 2
  EXPECT_EQ(count(plan, Use::Fitted), 30);
}

TEST(PlanLensViews, TakesARegionsThirdViewOfEveryFive) {
  std::vector<Observation> views;
  for (std::int64_t frame = 0; frame < 3; ++frame) {
    views.push_back(patch(frame, 1800.0, 900.0));
  }
  LensOptions options;
  std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views, options));
  EXPECT_EQ(plan[0].use, Use::Fitted);
  EXPECT_EQ(plan[1].use, Use::Fitted);
  EXPECT_EQ(plan[2].use, Use::HeldOut);

  views.pop_back();
  plan = value_of(plan_lens_views("bm-1", kSize, board(), views, options));
  EXPECT_EQ(count(plan, Use::HeldOut), 0);  // a region of 2 gives none
}

// A corner's only reaching view stays fitted, wherever it falls in the rhythm;
// with a second one, the rule holds the first out as usual.
TEST(PlanLensViews, NeverHoldsOutACornersOnlyReachingView) {
  // Capture 2 is the region's third, the one held out, and reaches the top
  // left: nine tenths of the way is about 96, 54 from the corner, and the
  // patch starts inside that.
  std::vector<Observation> views = {
      patch(0, 900.0, 500.0), patch(1, 900.0, 500.0), patch(2, 20.0, 20.0)};
  std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  ASSERT_GE(pixel_reach(kSize, views[2].points[0].pixel)[0], kLensReach);
  EXPECT_EQ(count(plan, Use::HeldOut), 0);

  // A second view reaching it: capture 2 may go, the last one stays.
  views.push_back(patch(3, 20.0, 20.0));
  plan = value_of(plan_lens_views("bm-1", kSize, board(), views));
  EXPECT_EQ(plan[2].use, Use::HeldOut);
  EXPECT_EQ(plan[3].use, Use::Fitted);

  // The same for a side alone: its view is a patch at the side's middle.
  views = {patch(0, 900.0, 900.0), patch(1, 900.0, 900.0),
           patch(2, 20.0, 1000.0)};
  ASSERT_GE(pixel_reach(kSize, views[2].points[0].pixel)[4], kLensReach);
  ASSERT_LT(pixel_reach(kSize, views[2].points[0].pixel)[0], kLensReach);
  plan = value_of(plan_lens_views("bm-1", kSize, board(), views));
  EXPECT_EQ(count(plan, Use::HeldOut), 0);
}

// Two reaching views, the 3rd and 8th of one region, both due to be held out:
// the first goes, and the second, the last left, stays.
TEST(PlanLensViews, KeepsTheSecondOfTwoReachingViewsBothDueToBeHeldOut) {
  std::vector<Observation> views;
  for (std::int64_t frame = 0; frame < 8; ++frame) {
    const bool reaching = frame == 2 || frame == 7;
    views.push_back(patch(frame, reaching ? 20.0 : 300.0, 20.0));
  }
  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  EXPECT_EQ(plan[2].use, Use::HeldOut);
  EXPECT_EQ(plan[7].use, Use::Fitted);
}

// A view belongs to the region its corners centre in, wherever its first
// corner falls: a board's ids run either way across the image.
TEST(PlanLensViews, PlacesAViewByTheCentreOfItsCorners) {
  std::vector<Observation> views;
  for (std::int64_t frame = 0; frame < 3; ++frame) {
    views.push_back(patch(frame, 1200.0, 500.0));  // centred left of 1280
  }
  ASSERT_LT(views[2].points.front().pixel.x, 1280.0);
  ASSERT_GT(views[2].points.back().pixel.x, 1280.0);
  std::reverse(views[2].points.begin(), views[2].points.end());
  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  EXPECT_EQ(plan[2].use, Use::HeldOut);
}

// The hold-out goes by capture, not by the order the caller keeps views in.
TEST(PlanLensViews, DependsOnCapturesNotTheViewsOrder) {
  std::vector<Observation> views =
      test::fisheye_views(board(), truth(), test::FisheyeCapture{});
  // Keep frame ids as they are, in a different order.
  const std::map<std::int64_t, Use> in_order =
      uses_by_capture(value_of(plan_lens_views("bm-1", kSize, board(), views)));
  std::mt19937 rng(7);
  std::shuffle(views.begin(), views.end(), rng);
  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  EXPECT_EQ(uses_by_capture(plan), in_order);
  EXPECT_TRUE(std::is_sorted(plan.begin(), plan.end(),
                             [](const LensView& a, const LensView& b) {
                               return a.frame_id < b.frame_id;
                             }));
}

TEST(PlanLensViews, TakesOnlyViewsThatFixAPoseOfEnoughCorners) {
  Observation few = patch(0, 900.0, 500.0);
  few.points.resize(11);
  Observation line = patch(1, 900.0, 500.0);  // every corner in one row
  for (int col = 0; col < 13; ++col) {
    line.points[static_cast<std::size_t>(col) % 16].id = col;
  }
  line.points.resize(13);
  Observation other = patch(2, 900.0, 500.0);
  other.camera_id = "bm-2";
  const std::vector<LensView> plan = value_of(plan_lens_views(
      "bm-1", kSize, board(), {few, line, other, patch(3, 900.0, 500.0)}));
  ASSERT_EQ(plan.size(), 3u);  // bm-2's is not this camera's
  EXPECT_EQ(plan[0].use, Use::TooFew);
  EXPECT_EQ(plan[0].corners, 11);
  EXPECT_EQ(plan[1].use, Use::TooFew);
  EXPECT_EQ(plan[2].use, Use::Fitted);
}

TEST(PlanLensViews, Refuses) {
  const std::vector<Observation> views = {patch(0, 900.0, 500.0)};
  const auto refused = [&](LensOptions options) {
    return plan_lens_views("bm-1", kSize, board(), views, options).status();
  };
  LensOptions options;
  options.min_views = 0;
  expect_refused(refused(options), "plan_lens_views", "options.min_views");
  options = {};
  options.min_corners = 3;
  expect_refused(refused(options), "plan_lens_views", "options.min_corners");
  options = {};
  options.holdout_every = 1;
  expect_refused(refused(options), "plan_lens_views", "options.holdout_every");
  options = {};
  options.outlier_rounds = -1;
  expect_refused(refused(options), "plan_lens_views", "options.outlier_rounds");
  options = {};
  options.max_iterations = 0;
  expect_refused(refused(options), "plan_lens_views", "options.max_iterations");
  expect_refused(plan_lens_views("", kSize, board(), views).status(),
                 "plan_lens_views", "no id");
  expect_refused(plan_lens_views("bm-1", {0, 2160}, board(), views).status(),
                 "plan_lens_views", "an image of 0 x 2160");
  expect_refused(plan_lens_views("bm-1", kSize, board(),
                                 {patch(4, 9.0, 9.0), patch(4, 9.0, 9.0)})
                     .status(),
                 "plan_lens_views", "two views of capture 4");
  Observation bad = patch(5, 900.0, 500.0);
  bad.points[3].pixel.x = std::numeric_limits<double>::quiet_NaN();
  expect_refused(plan_lens_views("bm-1", kSize, board(), {bad}).status(),
                 "plan_lens_views", "capture 5 has a corner not finite");
}

// R8 on a fisheye with corners out to 0.9 of the way: the design's capacity
// check, free of noise, on a seed the plan's boards reach every corner in.
TEST(SolveLens, FollowsAFisheyeCurveToTheFrameWithNoNoise) {
  const test::FisheyeTruth lens = truth();
  for (const std::uint32_t seed : {1u}) {
    test::FisheyeCapture capture;
    capture.seed = seed;
    const std::vector<Observation> views =
        test::fisheye_views(board(), lens, capture);
    ASSERT_GE(views.size(), 38u);
    LensReach reach{};
    for (const Observation& view : views) {
      for (const DetectedPoint& point : view.points) {
        const LensReach here = pixel_reach(kSize, point.pixel);
        for (std::size_t d = 0; d < reach.size(); ++d) {
          reach[d] = std::max(reach[d], here[d]);
        }
      }
    }
    for (const double r : reach) EXPECT_GE(r, kLensReach) << seed;

    const LensFit fit = value_of(solve_lens("bm-1", kSize, board(), views));
    EXPECT_LT(fit.rms_px, 0.05);
    EXPECT_LT(fit.holdout_rms_px, 0.05);
    EXPECT_LT(curve_error_px(fit.camera.model, lens, 68.0), 0.3) << seed;
    EXPECT_NEAR(fit.camera.model.intrinsics.fx, lens.f, 1.0);
  }
}

// The sigma OpenCV reports is honest about the centre: R8's focal length
// trades against its coefficients, its centre does not.
TEST(SolveLens, FindsTheTrueCentreWithinThreeSigma) {
  const test::FisheyeTruth lens = truth();
  for (const std::uint32_t seed : {1u, 2u, 3u}) {
    const LensFit& fit = noisy_fit(seed);
    const camera::PinholeIntrinsics& k = fit.camera.model.intrinsics;
    EXPECT_GT(fit.sigma.cx, 0.0);
    EXPECT_GT(fit.sigma.cy, 0.0);
    EXPECT_LT(std::abs(k.cx - lens.cx), 3.0 * fit.sigma.cx) << seed;
    EXPECT_LT(std::abs(k.cy - lens.cy), 3.0 * fit.sigma.cy) << seed;
    EXPECT_NEAR(fit.rms_px, 0.15 * std::sqrt(2.0), 0.02) << seed;
    EXPECT_NEAR(fit.holdout_rms_px, 0.15 * std::sqrt(2.0), 0.03) << seed;
  }
}

TEST(SolveLens, SaysWhatItDidWithEveryView) {
  const test::FisheyeTruth lens = truth();
  test::FisheyeCapture capture;
  capture.noise_px = 0.15;
  std::vector<Observation> views = test::fisheye_views(board(), lens, capture);
  Observation few = views.back();
  few.frame_id = 100;
  few.points.resize(5);
  views.push_back(few);
  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  const LensFit fit = value_of(solve_lens("bm-1", kSize, board(), views));

  EXPECT_EQ(fit.camera.id, "bm-1");
  EXPECT_EQ(fit.camera.source, camera::IntrinsicsSource::Calibrated);
  EXPECT_EQ(fit.camera.model.size.width, kSize.width);
  EXPECT_EQ(fit.camera.model.size.height, kSize.height);
  EXPECT_FALSE(fit.from_start);
  ASSERT_EQ(fit.views.size(), views.size());
  std::size_t residuals = 0;
  for (std::size_t v = 0; v < fit.views.size(); ++v) {
    const LensView& view = fit.views[v];
    EXPECT_EQ(view.frame_id, plan[v].frame_id);
    EXPECT_EQ(view.use, plan[v].use);
    if (view.use == Use::TooFew) {
      EXPECT_EQ(view.corners, 5);
      EXPECT_EQ(view.rms_px, 0.0);
      continue;
    }
    EXPECT_EQ(view.corners, static_cast<int>(views[v].points.size()));
    EXPECT_GT(view.rms_px, 0.1);
    EXPECT_LT(view.rms_px, 0.4);
    residuals += views[v].points.size();
  }
  EXPECT_GT(count(fit.views, Use::HeldOut), 3);
  EXPECT_EQ(count(fit.views, Use::TooFew), 1);
  // Each residual is the fit's own: projection minus detection, in order.
  ASSERT_EQ(fit.residuals.size(), residuals);
  double sum = 0.0;
  int fitted = 0;
  std::map<std::int64_t, Use> uses = uses_by_capture(fit.views);
  double held_sum = 0.0;
  int held = 0;
  for (const CornerResidual& r : fit.residuals) {
    EXPECT_EQ(r.camera, 0u);
    EXPECT_FALSE(r.dropped);
    const double e2 = glm::dot(r.residual, r.residual);
    if (uses[r.capture] == Use::Fitted) {
      sum += e2;
      ++fitted;
    } else {
      held_sum += e2;
      ++held;
    }
  }
  EXPECT_NEAR(std::sqrt(sum / fitted), fit.rms_px, 1e-6);
  EXPECT_NEAR(std::sqrt(held_sum / held), fit.holdout_rms_px, 1e-12);
}

// A camera's views are picked from any cameras', and their order is the
// captures', so nothing of the caller's bookkeeping moves the lens.
TEST(SolveLens, DependsOnCapturesNotTheViewsOrder) {
  const test::FisheyeTruth lens = truth();
  std::vector<Observation> views = noisy_views();
  const LensFit& in_order = noisy_fit();

  test::FisheyeCapture capture;
  capture.noise_px = 0.15;
  capture.camera = "bm-2";
  capture.seed = 2;
  const std::vector<Observation> others =
      test::fisheye_views(board(), lens, capture);
  views.insert(views.end(), others.begin(), others.end());
  std::mt19937 rng(11);
  std::shuffle(views.begin(), views.end(), rng);
  const LensFit shuffled = value_of(solve_lens("bm-1", kSize, board(), views));
  // OpenCV's threads leave the last digits to chance.
  EXPECT_NEAR(shuffled.camera.model.intrinsics.fx,
              in_order.camera.model.intrinsics.fx, 1e-6);
  EXPECT_NEAR(shuffled.camera.model.distortion.k1,
              in_order.camera.model.distortion.k1, 1e-6);
  EXPECT_NEAR(shuffled.holdout_rms_px, in_order.holdout_rms_px, 1e-6);
  ASSERT_EQ(shuffled.views.size(), in_order.views.size());
  for (std::size_t v = 0; v < in_order.views.size(); ++v) {
    EXPECT_EQ(shuffled.views[v].frame_id, in_order.views[v].frame_id);
    EXPECT_EQ(shuffled.views[v].use, in_order.views[v].use);
  }
}

TEST(SolveLens, StartsFromARealLensToo) {
  const test::FisheyeTruth lens = truth();
  const std::vector<Observation>& views = noisy_views();
  const LensFit& own = noisy_fit();
  // Its own answer as the start: it cannot do worse.
  const LensFit started =
      value_of(solve_lens("bm-1", kSize, board(), views, own.camera.model));
  EXPECT_LE(started.rms_px, own.rms_px + 1e-9);
  EXPECT_LT(curve_error_px(started.camera.model, lens, 56.0), 1.0);
}

// One iteration: OpenCV's own start is nowhere yet, and a right lens stays
// right. So the start is tried, it wins, and the fit says so, through the
// second pass that a dropped corner brings on too. On every other view of the
// session: a fit solves for every view's pose with the lens at once, at a cost
// that grows steeply with the views, which every fit here pays, one iteration
// or 500.
TEST(SolveLens, KeepsAGivenStartThatFitsBetterAndSaysSo) {
  std::vector<Observation> views = half_session();
  LensOptions options;
  options.min_views = 10;
  const LensFit own = value_of(
      solve_lens("bm-1", kSize, board(), views, std::nullopt, options));
  options.max_iterations = 1;
  options.outlier_rounds = 0;
  const LensFit unaided = value_of(
      solve_lens("bm-1", kSize, board(), views, std::nullopt, options));
  EXPECT_FALSE(unaided.from_start);
  EXPECT_GT(unaided.rms_px, 1.0);
  const LensFit started = value_of(
      solve_lens("bm-1", kSize, board(), views, own.camera.model, options));
  EXPECT_TRUE(started.from_start);
  EXPECT_LT(started.rms_px, 0.3);

  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  for (std::size_t v = 0; v < plan.size(); ++v) {
    if (plan[v].use != Use::Fitted) continue;
    views[v].points[2].pixel.x += 25.0;
    break;
  }
  options.outlier_rounds = 3;
  const LensFit second = value_of(
      solve_lens("bm-1", kSize, board(), views, own.camera.model, options));
  EXPECT_TRUE(second.from_start);
  EXPECT_TRUE(std::any_of(second.residuals.begin(), second.residuals.end(),
                          [](const CornerResidual& r) { return r.dropped; }));
}

// With no start given, no fit is named as if from one, a second pass or not.
// Every other view, as above.
TEST(SolveLens, SaysNoStartWonWhenNoneWasGiven) {
  std::vector<Observation> views = half_session();
  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  for (std::size_t v = 0; v < plan.size(); ++v) {
    if (plan[v].use != Use::Fitted) continue;
    views[v].points[2].pixel.x += 25.0;
    break;
  }
  LensOptions options;
  options.min_views = 10;
  const LensFit fit = value_of(
      solve_lens("bm-1", kSize, board(), views, std::nullopt, options));
  EXPECT_FALSE(fit.from_start);
  EXPECT_TRUE(std::any_of(fit.residuals.begin(), fit.residuals.end(),
                          [](const CornerResidual& r) { return r.dropped; }));
}

// OpenCV's own start can land far off at first, and what that fit condemns was
// never bad. Nothing here is, so nothing stays dropped. OpenCV's path differs
// in its last bits between machines, so the seed may not land wrong elsewhere.
TEST(SolveLens, AWrongFirstFitDropsNothingForGood) {
  test::FisheyeCapture capture;
  capture.seed = 19;
  const LensFit fit = value_of(solve_lens(
      "bm-1", kSize, board(), test::fisheye_views(board(), truth(), capture)));
  EXPECT_EQ(count(fit.views, Use::Dropped), 0);
  for (const CornerResidual& r : fit.residuals) ASSERT_FALSE(r.dropped);
}

// Each held-out view's RMS is that of its better pose through the fitted lens:
// IPPE's two, refined on the pixels, here as the solve should have them.
TEST(SolveLens, PosesEachHeldOutViewThroughTheLens) {
  int checked = 0;
  for (const std::uint32_t seed : {1u, 2u}) {
    const std::vector<Observation>& views = noisy_views(seed);
    const LensFit& fit = noisy_fit(seed);
    const compute::CvCamera cam =
        value_of(compute::cv_camera(fit.camera.model));
    for (const LensView& use : fit.views) {
      if (use.use != Use::HeldOut) continue;
      const auto view = std::find_if(
          views.begin(), views.end(),
          [&](const Observation& v) { return v.frame_id == use.frame_id; });
      ASSERT_NE(view, views.end());
      std::vector<cv::Point3d> object;
      std::vector<cv::Point2d> pixels;
      for (const DetectedPoint& point : view->points) {
        object.push_back(
            board().object_points()[static_cast<std::size_t>(point.id)]);
        pixels.emplace_back(point.pixel.x, point.pixel.y);
      }
      const ViewPose poses = value_of(view_pose(cam, board(), *view));
      std::vector<PoseCandidate> candidates{poses.best};
      if (poses.alternative) candidates.push_back(*poses.alternative);
      double least = std::numeric_limits<double>::infinity();
      for (const PoseCandidate& candidate : candidates) {
        compute::CvPose pose =
            value_of(compute::cv_pose(candidate.board_to_camera));
        cv::solvePnPRefineLM(object, pixels, cam.camera_matrix, cam.distortion,
                             pose.rvec, pose.tvec);
        std::vector<cv::Point2d> projected;
        cv::projectPoints(object, pose.rvec, pose.tvec, cam.camera_matrix,
                          cam.distortion, projected);
        double sum = 0.0;
        for (std::size_t i = 0; i < pixels.size(); ++i) {
          const cv::Point2d e = projected[i] - pixels[i];
          sum += e.dot(e);
        }
        least = std::min(least,
                         std::sqrt(sum / static_cast<double>(pixels.size())));
      }
      EXPECT_NEAR(use.rms_px, least, 1e-9) << seed << ' ' << use.frame_id;
      ++checked;
    }
  }
  EXPECT_GE(checked, 15);
}

// A bad corner in a held-out view goes as one in a fitted view does, and its
// pull on the pose with it: the view scores as if it had never been seen.
TEST(SolveLens, DropsABadHeldOutCornerAndPosesTheViewWithoutIt) {
  const test::FisheyeTruth lens = truth();
  test::FisheyeCapture capture;
  capture.noise_px = 0.15;
  std::vector<Observation> views = test::fisheye_views(board(), lens, capture);
  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  const auto held =
      std::find_if(plan.begin(), plan.end(),
                   [](const LensView& v) { return v.use == Use::HeldOut; });
  ASSERT_NE(held, plan.end());
  const std::size_t v = static_cast<std::size_t>(held - plan.begin());
  const std::size_t i = views[v].points.size() / 2;
  const int bad = views[v].points[i].id;

  std::vector<Observation> without = views;
  without[v].points.erase(without[v].points.begin() +
                          static_cast<std::ptrdiff_t>(i));
  const LensFit clean = value_of(solve_lens("bm-1", kSize, board(), without));

  views[v].points[i].pixel.x += 25.0;
  const LensFit fit = value_of(solve_lens("bm-1", kSize, board(), views));
  EXPECT_EQ(fit.views[v].use, Use::HeldOut);
  EXPECT_EQ(fit.views[v].corners, clean.views[v].corners);
  EXPECT_NEAR(fit.views[v].rms_px, clean.views[v].rms_px, 1e-6);
  EXPECT_NEAR(fit.holdout_rms_px, clean.holdout_rms_px, 1e-6);
  int named = 0;
  for (const CornerResidual& r : fit.residuals) {
    if (!r.dropped) continue;
    EXPECT_EQ(r.capture, views[v].frame_id);
    EXPECT_EQ(r.corner, bad);
    ++named;
  }
  EXPECT_EQ(named, 1);
}

// A held-out view with too few corners left has no pose, and says so: it can
// vouch for nothing.
TEST(SolveLens, AHeldOutViewLeftWithoutAPoseReadsInfinite) {
  const test::FisheyeTruth lens = truth();
  test::FisheyeCapture capture;
  capture.noise_px = 0.15;
  std::vector<Observation> views = test::fisheye_views(board(), lens, capture);
  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  const auto held =
      std::find_if(plan.begin(), plan.end(),
                   [](const LensView& v) { return v.use == Use::HeldOut; });
  ASSERT_NE(held, plan.end());
  const std::size_t v = static_cast<std::size_t>(held - plan.begin());
  for (std::size_t i = 0; i < views[v].points.size(); ++i) {
    views[v].points[i].pixel.y += i % 2 == 0 ? 20.0 : -20.0;
  }
  const LensFit fit = value_of(solve_lens("bm-1", kSize, board(), views));
  EXPECT_TRUE(std::isinf(fit.views[v].rms_px));
  EXPECT_TRUE(std::isinf(fit.holdout_rms_px));
}

// Three corners 25 px off and a view out as a whole: named, the lens no worse
// for them. A view shifted whole is not the case here: its pose takes that up,
// and it stays (DECISIONS.md).
TEST(SolveLens, DropsAndNamesBadCornersAndAShiftedView) {
  const test::FisheyeTruth lens = truth();
  test::FisheyeCapture capture;
  capture.noise_px = 0.15;
  std::vector<Observation> views = test::fisheye_views(board(), lens, capture);
  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  // Two fitted views well out from the axis: one loses three corners, the
  // other has its corners moved 6 px up and down in turn, which no pose takes
  // up.
  std::vector<std::size_t> fitted;
  for (std::size_t v = 25; v < plan.size(); ++v) {
    if (plan[v].use == Use::Fitted) fitted.push_back(v);
  }
  ASSERT_GE(fitted.size(), 2u);
  Observation& with_bad_corners = views[fitted[0]];
  ASSERT_GT(with_bad_corners.points.size(), 20u);
  const std::set<int> bad = {with_bad_corners.points[2].id,
                             with_bad_corners.points[9].id,
                             with_bad_corners.points[15].id};
  for (const std::size_t i : {2u, 9u, 15u}) {
    with_bad_corners.points[i].pixel.x += 25.0;
  }
  Observation& shifted = views[fitted[1]];
  for (std::size_t i = 0; i < shifted.points.size(); ++i) {
    shifted.points[i].pixel.y += i % 2 == 0 ? 6.0 : -6.0;
  }

  const LensFit fit = value_of(solve_lens("bm-1", kSize, board(), views));
  const std::map<std::int64_t, Use> uses = uses_by_capture(fit.views);
  EXPECT_EQ(uses.at(shifted.frame_id), Use::Dropped);
  EXPECT_EQ(uses.at(with_bad_corners.frame_id), Use::Fitted);
  EXPECT_EQ(count(fit.views, Use::Dropped), 1);
  std::set<int> named;
  for (const CornerResidual& r : fit.residuals) {
    EXPECT_NE(r.capture, shifted.frame_id);  // no pose in the final fit
    if (r.dropped) {
      EXPECT_EQ(r.capture, with_bad_corners.frame_id);
      named.insert(r.corner);
    }
  }
  // The fit has no robust loss, so the bad corners' pull may take a few
  // neighbours past the bound with them.
  for (const int corner : bad) EXPECT_EQ(named.count(corner), 1u) << corner;
  EXPECT_LE(named.size(), 10u);
  EXPECT_NEAR(fit.rms_px, 0.15 * std::sqrt(2.0), 0.02);
  EXPECT_LT(curve_error_px(fit.camera.model, lens, 56.0), 1.0);
}

// A view off by less than the corner bound in every corner goes by its RMS,
// its corners all kept until it does.
TEST(SolveLens, DropsAViewByItsRmsWhenNoCornerIsPastTheBound) {
  const test::FisheyeTruth lens = truth();
  test::FisheyeCapture capture;
  capture.noise_px = 0.15;
  std::vector<Observation> views = test::fisheye_views(board(), lens, capture);
  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  std::size_t off = plan.size();
  for (std::size_t v = 25; v < plan.size(); ++v) {
    if (plan[v].use == Use::Fitted) off = v;
  }
  ASSERT_LT(off, plan.size());
  for (std::size_t i = 0; i < views[off].points.size(); ++i) {
    views[off].points[i].pixel.y += i % 2 == 0 ? 2.0 : -2.0;
  }
  const LensFit fit = value_of(solve_lens("bm-1", kSize, board(), views));
  EXPECT_EQ(fit.views[off].use, Use::Dropped);
  EXPECT_NEAR(fit.views[off].rms_px, 2.0, 0.2);
  EXPECT_EQ(count(fit.views, Use::Dropped), 1);
  for (const CornerResidual& r : fit.residuals) EXPECT_FALSE(r.dropped);
}

TEST(SolveLens, RefusesWithTooFewViews) {
  const test::FisheyeTruth lens = truth();
  std::vector<Observation> views = test::fisheye_views(board(), lens);
  views.resize(10);
  expect_refused(solve_lens("bm-1", kSize, board(), views).status(),
                 "solve_lens", "at least 20 needed");
  expect_refused(solve_lens("bm-3", kSize, board(), views).status(),
                 "solve_lens", "camera bm-3 has 0 views to fit");
}

// Rounds that leave too few views are as refused as too few at first.
TEST(SolveLens, RefusesWhenTheOutlierRoundsLeaveTooFew) {
  const test::FisheyeTruth lens = truth();
  test::FisheyeCapture capture;
  capture.noise_px = 0.15;
  std::vector<Observation> views = test::fisheye_views(board(), lens, capture);
  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  int spoiled = 0;
  for (std::size_t v = 0; v < plan.size() && spoiled < 3; ++v) {
    if (plan[v].use != Use::Fitted) continue;
    for (std::size_t i = 0; i < views[v].points.size(); ++i) {
      views[v].points[i].pixel.y += i % 2 == 0 ? 12.0 : -12.0;
    }
    ++spoiled;
  }
  LensOptions options;
  options.min_views = count(plan, Use::Fitted) - 1;
  expect_refused(
      solve_lens("bm-1", kSize, board(), views, std::nullopt, options).status(),
      "solve_lens", "views to fit");
}

TEST(SolveLens, RefusesAStartItCannotUse) {
  const test::FisheyeTruth lens = truth();
  const std::vector<Observation> views = test::fisheye_views(board(), lens);
  camera::CameraModel start{{1920, 1080}, {850.0, 850.0, 960.0, 540.0}, {}};
  expect_refused(solve_lens("bm-1", kSize, board(), views, start).status(),
                 "solve_lens", "the start is a lens at 1920 x 1080");
  start = {kSize, {0.0, 850.0, 960.0, 540.0}, {}};
  expect_refused(solve_lens("bm-1", kSize, board(), views, start).status(),
                 "solve_lens", "start");
}

// Outlier rounds off: the same views, every corner kept.
TEST(SolveLens, KeepsEveryCornerWithNoOutlierRounds) {
  const test::FisheyeTruth lens = truth();
  test::FisheyeCapture capture;
  capture.noise_px = 0.15;
  std::vector<Observation> views = test::fisheye_views(board(), lens, capture);
  const std::vector<LensView> plan =
      value_of(plan_lens_views("bm-1", kSize, board(), views));
  for (std::size_t v = 0; v < plan.size(); ++v) {
    if (v == 3 || plan[v].use == Use::HeldOut) {
      views[v].points[2].pixel.x += 25.0;  // fitted and held out alike
    }
  }
  LensOptions options;
  options.outlier_rounds = 0;
  const LensFit fit = value_of(
      solve_lens("bm-1", kSize, board(), views, std::nullopt, options));
  EXPECT_EQ(count(fit.views, Use::Dropped), 0);
  for (const CornerResidual& r : fit.residuals) EXPECT_FALSE(r.dropped);
}

}  // namespace
}  // namespace volumetric_kit::calib::solve
