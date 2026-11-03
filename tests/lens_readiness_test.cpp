// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Whether a camera's views are enough to fit its lens: ready flips exactly at
// each limit, on the views the solve will fit.

#include "volumetric_kit/calib/quality/lens_readiness.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "result_checks.hpp"
#include "synthetic_rig.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = sensor::camera;
using test::expect_refused;
using test::value_of;

constexpr camera::ImageSize kSize{3840, 2160};
constexpr double kA = 1920.0;  // the image's centre to a side
constexpr double kB = 1080.0;  // and to the top
constexpr double kCentreU = 1919.5;
constexpr double kCentreV = 1079.5;

const compute::Board& board() {
  static const compute::Board once = test::lab_checkerboard();
  return once;
}

// A view of 12 corners at `pixels`, on two rows of the board, so it fixes a
// pose wherever they are: a view's corners need only lie in the image here.
Observation view_of(std::int64_t frame,
                    const std::array<camera::Vec2d, 12>& pixels) {
  Observation view;
  view.camera_id = "bm-1";
  view.frame_id = frame;
  for (int i = 0; i < 12; ++i) {
    view.points.push_back(
        {i < 6 ? i : i + 7, pixels[static_cast<std::size_t>(i)], 1.0});
  }
  return view;
}

// Twelve corners near the image's centre, in two of the 8 x 6 cells' worth.
std::array<camera::Vec2d, 12> central() {
  std::array<camera::Vec2d, 12> pixels;
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    pixels[i] = {1800.0 + 20.0 * static_cast<double>(i % 6),
                 1000.0 + 20.0 * static_cast<double>(i / 6)};
  }
  return pixels;
}

// The middle of a cell of the 8 x 6 grid.
camera::Vec2d middle_of(int cell) {
  return {(cell % 8 + 0.5) * 480.0 - 0.5, (cell / 8 + 0.5) * 360.0 - 0.5};
}

// A view with a corner in each of 12 cells from `first`, the last repeated
// past `end`.
Observation covering(std::int64_t frame, int first, int end) {
  std::array<camera::Vec2d, 12> pixels;
  for (int i = 0; i < 12; ++i) {
    pixels[static_cast<std::size_t>(i)] =
        middle_of(std::min(first + i, end - 1));
  }
  return view_of(frame, pixels);
}

// A pixel `fraction` of the way toward image corner or side `direction`, in
// LensReach's order, and short of the rest: toward a corner it hugs the top or
// bottom edge, so a side stays under 0.9.
camera::Vec2d toward(std::size_t direction, double fraction) {
  if (direction >= 4) {
    return {kCentreU + (direction == 4 ? -1.0 : 1.0) * fraction * kA, kCentreV};
  }
  const double y = 1070.0;
  const double x = (fraction * (kA * kA + kB * kB) - kB * y) / kA;
  return {kCentreU + (direction % 2 == 0 ? -x : x),
          kCentreV + (direction < 2 ? -y : y)};
}

// A view whose first corner reaches `direction` and the rest stay central.
Observation reaching(std::int64_t frame, std::size_t direction,
                     double fraction) {
  std::array<camera::Vec2d, 12> pixels = central();
  pixels[0] = toward(direction, fraction);
  return view_of(frame, pixels);
}

// A session: views covering the first `cells` cells of the grid, six reaching
// a little past 0.9, one each way, and `more` central.
std::vector<Observation> session(int cells, int more) {
  std::vector<Observation> views;
  std::int64_t frame = 0;
  for (int first = 0; first < 48; first += 12) {
    views.push_back(covering(frame++, first, cells));
  }
  for (std::size_t d = 0; d < 6; ++d) {
    views.push_back(reaching(frame++, d, 0.91));
  }
  for (int i = 0; i < more; ++i) views.push_back(view_of(frame++, central()));
  return views;
}

// Each reach direction, as the shortfall names it.
constexpr const char* kToward[] = {"top-left corner",    "top-right corner",
                                   "bottom-left corner", "bottom-right corner",
                                   "left side",          "right side"};

// A reach rounded down to two places, as the shortfall writes it.
std::string two_down(double reach) {
  char text[16];
  std::snprintf(text, sizeof text, "%.2f", std::floor(100.0 * reach) / 100.0);
  return text;
}

LensReadiness state_of(const std::vector<Observation>& views,
                       const std::vector<std::int64_t>* frontal = nullptr,
                       const LensReadyOptions& options = {}) {
  return value_of(
      lens_readiness("bm-1", kSize, board(), views, frontal, options));
}

// No view held out, so each counts: the limits alone are in play. A run this
// long reaches its middle at no view.
LensReadyOptions keeping_every_view() {
  LensReadyOptions options;
  options.lens.holdout_every = 1 << 20;
  return options;
}

TEST(LensReadiness, FlipsAtThirtyViewsTheSolveWillFit) {
  bool short_of_it = false;
  bool at_it = false;
  for (int more = 10; more < 40; ++more) {
    const LensReadiness state = state_of(session(48, more));
    short_of_it = short_of_it || state.views == 29;
    at_it = at_it || state.views == 30;
    EXPECT_EQ(state.ready, state.views >= 30) << state.views;
    EXPECT_EQ(state.unmet, state.ready
                               ? std::string()
                               : std::to_string(state.views) + " views of 30");
  }
  EXPECT_TRUE(short_of_it);
  EXPECT_TRUE(at_it);
}

// Views of fewer corners than the solve takes count for nothing, and those it
// holds out neither.
TEST(LensReadiness, CountsOnlyViewsTheSolveWillFit) {
  std::vector<Observation> views = session(48, 30);
  const LensReadiness before = state_of(views);
  EXPECT_LT(before.views, static_cast<int>(views.size()));
  Observation few = view_of(300, central());
  few.points.resize(11);
  few.points[0].pixel = {2.0, 2.0};
  views.push_back(few);
  const LensReadiness after = state_of(views);
  EXPECT_EQ(after.views, before.views);
  EXPECT_EQ(after.cells, before.cells);
  EXPECT_EQ(after.reach, before.reach);
}

// The reaching views add the two bottom corners' cells to the first 34: 36 of
// 48 is exactly three quarters.
TEST(LensReadiness, FlipsAtThreeQuartersOfTheCells) {
  const LensReadyOptions options = keeping_every_view();
  LensReadiness state = state_of(session(34, 30), nullptr, options);
  EXPECT_EQ(state.cells, 0.75);
  EXPECT_TRUE(state.ready);
  EXPECT_EQ(state.unmet, "");
  state = state_of(session(33, 30), nullptr, options);
  EXPECT_EQ(state.cells, 35.0 / 48.0);
  EXPECT_FALSE(state.ready);
  EXPECT_EQ(state.unmet, "73% of the cells, of 75%");
}

// Each direction alone, just short of 0.9 and just past it.
TEST(LensReadiness, FlipsAtEachCornersAndSidesReach) {
  for (std::size_t d = 0; d < 6; ++d) {
    std::vector<Observation> views = session(48, 30);
    Observation& view = views[4 + d];
    view = reaching(view.frame_id, d, 0.9 - 1e-6);
    LensReadiness state = state_of(views);
    EXPECT_FALSE(state.ready) << d;
    EXPECT_LT(state.reach[d], 0.9) << d;
    // Named, its reach rounded down so it never reads as the limit.
    EXPECT_EQ(state.unmet, "reach " + two_down(state.reach[d]) +
                               " toward the " + kToward[d] +
                               ", needs 0.9: capture there")
        << d;
    view = reaching(view.frame_id, d, 0.9 + 1e-6);
    state = state_of(views);
    EXPECT_TRUE(state.ready) << d;
    EXPECT_NEAR(state.reach[d], 0.9 + 1e-6, 1e-9) << d;
  }
  // Every direction short is named, in LensReach's order.
  std::vector<Observation> views = session(48, 30);
  views[4] = reaching(views[4].frame_id, 0, 0.884);
  views[9] = reaching(views[9].frame_id, 5, 0.897);
  const LensReadiness two = state_of(views);
  ASSERT_LT(two.reach[0], 0.9);
  ASSERT_LT(two.reach[5], 0.9);
  EXPECT_EQ(two.unmet, "reach " + two_down(two.reach[0]) +
                           " toward the top-left corner, " +
                           two_down(two.reach[5]) +
                           " toward the right side, needs 0.9: capture there");
  EXPECT_EQ(two_down(0.8999999), "0.89");
}

TEST(LensReadiness, FlipsAtHalfFrontalAndOnlyWithARealLens) {
  const LensReadyOptions options = keeping_every_view();
  const std::vector<Observation> views = session(48, 30);
  ASSERT_EQ(views.size(), 40u);
  const LensReadiness any = state_of(views, nullptr, options);
  ASSERT_TRUE(any.ready);
  EXPECT_FALSE(any.frontal.has_value());  // a stand-in's tilt is not judged

  std::vector<std::int64_t> frontal;
  for (std::int64_t frame = 0; frame < 20; ++frame) frontal.push_back(frame);
  LensReadiness state = state_of(views, &frontal, options);
  EXPECT_EQ(*state.frontal, 0.5);
  EXPECT_TRUE(state.ready);
  frontal.push_back(20);
  state = state_of(views, &frontal, options);
  EXPECT_EQ(*state.frontal, 21.0 / 40.0);
  EXPECT_FALSE(state.ready);
  EXPECT_EQ(state.unmet, "53% of its views frontal, over 50%");
}

// A frontal view the solve will not fit does not count.
TEST(LensReadiness, CountsFrontalViewsAmongThoseFitted) {
  const std::vector<Observation> views = session(48, 30);
  std::vector<std::int64_t> held_out;
  for (const solve::LensView& view :
       value_of(solve::plan_lens_views("bm-1", kSize, board(), views))) {
    if (view.use == solve::LensView::Use::HeldOut) {
      held_out.push_back(view.frame_id);
    }
  }
  ASSERT_FALSE(held_out.empty());
  EXPECT_EQ(*state_of(views, &held_out).frontal, 0.0);
}

TEST(LensReadiness, Refuses) {
  const std::vector<Observation> views = {view_of(0, central())};
  const auto refused = [&](const LensReadyOptions& options) {
    return lens_readiness("bm-1", kSize, board(), views, nullptr, options)
        .status();
  };
  LensReadyOptions options;
  options.min_views = 0;
  expect_refused(refused(options), "lens_readiness", "options.min_views");
  options = {};
  options.min_cells = 1.5;
  expect_refused(refused(options), "lens_readiness", "options.min_cells");
  options = {};
  options.min_reach = -0.1;
  expect_refused(refused(options), "lens_readiness", "options.min_reach");
  options = {};
  options.max_frontal = 2.0;
  expect_refused(refused(options), "lens_readiness", "options.max_frontal");
  options = {};
  options.lens.min_corners = 1;
  expect_refused(refused(options), "lens_readiness", "options.min_corners");
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
