// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Whether a view fixes the board's pose: enough corners, each the board's
// once, finite and inside the image, and not on one line.

#include "volumetric_kit/calib/compute/usable_for_pose.hpp"

#include <limits>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace volumetric_kit::calib::compute {
namespace {

const cv::Size kImage(3840, 2160);

Board femto_board() { return Board::create(legacy_femto_board()).value(); }

// A view of the given corners, each at a distinct pixel well inside.
Observation view_of(const std::vector<int>& ids) {
  Observation view;
  for (const int id : ids) {
    view.points.push_back(
        {id, {1000.0 + 37.0 * id, 600.0 + 23.0 * id * id}, 1.0});
  }
  return view;
}

ViewUse use_of(const Observation& view, const cv::Size& image = kImage) {
  core::Result<ViewUse> use = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(use = usable_for_pose(femto_board(), view, image));
  EXPECT_TRUE(use.ok()) << use.status().message();
  return use.ok() ? *use : ViewUse::Usable;
}

TEST(UsableForPose, AWholeViewAndAPartialOneAreUsable) {
  EXPECT_EQ(use_of(view_of({0, 1, 2, 3, 4, 5, 6, 7})), ViewUse::Usable);
  EXPECT_EQ(use_of(view_of({0, 1, 5, 6})), ViewUse::Usable);
}

TEST(UsableForPose, NeedsFourCorners) {
  EXPECT_EQ(use_of(view_of({})), ViewUse::TooFewCorners);
  EXPECT_EQ(use_of(view_of({0, 1, 5})), ViewUse::TooFewCorners);
}

// Corners on one line: the board's rows (4 corners on the 4 x 2 grid).
TEST(UsableForPose, RefusesCollinearCorners) {
  EXPECT_EQ(use_of(view_of({0, 1, 2, 3})), ViewUse::Collinear);
  EXPECT_EQ(use_of(view_of({4, 5, 6, 7})), ViewUse::Collinear);
}

// All corners but one on a line fix no homography, wherever the one is in the
// view; two off the line do.
TEST(UsableForPose, RefusesAllButOneCornerOnALine) {
  for (const std::vector<int>& ids :
       std::vector<std::vector<int>>{{0, 1, 2, 5},
                                     {5, 0, 1, 2},
                                     {0, 1, 2, 3, 7},
                                     {4, 0, 1, 2, 3},
                                     {4, 5, 6, 7, 1},
                                     {0, 4, 5, 6, 7}}) {
    EXPECT_EQ(use_of(view_of(ids)), ViewUse::Collinear) << ids.front();
  }
  EXPECT_EQ(use_of(view_of({0, 1, 2, 5, 6})), ViewUse::Usable);
  EXPECT_EQ(use_of(view_of({0, 3, 4, 7})), ViewUse::Usable);
  EXPECT_EQ(use_of(view_of({0, 1, 2, 3, 4, 7})), ViewUse::Usable);
}

TEST(UsableForPose, RefusesACornerTheBoardLacksOrRepeats) {
  EXPECT_EQ(use_of(view_of({0, 1, 5, 8})), ViewUse::UnknownCorner);
  EXPECT_EQ(use_of(view_of({-1, 0, 1, 5})), ViewUse::UnknownCorner);
  EXPECT_EQ(use_of(view_of({0, 1, 5, 5})), ViewUse::UnknownCorner);
}

// The image's edge is at -0.5 and size - 0.5: a corner 2 px inside it is
// usable, one a hair closer is not, nor one not finite.
TEST(UsableForPose, KeepsCornersTwoPixelsInsideTheImage) {
  const std::vector<sensor::camera::Vec2d> inside = {
      {1.5, 1.5}, {3837.5, 1.5}, {3837.5, 2157.5}, {1.5, 2157.5}};
  Observation view = view_of({0, 3, 7, 4});
  for (std::size_t i = 0; i < inside.size(); ++i) {
    view.points[i].pixel = inside[i];
  }
  EXPECT_EQ(use_of(view), ViewUse::Usable);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<sensor::camera::Vec2d> outside = {
      {1.49, 1000.0},    {3837.51, 1000.0}, {1000.0, 1.49},
      {1000.0, 2157.51}, {nan, 1000.0},     {1000.0, inf}};
  for (const sensor::camera::Vec2d& pixel : outside) {
    Observation off = view_of({0, 1, 5, 6});
    off.points[2].pixel = pixel;
    EXPECT_EQ(use_of(off), ViewUse::OutsideImage) << pixel.x << ", " << pixel.y;
  }
}

// The first check that fails is the answer.
TEST(UsableForPose, AnswersTheFirstCheckThatFails) {
  Observation view = view_of({0, 1, 2, 9});
  view.points[0].pixel.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(use_of(view), ViewUse::UnknownCorner);
  view.points[3].id = 3;  // now collinear too
  EXPECT_EQ(use_of(view), ViewUse::OutsideImage);
}

// Each answer has its own phrase, and a value out of range one too.
TEST(UsableForPose, DescribesEachAnswer) {
  const std::vector<ViewUse> uses = {
      ViewUse::Usable,       ViewUse::TooFewCorners, ViewUse::UnknownCorner,
      ViewUse::OutsideImage, ViewUse::Collinear,     static_cast<ViewUse>(99)};
  std::set<std::string> phrases;
  for (const ViewUse use : uses) {
    const char* phrase = describe(use);
    ASSERT_NE(phrase, nullptr);
    EXPECT_NE(phrase[0], '\0');
    phrases.insert(phrase);
  }
  EXPECT_EQ(phrases.size(), uses.size());
  EXPECT_STREQ(describe(ViewUse::TooFewCorners), "fewer than 4 corners");
  EXPECT_STREQ(describe(static_cast<ViewUse>(99)), "an unknown use");
}

TEST(UsableForPose, NamesEachAnswer) {
  EXPECT_STREQ(to_string(ViewUse::Usable), "usable");
  EXPECT_STREQ(to_string(ViewUse::TooFewCorners), "too_few_corners");
  EXPECT_STREQ(to_string(ViewUse::UnknownCorner), "unknown_corner");
  EXPECT_STREQ(to_string(ViewUse::OutsideImage), "outside_image");
  EXPECT_STREQ(to_string(ViewUse::Collinear), "collinear");
  EXPECT_STREQ(to_string(static_cast<ViewUse>(99)), "unknown");
}

TEST(UsableForPose, RefusesAnImageOfNoArea) {
  for (const cv::Size& image : {cv::Size(0, 2160), cv::Size(3840, -1)}) {
    core::Result<ViewUse> use = ViewUse::Usable;
    EXPECT_NO_THROW(
        use = usable_for_pose(femto_board(), view_of({0, 1, 5, 6}), image));
    EXPECT_EQ(use.status().domain(), core::Status::Code::InvalidArgument);
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::compute
