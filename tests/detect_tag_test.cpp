// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The origin tag in a luma image: tags warped into a 4K image at 100-160 px
// across against where their homographies put the corners, in OpenCV's order
// as the print turns, beside the board, rows stride apart and on several
// threads; no tag, another id, two prints and the board's own markers; the
// tags and images it refuses, and what OpenCV throws.

#include "volumetric_kit/calib/detect/tag.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "detect_helpers.hpp"
#include "synthetic_rig.hpp"

namespace volumetric_kit::calib::detect {
namespace {

using Code = core::Status::Code;
using Corners = std::array<cv::Point2d, 4>;

using test::luma;
using test::Refused;
using test::ThrowingParallelFor;

constexpr double kPi = 3.14159265358979323846;
constexpr float kBackground = 110.0f;

// The print `side_px` across, centred at `centre`: squashed to `squash` of
// that along its y axis, turned `roll_deg` (clockwise on screen, y being
// down), and seen in `perspective` (per pixel from the centre).
cv::Matx33d placed(const cv::Point2d& centre, double side_px, double roll_deg,
                   double squash = 1.0, const cv::Vec2d& perspective = {}) {
  const double c = std::cos(roll_deg * kPi / 180.0);
  const double s = std::sin(roll_deg * kPi / 180.0);
  const cv::Matx33d middle(1.0, 0.0, -0.5, 0.0, 1.0, -0.5, 0.0, 0.0, 1.0);
  const cv::Matx33d size(side_px, 0.0, 0.0, 0.0, squash * side_px, 0.0, 0.0,
                         0.0, 1.0);
  const cv::Matx33d turn(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);
  const cv::Matx33d view(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, perspective[0],
                         perspective[1], 1.0);
  const cv::Matx33d move(1.0, 0.0, centre.x, 0.0, 1.0, centre.y, 0.0, 0.0, 1.0);
  return move * view * turn * size * middle;
}

cv::Mat background() {
  return cv::Mat(2160, 3840, CV_8UC1, cv::Scalar(kBackground));
}

// A camera's picture of `render`: blurred 0.8 px and noisy (2.5 levels).
cv::Mat picture(const cv::Mat& render, std::uint64_t seed) {
  test::CameraOptions optics;
  optics.seed = seed;
  return test::camera_image(render, optics);
}

// One print of `tag` placed in the 4K image, as a camera sees it.
struct View {
  cv::Mat image;
  Corners truth;
};

View tag_view(const cv::Matx33d& print_to_image, std::uint64_t seed,
              const OriginTag& tag = kOriginTag) {
  cv::Mat render = background();
  View view;
  view.truth =
      test::draw_marker(render, tag.dictionary, tag.id, print_to_image);
  view.image = picture(render, seed);
  return view;
}

MarkerSearch search(const cv::Mat& image, const OriginTag& tag = kOriginTag) {
  core::Result<MarkerSearch> found = core::Status::invalid_argument("not run");
  EXPECT_NO_THROW(found = detect_tag(luma(image), tag));
  EXPECT_TRUE(found.ok()) << found.status().message();
  return found.ok() ? *std::move(found) : MarkerSearch{};
}

// Each corner's offset from where it should be, split along the corner's
// bisector (outward positive) and across it, and the corners' mean against
// theirs. A corner in the wrong place in the order would be a side's length
// away.
struct Errors {
  double max_across = 0.0;
  double min_along = std::numeric_limits<double>::infinity();
  double max_along = -std::numeric_limits<double>::infinity();
  double centre = 0.0;

  void add(const DetectedMarker& found, const Corners& truth) {
    cv::Point2d middle(0.0, 0.0);
    cv::Point2d mean(0.0, 0.0);
    for (std::size_t k = 0; k < truth.size(); ++k) {
      middle += 0.25 * truth[k];
      mean += 0.25 * cv::Point2d(found.corners[k].x, found.corners[k].y);
    }
    centre = std::max(centre, cv::norm(mean - middle));
    const auto unit = [](const cv::Point2d& v) { return v / cv::norm(v); };
    for (std::size_t k = 0; k < truth.size(); ++k) {
      const cv::Point2d out = -unit(unit(truth[(k + 3) % 4] - truth[k]) +
                                    unit(truth[(k + 1) % 4] - truth[k]));
      const cv::Point2d e(found.corners[k].x - truth[k].x,
                          found.corners[k].y - truth[k].y);
      const double along = e.dot(out);
      min_along = std::min(min_along, along);
      max_along = std::max(max_along, along);
      max_across = std::max(max_across, std::abs(e.x * out.y - e.y * out.x));
    }
  }
};

std::ostream& operator<<(std::ostream& out, const Errors& e) {
  return out << "across <= " << e.max_across << " px, along " << e.min_along
             << " to " << e.max_along << " px, centre " << e.centre << " px";
}

// SUBPIX's corners sit within 0.15 px of the truth across the line from the
// tag's centre, but its cornerSubPix pulls a black square's outer corner in
// along that line, 0.1-0.3 px at the camera's blur (DECISIONS.md, "The origin
// tag"): the centre holds, and the edge reads short.
::testing::AssertionResult Accurate(const Errors& e) {
  if (e.max_across <= 0.15 && e.min_along >= -0.35 && e.max_along <= 0.05 &&
      e.centre <= 0.1) {
    return ::testing::AssertionSuccess() << e;
  }
  return ::testing::AssertionFailure() << e;
}

// Prints 100-160 px across, turned, squashed and in perspective, as a floor
// tag looks to the rig's cameras 2-3 m away.
std::vector<cv::Matx33d> placements() {
  return {placed({612.3, 433.7}, 100.0, 0.0),
          placed({1920.4, 1080.6}, 160.0, 37.0, 0.7, {1e-3, -5e-4}),
          placed({3301.8, 1702.2}, 128.0, -118.0, 0.8, {-8e-4, 1e-3}),
          placed({2650.5, 512.9}, 140.0, 90.0, 0.9, {5e-4, 5e-4}),
          placed({1000.2, 1800.8}, 115.0, 195.0, 0.75, {-1e-3, 0.0}),
          placed({3500.6, 300.1}, 150.0, -61.0, 0.65, {0.0, -1e-3})};
}

TEST(DetectTag, FindsAWarpedTagInOpenCVsCornerOrder) {
  Errors errors;
  std::uint64_t seed = 1;
  for (const cv::Matx33d& placement : placements()) {
    const View view = tag_view(placement, seed++);
    const MarkerSearch found = search(view.image);
    EXPECT_EQ(found.matches, 1) << seed;
    ASSERT_TRUE(found.found.has_value()) << seed;
    EXPECT_EQ(found.found->id, 0);
    Errors one;
    one.add(*found.found, view.truth);
    EXPECT_TRUE(Accurate(one)) << "placement " << seed - 1;
    errors.add(*found.found, view.truth);
  }
  EXPECT_TRUE(Accurate(errors));
}

// The same print turned about its centre a quarter at a time: its border
// covers the same pixels, so its corners are found in the same four places,
// but the first is always the printed top-left: each quarter turn clockwise
// puts it where the top-right was.
TEST(DetectTag, FollowsThePrintedTopLeftAsThePrintTurns) {
  std::vector<DetectedMarker> turned;
  for (int quarter = 0; quarter < 4; ++quarter) {
    const View view =
        tag_view(placed({1500.4, 900.7}, 130.0, 12.0 + 90.0 * quarter), 20);
    const MarkerSearch found = search(view.image);
    ASSERT_TRUE(found.found.has_value()) << quarter;
    Errors errors;
    errors.add(*found.found, view.truth);
    EXPECT_TRUE(Accurate(errors)) << quarter;
    turned.push_back(*found.found);
  }
  for (std::size_t quarter = 1; quarter < turned.size(); ++quarter) {
    for (std::size_t k = 0; k < 4; ++k) {
      const recon::camera::Vec2d a = turned[quarter].corners[k];
      const recon::camera::Vec2d b = turned[quarter - 1].corners[(k + 1) % 4];
      EXPECT_LT(std::hypot(a.x - b.x, a.y - b.y), 0.5)
          << "quarter " << quarter << ", corner " << k;
    }
  }
}

// A capture's picture holds the board too: its markers are another
// dictionary's, and the tag is found as alone.
TEST(DetectTag, FindsTheTagBesideTheBoard) {
  const compute::Board board = test::femto_board();
  cv::Mat render = test::render_board(
      test::color_like_lens(),
      test::board_facing_camera(board, {-0.5, 0.0, 2.5}, 20.0, 5.0), board);
  const Corners truth = test::draw_marker(
      render, kOriginTag.dictionary, 0,
      placed({2900.3, 1400.6}, 120.0, -20.0, 0.7, {5e-4, -5e-4}));
  const MarkerSearch found = search(picture(render, 30));
  EXPECT_EQ(found.matches, 1);
  ASSERT_TRUE(found.found.has_value());
  Errors errors;
  errors.add(*found.found, truth);
  EXPECT_TRUE(Accurate(errors));
}

TEST(DetectTag, FindsNothingWithoutATag) {
  const MarkerSearch found = search(picture(background(), 40));
  EXPECT_FALSE(found.found.has_value());
  EXPECT_EQ(found.matches, 0);
}

// Another id of the tag's dictionary is not the tag, though a tag of that id
// would find it.
TEST(DetectTag, IgnoresAnotherId) {
  OriginTag one = kOriginTag;
  one.id = 1;
  const View view = tag_view(placements()[1], 50, one);
  const MarkerSearch origin = search(view.image);
  EXPECT_FALSE(origin.found.has_value());
  EXPECT_EQ(origin.matches, 0);
  const MarkerSearch other = search(view.image, one);
  EXPECT_EQ(other.matches, 1);
  ASSERT_TRUE(other.found.has_value());
  EXPECT_EQ(other.found->id, 1);
}

// Two prints of the tag give neither, so a second print is never taken for
// the first; beside a tag of another id, one print is found.
TEST(DetectTag, TakesNeitherOfTwoPrints) {
  cv::Mat render = background();
  test::draw_marker(render, kOriginTag.dictionary, 0, placements()[0]);
  test::draw_marker(render, kOriginTag.dictionary, 0, placements()[2]);
  const MarkerSearch two = search(picture(render, 60));
  EXPECT_FALSE(two.found.has_value());
  EXPECT_EQ(two.matches, 2);

  render = background();
  const Corners truth =
      test::draw_marker(render, kOriginTag.dictionary, 0, placements()[0]);
  test::draw_marker(render, kOriginTag.dictionary, 1, placements()[2]);
  const MarkerSearch one = search(picture(render, 61));
  EXPECT_EQ(one.matches, 1);
  ASSERT_TRUE(one.found.has_value());
  Errors errors;
  errors.add(*one.found, truth);
  EXPECT_TRUE(Accurate(errors));
}

// The rig's board alone: its DICT_4X4_100 markers are not the AprilTag,
// though a tag of that dictionary finds its marker 0, the dictionary taken
// by name.
TEST(DetectTag, FindsNothingInTheBoardsImage) {
  const compute::Board board = test::femto_board();
  const cv::Mat image = picture(
      test::render_board(
          test::color_like_lens(),
          test::board_facing_camera(board, {0.031, -0.017, 2.5}, 0.0, 7.0),
          board),
      70);
  const MarkerSearch origin = search(image);
  EXPECT_FALSE(origin.found.has_value());
  EXPECT_EQ(origin.matches, 0);
  const MarkerSearch marker =
      search(image, {MarkerDictionary::ArUco4x4_100, 0, 0.112});
  EXPECT_EQ(marker.matches, 1);
  EXPECT_TRUE(marker.found.has_value());
}

// Rows `stride` bytes apart: the padding is never read.
TEST(DetectTag, ReadsRowsStrideApart) {
  const View view = tag_view(placements()[1], 80);
  cv::Mat buffer(view.image.rows, view.image.cols + 64, CV_8UC1, cv::Scalar(0));
  view.image.copyTo(buffer(cv::Rect(0, 0, view.image.cols, view.image.rows)));
  const cv::Mat padded =
      buffer(cv::Rect(0, 0, view.image.cols, view.image.rows));
  ASSERT_EQ(padded.step[0], static_cast<std::size_t>(view.image.cols + 64));
  const MarkerSearch packed = search(view.image);
  const MarkerSearch strided = search(padded);
  ASSERT_TRUE(packed.found.has_value());
  ASSERT_TRUE(strided.found.has_value());
  EXPECT_EQ(strided.found->corners, packed.found->corners);
}

// The capture worker detects every camera's image at once: each of four
// threads searches its view three times, and every result is the one a
// single thread gets.
TEST(DetectTag, SearchesOnSeveralThreadsAtOnce) {
  constexpr int kRounds = 3;
  std::vector<View> views;
  std::vector<MarkerSearch> alone;
  for (std::size_t i = 0; i < 4; ++i) {
    views.push_back(tag_view(placements()[i], 90 + i));
    alone.push_back(search(views.back().image));
    ASSERT_TRUE(alone.back().found.has_value()) << i;
  }
  std::vector<int> differ(views.size(), 0);
  std::vector<std::thread> threads;
  for (std::size_t i = 0; i < views.size(); ++i) {
    threads.emplace_back([&, i] {
      for (int round = 0; round < kRounds; ++round) {
        const core::Result<MarkerSearch> found =
            detect_tag(luma(views[i].image), kOriginTag);
        if (!found.ok() || !found->found ||
            found->found->corners != alone[i].found->corners) {
          ++differ[i];
        }
      }
    });
  }
  for (std::thread& thread : threads) thread.join();
  for (std::size_t i = 0; i < views.size(); ++i) {
    EXPECT_EQ(differ[i], 0) << i;
  }
}

core::Status refusal(const LumaView& image, const OriginTag& tag = kOriginTag) {
  core::Result<MarkerSearch> found = MarkerSearch{};
  EXPECT_NO_THROW(found = detect_tag(image, tag));
  return found.ok() ? core::Status{} : found.status();
}

// Any image with a pixel is searched; one OpenCV cannot read is refused
// before anything reads its pixels.
TEST(DetectTag, RefusesAnImageItCannotRead) {
  const std::vector<std::uint8_t> pixels(64, 110);
  const LumaView good{8, 8, 8, pixels.data()};
  EXPECT_TRUE(refusal(good).ok());
  EXPECT_TRUE(refusal({1, 1, 1, pixels.data()}).ok());
  EXPECT_TRUE(refusal({64, 1, 64, pixels.data()}).ok());
  EXPECT_TRUE(refusal({1, 64, 1, pixels.data()}).ok());
  LumaView image = good;
  image.data = nullptr;
  EXPECT_TRUE(Refused(refusal(image), "detect_tag: image.data is null"));
  image = good;
  image.stride = 7;
  EXPECT_TRUE(
      Refused(refusal(image), "detect_tag: image.stride is below image.width"));
  for (const auto& [width, height] :
       {std::pair<std::uint32_t, std::uint32_t>{0, 8}, {8, 0}, {0, 0}}) {
    image = good;
    image.width = width;
    image.height = height;
    EXPECT_TRUE(Refused(refusal(image), "detect_tag: the image has no pixels"))
        << width << " x " << height;
  }
  image = good;
  image.width = std::uint32_t{1} << 31;
  image.stride = image.width;
  EXPECT_TRUE(
      Refused(refusal(image),
              "detect_tag: the image is larger than OpenCV's int sizes hold"));
  image = good;
  image.height = std::uint32_t{1} << 31;
  EXPECT_TRUE(
      Refused(refusal(image),
              "detect_tag: the image is larger than OpenCV's int sizes hold"));
}

// A tag check_origin_tag refuses is refused as it says, before the image is
// read.
TEST(DetectTag, RefusesATagCheckOriginTagRefuses) {
  OriginTag none = kOriginTag;
  none.dictionary = MarkerDictionary::None;
  OriginTag unknown = kOriginTag;
  unknown.dictionary = static_cast<MarkerDictionary>(99);
  OriginTag negative = kOriginTag;
  negative.id = -1;
  OriginTag past = kOriginTag;
  past.id = 587;
  OriginTag flat = kOriginTag;
  flat.edge_m = 0.0;
  for (const OriginTag& tag : {none, unknown, negative, past, flat}) {
    const core::Status refused = check_origin_tag(tag);
    ASSERT_FALSE(refused.ok());
    EXPECT_TRUE(Refused(refusal({0, 0, 0, nullptr}, tag), refused.message()));
  }
}

// Full of what the detector's loops would search; the loops throw instead.
TEST(DetectTag, ReturnsWhatOpenCVThrowsAsAStatus) {
  const View view = tag_view(placements()[0], 100);
  core::Result<MarkerSearch> found = MarkerSearch{};
  {
    const ThrowingParallelFor throwing(cv::Error::StsNotImplemented);
    EXPECT_NO_THROW(found = detect_tag(luma(view.image), kOriginTag));
  }
  ASSERT_FALSE(found.ok());
  EXPECT_EQ(found.status().domain(), Code::Backend);
  EXPECT_EQ(found.status().backend(), core::Status::Backend::Other);
  EXPECT_EQ(found.status().detail(), cv::Error::StsNotImplemented);
  EXPECT_EQ(found.status().message().rfind("detect_tag: ", 0), 0u)
      << found.status().message();
  EXPECT_NE(found.status().message().find("injected"), std::string::npos)
      << found.status().message();
  {
    const ThrowingParallelFor throwing(cv::Error::StsNoMem);
    EXPECT_NO_THROW(found = detect_tag(luma(view.image), kOriginTag));
  }
  ASSERT_FALSE(found.ok());
  EXPECT_EQ(found.status().domain(), Code::OutOfMemory);
  // OpenCV's own backend is back.
  EXPECT_TRUE(search(view.image).found.has_value());
}

}  // namespace
}  // namespace volumetric_kit::calib::detect
