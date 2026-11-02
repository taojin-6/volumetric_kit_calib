// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// One detector for both boards. The lab's checkerboard rendered at 4K through
// a fisheye truth (KB4, f 1702, k1 -0.033: the lab's Blackmagic lens) is
// found turned any way, cut by each edge of the picture and in each corner,
// in the picture and in its half, its grid numbered as a grid; corners SB
// adds off the board are dropped; a grid larger than the board is refused; a
// still board reads still, by position, between noise-only pictures. A
// ChArUco board is found as detect_charuco finds it. DetectBoardTiming prints
// findChessboardCornersSB's times on 20 pictures (`detect_board.timing`,
// labelled slow).

#include "volumetric_kit/calib/detect/board.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/objdetect.hpp>

#include "detect_helpers.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/detect/charuco.hpp"
#include "volumetric_kit/calib/quality/capture_rule.hpp"
#include "volumetric_kit/calib/quality/live_checks.hpp"

namespace volumetric_kit::calib::detect {
namespace {

namespace camera = sensor::camera;
using Code = core::Status::Code;
using test::luma;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

// The lab's Blackmagic lens at 4K.
const test::FisheyeTruth kLens;

// On several threads, SB's grid can differ from run to run on one picture
// (8 x 4 or 7 x 4 corners); on one, each picture gives one answer.
class OneOpenCvThread : public ::testing::Environment {
 public:
  void SetUp() override { cv::setNumThreads(1); }
};
::testing::Environment* const kOneThread =
    ::testing::AddGlobalTestEnvironment(new OneOpenCvThread);

// The board's centre `off_axis_deg` off the optical axis, toward
// `azimuth_deg` (0 the picture's right, 90 its bottom), `distance_m` away,
// its printed face turned to the camera, then rolled `roll_deg` about the
// line of sight, then tilted `tilt_deg` about its own x axis.
camera::Mat4d facing(const compute::Board& board, double off_axis_deg,
                     double azimuth_deg, double distance_m, double roll_deg,
                     double tilt_deg) {
  const double t = off_axis_deg * kDeg;
  const double a = azimuth_deg * kDeg;
  const camera::Vec3d centre =
      distance_m * camera::Vec3d(std::sin(t) * std::cos(a),
                                 std::sin(t) * std::sin(a), std::cos(t));
  const camera::Vec3d z = glm::normalize(centre);
  const camera::Vec3d x =
      glm::normalize(glm::cross(camera::Vec3d(0.0, 1.0, 0.0), z));
  const camera::Vec3d y = glm::cross(z, x);
  const camera::Mat3d turn =
      camera::Mat3d(x, y, z) *
      camera::rotation_from_rodrigues({0.0, 0.0, roll_deg * kDeg}) *
      camera::rotation_from_rodrigues({tilt_deg * kDeg, 0.0, 0.0});
  camera::Mat4d pose(turn);
  const cv::Point3d middle = board.centre();
  pose[3] = glm::dvec4(
      centre - turn * camera::Vec3d(middle.x, middle.y, middle.z), 1.0);
  return pose;
}

// The board's corners where the lens images them, by id.
std::vector<cv::Point2d> truth(const compute::Board& board,
                               const camera::Mat4d& pose) {
  std::vector<cv::Point2d> pixels;
  for (const cv::Point3d& p : board.object_points()) {
    const camera::Vec3d q(pose * glm::dvec4(p.x, p.y, p.z, 1.0));
    pixels.push_back(kLens.project({q.x, q.y, q.z}));
  }
  return pixels;
}

// Whether a detection is of a picture of the lens's size.
bool lens_sized(const Detection& found) {
  return found.image_size.width == kLens.size.width &&
         found.image_size.height == kLens.size.height;
}

bool in_frame(const cv::Point2d& p) {
  return p.x >= 2.0 && p.y >= 2.0 && p.x <= kLens.size.width - 3.0 &&
         p.y <= kLens.size.height - 3.0;
}

// A clean render of the board at `pose` through the lens, one square of
// white margin around it, as findChessboardCornersSB wants.
cv::Mat render(const compute::Board& board, const camera::Mat4d& pose) {
  test::RenderOptions options;
  options.ray = kLens.pixel_to_ray();
  options.supersampling = 2;
  options.margin_m = board.target().square_size_m;
  return test::render_board(kLens.frame(), pose, board, options);
}

// The camera's picture of a render: blurred, with its noise.
cv::Mat picture(const cv::Mat& clean, std::uint64_t seed) {
  test::CameraOptions optics;
  optics.seed = seed;
  return test::camera_image(clean, optics);
}

// The picture halved as sensor halves it on the GPU: each pixel the rounded
// mean of 2 x 2, an odd last row or column left out.
cv::Mat halved(const cv::Mat& full) {
  cv::Mat half(full.rows / 2, full.cols / 2, CV_8U);
  for (int y = 0; y < half.rows; ++y) {
    for (int x = 0; x < half.cols; ++x) {
      const int sum = full.at<std::uint8_t>(2 * y, 2 * x) +
                      full.at<std::uint8_t>(2 * y, 2 * x + 1) +
                      full.at<std::uint8_t>(2 * y + 1, 2 * x) +
                      full.at<std::uint8_t>(2 * y + 1, 2 * x + 1);
      half.at<std::uint8_t>(y, x) = static_cast<std::uint8_t>((sum + 2) / 4);
    }
  }
  return half;
}

Detection detect(const cv::Mat& image, const compute::Board& board,
                 const cv::Mat* half = nullptr) {
  DetectOptions options;
  const LumaView half_view = half != nullptr ? luma(*half) : LumaView{};
  if (half != nullptr) options.half = &half_view;
  core::Result<Detection> found = core::Status::invalid_argument("not run");
  EXPECT_NO_THROW(found = detect_board(luma(image), board, options));
  EXPECT_TRUE(found.ok()) << found.status().message();
  return found.ok() ? *std::move(found) : Detection{};
}

// How far each corner found is from the nearest true corner (pixels).
std::vector<double> errors(const Detection& found,
                           const std::vector<cv::Point2d>& corners) {
  std::vector<double> out;
  for (const DetectedPoint& point : found.observation.points) {
    double nearest = 1e300;
    for (const cv::Point2d& c : corners) {
      nearest = std::min(nearest,
                         std::hypot(point.pixel.x - c.x, point.pixel.y - c.y));
    }
    out.push_back(nearest);
  }
  return out;
}

double median_error(const Detection& found,
                    const std::vector<cv::Point2d>& corners) {
  return compute::median(errors(found, corners));
}

// Whether the ids found number the corners as a grid: one of the square's
// eight symmetries and a shift takes each corner's (row, column) by its id
// to the (row, column) of the true corner nearest it.
::testing::AssertionResult NumberedAsAGrid(
    const Detection& found, const std::vector<cv::Point2d>& corners, int cols) {
  if (found.observation.empty()) return ::testing::AssertionFailure() << "none";
  std::vector<std::array<int, 4>> pairs;  // found row, col; true row, col
  for (const DetectedPoint& point : found.observation.points) {
    std::size_t nearest = 0;
    double best = 1e300;
    for (std::size_t i = 0; i < corners.size(); ++i) {
      const double d = std::hypot(point.pixel.x - corners[i].x,
                                  point.pixel.y - corners[i].y);
      if (d < best) {
        best = d;
        nearest = i;
      }
    }
    const int t = static_cast<int>(nearest);
    pairs.push_back({point.id / cols, point.id % cols, t / cols, t % cols});
  }
  for (int a = -1; a <= 1; ++a) {
    for (int b = -1; b <= 1; ++b) {
      for (const bool swap : {false, true}) {
        if (a == 0 || b == 0) continue;
        const auto map = [&](const std::array<int, 4>& p) {
          return swap ? std::array<int, 2>{a * p[1], b * p[0]}
                      : std::array<int, 2>{a * p[0], b * p[1]};
        };
        const std::array<int, 2> m0 = map(pairs[0]);
        const int dr = pairs[0][2] - m0[0];
        const int dc = pairs[0][3] - m0[1];
        bool all = true;
        for (const std::array<int, 4>& p : pairs) {
          const std::array<int, 2> m = map(p);
          all = all && p[2] - m[0] == dr && p[3] - m[1] == dc;
        }
        if (all) return ::testing::AssertionSuccess();
      }
    }
  }
  return ::testing::AssertionFailure() << "the ids are not a grid's";
}

// A pose of the lab's checkerboard (facing's), and its clean render, made
// once.
struct Scene {
  const char* name;
  double off_axis_deg;
  double azimuth_deg;
  double distance_m;
  double roll_deg;
  double tilt_deg;
};

camera::Mat4d pose_of(const Scene& scene) {
  return facing(test::lab_checkerboard(), scene.off_axis_deg, scene.azimuth_deg,
                scene.distance_m, scene.roll_deg, scene.tilt_deg);
}

const cv::Mat& clean(const Scene& scene) {
  static std::map<std::string, cv::Mat> renders;
  cv::Mat& image = renders[scene.name];
  if (image.empty()) image = render(test::lab_checkerboard(), pose_of(scene));
  return image;
}

std::vector<cv::Point2d> truth(const Scene& scene) {
  return truth(test::lab_checkerboard(), pose_of(scene));
}

// Whole, near the centre, turned about the line of sight and tilted 25
// degrees, as a lens's views are.
const Scene kTurned[] = {
    {"turned 0", 5.0, 30.0, 0.6, 0.0, 25.0},
    {"turned 45", 5.0, 30.0, 0.6, 45.0, 25.0},
    {"turned 90", 5.0, 30.0, 0.7, 90.0, 25.0},
    {"turned 180", 5.0, 30.0, 0.6, 180.0, 25.0},
    {"turned 270", 5.0, 30.0, 0.7, 270.0, 25.0},
};
// Facing the camera squarely, where SB's corners err most.
const Scene kFrontal = {"frontal", 5.0, 30.0, 0.6, 0.0, 0.0};
// Cut by each edge of the picture (its sides 68 degrees off axis, its top and
// bottom 36), and in each corner (77-78 degrees).
const Scene kCut[] = {
    {"cut by the right edge", 66.0, 0.0, 0.5, 5.0, 0.0},
    {"cut by the left edge", 66.0, 180.0, 0.5, -5.0, 0.0},
    {"cut by the top edge", 36.0, -90.0, 0.5, 3.0, 0.0},
    {"cut by the bottom edge", 36.0, 90.0, 0.5, -3.0, 0.0},
    {"in the top-left corner", 66.0, -150.6, 0.5, 17.0, 0.0},
    {"in the top-right corner", 66.0, -29.4, 0.5, -17.0, 0.0},
    {"in the bottom-left corner", 66.0, 150.6, 0.5, 17.0, 0.0},
    {"in the bottom-right corner", 66.0, 29.4, 0.5, -17.0, 0.0},
};

// A whole board turned any way about the line of sight: every corner, each
// within half a pixel, numbered as a grid. SB's corners err least on a grid
// turned from the pixels' axes: under 0.1 px (median) at 45 degrees, 0.14 to
// 0.19 px square to them, here and on the design review's own renderer.
TEST(DetectBoard, FindsAWholeBoardTurnedAnyWay) {
  const compute::Board board = test::lab_checkerboard();
  for (const Scene& scene : kTurned) {
    const Detection found = detect(picture(clean(scene), 1), board);
    const std::vector<cv::Point2d> corners = truth(scene);
    EXPECT_TRUE(lens_sized(found)) << scene.name;
    ASSERT_EQ(found.observation.size(), 104u) << scene.name;
    EXPECT_TRUE(found.markers.empty()) << scene.name;
    EXPECT_TRUE(NumberedAsAGrid(found, corners, 13)) << scene.name;
    const std::vector<double> e = errors(found, corners);
    const bool square = std::fmod(scene.roll_deg, 90.0) == 0.0;
    EXPECT_LT(compute::median(e), square ? 0.25 : 0.1) << scene.name;
    EXPECT_LT(*std::max_element(e.begin(), e.end()), 0.5) << scene.name;
    for (std::size_t i = 1; i < found.observation.size(); ++i) {
      EXPECT_LT(found.observation.points[i - 1].id,
                found.observation.points[i].id);
    }
  }
  const Detection frontal = detect(picture(clean(kFrontal), 1), board);
  ASSERT_EQ(frontal.observation.size(), 104u);
  EXPECT_TRUE(NumberedAsAGrid(frontal, truth(kFrontal), 13));
  EXPECT_LT(median_error(frontal, truth(kFrontal)), 0.25);
}

// A board the picture's edge cuts, at a side or in a corner, is found: SB is
// asked for 5 x 4 corners, and any larger grid counts. Asked for the whole
// board, SB finds none of them.
TEST(DetectBoard, FindsABoardCutByEachEdgeAndInEachCorner) {
  const compute::Board board = test::lab_checkerboard();
  for (const Scene& scene : kCut) {
    const std::vector<cv::Point2d> corners = truth(scene);
    const auto inside = std::count_if(corners.begin(), corners.end(), in_frame);
    ASSERT_LT(inside, 100) << scene.name << ": not cut";
    const cv::Mat image = picture(clean(scene), 1);
    const Detection found = detect(image, board);
    EXPECT_GE(found.observation.size(), 20u) << scene.name;
    EXPECT_TRUE(NumberedAsAGrid(found, corners, 13)) << scene.name;
    EXPECT_LT(median_error(found, corners), 0.15) << scene.name;

    std::vector<cv::Point2f> whole;
    EXPECT_FALSE(cv::findChessboardCornersSB(image, cv::Size(13, 8), whole,
                                             cv::CALIB_CB_NORMALIZE_IMAGE |
                                                 cv::CALIB_CB_EXHAUSTIVE |
                                                 cv::CALIB_CB_LARGER))
        << scene.name;
  }
}

// Searched in the half and refined in the picture, the corners are within
// 0.05 px of the full search's (median error). SB in the half, without
// CALIB_CB_EXHAUSTIVE, misses more boards: here a tilted one square to the
// pixels (the turned 0 one returns a grid skipping every other row, which
// detect_board drops; KeepsOnlyCornersOnTheBoard).
TEST(DetectBoard, TheHalfFindsNearlyAsWellAsThePicture) {
  const compute::Board board = test::lab_checkerboard();
  std::size_t found = 0;
  for (const Scene& scene : {kTurned[1], kTurned[2], kTurned[3], kTurned[4],
                             kCut[0], kCut[3], kCut[4]}) {
    const cv::Mat image = picture(clean(scene), 1);
    const cv::Mat half = halved(image);
    const std::vector<cv::Point2d> corners = truth(scene);
    const Detection full = detect(image, board);
    const Detection live = detect(image, board, &half);
    if (live.observation.empty()) continue;
    ++found;
    EXPECT_GE(live.observation.size(), 20u) << scene.name;
    EXPECT_TRUE(lens_sized(live)) << scene.name;
    EXPECT_TRUE(NumberedAsAGrid(live, corners, 13)) << scene.name;
    EXPECT_LT(median_error(live, corners), median_error(full, corners) + 0.05)
        << scene.name;
  }
  EXPECT_GE(found, 5u);
}

// The board entered smaller than the one seen: the grid found is larger
// either way round, and refused, naming both sizes.
TEST(DetectBoard, RefusesAGridLargerThanTheBoard) {
  Target entered = legacy_checkerboard();
  entered.cols = 9;
  entered.rows = 6;
  const compute::Board small = compute::Board::create(entered).value();
  const cv::Mat image = picture(clean(kTurned[0]), 1);
  core::Result<Detection> found = Detection{};
  EXPECT_NO_THROW(found = detect_board(luma(image), small));
  ASSERT_EQ(found.status().domain(), Code::InvalidArgument);
  const std::string& message = found.status().message();
  EXPECT_TRUE(message.find("of 13 x 8 inner corners") != std::string::npos ||
              message.find("of 8 x 13 inner corners") != std::string::npos)
      << message;
  EXPECT_NE(message.find("the board's 9 x 6 either way round"),
            std::string::npos)
      << message;

  // Entered turned, 8 x 13, the board is found.
  entered = legacy_checkerboard();
  std::swap(entered.cols, entered.rows);
  const compute::Board turned = compute::Board::create(entered).value();
  EXPECT_EQ(detect(image, turned).observation.size(), 104u);
}

TEST(DetectBoard, FindsNothingInAPictureWithoutABoard) {
  const cv::Mat blank(2160, 3840, CV_8U, cv::Scalar(110));
  const cv::Mat image = picture(blank, 3);
  const cv::Mat half = halved(image);
  for (const compute::Board& board :
       {test::lab_checkerboard(), test::femto_board()}) {
    for (const cv::Mat* searched : {static_cast<const cv::Mat*>(nullptr),
                                    static_cast<const cv::Mat*>(&half)}) {
      const Detection found = detect(image, board, searched);
      EXPECT_TRUE(found.observation.empty());
      EXPECT_TRUE(found.markers.empty());
      EXPECT_TRUE(lens_sized(found));
    }
  }
}

// The picture and its half are checked as detect_charuco checks a picture,
// and the half must be the picture's halved size.
TEST(DetectBoard, RefusesAPictureOrHalfItCannotRead) {
  const compute::Board board = test::lab_checkerboard();
  const cv::Mat image(9, 7, CV_8U, cv::Scalar(0));
  const cv::Mat half(4, 3, CV_8U, cv::Scalar(0));
  const auto refused = [&](const LumaView& searched,
                           const LumaView* half_view) {
    DetectOptions options;
    options.half = half_view;
    core::Result<Detection> found = Detection{};
    EXPECT_NO_THROW(found = detect_board(searched, board, options));
    return found.status();
  };
  EXPECT_TRUE(test::Refused(refused({}, nullptr),
                            "detect_board: the image has no pixels"));
  LumaView no_data = luma(image);
  no_data.data = nullptr;
  EXPECT_TRUE(test::Refused(refused(no_data, nullptr),
                            "detect_board: image.data is null"));
  LumaView narrow = luma(half);
  narrow.stride = 2;
  const LumaView good = luma(half);
  EXPECT_TRUE(refused(luma(image), &good).ok());
  EXPECT_TRUE(test::Refused(
      refused(luma(image), &narrow),
      "detect_board: options.half: image.stride is below image.width"));
  const cv::Mat wrong(4, 4, CV_8U, cv::Scalar(0));
  const LumaView wrong_view = luma(wrong);
  EXPECT_TRUE(test::Refused(refused(luma(image), &wrong_view),
                            "detect_board: options.half is 4 x 4, not the "
                            "picture halved (3 x 4)"));
  const cv::Mat one(1, 1, CV_8U, cv::Scalar(0));
  EXPECT_TRUE(test::Refused(refused(luma(one), &good),
                            "detect_board: options.half is 3 x 4, not the "
                            "picture halved (0 x 0)"));
}

// A ChArUco board: as detect_charuco finds it, in Full mode without a half,
// and in Hybrid mode, whose own halving is the GPU's, with one.
TEST(DetectBoard, FindsAChArUcoBoardAsDetectCharucoDoes) {
  const compute::Board board = test::femto_board();
  const cv::Mat image = test::camera_image(test::render_board(
      test::color_like_lens(),
      test::board_facing_camera(board, {0.2, 0.1, 2.6}, 40.0, 15.0), board));
  const cv::Mat half = halved(image);
  const Detection full = detect(image, board);
  const Detection live = detect(image, board, &half);
  ASSERT_EQ(full.observation.size(), 8u);
  ASSERT_EQ(live.observation.size(), 8u);
  const Detection want_full =
      detect_charuco(luma(image), board, Mode::Full).value();
  const Detection want_live =
      detect_charuco(luma(image), board, Mode::Hybrid).value();
  for (const auto& [got, want] :
       {std::pair{&full, &want_full}, std::pair{&live, &want_live}}) {
    ASSERT_EQ(got->observation.size(), want->observation.size());
    for (std::size_t i = 0; i < got->observation.size(); ++i) {
      EXPECT_EQ(got->observation.points[i].id, want->observation.points[i].id);
      EXPECT_EQ(got->observation.points[i].pixel,
                want->observation.points[i].pixel);
    }
    ASSERT_EQ(got->markers.size(), want->markers.size());
    for (std::size_t i = 0; i < got->markers.size(); ++i) {
      EXPECT_EQ(got->markers[i].id, want->markers[i].id);
      EXPECT_EQ(got->markers[i].corners, want->markers[i].corners);
    }
  }
  // detect_charuco takes no checkerboard.
  EXPECT_EQ(detect_charuco(luma(image), test::lab_checkerboard(), Mode::Full)
                .status()
                .domain(),
            Code::InvalidArgument);
}

// A still board, turned 45 degrees or cut by an edge, in two pictures that
// differ only in noise: its corners, paired by position, move under the
// capture rule's 0.3 px, live and full alike; the rule takes the set; and
// the live corners agree with the full ones.
TEST(DetectBoard, AStillBoardReadsStill) {
  const compute::Board board = test::lab_checkerboard();
  const quality::CornerMatch match = quality::corner_match(board);
  ASSERT_EQ(match, quality::CornerMatch::Nearest);
  for (const Scene& scene : {kTurned[1], kCut[0], kCut[6]}) {
    const cv::Mat first = picture(clean(scene), 11);
    const cv::Mat second = picture(clean(scene), 12);
    const cv::Mat first_half = halved(first);
    const cv::Mat second_half = halved(second);
    const Detection live_before = detect(first, board, &first_half);
    const Detection live_after = detect(second, board, &second_half);
    const Detection full_after = detect(second, board);
    ASSERT_GE(live_after.observation.size(), 20u) << scene.name;
    ASSERT_GE(full_after.observation.size(), 20u) << scene.name;

    const quality::CornerShift live =
        quality::corner_shift({live_before}, {live_after}, match).value();
    ASSERT_TRUE(live.median_px) << scene.name;
    EXPECT_LT(*live.median_px, 0.3) << scene.name;

    quality::CaptureRule rule = quality::CaptureRule::create({"bm-1"}).value();
    const auto now = quality::CaptureRule::Clock::now();
    ASSERT_TRUE(rule.arm(now));
    quality::LiveSet set;
    set.present = {true};
    set.checks = {ViewCheck{}};
    set.motion_px = live.median_px;
    EXPECT_EQ(rule.update(set, now).value().state, quality::CaptureState::Take)
        << scene.name;

    const quality::CornerShift live_vs_full =
        quality::corner_shift({live_after}, {full_after}, match).value();
    ASSERT_TRUE(live_vs_full.median_px) << scene.name;
    EXPECT_FALSE(quality::live_vs_full_flagged(live_vs_full))
        << scene.name << ": median " << *live_vs_full.median_px << ", max "
        << *live_vs_full.max_px;
  }
}

// Grown past the grid asked for, SB at times adds corners past the board --
// T-junctions where it meets its white margin, or corners off it altogether
// (60 px and more in the fisheye's corners) -- or returns a grid that is no
// grid: in the half of the tilted whole board, one that skips every other row
// of squares. Over such views, in the picture and in its half, every corner
// detect_board keeps is within a pixel of a true one, and its grid is
// numbered as a grid.
TEST(DetectBoard, KeepsOnlyCornersOnTheBoard) {
  const compute::Board board = test::lab_checkerboard();
  const Scene scenes[] = {
      {"bottom-left, turned back", 66.0, 150.6, 0.5, -17.0, 0.0},
      {"bottom-right, turned back", 66.0, 29.4, 0.5, 17.0, 0.0},
      {"bottom-right, nearer the edge", 64.0, 29.4, 0.5, 0.0, 0.0},
      {"top-left, nearer the middle", 62.0, -150.6, 0.55, 17.0, 0.0},
      {"top-right, turned back", 66.0, -29.4, 0.5, -17.0, 0.0},
      {"whole, turned 45", 5.0, 30.0, 0.6, 45.0, 0.0},
      kTurned[0],
  };
  std::size_t kept = 0;
  for (const Scene& scene : scenes) {
    const std::vector<cv::Point2d> corners = truth(scene);
    for (const std::uint64_t seed : {1, 2}) {
      const cv::Mat image = picture(clean(scene), seed);
      const cv::Mat half = halved(image);
      for (const cv::Mat* searched : {static_cast<const cv::Mat*>(nullptr),
                                      static_cast<const cv::Mat*>(&half)}) {
        const Detection found = detect(image, board, searched);
        const char* in = searched != nullptr ? "half" : "picture";
        if (found.observation.empty()) continue;
        const std::vector<double> e = errors(found, corners);
        EXPECT_LT(*std::max_element(e.begin(), e.end()), 1.0)
            << scene.name << ", seed " << seed << ", " << in;
        EXPECT_TRUE(NumberedAsAGrid(found, corners, 13))
            << scene.name << ", seed " << seed << ", " << in;
        ++kept;
      }
    }
  }
  std::printf("%zu of %zu searches kept a board\n", kept,
              2 * std::size(scenes) * 2);
  EXPECT_GE(kept, 8u) << "few searches kept a board";
}

// U1: findChessboardCornersSB's time on 20 pictures at 4K, in the picture
// (captures) and in its half (the live view), board or none. Run on the lab
// box once: `ctest -L slow -R detect_board.timing -V`.
TEST(DetectBoardTiming, PrintsSearchTimes) {
  if (std::getenv("VC_TEST_TIMING") == nullptr) {
    GTEST_SKIP() << "set VC_TEST_TIMING (detect_board.timing)";
  }
  cv::setNumThreads(-1);  // OpenCV's own threads, as calib_rig runs it
  const compute::Board board = test::lab_checkerboard();
  std::vector<cv::Mat> images;
  for (int i = 0; i < 18; ++i) {
    const Scene scene{"",       10.0 + 3.0 * i, 20.0 * i, 0.5 + 0.05 * (i % 6),
                      15.0 * i, 5.0 * (i % 7)};
    images.push_back(picture(render(board, pose_of(scene)),
                             static_cast<std::uint64_t>(i + 1)));
  }
  for (int i = 0; i < 2; ++i) {
    images.push_back(
        picture(cv::Mat(2160, 3840, CV_8U, cv::Scalar(110)), 100 + i));
  }
  std::vector<double> full_ms, half_ms;
  std::size_t found = 0;
  for (const cv::Mat& image : images) {
    const cv::Mat half = halved(image);
    auto start = std::chrono::steady_clock::now();
    const Detection in_full = detect(image, board);
    full_ms.push_back(std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start)
                          .count());
    start = std::chrono::steady_clock::now();
    detect(image, board, &half);
    half_ms.push_back(std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start)
                          .count());
    if (!in_full.observation.empty()) ++found;
  }
  const auto report = [](const char* what, const std::vector<double>& ms) {
    std::printf("detect_board %s: median %.0f ms, max %.0f ms over %zu\n", what,
                compute::median(ms), *std::max_element(ms.begin(), ms.end()),
                ms.size());
  };
  std::printf("boards found in %zu of %zu pictures\n", found, images.size());
  report("in the picture (4K)", full_ms);
  report("in the half (1920 x 1080), refined at 4K", half_ms);
}

}  // namespace
}  // namespace volumetric_kit::calib::detect
