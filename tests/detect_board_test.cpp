// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// One detector for both boards. The lab's checkerboard rendered at 4K through
// a fisheye truth (KB4, f 1702, k1 -0.033: the lab's Blackmagic lens) is
// found turned, cut by the picture's edges in its corners, in the picture and
// in its half, its grid numbered as a grid; corners SB adds off the board and
// grids that skip squares are dropped; a grid larger than the board is
// refused; a picture gives one detection; a still board reads still, by
// position, between noise-only pictures. A ChArUco board is found in the
// picture and its half (detect_charuco_test holds that path to where the
// board projects).
//
// A search at 4K takes about a quarter of a second, so each rule is held on
// the fewest pictures that show it, each searched once (found()). The sweep,
// `detect_board.sweeps` (labelled slow, VC_TEST_SWEEPS), runs every scene,
// seed and state below and replays SB's own grids. DetectBoardTiming prints
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
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/objdetect.hpp>

#include "detect_helpers.hpp"
#include "synthetic_rig.hpp"
#include "synthetic_scene.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/quality/capture_rule.hpp"
#include "volumetric_kit/calib/quality/live_checks.hpp"

namespace volumetric_kit::calib::detect {
namespace {

namespace camera = sensor::camera;
using Code = core::Status::Code;
using test::luma;

// The lab's Blackmagic lens at 4K.
const test::FisheyeTruth kLens;

// Whether this run is the sweep, `detect_board.sweeps`.
bool sweep() { return test::sweep_size(0, 1) == 1; }

// The first `few` of `all`, or, in the sweep, all of them.
template <typename T, std::size_t N>
std::vector<T> first(const T (&all)[N], std::uint32_t few) {
  return std::vector<T>(all, all + test::sweep_size(few, N));
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

using test::halved;

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
// four rotations and a shift takes each corner's (row, column) by its id to
// the (row, column) of the true corner nearest it. A mirror would not do: it
// flips the board in every pose posed from those ids.
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
        if ((a * b > 0) == swap) continue;  // a mirror
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

// A pose of the lab's checkerboard (facing's).
struct Scene {
  const char* name;
  double off_axis_deg;
  double azimuth_deg;
  double distance_m;
  double roll_deg;
  double tilt_deg;
};

camera::Mat4d pose_of(const Scene& scene) {
  return test::facing(test::lab_checkerboard(), scene.off_axis_deg,
                      scene.azimuth_deg, scene.distance_m, scene.roll_deg,
                      scene.tilt_deg);
}

// A scene's clean render, made once.
const cv::Mat& clean(const Scene& scene) {
  static std::map<std::string, cv::Mat> renders;
  cv::Mat& image = renders[scene.name];
  if (image.empty()) image = render(test::lab_checkerboard(), pose_of(scene));
  return image;
}

// A scene's picture with noise `seed`, and its half: made once.
const cv::Mat& picture(const Scene& scene, std::uint64_t seed,
                       bool half = false) {
  static std::map<std::tuple<std::string, std::uint64_t, bool>, cv::Mat> made;
  cv::Mat& image = made[{scene.name, seed, half}];
  if (image.empty()) {
    image = half ? halved(picture(scene, seed)) : picture(clean(scene), seed);
  }
  return image;
}

// The lab's checkerboard found in a scene's picture (searched in its half
// when `in_half`): searched once.
const Detection& found(const Scene& scene, std::uint64_t seed,
                       bool in_half = false) {
  static std::map<std::tuple<std::string, std::uint64_t, bool>, Detection>
      searched;
  const auto key = std::make_tuple(std::string(scene.name), seed, in_half);
  const auto at = searched.find(key);
  if (at != searched.end()) return at->second;
  const cv::Mat* half = in_half ? &picture(scene, seed, true) : nullptr;
  return searched[key] =
             detect(picture(scene, seed), test::lab_checkerboard(), half);
}

std::vector<cv::Point2d> truth(const Scene& scene) {
  return truth(test::lab_checkerboard(), pose_of(scene));
}

// Whole, near the centre, turned about the line of sight and tilted 25
// degrees, as a lens's views are.
const Scene kTurned0 = {"turned 0", 5.0, 30.0, 0.6, 0.0, 25.0};
const Scene kTurned45 = {"turned 45", 5.0, 30.0, 0.6, 45.0, 25.0};
const Scene kTurned90 = {"turned 90", 5.0, 30.0, 0.7, 90.0, 25.0};
const Scene kTurned180 = {"turned 180", 5.0, 30.0, 0.6, 180.0, 25.0};
const Scene kTurned270 = {"turned 270", 5.0, 30.0, 0.7, 270.0, 25.0};
// Facing the camera squarely, where SB's corners err most.
const Scene kFrontal = {"frontal", 5.0, 30.0, 0.6, 0.0, 0.0};
// Cut by each edge of the picture (its sides 68 degrees off axis, its top and
// bottom 36), and in each corner (77-78 degrees).
const Scene kTopLeft = {"in the top-left corner", 66.0, -150.6, 0.5, 17.0, 0.0};
const Scene kBottomRight = {
    "in the bottom-right corner", 66.0, 29.4, 0.5, -17.0, 0.0};
const Scene kTopRight = {
    "in the top-right corner", 66.0, -29.4, 0.5, -17.0, 0.0};
const Scene kBottomLeft = {
    "in the bottom-left corner", 66.0, 150.6, 0.5, 17.0, 0.0};
const Scene kRightEdge = {"cut by the right edge", 66.0, 0.0, 0.5, 5.0, 0.0};
const Scene kLeftEdge = {"cut by the left edge", 66.0, 180.0, 0.5, -5.0, 0.0};
const Scene kTopEdge = {"cut by the top edge", 36.0, -90.0, 0.5, 3.0, 0.0};
const Scene kBottomEdge = {
    "cut by the bottom edge", 36.0, 90.0, 0.5, -3.0, 0.0};

// Turned a quarter, the board's rows are the grid's columns; frontal, they
// are its rows. The sweep turns it every way.
const Scene kTurned[] = {kTurned90, kFrontal,   kTurned0,
                         kTurned45, kTurned180, kTurned270};
// Two opposite corners are cut by all four edges.
const Scene kCorners[] = {kTopLeft, kBottomRight, kTopRight, kBottomLeft};
const Scene kCut[] = {kTopLeft,   kBottomRight, kTopRight, kBottomLeft,
                      kRightEdge, kLeftEdge,    kTopEdge,  kBottomEdge};

// A whole board turned about the line of sight: every corner, each within
// half a pixel, under 0.1 px at the median, numbered as a grid. On a board
// square to the pixels SB's own corners err 0.14 to 0.19 px: only the
// refinement holds this.
TEST(DetectBoard, FindsAWholeBoardTurnedAnyWay) {
  for (const Scene& scene : first(kTurned, 2)) {
    const Detection& board = found(scene, 1);
    const std::vector<cv::Point2d> corners = truth(scene);
    EXPECT_TRUE(lens_sized(board)) << scene.name;
    ASSERT_EQ(board.observation.size(), 104u) << scene.name;
    EXPECT_TRUE(board.markers.empty()) << scene.name;
    EXPECT_TRUE(NumberedAsAGrid(board, corners, 13)) << scene.name;
    const std::vector<double> e = errors(board, corners);
    EXPECT_LT(compute::median(e), 0.1) << scene.name;
    EXPECT_LT(*std::max_element(e.begin(), e.end()), 0.5) << scene.name;
    for (std::size_t i = 1; i < board.observation.size(); ++i) {
      EXPECT_LT(board.observation.points[i - 1].id,
                board.observation.points[i].id);
    }
  }
}

// A board the picture's edge cuts, at a side or in a corner, is found: SB is
// asked for 5 x 4 corners, and any larger grid counts. Asked for the whole
// board, SB finds none of them (replayed in the sweep).
TEST(DetectBoard, FindsABoardCutByEachEdgeAndInEachCorner) {
  for (const Scene& scene : first(kCut, 2)) {
    const std::vector<cv::Point2d> corners = truth(scene);
    const auto inside = std::count_if(corners.begin(), corners.end(), in_frame);
    ASSERT_LT(inside, 100) << scene.name << ": not cut";
    const Detection& board = found(scene, 1);
    EXPECT_GE(board.observation.size(), 20u) << scene.name;
    EXPECT_TRUE(NumberedAsAGrid(board, corners, 13)) << scene.name;
    EXPECT_LT(median_error(board, corners), 0.1) << scene.name;

    if (!sweep()) continue;
    std::vector<cv::Point2f> whole;
    EXPECT_FALSE(cv::findChessboardCornersSB(
        picture(scene, 1), cv::Size(13, 8), whole,
        cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE |
            cv::CALIB_CB_LARGER))
        << scene.name;
  }
}

// A corner beside the picture's edge, one of its squares' centres off the
// picture, is judged by the other three. (With a whole side of its squares
// off the picture, no diagonal pair is left, and it is dropped.)
TEST(DetectBoard, KeepsCornersBesideThePicturesEdge) {
  for (const Scene& scene : first(kCorners, 2)) {
    const Detection& board = found(scene, 1);
    ASSERT_FALSE(board.observation.empty()) << scene.name;
    double nearest = 1e300;
    for (const DetectedPoint& point : board.observation.points) {
      nearest = std::min({nearest, point.pixel.x, point.pixel.y,
                          kLens.size.width - 1.0 - point.pixel.x,
                          kLens.size.height - 1.0 - point.pixel.y});
    }
    EXPECT_LT(nearest, 35.0) << scene.name;
  }
}

// Searched in the half and refined in the picture, the corners are within
// 0.05 px of the full search's (median error). SB in the half, without
// CALIB_CB_EXHAUSTIVE, misses more boards: in the sweep, a tilted one square
// to the pixels (the turned 180 one; turned 0, it returns a grid skipping
// every other row, which detect_board drops: KeepsOnlyCornersOnTheBoard).
TEST(DetectBoard, TheHalfFindsNearlyAsWellAsThePicture) {
  const Scene scenes[] = {kTurned90,  kTopLeft,   kTurned45,  kTurned180,
                          kTurned270, kRightEdge, kBottomEdge};
  const std::vector<Scene> searched = first(scenes, 2);
  std::size_t in_half = 0;
  for (const Scene& scene : searched) {
    const std::vector<cv::Point2d> corners = truth(scene);
    const Detection& live = found(scene, 1, true);
    if (live.observation.empty()) continue;
    ++in_half;
    EXPECT_GE(live.observation.size(), 20u) << scene.name;
    EXPECT_TRUE(lens_sized(live)) << scene.name;
    EXPECT_TRUE(NumberedAsAGrid(live, corners, 13)) << scene.name;
    EXPECT_LT(median_error(live, corners),
              median_error(found(scene, 1), corners) + 0.05)
        << scene.name;
  }
  EXPECT_GE(in_half, sweep() ? 5u : searched.size());
}

// The board entered smaller than the one seen: the grid found is larger
// either way round, and refused, naming both sizes.
TEST(DetectBoard, RefusesAGridLargerThanTheBoard) {
  Target entered = legacy_checkerboard();
  entered.cols = 9;
  entered.rows = 6;
  const compute::Board small = compute::Board::create(entered).value();
  const cv::Mat& image = picture(kFrontal, 1);
  core::Result<Detection> board = Detection{};
  EXPECT_NO_THROW(board = detect_board(luma(image), small));
  ASSERT_EQ(board.status().domain(), Code::InvalidArgument);
  const std::string& message = board.status().message();
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
      const Detection board_found = detect(image, board, searched);
      EXPECT_TRUE(board_found.observation.empty());
      EXPECT_TRUE(board_found.markers.empty());
      EXPECT_TRUE(lens_sized(board_found));
    }
  }
}

// The picture and its half are checked alike, and the half must be the
// picture's halved size.
TEST(DetectBoard, RefusesAPictureOrHalfItCannotRead) {
  const compute::Board board = test::lab_checkerboard();
  const cv::Mat image(9, 7, CV_8U, cv::Scalar(0));
  const cv::Mat half(4, 3, CV_8U, cv::Scalar(0));
  const auto refused = [&](const LumaView& searched,
                           const LumaView* half_view) {
    DetectOptions options;
    options.half = half_view;
    core::Result<Detection> board_found = Detection{};
    EXPECT_NO_THROW(board_found = detect_board(searched, board, options));
    return board_found.status();
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

// A ChArUco board, in the picture and in its half, by id with its markers
// (detect_charuco_test holds both paths to where the board projects, so the
// render here samples each pixel once).
TEST(DetectBoard, FindsAChArUcoBoard) {
  const compute::Board board = test::femto_board();
  test::RenderOptions once;
  once.supersampling = 1;
  const cv::Mat image = test::camera_image(test::render_board(
      test::color_like_lens(),
      test::board_facing_camera(board, {0.2, 0.1, 2.6}, 40.0, 15.0), board,
      once));
  const cv::Mat half = halved(image);
  const Detection full = detect(image, board);
  const Detection live = detect(image, board, &half);
  ASSERT_EQ(full.observation.size(), 8u);
  ASSERT_EQ(live.observation.size(), 8u);
  for (std::size_t i = 0; i < full.observation.size(); ++i) {
    EXPECT_EQ(full.observation.points[i].id, static_cast<int>(i));
    EXPECT_EQ(live.observation.points[i].id, static_cast<int>(i));
  }
  EXPECT_EQ(full.markers.size(), 7u);
  EXPECT_EQ(live.markers.size(), 7u);
}

// A still board, in a corner (cut by two edges), turned 45 degrees or cut by
// one edge, in two pictures that differ only in noise: its corners, paired by
// position, move under the capture rule's 0.3 px, live and full alike; the
// rule takes the set; and the live corners agree with the full ones.
TEST(DetectBoard, AStillBoardReadsStill) {
  const compute::Board board = test::lab_checkerboard();
  const quality::CornerMatch match = quality::corner_match(board);
  ASSERT_EQ(match, quality::CornerMatch::Nearest);
  const Scene scenes[] = {kTopLeft, kTurned45, kRightEdge, kBottomLeft};
  for (const Scene& scene : first(scenes, 1)) {
    const Detection& live_before = found(scene, 11, true);
    const Detection& live_after = found(scene, 12, true);
    const Detection& full_after = found(scene, 12);
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

// One search of a scene's picture with noise `seed`, in its half or not.
struct Search {
  Scene scene;
  std::uint64_t seed;
  bool in_half;
};

// Grown past the grid asked for, SB at times adds corners past the board --
// T-junctions where it meets its white margin, or corners off it altogether
// (60 px and more in the fisheye's corners) -- or returns a grid that is no
// grid: in the half of the tilted whole board, one that skips every other row
// of squares. Over such views, in the picture and in its half, every corner
// detect_board keeps is within a pixel of a true one, and its grid is
// numbered as a grid. The first two searches fail without the filter (58 px
// off; not a grid); the sweep adds 28 more, and asks that 8 keep a board.
TEST(DetectBoard, KeepsOnlyCornersOnTheBoard) {
  const Scene bottom_left = {
      "bottom-left, turned back", 66.0, 150.6, 0.5, -17.0, 0.0};
  const Scene views[] = {
      bottom_left,
      {"bottom-right, turned back", 66.0, 29.4, 0.5, 17.0, 0.0},
      {"bottom-right, nearer the edge", 64.0, 29.4, 0.5, 0.0, 0.0},
      {"top-left, nearer the middle", 62.0, -150.6, 0.55, 17.0, 0.0},
      {"top-right, turned back", 66.0, -29.4, 0.5, -17.0, 0.0},
      {"whole, turned 45", 5.0, 30.0, 0.6, 45.0, 0.0},
      kTurned0,
  };
  std::vector<Search> searches = {{bottom_left, 1, false},
                                  {kTurned180, 1, true}};
  if (sweep()) {
    for (const Scene& scene : views) {
      for (const std::uint64_t seed : {1, 2}) {
        for (const bool in_half : {false, true}) {
          searches.push_back({scene, seed, in_half});
        }
      }
    }
  }
  std::size_t kept = 0;
  for (const Search& s : searches) {
    const Detection& board = found(s.scene, s.seed, s.in_half);
    const char* in = s.in_half ? "half" : "picture";
    if (board.observation.empty()) continue;
    const std::vector<cv::Point2d> corners = truth(s.scene);
    const std::vector<double> e = errors(board, corners);
    EXPECT_LT(*std::max_element(e.begin(), e.end()), 1.0)
        << s.scene.name << ", seed " << s.seed << ", " << in;
    EXPECT_TRUE(NumberedAsAGrid(board, corners, 13))
        << s.scene.name << ", seed " << s.seed << ", " << in;
    ++kept;
  }
  std::printf("%zu of %zu searches kept a board\n", kept, searches.size());
  if (sweep()) {
    EXPECT_GE(kept, 8u) << "few searches kept a board";
  }
}

// On these pictures SB, from the seed detect_board gives it, returns a grid
// that skips every other column of squares (one with a corner 64 px off the
// board too): true corners, numbered as neighbours. detect_board must drop
// such a grid, or keep only a grid's true corners. The sweep replays SB from
// that seed and says whether this OpenCV still returns such grids.
TEST(DetectBoard, DropsAGridThatSkipsSquares) {
  const Scene skips = {"skips columns", 62.0, 0.0, 0.55, 30.0, 0.0};
  const Scene strays = {
      "skips columns, one astray", 64.0, 0.0, 0.55, 30.0, 0.0};
  const Scene ends = {"skips at its ends", 20.0, 90.0, 0.75, 90.0, 35.0};
  const Search pictures[] = {{strays, 8, false},
                             {ends, 3, true},
                             {skips, 3, false},
                             {skips, 16, false}};
  std::size_t broken = 0;
  const std::vector<Search> searched = first(pictures, 2);
  for (const Search& p : searched) {
    const std::vector<cv::Point2d> corners = truth(p.scene);
    if (sweep()) {
      cv::theRNG() = cv::RNG();
      std::vector<cv::Point2f> grid;
      cv::Mat meta;
      cv::findChessboardCornersSB(
          picture(p.scene, p.seed, p.in_half), cv::Size(5, 4), grid,
          cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_LARGER |
              (p.in_half ? 0 : cv::CALIB_CB_EXHAUSTIVE),
          meta);
      const float scale = p.in_half ? 2.0f : 1.0f;
      Detection raw;
      for (std::size_t i = 0; !meta.empty() && i < grid.size(); ++i) {
        const int row = static_cast<int>(i) / meta.cols;
        const int column = static_cast<int>(i) % meta.cols;
        raw.observation.points.push_back(
            {row * 13 + column, {scale * grid[i].x, scale * grid[i].y}, 1.0});
      }
      if (!raw.observation.empty() && !NumberedAsAGrid(raw, corners, 13)) {
        ++broken;
      }
    }

    const Detection& board = found(p.scene, p.seed, p.in_half);
    if (board.observation.empty()) continue;
    const std::vector<double> e = errors(board, corners);
    EXPECT_LT(*std::max_element(e.begin(), e.end()), 1.0)
        << p.scene.name << ", noise " << p.seed;
    EXPECT_TRUE(NumberedAsAGrid(board, corners, 13))
        << p.scene.name << ", noise " << p.seed;
  }
  if (sweep()) {
    std::printf("SB's own grid was broken on %zu of %zu pictures\n", broken,
                searched.size());
  }
}

// SB's grid depends on OpenCV's random numbers: unseeded, the half of this
// picture is found from state 1 and not from 2 to 6 (OpenCV 5.0). detect_board
// seeds them
// for each search and puts them back: one picture gives one detection
// whatever they were, on any number of OpenCV threads. The sweep adds the
// picture itself, a board in a corner, and states 3 to 6.
TEST(DetectBoard, APictureGivesOneDetection) {
  std::vector<Search> searches = {{kTurned180, 2, true}};
  if (sweep()) {
    searches.push_back({kTurned180, 2, false});
    searches.push_back({kBottomLeft, 2, false});
    searches.push_back({kBottomLeft, 2, true});
  }
  const std::uint64_t states[] = {1, 2, 3, 4, 5, 6};
  const compute::Board board = test::lab_checkerboard();
  for (const Search& s : searches) {
    const cv::Mat& image = picture(s.scene, s.seed);
    const cv::Mat* half = s.in_half ? &picture(s.scene, s.seed, true) : nullptr;
    std::vector<Detection> runs;
    for (const std::uint64_t state : first(states, 2)) {
      cv::setNumThreads(state % 2 == 0 ? 1 : -1);
      cv::theRNG().state = state;
      runs.push_back(detect(image, board, half));
      EXPECT_EQ(cv::theRNG().state, state) << s.scene.name;
    }
    cv::setNumThreads(-1);
    for (const Detection& run : runs) {
      ASSERT_EQ(run.observation.size(), runs[0].observation.size())
          << s.scene.name;
      for (std::size_t i = 0; i < run.observation.size(); ++i) {
        EXPECT_EQ(run.observation.points[i].id,
                  runs[0].observation.points[i].id);
        EXPECT_EQ(run.observation.points[i].pixel,
                  runs[0].observation.points[i].pixel);
      }
    }
  }
}

// U1: findChessboardCornersSB's time on 20 pictures at 4K, in the picture
// (captures) and in its half (the live view), board or none. Run on the lab
// box once: `ctest -L slow -R detect_board.timing -V`.
TEST(DetectBoardTiming, PrintsSearchTimes) {
  if (std::getenv("VC_TEST_TIMING") == nullptr) {
    GTEST_SKIP() << "set VC_TEST_TIMING (detect_board.timing)";
  }
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
  std::size_t boards = 0;
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
    if (!in_full.observation.empty()) ++boards;
  }
  const auto report = [](const char* what, const std::vector<double>& ms) {
    std::printf("detect_board %s: median %.0f ms, max %.0f ms over %zu\n", what,
                compute::median(ms), *std::max_element(ms.begin(), ms.end()),
                ms.size());
  };
  std::printf("boards found in %zu of %zu pictures\n", boards, images.size());
  report("in the picture (4K)", full_ms);
  report("in the half (1920 x 1080), refined at 4K", half_ms);
}

}  // namespace
}  // namespace volumetric_kit::calib::detect
