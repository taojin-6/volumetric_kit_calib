// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The ChArUco board in a luma image: both modes against where the synthetic
// renderer's board projects (front-on, turned 60 and 70 degrees, a JPEG), an
// odd-sized image, several threads, part of a board, a collinear crop, no
// board, another board beside it, the hybrid's window and corner rejection,
// the images it refuses, and what OpenCV throws.

#include "volumetric_kit/calib/detect/charuco.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <ostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

#include "detect_helpers.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/usable_for_pose.hpp"

namespace volumetric_kit::calib::detect {
namespace {

namespace camera = recon::camera;
using Code = core::Status::Code;

using test::board_facing_camera;
using test::femto_board;
using test::luma;
using test::Refused;
using test::ThrowingParallelFor;

// An image of the board, and where its corners and markers project.
struct View {
  std::string name;
  cv::Mat image;
  std::vector<cv::Point2d> corners;               // by corner id
  std::vector<std::vector<cv::Point2d>> markers;  // by marker id
};

// The board at `board_to_camera` through the colour-like lens, with
// `options`' levels; `seed` seeds the camera's noise, or none when 0.
View render(std::string name, const compute::Board& board,
            const camera::Mat4d& board_to_camera, std::uint64_t seed,
            const test::RenderOptions& options = {}) {
  const camera::CameraModel model = test::color_like_lens();
  View view;
  view.name = std::move(name);
  test::CameraOptions optics;
  optics.seed = seed;
  if (seed == 0) optics.noise_dn = 0.0;
  view.image = test::camera_image(
      test::render_board(model, board_to_camera, board, options), optics);
  view.corners = test::project(model, board_to_camera, board.object_points());
  for (const std::vector<cv::Point3f>& marker :
       board.charuco().getObjPoints()) {
    view.markers.push_back(
        test::project(model, board_to_camera,
                      std::vector<cv::Point3d>(marker.begin(), marker.end())));
  }
  return view;
}

Detection detect(const cv::Mat& image, const compute::Board& board, Mode mode) {
  core::Result<Detection> detection = core::Status::invalid_argument("not run");
  EXPECT_NO_THROW(detection = detect_charuco(luma(image), board, mode));
  EXPECT_TRUE(detection.ok()) << detection.status().message();
  return detection.ok() ? *std::move(detection) : Detection{};
}

const char* name(Mode mode) { return mode == Mode::Full ? "Full" : "Hybrid"; }

std::vector<int> corner_ids(const Detection& detection) {
  std::vector<int> ids;
  for (const DetectedPoint& point : detection.observation.points) {
    ids.push_back(point.id);
  }
  return ids;
}

std::vector<int> marker_ids(const Detection& detection) {
  std::vector<int> ids;
  for (const DetectedMarker& marker : detection.markers) {
    ids.push_back(marker.id);
  }
  return ids;
}

// Whether two detections found the same features at the same pixels.
::testing::AssertionResult Same(const Detection& a, const Detection& b) {
  if (corner_ids(a) != corner_ids(b) || marker_ids(a) != marker_ids(b)) {
    return ::testing::AssertionFailure() << "other features";
  }
  for (std::size_t i = 0; i < a.observation.points.size(); ++i) {
    if (a.observation.points[i].pixel != b.observation.points[i].pixel) {
      return ::testing::AssertionFailure()
             << "corner " << a.observation.points[i].id << " moved";
    }
  }
  for (std::size_t i = 0; i < a.markers.size(); ++i) {
    if (a.markers[i].corners != b.markers[i].corners) {
      return ::testing::AssertionFailure()
             << "marker " << a.markers[i].id << " moved";
    }
  }
  return ::testing::AssertionSuccess();
}

// Offsets from where the features project: their mean length, the largest,
// and their mean (the signed bias).
struct Errors {
  std::size_t count = 0;
  double mean = 0.0;
  double max = 0.0;
  cv::Point2d bias{0.0, 0.0};

  void add(const camera::Vec2d& found, const cv::Point2d& truth) {
    const cv::Point2d e(found.x - truth.x, found.y - truth.y);
    ++count;
    mean += (std::hypot(e.x, e.y) - mean) / count;
    max = std::max(max, std::hypot(e.x, e.y));
    bias += (e - bias) * (1.0 / count);
  }
  void add_corners(const Detection& detection, const View& view) {
    for (const DetectedPoint& point : detection.observation.points) {
      add(point.pixel, view.corners.at(point.id));
    }
  }
  void add_markers(const Detection& detection, const View& view) {
    for (const DetectedMarker& marker : detection.markers) {
      for (std::size_t k = 0; k < marker.corners.size(); ++k) {
        add(marker.corners[k], view.markers.at(marker.id)[k]);
      }
    }
  }
};

std::ostream& operator<<(std::ostream& out, const Errors& e) {
  return out << e.count << " points: mean " << e.mean << " px, max " << e.max
             << " px, bias (" << e.bias.x << ", " << e.bias.y << ") px";
}

// Views in which both modes should find every corner: front-on, turned 60
// and 70 degrees, and a JPEG at quality 85, each off the pixel grid and
// rolled. Rendered once for the suite.
class FullViews : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    const compute::Board board = femto_board();
    views_ = std::make_unique<std::vector<View>>();
    views_->push_back(
        render("front-on", board,
               board_facing_camera(board, {0.031, -0.017, 2.5}, 0.0, 7.0), 1));
    views_->push_back(
        render("60 deg", board,
               board_facing_camera(board, {-0.12, 0.08, 2.0}, 60.0, -11.0), 2));
    views_->push_back(
        render("70 deg", board,
               board_facing_camera(board, {0.10, 0.05, 2.0}, -70.0, 23.0), 3));
    View compressed =
        render("JPEG q85", board,
               board_facing_camera(board, {0.21, 0.13, 2.7}, 25.0, -4.0), 4);
    std::vector<std::uint8_t> bytes;
    cv::imencode(".jpg", compressed.image, bytes,
                 {cv::IMWRITE_JPEG_QUALITY, 85});
    compressed.image = cv::imdecode(bytes, cv::IMREAD_GRAYSCALE);
    views_->push_back(std::move(compressed));
  }
  static void TearDownTestSuite() { views_.reset(); }
  static const std::vector<View>& views() { return *views_; }

  // Detects each view by `mode`, expecting every corner and marker; returns
  // the corners' errors over all the views.
  static Errors every_corner(Mode mode) {
    const compute::Board board = femto_board();
    Errors errors;
    for (const View& view : views()) {
      const Detection detection = detect(view.image, board, mode);
      EXPECT_EQ(corner_ids(detection),
                (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7}))
          << name(mode) << ", " << view.name;
      EXPECT_EQ(marker_ids(detection), (std::vector<int>{0, 1, 2, 3, 4, 5, 6}))
          << name(mode) << ", " << view.name;
      errors.add_corners(detection, view);
    }
    return errors;
  }

 private:
  static std::unique_ptr<std::vector<View>> views_;
};

std::unique_ptr<std::vector<View>> FullViews::views_;

TEST_F(FullViews, FullModeFindsEveryCornerWithinAFewHundredthsOfAPixel) {
  const Errors errors = every_corner(Mode::Full);
  EXPECT_EQ(errors.count, 8 * views().size());
  EXPECT_LT(errors.mean, 0.05) << errors;
  EXPECT_LT(errors.max, 0.3) << errors;
}

TEST_F(FullViews, HybridModeFindsEveryCornerNearlyAsWell) {
  const Errors errors = every_corner(Mode::Hybrid);
  EXPECT_EQ(errors.count, 8 * views().size());
  EXPECT_LT(errors.mean, 0.06) << errors;
  EXPECT_LT(errors.max, 0.8) << errors;
}

// Neither mode moves the corners off OpenCV's pixel centres.
TEST_F(FullViews, CornersHaveNoSignedBias) {
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    const Errors errors = every_corner(mode);
    EXPECT_LT(std::abs(errors.bias.x), 0.02) << name(mode) << ": " << errors;
    EXPECT_LT(std::abs(errors.bias.y), 0.02) << name(mode) << ": " << errors;
  }
}

// The hybrid finds markers in the image halved and reports them in the
// image: where Full finds them, to within OpenCV's unrefined marker corners.
// A corner refined in the image forgets its seed's half-pixel, so the
// markers are where a lost one shows (about -0.5 px on each axis).
TEST_F(FullViews, BothModesReportMarkersInTheImagesPixels) {
  const compute::Board board = femto_board();
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    Errors errors;
    for (const View& view : views()) {
      errors.add_markers(detect(view.image, board, mode), view);
    }
    EXPECT_EQ(errors.count, 4 * 7 * views().size()) << name(mode);
    EXPECT_LT(errors.mean, mode == Mode::Full ? 1.2 : 2.5)
        << name(mode) << ": " << errors;
    EXPECT_LT(std::abs(errors.bias.x), 0.2) << name(mode) << ": " << errors;
    EXPECT_LT(std::abs(errors.bias.y), 0.2) << name(mode) << ": " << errors;
  }
}

// An odd width or height: the hybrid halves the image's even part, so its
// markers stay on the image's pixels. (Halving the whole image put them
// 0.4-0.6 px off.)
TEST_F(FullViews, HybridHalvesAnOddSizedImage) {
  const compute::Board board = femto_board();
  for (const cv::Size size :
       {cv::Size(3839, 2159), cv::Size(3839, 2160), cv::Size(3840, 2159)}) {
    Errors corners;
    Errors markers;
    for (const View& view : views()) {
      const Detection detection = detect(
          view.image(cv::Rect(cv::Point(0, 0), size)), board, Mode::Hybrid);
      EXPECT_EQ(detection.image_size.width,
                static_cast<std::uint32_t>(size.width));
      EXPECT_EQ(detection.image_size.height,
                static_cast<std::uint32_t>(size.height));
      corners.add_corners(detection, view);
      markers.add_markers(detection, view);
    }
    EXPECT_EQ(corners.count, 8 * views().size()) << size;
    EXPECT_LT(corners.max, 0.8) << size << ": " << corners;
    EXPECT_EQ(markers.count, 4 * 7 * views().size()) << size;
    EXPECT_LT(std::abs(markers.bias.x), 0.2) << size << ": " << markers;
    EXPECT_LT(std::abs(markers.bias.y), 0.2) << size << ": " << markers;
  }
}

TEST_F(FullViews, ReportsTheImageAndFeaturesInIdOrder) {
  const compute::Board board = femto_board();
  const View& view = views().front();
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    const Detection detection = detect(view.image, board, mode);
    EXPECT_EQ(detection.image_size.width, 3840u) << name(mode);
    EXPECT_EQ(detection.image_size.height, 2160u) << name(mode);
    EXPECT_TRUE(detection.observation.camera_id.empty()) << name(mode);
    EXPECT_EQ(detection.observation.frame_id, 0) << name(mode);
    const std::vector<int> ids = corner_ids(detection);
    EXPECT_TRUE(std::is_sorted(ids.begin(), ids.end())) << name(mode);
    for (const DetectedPoint& point : detection.observation.points) {
      EXPECT_EQ(point.weight, 1.0) << name(mode) << ", corner " << point.id;
    }
    const std::vector<int> markers = marker_ids(detection);
    EXPECT_TRUE(std::is_sorted(markers.begin(), markers.end())) << name(mode);
  }
}

// Rows `stride` bytes apart: the padding is never read.
TEST_F(FullViews, ReadsRowsStrideApart) {
  const compute::Board board = femto_board();
  const cv::Mat& packed = views().front().image;
  cv::Mat buffer(packed.rows, packed.cols + 64, CV_8UC1, cv::Scalar(255));
  packed.copyTo(buffer(cv::Rect(0, 0, packed.cols, packed.rows)));
  const cv::Mat padded = buffer(cv::Rect(0, 0, packed.cols, packed.rows));
  ASSERT_EQ(padded.step[0], static_cast<std::size_t>(packed.cols + 64));
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    const Detection a = detect(packed, board, mode);
    EXPECT_EQ(a.observation.size(), 8u) << name(mode);
    EXPECT_TRUE(Same(a, detect(padded, board, mode))) << name(mode);
  }
}

// The live view detects every camera's image at once, with one board: each
// of four threads detects its view five times, and every result is the one
// a single thread gets.
TEST_F(FullViews, DetectsOnSeveralThreadsAtOnce) {
  constexpr int kRounds = 5;
  const compute::Board board = femto_board();
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    std::vector<Detection> alone;
    for (const View& view : views()) {
      alone.push_back(detect(view.image, board, mode));
    }
    std::vector<int> differ(views().size(), 0);
    std::vector<std::thread> threads;
    for (std::size_t i = 0; i < views().size(); ++i) {
      threads.emplace_back([&, i] {
        for (int round = 0; round < kRounds; ++round) {
          const core::Result<Detection> found =
              detect_charuco(luma(views()[i].image), board, mode);
          if (!found.ok() || !Same(*found, alone[i])) ++differ[i];
        }
      });
    }
    for (std::thread& thread : threads) thread.join();
    for (std::size_t i = 0; i < views().size(); ++i) {
      EXPECT_EQ(alone[i].observation.size(), 8u)
          << name(mode) << ", " << views()[i].name;
      EXPECT_EQ(differ[i], 0) << name(mode) << ", " << views()[i].name;
    }
  }
}

// The image's left edge cuts the first column of squares, and with it
// marker 2: the corners beside it go, though corner 0 is in the image, and
// the rest are found.
TEST(DetectCharuco, FindsPartOfABoardTheImageEdgeCuts) {
  const compute::Board board = femto_board();
  const View view =
      render("edge-cut", board,
             board_facing_camera(board, {-1.40, 0.02, 2.0}, 10.0, 0.0), 5);
  ASSERT_LT(view.markers[2][0].x, 0.0);
  ASSERT_GT(view.corners[0].x, 40.0);
  for (const cv::Point2d& p : view.markers[0]) ASSERT_GT(p.x, 40.0);
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    const Detection detection = detect(view.image, board, mode);
    EXPECT_EQ(corner_ids(detection), (std::vector<int>{1, 2, 3, 5, 6, 7}))
        << name(mode);
    EXPECT_EQ(marker_ids(detection), (std::vector<int>{0, 1, 3, 4, 5, 6}))
        << name(mode);
    Errors errors;
    errors.add_corners(detection, view);
    EXPECT_LT(errors.max, mode == Mode::Full ? 0.3 : 0.8)
        << name(mode) << ": " << errors;
  }
}

// The bottom row of squares cropped away, and with it markers 5 and 6: only
// the top row of corners keeps two markers beside it, and one row fixes no
// pose, as compute::usable_for_pose answers.
TEST(DetectCharuco, CollinearCropFindsOneRowOfCorners) {
  const compute::Board board = femto_board();
  const camera::Mat4d pose =
      board_facing_camera(board, {0.031, -0.017, 2.5}, 0.0, 0.0);
  View view = render("cropped", board, pose, 6);
  const double s = board.target().square_size_m;
  const std::vector<cv::Point3d> below = {{-0.1, 2.3 * s, 0.0},
                                          {6.0 * s, 2.3 * s, 0.0},
                                          {6.0 * s, 4.0 * s, 0.0},
                                          {-0.1, 4.0 * s, 0.0}};
  std::vector<cv::Point> polygon;
  for (const cv::Point2d& p :
       test::project(test::color_like_lens(), pose, below)) {
    polygon.emplace_back(static_cast<int>(std::lround(p.x)),
                         static_cast<int>(std::lround(p.y)));
  }
  cv::fillConvexPoly(view.image, polygon, cv::Scalar(110));
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    const Detection detection = detect(view.image, board, mode);
    EXPECT_EQ(corner_ids(detection), (std::vector<int>{0, 1, 2, 3}))
        << name(mode);
    const core::Result<compute::ViewUse> use = compute::usable_for_pose(
        board, detection.observation, view.image.size());
    ASSERT_TRUE(use.ok()) << use.status().message();
    EXPECT_EQ(*use, compute::ViewUse::Collinear) << name(mode);
    Errors errors;
    errors.add_corners(detection, view);
    EXPECT_LT(errors.max, mode == Mode::Full ? 0.3 : 0.8)
        << name(mode) << ": " << errors;
  }
}

TEST(DetectCharuco, FindsNothingInABlankImage) {
  const compute::Board board = femto_board();
  test::CameraOptions optics;
  optics.seed = 7;
  const cv::Mat blank =
      test::camera_image(cv::Mat(2160, 3840, CV_8UC1, 110.0), optics);
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    const Detection detection = detect(blank, board, mode);
    EXPECT_TRUE(detection.observation.empty()) << name(mode);
    EXPECT_TRUE(detection.markers.empty()) << name(mode);
  }
}

// The rig's board sought with another dictionary's markers.
TEST(DetectCharuco, FindsNothingWithTheWrongDictionary) {
  const compute::Board board = femto_board();
  Target other = legacy_femto_board();
  other.dictionary = MarkerDictionary::ArUco5x5_100;
  const compute::Board wrong = compute::Board::create(other).value();
  const View view =
      render("front-on", board,
             board_facing_camera(board, {0.031, -0.017, 2.5}, 0.0, 7.0), 8);
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    EXPECT_EQ(detect(view.image, board, mode).observation.size(), 8u)
        << name(mode);
    const Detection detection = detect(view.image, wrong, mode);
    EXPECT_TRUE(detection.observation.empty()) << name(mode);
    EXPECT_TRUE(detection.markers.empty()) << name(mode);
  }
}

// The rig's board on the image's left, and one of the same dictionary
// numbered from marker 20 on its right: each detection holds its own
// board's markers only.
TEST(DetectCharuco, ReportsOnlyItsBoardsMarkers) {
  const compute::Board board = femto_board();
  Target next = legacy_femto_board();
  next.first_marker_id = 20;
  const compute::Board other = compute::Board::create(next).value();
  const camera::CameraModel model = test::color_like_lens();
  const camera::Mat4d left =
      board_facing_camera(board, {-0.6, 0.0, 2.5}, 0.0, 5.0);
  const camera::Mat4d right =
      board_facing_camera(other, {0.6, 0.0, 2.5}, 0.0, -5.0);
  const double middle = 0.5 * model.size.width;
  for (const cv::Point2d& p :
       test::project(model, left, test::board_outline(board, 0.02, 4))) {
    ASSERT_LT(p.x, middle - 20.0);
  }
  for (const cv::Point2d& p :
       test::project(model, right, test::board_outline(other, 0.02, 4))) {
    ASSERT_GT(p.x, middle + 20.0);
  }
  cv::Mat both = test::render_board(model, right, other);
  const cv::Rect half(0, 0, both.cols / 2, both.rows);
  test::render_board(model, left, board)(half).copyTo(both(half));
  test::CameraOptions optics;
  optics.seed = 10;
  const cv::Mat image = test::camera_image(both, optics);
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    const Detection mine = detect(image, board, mode);
    EXPECT_EQ(corner_ids(mine), (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7}))
        << name(mode);
    EXPECT_EQ(marker_ids(mine), (std::vector<int>{0, 1, 2, 3, 4, 5, 6}))
        << name(mode);
    const Detection theirs = detect(image, other, mode);
    EXPECT_EQ(corner_ids(theirs), (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7}))
        << name(mode);
    EXPECT_EQ(marker_ids(theirs),
              (std::vector<int>{20, 21, 22, 23, 24, 25, 26}))
        << name(mode);
  }
}

// A board 5.5 m away and turned 45 degrees brings marker corners within
// 10 px of its corners. Half the distance keeps them out of the hybrid's
// window; a 10 px half-window, or the whole distance, lets them pull the
// corners (mean 0.06-0.10 px, max 0.11-0.24 px over noise seeds, against
// 0.02-0.03 and 0.03-0.06).
TEST(DetectCharuco, HybridKeepsNearbyMarkerCornersOutOfItsWindow) {
  const compute::Board board = femto_board();
  const View view = render(
      "far", board, board_facing_camera(board, {0.3, 0.2, 5.5}, 45.0, 31.0), 9);
  double nearest = std::numeric_limits<double>::infinity();
  for (const std::vector<cv::Point2d>& marker : view.markers) {
    for (const cv::Point2d& m : marker) {
      for (const cv::Point2d& c : view.corners) {
        nearest = std::min(nearest, cv::norm(m - c));
      }
    }
  }
  ASSERT_LT(nearest, 10.0);
  const Detection detection = detect(view.image, board, Mode::Hybrid);
  EXPECT_EQ(detection.observation.size(), 8u);
  Errors errors;
  errors.add_corners(detection, view);
  EXPECT_LT(errors.mean, 0.045) << errors;
  EXPECT_LT(errors.max, 0.09) << errors;
}

// The hybrid's seed for corner `id`, where the image halved puts it
// (2u + 0.5), and its distance to the nearest marker corner found there:
// twice the hybrid's half-window, before rounding and clamping.
struct Seed {
  cv::Point2f pixel;
  double nearest_marker = std::numeric_limits<double>::infinity();
};

Seed hybrid_seed(const cv::Mat& image, const compute::Board& board, int id) {
  cv::Mat half;
  cv::resize(image, half, cv::Size(image.cols / 2, image.rows / 2), 0.0, 0.0,
             cv::INTER_AREA);
  std::vector<cv::Point2f> corners;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> markers;
  std::vector<int> marker_ids;
  cv::aruco::CharucoDetector(board.charuco())
      .detectBoard(half, corners, ids, markers, marker_ids);
  const auto in_image = [](const cv::Point2f& p) {
    return cv::Point2f(2.0f * p.x + 0.5f, 2.0f * p.y + 0.5f);
  };
  Seed seed;
  const auto found = std::find(ids.begin(), ids.end(), id);
  EXPECT_NE(found, ids.end()) << "corner " << id << " not in the image halved";
  if (found == ids.end()) return seed;
  seed.pixel = in_image(corners[static_cast<std::size_t>(found - ids.begin())]);
  for (const std::vector<cv::Point2f>& marker : markers) {
    for (const cv::Point2f& p : marker) {
      seed.nearest_marker =
          std::min(seed.nearest_marker, cv::norm(in_image(p) - seed.pixel));
    }
  }
  return seed;
}

// Where cornerSubPix takes `seed` in `image` with half-window `half`, at the
// hybrid's criteria.
cv::Point2f refined(const cv::Mat& image, const cv::Point2f& seed, int half) {
  std::vector<cv::Point2f> point = {seed};
  cv::cornerSubPix(
      image, point, cv::Size(half, half), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100,
                       0.01));
  return point[0];
}

// Adds d * (P(x - x0) + P(y - y0)), P = [+1, -1, -1, +1] repeating, to the
// w x w pixels from (x0, y0), both even: a fine pattern every aligned 2 x 2
// block of which sums to zero, so the image halved does not change.
void add_fine_pattern(cv::Mat& image, int x0, int y0, int w, int d) {
  constexpr int kP[4] = {1, -1, -1, 1};
  for (int y = y0; y < y0 + w; ++y) {
    for (int x = x0; x < x0 + w; ++x) {
      std::uint8_t& level = image.at<std::uint8_t>(y, x);
      level = cv::saturate_cast<std::uint8_t>(
          level + d * (kP[(x - x0) % 4] + kP[(y - y0) % 4]));
    }
  }
}

bool same_halved(const cv::Mat& a, const cv::Mat& b) {
  cv::Mat half_a;
  cv::Mat half_b;
  cv::resize(a, half_a, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
  cv::resize(b, half_b, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
  return cv::norm(half_a, half_b, cv::NORM_INF) == 0.0;
}

// The board printed low in contrast (black at 100), so a fine pattern has
// room; no noise, so nothing clips.
test::RenderOptions faded() {
  test::RenderOptions options;
  options.black = 100.0f;
  return options;
}

// A faded board front-on 2.5 m away, with a fine pattern of amplitude `d`
// beside corner 5 in its black square, which the image halved does not see:
// where the hybrid's own refinement takes the corner from its seed.
struct Pulled {
  View view;      // the board, without the pattern
  cv::Mat image;  // with it
  Seed seed;
  cv::Point2f refined;
};

Pulled pulled_corner_5(int d) {
  const compute::Board board = femto_board();
  Pulled pulled;
  pulled.view = render(
      "faded", board,
      board_facing_camera(board, {0.031, -0.017, 2.5}, 0.0, 7.0), 0, faded());
  const cv::Point2d corner = pulled.view.corners[5];
  pulled.image = pulled.view.image.clone();
  add_fine_pattern(
      pulled.image, 2 * static_cast<int>(std::floor((corner.x - 12.0) / 2.0)),
      2 * static_cast<int>(std::floor((corner.y - 12.0) / 2.0)), 8, d);
  pulled.seed = hybrid_seed(pulled.view.image, board, 5);
  // Marker corners 20 px away or more: the hybrid's half-window is 10.
  EXPECT_GE(pulled.seed.nearest_marker, 20.0);
  pulled.refined = refined(pulled.image, pulled.seed.pixel, 10);
  return pulled;
}

std::vector<DetectedPoint> without_corner(std::vector<DetectedPoint> points,
                                          int id) {
  points.erase(
      std::remove_if(points.begin(), points.end(),
                     [id](const DetectedPoint& p) { return p.id == id; }),
      points.end());
  return points;
}

// The pattern pulls corner 5's refinement 1.2-1.8 px from its seed: the
// hybrid keeps the corner where its refinement took it, and the rest as they
// were.
TEST(DetectCharuco, HybridKeepsACornerItsRefinementMovesUnderTwoPixels) {
  const compute::Board board = femto_board();
  const Pulled pulled = pulled_corner_5(30);
  ASSERT_TRUE(same_halved(pulled.view.image, pulled.image));
  const double shift = cv::norm(pulled.refined - pulled.seed.pixel);
  ASSERT_GT(shift, 1.2);
  ASSERT_LT(shift, 1.8);

  Detection clean = detect(pulled.view.image, board, Mode::Hybrid);
  Detection found = detect(pulled.image, board, Mode::Hybrid);
  ASSERT_EQ(corner_ids(found), (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7}));
  EXPECT_NEAR(found.observation.points[5].pixel.x, pulled.refined.x, 1e-3);
  EXPECT_NEAR(found.observation.points[5].pixel.y, pulled.refined.y, 1e-3);
  clean.observation.points = without_corner(clean.observation.points, 5);
  found.observation.points = without_corner(found.observation.points, 5);
  EXPECT_TRUE(Same(found, clean));
}

// The pattern pulls corner 5's refinement 2.2-2.8 px from its seed: the
// hybrid drops it and keeps the rest as they were.
TEST(DetectCharuco, HybridDropsACornerItsRefinementMovesOverTwoPixels) {
  const compute::Board board = femto_board();
  const Pulled pulled = pulled_corner_5(32);
  ASSERT_TRUE(same_halved(pulled.view.image, pulled.image));
  const double shift = cv::norm(pulled.refined - pulled.seed.pixel);
  ASSERT_GT(shift, 2.2);
  ASSERT_LT(shift, 2.8);

  Detection clean = detect(pulled.view.image, board, Mode::Hybrid);
  ASSERT_EQ(corner_ids(clean), (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7}));
  clean.observation.points = without_corner(clean.observation.points, 5);
  EXPECT_TRUE(Same(detect(pulled.image, board, Mode::Hybrid), clean));
}

// 3.6 m away, half corner 5's distance to its nearest marker corner is
// between 8 and 9 px, so its half-window is 8. A fine pattern 9 px below it
// pulls a 9 px half-window's refinement more than 3 px, but an 8 px one's
// less than 1 px: the hybrid keeps the corner.
TEST(DetectCharuco, HybridRoundsItsHalfWindowDown) {
  const compute::Board board = femto_board();
  const View view = render(
      "faded, 3.6 m", board,
      board_facing_camera(board, {0.031, -0.017, 3.6}, 0.0, 7.0), 0, faded());
  const Seed seed = hybrid_seed(view.image, board, 5);
  ASSERT_GT(0.5 * seed.nearest_marker, 8.2);
  ASSERT_LT(0.5 * seed.nearest_marker, 9.0);
  cv::Mat pulled = view.image.clone();
  add_fine_pattern(pulled, 2 * static_cast<int>(std::floor(seed.pixel.x / 2.0)),
                   2 * static_cast<int>(std::floor((seed.pixel.y + 9.0) / 2.0)),
                   8, 45);
  ASSERT_TRUE(same_halved(view.image, pulled));
  ASSERT_GT(cv::norm(refined(pulled, seed.pixel, 9) - seed.pixel), 3.0);
  ASSERT_LT(cv::norm(refined(pulled, seed.pixel, 8) - seed.pixel), 1.0);

  EXPECT_EQ(corner_ids(detect(pulled, board, Mode::Hybrid)),
            (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7}));
}

core::Status refusal(const LumaView& image, Mode mode) {
  const compute::Board board = femto_board();
  core::Result<Detection> detection = Detection{};
  EXPECT_NO_THROW(detection = detect_charuco(image, board, mode));
  return detection.ok() ? core::Status{} : detection.status();
}

TEST(DetectCharuco, RefusesAnImageItCannotRead) {
  const std::vector<std::uint8_t> pixels(64, 110);
  const LumaView good{8, 8, 8, pixels.data()};
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    SCOPED_TRACE(name(mode));
    EXPECT_TRUE(refusal(good, mode).ok());
    LumaView image = good;
    image.data = nullptr;
    EXPECT_TRUE(
        Refused(refusal(image, mode), "detect_charuco: image.data is null"));
    image = good;
    image.stride = 7;
    EXPECT_TRUE(Refused(refusal(image, mode),
                        "detect_charuco: image.stride is below image.width"));
    for (const auto& [width, height] :
         {std::pair<std::uint32_t, std::uint32_t>{0, 8}, {8, 0}, {0, 0}}) {
      image = good;
      image.width = width;
      image.height = height;
      EXPECT_FALSE(refusal(image, mode).ok()) << width << " x " << height;
      EXPECT_EQ(refusal(image, mode).domain(), Code::InvalidArgument);
    }
    // Refused before anything reads the pixels.
    image = good;
    image.width = std::uint32_t{1} << 31;
    image.stride = image.width;
    EXPECT_TRUE(Refused(
        refusal(image, mode),
        "detect_charuco: the image is larger than OpenCV's int sizes hold"));
    image = good;
    image.height = std::uint32_t{1} << 31;
    EXPECT_TRUE(Refused(
        refusal(image, mode),
        "detect_charuco: the image is larger than OpenCV's int sizes hold"));
  }
  // Full takes any pixel; Hybrid halves the image, so needs 2 x 2.
  EXPECT_TRUE(refusal({1, 1, 1, pixels.data()}, Mode::Full).ok());
  EXPECT_TRUE(refusal({2, 2, 2, pixels.data()}, Mode::Hybrid).ok());
  EXPECT_TRUE(refusal({3, 3, 3, pixels.data()}, Mode::Hybrid).ok());
  for (const auto& [width, height] :
       {std::pair<std::uint32_t, std::uint32_t>{1, 8}, {8, 1}, {1, 1}}) {
    EXPECT_TRUE(
        Refused(refusal({width, height, 8, pixels.data()}, Mode::Hybrid),
                "detect_charuco: a Hybrid image is at least 2 x 2 "
                "pixels"))
        << width << " x " << height;
  }
  EXPECT_TRUE(Refused(refusal(good, static_cast<Mode>(2)),
                      "detect_charuco: mode is not a Mode"));
  EXPECT_TRUE(Refused(refusal(good, static_cast<Mode>(-1)),
                      "detect_charuco: mode is not a Mode"));
}

// Full mode: Hybrid's first loop is in the resize, inside OpenCV's KleidiCV
// HAL on Arm, whose C code an exception cannot cross. Both modes run one
// guarded body.
TEST(DetectCharuco, ReturnsWhatOpenCVThrowsAsAStatus) {
  const compute::Board board = femto_board();
  const cv::Mat image(2160, 3840, CV_8UC1, cv::Scalar(110));
  core::Result<Detection> detection = Detection{};
  {
    const ThrowingParallelFor throwing(cv::Error::StsNotImplemented);
    EXPECT_NO_THROW(detection = detect_charuco(luma(image), board, Mode::Full));
  }
  ASSERT_FALSE(detection.ok());
  EXPECT_EQ(detection.status().domain(), Code::Backend);
  EXPECT_EQ(detection.status().backend(), core::Status::Backend::Other);
  EXPECT_EQ(detection.status().detail(), cv::Error::StsNotImplemented);
  EXPECT_EQ(detection.status().message().rfind("detect_charuco: ", 0), 0u)
      << detection.status().message();
  EXPECT_NE(detection.status().message().find("injected"), std::string::npos)
      << detection.status().message();
  {
    const ThrowingParallelFor throwing(cv::Error::StsNoMem);
    EXPECT_NO_THROW(detection = detect_charuco(luma(image), board, Mode::Full));
  }
  ASSERT_FALSE(detection.ok());
  EXPECT_EQ(detection.status().domain(), Code::OutOfMemory);
  // OpenCV's own backend is back.
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    EXPECT_TRUE(detect_charuco(luma(image), board, mode).ok()) << name(mode);
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::detect
