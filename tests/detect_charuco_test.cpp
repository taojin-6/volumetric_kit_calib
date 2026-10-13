// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The ChArUco board in a luma image: both modes against where the synthetic
// renderer's board projects (front-on, turned 60 and 70 degrees, a JPEG), on
// several threads, part of a board, a collinear crop, no board, the hybrid's
// window and corner rejection, the images it refuses, what OpenCV throws, and
// the usable-view check.

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
#include <opencv2/core/parallel/parallel_backend.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "synthetic_rig.hpp"

namespace volumetric_kit::calib::detect {
namespace {

namespace camera = recon::camera;
using Code = core::Status::Code;

constexpr double kPi = 3.14159265358979323846;

compute::Board femto_board() {
  return compute::Board::create(legacy_femto_board()).value();
}

// The board's centre at `centre` in the camera, its printed face turned
// `tilt_deg` about its x axis from facing the camera, then rolled `roll_deg`
// about the camera's axis.
camera::Mat4d facing(const compute::Board& board, const camera::Vec3d& centre,
                     double tilt_deg, double roll_deg) {
  const camera::Mat3d rotation =
      camera::rotation_from_rodrigues({0.0, 0.0, roll_deg * kPi / 180.0}) *
      camera::rotation_from_rodrigues({tilt_deg * kPi / 180.0, 0.0, 0.0});
  const cv::Point3f corner = board.charuco().getRightBottomCorner();
  const camera::Vec3d middle(0.5 * corner.x, 0.5 * corner.y, 0.0);
  camera::Mat4d m(rotation);
  m[3] = glm::dvec4(centre - rotation * middle, 1.0);
  return m;
}

// A camera's picture of a render: 0.8 px of optical blur and 2.5 DN of noise.
cv::Mat camera_image(const cv::Mat& render, std::uint64_t seed) {
  cv::Mat blurred;
  cv::GaussianBlur(render, blurred, cv::Size(), 0.8);
  cv::Mat noise(render.size(), CV_32F);
  cv::RNG(seed).fill(noise, cv::RNG::NORMAL, 0.0, 2.5);
  cv::Mat level;
  blurred.convertTo(level, CV_32F);
  cv::Mat out;
  cv::Mat(level + noise).convertTo(out, CV_8U);
  return out;
}

LumaView luma(const cv::Mat& image) {
  return {static_cast<std::uint32_t>(image.cols),
          static_cast<std::uint32_t>(image.rows), image.step[0], image.data};
}

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
  const cv::Mat drawn =
      test::render_board(model, board_to_camera, board, options);
  if (seed != 0) {
    view.image = camera_image(drawn, seed);
  } else {
    cv::GaussianBlur(drawn, view.image, cv::Size(), 0.8);
  }
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

Usability usability(const Detection& detection, const compute::Board& board) {
  core::Result<Usability> use = core::Status::invalid_argument("not run");
  EXPECT_NO_THROW(use = usable_for_pose(detection, board));
  EXPECT_TRUE(use.ok()) << use.status().message();
  return use.ok() ? *use : Usability::TooFewCorners;
}

// Views in which both modes should find every corner: front-on, turned 60
// and 70 degrees, and a JPEG at quality 85, each off the pixel grid and
// rolled. Rendered once for the suite.
class FullViews : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    const compute::Board board = femto_board();
    views_ = std::make_unique<std::vector<View>>();
    views_->push_back(render("front-on", board,
                             facing(board, {0.031, -0.017, 2.5}, 0.0, 7.0), 1));
    views_->push_back(render(
        "60 deg", board, facing(board, {-0.12, 0.08, 2.0}, 60.0, -11.0), 2));
    views_->push_back(render("70 deg", board,
                             facing(board, {0.10, 0.05, 2.0}, -70.0, 23.0), 3));
    View compressed = render("JPEG q85", board,
                             facing(board, {0.21, 0.13, 2.7}, 25.0, -4.0), 4);
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

TEST_F(FullViews, EveryViewIsUsable) {
  const compute::Board board = femto_board();
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    for (const View& view : views()) {
      EXPECT_EQ(usability(detect(view.image, board, mode), board),
                Usability::Usable)
          << name(mode) << ", " << view.name;
    }
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
// the rest are found and usable.
TEST(DetectCharuco, FindsPartOfABoardTheImageEdgeCuts) {
  const compute::Board board = femto_board();
  const View view = render("edge-cut", board,
                           facing(board, {-1.40, 0.02, 2.0}, 10.0, 0.0), 5);
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
    EXPECT_EQ(usability(detection, board), Usability::Usable) << name(mode);
  }
}

// The bottom row of squares cropped away, and with it markers 5 and 6: only
// the top row of corners keeps two markers beside it, and one row fixes no
// pose.
TEST(DetectCharuco, CollinearCropIsNotUsable) {
  const compute::Board board = femto_board();
  const camera::Mat4d pose = facing(board, {0.031, -0.017, 2.5}, 0.0, 0.0);
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
    Errors errors;
    errors.add_corners(detection, view);
    EXPECT_LT(errors.max, mode == Mode::Full ? 0.3 : 0.8)
        << name(mode) << ": " << errors;
    EXPECT_EQ(usability(detection, board), Usability::Collinear) << name(mode);
  }
}

TEST(DetectCharuco, FindsNothingInABlankImage) {
  const compute::Board board = femto_board();
  const cv::Mat blank = camera_image(cv::Mat(2160, 3840, CV_8UC1, 110.0), 7);
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    const Detection detection = detect(blank, board, mode);
    EXPECT_TRUE(detection.observation.empty()) << name(mode);
    EXPECT_TRUE(detection.markers.empty()) << name(mode);
    EXPECT_EQ(usability(detection, board), Usability::TooFewCorners)
        << name(mode);
  }
}

// The rig's board sought with another dictionary's markers.
TEST(DetectCharuco, FindsNothingWithTheWrongDictionary) {
  const compute::Board board = femto_board();
  Target other = legacy_femto_board();
  other.dictionary = MarkerDictionary::ArUco5x5_100;
  const compute::Board wrong = compute::Board::create(other).value();
  const View view = render("front-on", board,
                           facing(board, {0.031, -0.017, 2.5}, 0.0, 7.0), 8);
  for (const Mode mode : {Mode::Full, Mode::Hybrid}) {
    EXPECT_EQ(detect(view.image, board, mode).observation.size(), 8u)
        << name(mode);
    const Detection detection = detect(view.image, wrong, mode);
    EXPECT_TRUE(detection.observation.empty()) << name(mode);
    EXPECT_TRUE(detection.markers.empty()) << name(mode);
  }
}

// A board 5.5 m away and turned 45 degrees brings marker corners within
// 10 px of its corners. Half the distance keeps them out of the hybrid's
// window; a 10 px half-window, or the whole distance, lets them pull the
// corners (mean 0.06-0.10 px, max 0.11-0.24 px over noise seeds, against
// 0.02-0.03 and 0.03-0.06).
TEST(DetectCharuco, HybridKeepsNearbyMarkerCornersOutOfItsWindow) {
  const compute::Board board = femto_board();
  const View view =
      render("far", board, facing(board, {0.3, 0.2, 5.5}, 45.0, 31.0), 9);
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

// A pattern too fine for the image halved, beside corner 5 in its black
// square, pulls that corner's refinement in the image more than 2 px: the
// hybrid drops it and keeps the rest as they were. The board is printed low
// in contrast (black at 100) so the pattern has room; no noise, so nothing
// clips.
TEST(DetectCharuco, HybridDropsACornerItsRefinementMovesMoreThanTwoPixels) {
  const compute::Board board = femto_board();
  test::RenderOptions faded;
  faded.black = 100.0f;
  const View view = render(
      "faded", board, facing(board, {0.031, -0.017, 2.5}, 0.0, 7.0), 0, faded);
  const cv::Point2d corner = view.corners[5];
  cv::Mat pulled = view.image.clone();
  const int x0 = 2 * static_cast<int>(std::floor((corner.x - 12.0) / 2.0));
  const int y0 = 2 * static_cast<int>(std::floor((corner.y - 12.0) / 2.0));
  add_fine_pattern(pulled, x0, y0, 8, 40);

  // The premise: the image halved is the same, and the pattern pulls the
  // corner well inside the 2-10 px the hybrid drops (its window is 21 x 21
  // here).
  cv::Mat half, pulled_half;
  cv::resize(view.image, half, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
  cv::resize(pulled, pulled_half, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
  ASSERT_EQ(cv::norm(half, pulled_half, cv::NORM_INF), 0.0);
  std::vector<cv::Point2f> refined = {cv::Point2f(corner)};
  cv::cornerSubPix(
      pulled, refined, cv::Size(10, 10), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100,
                       0.01));
  const double pull = cv::norm(cv::Point2d(refined[0]) - corner);
  ASSERT_GT(pull, 3.0);
  ASSERT_LT(pull, 9.0);

  Detection clean = detect(view.image, board, Mode::Hybrid);
  ASSERT_EQ(corner_ids(clean), (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7}));
  std::vector<DetectedPoint>& points = clean.observation.points;
  points.erase(points.begin() + 5);
  EXPECT_TRUE(Same(detect(pulled, board, Mode::Hybrid), clean));
}

core::Status refusal(const LumaView& image, Mode mode) {
  const compute::Board board = femto_board();
  core::Result<Detection> detection = Detection{};
  EXPECT_NO_THROW(detection = detect_charuco(image, board, mode));
  return detection.ok() ? core::Status{} : detection.status();
}

::testing::AssertionResult Refused(const core::Status& status,
                                   const std::string& message) {
  if (status.ok()) return ::testing::AssertionFailure() << "accepted";
  if (status.domain() != Code::InvalidArgument) {
    return ::testing::AssertionFailure()
           << "refused as " << core::to_string(status.domain()) << ": "
           << status.message();
  }
  if (status.message() != message) {
    return ::testing::AssertionFailure() << "refused with " << status.message();
  }
  return ::testing::AssertionSuccess();
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

// OpenCV's parallel_for_ backend replaced, while it lives, by one that throws
// `code` instead of running the loop; OpenCV's detector runs loops on it.
class ThrowingParallelFor {
 public:
  explicit ThrowingParallelFor(cv::Error::Code code)
      : threads_(cv::getNumThreads()) {
    cv::parallel::setParallelForBackend(std::make_shared<Backend>(code), false);
    // parallel_for_ calls a backend only with more than one thread.
    cv::setNumThreads(4);
  }
  ~ThrowingParallelFor() {
    cv::parallel::setParallelForBackend(
        std::shared_ptr<cv::parallel::ParallelForAPI>(), false);
    cv::setNumThreads(threads_);
  }
  ThrowingParallelFor(const ThrowingParallelFor&) = delete;
  ThrowingParallelFor& operator=(const ThrowingParallelFor&) = delete;

 private:
  class Backend : public cv::parallel::ParallelForAPI {
   public:
    explicit Backend(cv::Error::Code code) : code_(code) {}
    void parallel_for(int, FN_parallel_for_body_cb_t, void*) override {
      throw cv::Exception(code_, "injected", "parallel_for", __FILE__,
                          __LINE__);
    }
    int getThreadNum() const override { return 0; }
    int getNumThreads() const override { return 4; }
    int setNumThreads(int) override { return 4; }
    const char* getName() const override { return "throwing"; }

   private:
    cv::Error::Code code_;
  };

  int threads_;
};

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

// A detection of `ids` on a 3840 x 2160 image, corner i at the pixel of a
// 100 px grid from (1000, 800).
Detection grid(const std::vector<int>& ids) {
  Detection detection;
  detection.image_size = {3840, 2160};
  for (const int id : ids) {
    detection.observation.points.push_back(
        {id, {1000.0 + 100.0 * (id % 4), 800.0 + 100.0 * (id / 4)}, 1.0});
  }
  return detection;
}

TEST(UsableForPose, NeedsFourCorners) {
  const compute::Board board = femto_board();
  EXPECT_EQ(usability(grid({}), board), Usability::TooFewCorners);
  EXPECT_EQ(usability(grid({0, 1, 5}), board), Usability::TooFewCorners);
  EXPECT_EQ(usability(grid({0, 1, 5, 6}), board), Usability::Usable);
  EXPECT_EQ(usability(grid({0, 1, 2, 3, 4, 5, 6, 7}), board),
            Usability::Usable);
}

TEST(UsableForPose, RefusesCornersOnOneLine) {
  const compute::Board board = femto_board();
  EXPECT_EQ(usability(grid({0, 1, 2, 3}), board), Usability::Collinear);
  EXPECT_EQ(usability(grid({4, 5, 6, 7}), board), Usability::Collinear);
  EXPECT_EQ(usability(grid({0, 1, 2, 3, 7}), board), Usability::Usable);
}

// 2 <= x <= width - 3 and 2 <= y <= height - 3, and finite.
TEST(UsableForPose, NeedsEveryCornerTwoPixelsInside) {
  const compute::Board board = femto_board();
  const auto moved = [&](double x, double y) {
    Detection detection = grid({0, 1, 5, 6});
    detection.observation.points[2].pixel = {x, y};
    return usability(detection, board);
  };
  EXPECT_EQ(moved(2.0, 2.0), Usability::Usable);
  EXPECT_EQ(moved(3837.0, 2157.0), Usability::Usable);
  EXPECT_EQ(moved(1.999, 500.0), Usability::NearEdge);
  EXPECT_EQ(moved(500.0, 1.999), Usability::NearEdge);
  EXPECT_EQ(moved(3837.001, 500.0), Usability::NearEdge);
  EXPECT_EQ(moved(500.0, 2157.001), Usability::NearEdge);
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(moved(kNaN, 500.0), Usability::NearEdge);
  EXPECT_EQ(moved(500.0, kNaN), Usability::NearEdge);
  EXPECT_EQ(moved(kInf, 500.0), Usability::NearEdge);
  EXPECT_EQ(moved(500.0, -kInf), Usability::NearEdge);
}

// Too few, then collinear, then near the edge.
TEST(UsableForPose, AnswersTheFirstCheckThatFails) {
  const compute::Board board = femto_board();
  EXPECT_EQ(usability(grid({0, 1, 2}), board), Usability::TooFewCorners);
  Detection few = grid({0, 1, 5});
  few.observation.points[0].pixel.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(usability(few, board), Usability::TooFewCorners);
  Detection line = grid({0, 1, 2, 3});
  line.observation.points[0].pixel.x = 0.0;
  EXPECT_EQ(usability(line, board), Usability::Collinear);
}

TEST(UsableForPose, RefusesCornersNotOnTheBoard) {
  const compute::Board board = femto_board();
  const auto refused = [&](const Detection& detection) {
    core::Result<Usability> use = Usability::Usable;
    EXPECT_NO_THROW(use = usable_for_pose(detection, board));
    return use.ok() ? core::Status{} : use.status();
  };
  EXPECT_TRUE(Refused(refused(grid({0, 1, 5, -1})),
                      "usable_for_pose: corner id -1 is not the board's"));
  EXPECT_TRUE(Refused(refused(grid({0, 1, 5, 8})),
                      "usable_for_pose: corner id 8 is not the board's"));
  EXPECT_TRUE(Refused(refused(grid({0, 1, 5, 1})),
                      "usable_for_pose: corner id 1 appears twice"));
  EXPECT_TRUE(Refused(refused(grid({0, 0, 0, 0})),
                      "usable_for_pose: corner id 0 appears twice"));
}

}  // namespace
}  // namespace volumetric_kit::calib::detect
