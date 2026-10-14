// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The live checks: a view's counts, pose measures, cell and verdict against
// known poses, and how far corners move between two sets of detections,
// which gates stillness and flags a capture's live corners.

#include "volumetric_kit/calib/quality/live_checks.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/detect/charuco.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;
using Code = core::Status::Code;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

const std::vector<int> kAll = {0, 1, 2, 3, 4, 5, 6, 7};

compute::CvCamera cv_of(const camera::CameraModel& model) {
  return compute::cv_camera(model).value();
}

// What a detector finds of `ids` where `model` images the board at
// `board_to_camera`, with seven markers.
Detection detection_of(const camera::CameraModel& model,
                       const compute::Board& board,
                       const camera::Mat4d& board_to_camera,
                       const std::vector<int>& ids) {
  const std::vector<cv::Point2d> pixels =
      test::project(model, board_to_camera, board.object_points());
  Detection detection;
  detection.image_size = model.size;
  for (const int id : ids) {
    const cv::Point2d& p = pixels[static_cast<std::size_t>(id)];
    detection.observation.points.push_back({id, {p.x, p.y}, 1.0});
  }
  detection.markers.resize(7);
  return detection;
}

CheckedView check(const Detection& detection,
                  const ViewCheckOptions& options = {}) {
  const camera::CameraModel model = test::color_like_lens();
  core::Result<CheckedView> view = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(
      view = check_view(cv_of(model), test::femto_board(), detection, options));
  EXPECT_TRUE(view.ok()) << view.status().message();
  return view.ok() ? *view : CheckedView{};
}

// The board in front of the colour lens: its centre at `centre`, turned
// `tilt_deg` from facing the camera.
CheckedView check_at(const camera::Vec3d& centre, double tilt_deg,
                     const std::vector<int>& ids = kAll,
                     const ViewCheckOptions& options = {}) {
  const compute::Board board = test::femto_board();
  return check(
      detection_of(test::color_like_lens(), board,
                   test::board_facing_camera(board, centre, tilt_deg, 10.0),
                   ids),
      options);
}

TEST(CheckView, MeasuresDistanceAndTiltOfAKnownPose) {
  const compute::Board board = test::femto_board();
  const camera::CameraModel model = test::color_like_lens();
  for (const camera::Vec3d& centre :
       {camera::Vec3d(0.0, 0.0, 2.5), camera::Vec3d(0.6, -0.4, 3.2)}) {
    const camera::Mat4d truth =
        test::board_facing_camera(board, centre, 35.0, 10.0);
    const CheckedView view =
        check(detection_of(model, board, truth, {0, 1, 2, 3, 4, 5, 6, 7}));
    ASSERT_TRUE(view.board_to_camera.has_value());
    EXPECT_NEAR(compute::rotation_angle_deg(
                    camera::Mat3d(*view.board_to_camera), camera::Mat3d(truth)),
                0.0, 1e-6);
    ASSERT_TRUE(view.check.distance_m && view.check.tilt_deg &&
                view.check.pnp_rms_px);
    EXPECT_NEAR(*view.check.distance_m, glm::length(centre), 1e-7);
    // The printed face's normal against the line back to the camera.
    EXPECT_NEAR(*view.check.tilt_deg,
                compute::angle_deg(-camera::Vec3d(truth[2]), -centre), 1e-6);
    EXPECT_LT(*view.check.pnp_rms_px, 1e-6);
    EXPECT_EQ(view.check.corners, 8);
    EXPECT_EQ(view.check.markers, 7);
    EXPECT_FALSE(view.check.collinear);
    EXPECT_EQ(view.check.verdict, ViewVerdict::Good);
  }
  // On the axis, the face's turn from the camera is the tilt.
  EXPECT_NEAR(*check_at({0.0, 0.0, 2.5}, 35.0).check.tilt_deg, 35.0, 1e-6);
}

// The cell of a 3 x 3 grid that holds the board centre's projection, row by
// row from the top left; none when it falls outside the image.
TEST(CheckView, PutsTheBoardsCentreInItsCell) {
  const camera::CameraModel model = test::color_like_lens();
  const compute::CvCamera cam = cv_of(model);
  const double w = model.size.width;
  const double h = model.size.height;
  // The board's centre 2.5 m along the ray of `pixel`.
  const auto centre_at = [&](double u, double v) {
    const cv::Point2d ray =
        compute::undistort_points(cam, {cv::Point2d(u, v)})->front();
    return camera::Vec3d(ray.x, ray.y, 1.0) * 2.5;
  };
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const CheckedView view = check_at(
          centre_at((col + 0.5) * w / 3.0, (row + 0.5) * h / 3.0), 20.0);
      EXPECT_EQ(view.check.cell, row * 3 + col) << row << ", " << col;
    }
  }
  // The image's edge is at -0.5, so a cell's is half a pixel before its
  // first pixel centre.
  EXPECT_EQ(check_at(centre_at(w / 3.0 - 0.75, h / 2.0), 20.0).check.cell, 3);
  EXPECT_EQ(check_at(centre_at(w / 3.0 - 0.25, h / 2.0), 20.0).check.cell, 4);
  // The centre left of the image, four corners in it: posed, with no cell.
  const compute::Board board = test::femto_board();
  const cv::Point2d left =
      compute::undistort_points(cam, {cv::Point2d(-60.0, h / 2.0)})->front();
  const camera::Mat4d at = test::board_facing_camera(
      board, camera::Vec3d(left.x, left.y, 1.0) * 2.5, 0.0, 0.0);
  const std::vector<cv::Point2d> pixels =
      test::project(model, at, board.object_points());
  std::vector<int> inside;
  for (int id = 0; id < 8; ++id) {
    if (pixels[static_cast<std::size_t>(id)].x > 2.0) inside.push_back(id);
  }
  ASSERT_EQ(inside, (std::vector<int>{2, 3, 6, 7}));
  const CheckedView view = check(detection_of(model, board, at, inside));
  EXPECT_EQ(view.check.verdict, ViewVerdict::Partial);
  EXPECT_FALSE(view.check.cell.has_value());
  EXPECT_TRUE(view.check.distance_m.has_value());
}

TEST(CheckView, GivesEachVerdictByItsRule) {
  const camera::Vec3d near(0.0, 0.0, 2.5);
  EXPECT_EQ(check_at(near, 35.0).check.verdict, ViewVerdict::Good);
  // Some of the corners.
  const CheckedView partial = check_at(near, 35.0, {0, 1, 2, 4, 5, 6});
  EXPECT_EQ(partial.check.verdict, ViewVerdict::Partial);
  EXPECT_EQ(partial.check.corners, 6);
  // One row: no pose, and nothing measured from one.
  const CheckedView row = check_at(near, 35.0, {0, 1, 2, 3});
  EXPECT_EQ(row.check.verdict, ViewVerdict::Collinear);
  EXPECT_TRUE(row.check.collinear);
  EXPECT_FALSE(row.board_to_camera || row.check.distance_m ||
               row.check.tilt_deg || row.check.pnp_rms_px || row.check.cell);
  // Too few corners, or none.
  const CheckedView few = check_at(near, 35.0, {0, 1, 5});
  EXPECT_EQ(few.check.verdict, ViewVerdict::None);
  EXPECT_FALSE(few.check.collinear || few.board_to_camera);
  EXPECT_EQ(check_at(near, 35.0, {}).check.verdict, ViewVerdict::None);
  EXPECT_EQ(check_at(near, 35.0, {}).check.corners, 0);
  // Oblique above 60 degrees, far beyond 4.5 m; oblique first.
  EXPECT_EQ(check_at(near, 59.0).check.verdict, ViewVerdict::Good);
  EXPECT_EQ(check_at(near, 61.0).check.verdict, ViewVerdict::Oblique);
  EXPECT_EQ(check_at({0.0, 0.0, 4.4}, 35.0).check.verdict, ViewVerdict::Good);
  EXPECT_EQ(check_at({0.0, 0.0, 4.6}, 35.0).check.verdict, ViewVerdict::Far);
  EXPECT_EQ(check_at({0.0, 0.0, 4.6}, 61.0).check.verdict,
            ViewVerdict::Oblique);
  EXPECT_EQ(check_at({0.0, 0.0, 4.6}, 35.0, {0, 1, 2, 4, 5, 6}).check.verdict,
            ViewVerdict::Far);
  // The thresholds are the options'.
  ViewCheckOptions options;
  options.oblique_deg = 50.0;
  options.far_m = 2.0;
  EXPECT_EQ(check_at(near, 55.0, kAll, options).check.verdict,
            ViewVerdict::Oblique);
  EXPECT_EQ(check_at(near, 35.0, kAll, options).check.verdict,
            ViewVerdict::Far);
}

// The rig's board fixes its rotation less than half as well facing the
// camera as at 45 degrees: frontal below 20 degrees.
TEST(CheckView, CallsAViewFrontalBelowItsTilt) {
  const camera::Vec3d near(0.0, 0.0, 2.5);
  EXPECT_TRUE(check_at(near, 19.0).frontal);
  EXPECT_FALSE(check_at(near, 21.0).frontal);
  EXPECT_EQ(check_at(near, 19.0).check.verdict, ViewVerdict::Good);
  ViewCheckOptions options;
  options.frontal_deg = 30.0;
  EXPECT_TRUE(check_at(near, 25.0, kAll, options).frontal);
  // Unposed views are not frontal.
  EXPECT_FALSE(check_at(near, 0.0, {0, 1, 2, 3}).frontal);
}

// Corners seen through a pinhole from a board that straddles the camera:
// usable, but IPPE has no pose with every corner in front.
TEST(CheckView, AViewWithNoPoseIsOfNoUse) {
  const compute::Board board = test::femto_board();
  const camera::CameraModel pinhole{
      {3840, 2160}, {2240.0, 2240.0, 1919.5, 1079.5}, {}};
  camera::Mat4d along_minus_z(0.0);
  along_minus_z[0] = glm::dvec4(0.0, 0.0, -1.0, 0.0);
  along_minus_z[1] = glm::dvec4(1.0, 0.0, 0.0, 0.0);
  along_minus_z[2] = glm::dvec4(0.0, -1.0, 0.0, 0.0);
  along_minus_z[3] = glm::dvec4(-0.225, 0.05, 0.3, 1.0);
  const Detection detection =
      detection_of(pinhole, board, along_minus_z, {0, 3, 4, 7});
  core::Result<CheckedView> view = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(view = check_view(cv_of(pinhole), board, detection));
  ASSERT_TRUE(view.ok()) << view.status().message();
  EXPECT_EQ(view->check.verdict, ViewVerdict::None);
  EXPECT_EQ(view->check.corners, 4);
  EXPECT_FALSE(view->board_to_camera || view->check.distance_m);
}

TEST(CheckView, RefusesWhatItCannotCheck) {
  const compute::Board board = test::femto_board();
  const camera::CameraModel model = test::color_like_lens();
  const Detection good = detection_of(
      model, board, test::board_facing_camera(board, {0, 0, 2.5}, 35.0, 0.0),
      kAll);
  const auto refused = [&](const Detection& detection,
                           const ViewCheckOptions& options) {
    core::Result<CheckedView> view = CheckedView{};
    EXPECT_NO_THROW(view = check_view(cv_of(model), board, detection, options));
    EXPECT_EQ(view.status().domain(), Code::InvalidArgument);
    return view.status().message();
  };
  Detection smaller = good;
  smaller.image_size = {1920, 1080};
  EXPECT_EQ(refused(smaller, {}),
            "check_view: the detection is of a 1920x1080 image, the camera's "
            "is 3840x2160");
  for (const auto& [field, value] :
       {std::pair{&ViewCheckOptions::oblique_deg, 0.0},
        std::pair{&ViewCheckOptions::oblique_deg, 181.0},
        std::pair{&ViewCheckOptions::far_m, 0.0},
        std::pair{&ViewCheckOptions::far_m, kNaN},
        std::pair{&ViewCheckOptions::frontal_deg, -1.0},
        std::pair{&ViewCheckOptions::frontal_deg, kNaN}}) {
    ViewCheckOptions options;
    options.*field = value;
    EXPECT_NE(refused(good, options).find("check_view: options."),
              std::string::npos);
  }
  // A corner not finite is no use, not a failure.
  Detection nan = good;
  nan.observation.points[2].pixel.x = kNaN;
  EXPECT_EQ(check(nan).check.verdict, ViewVerdict::None);
}

Detection corners(std::vector<std::pair<int, camera::Vec2d>> points) {
  Detection detection;
  detection.image_size = {3840, 2160};
  for (const auto& [id, pixel] : points) {
    detection.observation.points.push_back({id, pixel, 1.0});
  }
  return detection;
}

CornerShift shift_of(const std::vector<Detection>& before,
                     const std::vector<Detection>& after) {
  core::Result<CornerShift> shift = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(shift = corner_shift(before, after));
  EXPECT_TRUE(shift.ok()) << shift.status().message();
  return shift.ok() ? *shift : CornerShift{};
}

// Corners matched by id, whatever their order; a corner in one set only
// counts for nothing.
TEST(CornerShift, MatchesCornersById) {
  const std::vector<Detection> before = {
      corners({{3, {10, 10}}, {0, {100, 100}}, {1, {200, 100}}, {5, {7, 7}}}),
      corners({{2, {50, 50}}, {4, {60, 60}}}),
      corners({{1, {1, 1}}}),
  };
  const std::vector<Detection> after = {
      corners({{1, {200.3, 100.4}}, {0, {100.0, 100.1}}, {3, {13, 14}}}),
      corners({{4, {60.6, 60.8}}, {2, {50, 50.2}}, {6, {0, 0}}}),
      corners({}),
  };
  const CornerShift shift = shift_of(before, after);
  ASSERT_EQ(shift.cameras.size(), 3u);
  EXPECT_NEAR(shift.cameras[0].value(), 0.5, 1e-12);  // of 0.1, 0.5, 5
  EXPECT_NEAR(shift.cameras[1].value(), 0.6, 1e-12);  // of 0.2, 1.0
  EXPECT_FALSE(shift.cameras[2].has_value());
  EXPECT_NEAR(shift.median_px.value(), 0.5, 1e-12);  // of 0.1, 0.2, 0.5, 1, 5
  EXPECT_NEAR(shift.max_px.value(), 5.0, 1e-12);
}

TEST(CornerShift, IsEmptyWithNoCornerInBoth) {
  const CornerShift none =
      shift_of({corners({{0, {1, 1}}}), Detection{}},
               {corners({{1, {1, 1}}}), corners({{0, {1, 1}}})});
  EXPECT_EQ(none.cameras,
            (std::vector<std::optional<double>>{std::nullopt, std::nullopt}));
  EXPECT_FALSE(none.median_px || none.max_px);
  EXPECT_TRUE(shift_of({}, {}).cameras.empty());
}

TEST(CornerShift, RefusesDetectionsThatDoNotPair) {
  const auto refused = [](const std::vector<Detection>& before,
                          const std::vector<Detection>& after) {
    core::Result<CornerShift> shift = CornerShift{};
    EXPECT_NO_THROW(shift = corner_shift(before, after));
    EXPECT_EQ(shift.status().domain(), Code::InvalidArgument);
    return shift.status().message();
  };
  const Detection one = corners({{0, {1, 1}}});
  EXPECT_EQ(refused({one}, {one, one}),
            "corner_shift: 1 detections before, 2 after");
  EXPECT_EQ(refused({one, one}, {one, corners({{0, {kNaN, 1}}})}),
            "corner_shift: camera 1's later corner 0 is not finite");
  EXPECT_EQ(refused({corners({{0, {1, 1}}, {0, {2, 2}}})}, {one}),
            "corner_shift: camera 0's earlier detection repeats corner 0");
  Detection smaller = one;
  smaller.image_size = {1920, 1080};
  EXPECT_EQ(refused({one}, {smaller}),
            "corner_shift: camera 0's detections are of images of different "
            "sizes");
  // A camera without corners may have no image.
  EXPECT_TRUE(corner_shift({Detection{}}, {one}).ok());
}

TEST(LiveVsFull, FlagsAMedianOrAMaxAboveItsLimit) {
  CornerShift shift;
  EXPECT_FALSE(live_vs_full_flagged(shift));
  shift.median_px = 0.3;
  shift.max_px = 1.0;
  EXPECT_FALSE(live_vs_full_flagged(shift));
  shift.median_px = 0.31;
  EXPECT_TRUE(live_vs_full_flagged(shift));
  shift.median_px = 0.1;
  shift.max_px = 1.01;
  EXPECT_TRUE(live_vs_full_flagged(shift));
  LiveVsFullOptions options;
  options.max_px = 1.5;
  EXPECT_FALSE(live_vs_full_flagged(shift, options));
  options.median_px = 0.05;
  EXPECT_TRUE(live_vs_full_flagged(shift, options));
}

// A camera's picture of the board, detected live (hybrid) and in full: the
// two agree, unless a live corner is off by more than a pixel.
TEST(LiveVsFull, ARenderedViewAgreesUntilACornerIsOff) {
  const compute::Board board = test::femto_board();
  const camera::CameraModel model = test::color_like_lens();
  const camera::Mat4d at =
      test::board_facing_camera(board, {0.2, 0.1, 2.6}, 40.0, 15.0);
  const cv::Mat image =
      test::camera_image(test::render_board(model, at, board));
  const detect::LumaView luma{static_cast<std::uint32_t>(image.cols),
                              static_cast<std::uint32_t>(image.rows),
                              image.step[0], image.data};
  Detection live =
      detect::detect_charuco(luma, board, detect::Mode::Hybrid).value();
  const Detection full =
      detect::detect_charuco(luma, board, detect::Mode::Full).value();
  ASSERT_EQ(full.observation.points.size(), 8u);
  EXPECT_EQ(check(full).check.verdict, ViewVerdict::Good);

  const CornerShift agree = shift_of({live}, {full});
  ASSERT_TRUE(agree.median_px.has_value());
  EXPECT_LT(*agree.max_px, 0.3);
  EXPECT_FALSE(live_vs_full_flagged(agree));
  live.observation.points[3].pixel.x += 1.2;
  EXPECT_TRUE(live_vs_full_flagged(shift_of({live}, {full})));
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
