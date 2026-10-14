// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// One view's own pose: IPPE's two solutions through the whole lens, the better
// first, refused when they put a corner behind the camera; and the board's
// tilt.

#include "volumetric_kit/calib/solve/view_pose.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/compute/usable_for_pose.hpp"

namespace volumetric_kit::calib::solve {
namespace {

namespace camera = recon::camera;
using Code = core::Status::Code;

constexpr double kPi = 3.14159265358979323846;

compute::Board femto_board() {
  return compute::Board::create(legacy_femto_board()).value();
}

compute::CvCamera pinhole() {
  return {cv::Size(3840, 2160),
          cv::Matx33d(2240.0, 0.0, 1919.5, 0.0, 2240.0, 1079.5, 0.0, 0.0, 1.0),
          cv::Vec<double, 8>::all(0.0)};
}

// The rotation angle between two transforms (radians).
double rotation_error(const camera::Mat4d& a, const camera::Mat4d& b) {
  return compute::rotation_angle_deg(camera::Mat3d(a), camera::Mat3d(b)) * kPi /
         180.0;
}

double translation_error(const camera::Mat4d& a, const camera::Mat4d& b) {
  return glm::length(camera::Vec3d(a[3] - b[3]));
}

// A view of `ids` through a pinhole, the board at `board_to_camera`, with no
// check that the corners are in front.
Observation pinhole_view(const compute::Board& board,
                         const camera::Mat4d& board_to_camera,
                         const std::vector<int>& ids) {
  const compute::CvCamera cam = pinhole();
  Observation view;
  for (const int id : ids) {
    const cv::Point3d& x = board.object_points()[static_cast<std::size_t>(id)];
    const glm::dvec4 q = board_to_camera * glm::dvec4(x.x, x.y, x.z, 1.0);
    view.points.push_back(
        {id,
         {cam.camera_matrix(0, 0) * q.x / q.z + cam.camera_matrix(0, 2),
          cam.camera_matrix(1, 1) * q.y / q.z + cam.camera_matrix(1, 2)},
         1.0});
  }
  return view;
}

// Every view of the synthetic plan, without noise: the best pose is the
// board's, through the colour lens's eight coefficients.
TEST(ViewPose, PosesExactViewsThroughTheWholeLens) {
  const compute::Board board = femto_board();
  const std::vector<test::SyntheticCamera> rig = test::synthetic_rig();
  test::PlanOptions plan_options;
  plan_options.count = 20;
  const std::vector<test::Placement> plan =
      test::synthetic_plan(rig, board, plan_options);
  test::ObserveOptions exact;
  exact.noise_px = 0.0;
  const std::vector<Observation> views = test::observe(rig, board, plan, exact);
  ASSERT_GE(views.size(), 40u);
  for (const Observation& view : views) {
    std::size_t c = 0;
    while (rig[c].id != view.camera_id) ++c;
    const camera::Mat4d truth =
        rig[c].world_to_camera * plan[view.frame_id].board_to_world;
    core::Result<ViewPose> pose = core::Status::invalid_argument("unset");
    EXPECT_NO_THROW(pose = view_pose(compute::cv_camera(rig[c].model).value(),
                                     board, view));
    ASSERT_TRUE(pose.ok()) << pose.status().message();
    EXPECT_LT(rotation_error(pose->best.board_to_camera, truth), 1e-9);
    EXPECT_LT(translation_error(pose->best.board_to_camera, truth), 1e-9);
    EXPECT_LT(pose->best.rms_px, 1e-6);
    ASSERT_TRUE(pose->alternative.has_value());
    EXPECT_GE(pose->alternative->rms_px, pose->best.rms_px);
    EXPECT_GT(pose->ambiguity(), 2.0);
  }
}

// Exact views near the edges of a lens where undistortPoints' default
// criteria, solvePnPGeneric's own, leave these views' corners up to 0.18 px
// out: the board parallel to the image 3 m away, its outline 30 px or more
// inside, gives the board's pose.
TEST(ViewPose, PosesExactViewsAtTheEdgesOfAStronglyDistortedLens) {
  const compute::Board board = femto_board();
  const camera::CameraModel lens = test::strongly_distorted_lens();
  const compute::CvCamera cam = compute::cv_camera(lens).value();
  // The board's centre at each pixel.
  for (const cv::Point2d& centre :
       {cv::Point2d(210.0, 118.0), cv::Point2d(1069.0, 118.0),
        cv::Point2d(210.0, 601.0), cv::Point2d(1069.0, 601.0),
        cv::Point2d(174.0, 359.5), cv::Point2d(639.5, 102.0)}) {
    const cv::Point2d ray = compute::undistort_points(cam, {centre}).value()[0];
    camera::Mat4d truth(1.0);
    truth[3] = glm::dvec4(3.0 * ray.x - 0.375, 3.0 * ray.y - 0.225, 3.0, 1.0);
    const std::vector<cv::Point2d> pixels =
        test::project(lens, truth, board.object_points());
    Observation view;
    for (std::size_t i = 0; i < pixels.size(); ++i) {
      view.points.push_back(
          {static_cast<int>(i), {pixels[i].x, pixels[i].y}, 1.0});
    }
    const core::Result<ViewPose> pose = view_pose(cam, board, view);
    ASSERT_TRUE(pose.ok()) << pose.status().message();
    EXPECT_LT(rotation_error(pose->best.board_to_camera, truth), 1e-7)
        << centre;
    EXPECT_LT(translation_error(pose->best.board_to_camera, truth), 1e-7)
        << centre;
    EXPECT_LT(pose->best.rms_px, 1e-6) << centre;
  }
}

// A board turned a half turn, or within 1e-6 rad of one, about axes that
// keep its face toward the camera, where IPPE's rotation vectors lose the
// angle: the pose is exact, and its mirror image a far worse fit.
TEST(ViewPose, PosesABoardTurnedHalfATurn) {
  const compute::Board board = femto_board();
  const camera::Vec3d middle(0.375, 0.225, 0.0);  // the 5 x 3 squares' middle
  const double tilt = 15.0 * kPi / 180.0;
  for (const camera::Vec3d& axis :
       {camera::Vec3d(0.0, 0.0, 1.0),
        camera::Vec3d(0.0, std::sin(tilt), std::cos(tilt)),
        glm::normalize(camera::Vec3d(0.27, 0.42, 0.87))}) {
    for (const camera::Vec3d& centre :
         {camera::Vec3d(0.6, 0.3, 2.5), camera::Vec3d(-0.4, 0.2, 1.5)}) {
      for (const double short_of_half : {0.0, 1e-9, 1e-7, 1e-6}) {
        const camera::Mat3d rotation =
            camera::rotation_from_rodrigues(axis * (kPi - short_of_half));
        camera::Mat4d truth(rotation);
        truth[3] = glm::dvec4(centre - rotation * middle, 1.0);
        SCOPED_TRACE(testing::Message()
                     << "axis (" << axis.x << ", " << axis.y << ", " << axis.z
                     << "), centre (" << centre.x << ", " << centre.y << ", "
                     << centre.z << "), " << short_of_half << " rad short");
        const Observation view =
            pinhole_view(board, truth, {0, 1, 2, 3, 4, 5, 6, 7});
        const core::Result<ViewPose> pose = view_pose(pinhole(), board, view);
        ASSERT_TRUE(pose.ok()) << pose.status().message();
        EXPECT_LT(rotation_error(pose->best.board_to_camera, truth), 1e-9);
        EXPECT_LT(translation_error(pose->best.board_to_camera, truth), 1e-9);
        EXPECT_LT(pose->best.rms_px, 1e-6);
        EXPECT_GT(pose->ambiguity(), 2.0);
      }
    }
  }
}

// Every subset of 4 to 8 corners of each exact view that usable_for_pose
// accepts gives IPPE's two poses, the best the board's: the check refuses
// every view IPPE cannot pose, all but one corner on a line included.
TEST(ViewPose, PosesEveryUsableSubsetOfAnExactView) {
  const compute::Board board = femto_board();
  const std::vector<test::SyntheticCamera> rig = test::synthetic_rig();
  test::PlanOptions plan_options;
  plan_options.count = 10;
  const std::vector<test::Placement> plan =
      test::synthetic_plan(rig, board, plan_options);
  test::ObserveOptions exact;
  exact.noise_px = 0.0;
  const std::vector<Observation> views = test::observe(rig, board, plan, exact);
  std::size_t usable = 0;
  std::size_t refused = 0;
  for (const Observation& view : views) {
    ASSERT_EQ(view.points.size(), 8u);
    std::size_t c = 0;
    while (rig[c].id != view.camera_id) ++c;
    const compute::CvCamera cam = compute::cv_camera(rig[c].model).value();
    const camera::Mat4d truth =
        rig[c].world_to_camera * plan[view.frame_id].board_to_world;
    for (unsigned mask = 0; mask < 256u; ++mask) {
      Observation subset;
      for (std::size_t i = 0; i < 8; ++i) {
        if (mask & (1u << i)) subset.points.push_back(view.points[i]);
      }
      if (subset.points.size() < 4) continue;
      if (compute::usable_for_pose(board, subset, cam.size).value() !=
          compute::ViewUse::Usable) {
        ++refused;
        continue;
      }
      ++usable;
      const core::Result<ViewPose> pose = view_pose(cam, board, subset);
      ASSERT_TRUE(pose.ok())
          << "mask " << mask << ": " << pose.status().message();
      EXPECT_TRUE(pose->alternative.has_value()) << mask;
      EXPECT_LT(rotation_error(pose->best.board_to_camera, truth), 1e-7)
          << mask;
      EXPECT_LT(translation_error(pose->best.board_to_camera, truth), 1e-7)
          << mask;
    }
  }
  // On the 4 x 2 grid, 2 corners or more on each row: 121 of the 163
  // subsets of 4 or more.
  EXPECT_EQ(usable, 121u * views.size());
  EXPECT_EQ(refused, 42u * views.size());
}

// With noise, the better fit comes first, and the ambiguity is the ratio of
// the fits; the RMS is each corner's Euclidean error.
TEST(ViewPose, PutsTheBetterFitFirst) {
  const compute::Board board = femto_board();
  const std::vector<test::SyntheticCamera> rig = test::synthetic_rig();
  test::PlanOptions plan_options;
  plan_options.count = 10;
  const std::vector<test::Placement> plan =
      test::synthetic_plan(rig, board, plan_options);
  const std::vector<Observation> views = test::observe(rig, board, plan);
  for (const Observation& view : views) {
    std::size_t c = 0;
    while (rig[c].id != view.camera_id) ++c;
    const compute::CvCamera cam = compute::cv_camera(rig[c].model).value();
    const core::Result<ViewPose> pose = view_pose(cam, board, view);
    ASSERT_TRUE(pose.ok()) << pose.status().message();
    ASSERT_TRUE(pose->alternative.has_value());
    EXPECT_LE(pose->best.rms_px, pose->alternative->rms_px);
    EXPECT_DOUBLE_EQ(pose->ambiguity(),
                     pose->alternative->rms_px / pose->best.rms_px);
    // 0.3 px on each axis: about 0.42 px per corner, less what the pose fits.
    EXPECT_GT(pose->best.rms_px, 0.1);
    EXPECT_LT(pose->best.rms_px, 0.8);

    // The RMS, recomputed through OpenCV's projection.
    std::vector<cv::Point3d> object;
    for (const DetectedPoint& p : view.points) {
      object.push_back(board.object_points()[static_cast<std::size_t>(p.id)]);
    }
    const std::vector<cv::Point2d> pixels =
        test::project(rig[c].model, pose->best.board_to_camera, object);
    double sum = 0.0;
    for (std::size_t i = 0; i < pixels.size(); ++i) {
      const double dx = pixels[i].x - view.points[i].pixel.x;
      const double dy = pixels[i].y - view.points[i].pixel.y;
      sum += dx * dx + dy * dy;
    }
    EXPECT_NEAR(pose->best.rms_px, std::sqrt(sum / pixels.size()), 1e-9);
  }
}

// A small square face-on and far away, its corners 1 px out, fits its mirror
// image almost as well.
TEST(ViewPose, AFarFaceOnSquareIsAmbiguous) {
  const compute::Board board = femto_board();
  camera::Mat4d board_to_camera(1.0);
  // Corners 1, 2, 5 and 6 (one 0.15 m square) centred 6 m ahead.
  board_to_camera[3] = glm::dvec4(-0.375, -0.225, 6.0, 1.0);
  Observation view = pinhole_view(board, board_to_camera, {1, 2, 5, 6});
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 1.0);
  for (DetectedPoint& p : view.points) {
    p.pixel += camera::Vec2d(noise(rng), noise(rng));
  }
  const core::Result<ViewPose> pose = view_pose(pinhole(), board, view);
  ASSERT_TRUE(pose.ok()) << pose.status().message();
  EXPECT_LT(pose->ambiguity(), 2.0);
}

TEST(ViewPose, AmbiguityOfExactAndSingleFits) {
  ViewPose pose;
  pose.best.rms_px = 0.5;
  EXPECT_EQ(pose.ambiguity(), std::numeric_limits<double>::infinity());
  pose.alternative = PoseCandidate{camera::Mat4d(1.0), 1.5};
  EXPECT_DOUBLE_EQ(pose.ambiguity(), 3.0);
  pose.best.rms_px = 0.0;
  EXPECT_EQ(pose.ambiguity(), std::numeric_limits<double>::infinity());
  pose.alternative->rms_px = 0.0;
  EXPECT_EQ(pose.ambiguity(), 1.0);
}

// Corners seen through a pinhole from a board that straddles the camera: a
// pose that put some of them behind it would fit exactly, as OpenCV projects
// a point behind the camera mirrored, so none is accepted. The board's origin
// is in front in the first, behind in the second.
TEST(ViewPose, RefusesPosesWithACornerBehindTheCamera) {
  const compute::Board board = femto_board();
  // The board's x along the camera's -z or +z, its y along the camera's x.
  camera::Mat4d along_minus_z(0.0);
  along_minus_z[0] = glm::dvec4(0.0, 0.0, -1.0, 0.0);
  along_minus_z[1] = glm::dvec4(1.0, 0.0, 0.0, 0.0);
  along_minus_z[2] = glm::dvec4(0.0, -1.0, 0.0, 0.0);
  along_minus_z[3] = glm::dvec4(-0.225, 0.05, 0.3, 1.0);
  camera::Mat4d along_z(0.0);
  along_z[0] = glm::dvec4(0.0, 0.0, 1.0, 0.0);
  along_z[1] = glm::dvec4(1.0, 0.0, 0.0, 0.0);
  along_z[2] = glm::dvec4(0.0, 1.0, 0.0, 0.0);
  along_z[3] = glm::dvec4(-0.225, 0.1, -0.375, 1.0);
  for (const camera::Mat4d& board_to_camera : {along_minus_z, along_z}) {
    ASSERT_TRUE(camera::check_rigid(board_to_camera).ok());
    const Observation view = pinhole_view(board, board_to_camera, {0, 3, 4, 7});
    ASSERT_EQ(compute::usable_for_pose(board, view, pinhole().size).value(),
              compute::ViewUse::Usable);
    core::Result<ViewPose> pose = ViewPose{};
    EXPECT_NO_THROW(pose = view_pose(pinhole(), board, view));
    EXPECT_EQ(pose.status().domain(), Code::Numerical)
        << pose.status().message();
  }
}

// A lens no pose can be projected through: nothing finite is accepted.
TEST(ViewPose, RefusesPosesThatProjectToNothingFinite) {
  const compute::Board board = femto_board();
  camera::Mat4d board_to_camera(1.0);
  board_to_camera[3] = glm::dvec4(-0.375, -0.225, 2.0, 1.0);
  const Observation view =
      pinhole_view(board, board_to_camera, {0, 1, 2, 3, 4, 5, 6, 7});
  compute::CvCamera wild = pinhole();
  wild.distortion[0] = 1e300;  // k1
  core::Result<ViewPose> pose = ViewPose{};
  EXPECT_NO_THROW(pose = view_pose(wild, board, view));
  EXPECT_FALSE(pose.ok());
}

// A view usable_for_pose refuses is refused, saying why, and nothing throws.
TEST(ViewPose, RefusesAViewThatFixesNoPose) {
  const compute::Board board = femto_board();
  camera::Mat4d board_to_camera(1.0);
  board_to_camera[3] = glm::dvec4(-0.375, -0.225, 2.0, 1.0);
  const Observation whole =
      pinhole_view(board, board_to_camera, {0, 1, 2, 3, 4, 5, 6, 7});
  ASSERT_TRUE(view_pose(pinhole(), board, whole).ok());

  Observation nan = whole;
  nan.points[3].pixel.y = std::numeric_limits<double>::quiet_NaN();
  Observation collinear = whole;
  collinear.points.resize(4);  // corners 0-3, one row
  Observation repeated = whole;
  repeated.points[1].id = 0;
  const struct {
    const char* name;
    Observation view;
    const char* reason;
  } cases[] = {{"empty", Observation{}, "fewer than 4 corners"},
               {"a NaN corner", nan, "not finite"},
               {"collinear", collinear, "one line"},
               {"a repeated id", repeated, "repeated"}};
  for (const auto& c : cases) {
    core::Result<ViewPose> pose = ViewPose{};
    EXPECT_NO_THROW(pose = view_pose(pinhole(), board, c.view)) << c.name;
    EXPECT_EQ(pose.status().domain(), Code::InvalidArgument) << c.name;
    EXPECT_EQ(pose.status().message().rfind("view_pose: ", 0), 0u) << c.name;
    EXPECT_NE(pose.status().message().find(c.reason), std::string::npos)
        << c.name << ": " << pose.status().message();
  }

  // A camera of no image size: usable_for_pose's refusal passes through.
  compute::CvCamera no_image = pinhole();
  no_image.size = cv::Size(0, 0);
  core::Result<ViewPose> pose = ViewPose{};
  EXPECT_NO_THROW(pose = view_pose(no_image, board, whole));
  EXPECT_EQ(pose.status().domain(), Code::InvalidArgument);
}

// Tilt: 0 face-on, the turn about the board's centre otherwise, and past 90
// from behind; the same wherever the board is turned about its normal.
TEST(ViewPose, TiltIsTheFacesTurnFromTheCamera) {
  const compute::Board board = femto_board();
  const camera::Vec3d centre(0.375, 0.225, 0.0);  // the 5 x 3 squares' middle
  for (const double tilt : {0.0, 25.0, 60.0, 89.0, 120.0}) {
    for (const double spin : {0.0, 40.0}) {
      // The board's centre 2 m ahead on a ray 10 degrees off the axis; its
      // face turned `tilt` away from that ray about an axis across it.
      const camera::Vec3d up(0.0, 1.0, 0.0);
      const camera::Vec3d ray = glm::normalize(camera::Vec3d(
          std::sin(10.0 * kPi / 180.0), 0.0, std::cos(10.0 * kPi / 180.0)));
      const camera::Vec3d across = glm::normalize(glm::cross(ray, up));
      // Face-on: the board's z along the ray, so its face looks back along it.
      const camera::Vec3d x = glm::normalize(glm::cross(up, ray));
      const camera::Mat3d face_on(x, glm::cross(ray, x), ray);
      const camera::Mat3d rotation =
          camera::rotation_from_rodrigues(across * (tilt * kPi / 180.0)) *
          face_on *
          camera::rotation_from_rodrigues(
              camera::Vec3d(0.0, 0.0, spin * kPi / 180.0));
      camera::Mat4d board_to_camera(rotation);
      board_to_camera[3] = glm::dvec4(2.0 * ray - rotation * centre, 1.0);
      EXPECT_NEAR(tilt_deg(board, board_to_camera), tilt, 1e-9)
          << tilt << " deg, spun " << spin;
    }
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::solve
