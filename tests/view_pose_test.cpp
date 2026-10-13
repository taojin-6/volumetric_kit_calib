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
  return glm::length(camera::rodrigues_from_rotation(
      glm::transpose(camera::Mat3d(a)) * camera::Mat3d(b)));
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
