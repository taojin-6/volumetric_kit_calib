// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The synthetic source every calib test draws on: its lenses and rig, its plan
// of placements, and its renderer, whose inverse map and orientation the
// detector's tests will rely on.

#include "synthetic_rig.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/geometry.hpp>

#include "volumetric_kit/calib/compute/cv_camera.hpp"

namespace volumetric_kit::calib::testing {
namespace {

namespace camera = recon::camera;

compute::Board femto_board() {
  return compute::Board::create(legacy_femto_board()).value();
}

TEST(SyntheticRig, LensesAreValid) {
  for (const camera::CameraModel& model :
       {color_like_lens(), depth_like_lens()}) {
    const core::Status status = camera::check_camera_model(model);
    EXPECT_TRUE(status.ok()) << status.message();
  }
}

// Pixels through the renderer's inverse map and back through the lens land
// where they started, across the whole image.
TEST(SyntheticRig, InverseMapRoundTripsBelowANanopixel) {
  for (const camera::CameraModel& model :
       {color_like_lens(), depth_like_lens()}) {
    const double w = model.size.width;
    const double h = model.size.height;
    const double step = w / 128.0;
    std::vector<cv::Point2d> pixels;
    for (double v = -0.5; v <= h - 0.5; v += step) {
      for (double u = -0.5; u <= w - 0.5; u += step) pixels.emplace_back(u, v);
      pixels.emplace_back(w - 0.5, v);
    }
    const std::vector<cv::Point2d> rays = undistort_pixels(model, pixels);
    std::vector<cv::Point3d> points;
    for (const cv::Point2d& r : rays) points.emplace_back(r.x, r.y, 1.0);
    const std::vector<cv::Point2d> back =
        project(model, camera::Mat4d(1.0), points);
    double worst = 0.0;
    for (std::size_t i = 0; i < pixels.size(); ++i) {
      worst = std::max(worst, cv::norm(back[i] - pixels[i]));
    }
    EXPECT_LE(worst, 1e-9) << model.size.width << "x" << model.size.height;
  }
}

TEST(SyntheticRig, ProjectsThroughTheLens) {
  const camera::CameraModel model = color_like_lens();
  const compute::CvCamera cam = compute::cv_camera(model).value();
  camera::Mat4d to_camera(1.0);
  to_camera[3] = glm::dvec4(0.1, -0.2, 2.0, 1.0);
  const std::vector<cv::Point3d> points = {{0.0, 0.0, 0.0}, {0.4, 0.3, 0.5}};
  std::vector<cv::Point2d> expected;
  cv::projectPoints(std::vector<cv::Point3d>{{0.1, -0.2, 2.0}, {0.5, 0.1, 2.5}},
                    cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0),
                    cam.camera_matrix, cam.distortion, expected);
  const std::vector<cv::Point2d> got = project(model, to_camera, points);
  ASSERT_EQ(got.size(), 2u);
  for (std::size_t i = 0; i < got.size(); ++i) {
    EXPECT_LE(cv::norm(got[i] - expected[i]), 1e-12) << i;
  }
}

// Four valid cameras, 2.4-2.6 m up around the floor's middle, each seeing it.
TEST(SyntheticRig, RigSurroundsTheFloor) {
  const std::vector<SyntheticCamera> rig = synthetic_rig();
  ASSERT_EQ(rig.size(), 4u);
  std::set<std::string> ids;
  for (const SyntheticCamera& cam : rig) {
    ids.insert(cam.id);
    EXPECT_TRUE(camera::check_camera_model(cam.model).ok()) << cam.id;
    EXPECT_TRUE(camera::check_rigid(cam.world_to_camera).ok()) << cam.id;
    const camera::Vec3d centre(camera::rigid_inverse(cam.world_to_camera)[3]);
    EXPECT_GT(centre.z, 2.4) << cam.id;
    EXPECT_LT(centre.z, 2.6) << cam.id;
    const cv::Point2d middle =
        project(cam.model, cam.world_to_camera, {{0.0, 0.0, 0.0}})[0];
    EXPECT_GT(middle.x, 0.0) << cam.id;
    EXPECT_GT(middle.y, 0.0) << cam.id;
    EXPECT_LT(middle.x, cam.model.size.width - 1.0) << cam.id;
    EXPECT_LT(middle.y, cam.model.size.height - 1.0) << cam.id;
  }
  EXPECT_EQ(ids.size(), rig.size());
}

TEST(SyntheticRig, PlanPlacesBoardsTheCamerasSee) {
  const std::vector<SyntheticCamera> rig = synthetic_rig();
  const compute::Board board = femto_board();
  PlanOptions options;
  options.count = 40;
  const std::vector<Placement> plan = synthetic_plan(rig, board, options);
  ASSERT_EQ(plan.size(), options.count);
  std::size_t floor = 0;
  for (const Placement& placement : plan) {
    EXPECT_TRUE(camera::check_rigid(placement.board_to_world).ok());
    EXPECT_GE(placement.cameras.size(), options.min_cameras);
    if (std::abs(placement.board_to_world[3].z) < 1e-12) ++floor;
    for (const std::size_t c : placement.cameras) {
      ASSERT_LT(c, rig.size());
      const camera::Mat4d board_to_camera =
          rig[c].world_to_camera * placement.board_to_world;
      for (const cv::Point2d& q :
           project(rig[c].model, board_to_camera, board.object_points())) {
        EXPECT_GE(q.x, options.margin_px);
        EXPECT_GE(q.y, options.margin_px);
        EXPECT_LE(q.x, rig[c].model.size.width - 1.0 - options.margin_px);
        EXPECT_LE(q.y, rig[c].model.size.height - 1.0 - options.margin_px);
      }
      // The printed face (-z) turns toward the camera.
      const camera::Vec3d face = -camera::Vec3d(board_to_camera[2]);
      const camera::Vec3d at(board_to_camera[3]);
      EXPECT_LT(glm::dot(face, at), 0.0);
    }
  }
  EXPECT_GT(floor, 0u);
  EXPECT_LT(floor, plan.size());
}

TEST(SyntheticRig, PlanIsTheSameForTheSameSeed) {
  const std::vector<SyntheticCamera> rig = synthetic_rig();
  const compute::Board board = femto_board();
  PlanOptions options;
  options.count = 10;
  const std::vector<Placement> a = synthetic_plan(rig, board, options);
  const std::vector<Placement> b = synthetic_plan(rig, board, options);
  options.seed = 2;
  const std::vector<Placement> c = synthetic_plan(rig, board, options);
  ASSERT_EQ(a.size(), b.size());
  ASSERT_FALSE(c.empty());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].board_to_world, b[i].board_to_world) << i;
    EXPECT_EQ(a[i].cameras, b[i].cameras) << i;
  }
  EXPECT_NE(a[0].board_to_world, c[0].board_to_world);
}

// A camera on the printed side, looking along the board's +z with its axes on
// the board's, sees the board as OpenCV draws it: not mirrored, the
// top-left square black, at 1000 px per metre.
TEST(SyntheticRig, RendersThePrintedFaceUnmirrored) {
  const compute::Board board = femto_board();
  const camera::CameraModel pinhole{
      {800, 500}, {1000.0, 1000.0, 399.5, 249.5}, {}};
  camera::Mat4d board_to_camera(1.0);
  board_to_camera[3] = glm::dvec4(-0.375, -0.225, 1.0, 1.0);
  const cv::Mat image = render_board(pinhole, board_to_camera, board);
  ASSERT_EQ(image.type(), CV_8UC1);
  ASSERT_EQ(image.size(), cv::Size(800, 500));

  // The board fills pixels [25, 775) x [25, 475).
  cv::Mat drawn, expected;
  board.charuco().generateImage(cv::Size(750, 450), drawn, 0, 1);
  const RenderOptions levels;
  drawn.convertTo(expected, CV_8U, (levels.white - levels.black) / 255.0,
                  levels.black);
  const cv::Mat seen = image(cv::Rect(25, 25, 750, 450));
  cv::Mat mirrored;
  cv::flip(expected, mirrored, 1);
  const double error = cv::norm(seen, expected, cv::NORM_L1) / seen.total();
  const double mirrored_error =
      cv::norm(seen, mirrored, cv::NORM_L1) / seen.total();
  // Only the marker cells' edges, off the pixel grid, differ.
  EXPECT_LT(error, 1.5);
  EXPECT_GT(mirrored_error, 5.0 * error);
  // At (row, column): square (0, 0), off the board and its margin, and
  // square (1, 0) around its marker.
  EXPECT_EQ(image.at<unsigned char>(100, 100), levels.black);
  EXPECT_EQ(image.at<unsigned char>(2, 2), levels.background);
  EXPECT_EQ(image.at<unsigned char>(30, 180), levels.white);
}

}  // namespace
}  // namespace volumetric_kit::calib::testing
