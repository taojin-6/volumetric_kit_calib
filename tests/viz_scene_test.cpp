// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The 3D view's geometry, from host data: a frustum's corners are its image's
// outer edges taken back through a distorted lens, 0.3 m ahead; the axes and
// a solved board's outline lie where they should; the scene's lines for 1, 3
// and 5 cameras leave out the unplaced and the hidden; the display rotation
// takes the cameras' mean up to +Y and their levelled forward to -Z, for the
// reference alone, an outside-in ring, and a ring whose forwards cancel; the
// view aims at the rig without mirroring it; and the window splits between
// the mosaic and the view.

#include "volumetric_kit/calib/viz/scene_view.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/vec4.hpp>

#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"

namespace volumetric_kit::calib::viz {
namespace {

namespace camera = recon::camera;
namespace pipelines = gfx::pipelines;
using camera::Mat3d;
using camera::Mat4d;
using camera::Vec3d;
using Code = core::Status::Code;

constexpr std::uint32_t kTint = pipelines::pack_rgba8(10, 200, 90);

// A rotation of `degrees` about `axis`, then a translation.
Mat4d pose(double degrees, Vec3d axis, Vec3d translation) {
  return glm::translate(Mat4d(1.0), translation) *
         glm::rotate(Mat4d(1.0), glm::radians(degrees), axis);
}

Vec3d point(const pipelines::ColorVertex& v) { return Vec3d(v.position); }

Vec3d apply(const Mat4d& m, const Vec3d& p) {
  return Vec3d(m * glm::dvec4(p, 1.0));
}

void expect_near(const Vec3d& a, const Vec3d& b, double tolerance,
                 const std::string& what) {
  EXPECT_NEAR(a.x, b.x, tolerance) << what;
  EXPECT_NEAR(a.y, b.y, tolerance) << what;
  EXPECT_NEAR(a.z, b.z, tolerance) << what;
}

// A camera at `centre`, its +z along `forward` and its +y as near the world's
// +y (down) as `forward` allows: upright in the rig's world.
Mat4d upright(Vec3d centre, Vec3d forward) {
  const Vec3d z = glm::normalize(forward);
  const Vec3d x = glm::normalize(glm::cross(Vec3d(0.0, 1.0, 0.0), z));
  const Vec3d y = glm::cross(z, x);
  Mat4d m(1.0);
  m[0] = glm::dvec4(x, 0.0);
  m[1] = glm::dvec4(y, 0.0);
  m[2] = glm::dvec4(z, 0.0);
  m[3] = glm::dvec4(centre, 1.0);
  return m;
}

// The frustum's centre is the camera's, and each corner, 0.3 m ahead along
// its +z, images back onto its image's outer edge: half a pixel outside the
// corner pixels' centres, through the colour lens's distortion.
TEST(VizScene, AFrustumReachesTheImagesEdgesThroughItsLens) {
  const camera::CameraModel lens = test::color_like_lens();
  const Mat4d to_world = pose(20.0, {0.3, 1.0, 0.2}, {0.4, -0.2, 1.5});
  const core::Result<std::vector<pipelines::ColorVertex>> lines =
      frustum_lines(lens, to_world, 0.3, kTint);
  ASSERT_TRUE(lines.ok()) << lines.status().message();
  ASSERT_EQ(lines.value().size(), 16u);
  for (const pipelines::ColorVertex& v : lines.value()) {
    EXPECT_EQ(v.rgba, kTint);
  }
  const core::Result<compute::CvCamera> cv = compute::cv_camera(lens);
  ASSERT_TRUE(cv.ok());
  const Mat4d to_camera = camera::rigid_inverse(to_world);
  const double w = lens.size.width;
  const double h = lens.size.height;
  const camera::Vec2d edges[4] = {
      {-0.5, -0.5}, {w - 0.5, -0.5}, {w - 0.5, h - 0.5}, {-0.5, h - 0.5}};
  const std::vector<pipelines::ColorVertex>& v = lines.value();
  for (int i = 0; i < 4; ++i) {
    SCOPED_TRACE("corner " + std::to_string(i));
    expect_near(point(v[2 * i]), Vec3d(to_world[3]), 1e-6, "the centre");
    const Vec3d corner = point(v[2 * i + 1]);
    const Vec3d in_camera = apply(to_camera, corner);
    EXPECT_NEAR(in_camera.z, 0.3, 1e-6);
    const std::optional<camera::Vec2d> pixel =
        compute::project(cv.value(), in_camera);
    ASSERT_TRUE(pixel.has_value());
    EXPECT_NEAR(pixel->x, edges[i].x, 0.01);
    EXPECT_NEAR(pixel->y, edges[i].y, 0.01);
    // The rectangle joins each corner to the next.
    expect_near(point(v[8 + 2 * i]), corner, 0.0, "the rectangle's start");
    expect_near(point(v[9 + 2 * (i + 3) % 8]), corner, 0.0,
                "the previous side's end");
  }
}

TEST(VizScene, AFrustumRefusesWhatItCannotDraw) {
  const camera::CameraModel lens = test::color_like_lens();
  const Mat4d to_world(1.0);
  for (const double depth :
       {0.0, -0.3, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()}) {
    const core::Status refused =
        frustum_lines(lens, to_world, depth, kTint).status();
    EXPECT_EQ(refused.domain(), Code::InvalidArgument) << depth;
  }
  const core::Status scaled =
      frustum_lines(lens, glm::scale(Mat4d(1.0), Vec3d(2.0)), 0.3, kTint)
          .status();
  EXPECT_EQ(scaled.domain(), Code::InvalidArgument);
  EXPECT_NE(scaled.message().find("frustum_lines"), std::string::npos);
  camera::CameraModel empty;
  EXPECT_EQ(frustum_lines(empty, to_world, 0.3, kTint).status().domain(),
            Code::InvalidArgument);
}

TEST(VizScene, TheAxesRunFromTheOriginInRedGreenAndBlue) {
  const core::Result<std::vector<pipelines::ColorVertex>> axes =
      axes_lines(0.5);
  ASSERT_TRUE(axes.ok());
  ASSERT_EQ(axes.value().size(), 6u);
  const Vec3d ends[3] = {{0.5, 0.0, 0.0}, {0.0, 0.5, 0.0}, {0.0, 0.0, 0.5}};
  for (int a = 0; a < 3; ++a) {
    expect_near(point(axes.value()[2 * a]), Vec3d(0.0), 0.0, "origin");
    expect_near(point(axes.value()[2 * a + 1]), ends[a], 0.0, "end");
    EXPECT_EQ(axes.value()[2 * a].rgba, axes.value()[2 * a + 1].rgba);
  }
  // Red, green, blue: each axis's strongest channel is its own.
  for (int a = 0; a < 3; ++a) {
    const std::uint32_t rgba = axes.value()[2 * a].rgba;
    const int own = static_cast<int>((rgba >> (8 * a)) & 0xFFu);
    for (int c = 0; c < 3; ++c) {
      if (c != a) {
        EXPECT_GT(own, static_cast<int>((rgba >> (8 * c)) & 0xFFu));
      }
    }
  }
  for (const double length :
       {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_EQ(axes_lines(length).status().domain(), Code::InvalidArgument);
  }
}

// A solved board's outline is the chessboard's outer edge where the solve put
// it.
TEST(VizScene, ABoardsOutlineIsItsChessboardsEdge) {
  const compute::Board board = test::femto_board();
  const Mat4d board_to_world = pose(35.0, {1.0, 0.2, -0.4}, {0.1, 0.9, 2.2});
  const core::Result<std::vector<pipelines::ColorVertex>> quad =
      board_lines(board, board_to_world, kTint);
  ASSERT_TRUE(quad.ok()) << quad.status().message();
  ASSERT_EQ(quad.value().size(), 8u);
  const std::vector<cv::Point3d> edge = test::board_outline(board, 0.0, 1);
  ASSERT_EQ(edge.size(), 4u);
  for (std::size_t i = 0; i < 4; ++i) {
    const cv::Point3d& a = edge[i];
    const cv::Point3d& b = edge[(i + 1) % 4];
    expect_near(point(quad.value()[2 * i]),
                apply(board_to_world, Vec3d(a.x, a.y, a.z)), 1e-6, "start");
    expect_near(point(quad.value()[2 * i + 1]),
                apply(board_to_world, Vec3d(b.x, b.y, b.z)), 1e-6, "end");
    EXPECT_EQ(quad.value()[2 * i].rgba, kTint);
  }
  EXPECT_EQ(board_lines(board, glm::scale(Mat4d(1.0), Vec3d(1.5)), kTint)
                .status()
                .domain(),
            Code::InvalidArgument);
}

// Every camera placed and shown has its frustum, and its depth frustum when
// asked, at its depth camera's pose and darker; the rest have none; each
// solved board adds its outline. For 1, 3 and 5 cameras.
TEST(VizScene, TheScenesLinesAreItsPlacedShownCamerasAndBoards) {
  const compute::Board board = test::femto_board();
  const Mat4d depth_to_color = pose(1.0, {0.0, 1.0, 0.0}, {0.032, -0.002, 0.0});
  for (const std::size_t n : {1u, 3u, 5u}) {
    SCOPED_TRACE(std::to_string(n) + " cameras");
    std::vector<SceneCamera> cameras(n);
    std::size_t drawn = 0;
    for (std::size_t i = 0; i < n; ++i) {
      cameras[i].color = test::color_like_lens();
      cameras[i].depth = test::depth_like_lens();
      cameras[i].depth_to_color = depth_to_color;
      cameras[i].tint =
          pipelines::pack_rgba8(200, 100, static_cast<std::uint8_t>(i));
      // Camera 1 is not placed; camera 3 is hidden.
      if (i != 1) {
        const double k = static_cast<double>(i);
        cameras[i].color_to_world =
            pose(10.0 * k, {0.0, 1.0, 0.0}, {0.5 * k, 0.0, 0.0});
      }
      cameras[i].shown = i != 3;
      drawn += i != 1 && i != 3 ? 1 : 0;
    }
    SceneOptions options;
    core::Result<std::vector<pipelines::ColorVertex>> lines =
        scene_lines(cameras, nullptr, {}, options);
    ASSERT_TRUE(lines.ok()) << lines.status().message();
    EXPECT_EQ(lines.value().size(), 6 + 16 * drawn);

    options.depth_frusta = true;
    options.axes_m = 0.0;
    const std::vector<Mat4d> boards = {
        pose(5.0, {1.0, 0.0, 0.0}, {0.0, 0.5, 2.0}),
        pose(-5.0, {1.0, 0.0, 0.0}, {0.0, 0.4, 2.0})};
    lines = scene_lines(cameras, &board, boards, options);
    ASSERT_TRUE(lines.ok()) << lines.status().message();
    ASSERT_EQ(lines.value().size(), 32 * drawn + 8 * boards.size());
    // The first camera's depth frustum follows its colour frustum: centred
    // at its depth camera, in its tint at 60 %.
    const pipelines::ColorVertex& depth_centre = lines.value()[16];
    expect_near(point(depth_centre),
                Vec3d((*cameras[0].color_to_world * depth_to_color)[3]), 1e-6,
                "the depth camera's centre");
    EXPECT_EQ(depth_centre.rgba, pipelines::pack_rgba8(120, 60, 0));

    EXPECT_EQ(scene_lines(cameras, nullptr, boards, options).status().domain(),
              Code::InvalidArgument);
  }
}

// The reference camera alone: its up, -y, turns to +Y and its forward to -Z,
// a half turn about x.
TEST(VizScene, TheReferenceAloneTurnsAHalfTurnAboutX) {
  const core::Result<Mat3d> turn = display_rotation({Mat4d(1.0)});
  ASSERT_TRUE(turn.ok()) << turn.status().message();
  const Mat3d expected(Vec3d(1.0, 0.0, 0.0), Vec3d(0.0, -1.0, 0.0),
                       Vec3d(0.0, 0.0, -1.0));
  for (int c = 0; c < 3; ++c) {
    for (int r = 0; r < 3; ++r) {
      EXPECT_NEAR(turn.value()[c][r], expected[c][r], 1e-12);
    }
  }
}

void expect_rotation(const Mat3d& m) {
  const Mat3d identity = glm::transpose(m) * m;
  for (int c = 0; c < 3; ++c) {
    for (int r = 0; r < 3; ++r) {
      EXPECT_NEAR(identity[c][r], c == r ? 1.0 : 0.0, 1e-9);
    }
  }
  EXPECT_NEAR(glm::determinant(m), 1.0, 1e-9);
}

// The synthetic rig's outside-in ring of four, looking down at the floor's
// middle: the cameras' mean -y turns to +Y and their mean forward, levelled,
// to -Z.
TEST(VizScene, TheMeanCameraUpTurnsToPlusY) {
  std::vector<Mat4d> poses;
  Vec3d up(0.0);
  Vec3d forward(0.0);
  for (const test::SyntheticCamera& c : test::synthetic_rig()) {
    poses.push_back(camera::rigid_inverse(c.world_to_camera));
    up -= Mat3d(poses.back())[1];
    forward += Mat3d(poses.back())[2];
  }
  up = glm::normalize(up);
  const core::Result<Mat3d> turn = display_rotation(poses);
  ASSERT_TRUE(turn.ok()) << turn.status().message();
  expect_rotation(turn.value());
  expect_near(turn.value() * up, Vec3d(0.0, 1.0, 0.0), 1e-9, "the mean up");
  const Vec3d level = glm::normalize(forward - glm::dot(forward, up) * up);
  expect_near(turn.value() * level, Vec3d(0.0, 0.0, -1.0), 1e-9,
              "the levelled forward");
}

// Four cameras in a level ring looking at its middle: their forwards cancel,
// so the first camera's gives the view its forward.
TEST(VizScene, ARingWhoseForwardsCancelTakesTheFirstCamerasForward) {
  std::vector<Mat4d> poses;
  for (int k = 0; k < 4; ++k) {
    const double a = glm::radians(90.0 * k);
    const Vec3d centre(3.0 * std::sin(a), -2.0, -3.0 * std::cos(a));
    poses.push_back(upright(centre, -centre * Vec3d(1.0, 0.0, 1.0)));
  }
  const core::Result<Mat3d> turn = display_rotation(poses);
  ASSERT_TRUE(turn.ok()) << turn.status().message();
  expect_rotation(turn.value());
  expect_near(turn.value() * Vec3d(0.0, -1.0, 0.0), Vec3d(0.0, 1.0, 0.0), 1e-9,
              "up");
  expect_near(turn.value() * Vec3d(Mat3d(poses[0])[2]), Vec3d(0.0, 0.0, -1.0),
              1e-9, "the first camera's forward");
}

TEST(VizScene, TheDisplayRotationRefusesNoCameraAndUpsThatCancel) {
  EXPECT_EQ(display_rotation({}).status().domain(), Code::InvalidArgument);
  const Mat4d flipped = pose(180.0, {0.0, 0.0, 1.0}, {0.0, 0.0, 0.0});
  const core::Status cancel = display_rotation({Mat4d(1.0), flipped}).status();
  EXPECT_EQ(cancel.domain(), Code::InvalidArgument);
  EXPECT_NE(cancel.message().find("cancel"), std::string::npos);
  EXPECT_EQ(display_rotation({glm::scale(Mat4d(1.0), Vec3d(2.0))})
                .status()
                .domain(),
            Code::InvalidArgument);
}

// Clip space of a world point through a view.
glm::dvec3 ndc(const glm::mat4& view_proj, const Vec3d& world) {
  const glm::vec4 clip = view_proj * glm::vec4(glm::vec3(world), 1.0f);
  return {clip.x / clip.w, clip.y / clip.w, clip.z / clip.w};
}

// Aimed at the reference alone, the view looks along it from behind and
// above, at a point 2 m ahead, with the world's right on the right and its up
// (-y) at the top: not mirrored.
TEST(VizScene, TheViewAimsAtTheRigUprightAndUnmirrored) {
  SceneView view;
  ASSERT_TRUE(view.frame({Mat4d(1.0)}).ok());
  EXPECT_NEAR(view.orbit().target().z, -2.0f, 1e-6f);
  EXPECT_FLOAT_EQ(view.orbit().distance(), 3.0f);
  const VkRect2D viewport{{0, 0}, {800, 600}};
  const glm::mat4 view_proj = view.view_proj(viewport);
  const glm::dvec3 ahead = ndc(view_proj, {0.0, 0.0, 2.0});
  EXPECT_NEAR(ahead.x, 0.0, 1e-5);
  EXPECT_NEAR(ahead.y, 0.0, 1e-5);
  EXPECT_GT(ahead.z, 0.0);
  EXPECT_LT(ahead.z, 1.0);
  EXPECT_GT(ndc(view_proj, {0.5, 0.0, 2.0}).x, 0.0) << "the world's right";
  // Vulkan's framebuffer runs down, so the top of the view is negative.
  EXPECT_LT(ndc(view_proj, {0.0, -0.5, 2.0}).y, 0.0) << "the world's up";
  // A failed frame leaves the view as it was.
  EXPECT_FALSE(view.frame({}).ok());
  EXPECT_NEAR(view.orbit().target().z, -2.0f, 1e-6f);
}

TEST(VizScene, TheWindowSplitsBetweenTheMosaicAndTheView) {
  const VkExtent2D extent{1281, 720};
  const WindowSplit hidden = window_split(ScenePlacement::Hidden, extent);
  EXPECT_EQ(hidden.mosaic.extent.width, 1281u);
  EXPECT_EQ(hidden.scene.extent.width, 0u);
  const WindowSplit beside = window_split(ScenePlacement::Beside, extent);
  EXPECT_EQ(beside.mosaic.offset.x, 0);
  EXPECT_EQ(beside.mosaic.extent.width, 641u);
  EXPECT_EQ(beside.scene.offset.x, 641);
  EXPECT_EQ(beside.scene.extent.width, 640u);
  EXPECT_EQ(beside.scene.extent.height, 720u);
  const WindowSplit full = window_split(ScenePlacement::FullWindow, extent);
  EXPECT_EQ(full.mosaic.extent.width, 0u);
  EXPECT_EQ(full.scene.extent.width, 1281u);
  EXPECT_EQ(full.scene.extent.height, 720u);
}

}  // namespace
}  // namespace volumetric_kit::calib::viz
