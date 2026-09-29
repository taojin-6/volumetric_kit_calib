// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/core/camera_model.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

namespace volumetric_kit::calib {
namespace {

using Intrinsics = PinholeBrownConrady::Intrinsics;
using Distortion = PinholeBrownConrady::Distortion;

constexpr Intrinsics kVga{/*fx=*/1000.0, /*fy=*/1000.0, /*cx=*/640.0,
                          /*cy=*/360.0};

TEST(CameraModel, RejectsNonPositiveFocalLength) {
  EXPECT_FALSE(PinholeBrownConrady::create({0.0, 1000.0, 640.0, 360.0}).ok());
  EXPECT_FALSE(PinholeBrownConrady::create({1000.0, -1.0, 640.0, 360.0}).ok());
}

TEST(CameraModel, ProjectsPrincipalRayToPrincipalPoint) {
  auto cam = PinholeBrownConrady::create(kVga);
  ASSERT_TRUE(cam.ok());
  const Eigen::Vector2d px = cam->project({0.0, 0.0, 1.0});
  EXPECT_NEAR(px.x(), 640.0, 1e-9);
  EXPECT_NEAR(px.y(), 360.0, 1e-9);
}

TEST(CameraModel, ProjectsKnownPointWithoutDistortion) {
  auto cam = PinholeBrownConrady::create(kVga);
  ASSERT_TRUE(cam.ok());
  // (0.1, -0.2, 2) -> normalized (0.05, -0.1) -> (640+50, 360-100).
  const Eigen::Vector2d px = cam->project({0.1, -0.2, 2.0});
  EXPECT_NEAR(px.x(), 690.0, 1e-9);
  EXPECT_NEAR(px.y(), 260.0, 1e-9);
}

TEST(CameraModel, UnprojectInvertsProjectWithoutDistortion) {
  auto cam = PinholeBrownConrady::create(kVga);
  ASSERT_TRUE(cam.ok());
  const Eigen::Vector2d normalized{0.05, -0.1};
  const Eigen::Vector2d px =
      cam->project({normalized.x(), normalized.y(), 1.0});
  const Eigen::Vector2d back = cam->unproject(px);
  EXPECT_NEAR(back.x(), normalized.x(), 1e-12);
  EXPECT_NEAR(back.y(), normalized.y(), 1e-12);
}

TEST(CameraModel, UnprojectInvertsProjectWithDistortion) {
  auto cam = PinholeBrownConrady::create(
      kVga, {/*k1=*/-0.28, /*k2=*/0.10, /*p1=*/0.001, /*p2=*/-0.0005,
             /*k3=*/0.0});
  ASSERT_TRUE(cam.ok());
  // Round-trip a spread of rays; the iterative undistort should invert project
  // to sub-thousandth-pixel-equivalent accuracy in normalized space.
  for (double x : {-0.3, -0.1, 0.0, 0.15, 0.3}) {
    for (double y : {-0.25, 0.0, 0.2}) {
      const Eigen::Vector2d px = cam->project({x, y, 1.0});
      const Eigen::Vector2d back = cam->unproject(px);
      EXPECT_NEAR(back.x(), x, 1e-8);
      EXPECT_NEAR(back.y(), y, 1e-8);
    }
  }
}

}  // namespace
}  // namespace volumetric_kit::calib
