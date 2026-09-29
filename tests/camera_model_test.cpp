// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/core/camera_model.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <Eigen/Core>

namespace volumetric_kit::calib {
namespace {

using Intrinsics = PinholeBrownConrady::Intrinsics;
using Distortion = PinholeBrownConrady::Distortion;

constexpr ImageSize kHd{1280, 720};
constexpr Intrinsics kHdIntrinsics{/*fx=*/1000.0, /*fy=*/1000.0, /*cx=*/640.0,
                                   /*cy=*/360.0};
constexpr Distortion kBarrel{/*k1=*/-0.28, /*k2=*/0.10, /*p1=*/0.001,
                             /*p2=*/-0.0005, /*k3=*/0.0};
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

PinholeBrownConrady make(const ImageSize& size, const Intrinsics& intrinsics,
                         const Distortion& distortion = {}) {
  Result<PinholeBrownConrady> cam =
      PinholeBrownConrady::create(size, intrinsics, distortion);
  EXPECT_TRUE(cam.ok()) << cam.status().message();
  return std::move(cam).value();
}

// Unprojects a grid of pixels spanning the whole image, corners included, and
// checks each one projects back onto itself.
void expect_round_trips_across_image(const PinholeBrownConrady& cam) {
  const ImageSize& size = cam.image_size();
  for (int i = 0; i <= 32; ++i) {
    for (int k = 0; k <= 18; ++k) {
      const Eigen::Vector2d pixel{size.width * i / 32.0,
                                  size.height * k / 18.0};
      const Result<Eigen::Vector2d> ray = cam.unproject(pixel);
      ASSERT_TRUE(ray.ok()) << "pixel (" << pixel.x() << ", " << pixel.y()
                            << "): " << ray.status().message();
      const Result<Eigen::Vector2d> back = cam.project({ray->x(), ray->y(), 1});
      ASSERT_TRUE(back.ok());
      EXPECT_LT((*back - pixel).norm(), 1e-8)
          << "pixel (" << pixel.x() << ", " << pixel.y() << ")";
    }
  }
}

// --- create -----------------------------------------------------------------

TEST(CameraModel, RejectsNonPositiveFocalLength) {
  EXPECT_FALSE(PinholeBrownConrady::create(kHd, {0.0, 1000.0, 640.0, 360.0}));
  EXPECT_FALSE(PinholeBrownConrady::create(kHd, {1000.0, -1.0, 640.0, 360.0}));
}

TEST(CameraModel, RejectsNonPositiveImageSize) {
  EXPECT_FALSE(PinholeBrownConrady::create({0, 720}, kHdIntrinsics));
  EXPECT_FALSE(PinholeBrownConrady::create({1280, -1}, kHdIntrinsics));
}

TEST(CameraModel, RejectsNonFiniteParameters) {
  const auto domain = [](const Result<PinholeBrownConrady>& r) {
    return r.status().domain();
  };
  EXPECT_EQ(
      domain(PinholeBrownConrady::create(kHd, {1000.0, 1000.0, kNaN, 360.0})),
      Status::Code::InvalidArgument);
  EXPECT_EQ(
      domain(PinholeBrownConrady::create(kHd, {1000.0, kInf, 640.0, 360.0})),
      Status::Code::InvalidArgument);
  EXPECT_EQ(domain(PinholeBrownConrady::create(kHd, kHdIntrinsics,
                                               {kNaN, 0.0, 0.0, 0.0, 0.0})),
            Status::Code::InvalidArgument);
  EXPECT_EQ(domain(PinholeBrownConrady::create(kHd, kHdIntrinsics,
                                               {0.0, 0.0, 0.0, 0.0, -kInf})),
            Status::Code::InvalidArgument);
}

TEST(CameraModel, KeepsItsImageSize) {
  const PinholeBrownConrady cam = make(kHd, kHdIntrinsics);
  EXPECT_EQ(cam.image_size().width, 1280);
  EXPECT_EQ(cam.image_size().height, 720);
}

// --- project ----------------------------------------------------------------

TEST(CameraModel, ProjectsPrincipalRayToPrincipalPoint) {
  const Result<Eigen::Vector2d> px =
      make(kHd, kHdIntrinsics).project({0.0, 0.0, 1.0});
  ASSERT_TRUE(px.ok());
  EXPECT_NEAR(px->x(), 640.0, 1e-9);
  EXPECT_NEAR(px->y(), 360.0, 1e-9);
}

TEST(CameraModel, ProjectsKnownPointWithoutDistortion) {
  // (0.1, -0.2, 2) -> normalized (0.05, -0.1) -> (640+50, 360-100).
  const Result<Eigen::Vector2d> px =
      make(kHd, kHdIntrinsics).project({0.1, -0.2, 2.0});
  ASSERT_TRUE(px.ok());
  EXPECT_NEAR(px->x(), 690.0, 1e-9);
  EXPECT_NEAR(px->y(), 260.0, 1e-9);
}

TEST(CameraModel, ProjectMatchesOpenCv) {
  // Expected pixels from OpenCV 5.0 cv2.projectPoints (zero rvec/tvec) with
  // K = [1210.5 0 640.5; 0 1198.25 359.75; 0 0 1] and
  // dist = (k1, k2, p1, p2, k3) = (-0.28, 0.10, 0.0012, -0.0007, 0.02).
  // Unequal focal lengths and non-zero tangential and k3 terms mean a swapped
  // or dropped term shows up here, which a project/unproject round trip (the
  // same polynomial both ways) cannot catch.
  const PinholeBrownConrady cam =
      make(kHd, {1210.5, 1198.25, 640.5, 359.75},
           {/*k1=*/-0.28, /*k2=*/0.10, /*p1=*/0.0012, /*p2=*/-0.0007,
            /*k3=*/0.02});
  struct Case {
    Eigen::Vector3d point;
    Eigen::Vector2d pixel;
  };
  const Case cases[] = {
      {{0.0, 0.0, 1.0}, {640.5000000000, 359.7500000000}},
      {{0.1, -0.2, 2.0}, {700.7847559424, 240.3976300537}},
      {{-0.45, 0.3, 1.5}, {289.5019891890, 591.4948907210}},
      {{0.8, 0.45, 1.6}, {1196.6699686613, 670.0581658741}},
      {{-1.2, -0.7, 2.5}, {103.5545417229, 50.2966316084}},
  };
  for (const Case& c : cases) {
    const Result<Eigen::Vector2d> px = cam.project(c.point);
    ASSERT_TRUE(px.ok());
    EXPECT_NEAR(px->x(), c.pixel.x(), 1e-8);
    EXPECT_NEAR(px->y(), c.pixel.y(), 1e-8);
  }
}

TEST(CameraModel, ProjectRejectsPointsNotInFrontOfTheCamera) {
  const PinholeBrownConrady cam = make(kHd, kHdIntrinsics, kBarrel);
  for (const Eigen::Vector3d& p :
       {Eigen::Vector3d{0.1, 0.2, 0.0}, Eigen::Vector3d{0.1, 0.2, -1.0},
        Eigen::Vector3d{0.1, 0.2, kNaN}, Eigen::Vector3d{kInf, 0.2, 1.0}}) {
    const Result<Eigen::Vector2d> px = cam.project(p);
    EXPECT_FALSE(px.ok());
    EXPECT_EQ(px.status().domain(), Status::Code::InvalidArgument);
  }
}

// --- unproject --------------------------------------------------------------

TEST(CameraModel, UnprojectInvertsProjectWithoutDistortion) {
  const PinholeBrownConrady cam = make(kHd, kHdIntrinsics);
  const Result<Eigen::Vector2d> back = cam.unproject({690.0, 260.0});
  ASSERT_TRUE(back.ok());
  EXPECT_NEAR(back->x(), 0.05, 1e-12);
  EXPECT_NEAR(back->y(), -0.1, 1e-12);
}

TEST(CameraModel, UnprojectRecoversRaysWithDistortion) {
  const PinholeBrownConrady cam = make(kHd, kHdIntrinsics, kBarrel);
  // Rays out to the image corner (normalized radius ~0.87 once undistorted).
  for (double x : {-0.7, -0.3, 0.0, 0.15, 0.7}) {
    for (double y : {-0.45, 0.0, 0.2, 0.45}) {
      const Result<Eigen::Vector2d> px = cam.project({x, y, 1.0});
      ASSERT_TRUE(px.ok());
      const Result<Eigen::Vector2d> back = cam.unproject(*px);
      ASSERT_TRUE(back.ok()) << back.status().message();
      EXPECT_NEAR(back->x(), x, 1e-11);
      EXPECT_NEAR(back->y(), y, 1e-11);
    }
  }
}

TEST(CameraModel, UnprojectRoundTripsEveryPixelOfTheImage) {
  expect_round_trips_across_image(make(kHd, kHdIntrinsics, kBarrel));
}

TEST(CameraModel, UnprojectConvergesUnderStrongWideAngleDistortion) {
  // ~112 degrees diagonal. This distortion is monotonic, so every pixel has
  // exactly one inverse, but a fixed-point iteration (OpenCV's scheme) misses
  // it by tens of pixels in the corners at any iteration count.
  expect_round_trips_across_image(make(
      kHd, {600.0, 600.0, 640.0, 360.0},
      {/*k1=*/-0.35, /*k2=*/0.12, /*p1=*/0.0005, /*p2=*/-0.0003, /*k3=*/0.0}));
}

TEST(CameraModel, UnprojectReportsPixelsPastTheDistortionFold) {
  // The radius r * (1 - 0.40 r^2 + 0.15 r^4 - 0.03 r^6) peaks at ~0.79 (at
  // r ~ 1.36) and then shrinks, so a pixel whose distorted radius is beyond
  // 0.79 has no inverse inside the fold.
  const PinholeBrownConrady cam =
      make(kHd, {600.0, 600.0, 640.0, 360.0},
           {/*k1=*/-0.40, /*k2=*/0.15, /*p1=*/0.0, /*p2=*/0.0, /*k3=*/-0.03});
  const Result<Eigen::Vector2d> corner = cam.unproject({1280.0, 720.0});
  EXPECT_FALSE(corner.ok());
  EXPECT_EQ(corner.status().domain(), Status::Code::Numerical);

  // Inside the fold the same model still inverts: distorted radius 0.5.
  const Result<Eigen::Vector2d> inside = cam.unproject({940.0, 360.0});
  ASSERT_TRUE(inside.ok()) << inside.status().message();
  const Result<Eigen::Vector2d> back =
      cam.project({inside->x(), inside->y(), 1.0});
  ASSERT_TRUE(back.ok());
  EXPECT_NEAR(back->x(), 940.0, 1e-8);
  EXPECT_NEAR(back->y(), 360.0, 1e-8);
}

TEST(CameraModel, UnprojectRejectsNonFinitePixels) {
  const PinholeBrownConrady cam = make(kHd, kHdIntrinsics, kBarrel);
  const Result<Eigen::Vector2d> r = cam.unproject({kNaN, 100.0});
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.status().domain(), Status::Code::InvalidArgument);
}

}  // namespace
}  // namespace volumetric_kit::calib
