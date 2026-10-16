// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The measures the solve and the checks share: a median, and the angles
// between directions and between rotations, accurate near 0 and 180 degrees.

#include "volumetric_kit/calib/compute/measures.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace volumetric_kit::calib::compute {
namespace {

namespace camera = recon::camera;

constexpr double kPi = 3.14159265358979323846;

TEST(Median, IsTheMiddleOrTheMeanOfTheMiddleTwo) {
  EXPECT_EQ(median({0.4, 0.1, 0.2}), 0.2);
  EXPECT_DOUBLE_EQ(median({0.4, 0.1, 0.2, 9.0}), 0.3);
  EXPECT_EQ(median({-1.0}), -1.0);
  EXPECT_TRUE(std::isnan(median({})));
}

TEST(AngleDeg, IsAccurateNearZeroAndAHalfTurn) {
  EXPECT_NEAR(angle_deg({0, 0, 1}, {0, 1, 1}), 45.0, 1e-12);
  EXPECT_NEAR(angle_deg({1, 0, 0}, {0, 2, 0}), 90.0, 1e-12);
  // acos would lose these: cos(1e-9 rad) rounds to 1.
  const double tiny = 1e-9;
  EXPECT_NEAR(angle_deg({1, 0, 0}, {std::cos(tiny), std::sin(tiny), 0}),
              tiny * 180.0 / kPi, 1e-18);
  EXPECT_NEAR(angle_deg({1, 0, 0}, {-std::cos(tiny), std::sin(tiny), 0}),
              180.0 - tiny * 180.0 / kPi, 1e-12);
}

TEST(RotationAngleDeg, IsTheAngleOfTheRotationBetween) {
  const camera::Mat3d a = camera::rotation_from_rodrigues({0.3, -0.2, 0.1});
  for (const double deg : {1e-7, 2.0, 90.0, 179.9999}) {
    const camera::Mat3d turn = camera::rotation_from_rodrigues(
        glm::normalize(camera::Vec3d(1, 2, 3)) * (deg * kPi / 180.0));
    EXPECT_NEAR(rotation_angle_deg(a, a * turn), deg, 1e-9 * (1.0 + deg))
        << deg;
    EXPECT_NEAR(rotation_angle_deg(a * turn, a), deg, 1e-9 * (1.0 + deg))
        << deg;
  }
  EXPECT_NEAR(rotation_angle_deg(a, a), 0.0, 1e-12);
}

}  // namespace
}  // namespace volumetric_kit::calib::compute
