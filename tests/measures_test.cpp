// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The measures the solve and the checks share: a median, numpy's linear
// quantile, the angles between directions and between rotations, accurate
// near 0 and 180 degrees, and whether a transform is finite.

#include "volumetric_kit/calib/compute/measures.hpp"

#include <cmath>
#include <limits>
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

// numpy's percentile, linear: np.percentile([1, 2, 3, 4], [90, 50, 0, 100])
// is [3.7, 2.5, 1, 4].
TEST(Quantile, InterpolatesBetweenRanksAsNumpyDoes) {
  const std::vector<double> values = {4.0, 1.0, 3.0, 2.0};
  EXPECT_DOUBLE_EQ(quantile(values, 0.9), 3.7);
  EXPECT_EQ(quantile(values, 0.5), 2.5);
  EXPECT_EQ(quantile(values, 0.0), 1.0);
  EXPECT_EQ(quantile(values, 1.0), 4.0);
  EXPECT_EQ(quantile({7.0}, 0.9), 7.0);
  EXPECT_TRUE(std::isnan(quantile({}, 0.5)));
  for (const double q : {-0.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_TRUE(std::isnan(quantile(values, q))) << q;
  }
}

// An infinite neighbour gives that infinity, never inf - inf; a NaN sorts
// above every number.
TEST(Quantile, KeepsInfinitiesAndSortsANanLast) {
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(quantile({inf, 1.0, inf}, 0.9), inf);  // between two infinities
  EXPECT_EQ(quantile({1.0, 2.0, inf}, 0.9), inf);
  EXPECT_EQ(quantile({-inf, 1.0}, 0.5), -inf);
  EXPECT_EQ(quantile({nan, 3.0, 1.0, 2.0}, 0.5), 2.5);
  EXPECT_EQ(quantile({2.0, nan, 1.0}, 0.5), 2.0);
  EXPECT_TRUE(std::isnan(quantile({2.0, nan, 1.0}, 1.0)));
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

TEST(Finite, IsFalseForAnyOfTheSixNumbersNotFinite) {
  const camera::RodriguesTransform t{{0.3, -0.2, 0.1}, {1.0, 2.0, -3.0}};
  EXPECT_TRUE(finite(t));
  EXPECT_TRUE(finite(camera::RodriguesTransform{}));
  for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()}) {
    for (int i = 0; i < 3; ++i) {
      camera::RodriguesTransform r = t;
      r.rvec[i] = bad;
      EXPECT_FALSE(finite(r)) << "rvec " << i << ' ' << bad;
      camera::RodriguesTransform s = t;
      s.tvec[i] = bad;
      EXPECT_FALSE(finite(s)) << "tvec " << i << ' ' << bad;
    }
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::compute
