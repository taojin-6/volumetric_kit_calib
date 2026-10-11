// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/core/lens_profile.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/core/sensor_profile.hpp"
#include "volumetric_kit/calib/core/target.hpp"

#include <gtest/gtest.h>

namespace volumetric_kit::calib {
namespace {

// The quality tier reports a missing input rather than assuming one, so a
// default-constructed profile must know nothing.
TEST(SensorProfile, StartsWithEveryFieldUnknown) {
  const SensorProfile s;
  EXPECT_FALSE(s.pixel_pitch_um);
  EXPECT_FALSE(s.width_px);
  EXPECT_FALSE(s.height_px);
  EXPECT_FALSE(s.read_noise_e);
  EXPECT_FALSE(s.full_well_e);
  EXPECT_FALSE(s.gain_e_per_dn);
  EXPECT_FALSE(s.quantum_efficiency);
  EXPECT_FALSE(s.dark_current_e_per_s);
  EXPECT_FALSE(s.rolling_shutter_line_time_us);
}

TEST(LensProfile, StartsWithEveryFieldUnknown) {
  const LensProfile lens;
  EXPECT_FALSE(lens.focal_length_mm);
  EXPECT_FALSE(lens.f_number);
  EXPECT_FALSE(lens.focus_distance_m);
  EXPECT_FALSE(lens.has_image_stabilization);
  EXPECT_FALSE(lens.has_autofocus);
  EXPECT_TRUE(lens.mount.empty());
}

TEST(Target, TolerancesStartUnknown) {
  const Target board;
  EXPECT_FALSE(board.square_size_tolerance_m);
  EXPECT_FALSE(board.planarity_bow_m);
}

TEST(Observation, CountsItsPoints) {
  Observation obs;
  EXPECT_TRUE(obs.empty());
  obs.points.push_back({/*id=*/3, {812.4, 390.1}, /*weight=*/400.0});
  obs.points.push_back({/*id=*/4, {850.0, 391.0}, /*weight=*/400.0});
  EXPECT_FALSE(obs.empty());
  EXPECT_EQ(obs.size(), 2u);
}

}  // namespace
}  // namespace volumetric_kit::calib
