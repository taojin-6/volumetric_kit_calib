// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include <type_traits>

#include <gtest/gtest.h>

#include "volumetric_kit/calib/core/capture_meta.hpp"
#include "volumetric_kit/calib/core/lens_profile.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/core/sensor_profile.hpp"
#include "volumetric_kit/calib/core/view_check.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

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

TEST(Observation, CountsItsPoints) {
  Observation obs;
  EXPECT_TRUE(obs.empty());
  obs.points.push_back({/*id=*/3, {812.4, 390.1}, /*weight=*/400.0});
  obs.points.push_back({/*id=*/4, {850.0, 391.0}, /*weight=*/400.0});
  EXPECT_FALSE(obs.empty());
  EXPECT_EQ(obs.size(), 2u);
}

// A pixel is recon's double vector, so a detection passes to recon's camera
// tier, and from there to OpenCV, unconverted.
TEST(Observation, PixelIsReconsVec2d) {
  static_assert(
      std::is_same_v<decltype(DetectedPoint::pixel), recon::camera::Vec2d>);
  const DetectedPoint unset;
  EXPECT_EQ(unset.pixel, recon::camera::Vec2d(0.0, 0.0));
  EXPECT_EQ(unset.id, -1);
  EXPECT_EQ(unset.weight, 1.0);

  Observation obs;
  obs.points.push_back({/*id=*/5, {3839.25, 0.5}, /*weight=*/1.0});
  EXPECT_EQ(obs.points[0].pixel.x, 3839.25);
  EXPECT_EQ(obs.points[0].pixel.y, 0.5);
}

// A view nothing was detected in has no verdict and no pose measures, so a
// panel or a capture shows them as missing rather than as zero.
TEST(ViewCheck, StartsWithNoDetectionAndNoPose) {
  const ViewCheck check;
  EXPECT_EQ(check.corners, 0);
  EXPECT_EQ(check.markers, 0);
  EXPECT_FALSE(check.collinear);
  EXPECT_FALSE(check.distance_m);
  EXPECT_FALSE(check.tilt_deg);
  EXPECT_FALSE(check.pnp_rms_px);
  EXPECT_FALSE(check.cell);
  EXPECT_EQ(check.verdict, ViewVerdict::None);
}

TEST(CaptureMeta, StartsWithoutALiveVsFullComparison) {
  const CaptureMeta meta;
  EXPECT_EQ(meta.index, 0);
  EXPECT_EQ(meta.time_ns, 0);
  EXPECT_EQ(meta.trigger_ns, 0u);
  EXPECT_EQ(meta.sequence, 0u);
  EXPECT_EQ(meta.spread_ms, 0.0);
  EXPECT_EQ(meta.stillness_px, 0.0);
  EXPECT_FALSE(meta.live_vs_full_px);
}

}  // namespace
}  // namespace volumetric_kit::calib
