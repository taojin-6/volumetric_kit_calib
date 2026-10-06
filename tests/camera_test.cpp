// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib's camera model is recon's camera tier (camera/camera_model.hpp).
// These tests hold, from calib's side, what calib relies on: the
// Brown-Conrady lens calib modelled before unprojects and projects back across
// the whole image; GLM's double matrices are Eigen's in place, so the solver
// works on them without copying; and the calibration file calib will write
// reads back as written.

#include <cmath>
#include <string>

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <glm/gtc/type_ptr.hpp>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace camera = volumetric_kit::recon::camera;

namespace {

// The lens calib's own model was tested with: a 1280x720 camera, strong
// barrel distortion, a little tangential.
const camera::CameraModel kBarrel{{1280, 720},
                                  {1000.0, 1000.0, 640.0, 360.0},
                                  {-0.28, 0.10, 0.001, -0.0005, 0.0}};

TEST(Camera, BrownConradyRoundTripsAcrossTheImage) {
  const auto projection = camera::CameraProjection::create(kBarrel);
  ASSERT_TRUE(projection.ok()) << projection.status().message();
  for (int i = 0; i <= 32; ++i) {
    for (int k = 0; k <= 18; ++k) {
      const camera::Vec2d pixel(-0.5 + 1280.0 * i / 32.0,
                                -0.5 + 720.0 * k / 18.0);
      const auto ray = projection.value().unproject(pixel);
      ASSERT_TRUE(ray.ok()) << ray.status().message();
      const auto back =
          projection.value().project({ray.value().x, ray.value().y, 1.0});
      ASSERT_TRUE(back.ok()) << back.status().message();
      EXPECT_LT(std::hypot(back.value().x - pixel.x, back.value().y - pixel.y),
                1e-9);
    }
  }
}

TEST(Camera, GlmMatricesAreEigenMatricesInPlace) {
  // Both column-major, so the solver maps a pose rather than copying it.
  const camera::Mat4d pose =
      camera::matrix_from_rodrigues({{0.3, -0.8, 0.52}, {1.25, -2.5, 0.004}});
  const Eigen::Map<const Eigen::Matrix4d> eigen(glm::value_ptr(pose));
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      EXPECT_EQ(eigen(row, col), pose[col][row]);
    }
  }
}

TEST(Camera, CalibrationFileReadsBackAsWritten) {
  camera::ArrayCalibration written;
  written.world.kind = camera::WorldFrame::Kind::AprilTag;
  written.world.tag = {"tag36h11", 0, 0.22};
  for (const char* id : {"CL2A141000N", "CL2A141000G"}) {
    camera::SensorCalibration s;
    s.id = id;
    s.color_to_world = camera::rigid_inverse(
        camera::matrix_from_rodrigues({{0.0, 0.4, 0.0}, {0.1, 0.0, 2.5}}));
    s.color = camera::CameraCalibration{kBarrel,
                                        camera::IntrinsicsSource::Calibrated};
    written.sensors.push_back(s);
  }
  const std::string path =
      std::string(VC_TEST_SCRATCH_DIR) + "/array_calibration.json";
  ASSERT_TRUE(camera::write_array_calibration(path, written).ok());
  const auto read = camera::read_array_calibration(path);
  ASSERT_TRUE(read.ok()) << read.status().message();
  ASSERT_EQ(read.value().sensors.size(), 2u);
  EXPECT_EQ(read.value().world.kind, camera::WorldFrame::Kind::AprilTag);
  for (const camera::SensorCalibration& s : read.value().sensors) {
    ASSERT_TRUE(s.color.has_value());
    EXPECT_EQ(s.color->source, camera::IntrinsicsSource::Calibrated);
    EXPECT_EQ(s.color->model.intrinsics.fx, kBarrel.intrinsics.fx);
    EXPECT_EQ(s.color->model.distortion.p2, kBarrel.distortion.p2);
  }
}

}  // namespace
