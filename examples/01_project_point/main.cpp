// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// 01_project_point: the smallest end-to-end use of calib_core. Build a
// validated pinhole + Brown-Conrady camera, project a camera-space point to a
// pixel, then recover the undistorted normalized ray. Demonstrates the
// exception-free Result/Status API.

#include <cstdio>
#include <string>

#include <Eigen/Core>

#include "volumetric_kit/calib/core/camera_model.hpp"
#include "volumetric_kit/calib/version.hpp"

namespace vc = volumetric_kit::calib;

int main() {
  std::printf("volumetric_kit_calib %s — project/unproject demo\n\n",
              vc::version_string());

  const vc::PinholeBrownConrady::Intrinsics intrinsics{
      /*fx=*/1000.0, /*fy=*/1000.0, /*cx=*/640.0, /*cy=*/360.0};
  const vc::PinholeBrownConrady::Distortion distortion{
      /*k1=*/-0.28, /*k2=*/0.10, /*p1=*/0.001, /*p2=*/-0.0005, /*k3=*/0.0};

  const auto camera = vc::PinholeBrownConrady::create(intrinsics, distortion);
  if (!camera) {
    std::fprintf(stderr, "failed to build camera: %s: %s\n",
                 std::string(vc::to_string(camera.status().domain())).c_str(),
                 camera.status().message().c_str());
    return 1;
  }

  const Eigen::Vector3d point_camera{0.1, -0.2, 2.0};
  const Eigen::Vector2d pixel = camera->project(point_camera);
  const Eigen::Vector2d normalized = camera->unproject(pixel);

  std::printf("  point (camera) : (%.3f, %.3f, %.3f)\n", point_camera.x(),
              point_camera.y(), point_camera.z());
  std::printf("  -> pixel       : (%.3f, %.3f)\n", pixel.x(), pixel.y());
  std::printf("  -> unprojected : (%.5f, %.5f)  [normalized, z=1]\n",
              normalized.x(), normalized.y());
  return 0;
}
