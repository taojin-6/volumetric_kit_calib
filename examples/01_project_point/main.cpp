// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// 01_project_point: the smallest end-to-end use of calib's camera model,
// which is recon's camera tier. Build a camera with a Brown-Conrady lens,
// project a camera-space point to a pixel, then recover the undistorted
// normalized ray. Demonstrates the exception-free Result/Status API and its
// VKC_ASSIGN propagation.

#include <cstdio>
#include <string>

#include "volumetric_kit/calib/version.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"

namespace vc = volumetric_kit::calib;
namespace vkc = volumetric_kit::core;
namespace camera = volumetric_kit::recon::camera;

namespace {

// Everything fallible lives here, so VKC_ASSIGN can hand failures to main().
vkc::Status run() {
  const camera::CameraModel model{
      /*size=*/{1280, 720},
      /*intrinsics=*/{/*fx=*/1000.0, /*fy=*/1000.0, /*cx=*/640.0, /*cy=*/360.0},
      /*distortion=*/
      {/*k1=*/-0.28, /*k2=*/0.10, /*p1=*/0.001,
       /*p2=*/-0.0005, /*k3=*/0.0}};
  VKC_ASSIGN(const camera::CameraProjection projection,
             camera::CameraProjection::create(model));

  const camera::Vec3d point_camera{0.1, -0.2, 2.0};
  VKC_ASSIGN(const camera::Vec2d pixel, projection.project(point_camera));
  VKC_ASSIGN(const camera::Vec2d normalized, projection.unproject(pixel));

  std::printf("  point (camera) : (%.3f, %.3f, %.3f)\n", point_camera.x,
              point_camera.y, point_camera.z);
  std::printf("  -> pixel       : (%.3f, %.3f)\n", pixel.x, pixel.y);
  std::printf("  -> unprojected : (%.5f, %.5f)  [normalized, z=1]\n",
              normalized.x, normalized.y);
  return {};
}

}  // namespace

int main() {
  std::printf("volumetric_kit_calib %s — project/unproject demo\n\n",
              vc::version_string());
  const vkc::Status status = run();
  if (!status) {
    std::fprintf(stderr, "error: %s: %s\n",
                 std::string(vkc::to_string(status.domain())).c_str(),
                 status.message().c_str());
    return 1;
  }
  return 0;
}
