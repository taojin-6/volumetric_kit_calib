// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// 01_project_point: the smallest end-to-end use of calib_core. Build a
// validated pinhole + Brown-Conrady camera, project a camera-space point to a
// pixel, then recover the undistorted normalized ray. Demonstrates the
// exception-free Result/Status API and its VKC_ASSIGN propagation.

#include <cstdio>
#include <string>

#include <Eigen/Core>

#include "volumetric_kit/calib/core/camera_model.hpp"
#include "volumetric_kit/calib/version.hpp"
#include "volumetric_kit/core/config/config.hpp"

namespace vc = volumetric_kit::calib;
namespace vkc = volumetric_kit::core;

namespace {

// Everything fallible lives here, so VKC_ASSIGN can hand failures to main().
vkc::Status run(const vkc::config::Calibration& settings) {
  const vc::ImageSize image_size{settings.image_width, settings.image_height};
  const vc::PinholeBrownConrady::Intrinsics intrinsics{
      settings.intrinsics[0], settings.intrinsics[1], settings.intrinsics[2],
      settings.intrinsics[3]};
  const vc::PinholeBrownConrady::Distortion distortion{
      settings.distortion[0], settings.distortion[1], settings.distortion[2],
      settings.distortion[3], settings.distortion[4]};

  VKC_ASSIGN(
      const vc::PinholeBrownConrady camera,
      vc::PinholeBrownConrady::create(image_size, intrinsics, distortion));

  const Eigen::Vector3d point_camera{settings.point[0], settings.point[1],
                                     settings.point[2]};
  VKC_ASSIGN(const Eigen::Vector2d pixel, camera.project(point_camera));
  VKC_ASSIGN(const Eigen::Vector2d normalized, camera.unproject(pixel));

  std::printf("  point (camera) : (%.3f, %.3f, %.3f)\n", point_camera.x(),
              point_camera.y(), point_camera.z());
  std::printf("  -> pixel       : (%.3f, %.3f)\n", pixel.x(), pixel.y());
  std::printf("  -> unprojected : (%.5f, %.5f)  [normalized, z=1]\n",
              normalized.x(), normalized.y());
  return {};
}

vkc::Status launch(int argc, const char* const* argv) {
  VKC_ASSIGN(const auto arguments, vkc::config::parse_args(argc, argv));
  if (arguments.help) {
    std::printf("%s", vkc::config::usage());
    return {};
  }
  VKC_ASSIGN(const auto config,
             vkc::config::load(arguments.config_path,
                               vkc::config::Application::CalibProjectPoint));
  std::printf("volumetric_kit_calib %s — project/unproject demo\n\n",
              vc::version_string());
  return run(config.calibration);
}

}  // namespace

int main(int argc, char** argv) {
  const vkc::Status status = launch(argc, argv);
  if (!status) {
    std::fprintf(stderr, "error: %s: %s\n",
                 std::string(vkc::to_string(status.domain())).c_str(),
                 status.message().c_str());
    return 1;
  }
  return 0;
}
