// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// 01_project_point: the smallest end-to-end use of calib's camera model, which
// is recon's. Map the scenario's camera into a recon CameraModel, check it,
// and project a camera-space point to a pixel through its lens. Demonstrates
// the exception-free Result/Status API and its VKC_TRY propagation.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "volumetric_kit/calib/version.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"
#include "volumetric_kit/recon/camera/projection.hpp"

namespace vc = volumetric_kit::calib;
namespace vkc = volumetric_kit::core;
namespace camera = volumetric_kit::recon::camera;

namespace {

// Everything fallible lives here, so VKC_TRY can hand failures to main().
vkc::Status run(const vkc::config::Calibration& settings) {
  // The scenario's five coefficients are Brown-Conrady: the rational model
  // with k4 = k5 = k6 = 0.
  const camera::CameraModel model{
      {static_cast<std::uint32_t>(settings.image_width),
       static_cast<std::uint32_t>(settings.image_height)},
      {settings.intrinsics[0], settings.intrinsics[1], settings.intrinsics[2],
       settings.intrinsics[3]},
      {settings.distortion[0], settings.distortion[1], settings.distortion[2],
       settings.distortion[3], settings.distortion[4], 0.0, 0.0, 0.0}};
  VKC_TRY(camera::check_camera_model(model));

  const camera::Vec3d point{settings.point[0], settings.point[1],
                            settings.point[2]};
  const camera::RationalDistortion& d = model.distortion;
  const double lens[8] = {d.k1, d.k2, d.p1, d.p2, d.k3, d.k4, d.k5, d.k6};
  camera::Vec2d distorted;
  camera::distort_rational(lens, point.x / point.z, point.y / point.z,
                           &distorted.x, &distorted.y);
  const camera::PinholeIntrinsics& k = model.intrinsics;
  const camera::Vec2d pixel{k.fx * distorted.x + k.cx,
                            k.fy * distorted.y + k.cy};
  // A valid lens can still overflow far enough off-axis, and
  // distort_rational checks nothing.
  if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y)) {
    return vkc::Status::numerical("the point is too far off-axis for the lens");
  }

  std::printf("  point (camera) : (%.3f, %.3f, %.3f)\n", point.x, point.y,
              point.z);
  std::printf("  -> pixel       : (%.3f, %.3f)\n", pixel.x, pixel.y);
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
  std::printf("volumetric_kit_calib %s — projection demo\n\n",
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
