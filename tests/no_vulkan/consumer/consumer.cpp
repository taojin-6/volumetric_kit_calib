// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// An application's use of calib's brain from an installed package: it includes
// each of calib_core's public headers, so each must compile from the install,
// and checks the rig's board and an observation in recon's types, so it links
// calib_core, recon_camera and the core's base tier.

#include "volumetric_kit/calib/core/capture_meta.hpp"
#include "volumetric_kit/calib/core/lens_profile.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/core/sensor_profile.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/core/view_check.hpp"
#include "volumetric_kit/calib/version.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"

int main() {
  namespace vc = volumetric_kit::calib;
  namespace camera = volumetric_kit::recon::camera;
  vc::Observation seen;
  seen.points.push_back({0, camera::Vec2d{1920.0, 1080.0}, 1.0});
  const camera::CameraModel color{
      {3840, 2160}, {2240.0, 2240.0, 1919.5, 1079.5}, {}};
  const bool ok = vc::check_target(vc::legacy_femto_board()).ok() &&
                  camera::check_camera_model(color).ok() && seen.size() == 1 &&
                  vc::version_string()[0] != '\0';
  return ok ? 0 : 1;
}
