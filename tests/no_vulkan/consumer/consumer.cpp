// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// An application's use of calib's brain from an installed package: it includes
// each of calib_core's public headers, so each must compile from the install,
// and checks the rig's board and an observation in recon's types, so it links
// calib_core, recon_camera and the core's base tier. With compute
// (VC_CONSUMER_COMPUTE) it includes compute's, detect's and solve's headers
// too, builds the board and the camera in OpenCV's types, seeks the board in a
// blank image, and builds a rig's tree and a view's ambiguity, so it links
// calib_compute, calib_detect, calib_solve and OpenCV.

#include "volumetric_kit/calib/core/capture_meta.hpp"
#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/core/lens_profile.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/core/sensor_profile.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/core/view_check.hpp"
#include "volumetric_kit/calib/version.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"

#ifdef VC_CONSUMER_COMPUTE
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/cv_pose.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/usable_for_pose.hpp"
#include "volumetric_kit/calib/detect/charuco.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/calib/solve/spanning_tree.hpp"
#include "volumetric_kit/calib/solve/view_pose.hpp"
#endif

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
#ifdef VC_CONSUMER_COMPUTE
  namespace compute = vc::compute;
  namespace detect = vc::detect;
  const auto board = compute::Board::create(vc::legacy_femto_board());
  const bool bridged =
      board.ok() && compute::cv_camera(color).ok() &&
      compute::cv_pose(compute::matrix_from_cv_pose({})).ok() &&
      compute::guarded("consumer",
                       [] { return volumetric_kit::core::Status{}; })
          .ok() &&
      vc::solve::spanning_tree({{2, 1}, {1, 2}}, {"A", "B"}, 0, 5)
              ->links.size() == 1 &&
      vc::solve::ViewPose{}.ambiguity() > 1.0;
  if (!bridged) return 1;
  const unsigned char blank[64 * 64] = {};
  const auto found =
      detect::detect_charuco({64, 64, 64, blank}, *board, detect::Mode::Hybrid);
  const bool detected = found.ok() && found->observation.empty() &&
                        found->markers.empty() && found->image_size.width == 64;
  return ok && detected ? 0 : 1;
#else
  return ok ? 0 : 1;
#endif
}
