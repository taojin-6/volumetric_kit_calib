// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The solve's one projection of a view's corners: private to the tier.

#include <vector>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::solve::detail {

// Each of `view`'s corners' squared reprojection error (pixels^2) with the
// board at `board_to_camera`: moved into the camera in double, then through
// the lens with cv::projectPoints. A corner not in front of the camera (z <= 0,
// or not finite) is infinity. `view`'s ids must be the board's
// (compute::usable_for_pose). Throws what OpenCV throws.
std::vector<double> squared_errors(const compute::CvCamera& camera,
                                   const compute::Board& board,
                                   const Observation& view,
                                   const recon::camera::Mat4d& board_to_camera);

}  // namespace volumetric_kit::calib::solve::detail
