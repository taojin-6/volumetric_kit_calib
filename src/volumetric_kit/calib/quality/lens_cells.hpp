// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The 8 x 6 cells' mean residuals that judge_lens's bias gate and the lens
// checks judge: private to the tier.

#include <array>
#include <vector>

#include "volumetric_kit/calib/quality/lens_check.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"

namespace volumetric_kit::calib::quality::detail {

// The standard errors of a cell's mean its limit allows.
enum class CellErrors {
  Three,     // 3: judge_lens's bias gate, its cells of 30 corners or more
  StudentT,  // student_t for the cell's corners: the lens checks'
};

// Each cell of the 8 x 6 grid over an image of `size`: the corners of
// `corners` in it whose residuals are finite and, in a cell of at least
// `min_corners`, their mean residual and its limit: `max_bias_px`, or
// `errors` standard errors of that mean if larger, the error taken as a
// vector (the sample variance about the mean, of both components, over n).
std::array<LensCell, kLensCells> lens_cells(
    const std::vector<const solve::CornerResidual*>& corners,
    sensor::camera::ImageSize size, double max_bias_px, int min_corners,
    CellErrors errors);

// `px` at 3840 wide, scaled to an image of `size`: the lens gates' RMS limits.
inline double at_width(double px, sensor::camera::ImageSize size) noexcept {
  return px * (size.width / 3840.0);
}

}  // namespace volumetric_kit::calib::quality::detail
