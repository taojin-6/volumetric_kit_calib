// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file result.hpp
/// @brief calib's error handling: volumetric_kit_core's `Status` and `Result`,
///        named in this namespace.
///
/// calib defines no error types of its own. It uses the family's shared ones,
/// from volumetric_kit_core's base tier (DECISIONS.md, "Build on
/// volumetric_kit_core"), so a `Status` from calib is the same type as one from
/// recon or gfx and passes between them unchanged. The using-declarations below
/// let calib and its consumers keep writing `Status` and `Result<T>` in this
/// namespace.
///
/// Fallible calls return `Status` or `Result<T>`, both `[[nodiscard]]`, since
/// mobile consumers build with `-fno-exceptions`. Propagate with `VKC_TRY` /
/// `VKC_ASSIGN`; a programmer error (reading an error `Result`'s value) aborts
/// via `VKC_CHECK`. Solver failures use `Status::Code::Numerical`. The full
/// contract is in the core's `volumetric_kit/core/base/result.hpp`.
///
/// @code
/// Result<Eigen::Vector2d> pixel_of(const Eigen::Vector3d& point) {
///   VKC_ASSIGN(const PinholeBrownConrady cam,
///              PinholeBrownConrady::create({1280, 720},
///                                          {1000, 1000, 640, 360}));
///   return cam.project(point);  // an error Result propagates as it is
/// }
/// @endcode

#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib {

using core::Result;
using core::Status;
using core::to_string;

}  // namespace volumetric_kit::calib
