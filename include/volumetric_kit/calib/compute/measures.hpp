// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file measures.hpp
/// @brief The small measures the solve and the checks share: a median, the
///        angle between two directions, and the angle between two rotations.

#include <vector>

#include "volumetric_kit/calib/compute/export.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::compute {

/// @brief The median of @p values: the middle one, or the mean of the middle
///        two of an even count.
/// @param values  The values, finite.
/// @return The median; NaN for no values.
///
/// @code
/// const double m = median({0.2, 0.1, 0.4});  // 0.2
/// @endcode
VC_COMPUTE_API double median(std::vector<double> values) noexcept;

/// @brief The angle between two directions, accurate near 0 and 180 degrees
///        (from `atan2`, not `acos`).
/// @param a  A direction; any length but 0.
/// @param b  Another.
/// @return The angle (degrees), in [0, 180].
///
/// @code
/// const double deg = angle_deg({0, 0, 1}, {0, 1, 1});  // 45
/// @endcode
VC_COMPUTE_API double angle_deg(const recon::camera::Vec3d& a,
                                const recon::camera::Vec3d& b) noexcept;

/// @brief The angle of the rotation from @p a to @p b, accurate near 0 and
///        180 degrees (recon's `rodrigues_from_rotation`).
/// @pre @p a and @p b are rotations.
/// @param a  A rotation.
/// @param b  Another.
/// @return The angle (degrees), in [0, 180].
///
/// @code
/// const double deg = rotation_angle_deg(recon::camera::Mat3d(T_ab),
///                                       recon::camera::Mat3d(T_ac));
/// @endcode
VC_COMPUTE_API double rotation_angle_deg(
    const recon::camera::Mat3d& a, const recon::camera::Mat3d& b) noexcept;

}  // namespace volumetric_kit::calib::compute
