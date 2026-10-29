// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file measures.hpp
/// @brief The small measures the solve and the checks share: a median and a
///        quantile, the angle between two directions, the angle between two
///        rotations, and whether a transform is finite.

#include <vector>

#include "volumetric_kit/calib/compute/export.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"

namespace volumetric_kit::calib::compute {

/// @brief The median of @p values: the middle one, or the mean of the middle
///        two of an even count.
/// @param values  The values, none not a number; infinities sort as numbers.
/// @return The median; NaN for no values.
///
/// @code
/// const double m = median({0.2, 0.1, 0.4});  // 0.2
/// @endcode
VC_COMPUTE_API double median(std::vector<double> values) noexcept;

/// @brief The @p q quantile of @p values, interpolated linearly between ranks
///        as numpy's `percentile` is by default: at rank `q (n - 1)` of the
///        values in ascending order.
///
/// An infinite neighbour gives that infinity, never `inf - inf`; a NaN sorts
/// above every number.
/// @param values  The values.
/// @param q       The quantile, in [0, 1]: 0.9 for the 90th percentile, 1 for
///                the largest.
/// @return The quantile; NaN for no values or a @p q outside [0, 1].
///
/// @code
/// const double p90 = quantile({1.0, 2.0, 3.0, 4.0}, 0.9);  // 3.7
/// @endcode
VC_COMPUTE_API double quantile(std::vector<double> values, double q) noexcept;

/// @brief The angle between two directions, accurate near 0 and 180 degrees
///        (from `atan2`, not `acos`).
/// @param a  A direction; any length but 0.
/// @param b  Another.
/// @return The angle (degrees), in [0, 180].
///
/// @code
/// const double deg = angle_deg({0, 0, 1}, {0, 1, 1});  // 45
/// @endcode
VC_COMPUTE_API double angle_deg(const sensor::camera::Vec3d& a,
                                const sensor::camera::Vec3d& b) noexcept;

/// @brief The angle of the rotation from @p a to @p b, accurate near 0 and
///        180 degrees (sensor's `rodrigues_from_rotation`).
/// @pre @p a and @p b are rotations.
/// @param a  A rotation.
/// @param b  Another.
/// @return The angle (degrees), in [0, 180].
///
/// @code
/// const double deg = rotation_angle_deg(sensor::camera::Mat3d(T_ab),
///                                       sensor::camera::Mat3d(T_ac));
/// @endcode
VC_COMPUTE_API double rotation_angle_deg(
    const sensor::camera::Mat3d& a, const sensor::camera::Mat3d& b) noexcept;

/// @brief Whether a transform's six numbers are finite.
/// @param transform  A transform, as OpenCV writes one.
/// @return False when a component of its `rvec` or `tvec` is infinite or not
///         a number.
///
/// @code
/// if (!finite(solution.world_to_camera[c])) return refuse(c);
/// @endcode
VC_COMPUTE_API bool finite(
    const sensor::camera::RodriguesTransform& transform) noexcept;

}  // namespace volumetric_kit::calib::compute
