// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file camera_model.hpp
/// @brief Projective camera models: map between 3D camera-space rays and 2D
///        pixels, with lens distortion.
///
/// @ref PinholeBrownConrady is the default model (a pinhole with 5-parameter
/// Brown-Conrady radial + tangential distortion), matching the OpenCV
/// convention `(k1, k2, p1, p2, k3)`. It is the first concrete model; the
/// fisheye (Kannala-Brandt) and rational variants in DESIGN.md follow the same
/// project/unproject shape.
///
/// @code
/// auto cam = PinholeBrownConrady::create({.fx = 1000, .fy = 1000,
///                                         .cx = 640, .cy = 360});
/// if (!cam) return cam.status();
/// Eigen::Vector2d px = cam->project({0.0, 0.0, 1.0});  // -> (640, 360)
/// @endcode

#include <Eigen/Core>

#include "volumetric_kit/calib/core/export.hpp"
#include "volumetric_kit/calib/core/result.hpp"

namespace volumetric_kit::calib {

/// @brief Pinhole camera with 5-parameter Brown-Conrady distortion.
///
/// Construct via @ref create, which validates the intrinsics (positive, finite
/// focal lengths). `project` is closed-form; `unproject` removes distortion
/// iteratively.
///
/// @note TODO: this will become one alternative of a `CameraModel` variant
/// (adding `FisheyeKB`, `Rational`) once the solve tier needs polymorphic
/// dispatch; analytic parameter/pose Jacobians (`dproject_d*`) land with it.
class VC_CORE_API PinholeBrownConrady {
 public:
  /// @brief Pinhole intrinsics in pixels.
  struct Intrinsics {
    double fx = 0.0;  ///< Focal length along x (pixels).
    double fy = 0.0;  ///< Focal length along y (pixels).
    double cx = 0.0;  ///< Principal point x (pixels).
    double cy = 0.0;  ///< Principal point y (pixels).
  };

  /// @brief Brown-Conrady distortion coefficients (OpenCV order).
  struct Distortion {
    double k1 = 0.0;  ///< Radial.
    double k2 = 0.0;  ///< Radial.
    double p1 = 0.0;  ///< Tangential.
    double p2 = 0.0;  ///< Tangential.
    double k3 = 0.0;  ///< Radial (3rd order).
  };

  /// @brief Build a validated camera model with the given distortion.
  /// @param intrinsics  Pinhole intrinsics; `fx`, `fy` must be finite and > 0.
  /// @param distortion  Distortion coefficients.
  /// @return The model, or @ref Status::Code::InvalidArgument if the intrinsics
  ///         are non-finite or non-positive.
  static Result<PinholeBrownConrady> create(const Intrinsics& intrinsics,
                                            const Distortion& distortion);

  /// @brief Build a validated, distortion-free camera model.
  /// @param intrinsics  Pinhole intrinsics; `fx`, `fy` must be finite and > 0.
  /// @return The model, or @ref Status::Code::InvalidArgument on bad
  /// intrinsics.
  /// @note A nested aggregate's default-member-initializer cannot appear in a
  ///       `= {}` default argument here, so this is a separate overload rather
  ///       than a default parameter.
  static Result<PinholeBrownConrady> create(const Intrinsics& intrinsics);

  /// @brief Project a camera-space point to a pixel.
  /// @param point_camera  A point in camera coordinates.
  /// @pre `point_camera.z() > 0` (the point is in front of the camera).
  /// @return The distorted pixel coordinate.
  Eigen::Vector2d project(const Eigen::Vector3d& point_camera) const;

  /// @brief Recover the undistorted normalized image point for a pixel.
  /// @param pixel       A pixel coordinate.
  /// @param iterations  Fixed-point iterations for distortion removal.
  /// @return The normalized image point `(x, y)` at `z = 1` (i.e. the direction
  ///         `(x, y, 1)` up to scale), with distortion removed.
  Eigen::Vector2d unproject(const Eigen::Vector2d& pixel,
                            int iterations = 20) const;

  /// @return The pinhole intrinsics.
  const Intrinsics& intrinsics() const noexcept { return intrinsics_; }
  /// @return The distortion coefficients.
  const Distortion& distortion() const noexcept { return distortion_; }

 private:
  PinholeBrownConrady(const Intrinsics& intrinsics,
                      const Distortion& distortion)
      : intrinsics_(intrinsics), distortion_(distortion) {}

  Intrinsics intrinsics_;
  Distortion distortion_;
};

}  // namespace volumetric_kit::calib
