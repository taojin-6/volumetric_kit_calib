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
/// project/unproject shape. Every model records the @ref ImageSize it was
/// calibrated at, because intrinsics only hold at that resolution.

#include <Eigen/Core>

#include "volumetric_kit/calib/core/export.hpp"
#include "volumetric_kit/calib/core/result.hpp"

namespace volumetric_kit::calib {

/// @brief The image resolution a camera model is calibrated at, in pixels.
///
/// Binning, cropping or another capture mode changes the intrinsics, so a
/// model always records the size it describes. It can differ from the active
/// sensor area in `SensorProfile`.
///
/// @code
/// const ImageSize uhd{3840, 2160};
/// @endcode
struct ImageSize {
  int width = 0;   ///< Image width (pixels); must be > 0.
  int height = 0;  ///< Image height (pixels); must be > 0.
};

/// @brief Pinhole camera with 5-parameter Brown-Conrady distortion.
///
/// Construct via @ref create, which validates every parameter. `project` is
/// closed-form. `unproject` inverts the distortion with Newton's method and
/// returns an error, never a wrong answer, when a pixel has no inverse the
/// model can vouch for.
///
/// @code
/// Result<Eigen::Vector2d> ray_through(const Eigen::Vector3d& point) {
///   VKC_ASSIGN(const PinholeBrownConrady cam,
///              PinholeBrownConrady::create(
///                  {1280, 720},
///                  {/*fx=*/1000, /*fy=*/1000, /*cx=*/640, /*cy=*/360},
///                  {/*k1=*/-0.28, /*k2=*/0.10, /*p1=*/0, /*p2=*/0,
///                   /*k3=*/0}));
///   VKC_ASSIGN(const Eigen::Vector2d pixel, cam.project(point));
///   return cam.unproject(pixel);  // point.head<2>() / point.z(), recovered
/// }
/// @endcode
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
  /// @param image_size  The calibrated resolution; both sides must be > 0.
  /// @param intrinsics  Pinhole intrinsics; all finite, with `fx`, `fy` > 0.
  /// @param distortion  Distortion coefficients; all finite.
  /// @return The model, or @ref Status::Code::InvalidArgument naming the
  ///         parameter that failed validation.
  static Result<PinholeBrownConrady> create(const ImageSize& image_size,
                                            const Intrinsics& intrinsics,
                                            const Distortion& distortion);

  /// @brief Build a validated, distortion-free camera model.
  /// @param image_size  The calibrated resolution; both sides must be > 0.
  /// @param intrinsics  Pinhole intrinsics; all finite, with `fx`, `fy` > 0.
  /// @return The model, or @ref Status::Code::InvalidArgument naming the
  ///         parameter that failed validation.
  /// @note A nested aggregate's default-member-initializer cannot appear in a
  ///       `= {}` default argument here, so this is a separate overload rather
  ///       than a default parameter.
  static Result<PinholeBrownConrady> create(const ImageSize& image_size,
                                            const Intrinsics& intrinsics);

  /// @brief Project a camera-space point to a pixel.
  /// @param point_camera  A point in camera coordinates.
  /// @return The distorted pixel coordinate, which may lie outside the image;
  ///         @ref Status::Code::InvalidArgument if the point is non-finite or
  ///         not in front of the camera (`z <= 0`), or
  ///         @ref Status::Code::Numerical if the projection overflows.
  Result<Eigen::Vector2d> project(const Eigen::Vector3d& point_camera) const;

  /// @brief Recover the undistorted normalized image point for a pixel.
  /// @param pixel  A pixel coordinate.
  /// @return The normalized point `(x, y)` at `z = 1` (the ray `(x, y, 1)` up
  ///         to scale) that @ref project maps to within 1e-9 px of @p pixel;
  ///         @ref Status::Code::InvalidArgument for a non-finite pixel; or
  ///         @ref Status::Code::Numerical if no such point exists inside the
  ///         radius where the radial distortion folds back on itself. Past
  ///         that fold, strong distortion maps several rays to one pixel, so
  ///         the model cannot say which is right.
  Result<Eigen::Vector2d> unproject(const Eigen::Vector2d& pixel) const;

  /// @return The resolution the model is calibrated at.
  const ImageSize& image_size() const noexcept { return image_size_; }
  /// @return The pinhole intrinsics.
  const Intrinsics& intrinsics() const noexcept { return intrinsics_; }
  /// @return The distortion coefficients.
  const Distortion& distortion() const noexcept { return distortion_; }

 private:
  PinholeBrownConrady(const ImageSize& image_size, const Intrinsics& intrinsics,
                      const Distortion& distortion);

  ImageSize image_size_;
  Intrinsics intrinsics_;
  Distortion distortion_;
  /// Squared normalized radius at which the radial distortion's radius stops
  /// increasing (its first fold); +infinity when it never does.
  double fold_r2_;
};

}  // namespace volumetric_kit::calib
