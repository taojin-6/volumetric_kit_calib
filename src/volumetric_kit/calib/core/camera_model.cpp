// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/core/camera_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace volumetric_kit::calib {
namespace {

using Distortion = PinholeBrownConrady::Distortion;

// unproject() stops once the re-projection error is below this. Far finer than
// any detector, yet ~100x above double rounding at the widest radii.
constexpr double kUnprojectTolerancePx = 1e-9;
constexpr int kUnprojectMaxIterations = 50;
constexpr int kMaxStepHalvings = 30;

// Brown-Conrady distortion of an undistorted normalized point.
Eigen::Vector2d distort(const Distortion& d, const Eigen::Vector2d& u) {
  const double x = u.x();
  const double y = u.y();
  const double r2 = x * x + y * y;
  const double radial = 1.0 + r2 * (d.k1 + r2 * (d.k2 + r2 * d.k3));
  return {x * radial + 2.0 * d.p1 * x * y + d.p2 * (r2 + 2.0 * x * x),
          y * radial + d.p1 * (r2 + 2.0 * y * y) + 2.0 * d.p2 * x * y};
}

// Jacobian of distort() with respect to the undistorted point.
Eigen::Matrix2d distort_jacobian(const Distortion& d,
                                 const Eigen::Vector2d& u) {
  const double x = u.x();
  const double y = u.y();
  const double r2 = x * x + y * y;
  const double radial = 1.0 + r2 * (d.k1 + r2 * (d.k2 + r2 * d.k3));
  const double dradial_dr2 = d.k1 + r2 * (2.0 * d.k2 + 3.0 * r2 * d.k3);
  const double cross =
      2.0 * x * y * dradial_dr2 + 2.0 * d.p1 * x + 2.0 * d.p2 * y;
  Eigen::Matrix2d j;
  j << radial + 2.0 * x * x * dradial_dr2 + 2.0 * d.p1 * y + 6.0 * d.p2 * x,
      cross, cross,
      radial + 2.0 * y * y * dradial_dr2 + 6.0 * d.p1 * y + 2.0 * d.p2 * x;
  return j;
}

// The smallest s = r^2 > 0 at which the radial distortion's radius,
// r * (1 + k1 s + k2 s^2 + k3 s^3), stops increasing: the first positive root
// of its derivative, 1 + 3 k1 s + 5 k2 s^2 + 7 k3 s^3. +infinity if none.
double first_fold_r2(const Distortion& d) {
  const double a = 3.0 * d.k1;
  const double b = 5.0 * d.k2;
  const double c = 7.0 * d.k3;
  const auto slope = [&](double s) { return 1.0 + s * (a + s * (b + s * c)); };
  // Returns the last s in [lo, hi] with a positive slope, for a slope that is
  // positive at lo, non-positive at hi, and monotone in between.
  const auto bisect = [&](double lo, double hi) {
    for (int i = 0; i < 200 && lo < hi; ++i) {
      const double mid = 0.5 * (lo + hi);
      if (mid <= lo || mid >= hi) break;
      (slope(mid) > 0.0 ? lo : hi) = mid;
    }
    return lo;
  };

  // The cubic is monotone between its critical points (the roots of
  // a + 2 b s + 3 c s^2), so its first sign change is found piece by piece.
  double breaks[2];
  int n = 0;
  if (c != 0.0) {
    const double disc = b * b - 3.0 * a * c;
    if (disc >= 0.0) {
      double s1 = (-b - std::sqrt(disc)) / (3.0 * c);
      double s2 = (-b + std::sqrt(disc)) / (3.0 * c);
      if (s1 > s2) std::swap(s1, s2);
      if (s1 > 0.0) breaks[n++] = s1;
      if (s2 > 0.0) breaks[n++] = s2;
    }
  } else if (b != 0.0) {
    const double s = -a / (2.0 * b);
    if (s > 0.0) breaks[n++] = s;
  }

  double lo = 0.0;
  for (int i = 0; i < n; ++i) {
    if (slope(breaks[i]) <= 0.0) return bisect(lo, breaks[i]);
    lo = breaks[i];
  }
  // The last piece is monotone out to infinity. Past r = 1000 (89.94 degrees
  // off-axis) no pinhole lens images anything, so stop looking there.
  constexpr double kMaxR2 = 1e6;
  double hi = std::max(1.0, 2.0 * lo);
  while (hi < kMaxR2 && slope(hi) > 0.0) hi *= 2.0;
  if (slope(hi) > 0.0) return std::numeric_limits<double>::infinity();
  return bisect(lo, hi);
}

}  // namespace

PinholeBrownConrady::PinholeBrownConrady(const ImageSize& image_size,
                                         const Intrinsics& intrinsics,
                                         const Distortion& distortion)
    : image_size_(image_size),
      intrinsics_(intrinsics),
      distortion_(distortion),
      fold_r2_(first_fold_r2(distortion)) {}

Result<PinholeBrownConrady> PinholeBrownConrady::create(
    const ImageSize& image_size, const Intrinsics& intrinsics,
    const Distortion& distortion) {
  if (image_size.width <= 0 || image_size.height <= 0) {
    return Status::invalid_argument("image width and height must be positive");
  }
  if (!std::isfinite(intrinsics.fx) || !std::isfinite(intrinsics.fy) ||
      !std::isfinite(intrinsics.cx) || !std::isfinite(intrinsics.cy)) {
    return Status::invalid_argument("intrinsics must be finite");
  }
  if (intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
    return Status::invalid_argument("focal lengths fx, fy must be positive");
  }
  if (!std::isfinite(distortion.k1) || !std::isfinite(distortion.k2) ||
      !std::isfinite(distortion.p1) || !std::isfinite(distortion.p2) ||
      !std::isfinite(distortion.k3)) {
    return Status::invalid_argument("distortion coefficients must be finite");
  }
  return PinholeBrownConrady{image_size, intrinsics, distortion};
}

Result<PinholeBrownConrady> PinholeBrownConrady::create(
    const ImageSize& image_size, const Intrinsics& intrinsics) {
  return create(image_size, intrinsics, Distortion{});
}

Result<Eigen::Vector2d> PinholeBrownConrady::project(
    const Eigen::Vector3d& point_camera) const {
  if (!point_camera.allFinite()) {
    return Status::invalid_argument("project() needs a finite point");
  }
  if (point_camera.z() <= 0.0) {
    return Status::invalid_argument(
        "project() needs a point in front of the camera (z > 0)");
  }
  const Eigen::Vector2d distorted =
      distort(distortion_, point_camera.head<2>() / point_camera.z());
  const Eigen::Vector2d pixel{intrinsics_.fx * distorted.x() + intrinsics_.cx,
                              intrinsics_.fy * distorted.y() + intrinsics_.cy};
  if (!pixel.allFinite()) {
    return Status::numerical("project() overflowed: point is too far off-axis");
  }
  return pixel;
}

Result<Eigen::Vector2d> PinholeBrownConrady::unproject(
    const Eigen::Vector2d& pixel) const {
  if (!pixel.allFinite()) {
    return Status::invalid_argument("unproject() needs a finite pixel");
  }
  const Eigen::Vector2d target{(pixel.x() - intrinsics_.cx) / intrinsics_.fx,
                               (pixel.y() - intrinsics_.cy) / intrinsics_.fy};
  // Measured in pixels, so the tolerance means the same at any focal length.
  const auto error_px = [&](const Eigen::Vector2d& residual) {
    return std::hypot(intrinsics_.fx * residual.x(),
                      intrinsics_.fy * residual.y());
  };

  // Newton's method on distort(u) = target, starting from the distorted point
  // (exact for no distortion). Every iterate stays inside the fold, so the
  // search cannot settle on a folded-back ray that also lands on the pixel.
  Eigen::Vector2d u = target;
  if (u.squaredNorm() >= fold_r2_) {
    u *= std::sqrt(0.5 * fold_r2_ / u.squaredNorm());
  }
  Eigen::Vector2d residual = distort(distortion_, u) - target;
  double error = error_px(residual);
  for (int iteration = 0; iteration < kUnprojectMaxIterations; ++iteration) {
    const Eigen::Matrix2d j = distort_jacobian(distortion_, u);
    const double det = j(0, 0) * j(1, 1) - j(0, 1) * j(1, 0);
    if (!(det > 0.0)) break;  // not locally invertible here
    if (error <= kUnprojectTolerancePx) return u;

    // Solve j * step = -residual (2x2, by the adjugate).
    const Eigen::Vector2d step{
        (-j(1, 1) * residual.x() + j(0, 1) * residual.y()) / det,
        (j(1, 0) * residual.x() - j(0, 0) * residual.y()) / det};
    // Backtrack until the step reduces the error without leaving the fold.
    bool improved = false;
    double t = 1.0;
    for (int halving = 0; halving < kMaxStepHalvings; ++halving, t *= 0.5) {
      const Eigen::Vector2d candidate = u + t * step;
      if (candidate.squaredNorm() >= fold_r2_) continue;
      const Eigen::Vector2d candidate_residual =
          distort(distortion_, candidate) - target;
      const double candidate_error = error_px(candidate_residual);
      if (candidate_error < error) {
        u = candidate;
        residual = candidate_residual;
        error = candidate_error;
        improved = true;
        break;
      }
    }
    if (!improved) break;
  }
  return Status::numerical(
      "unproject(): no ray inside the distortion's invertible region maps to "
      "this pixel");
}

}  // namespace volumetric_kit::calib
