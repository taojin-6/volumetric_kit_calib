// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file depth_scene.hpp
/// @brief Depth images rendered in double for the depth checks' tests: a
///        prepared depth's pinhole, the surfaces its rays meet, the depth
///        each pixel sees, prepared or raw through a lens, and turns to pose
///        them. Its users link vc_synthetic_rig.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include "synthetic_rig.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"
#include "volumetric_kit/recon/camera/projection.hpp"

namespace volumetric_kit::calib::test {

/// @brief A turn of @p deg degrees about @p axis.
/// @param axis  The axis, of any length but 0.
/// @param deg   The angle, right-handed about @p axis (degrees).
/// @return The rotation.
///
/// @code
/// const recon::camera::Mat3d down = rotation({1.0, 0.0, 0.0}, -5.5);
/// @endcode
inline recon::camera::Mat3d rotation(const recon::camera::Vec3d& axis,
                                     double deg) {
  return recon::camera::rotation_from_rodrigues(glm::normalize(axis) * deg *
                                                glm::pi<double>() / 180.0);
}

/// @brief A prepared depth's pinhole, its pixel centres at integers.
///
/// @code
/// const Pinhole p = nfov();
/// const recon::camera::Vec3d ray = ray_of(p, 320, 288);
/// @endcode
struct Pinhole {
  recon::camera::ImageSize size;       ///< The image's size.
  recon::camera::PinholeIntrinsics k;  ///< Its intrinsics.
};

/// @return A Femto's narrow depth: @ref depth_like_lens's pinhole.
inline Pinhole nfov() {
  const recon::camera::CameraModel lens = depth_like_lens();
  return {lens.size, lens.intrinsics};
}
/// @return The narrow depth binned 2 x 2.
inline Pinhole binned() { return {{320, 288}, {252.1, 252.2, 160.3, 165.1}}; }
/// @return A wide depth.
inline Pinhole wide() { return {{512, 512}, {252.4, 252.5, 255.6, 256.2}}; }

/// @brief Pixel (@p u, @p v)'s ray at depth 1.
/// @param p  The pinhole.
/// @param u  The column.
/// @param v  The row.
/// @return The ray, in the camera's frame.
inline recon::camera::Vec3d ray_of(const Pinhole& p, std::uint32_t u,
                                   std::uint32_t v) {
  return {(u - p.k.cx) / p.k.fx, (v - p.k.cy) / p.k.fy, 1.0};
}

/// @brief A plane in a scene's frame.
///
/// @code
/// const Plane floor{{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}};
/// @endcode
struct Plane {
  recon::camera::Vec3d point;   ///< A point on it.
  recon::camera::Vec3d normal;  ///< Its normal, of any length but 0.
};

/// @brief Where a ray from an origin along a direction (the scene's frame;
///        the camera's z grows by 1 per unit along it) meets a surface: its
///        depth, or 0.
using Surface = std::function<double(const recon::camera::Vec3d&,
                                     const recon::camera::Vec3d&)>;

/// @brief Where a ray meets @p plane, as a @ref Surface.
/// @param plane      The plane.
/// @param origin     The ray's origin.
/// @param direction  Its direction.
/// @return The depth; 0 behind the origin, along the plane or not finite.
inline double hit_plane(const Plane& plane, const recon::camera::Vec3d& origin,
                        const recon::camera::Vec3d& direction) {
  const double s = glm::dot(plane.normal, plane.point - origin) /
                   glm::dot(plane.normal, direction);
  return s > 0.0 && std::isfinite(s) ? s : 0.0;
}

/// @brief Where a ray meets the box from @p lo to @p hi (the scene's axes)
///        first, by slabs, as a @ref Surface.
/// @param lo         The box's least corner.
/// @param hi         Its greatest.
/// @param origin     The ray's origin.
/// @param direction  Its direction.
/// @return The depth of its nearest face; 0 when the ray misses it.
inline double hit_box(const recon::camera::Vec3d& lo,
                      const recon::camera::Vec3d& hi,
                      const recon::camera::Vec3d& origin,
                      const recon::camera::Vec3d& direction) {
  double near = 0.0;
  double far = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i) {
    const double a = (lo[i] - origin[i]) / direction[i];
    const double b = (hi[i] - origin[i]) / direction[i];
    near = std::max(near, std::min(a, b));
    far = std::min(far, std::max(a, b));
  }
  return near > 0.0 && near <= far ? near : 0.0;
}

/// @brief The depth each pixel of @p p sees of @p surface, row-major.
/// @param p                The camera's pinhole.
/// @param camera_to_scene  The camera in the scene; rigid.
/// @param surface          What its rays meet.
/// @return Each pixel's depth, rounded to float, 0 where it meets nothing.
inline std::vector<float> render(const Pinhole& p,
                                 const recon::camera::Mat4d& camera_to_scene,
                                 const Surface& surface) {
  const recon::camera::Vec3d origin(camera_to_scene[3]);
  const recon::camera::Mat3d turn(camera_to_scene);
  std::vector<float> depth(std::size_t{p.size.width} * p.size.height);
  for (std::uint32_t v = 0; v < p.size.height; ++v) {
    for (std::uint32_t u = 0; u < p.size.width; ++u) {
      depth[std::size_t{v} * p.size.width + u] =
          static_cast<float>(surface(origin, turn * ray_of(p, u, v)));
    }
  }
  return depth;
}

/// @brief Each pixel's ray through @p lens at depth 1: the pinhole point the
///        lens images there, by Newton's method on recon's model
///        (`camera::distort_rational`), to 1e-13 in normalized coordinates.
/// @param lens  The lens.
/// @return The rays, row-major; (0, 0, 0) where the method does not converge.
inline std::vector<recon::camera::Vec3d> lens_rays(
    const recon::camera::CameraModel& lens) {
  const recon::camera::RationalDistortion& k = lens.distortion;
  const double d[8] = {k.k1, k.k2, k.p1, k.p2, k.k3, k.k4, k.k5, k.k6};
  const auto distort = [&](double x, double y) {
    glm::dvec2 out;
    recon::camera::distort_rational(d, x, y, &out.x, &out.y);
    return out;
  };
  const recon::camera::PinholeIntrinsics& f = lens.intrinsics;
  std::vector<recon::camera::Vec3d> rays(std::size_t{lens.size.width} *
                                         lens.size.height);
  for (std::uint32_t v = 0; v < lens.size.height; ++v) {
    for (std::uint32_t u = 0; u < lens.size.width; ++u) {
      const glm::dvec2 target((u - f.cx) / f.fx, (v - f.cy) / f.fy);
      glm::dvec2 p = target;
      bool converged = false;
      for (int i = 0; i < 50 && !converged; ++i) {
        const glm::dvec2 e = distort(p.x, p.y) - target;
        if (glm::length(e) < 1e-13) {
          converged = true;
          break;
        }
        constexpr double h = 1e-7;
        const glm::dvec2 dx =
            (distort(p.x + h, p.y) - distort(p.x - h, p.y)) / (2.0 * h);
        const glm::dvec2 dy =
            (distort(p.x, p.y + h) - distort(p.x, p.y - h)) / (2.0 * h);
        const double det = dx.x * dy.y - dy.x * dx.y;
        if (!(std::abs(det) > 1e-12)) break;
        p -=
            glm::dvec2(dy.y * e.x - dy.x * e.y, -dx.y * e.x + dx.x * e.y) / det;
      }
      if (converged) rays[std::size_t{v} * lens.size.width + u] = {p, 1.0};
    }
  }
  return rays;
}

/// @brief The raw depth a camera records of @p surface, as a driver hands it
///        out: each pixel's ray through its lens (@ref lens_rays) met with
///        the surface, the depth in units of @p metres_per_unit, rounded.
/// @param rays             Each pixel's ray, row-major (@ref lens_rays).
/// @param camera_to_scene  The camera in the scene; rigid.
/// @param surface          What its rays meet.
/// @param metres_per_unit  What a unit is (metres).
/// @param bias_m           Added to every depth measured (metres).
/// @return Each pixel's raw depth; 0 where it meets nothing, its ray is
///         unknown, or the depth is past what the units hold.
inline std::vector<std::uint16_t> render_raw(
    const std::vector<recon::camera::Vec3d>& rays,
    const recon::camera::Mat4d& camera_to_scene, const Surface& surface,
    double metres_per_unit, double bias_m = 0.0) {
  const recon::camera::Vec3d origin(camera_to_scene[3]);
  const recon::camera::Mat3d turn(camera_to_scene);
  std::vector<std::uint16_t> raw(rays.size(), 0);
  for (std::size_t i = 0; i < rays.size(); ++i) {
    if (rays[i].z == 0.0) continue;
    const double z = surface(origin, turn * rays[i]);
    if (!(z > 0.0)) continue;
    const double units = std::round((z + bias_m) / metres_per_unit);
    if (units > 0.0 && units <= 65535.0) {
      raw[i] = static_cast<std::uint16_t>(units);
    }
  }
  return raw;
}

}  // namespace volumetric_kit::calib::test
