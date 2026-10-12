// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// recon's camera model in OpenCV's types: its layout, cv::projectPoints equal
// to recon's distort_rational, and the models it refuses.

#include "volumetric_kit/calib/compute/cv_camera.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/geometry.hpp>

#include "synthetic_rig.hpp"
#include "volumetric_kit/recon/camera/projection.hpp"

namespace volumetric_kit::calib::compute {
namespace {

namespace camera = recon::camera;

// The lens's radial map, r -> r * radial(r^2), with no tangential part.
double radial_map(const camera::RationalDistortion& d, double r) {
  const double r2 = r * r;
  return r * (1.0 + r2 * (d.k1 + r2 * (d.k2 + r2 * d.k3))) /
         (1.0 + r2 * (d.k4 + r2 * (d.k5 + r2 * d.k6)));
}

// The largest normalized radius, up to `limit`, below which the radial map
// keeps increasing; past it the lens folds over and stops being a camera.
double monotonic_radius(const camera::RationalDistortion& d, double limit) {
  constexpr double kStep = 1e-4;
  double previous = 0.0;
  for (double r = kStep; r <= limit; r += kStep) {
    const double mapped = radial_map(d, r);
    if (!(mapped > previous)) return r - kStep;
    previous = mapped;
  }
  return limit;
}

// The normalized radius of the image's farthest corner, as a pinhole sees it.
double corner_radius(const camera::CameraModel& model) {
  const camera::PinholeIntrinsics& k = model.intrinsics;
  double radius = 0.0;
  for (const double u : {-0.5, model.size.width - 0.5}) {
    for (const double v : {-0.5, model.size.height - 0.5}) {
      radius =
          std::max(radius, std::hypot((u - k.cx) / k.fx, (v - k.cy) / k.fy));
    }
  }
  return radius;
}

void expect_projects_as_recon(const camera::CameraModel& model) {
  const core::Result<CvCamera> cam = cv_camera(model);
  ASSERT_TRUE(cam.ok()) << cam.status().message();
  const double limit = corner_radius(model);
  const double radius = monotonic_radius(model.distortion, limit);
  ASSERT_GT(radius, 0.9 * limit) << "the lens folds over inside the image";

  std::vector<cv::Point3d> points;
  for (int i = 0; i <= 20; ++i) {
    for (int a = 0; a < 24; ++a) {
      const double r = radius * i / 20.0;
      const double angle = a * 3.14159265358979323846 / 12.0;
      const double z = 0.5 + 0.25 * a;  // the depth does not matter
      points.emplace_back(r * std::cos(angle) * z, r * std::sin(angle) * z, z);
    }
  }
  std::vector<cv::Point2d> pixels;
  cv::projectPoints(points, cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0),
                    cam->camera_matrix, cam->distortion, pixels);

  const camera::RationalDistortion& d = model.distortion;
  const double lens[8] = {d.k1, d.k2, d.p1, d.p2, d.k3, d.k4, d.k5, d.k6};
  const camera::PinholeIntrinsics& k = model.intrinsics;
  double worst = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    double xd = 0.0, yd = 0.0;
    camera::distort_rational(lens, points[i].x / points[i].z,
                             points[i].y / points[i].z, &xd, &yd);
    worst = std::max(worst, std::hypot(k.fx * xd + k.cx - pixels[i].x,
                                       k.fy * yd + k.cy - pixels[i].y));
  }
  EXPECT_LE(worst, 1e-9);
}

TEST(CvCamera, HoldsTheModelInOpenCVsLayout) {
  // Every coefficient differs, so a misplaced one shows.
  const camera::CameraModel model = testing::depth_like_lens();
  const core::Result<CvCamera> cam = cv_camera(model);
  ASSERT_TRUE(cam.ok()) << cam.status().message();
  EXPECT_EQ(cam->size, cv::Size(640, 576));
  EXPECT_EQ(cam->camera_matrix,
            cv::Matx33d(504.2, 0.0, 321.1, 0.0, 504.3, 330.6, 0.0, 0.0, 1.0));
  const cv::Vec<double, 8> lens(29.29, 9.9, 1e-5, -2e-5, 0.3, 29.6, 15.0, 3.77);
  EXPECT_EQ(cam->distortion, lens);
}

TEST(CvCamera, ProjectsAsReconsColourLikeLens) {
  expect_projects_as_recon(testing::color_like_lens());
}

TEST(CvCamera, ProjectsAsReconsDepthLikeLens) {
  expect_projects_as_recon(testing::depth_like_lens());
}

TEST(CvCamera, RefusesAModelReconRefuses) {
  camera::CameraModel no_focal = testing::color_like_lens();
  no_focal.intrinsics.fx = std::numeric_limits<double>::quiet_NaN();
  camera::CameraModel no_width = testing::color_like_lens();
  no_width.size.width = 0;
  camera::CameraModel no_lens = testing::color_like_lens();
  no_lens.distortion.k5 = std::numeric_limits<double>::infinity();
  for (const camera::CameraModel& model : {no_focal, no_width, no_lens}) {
    core::Result<CvCamera> cam = core::Status::invalid_argument("unset");
    EXPECT_NO_THROW(cam = cv_camera(model));
    EXPECT_EQ(cam.status().domain(), core::Status::Code::InvalidArgument);
  }
}

TEST(CvCamera, RefusesASideOpenCVCannotHold) {
  camera::CameraModel wide = testing::color_like_lens();
  wide.size.width =
      static_cast<std::uint32_t>(std::numeric_limits<int>::max()) + 1u;
  core::Result<CvCamera> cam = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(cam = cv_camera(wide));
  EXPECT_EQ(cam.status().domain(), core::Status::Code::InvalidArgument);
  EXPECT_NE(cam.status().message().find("cv_camera"), std::string::npos)
      << cam.status().message();

  wide.size.width = static_cast<std::uint32_t>(std::numeric_limits<int>::max());
  EXPECT_TRUE(cv_camera(wide).ok());
}

}  // namespace
}  // namespace volumetric_kit::calib::compute
