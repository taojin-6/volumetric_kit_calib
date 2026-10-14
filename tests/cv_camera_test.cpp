// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// recon's camera model in OpenCV's types: its layout, cv::projectPoints equal
// to recon's distort_rational, pixels undistorted to convergence, and the
// models it refuses.

#include "volumetric_kit/calib/compute/cv_camera.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
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

// The normalized radius past which the radial map stops increasing: the
// lens folds over there and stops being a camera. `limit` if it does not
// fold before it.
double fold_radius(const camera::RationalDistortion& d, double limit) {
  constexpr double kStep = 1e-4;
  double previous = 0.0;
  for (double r = kStep; r <= limit; r += kStep) {
    const double mapped = radial_map(d, r);
    if (!(mapped > previous)) return r - kStep;
    previous = mapped;
  }
  return limit;
}

// The distorted normalized radius of the image's farthest corner: where a
// pinhole with the model's intrinsics puts it.
double corner_distorted_radius(const camera::CameraModel& model) {
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

// The radius the radial map takes to `distorted`, by bisection below the
// fold, where the map increases.
double undistorted_radius(const camera::RationalDistortion& d, double distorted,
                          double fold) {
  double low = 0.0;
  double high = fold;
  for (int i = 0; i < 100; ++i) {
    const double mid = 0.5 * (low + high);
    (radial_map(d, mid) < distorted ? low : high) = mid;
  }
  return high;
}

// cv::projectPoints with the model equals recon's distort_rational out to
// the image's corners, which the lens takes from `expected_corner`, an
// undistorted normalized radius below its fold.
void expect_projects_as_recon(const camera::CameraModel& model,
                              double expected_corner) {
  const core::Result<CvCamera> cam = cv_camera(model);
  ASSERT_TRUE(cam.ok()) << cam.status().message();
  const double fold = fold_radius(model.distortion, 10.0);
  const double corner_distorted = corner_distorted_radius(model);
  ASSERT_GT(radial_map(model.distortion, fold), corner_distorted)
      << "the lens folds over inside the image";
  const double radius =
      undistorted_radius(model.distortion, corner_distorted, fold);
  EXPECT_NEAR(radius, expected_corner, 1e-3);

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
  const camera::CameraModel model = test::depth_like_lens();
  const core::Result<CvCamera> cam = cv_camera(model);
  ASSERT_TRUE(cam.ok()) << cam.status().message();
  EXPECT_EQ(cam->size, cv::Size(640, 576));
  EXPECT_EQ(cam->camera_matrix,
            cv::Matx33d(504.2, 0.0, 321.1, 0.0, 504.3, 330.6, 0.0, 0.0, 1.0));
  const cv::Vec<double, 8> lens(29.29, 9.9, 1e-5, -2e-5, 0.3, 29.6, 15.0, 3.77);
  EXPECT_EQ(cam->distortion, lens);
}

TEST(CvCamera, ProjectsAsReconsColourLikeLens) {
  expect_projects_as_recon(test::color_like_lens(), 0.979);
}

// The depth-like lens folds over at r = 1.83, past its corners at 1.244.
TEST(CvCamera, ProjectsAsReconsDepthLikeLens) {
  expect_projects_as_recon(test::depth_like_lens(), 1.244);
}

// Pixels across the whole image, its corners included.
std::vector<cv::Point2d> pixel_grid(const cv::Size& size) {
  std::vector<cv::Point2d> pixels;
  for (int i = 0; i <= 16; ++i) {
    for (int j = 0; j <= 9; ++j) {
      pixels.emplace_back(-0.5 + i * size.width / 16.0,
                          -0.5 + j * size.height / 9.0);
    }
  }
  return pixels;
}

// How far the lens takes `rays` from `pixels`, at most (pixels).
double round_trip_px(const CvCamera& cam, const std::vector<cv::Point2d>& rays,
                     const std::vector<cv::Point2d>& pixels) {
  std::vector<cv::Point3d> points;
  for (const cv::Point2d& ray : rays) points.emplace_back(ray.x, ray.y, 1.0);
  std::vector<cv::Point2d> back;
  cv::projectPoints(points, cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0),
                    cam.camera_matrix, cam.distortion, back);
  double worst = 0.0;
  for (std::size_t i = 0; i < back.size(); ++i) {
    worst = std::max(
        worst, std::hypot(back[i].x - pixels[i].x, back[i].y - pixels[i].y));
  }
  return worst;
}

// Every ray goes back through the lens to its pixel, out to the image's
// corners, where undistortPoints' default criteria fall short on the strong
// lens.
TEST(CvCamera, UndistortsPixelsToConvergence) {
  for (const camera::CameraModel& model :
       {test::color_like_lens(), test::depth_like_lens(),
        test::strongly_distorted_lens()}) {
    const CvCamera cam = cv_camera(model).value();
    const std::vector<cv::Point2d> pixels = pixel_grid(cam.size);
    const core::Result<std::vector<cv::Point2d>> rays =
        undistort_points(cam, pixels);
    ASSERT_TRUE(rays.ok()) << rays.status().message();
    ASSERT_EQ(rays->size(), pixels.size());
    EXPECT_LT(round_trip_px(cam, *rays, pixels), 1e-9) << cam.size;
  }

  const CvCamera strong = cv_camera(test::strongly_distorted_lens()).value();
  const std::vector<cv::Point2d> pixels = pixel_grid(strong.size);
  std::vector<cv::Point2d> rays;
  cv::undistortPoints(pixels, rays, strong.camera_matrix, strong.distortion);
  EXPECT_GT(round_trip_px(strong, rays, pixels), 0.5);

  const core::Result<std::vector<cv::Point2d>> none =
      undistort_points(strong, {});
  ASSERT_TRUE(none.ok()) << none.status().message();
  EXPECT_TRUE(none->empty());
}

TEST(CvCamera, RefusesAModelReconRefuses) {
  camera::CameraModel no_focal = test::color_like_lens();
  no_focal.intrinsics.fx = std::numeric_limits<double>::quiet_NaN();
  camera::CameraModel no_width = test::color_like_lens();
  no_width.size.width = 0;
  camera::CameraModel no_lens = test::color_like_lens();
  no_lens.distortion.k5 = std::numeric_limits<double>::infinity();
  for (const camera::CameraModel& model : {no_focal, no_width, no_lens}) {
    core::Result<CvCamera> cam = core::Status::invalid_argument("unset");
    EXPECT_NO_THROW(cam = cv_camera(model));
    EXPECT_EQ(cam.status().domain(), core::Status::Code::InvalidArgument);
    EXPECT_EQ(cam.status().message().rfind("cv_camera: ", 0), 0u)
        << cam.status().message();
  }
}

TEST(CvCamera, RefusesASideOpenCVCannotHold) {
  camera::CameraModel wide = test::color_like_lens();
  wide.size.width =
      static_cast<std::uint32_t>(std::numeric_limits<int>::max()) + 1u;
  core::Result<CvCamera> cam = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(cam = cv_camera(wide));
  EXPECT_EQ(cam.status().domain(), core::Status::Code::InvalidArgument);
  EXPECT_EQ(cam.status().message().rfind("cv_camera: ", 0), 0u)
      << cam.status().message();

  wide.size.width = static_cast<std::uint32_t>(std::numeric_limits<int>::max());
  EXPECT_TRUE(cv_camera(wide).ok());
}

}  // namespace
}  // namespace volumetric_kit::calib::compute
