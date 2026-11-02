// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A stereo pair's rectified rows, on a pair like the lab's (the lab's lens on
// both cameras, 0.40 m apart, turned by the lab pair's magnitudes) at the
// runtime's 960 x 544: corners placed in the rectified view at 1.2-4 m and
// projected through the true pair. An exact pair reads nothing in any cell; a
// 1 mrad pitch reads f times it, as analysed, the means equal to rows through
// rectify_stereo's own R1, R2 and K to 1e-9 px; noise passes, its 95th
// percentile as the noise and each corner's slope predict, and heavy noise is
// held to its own standard error; a lens bent at one edge fails that cell,
// where a percentile would pass it; no corner, corners in the centre alone,
// or no side are not enough, and say where to capture; the pair named the
// other way round is Swapped, a vertical one Refused; corners outside the
// view are not counted; and calib's undistortion is exact to the lab lens's
// image corners.

#include "volumetric_kit/calib/quality/pair_row_check.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/geometry.hpp>

#include "result_checks.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/stereo_rectification.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = sensor::camera;
using test::expect_refused;
using test::value_of;
using Verdict = PairRowCheck::Verdict;
using Rig = std::vector<test::SyntheticCamera>;

constexpr double kPi = 3.14159265358979323846;
constexpr camera::ImageSize kRectified{960, 544};  // the runtime's

camera::Vec3d apply(const camera::Mat4d& m, const camera::Vec3d& p) {
  return camera::Vec3d(m * glm::dvec4(p, 1.0));
}

// A uniform draw in [0, 1) and a standard normal one, the same with every
// standard library.
double uniform(std::mt19937& rng) {
  return (static_cast<double>(rng()) + 0.5) / 4294967296.0;
}

double normal(std::mt19937& rng) {
  const double u1 = uniform(rng);
  const double u2 = uniform(rng);
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

// `rig`'s cameras as a solve writes them: posed, their lenses calibrated.
camera::ArrayCalibration file_of(const Rig& rig) {
  camera::ArrayCalibration file;
  for (const test::SyntheticCamera& cam : rig) {
    camera::SensorCalibration& sensor = file.sensors.emplace_back();
    sensor.id = cam.id;
    sensor.color_to_world = camera::rigid_inverse(cam.world_to_camera);
    sensor.color = camera::CameraCalibration{
        cam.model, camera::IntrinsicsSource::Calibrated};
  }
  return file;
}

sensor::StereoRectification rectification(const Rig& rig) {
  const sensor::StereoCalibration pair =
      value_of(sensor::stereo_calibration(file_of(rig), "bm-1", "bm-2"));
  return value_of(sensor::rectify_stereo(pair, kRectified, 0.0));
}

PairRowCheckOptions options() {
  PairRowCheckOptions o;
  o.rectified = kRectified;
  return o;
}

// The right camera turned by `rvec` about its own axes, where it stands.
Rig turned(Rig rig, const camera::Vec3d& rvec) {
  const camera::Mat4d turn(camera::rotation_from_rodrigues(rvec));
  rig[1].world_to_camera = turn * rig[1].world_to_camera;
  return rig;
}

// One board corner: its capture, id, place, and where it was placed in the
// left rectified view (the fractions of its width and height).
struct Corner {
  std::int64_t capture = 0;
  int id = 0;
  camera::Vec3d world{0.0};
  camera::Vec2d at{0.0};
};

// Where corners are placed: fractions of the rectified view, beyond [0, 1]
// for outside it.
struct Region {
  double u0 = 0.0, u1 = 1.0, v0 = 0.0, v1 = 1.0;
};

// `captures` captures of `per_capture` corners each, placed uniformly in
// `region` of `rig`'s left rectified view, 1.2-4 m deep.
std::vector<Corner> place(const Rig& rig, const Region& region, int captures,
                          int per_capture, std::uint32_t seed = 1,
                          std::int64_t first = 0) {
  const sensor::StereoRectification rect = rectification(rig);
  const camera::PinholeIntrinsics& k = rect.rig.camera.intrinsics;
  const camera::Mat4d to_world = camera::rigid_inverse(rig[0].world_to_camera) *
                                 rect.rig.rectified_to_left;
  std::mt19937 rng(seed);
  std::vector<Corner> corners;
  for (int c = 0; c < captures; ++c) {
    for (int i = 0; i < per_capture; ++i) {
      const double fu = region.u0 + (region.u1 - region.u0) * uniform(rng);
      const double fv = region.v0 + (region.v1 - region.v0) * uniform(rng);
      const double z = 1.2 + 2.8 * uniform(rng);
      const double u = fu * kRectified.width - 0.5;
      const double v = fv * kRectified.height - 0.5;
      const camera::Vec3d in_view((u - k.cx) / k.fx * z, (v - k.cy) / k.fy * z,
                                  z);
      corners.push_back({first + c, i, apply(to_world, in_view), {fu, fv}});
    }
  }
  return corners;
}

// What each camera of `truth` sees of `corners`, plus Gaussian noise of
// `noise_px` on each axis: one view a camera a capture.
std::vector<Observation> observe(const Rig& truth,
                                 const std::vector<Corner>& corners,
                                 double noise_px = 0.0,
                                 std::uint32_t seed = 7) {
  std::mt19937 rng(seed);
  std::vector<Observation> views;
  for (std::size_t c = 0; c < truth.size(); ++c) {
    std::vector<cv::Point3d> points;
    for (const Corner& corner : corners) {
      points.emplace_back(corner.world.x, corner.world.y, corner.world.z);
    }
    const std::vector<cv::Point2d> pixels =
        test::project(truth[c].model, truth[c].world_to_camera, points);
    for (std::size_t i = 0; i < corners.size(); ++i) {
      if (views.empty() || views.back().camera_id != truth[c].id ||
          views.back().frame_id != corners[i].capture) {
        Observation& view = views.emplace_back();
        view.camera_id = truth[c].id;
        view.frame_id = corners[i].capture;
      }
      const double dx = noise_px * normal(rng);
      const double dy = noise_px * normal(rng);
      views.back().points.push_back(
          {corners[i].id, {pixels[i].x + dx, pixels[i].y + dy}, 1.0});
    }
  }
  return views;
}

// A point's pixels in both rectified views of `rect`, through `truth`'s exact
// rays: the left's (x, y), then the right's.
std::array<camera::Vec2d, 2> rows_of(const sensor::StereoRectification& rect,
                                     const Rig& truth,
                                     const camera::Vec3d& world) {
  const camera::PinholeIntrinsics& k = rect.rig.camera.intrinsics;
  const camera::Mat3d r1 =
      glm::transpose(camera::Mat3d(rect.rig.rectified_to_left));
  const camera::Mat3d* turn[2] = {&r1, &rect.right_to_rectified};
  std::array<camera::Vec2d, 2> out;
  for (std::size_t c = 0; c < 2; ++c) {
    const camera::Vec3d q = *turn[c] * apply(truth[c].world_to_camera, world);
    out[c] = {k.fx * q.x / q.z + k.cx, k.fy * q.y / q.z + k.cy};
  }
  return out;
}

// The cell of the 3 x 3 grid over the rectified view holding `p`; -1
// outside.
int cell_of(const camera::Vec2d& p) {
  const double x = (p.x + 0.5) / kRectified.width;
  const double y = (p.y + 0.5) / kRectified.height;
  if (!(x >= 0.0 && x < 1.0 && y >= 0.0 && y < 1.0)) return -1;
  return static_cast<int>(y * 3) * 3 + static_cast<int>(x * 3);
}

// Each cell's mean of `f` over the corners inside both views of `rect`
// through `truth`, and their count.
template <class F>
std::array<double, kPairRowCells> cell_means(
    const sensor::StereoRectification& rect, const Rig& truth,
    const std::vector<Corner>& corners, F f,
    std::array<std::size_t, kPairRowCells>* counts = nullptr) {
  std::array<double, kPairRowCells> sum{};
  std::array<std::size_t, kPairRowCells> n{};
  for (const Corner& corner : corners) {
    const std::array<camera::Vec2d, 2> at = rows_of(rect, truth, corner.world);
    const int cell = cell_of(at[0]);
    if (cell < 0 || cell_of(at[1]) < 0) continue;
    sum[static_cast<std::size_t>(cell)] += f(at);
    ++n[static_cast<std::size_t>(cell)];
  }
  for (std::size_t c = 0; c < sum.size(); ++c) sum[c] /= n[c];
  if (counts != nullptr) *counts = n;
  return sum;
}

std::string named(const PairRowCheck& check) {
  std::string text =
      std::string(to_string(check.verdict)) + ": " + check.reason + "; cells";
  for (std::size_t c = 0; c < check.cell_mean_px.size(); ++c) {
    text += " " + std::to_string(check.cell_corners[c]) + "@" +
            std::to_string(check.cell_mean_px[c]);
  }
  return text;
}

TEST(PairRowCheck, AnExactPairReadsNothingInAnyCell) {
  const Rig pair = test::lab_pair();
  const std::vector<Corner> corners = place(pair, {}, 60, 40);
  const PairRowCheck check = value_of(check_pair_rows(
      file_of(pair), "bm-1", "bm-2", observe(pair, corners), options()));
  ASSERT_EQ(check.verdict, Verdict::Pass) << named(check);
  EXPECT_TRUE(check.reason.empty());
  EXPECT_EQ(check.left, "bm-1");
  EXPECT_EQ(check.right, "bm-2");
  EXPECT_EQ(check.captures, 60u);
  for (std::size_t c = 0; c < check.cell_mean_px.size(); ++c) {
    EXPECT_LT(std::fabs(check.cell_mean_px[c]), 0.02) << "cell " << c;
    EXPECT_EQ(check.cell_limit_px[c], 0.15) << "cell " << c;
  }
  EXPECT_LT(check.dy_p95_px, 0.02);
  EXPECT_LE(check.dy_median_px, check.dy_p95_px);
  EXPECT_NEAR(check.baseline_m, 0.40, 0.001);
}

TEST(PairRowCheck, MeasuresThroughRectifyStereosRotationsAndCamera) {
  // The solved pair a milliradian off in pitch and roll: each corner's rows,
  // through the solved pair's rectify_stereo and the true rays, are what the
  // check reads, cell by cell.
  const Rig truth = test::lab_pair();
  const Rig solved = turned(truth, {0.001, 0.0, 0.0005});
  const std::vector<Corner> corners = place(truth, {}, 40, 40);
  const PairRowCheck check = value_of(check_pair_rows(
      file_of(solved), "bm-1", "bm-2", observe(truth, corners), options()));
  const sensor::StereoRectification rect = rectification(solved);
  std::array<std::size_t, kPairRowCells> counts{};
  const std::array<double, kPairRowCells> expected = cell_means(
      rect, truth, corners,
      [](const std::array<camera::Vec2d, 2>& at) { return at[0].y - at[1].y; },
      &counts);
  EXPECT_EQ(check.cell_corners, counts);
  for (std::size_t c = 0; c < expected.size(); ++c) {
    EXPECT_NEAR(check.cell_mean_px[c], expected[c], 1e-9) << "cell " << c;
  }
  EXPECT_EQ(check.baseline_m, rect.rig.baseline);
}

TEST(PairRowCheck, AMilliradianOfPitchReadsFTimesItInEveryCell) {
  // A pitch turns a ray's row by f (1 + y^2) times it, y its normalized row
  // in the view: at f 371, 0.38 px in the middle row of cells, 0.45-0.48 px
  // on average in the top and bottom rows.
  const Rig truth = test::lab_pair();
  const Rig solved = turned(truth, {0.001, 0.0, 0.0});
  const std::vector<Corner> corners = place(truth, {}, 60, 40);
  const PairRowCheck check = value_of(check_pair_rows(
      file_of(solved), "bm-1", "bm-2", observe(truth, corners), options()));
  EXPECT_EQ(check.verdict, Verdict::Fail) << named(check);
  const sensor::StereoRectification rect = rectification(solved);
  const camera::PinholeIntrinsics& k = rect.rig.camera.intrinsics;
  EXPECT_NEAR(k.fy, 371.0, 1.0);
  const std::array<double, kPairRowCells> analytic = cell_means(
      rect, truth, corners, [&](const std::array<camera::Vec2d, 2>& at) {
        const double y = (at[0].y - k.cy) / k.fy;
        return k.fy * 0.001 * (1.0 + y * y);
      });
  for (std::size_t c = 0; c < analytic.size(); ++c) {
    EXPECT_NEAR(std::fabs(check.cell_mean_px[c]), analytic[c],
                0.1 * analytic[c])
        << "cell " << c;
  }
  for (const std::size_t c : {3u, 4u, 5u}) {
    EXPECT_NEAR(std::fabs(check.cell_mean_px[c]), 0.38, 0.038) << "cell " << c;
  }
}

// Each corner's sigma of Δy for `noise_px` of noise on each axis in each
// camera: its rectified row's slope against its pixel in each camera, through
// the lens forward (the inverse of the Jacobian of rectified view to pixel).
std::vector<double> dy_sigmas(const Rig& truth,
                              const std::vector<Corner>& corners,
                              double noise_px) {
  const sensor::StereoRectification rect = rectification(truth);
  const camera::PinholeIntrinsics& k = rect.rig.camera.intrinsics;
  const camera::Mat3d r1 =
      glm::transpose(camera::Mat3d(rect.rig.rectified_to_left));
  const camera::Mat3d to_camera[2] = {glm::transpose(r1),
                                      glm::transpose(rect.right_to_rectified)};
  constexpr double h = 0.01;
  std::vector<double> sigmas;
  for (const Corner& corner : corners) {
    const std::array<camera::Vec2d, 2> at = rows_of(rect, truth, corner.world);
    if (cell_of(at[0]) < 0 || cell_of(at[1]) < 0) continue;
    double variance = 0.0;
    for (std::size_t c = 0; c < 2; ++c) {
      std::vector<cv::Point3d> rays;
      for (const camera::Vec2d& d :
           {camera::Vec2d(h, 0), camera::Vec2d(-h, 0), camera::Vec2d(0, h),
            camera::Vec2d(0, -h)}) {
        const camera::Vec2d p = at[c] + d;
        const camera::Vec3d ray =
            to_camera[c] *
            camera::Vec3d((p.x - k.cx) / k.fx, (p.y - k.cy) / k.fy, 1.0);
        rays.emplace_back(ray.x, ray.y, ray.z);
      }
      const std::vector<cv::Point2d> px =
          test::project(truth[c].model, camera::Mat4d(1.0), rays);
      // Pixel against rectified view, then its inverse's second row.
      const double a = (px[0].x - px[1].x) / (2 * h);
      const double b = (px[2].x - px[3].x) / (2 * h);
      const double cc = (px[0].y - px[1].y) / (2 * h);
      const double d = (px[2].y - px[3].y) / (2 * h);
      const double det = a * d - b * cc;
      const double gx = -cc / det;
      const double gy = a / det;
      variance += noise_px * noise_px * (gx * gx + gy * gy);
    }
    sigmas.push_back(std::sqrt(variance));
  }
  return sigmas;
}

// The 95th percentile of |Δy| over corners of these sigmas, each Gaussian.
double p95_of(const std::vector<double>& sigmas) {
  const auto share_below = [&](double t) {
    double sum = 0.0;
    for (const double s : sigmas) sum += std::erf(t / (s * std::sqrt(2.0)));
    return sum / sigmas.size();
  };
  double lo = 0.0;
  double hi = 10.0 * *std::max_element(sigmas.begin(), sigmas.end());
  for (int i = 0; i < 100; ++i) {
    const double mid = 0.5 * (lo + hi);
    (share_below(mid) < 0.95 ? lo : hi) = mid;
  }
  return 0.5 * (lo + hi);
}

TEST(PairRowCheck, NoisePassesItsPercentileAsPredicted) {
  const Rig pair = test::lab_pair();
  const std::vector<Corner> corners = place(pair, {}, 60, 40);
  const PairRowCheck check = value_of(check_pair_rows(
      file_of(pair), "bm-1", "bm-2", observe(pair, corners, 0.1), options()));
  EXPECT_EQ(check.verdict, Verdict::Pass) << named(check);
  const std::vector<double> sigmas = dy_sigmas(pair, corners, 0.1);
  EXPECT_EQ(check.corners, sigmas.size());
  const double predicted = p95_of(sigmas);
  EXPECT_NEAR(check.dy_p95_px, predicted, 0.1 * predicted);
}

TEST(PairRowCheck, HeavyNoiseIsJudgedByItsOwnStandardError) {
  // 1.5 px of noise on few corners: a cell's mean strays past 0.15 px by
  // chance, within 3 standard errors of it.
  const Rig pair = test::lab_pair();
  const std::vector<Corner> corners = place(pair, {}, 6, 40);
  const PairRowCheck check = value_of(check_pair_rows(
      file_of(pair), "bm-1", "bm-2", observe(pair, corners, 1.5), options()));
  EXPECT_EQ(check.verdict, Verdict::Pass) << named(check);
  bool strayed = false;
  for (std::size_t c = 0; c < check.cell_mean_px.size(); ++c) {
    EXPECT_GT(check.cell_limit_px[c], 0.15) << "cell " << c;
    strayed = strayed || std::fabs(check.cell_mean_px[c]) > 0.15;
  }
  EXPECT_TRUE(strayed) << named(check);
}

TEST(PairRowCheck, ALensBentAtOneEdgeFailsThatCellWhereAPercentilePasses) {
  // The left lens 3 px off in rows at the outer edge of the view's right part
  // (its outer 30%): a cell's mean sees it, the 95th percentile over every
  // corner does not.
  const Rig pair = test::lab_pair();
  const std::vector<Corner> corners = place(pair, {}, 60, 40);
  std::vector<Observation> views = observe(pair, corners, 0.1);
  std::size_t bent = 0;
  for (Observation& view : views) {
    if (view.camera_id != "bm-1") continue;
    for (DetectedPoint& p : view.points) {
      const Corner& corner =
          corners[static_cast<std::size_t>(view.frame_id) * 40 +
                  static_cast<std::size_t>(p.id)];
      if (corner.at.x < 0.9 || corner.at.y < 1.0 / 3 || corner.at.y >= 2.0 / 3)
        continue;
      p.pixel.y += 3.0;
      ++bent;
    }
  }
  ASSERT_GT(bent, 0u);
  const PairRowCheck check = value_of(
      check_pair_rows(file_of(pair), "bm-1", "bm-2", views, options()));
  EXPECT_EQ(check.verdict, Verdict::Fail) << named(check);
  EXPECT_NE(check.reason.find("in the view's right,"), std::string::npos)
      << check.reason;
  for (std::size_t c = 0; c < check.cell_mean_px.size(); ++c) {
    EXPECT_EQ(std::fabs(check.cell_mean_px[c]) > check.cell_limit_px[c], c == 5)
        << "cell " << c << ": " << named(check);
  }
  EXPECT_LT(check.dy_p95_px, options().max_cell_mean_px);
}

TEST(PairRowCheck, CornersInTheCentreAloneAreNotEnoughAndSayWhereToCapture) {
  const Rig pair = test::lab_pair();
  const std::vector<Corner> corners =
      place(pair, {0.36, 0.64, 0.36, 0.64}, 20, 40);
  const PairRowCheck check = value_of(check_pair_rows(
      file_of(pair), "bm-1", "bm-2", observe(pair, corners), options()));
  ASSERT_EQ(check.verdict, Verdict::Insufficient) << named(check);
  for (std::size_t c = 0; c < check.cell_corners.size(); ++c) {
    EXPECT_EQ(check.cell_corners[c] > 0, c == 4) << "cell " << c;
    EXPECT_EQ(std::isnan(check.cell_mean_px[c]), c != 4) << "cell " << c;
  }
  for (const char* where :
       {"top-left (0)", "top (0)", "top-right (0)", "left (0)", "right (0)",
        "bottom-left (0)", "bottom (0)", "bottom-right (0)"}) {
    EXPECT_NE(check.reason.find(where), std::string::npos) << check.reason;
  }
  EXPECT_EQ(check.reason.find("centre"), std::string::npos) << check.reason;
}

TEST(PairRowCheck, NoCornerSeenByBothIsNotEnough) {
  const Rig pair = test::lab_pair();
  std::vector<Observation> views = observe(pair, place(pair, {}, 3, 40));
  for (Observation& view : views) {
    if (view.camera_id == "bm-2") view.frame_id += 100;
  }
  for (const std::vector<Observation>& given :
       {std::vector<Observation>{}, views}) {
    const PairRowCheck check = value_of(
        check_pair_rows(file_of(pair), "bm-1", "bm-2", given, options()));
    EXPECT_EQ(check.verdict, Verdict::Insufficient) << named(check);
    EXPECT_EQ(check.captures, 0u);
    EXPECT_EQ(check.corners, 0u);
    EXPECT_TRUE(std::isnan(check.dy_median_px));
    EXPECT_TRUE(std::isnan(check.dy_p95_px));
    EXPECT_NE(check.reason.find("centre (0)"), std::string::npos)
        << check.reason;
    EXPECT_NEAR(check.baseline_m, 0.40, 0.001);
  }
}

TEST(PairRowCheck, EachSideNeedsACell) {
  // Six cells judged where six are asked, but none on the left: not enough
  // until the left has its corners.
  const Rig pair = test::lab_pair();
  PairRowCheckOptions six = options();
  six.min_cells = 6;
  std::vector<Corner> corners = place(pair, {0.34, 1.0, 0.0, 1.0}, 40, 40);
  PairRowCheck check = value_of(check_pair_rows(file_of(pair), "bm-1", "bm-2",
                                                observe(pair, corners), six));
  ASSERT_EQ(check.verdict, Verdict::Insufficient) << named(check);
  EXPECT_NE(check.reason.find(", left (0),"), std::string::npos)
      << check.reason;
  const std::vector<Corner> left =
      place(pair, {0.1, 0.3, 0.4, 0.6}, 1, 40, 2, 40);
  corners.insert(corners.end(), left.begin(), left.end());
  check = value_of(check_pair_rows(file_of(pair), "bm-1", "bm-2",
                                   observe(pair, corners), six));
  EXPECT_EQ(check.verdict, Verdict::Pass) << named(check);
}

TEST(PairRowCheck, ThePairNamedTheOtherWayRoundIsSwapped) {
  const Rig pair = test::lab_pair();
  const std::vector<Corner> corners = place(pair, {}, 10, 40);
  const PairRowCheck check = value_of(check_pair_rows(
      file_of(pair), "bm-2", "bm-1", observe(pair, corners), options()));
  EXPECT_EQ(check.verdict, Verdict::Swapped) << named(check);
  EXPECT_EQ(check.left, "bm-2");
  EXPECT_EQ(check.right, "bm-1");
  EXPECT_NE(check.reason.find("\"bm-1\" is the left camera"), std::string::npos)
      << check.reason;
  EXPECT_EQ(check.corners, 0u);
  EXPECT_TRUE(std::isnan(check.cell_mean_px[4]));
}

TEST(PairRowCheck, AVerticalPairIsRefusedEitherWayRound) {
  // bm-2 0.4 m below bm-1, along its +y.
  Rig pair = test::lab_pair();
  camera::Mat4d below(1.0);
  below[3] = glm::dvec4(0.0, -0.4, 0.0, 1.0);
  pair[1].world_to_camera = below * pair[0].world_to_camera;
  const std::vector<Corner> corners = place(test::lab_pair(), {}, 10, 40);
  for (const bool reversed : {false, true}) {
    const PairRowCheck check = value_of(check_pair_rows(
        file_of(pair), reversed ? "bm-2" : "bm-1", reversed ? "bm-1" : "bm-2",
        observe(pair, corners), options()));
    EXPECT_EQ(check.verdict, Verdict::Refused) << named(check);
    EXPECT_NE(check.reason.find("vertical"), std::string::npos) << check.reason;
    EXPECT_EQ(check.baseline_m, 0.0);
  }
}

// Whether `p` lies `margin` px or more inside the rectified view, or as far
// outside it.
bool clear_of_the_edge(const camera::Vec2d& p, double margin) {
  const double right = kRectified.width - 0.5;
  const double bottom = kRectified.height - 0.5;
  const bool inside = p.x - margin >= -0.5 && p.x + margin < right &&
                      p.y - margin >= -0.5 && p.y + margin < bottom;
  const bool outside = p.x + margin < -0.5 || p.x - margin >= right ||
                       p.y + margin < -0.5 || p.y - margin >= bottom;
  return inside || outside;
}

TEST(PairRowCheck, CornersOutsideTheViewAreNotCounted) {
  // Corners placed past the view's edges, those outside either view a pixel
  // off in the left camera's rows (not enough to cross 3 px, so those near
  // an edge are left out): the check counts and judges only those inside.
  const Rig pair = test::lab_pair();
  const sensor::StereoRectification rect = rectification(pair);
  std::vector<Corner> corners;
  for (const Corner& corner : place(pair, {-0.3, 1.3, -0.3, 1.3}, 60, 40)) {
    const std::array<camera::Vec2d, 2> at = rows_of(rect, pair, corner.world);
    if (clear_of_the_edge(at[0], 3.0) && clear_of_the_edge(at[1], 3.0)) {
      corners.push_back(corner);
    }
  }
  std::vector<Observation> views = observe(pair, corners);
  std::size_t inside = 0;
  std::size_t next = 0;  // views[0] is bm-1's first, in `corners`' order
  for (Observation& view : views) {
    if (view.camera_id != "bm-1") continue;
    for (DetectedPoint& p : view.points) {
      const std::array<camera::Vec2d, 2> at =
          rows_of(rect, pair, corners[next++].world);
      const bool in = cell_of(at[0]) >= 0 && cell_of(at[1]) >= 0;
      inside += in ? 1 : 0;
      if (!in) p.pixel.y += 1.0;
    }
  }
  ASSERT_EQ(next, corners.size());
  ASSERT_LT(inside, corners.size() * 3 / 4);
  const PairRowCheck check = value_of(
      check_pair_rows(file_of(pair), "bm-1", "bm-2", views, options()));
  EXPECT_EQ(check.verdict, Verdict::Pass) << named(check);
  EXPECT_EQ(check.corners, inside);
  EXPECT_LT(check.dy_p95_px, 0.02);
}

TEST(PairRowCheck, CalibsUndistortionIsExactToTheLabLensCorners) {
  // Every pixel of a grid over the 4K picture, its corners 77.9 degrees off
  // axis, back through the lens within 1e-6 px; OpenCV's default five
  // iterations miss by pixels there.
  const camera::CameraModel lens = test::lab_lens();
  const compute::CvCamera cv = value_of(compute::cv_camera(lens));
  std::vector<cv::Point2d> pixels;
  for (int i = 0; i <= 32; ++i) {
    for (int j = 0; j <= 18; ++j) {
      pixels.emplace_back(-0.5 + 3840.0 * i / 32, -0.5 + 2160.0 * j / 18);
    }
  }
  const std::vector<cv::Point2d> rays =
      value_of(compute::undistort_points(cv, pixels));
  std::vector<cv::Point3d> points;
  double widest = 0.0;
  for (const cv::Point2d& r : rays) {
    points.emplace_back(r.x, r.y, 1.0);
    widest = std::max(widest, std::atan(std::hypot(r.x, r.y)) * 180.0 / kPi);
  }
  EXPECT_GT(widest, 77.9);
  const std::vector<cv::Point2d> back =
      test::project(lens, camera::Mat4d(1.0), points);
  double worst = 0.0;
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    worst = std::max(worst, cv::norm(back[i] - pixels[i]));
  }
  EXPECT_LT(worst, 1e-6);

  std::vector<cv::Point2d> opencv;
  cv::undistortPoints(pixels, opencv, cv.camera_matrix, cv.distortion);
  std::vector<cv::Point3d> opencv_points;
  for (const cv::Point2d& r : opencv) opencv_points.emplace_back(r.x, r.y, 1.0);
  const std::vector<cv::Point2d> opencv_back =
      test::project(lens, camera::Mat4d(1.0), opencv_points);
  double opencv_worst = 0.0;
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    opencv_worst = std::max(opencv_worst, cv::norm(opencv_back[i] - pixels[i]));
  }
  EXPECT_GT(opencv_worst, 1.0);
}

TEST(PairRowCheck, RefusesWhatItCannotCheck) {
  const Rig pair = test::lab_pair();
  const camera::ArrayCalibration file = file_of(pair);
  const std::vector<Observation> views = observe(pair, place(pair, {}, 2, 10));
  const auto refused = [&](PairRowCheckOptions o, const char* says) {
    expect_refused(check_pair_rows(file, "bm-1", "bm-2", views, o).status(),
                   "check_pair_rows", says);
  };
  PairRowCheckOptions o = options();
  o.rectified = {0, 0};
  refused(o, "options.rectified");
  o = options();
  o.rectified = {960, 0};
  refused(o, "options.rectified");
  o = options();
  o.alpha = 1.5;
  refused(o, "options.alpha");
  o.alpha = std::nan("");
  refused(o, "options.alpha");
  o = options();
  o.max_cell_mean_px = 0.0;
  refused(o, "options.max_cell_mean_px");
  o = options();
  o.min_cell_corners = 1;
  refused(o, "options.min_cell_corners");
  o = options();
  o.min_cells = 0;
  refused(o, "options.min_cells");
  o.min_cells = 10;
  refused(o, "options.min_cells");

  expect_refused(
      check_pair_rows(file, "bm-1", "bm-3", views, options()).status(),
      "check_pair_rows", "\"bm-3\" is not in the file");
  expect_refused(
      check_pair_rows(file, "bm-1", "bm-1", views, options()).status(),
      "check_pair_rows", "both \"bm-1\"");
  camera::ArrayCalibration unposed = file;
  unposed.sensors[1].color_to_world.reset();
  expect_refused(
      check_pair_rows(unposed, "bm-1", "bm-2", views, options()).status(),
      "check_pair_rows", "unposed");
  std::vector<Observation> twice = views;
  twice.push_back(views.front());
  expect_refused(
      check_pair_rows(file, "bm-1", "bm-2", twice, options()).status(),
      "check_pair_rows", "\"bm-1\" has two views of capture 0");
}

TEST(PairRowCheck, OtherCamerasAndCornersSeenOnceAreIgnored) {
  const Rig pair = test::lab_pair();
  const std::vector<Corner> corners = place(pair, {}, 30, 40);
  std::vector<Observation> views = observe(pair, corners);
  const PairRowCheck all = value_of(
      check_pair_rows(file_of(pair), "bm-1", "bm-2", views, options()));
  // A third camera's views, and corners only the left camera saw.
  for (std::size_t i = 0, n = views.size(); i < n; ++i) {
    Observation other = views[i];
    other.camera_id = "bm-3";
    views.push_back(other);
    if (views[i].camera_id == "bm-1") {
      views[i].points.push_back({1000, {1920.0, 1080.0}, 1.0});
    }
  }
  const PairRowCheck again = value_of(
      check_pair_rows(file_of(pair), "bm-1", "bm-2", views, options()));
  EXPECT_EQ(again.corners, all.corners);
  EXPECT_EQ(again.cell_mean_px, all.cell_mean_px);
}

TEST(PairRowCheck, NamesEachVerdict) {
  EXPECT_STREQ(to_string(Verdict::Pass), "pass");
  EXPECT_STREQ(to_string(Verdict::Fail), "fail");
  EXPECT_STREQ(to_string(Verdict::Insufficient), "insufficient");
  EXPECT_STREQ(to_string(Verdict::Swapped), "swapped");
  EXPECT_STREQ(to_string(Verdict::Refused), "refused");
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
