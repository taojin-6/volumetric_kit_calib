// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A tracked camera's world at the origin tag, from its stills of it: exact
// without noise for either up, face-on and from one still included, whatever
// the views' order; on the detector's corners in drawn stills; the pass bars
// over 300 noisy holds a case; each miss and refusal; and the registration it
// gives, written by sensor and read back. The corners are the synthetic
// rig's floor tag (floor_tag_corners), never the solver's square.

#include "volumetric_kit/calib/solve/tracked_world.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "detect_helpers.hpp"
#include "result_checks.hpp"
#include "scratch_dir.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/cv_pose.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/detect/tag.hpp"

namespace volumetric_kit::calib::solve {
namespace {

namespace camera = sensor::camera;

constexpr double kEdge = 0.14725;  // the rig's measured edge
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
const camera::Vec3d kYUp(0.0, 1.0, 0.0);
const camera::Vec3d kZUp(0.0, 0.0, 1.0);

cv::Matx33d rz(double deg) {
  const double c = std::cos(deg * kDeg);
  const double s = std::sin(deg * kDeg);
  return {c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0};
}

camera::Mat4d rigid(const cv::Matx33d& r, const cv::Vec3d& t) {
  cv::Matx44d m = cv::Matx44d::eye();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) m(row, col) = r(row, col);
    m(row, 3) = t[row];
  }
  return compute::matrix_from_cv(m);
}

cv::Matx33d rotation_of(const camera::Mat4d& m) {
  cv::Matx33d r;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) r(row, col) = m[col][row];
  }
  return r;
}

camera::Vec3d centre_of(const camera::Mat4d& m) { return camera::Vec3d(m[3]); }

// The tracking world to the tag's in truth, [Rz(yaw) B | t], B turning `up`
// onto +z: a quarter turn about x for ARKit's +y, none for +z.
camera::Mat4d truth(const camera::Vec3d& up, double yaw_deg,
                    const cv::Vec3d& t) {
  const cv::Matx33d b =
      up.z != 0.0 ? cv::Matx33d::eye()
                  : cv::Matx33d(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
  return rigid(rz(yaw_deg) * b, t);
}

// A still of the floor tag of edge kEdge by a camera at `cam_to_world`, its
// pose recorded as `cam_to_tracking`, its corners projected exactly.
TrackedTagView still(const std::string& name, const camera::Mat4d& cam_to_world,
                     const camera::Mat4d& cam_to_tracking) {
  const camera::CameraModel model = test::tracked_pinhole();
  const std::array<cv::Point3d, 4> corners =
      test::floor_tag_corners({kEdge, {}, 0.0});
  const std::vector<cv::Point2d> pixels =
      test::project(model, camera::rigid_inverse(cam_to_world),
                    {corners.begin(), corners.end()});
  TrackedTagView view{name, model.size, model, cam_to_tracking, {}};
  view.marker.id = kOriginTag.id;
  for (std::size_t k = 0; k < 4; ++k) {
    view.marker.corners[k] = {pixels[k].x, pixels[k].y};
  }
  return view;
}

// A still posed through `tracking_to_world` exactly.
TrackedTagView posed(const std::string& name, const camera::Mat4d& cam_to_world,
                     const camera::Mat4d& tracking_to_world) {
  return still(name, cam_to_world,
               camera::rigid_inverse(tracking_to_world) * cam_to_world);
}

core::Result<TrackedWorld> place(const std::vector<TrackedTagView>& views,
                                 const camera::Vec3d& up = kYUp,
                                 double edge_m = kEdge,
                                 const OriginTag& tag = kOriginTag) {
  core::Result<TrackedWorld> found = core::Status::invalid_argument("unset");
  TrackedWorldOptions options;
  options.up = up;
  EXPECT_NO_THROW(found = tracked_world_from_tag(views, tag, edge_m, options));
  return found;
}

TrackedTagFit fit_of(const core::Result<TrackedWorld>& found) {
  const TrackedWorld& w = test::value_of(found);
  if (!w.fit) throw std::runtime_error("no fit: " + w.reason);
  return *w.fit;
}

double yaw_error_deg(const camera::Mat4d& fit, const camera::Mat4d& truth) {
  const cv::Matx33d d = rotation_of(fit) * rotation_of(truth).t();
  return std::abs(std::atan2(d(1, 0), d(0, 0))) / kDeg;
}

// Noise-free corners through known poses: a camera 1.2 m out at 40 degrees,
// one 2.2 m out at 55, and one face-on, the tracking world's up +y, +z or
// +y three times over.
TEST(TrackedWorld, IsExactWithoutNoise) {
  for (const camera::Vec3d& up : {kYUp, kZUp, camera::Vec3d(0.0, 3.0, 0.0)}) {
    SCOPED_TRACE("up " + std::to_string(up.y) + ", " + std::to_string(up.z));
    const camera::Mat4d t = truth(up, 37.0, {0.4, -1.1, 0.8});
    const camera::Mat4d at[3] = {test::camera_over_tag(1.2, 40.0, 20.0, 15.0),
                                 test::camera_over_tag(2.2, 55.0, 200.0, -70.0),
                                 test::camera_over_tag(1.2, 0.0, 0.0, 0.0)};
    std::vector<TrackedTagView> views;
    for (int i = 0; i < 3; ++i) {
      views.push_back(posed("still_00" + std::to_string(i) + ".png", at[i], t));
    }
    const core::Result<TrackedWorld> found = place(views, up);
    const TrackedWorld& w = test::value_of(found);
    ASSERT_TRUE(w.fit) << w.reason;
    EXPECT_TRUE(w.reason.empty());
    EXPECT_EQ(w.sought.id, kOriginTag.id);
    EXPECT_EQ(w.sought.edge_m, kOriginTag.edge_m);
    const TrackedTagFit& f = *w.fit;
    EXPECT_LT(yaw_error_deg(f.tracking_to_world, t), 1e-7);
    EXPECT_LT(glm::length(centre_of(f.tracking_to_world) - centre_of(t)), 1e-7);
    // The tracking world's up goes to +z.
    EXPECT_LT(
        glm::length(camera::Mat3d(f.tracking_to_world) * glm::normalize(up) -
                    kZUp),
        1e-12);
    EXPECT_LT(f.rms_px, 1e-6);
    EXPECT_LT(f.worst_px, 1e-6);
    EXPECT_EQ(f.edge_m, kEdge);
    ASSERT_EQ(f.stills.size(), 3u);
    const double range[3] = {1.2, 2.2, 1.2};
    const double off_normal[3] = {40.0, 55.0, 0.0};
    for (std::size_t i = 0; i < 3; ++i) {
      EXPECT_EQ(f.stills[i].still, views[i].still);
      EXPECT_NEAR(f.stills[i].range_m, range[i], 1e-7);
      EXPECT_NEAR(f.stills[i].off_normal_deg, off_normal[i], 1e-5);
      EXPECT_LE(f.stills[i].rms_px, f.stills[i].worst_px);
      // Each still alone holds the yaw and position, face-on included.
      const TrackedTagFit alone = fit_of(place({views[i]}, up));
      EXPECT_LT(yaw_error_deg(alone.tracking_to_world, t), 1e-7) << i;
      EXPECT_LT(glm::length(centre_of(alone.tracking_to_world) - centre_of(t)),
                1e-7)
          << i;
    }
    // In any order, the same fit.
    std::vector<TrackedTagView> turned = {views[2], views[0], views[1]};
    const TrackedTagFit shuffled = fit_of(place(turned, up));
    EXPECT_EQ(shuffled.tracking_to_world, f.tracking_to_world);
    for (std::size_t i = 0; i < 3; ++i) {
      EXPECT_EQ(shuffled.stills[i].still, f.stills[i].still);
    }
  }
}

// Tag 0 drawn into 1920 x 1440 stills, blurred and noisy, and found by the
// detector: its corner order and the up axis, on the real detector.
TEST(TrackedWorld, FitsTheDetectorsCorners) {
  const camera::Mat4d t = truth(kYUp, -63.0, {0.2, 0.9, -0.4});
  const camera::CameraModel model = test::tracked_pinhole();
  const camera::Mat4d at[2] = {test::camera_over_tag(1.2, 40.0, 30.0, 10.0),
                               test::camera_over_tag(1.25, 42.0, 33.0, 12.0)};
  std::vector<TrackedTagView> views;
  for (std::uint64_t i = 0; i < 2; ++i) {
    cv::Mat image(1440, 1920, CV_8UC1, cv::Scalar(110));
    test::draw_marker(image, kOriginTag.dictionary, kOriginTag.id,
                      test::floor_tag_print(model, camera::rigid_inverse(at[i]),
                                            {kEdge, {}, 0.0}));
    test::CameraOptions optics;
    optics.seed = i + 1;
    const cv::Mat picture = test::camera_image(image, optics);
    const detect::MarkerSearch search =
        test::value_of(detect::detect_tag(test::luma(picture), kOriginTag));
    ASSERT_TRUE(search.found) << i;
    views.push_back({"still_" + std::to_string(i), model.size, model,
                     camera::rigid_inverse(t) * at[i], *search.found});
  }
  const TrackedTagFit f = fit_of(place(views));
  EXPECT_LT(yaw_error_deg(f.tracking_to_world, t), 0.1);
  for (std::size_t i = 0; i < 2; ++i) {
    const camera::Mat4d placed = f.tracking_to_world * views[i].cam_to_tracking;
    EXPECT_LT(glm::length(centre_of(placed) - centre_of(at[i])), 0.005) << i;
  }
  EXPECT_LT(f.rms_px, 0.5);
}

// The rotation of Rodrigues vector `rvec`.
cv::Matx33d turn(const cv::Vec3d& rvec) {
  return rotation_of(camera::Mat4d(
      camera::rotation_from_rodrigues({rvec[0], rvec[1], rvec[2]})));
}

// A rotation about a random axis by |N(0, sigma)| degrees.
cv::Matx33d jitter(cv::RNG& rng, double sigma_deg) {
  cv::Vec3d axis(rng.gaussian(1.0), rng.gaussian(1.0), rng.gaussian(1.0));
  axis = cv::normalize(axis);
  return turn(axis * (std::abs(rng.gaussian(sigma_deg)) * kDeg));
}

struct Case {
  const char* name;
  int stills;
  double distance_m;
  double off_normal_deg;
  double tilt_deg;      // gravity against the tag's z
  double camera_bar_m;  // p95 of the stills' own camera centres
};

struct Errors {
  std::vector<double> yaw_deg;
  std::vector<double> rotation_deg;
  std::vector<double> camera_m;
  std::vector<double> rms_px;
};

// 300 holds of `c`: fx 1600 at 1920 x 1440, edge kEdge, corner noise 0.5 px,
// the hand's tremor 1 cm / 0.5 degrees between stills, the tracking off by
// 0.1 degrees / 2 mm per still, each trial its own yaw, translation,
// azimuth, roll and tilt direction.
Errors holds(const Case& c, std::uint64_t seed) {
  cv::RNG rng(seed);
  const camera::CameraModel model = test::tracked_pinhole();
  const std::array<cv::Point3d, 4> square =
      test::floor_tag_corners({kEdge, {}, 0.0});
  Errors e;
  for (int trial = 0; trial < 300; ++trial) {
    const double yaw = rng.uniform(-180.0, 180.0);
    const cv::Vec3d t0(rng.gaussian(1.0), rng.gaussian(1.0), rng.gaussian(1.0));
    const double azimuth = rng.uniform(0.0, 360.0);
    const double roll = rng.uniform(-180.0, 180.0);
    const double tilt_dir = rng.uniform(0.0, 2.0 * kPi);
    const cv::Matx33d tilt =
        turn(cv::Vec3d(std::cos(tilt_dir), std::sin(tilt_dir), 0.0) *
             (c.tilt_deg * kDeg));
    const camera::Mat4d t = rigid(tilt, {}) * truth(kYUp, yaw, t0);
    std::vector<TrackedTagView> views;
    std::vector<camera::Vec3d> centres;
    for (int i = 0; i < c.stills; ++i) {
      camera::Mat4d at =
          test::camera_over_tag(c.distance_m, c.off_normal_deg, azimuth, roll);
      at = rigid(
          jitter(rng, 0.5) * rotation_of(at),
          cv::Vec3d(at[3].x + rng.gaussian(0.01), at[3].y + rng.gaussian(0.01),
                    at[3].z + rng.gaussian(0.01)));
      const camera::Mat4d off = rigid(
          jitter(rng, 0.1), cv::Vec3d(rng.gaussian(0.002), rng.gaussian(0.002),
                                      rng.gaussian(0.002)));
      const std::vector<cv::Point2d> pixels = test::project(
          model, camera::rigid_inverse(at), {square.begin(), square.end()});
      TrackedTagView view{"still_" + std::to_string(i),
                          model.size,
                          model,
                          camera::rigid_inverse(t) * at * off,
                          {}};
      view.marker.id = kOriginTag.id;
      for (std::size_t k = 0; k < 4; ++k) {
        view.marker.corners[k] = {pixels[k].x + rng.gaussian(0.5),
                                  pixels[k].y + rng.gaussian(0.5)};
        EXPECT_TRUE(view.marker.corners[k].x > 0.0 &&
                    view.marker.corners[k].x < 1919.0 &&
                    view.marker.corners[k].y > 0.0 &&
                    view.marker.corners[k].y < 1439.0);
      }
      views.push_back(std::move(view));
      centres.push_back(centre_of(at));
    }
    const core::Result<TrackedWorld> found = place(views);
    const TrackedWorld& w = test::value_of(found);
    if (!w.fit) {
      ADD_FAILURE() << c.name << ", trial " << trial << ": " << w.reason;
      continue;
    }
    const camera::Mat4d& fit = w.fit->tracking_to_world;
    e.yaw_deg.push_back(yaw_error_deg(fit, t));
    e.rms_px.push_back(w.fit->rms_px);
    e.rotation_deg.push_back(
        compute::rotation_angle_deg(camera::Mat3d(fit), camera::Mat3d(t)));
    for (std::size_t i = 0; i < views.size(); ++i) {
      e.camera_m.push_back(
          glm::length(centre_of(fit * views[i].cam_to_tracking) - centres[i]));
    }
  }
  return e;
}

// The pass bars: every hold fits (within kMaxTrackedRmsPx), yaw p95 within
// 0.5 degrees, and the stills' own camera centres within 2 cm, or 5 cm from
// 2 stills at 2.2 m or with gravity 0.65 degrees off the tag's z (the
// tracker's basin), whose rotation is then that tilt, 0.65 +- 0.15 degrees.
TEST(TrackedWorld, MeetsThePassBarsOnNoisyHolds) {
  const Case cases[] = {
      {"1.2 m, 40 deg, 2 stills", 2, 1.2, 40.0, 0.0, 0.02},
      {"1.2 m, 40 deg, 4 stills", 4, 1.2, 40.0, 0.0, 0.02},
      {"2.2 m, 55 deg, 4 stills", 4, 2.2, 55.0, 0.0, 0.02},
      {"2.2 m, 55 deg, 2 stills", 2, 2.2, 55.0, 0.0, 0.05},
      {"1.2 m, face-on, 4 stills", 4, 1.2, 0.0, 0.0, 0.02},
      {"1.2 m, 40 deg, tilt 0.65", 4, 1.2, 40.0, 0.65, 0.05},
      {"2.2 m, 55 deg, tilt 0.65", 4, 2.2, 55.0, 0.65, 0.05},
  };
  std::uint64_t seed = 0x5eed;
  for (const Case& c : cases) {
    const Errors e = holds(c, seed++);
    ASSERT_EQ(e.yaw_deg.size(), 300u) << c.name;
    const double yaw = compute::quantile(e.yaw_deg, 0.95);
    const double cam = compute::quantile(e.camera_m, 0.95);
    const double rot_lo = compute::quantile(e.rotation_deg, 0.05);
    const double rot_hi = compute::quantile(e.rotation_deg, 0.95);
    std::printf(
        "%-26s yaw p95 %.3f deg, camera p95 %.1f mm, rotation "
        "p5-p95 %.3f-%.3f deg, RMS p95 %.2f max %.2f px\n",
        c.name, yaw, 1000.0 * cam, rot_lo, rot_hi,
        compute::quantile(e.rms_px, 0.95), compute::quantile(e.rms_px, 1.0));
    EXPECT_LE(yaw, 0.5) << c.name;
    EXPECT_LE(cam, c.camera_bar_m) << c.name;
    if (c.tilt_deg > 0.0) {
      EXPECT_GE(rot_lo, c.tilt_deg - 0.15) << c.name;
      EXPECT_LE(rot_hi, c.tilt_deg + 0.15) << c.name;
    }
  }
}

// The views of an exact hold 1.2 m out at 40 degrees, named still_0...
std::vector<TrackedTagView> hold(int stills, const camera::Mat4d& t) {
  std::vector<TrackedTagView> views;
  for (int i = 0; i < stills; ++i) {
    views.push_back(posed(
        "still_" + std::to_string(i) + ".png",
        test::camera_over_tag(1.2, 40.0, 25.0 + 4.0 * i, 10.0 - 3.0 * i), t));
  }
  return views;
}

const camera::Mat4d kTruth = truth(kYUp, 112.0, {-0.3, 0.6, 1.4});

TEST(TrackedWorld, SeesNoTagWithoutAStill) {
  const TrackedWorld w = test::value_of(place({}));
  EXPECT_FALSE(w.fit);
  EXPECT_EQ(w.miss, TagMiss::NotSeen);
  EXPECT_FALSE(w.reason.empty());
  EXPECT_EQ(w.sought.dictionary, kOriginTag.dictionary);
}

// A camera below the tag's plane, looking up at it, sees its corners
// mirrored, as a tag lying face down shows them: the still is named.
TEST(TrackedWorld, NamesAStillBelowTheTag) {
  std::vector<TrackedTagView> views = hold(1, kTruth);
  views.push_back(posed("still_9.png",
                        test::camera_over_tag(1.2, 150.0, 60.0, 5.0), kTruth));
  const TrackedWorld w = test::value_of(place(views));
  EXPECT_FALSE(w.fit);
  EXPECT_EQ(w.miss, TagMiss::CameraBelowTag);
  EXPECT_EQ(w.still, "still_9.png");
  EXPECT_NE(w.reason.find("still_9.png"), std::string::npos) << w.reason;
  EXPECT_NE(w.reason.find("below the tag's plane"), std::string::npos);
}

// One still's pose moved 10 cm: no still is dropped, and the fit is refused
// for its RMS.
TEST(TrackedWorld, RefusesAPoorFit) {
  std::vector<TrackedTagView> views = hold(2, kTruth);
  views[1].cam_to_tracking[3].x += 0.10;
  const TrackedWorld w = test::value_of(place(views));
  EXPECT_FALSE(w.fit);
  EXPECT_EQ(w.miss, TagMiss::PoorFit);
  EXPECT_NE(w.reason.find("px RMS"), std::string::npos) << w.reason;
  EXPECT_TRUE(w.still.empty());
}

TEST(TrackedWorld, MissesCornersOnALine) {
  std::vector<TrackedTagView> views = hold(2, kTruth);
  std::array<camera::Vec2d, 4>& c = views[1].marker.corners;
  for (std::size_t k = 1; k < 4; ++k) c[k] = c[0] + double(k) * (c[1] - c[0]);
  const TrackedWorld w = test::value_of(place(views));
  EXPECT_FALSE(w.fit);
  EXPECT_EQ(w.miss, TagMiss::Numerical);
  EXPECT_NE(w.reason.find("still_1.png"), std::string::npos) << w.reason;
}

TEST(TrackedWorld, RefusesWhatItCannotTake) {
  const std::vector<TrackedTagView> good = hold(2, kTruth);
  const auto refused =
      [&](const std::vector<TrackedTagView>& views, const std::string& says,
          const camera::Vec3d& up = kYUp, double edge_m = kEdge,
          const OriginTag& tag = kOriginTag) {
        test::expect_refused(place(views, up, edge_m, tag).status(),
                             "tracked_world_from_tag", says);
      };
  refused(good, "options.up", camera::Vec3d(0.0));
  refused(good, "options.up", camera::Vec3d(0.0, std::nan(""), 0.0));
  refused(good, "edge_m", kYUp, 0.0);
  refused(good, "edge_m", kYUp, std::nan(""));
  OriginTag bad = kOriginTag;
  bad.edge_m = -1.0;
  refused(good, "origin_tag.edge_m", kYUp, kEdge, bad);
  std::vector<TrackedTagView> v = good;
  v[1].marker.id = 3;
  refused(v, "of marker 3");
  v = good;
  v[1].still = v[0].still;
  refused(v, "named before");
  v = good;
  v[1].model.intrinsics.fx = 0.0;
  refused(v, "view 1");
  v = good;
  v[1].image_size = {1280, 720};
  refused(v, "1280x720");
  v = good;
  v[1].cam_to_tracking[0] *= 2.0;
  refused(v, "cam_to_tracking");
  v = good;
  v[1].marker.corners[2].y = std::nan("");
  refused(v, "not finite");
}

// The stills a session records for `views`.
camera::TrackedStills stills_of(const std::vector<TrackedTagView>& views) {
  camera::TrackedStills stills;
  stills.sensor = "ipad-pro-m5";
  for (std::uint8_t i = 0; i < 16; ++i) stills.session[i] = 0x10 + i;
  stills.world_epoch = 2;
  std::uint64_t time = 1761957000000000000u;
  for (const TrackedTagView& v : views) {
    stills.stills.push_back({v.still, time, v.model, v.cam_to_tracking});
    time += 100000000u;
  }
  return stills;
}

const camera::WorldFrame kTagWorld{{},
                                   camera::AprilTagWorld{"tag36h11", 0, 0.15}};

// Written by sensor and read back, the registration places each still's
// camera where the fit does.
TEST(RegisterTracked, PlacesTheStillsWhereTheFitDoes) {
  const std::vector<TrackedTagView> views = hold(3, kTruth);
  const camera::TrackedStills stills = stills_of(views);
  const TrackedWorld found = test::value_of(place(views));
  ASSERT_TRUE(found.fit) << found.reason;
  const camera::TrackedRegistration reg =
      test::value_of(register_tracked(stills, kTagWorld, found));
  const test::ScratchDir dir;
  test::check(camera::write_tracked_registration(dir / "registration.json", reg,
                                                 core::WriteMode::CreateNew));
  const camera::TrackedRegistration back = test::value_of(
      camera::read_tracked_registration(dir / "registration.json"));
  EXPECT_EQ(back.sensor, "ipad-pro-m5");
  EXPECT_EQ(back.session, stills.session);
  EXPECT_EQ(back.world_epoch, 2);
  ASSERT_TRUE(back.world.apriltag);
  EXPECT_EQ(back.world.apriltag->family, "tag36h11");
  EXPECT_EQ(back.world.apriltag->id, 0);
  EXPECT_EQ(back.world.apriltag->size_m, 0.15);
  EXPECT_EQ(back.views, 3u);
  EXPECT_EQ(back.edge_m, kEdge);
  EXPECT_EQ(back.rms_px, found.fit->rms_px);
  for (const camera::TrackedStill& s : stills.stills) {
    const camera::Vec3d fitted =
        centre_of(found.fit->tracking_to_world * s.cam_to_tracking);
    EXPECT_LT(
        glm::length(centre_of(back.tracking_to_world * s.cam_to_tracking) -
                    fitted),
        1e-9)
        << s.image;
  }
}

TEST(RegisterTracked, RefusesAMissAndAnotherWorld) {
  const std::vector<TrackedTagView> views = hold(2, kTruth);
  const camera::TrackedStills stills = stills_of(views);
  const TrackedWorld found = test::value_of(place(views));
  ASSERT_TRUE(found.fit);
  const auto refused = [&](const camera::TrackedStills& s,
                           const camera::WorldFrame& world,
                           const TrackedWorld& f, const std::string& says) {
    test::expect_refused(register_tracked(s, world, f).status(),
                         "register_tracked", says);
  };
  const TrackedWorld none = test::value_of(place({}));
  refused(stills, kTagWorld, none, none.reason);
  refused(stills, camera::WorldFrame{"femto-1", {}}, found,
          "sensor \"femto-1\"");
  camera::WorldFrame other = kTagWorld;
  other.apriltag->id = 1;
  refused(stills, other, found, "its id 1");
  other = kTagWorld;
  other.apriltag->size_m = 0.16;
  refused(stills, other, found, "its size 0.16 m");
  camera::TrackedStills zero = stills;
  zero.session = {};
  refused(zero, kTagWorld, found, "session");
}

}  // namespace
}  // namespace volumetric_kit::calib::solve
