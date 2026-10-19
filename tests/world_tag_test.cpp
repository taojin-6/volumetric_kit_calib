// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The world at the origin tag on the synthetic rig (z up, the floor at
// z = 0), for 2 to 5 cameras whose ids sort in another order than the rig's:
// the tag located from the rig's truth, exactly without noise and within the
// noise at 0.3 px; its frame's axes, its size not assumed, each miss,
// outliers and order; the refusals; the outputs moved into its world; and
// the whole chain from the bundle adjustment to the anchored poses.

#include "volumetric_kit/calib/solve/world_tag.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <glm/trigonometric.hpp>

#include "result_checks.hpp"
#include "synthetic_scene.hpp"

namespace volumetric_kit::calib::solve {
namespace {

namespace camera = recon::camera;
using test::FloorTag;
using test::SyntheticCamera;

// The rig at its truth in camera `reference`'s gauge, as bundle_adjust
// places it: each T_cw, the reference's exactly zero, and recon's
// calibration of them, its sensors in id order.
RigSolution truth_solution(const std::vector<SyntheticCamera>& rig,
                           std::size_t reference) {
  RigSolution s;
  s.reference = reference;
  const camera::Mat4d reference_to_world =
      camera::rigid_inverse(rig[reference].world_to_camera);
  for (std::size_t c = 0; c < rig.size(); ++c) {
    s.world_to_camera.push_back(
        c == reference ? camera::RodriguesTransform{}
                       : camera::rodrigues_from_matrix(rig[c].world_to_camera *
                                                       reference_to_world));
  }
  std::vector<std::size_t> by_id(rig.size());
  std::iota(by_id.begin(), by_id.end(), std::size_t{0});
  std::sort(by_id.begin(), by_id.end(), [&](std::size_t a, std::size_t b) {
    return rig[a].id < rig[b].id;
  });
  s.calibration.world.sensor = rig[reference].id;
  for (const std::size_t c : by_id) {
    camera::SensorCalibration& sensor = s.calibration.sensors.emplace_back();
    sensor.id = rig[c].id;
    sensor.color_to_world = camera::rigid_inverse(
        camera::matrix_from_rodrigues(s.world_to_camera[c]));
    sensor.color = camera::CameraCalibration{rig[c].model,
                                             camera::IntrinsicsSource::Factory};
  }
  return s;
}

// The tag's frame to the world's.
camera::Mat4d tag_to_world(const FloorTag& tag) {
  const double a = glm::radians(tag.yaw_deg);
  camera::Mat4d m(1.0);
  m[0] = glm::dvec4(std::cos(a), std::sin(a), 0.0, 0.0);
  m[1] = glm::dvec4(-std::sin(a), std::cos(a), 0.0, 0.0);
  m[3] = glm::dvec4(tag.centre, 1.0);
  return m;
}

// The reference's frame to the tag's, in truth.
camera::Mat4d true_reference_to_tag(const std::vector<SyntheticCamera>& rig,
                                    std::size_t reference,
                                    const FloorTag& tag = {}) {
  return camera::rigid_inverse(tag_to_world(tag)) *
         camera::rigid_inverse(rig[reference].world_to_camera);
}

std::vector<TagView> tag_views(const std::vector<SyntheticCamera>& rig,
                               const std::vector<test::MarkerSighting>& seen) {
  std::vector<TagView> views;
  for (const test::MarkerSighting& s : seen) {
    views.push_back(
        {rig[s.camera].id, s.capture, rig[s.camera].model.size, s.marker});
  }
  return views;
}

// What the rig sees of `tag` over `captures` captures.
std::vector<TagView> see(const std::vector<SyntheticCamera>& rig,
                         const FloorTag& tag, std::size_t captures,
                         double noise_px, std::uint32_t seed = 1) {
  return tag_views(rig, test::observe_marker(rig, kOriginTag.id,
                                             test::floor_tag_corners(tag),
                                             captures, {noise_px, seed}));
}

core::Result<WorldOrigin> locate(const std::vector<SyntheticCamera>& rig,
                                 const RigSolution& solution,
                                 const std::vector<TagView>& views,
                                 const TagFrameOptions& options = {},
                                 const OriginTag& tag = kOriginTag) {
  core::Result<WorldOrigin> origin = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(origin = origin_tag_frame(test::rig_cameras(rig), solution,
                                            views, tag, options));
  return origin;
}

// The tag's world, or the test fails here.
TagWorld world_of(const core::Result<WorldOrigin>& origin) {
  const WorldOrigin& o = test::value_of(origin);
  if (!o.tag) throw std::runtime_error("no tag world: " + o.reason);
  return *o.tag;
}

// How far one frame's origin (metres) and axes (degrees) are from another's,
// each the reference's frame to a tag's.
std::pair<double, double> frame_error(const camera::Mat4d& a,
                                      const camera::Mat4d& b) {
  const camera::Mat4d ia = camera::rigid_inverse(a);
  const camera::Mat4d ib = camera::rigid_inverse(b);
  return {glm::length(camera::Vec3d(ia[3]) - camera::Vec3d(ib[3])),
          test::angle_deg(ia, ib)};
}

double largest_difference(const camera::Mat4d& a, const camera::Mat4d& b) {
  double d = 0.0;
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      d = std::max(d, std::abs(a[col][row] - b[col][row]));
    }
  }
  return d;
}

camera::Vec3d apply(const camera::Mat4d& m, const camera::Vec3d& p) {
  return camera::Vec3d(m * glm::dvec4(p, 1.0));
}

// Without noise the tag is where the truth puts it, to round-off: its
// frame, corners, edges, the cameras' heights, and every view used, by
// capture and then camera id.
TEST(OriginTagFrame, IsExactWithoutNoise) {
  for (std::size_t n = 2; n <= test::kKeptSceneMaxCameras; ++n) {
    SCOPED_TRACE(std::to_string(n) + " cameras");
    const std::vector<SyntheticCamera> rig = test::kept_scene_rig(n);
    const std::vector<TagView> views = see(rig, {}, 3, 0.0);
    ASSERT_EQ(views.size(), 3 * n);  // every camera sees it
    const WorldOrigin origin =
        test::value_of(locate(rig, truth_solution(rig, 1), views));
    ASSERT_TRUE(origin.tag) << origin.reason;
    EXPECT_TRUE(origin.reason.empty());
    const TagWorld& w = *origin.tag;
    EXPECT_LT(
        largest_difference(w.reference_to_world, true_reference_to_tag(rig, 1)),
        1e-9);
    EXPECT_EQ(origin.reference_to_world(), w.reference_to_world);
    const std::array<cv::Point3d, 4> truth = test::floor_tag_corners();
    for (std::size_t k = 0; k < 4; ++k) {
      EXPECT_LT(glm::length(w.corners_m[k] -
                            camera::Vec3d(truth[k].x, truth[k].y, truth[k].z)),
                1e-9);
      EXPECT_NEAR(w.edges_m[k], 0.15, 1e-9);
    }
    EXPECT_NEAR(w.diagonals_m[0], 0.15 * std::sqrt(2.0), 1e-9);
    EXPECT_NEAR(w.diagonals_m[1], 0.15 * std::sqrt(2.0), 1e-9);
    EXPECT_NEAR(w.edge_m, 0.15, 1e-9);
    EXPECT_NEAR(w.edge_error_percent, 0.0, 1e-6);
    EXPECT_FALSE(w.edge_warning);
    EXPECT_LT(w.out_of_plane_m, 1e-9);
    EXPECT_LT(w.rms_px, 1e-6);
    EXPECT_LT(w.worst_px, 1e-6);
    EXPECT_EQ(w.solves, 1);
    ASSERT_EQ(w.cameras.size(), n);
    for (std::size_t c = 0; c < n; ++c) {
      EXPECT_EQ(w.cameras[c].id, rig[c].id);
      EXPECT_EQ(w.cameras[c].views, 3);
      EXPECT_EQ(w.cameras[c].used, 3);
      EXPECT_NEAR(w.cameras[c].height_m,
                  test::centre_of(rig[c].world_to_camera).z, 1e-9);
    }
    ASSERT_EQ(w.views.size(), 3 * n);
    for (std::size_t i = 0; i < w.views.size(); ++i) {
      EXPECT_TRUE(w.views[i].used);
      EXPECT_EQ(w.views[i].round, 0);
      if (i == 0) continue;
      const TagViewFit& a = w.views[i - 1];
      const TagViewFit& b = w.views[i];
      EXPECT_TRUE(a.capture < b.capture ||
                  (a.capture == b.capture && a.camera < b.camera));
    }
  }
}

// At 0.3 px of noise, 5 captures, against bounds about 1.3 times the worst
// over seeds 1 to 50. For 2 cameras that worst was 0.51 mm for the origin,
// 0.29 degrees for the axes, 9.7 mm for a height and 0.64 mm for an edge;
// for 3 to 5, 0.26 mm, 0.23 degrees, 6.6 mm and 0.55 mm. The axes' RMS error
// is 0.15 degrees for 2 cameras and 0.08-0.10 for 3 to 5, and a camera's
// height follows the tilt over its 2 m or so from the tag.
TEST(OriginTagFrame, PlacesTheTagWithinTheNoise) {
  for (std::size_t n = 2; n <= test::kKeptSceneMaxCameras; ++n) {
    const std::vector<SyntheticCamera> rig = test::kept_scene_rig(n);
    const RigSolution solution = truth_solution(rig, 1);
    const bool two = n == 2;
    for (std::uint32_t seed = 1; seed <= 3; ++seed) {
      SCOPED_TRACE(std::to_string(n) + " cameras, seed " +
                   std::to_string(seed));
      const TagWorld w =
          world_of(locate(rig, solution, see(rig, {}, 5, 0.3, seed)));
      const auto [moved_m, turned_deg] =
          frame_error(w.reference_to_world, true_reference_to_tag(rig, 1));
      EXPECT_LT(moved_m, two ? 0.7e-3 : 0.35e-3);
      EXPECT_LT(turned_deg, two ? 0.4 : 0.3);
      for (std::size_t c = 0; c < n; ++c) {
        EXPECT_NEAR(w.cameras[c].height_m,
                    test::centre_of(rig[c].world_to_camera).z,
                    two ? 13e-3 : 9e-3);
      }
      for (const double edge : w.edges_m) {
        EXPECT_NEAR(edge, 0.15, two ? 0.85e-3 : 0.7e-3);
      }
      EXPECT_FALSE(w.edge_warning);
      EXPECT_LT(w.rms_px, 0.5);
      for (const TagViewFit& v : w.views) EXPECT_TRUE(v.used);
    }
  }
}

// The frame is the tag's own, wherever it lies and however it is turned: x
// from its first corner to its second, z up out of its face, every camera
// above it.
TEST(OriginTagFrame, TakesTheTagsOwnAxes) {
  const std::vector<SyntheticCamera> rig = test::kept_scene_rig(4);
  const FloorTag tag{0.15, {0.4, -0.2, 0.0}, 30.0};
  const std::vector<TagView> views = see(rig, tag, 3, 0.0);
  ASSERT_GE(views.size(), 3u * 3u);  // at least 3 cameras see it
  const TagWorld w = world_of(locate(rig, truth_solution(rig, 1), views));
  EXPECT_LT(largest_difference(w.reference_to_world,
                               true_reference_to_tag(rig, 1, tag)),
            1e-9);
  // In the reference's frame: x along the first corner to the second, z
  // the floor's up.
  const camera::Mat4d tag_in_reference =
      camera::rigid_inverse(w.reference_to_world);
  const camera::Mat4d world_to_reference = rig[1].world_to_camera;
  const std::array<cv::Point3d, 4> corners = test::floor_tag_corners(tag);
  const camera::Vec3d first =
      apply(world_to_reference, {corners[0].x, corners[0].y, corners[0].z});
  const camera::Vec3d second =
      apply(world_to_reference, {corners[1].x, corners[1].y, corners[1].z});
  EXPECT_LT(glm::length(camera::Vec3d(tag_in_reference[0]) -
                        glm::normalize(second - first)),
            1e-9);
  EXPECT_LT(glm::length(camera::Vec3d(tag_in_reference[2]) -
                        camera::Mat3d(world_to_reference) *
                            camera::Vec3d(0.0, 0.0, 1.0)),
            1e-9);
  for (const TagCamera& c : w.cameras) EXPECT_GT(c.height_m, 1.0) << c.id;
}

// No size is assumed: a print 148 mm across reads 148 mm, with the same
// frame, and one 155 mm across is warned of.
TEST(OriginTagFrame, AssumesNoTagSize) {
  const std::vector<SyntheticCamera> rig = test::kept_scene_rig(4);
  const RigSolution solution = truth_solution(rig, 1);
  const auto at = [&](double edge_m, const TagFrameOptions& options = {}) {
    return world_of(locate(rig, solution,
                           see(rig, {edge_m, camera::Vec3d(0.0), 0.0}, 5, 0.3),
                           options));
  };
  const TagWorld printed = at(0.150);
  const TagWorld small = at(0.148);
  EXPECT_NEAR(small.edge_m, 0.148, 0.5e-3);
  EXPECT_NEAR(small.edge_error_percent, -100.0 * 0.002 / 0.15, 0.35);
  EXPECT_FALSE(small.edge_warning);
  const auto [moved_m, turned_deg] =
      frame_error(small.reference_to_world, printed.reference_to_world);
  EXPECT_LT(moved_m, 0.1e-3);
  EXPECT_LT(turned_deg, 0.2);
  TagFrameOptions strict;
  strict.edge_warn_fraction = 0.01;
  EXPECT_TRUE(at(0.148, strict).edge_warning);

  const TagWorld large = at(0.155);
  EXPECT_NEAR(large.edge_m, 0.155, 0.5e-3);
  EXPECT_NEAR(large.edge_error_percent, 100.0 * 0.005 / 0.15, 0.35);
  EXPECT_TRUE(large.edge_warning);
}

WorldOrigin missed(const core::Result<WorldOrigin>& result) {
  const WorldOrigin& origin = test::value_of(result);
  EXPECT_FALSE(origin.tag);
  EXPECT_FALSE(origin.reason.empty());
  EXPECT_EQ(origin.reference_to_world(), camera::Mat4d(1.0));
  return origin;
}

// Each miss, named: the reference stays the origin.
TEST(OriginTagFrame, NamesEachMiss) {
  const std::vector<SyntheticCamera> rig = test::kept_scene_rig(4);
  const RigSolution solution = truth_solution(rig, 1);
  const std::vector<TagView> all = see(rig, {}, 5, 0.3);

  const WorldOrigin none = missed(locate(rig, solution, {}));
  EXPECT_EQ(none.miss, TagMiss::NotSeen);
  EXPECT_TRUE(none.camera.empty());

  std::vector<TagView> one;
  for (const TagView& v : all) {
    if (v.camera == rig[2].id) one.push_back(v);
  }
  const WorldOrigin alone = missed(locate(rig, solution, one));
  EXPECT_EQ(alone.miss, TagMiss::OneCamera);
  EXPECT_EQ(alone.camera, rig[2].id);

  // Mirrored corners put every camera below the tag: the first by id is
  // named.
  std::vector<TagView> mirrored = all;
  for (TagView& v : mirrored)
    std::swap(v.marker.corners[1], v.marker.corners[3]);
  const WorldOrigin below = missed(locate(rig, solution, mirrored));
  EXPECT_EQ(below.miss, TagMiss::CameraBelowTag);
  EXPECT_EQ(below.camera, "CL0B");
  EXPECT_NE(below.reason.find("face down"), std::string::npos) << below.reason;

  // Two cameras where one sees it more often is not enough when
  // min_cameras asks for 3.
  std::vector<TagView> two;
  for (const TagView& v : all) {
    if (v.camera == rig[0].id || v.camera == rig[1].id) two.push_back(v);
  }
  TagFrameOptions three;
  three.min_cameras = 3;
  EXPECT_EQ(missed(locate(rig, solution, two, three)).miss,
            TagMiss::FewCameras);

  // Corners that meet at one point make no square.
  std::vector<TagView> point = all;
  for (TagView& v : point) {
    for (camera::Vec2d& corner : v.marker.corners) {
      corner = v.marker.corners[0];
    }
  }
  const WorldOrigin collapsed = missed(locate(rig, solution, point));
  EXPECT_EQ(collapsed.miss, TagMiss::Numerical);
  EXPECT_NE(collapsed.reason.find("no square"), std::string::npos)
      << collapsed.reason;

  EXPECT_STREQ(to_string(TagMiss::NotSeen), "not_seen");
  EXPECT_STREQ(to_string(TagMiss::OneCamera), "one_camera");
  EXPECT_STREQ(to_string(TagMiss::FewCameras), "few_cameras");
  EXPECT_STREQ(to_string(TagMiss::CameraBelowTag), "camera_below_tag");
  EXPECT_STREQ(to_string(TagMiss::Numerical), "numerical");
  EXPECT_STREQ(to_string(static_cast<TagMiss>(99)), "unknown");
}

// Outlier rounds that leave one camera: camera 1's one view, its corners
// turned 20 degrees about their centre, cannot meet camera 0's rays.
TEST(OriginTagFrame, MissesWhenOutliersLeaveOneCamera) {
  const std::vector<SyntheticCamera> rig = test::kept_scene_rig(2);
  std::vector<TagView> views;
  for (const TagView& v : see(rig, {}, 5, 0.0)) {
    if (v.camera == rig[0].id || v.capture == 0) views.push_back(v);
  }
  ASSERT_EQ(views.size(), 6u);
  for (TagView& v : views) {
    if (v.camera != rig[1].id) continue;
    camera::Vec2d centre(0.0);
    for (const camera::Vec2d& c : v.marker.corners) centre += 0.25 * c;
    const double a = glm::radians(20.0);
    for (camera::Vec2d& c : v.marker.corners) {
      const camera::Vec2d d = c - centre;
      c = centre + camera::Vec2d(std::cos(a) * d.x - std::sin(a) * d.y,
                                 std::sin(a) * d.x + std::cos(a) * d.y);
    }
  }
  const WorldOrigin origin = missed(locate(rig, truth_solution(rig, 1), views));
  EXPECT_EQ(origin.miss, TagMiss::FewCameras);
  EXPECT_NE(origin.reason.find("1 camera keeps a view"), std::string::npos)
      << origin.reason;
}

// Two cameras whose rays disagree by a few pixels, as a small pose or lens
// error makes them: camera 0's corners 3.4 px off in every view put both in
// Huber's linear region, where cv::LevMarq crawls, and the tag is still
// placed between them.
TEST(OriginTagFrame, PlacesTheTagBetweenTwoCamerasThatDisagree) {
  const std::vector<SyntheticCamera> rig = test::kept_scene_rig(2);
  for (const std::size_t captures : {1u, 5u, 20u}) {
    SCOPED_TRACE(std::to_string(captures) + " captures");
    std::vector<TagView> views = see(rig, {}, captures, 0.0);
    ASSERT_EQ(views.size(), 2 * captures);
    for (TagView& v : views) {
      if (v.camera != rig[0].id) continue;
      for (camera::Vec2d& c : v.marker.corners) c += camera::Vec2d(3.0, 1.5);
    }
    const WorldOrigin origin =
        test::value_of(locate(rig, truth_solution(rig, 1), views));
    ASSERT_TRUE(origin.tag) << origin.reason;
    EXPECT_EQ(origin.tag->solves, 1);
    EXPECT_NEAR(origin.tag->rms_px, 1.47, 0.05);
    for (const TagViewFit& v : origin.tag->views) EXPECT_TRUE(v.used);
  }
}

// A tag kicked 6 cm in one capture: every view of that capture is dropped
// in the first round, and listed, and the frame is the still tag's.
TEST(OriginTagFrame, DropsTheViewsOfATagMoved) {
  const std::vector<SyntheticCamera> rig = test::kept_scene_rig(4);
  std::vector<TagView> views = see(rig, {}, 5, 0.3);
  const std::vector<TagView> kicked =
      see(rig, {0.15, {0.05, 0.03, 0.0}, 4.0}, 1, 0.3, 2);
  ASSERT_EQ(kicked.size(), 4u);
  for (TagView& v : views) {
    if (v.capture != 2) continue;
    for (const TagView& k : kicked) {
      if (k.camera == v.camera) v.marker = k.marker;
    }
  }
  const TagWorld w = world_of(locate(rig, truth_solution(rig, 1), views));
  EXPECT_EQ(w.solves, 2);
  for (const TagViewFit& v : w.views) {
    SCOPED_TRACE(v.camera + ", capture " + std::to_string(v.capture));
    EXPECT_EQ(v.used, v.capture != 2);
    EXPECT_EQ(v.round, v.capture == 2 ? 1 : 0);
    if (v.capture == 2) {
      EXPECT_GE(v.bound_px, 3.0);
      EXPECT_GT(v.worst_px, v.bound_px);
    }
  }
  for (const TagCamera& c : w.cameras) {
    EXPECT_EQ(c.views, 5);
    EXPECT_EQ(c.used, 4);
  }
  const auto [moved_m, turned_deg] =
      frame_error(w.reference_to_world, true_reference_to_tag(rig, 1));
  EXPECT_LT(moved_m, 0.5e-3);
  EXPECT_LT(turned_deg, 0.5);
}

// The views' order changes nothing, to the bit.
TEST(OriginTagFrame, DoesNotDependOnTheViewsOrder) {
  const std::vector<SyntheticCamera> rig = test::kept_scene_rig(5);
  const RigSolution solution = truth_solution(rig, 1);
  std::vector<TagView> views = see(rig, {}, 4, 0.3);
  const TagWorld first = world_of(locate(rig, solution, views));
  std::reverse(views.begin(), views.end());
  std::rotate(views.begin(), views.begin() + 3, views.end());
  const TagWorld second = world_of(locate(rig, solution, views));
  EXPECT_EQ(first.reference_to_world, second.reference_to_world);
  EXPECT_EQ(first.rms_px, second.rms_px);
  ASSERT_EQ(first.views.size(), second.views.size());
  for (std::size_t i = 0; i < first.views.size(); ++i) {
    EXPECT_EQ(first.views[i].camera, second.views[i].camera);
    EXPECT_EQ(first.views[i].capture, second.views[i].capture);
    EXPECT_EQ(first.views[i].worst_px, second.views[i].worst_px);
  }
}

// Bad input is refused, naming what is wrong, and nothing throws.
TEST(OriginTagFrame, RefusesWhatItCannotTake) {
  const std::vector<SyntheticCamera> rig = test::kept_scene_rig(3);
  const RigSolution solution = truth_solution(rig, 1);
  const std::vector<TagView> views = see(rig, {}, 2, 0.3);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto refused = [&](const std::vector<SyntheticCamera>& cams,
                           const RigSolution& s, const std::vector<TagView>& v,
                           const TagFrameOptions& options, const OriginTag& tag,
                           const std::string& says) {
    SCOPED_TRACE(says);
    test::expect_refused(locate(cams, s, v, options, tag).status(),
                         "origin_tag_frame", says);
  };
  const auto with = [&](auto change, const std::string& says) {
    TagFrameOptions options;
    change(options);
    refused(rig, solution, views, options, kOriginTag, says);
  };
  with([](TagFrameOptions& o) { o.huber_px = 0.0; },
       "options.huber_px must be positive and finite");
  with([&](TagFrameOptions& o) { o.outlier_px = nan; },
       "options.outlier_px must be positive and finite");
  with([](TagFrameOptions& o) { o.outlier_mads = -1.0; },
       "options.outlier_mads must be positive and finite");
  with([](TagFrameOptions& o) { o.edge_warn_fraction = 0.0; },
       "options.edge_warn_fraction must be positive and finite");
  with([](TagFrameOptions& o) { o.outlier_rounds = -1; },
       "options.outlier_rounds must be at least 0");
  with([](TagFrameOptions& o) { o.min_cameras = 1; },
       "options.min_cameras must be at least 2");

  OriginTag no_dictionary = kOriginTag;
  no_dictionary.dictionary = MarkerDictionary::None;
  refused(rig, solution, views, {}, no_dictionary, "origin_tag.");

  std::vector<SyntheticCamera> unnamed = rig;
  unnamed[2].id.clear();
  refused(unnamed, solution, views, {}, kOriginTag, "camera 2 has no id");

  RigSolution short_of = solution;
  short_of.world_to_camera.pop_back();
  refused(rig, short_of, views, {}, kOriginTag,
          "the solution places 2 cameras of 3");
  RigSolution far = solution;
  far.reference = 3;
  refused(rig, far, views, {}, kOriginTag,
          "the solution's reference 3 is not one of the rig's 3 cameras");
  RigSolution unknown = solution;
  unknown.world_to_camera[2].tvec.y = nan;
  refused(rig, unknown, views, {}, kOriginTag,
          "the solution's pose of camera " + rig[2].id + " is not finite");

  const auto view = [&](auto change, const std::string& says) {
    std::vector<TagView> changed = views;
    change(changed[3]);
    refused(rig, solution, changed, {}, kOriginTag, says);
  };
  const std::string third = "view 3 (camera " + views[3].camera + ", capture " +
                            std::to_string(views[3].capture) + ")";
  view([](TagView& v) { v.camera = "nobody"; },
       "view 3 names camera \"nobody\", not one of the rig's");
  view([](TagView& v) { v.marker.id = 7; },
       third + " is of marker 7, not the tag's id 0");
  view([](TagView& v) { v.image_size = {1920, 1080}; },
       third + " is of a 1920x1080 image, not its lens's 3840x2160");
  view([&](TagView& v) { v.marker.corners[2].x = nan; },
       third + " has a corner not finite");
  view(
      [&](TagView& v) {
        v.camera = views[4].camera;
        v.capture = views[4].capture;
      },
      "camera " + views[4].camera + " has two views of capture " +
          std::to_string(views[4].capture));
}

// A solution solved by the bundle adjustment, and its tag's world.
struct Solved {
  test::Scene scene;
  RigSolution solution;
  WorldOrigin origin;
};

Solved solved(std::size_t cameras, std::size_t placements,
              const FloorTag& tag = {}, std::uint32_t seed = 1) {
  Solved s{test::make_kept_scene(cameras, placements, 0.3, seed), {}, {}};
  const InitialRig start = test::value_of(
      initialise(s.scene.cameras, s.scene.board, s.scene.views, 1));
  s.solution = test::value_of(
      bundle_adjust(s.scene.cameras, s.scene.board, s.scene.views, start));
  s.origin = test::value_of(
      locate(s.scene.rig, s.solution,
             see(s.scene.rig, tag, s.scene.plan.size(), 0.3, seed + 100)));
  return s;
}

core::Result<AnchoredRig> anchor(const RigSolution& solution,
                                 const WorldOrigin& origin) {
  core::Result<AnchoredRig> rig = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(rig = anchor_rig(solution, origin));
  return rig;
}

// Without a tag the outputs are the solution's own, to the bit: the
// reference's world, its id, and each pose, by id and in rig order.
TEST(AnchorRig, LeavesTheReferencesWorldAsItIs) {
  const Solved s = solved(4, 20);
  const AnchoredRig rig = test::value_of(anchor(s.solution, WorldOrigin{}));
  EXPECT_EQ(rig.calibration.world.sensor, s.solution.calibration.world.sensor);
  ASSERT_EQ(rig.calibration.sensors.size(),
            s.solution.calibration.sensors.size());
  for (std::size_t i = 0; i < rig.calibration.sensors.size(); ++i) {
    EXPECT_EQ(rig.calibration.sensors[i].id,
              s.solution.calibration.sensors[i].id);
    EXPECT_EQ(rig.calibration.sensors[i].color_to_world,
              s.solution.calibration.sensors[i].color_to_world);
  }
  ASSERT_EQ(rig.color_to_world.size(), 4u);
  for (std::size_t c = 0; c < 4; ++c) {
    EXPECT_EQ(rig.color_to_world[c],
              camera::rigid_inverse(camera::matrix_from_rodrigues(
                  s.solution.world_to_camera[c])));
  }
  EXPECT_EQ(rig.color_to_world[1], camera::Mat4d(1.0));
  ASSERT_EQ(rig.boards_to_world.size(), s.solution.boards.size());
  for (std::size_t b = 0; b < rig.boards_to_world.size(); ++b) {
    EXPECT_EQ(rig.boards_to_world[b], camera::matrix_from_rodrigues(
                                          s.solution.boards[b].board_to_world));
  }
}

// With a tag the outputs move into its world: every camera sees every point
// and board where it did, each sensor is found by its id though the rig's
// order is not the ids', and the file names no world but validates and reads
// back.
TEST(AnchorRig, MovesTheOutputsIntoTheTagsWorld) {
  const Solved s = solved(4, 20, {0.15, {0.3, 0.1, 0.0}, -50.0});
  ASSERT_TRUE(s.origin.tag) << s.origin.reason;
  const camera::Mat4d w = s.origin.reference_to_world();
  const AnchoredRig rig = test::value_of(anchor(s.solution, s.origin));

  EXPECT_TRUE(rig.calibration.world.sensor.empty());
  EXPECT_TRUE(camera::validate_array_calibration(rig.calibration).ok());
  const std::string text =
      test::value_of(camera::format_array_calibration(rig.calibration));
  const camera::ArrayCalibration back =
      test::value_of(camera::parse_array_calibration(text));
  EXPECT_TRUE(back.world.sensor.empty());

  const std::vector<camera::Vec3d> points = {
      {0.0, 0.0, 0.0}, {0.4, -0.3, 2.0}, {-1.0, 0.5, 3.5}};
  for (std::size_t c = 0; c < 4; ++c) {
    const std::string& id = s.scene.cameras[c].id;
    SCOPED_TRACE(id);
    const camera::SensorCalibration* sensor =
        camera::find_sensor(rig.calibration, id);
    ASSERT_NE(sensor, nullptr);
    EXPECT_EQ(*sensor->color_to_world, rig.color_to_world[c]);
    const camera::SensorCalibration* read = camera::find_sensor(back, id);
    ASSERT_NE(read, nullptr);
    EXPECT_LT(largest_difference(*read->color_to_world, rig.color_to_world[c]),
              1e-12);

    const camera::Mat4d before =
        camera::matrix_from_rodrigues(s.solution.world_to_camera[c]);
    const camera::Mat4d after = camera::rigid_inverse(rig.color_to_world[c]);
    EXPECT_LT(largest_difference(after * w, before), 1e-12);
    for (const camera::Vec3d& x : points) {
      EXPECT_LT(glm::length(apply(after, apply(w, x)) - apply(before, x)),
                1e-12);
    }
    for (std::size_t b = 0; b < rig.boards_to_world.size(); ++b) {
      EXPECT_LT(
          largest_difference(after * rig.boards_to_world[b],
                             before * camera::matrix_from_rodrigues(
                                          s.solution.boards[b].board_to_world)),
          1e-12);
    }
  }
}

// What anchor_rig cannot place is refused, and nothing throws.
TEST(AnchorRig, RefusesWhatItCannotPlace) {
  const Solved s = solved(3, 12);
  ASSERT_TRUE(s.origin.tag) << s.origin.reason;
  const auto refused = [&](const RigSolution& solution,
                           const WorldOrigin& origin, const std::string& says) {
    SCOPED_TRACE(says);
    test::expect_refused(anchor(solution, origin).status(), "anchor_rig", says);
  };
  refused(RigSolution{}, s.origin, "the solution places no camera");
  WorldOrigin scaled = s.origin;
  scaled.tag->reference_to_world[0][0] *= 1.01;
  refused(s.solution, scaled, "the origin's reference_to_world");
  RigSolution unknown = s.solution;
  unknown.world_to_camera[2].rvec.x = std::numeric_limits<double>::infinity();
  refused(unknown, s.origin, "the solution's pose 2 is not finite");
  RigSolution unposed = s.solution;
  unposed.calibration.sensors[0].color_to_world.reset();
  refused(unposed, s.origin,
          "the solution's sensor " + unposed.calibration.sensors[0].id +
              " is not posed");
  RigSolution adrift = s.solution;
  adrift.boards[0].board_to_world.tvec.z =
      std::numeric_limits<double>::quiet_NaN();
  refused(adrift, s.origin,
          "the solution's board of capture " +
              std::to_string(adrift.boards[0].capture) + " is not finite");
}

// The whole chain, for 2 to 5 cameras: the bundle adjustment of 60
// placements at 0.3 px, the tag seen in each capture at 0.3 px, and the rig
// anchored to it, against the synthetic world's truth, the tag at its
// origin. Over seeds 1 to 8 the worst camera was 5.9 mm and 0.10 degrees off
// with 2 cameras, and 2.7 mm and 0.053 degrees with 3 to 5: mostly the tag's
// own tilt over the cameras' 3 m from it.
TEST(OriginTagFrame, AnchorsASolvedRigToTheTruth) {
  for (std::size_t n = 2; n <= test::kKeptSceneMaxCameras; ++n) {
    SCOPED_TRACE(std::to_string(n) + " cameras");
    const Solved s = solved(n, 60);
    ASSERT_TRUE(s.origin.tag) << s.origin.reason;
    const AnchoredRig rig = test::value_of(anchor(s.solution, s.origin));
    const bool two = n == 2;
    for (std::size_t c = 0; c < n; ++c) {
      const camera::Mat4d truth =
          camera::rigid_inverse(s.scene.rig[c].world_to_camera);
      EXPECT_LT(glm::length(camera::Vec3d(rig.color_to_world[c][3]) -
                            camera::Vec3d(truth[3])),
                two ? 8e-3 : 3.5e-3)
          << s.scene.rig[c].id;
      EXPECT_LT(test::angle_deg(rig.color_to_world[c], truth),
                two ? 0.14 : 0.07)
          << s.scene.rig[c].id;
    }
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::solve
