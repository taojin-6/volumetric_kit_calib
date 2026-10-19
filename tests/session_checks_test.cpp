// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The session's checks on the synthetic rig: views, coverage and shared
// captures as the solve counts them, the graded tree preview and when
// Calibrate may run, drops, and the duplicate, floor-only and frontal
// warnings.

#include "volumetric_kit/calib/quality/session_checks.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;
using Code = core::Status::Code;

constexpr double kPi = 3.14159265358979323846;

struct Scene {
  compute::Board board = test::femto_board();
  std::vector<test::SyntheticCamera> rig = test::synthetic_rig();
  std::vector<solve::RigCamera> cameras;
  // A placement all four cameras see.
  camera::Mat4d everyone{1.0};

  Scene() {
    for (const test::SyntheticCamera& cam : rig) {
      cameras.push_back({cam.id, cam.model});
    }
    test::PlanOptions options;
    options.count = 1;
    options.min_cameras = 4;
    everyone = test::synthetic_plan(rig, board, options).at(0).board_to_world;
  }

  // Each camera's detection of the board at `board_to_world`: those in
  // `seeing` find all of it, with `noise_px` of noise from `seed`, the others
  // nothing.
  std::vector<Detection> capture(const camera::Mat4d& board_to_world,
                                 const std::vector<std::size_t>& seeing,
                                 double noise_px = 0.0,
                                 std::uint32_t seed = 1) const {
    test::ObserveOptions noise;
    noise.noise_px = noise_px;
    noise.seed = seed;
    const std::vector<Observation> views = test::observe(
        rig, board, {test::Placement{board_to_world, seeing}}, noise);
    std::vector<Detection> detections(rig.size());
    for (std::size_t c = 0; c < rig.size(); ++c) {
      detections[c].image_size = rig[c].model.size;
    }
    for (std::size_t i = 0; i < seeing.size(); ++i) {
      detections[seeing[i]].observation = views[i];
    }
    return detections;
  }

  // Adds capture `index`'s views to `views`, as the solve takes them.
  void record(std::vector<Observation>& views, int index,
              const std::vector<Detection>& detections) const {
    for (std::size_t c = 0; c < detections.size(); ++c) {
      if (detections[c].observation.points.empty()) continue;
      Observation view = detections[c].observation;
      view.camera_id = cameras[c].id;
      view.frame_id = index;
      views.push_back(std::move(view));
    }
  }

  SessionChecks checks(std::size_t reference = 0,
                       const SessionOptions& options = {}) const {
    return SessionChecks::create(cameras, board, reference, options).value();
  }

  // The board at `board_to_world` turned `deg` about `axis` (world) through
  // its centre, then moved by `shift`.
  camera::Mat4d moved(const camera::Mat4d& board_to_world,
                      const camera::Vec3d& axis, double deg,
                      const camera::Vec3d& shift = {}) const {
    const camera::Vec3d c = compute::centre_in(board, board_to_world);
    const camera::Mat3d turn = camera::rotation_from_rodrigues(
        glm::normalize(axis) * (deg * kPi / 180.0));
    camera::Mat4d out(turn * camera::Mat3d(board_to_world));
    out[3] = glm::dvec4(
        turn * (camera::Vec3d(board_to_world[3]) - c) + c + shift, 1.0);
    return out;
  }

  // `board_to_world` turned about its centre so its printed face is
  // `tilt_deg` from facing camera `c`: each turn of one placement is about
  // the same axis.
  camera::Mat4d tilted_to(const camera::Mat4d& board_to_world, std::size_t c,
                          double tilt_deg) const {
    const camera::Vec3d to_camera =
        camera::Vec3d(camera::rigid_inverse(rig[c].world_to_camera)[3]) -
        compute::centre_in(board, board_to_world);
    const camera::Vec3d face = compute::face_normal(board_to_world);
    return moved(board_to_world, glm::cross(face, to_camera),
                 compute::angle_deg(face, to_camera) - tilt_deg);
  }

  // `board_to_world` turned `deg` about its own normal: it faces alike.
  camera::Mat4d spun(const camera::Mat4d& board_to_world, double deg) const {
    return moved(board_to_world, compute::face_normal(board_to_world), deg);
  }
};

SessionSummary summary_of(const SessionChecks& checks) {
  core::Result<SessionSummary> summary = core::Status::invalid_argument("");
  EXPECT_NO_THROW(summary = checks.summary());
  EXPECT_TRUE(summary.ok()) << summary.status().message();
  return summary.ok() ? *summary : SessionSummary{};
}

void add(SessionChecks& checks, int index,
         const std::vector<Detection>& detections) {
  core::Status added = core::Status::invalid_argument("");
  EXPECT_NO_THROW(added = checks.add(index, detections));
  EXPECT_TRUE(added.ok()) << added.message();
}

// `summary` counts the views the solve uses, the captures each pair of
// cameras shares through them and the tree's links as solve::initialise does
// on the same views; where initialise finds a camera outside the tree, the
// summary leaves one unlinked and is not ready.
void expect_as_the_solve(const SessionSummary& summary, const Scene& scene,
                         const std::vector<Observation>& views,
                         std::size_t reference,
                         const solve::InitialiseOptions& options = {}) {
  const core::Result<solve::InitialRig> rig =
      solve::initialise(scene.cameras, scene.board, views, reference, options);
  const std::size_t k = scene.cameras.size();
  ASSERT_EQ(summary.cameras.size(), k);
  if (!rig.ok()) {
    ASSERT_EQ(rig.status().domain(), Code::InvalidArgument);
    EXPECT_NE(rig.status().message().find("no capture with the tree"),
              std::string::npos)
        << rig.status().message();
    EXPECT_FALSE(summary.ready);
    bool unlinked = false;
    for (const CameraSummary& cam : summary.cameras) {
      unlinked = unlinked || !cam.linked;
    }
    EXPECT_TRUE(unlinked);
    return;
  }
  std::vector<int> used(k, 0);
  std::vector<int> ambiguous(k, 0);
  std::map<std::int64_t, std::vector<std::size_t>> used_in;
  for (const solve::ViewRecord& view : rig->views) {
    if (view.fate == solve::ViewFate::Used) {
      ++used[view.camera];
      used_in[view.capture].push_back(view.camera);
    }
    if (view.fate == solve::ViewFate::Ambiguous) ++ambiguous[view.camera];
  }
  std::vector<std::vector<std::size_t>> seen_by;
  for (const auto& entry : used_in) seen_by.push_back(entry.second);
  EXPECT_EQ(summary.covisibility, solve::covisibility(k, seen_by).value());
  for (std::size_t c = 0; c < k; ++c) {
    EXPECT_EQ(summary.cameras[c].used_views, used[c]) << c;
    EXPECT_EQ(summary.cameras[c].ambiguous_views, ambiguous[c]) << c;
    EXPECT_TRUE(summary.cameras[c].linked) << c;
  }
  EXPECT_FALSE(summary.cameras[reference].link.has_value());
  bool strong_enough = true;
  for (const solve::PlacedLink& link : rig->links) {
    const CameraSummary& cam = summary.cameras[link.camera];
    ASSERT_TRUE(cam.link.has_value()) << link.camera;
    EXPECT_EQ(cam.link->parent, link.parent) << link.camera;
    EXPECT_EQ(cam.link->shared, link.shared) << link.camera;
    EXPECT_EQ(cam.link->strength == LinkStrength::Weak, link.weak)
        << link.camera;
    strong_enough = strong_enough && link.shared >= 3;
  }
  EXPECT_EQ(summary.ready, k > 1 && strong_enough);
}

// Over a plan of noisy placements: each camera's usable views, and the views,
// shared captures and tree the solve finds; each usable corner lands in its
// cell of the coverage grid.
TEST(SessionChecks, CountsViewsSharesAndCoverageAsTheSolveDoes) {
  const Scene scene;
  test::PlanOptions options;
  options.count = 30;
  const std::vector<test::Placement> plan =
      test::synthetic_plan(scene.rig, scene.board, options);
  SessionChecks checks = scene.checks();
  std::vector<Observation> views;
  std::vector<int> seen(4, 0);
  std::vector<std::array<int, kCoverageCols * kCoverageRows>> cells(4);
  for (std::size_t i = 0; i < plan.size(); ++i) {
    const std::vector<Detection> detections =
        scene.capture(plan[i].board_to_world, plan[i].cameras, 0.3,
                      static_cast<std::uint32_t>(i + 1));
    add(checks, static_cast<int>(i), detections);
    scene.record(views, static_cast<int>(i), detections);
    for (const std::size_t c : plan[i].cameras) {
      ++seen[c];
      for (const DetectedPoint& p : detections[c].observation.points) {
        const int col = static_cast<int>(std::floor((p.pixel.x + 0.5) / 480));
        const int row = static_cast<int>(std::floor((p.pixel.y + 0.5) / 360));
        ++cells[c][static_cast<std::size_t>(row * kCoverageCols + col)];
      }
    }
  }
  const SessionSummary summary = summary_of(checks);
  EXPECT_EQ(summary.kept, 30);
  EXPECT_EQ(summary.dropped, 0);
  ASSERT_EQ(summary.cameras.size(), 4u);
  for (std::size_t c = 0; c < 4; ++c) {
    const CameraSummary& cam = summary.cameras[c];
    EXPECT_EQ(cam.usable_views, seen[c]) << c;
    EXPECT_EQ(cam.coverage, cells[c]) << c;
    int filled = 0;
    for (const int n : cells[c]) filled += n > 0 ? 1 : 0;
    EXPECT_EQ(cam.coverage_filled, filled / 48.0) << c;
    EXPECT_GT(cam.coverage_filled, 0.0) << c;
  }
  expect_as_the_solve(summary, scene, views, 0);
}

// A board facing a camera fits two poses almost alike, so the solve places
// no camera by that view: a link whose shared captures all face one of its
// cameras holds Calibrate back, as initialise refuses it, until captures the
// solve uses join the two. The solve's thresholds are the session's.
TEST(SessionChecks, CountsOnlyTheViewsTheSolveUses) {
  Scene scene;
  scene.rig.resize(2);
  scene.cameras.resize(2);
  const compute::CvCamera lens =
      compute::cv_camera(scene.cameras[1].model).value();
  const camera::Mat4d face_on = scene.tilted_to(scene.everyone, 1, 0.0);
  std::vector<std::vector<Detection>> captures;
  // Noisy captures facing SYN1 whose view in it is ambiguous; most are.
  for (std::uint32_t seed = 1; captures.size() < 3 && seed <= 20; ++seed) {
    std::vector<Detection> detections = scene.capture(
        scene.spun(face_on, 10.0 * static_cast<double>(captures.size())),
        {0, 1}, 0.3, seed);
    const solve::ViewPose pose =
        solve::view_pose(lens, scene.board, detections[1].observation).value();
    if (pose.ambiguity() < 2.0) captures.push_back(std::move(detections));
  }
  ASSERT_EQ(captures.size(), 3u);
  const camera::Mat4d tilted = scene.tilted_to(scene.everyone, 1, 35.0);
  const auto session = [&](std::size_t count, const SessionOptions& options,
                           std::vector<Observation>& views) {
    SessionChecks checks = scene.checks(0, options);
    for (std::size_t i = 0; i < count; ++i) {
      const int index = static_cast<int>(i);
      const std::vector<Detection> detections =
          i < captures.size()
              ? captures[i]
              : scene.capture(scene.spun(tilted, 10.0 * static_cast<double>(i)),
                              {0, 1}, 0.3, 100 + static_cast<std::uint32_t>(i));
      add(checks, index, detections);
      scene.record(views, index, detections);
    }
    return summary_of(checks);
  };

  std::vector<Observation> views;
  SessionSummary summary = session(3, {}, views);
  EXPECT_EQ(summary.cameras[1].usable_views, 3);
  EXPECT_EQ(summary.cameras[1].used_views, 0);
  EXPECT_EQ(summary.cameras[1].ambiguous_views, 3);
  EXPECT_EQ(summary.cameras[0].used_views, 3);
  EXPECT_EQ(summary.covisibility[0][1], 0);
  EXPECT_FALSE(summary.cameras[1].linked);
  EXPECT_FALSE(summary.ready);
  expect_as_the_solve(summary, scene, views, 0);

  views.clear();
  summary = session(6, {}, views);
  EXPECT_EQ(summary.cameras[1].usable_views, 6);
  EXPECT_EQ(summary.cameras[1].used_views, 3);
  EXPECT_EQ(summary.cameras[1].ambiguous_views, 3);
  ASSERT_TRUE(summary.cameras[1].link.has_value());
  EXPECT_EQ(summary.cameras[1].link->shared, 3);
  EXPECT_TRUE(summary.ready);
  expect_as_the_solve(summary, scene, views, 0);

  // Every view fits at least as well as its mirror, so a ratio of 1 takes
  // them all; and no noisy view fits within 0.01 px.
  SessionOptions options;
  options.solve.min_ambiguity = 1.0;
  views.clear();
  summary = session(6, options, views);
  EXPECT_EQ(summary.cameras[1].used_views, 6);
  EXPECT_EQ(summary.covisibility[0][1], 6);
  expect_as_the_solve(summary, scene, views, 0, options.solve);
  options = {};
  options.solve.max_pnp_rms_px = 0.01;
  views.clear();
  summary = session(6, options, views);
  EXPECT_EQ(summary.cameras[0].usable_views, 6);
  EXPECT_EQ(summary.cameras[0].used_views, 0);
  EXPECT_EQ(summary.cameras[1].ambiguous_views, 0);
  EXPECT_FALSE(summary.ready);
  expect_as_the_solve(summary, scene, views, 0, options.solve);
}

// Links of 10 shared captures or more are strong, from the solve's
// `weak_below` (5) fair, fewer weak; and Calibrate may run once each shares 3.
TEST(SessionChecks, GradesTheTreePreviewAndSaysWhenItIsReady) {
  const Scene scene;
  SessionChecks checks = scene.checks();
  int index = 0;
  const auto add_times = [&](int times,
                             const std::vector<std::size_t>& seeing) {
    for (int i = 0; i < times; ++i) {
      add(checks, index++, scene.capture(scene.everyone, seeing));
    }
  };
  add_times(10, {0, 1});
  add_times(6, {1, 2});
  add_times(2, {0, 3});
  SessionSummary summary = summary_of(checks);
  EXPECT_FALSE(summary.cameras[0].link.has_value());
  const auto expect_link = [&](std::size_t c, std::size_t parent, int shared,
                               LinkStrength strength) {
    const CameraSummary& cam = summary.cameras[c];
    EXPECT_TRUE(cam.linked) << c;
    ASSERT_TRUE(cam.link.has_value()) << c;
    EXPECT_EQ(cam.link->parent, parent) << c;
    EXPECT_EQ(cam.link->shared, shared) << c;
    EXPECT_EQ(cam.link->strength, strength) << c;
  };
  EXPECT_TRUE(summary.cameras[0].linked);
  expect_link(1, 0, 10, LinkStrength::Strong);
  expect_link(2, 1, 6, LinkStrength::Fair);
  expect_link(3, 0, 2, LinkStrength::Weak);
  EXPECT_FALSE(summary.ready);

  add_times(1, {0, 3});
  summary = summary_of(checks);
  expect_link(3, 0, 3, LinkStrength::Weak);
  EXPECT_TRUE(summary.ready);
  ASSERT_TRUE(checks.drop(index - 1).ok());
  EXPECT_FALSE(summary_of(checks).ready);

  // The tree grows from the reference; the grades follow the options.
  const auto seeing = [](int i) {
    return i < 10   ? std::vector<std::size_t>{0, 1}
           : i < 16 ? std::vector<std::size_t>{1, 2}
                    : std::vector<std::size_t>{0, 3};
  };
  const auto session = [&](std::size_t reference,
                           const SessionOptions& options) {
    SessionChecks other = scene.checks(reference, options);
    for (int i = 0; i < index - 1; ++i) {
      add(other, i, scene.capture(scene.everyone, seeing(i)));
    }
    return summary_of(other);
  };
  summary = session(2, {});
  EXPECT_FALSE(summary.cameras[2].link.has_value());
  expect_link(1, 2, 6, LinkStrength::Fair);
  expect_link(0, 1, 10, LinkStrength::Strong);
  expect_link(3, 0, 2, LinkStrength::Weak);
  SessionOptions graded;
  graded.solve.weak_below = 2;
  graded.strong_from = 6;
  summary = session(0, graded);
  expect_link(1, 0, 10, LinkStrength::Strong);
  expect_link(2, 1, 6, LinkStrength::Strong);
  expect_link(3, 0, 2, LinkStrength::Fair);
}

// Calibrate is enabled by the adjustment's own guard, so a session the checks
// call ready is not one bundle_adjust refuses for too few shared captures.
TEST(SessionChecks, IsReadyByTheAdjustmentsOwnGuard) {
  EXPECT_EQ(SessionOptions{}.ready_from,
            solve::BundleAdjustOptions{}.min_shared);
}

// A camera the reference's tree does not reach is unlinked, and Calibrate
// waits for it; each linked camera names its parent in the rig. A rig of one
// camera has nothing to calibrate.
TEST(SessionChecks, AnUnlinkedCameraHoldsCalibrateBack) {
  const Scene scene;
  for (const std::size_t reference : {std::size_t{0}, std::size_t{3}}) {
    SessionChecks checks = scene.checks(reference);
    for (int i = 0; i < 6; ++i) {
      add(checks, i,
          scene.capture(scene.everyone, i % 2 == 0
                                            ? std::vector<std::size_t>{0, 1}
                                            : std::vector<std::size_t>{2, 3}));
    }
    const SessionSummary summary = summary_of(checks);
    const bool zero_side = reference == 0;
    for (std::size_t c = 0; c < 4; ++c) {
      EXPECT_EQ(summary.cameras[c].linked, (c < 2) == zero_side) << c;
      EXPECT_EQ(summary.cameras[c].link.has_value(),
                summary.cameras[c].linked && c != reference)
          << c;
    }
    // The other camera of the reference's pair, placed from it.
    const std::size_t partner = zero_side ? 1 : 2;
    ASSERT_TRUE(summary.cameras[partner].link.has_value());
    EXPECT_EQ(summary.cameras[partner].link->parent, reference);
    EXPECT_EQ(summary.cameras[partner].link->shared, 3);
    EXPECT_FALSE(summary.ready);
  }

  Scene one;
  one.cameras.resize(1);
  SessionChecks single =
      SessionChecks::create(one.cameras, one.board, 0).value();
  for (int i = 0; i < 5; ++i) {
    std::vector<Detection> detections = one.capture(one.everyone, {0});
    detections.resize(1);
    add(single, i, detections);
  }
  const SessionSummary summary = summary_of(single);
  EXPECT_EQ(summary.cameras[0].usable_views, 5);
  EXPECT_TRUE(summary.cameras[0].linked);
  EXPECT_FALSE(summary.ready);
}

// A capture repeats an earlier one when every camera that sees its board saw
// the earlier one's within 2 cm and 2 degrees; dropped captures repeat
// nothing.
TEST(SessionChecks, WarnsOfACaptureThatRepeatsAnEarlierOne) {
  const Scene scene;
  SessionChecks checks = scene.checks();
  const camera::Mat4d p = scene.everyone;
  const camera::Vec3d axis(0.3, -0.5, 0.8);
  // 1 within 1.6 cm and 1.9 degrees of 0; 2 and 3 just beyond, and each
  // turned or moved away from 1.
  add(checks, 0, scene.capture(p, {0, 1}));
  add(checks, 1,
      scene.capture(scene.moved(p, axis, 1.9, {0.011, 0.0, -0.011}), {0, 1}));
  add(checks, 2,
      scene.capture(scene.moved(p, axis, 0.0, {-0.021, 0.0, 0.0}), {0, 1}));
  add(checks, 3, scene.capture(scene.moved(p, axis, -2.1), {0, 1}));
  add(checks, 4, scene.capture(p, {0, 1, 2}));  // camera 2 saw none before
  add(checks, 5, scene.capture(p, {2}));
  add(checks, 6, scene.capture(p, {}));  // seen by none: repeats nothing
  SessionSummary summary = summary_of(checks);
  ASSERT_EQ(summary.duplicates.size(), 2u);
  EXPECT_EQ(summary.duplicates[0].capture, 1);
  EXPECT_EQ(summary.duplicates[0].earlier, 0);
  EXPECT_EQ(summary.duplicates[1].capture, 5);
  EXPECT_EQ(summary.duplicates[1].earlier, 4);

  ASSERT_TRUE(checks.drop(0).ok());
  summary = summary_of(checks);
  ASSERT_EQ(summary.duplicates.size(), 1u);
  EXPECT_EQ(summary.duplicates[0].capture, 5);

  SessionOptions options;
  options.duplicate_m = 0.03;
  options.duplicate_deg = 3.0;
  SessionChecks lenient = scene.checks(0, options);
  for (int i = 1; i <= 3; ++i) {
    add(lenient, i,
        scene.capture(i == 1   ? p
                      : i == 2 ? scene.moved(p, axis, 0.0, {0.021, 0, 0})
                               : scene.moved(p, axis, 2.1),
                      {0, 1}));
  }
  EXPECT_EQ(summary_of(lenient).duplicates.size(), 2u);
}

// Floor-only: 80 % of a camera's usable views' board normals within 10
// degrees of one of them -- whichever gathers the most -- once it has 5
// usable views. Boards spun about their own normal face alike.
TEST(SessionChecks, WarnsWhenACamerasBoardsNearlyAllFaceAlike) {
  const Scene scene;
  // Camera 0's warning over boards whose normals are turned `degs` from one
  // tilted 35 degrees from facing it, all about one axis, each spun.
  const auto floor_only = [&](const std::vector<double>& degs,
                              const SessionOptions& options = {}) {
    SessionChecks checks = scene.checks(0, options);
    for (std::size_t i = 0; i < degs.size(); ++i) {
      const camera::Mat4d board_to_world =
          scene.spun(scene.tilted_to(scene.everyone, 0, 35.0 - degs[i]),
                     8.0 * static_cast<double>(i));
      add(checks, static_cast<int>(i), scene.capture(board_to_world, {0}));
    }
    const SessionSummary summary = summary_of(checks);
    EXPECT_EQ(summary.cameras[0].usable_views, static_cast<int>(degs.size()));
    EXPECT_FALSE(summary.cameras[1].floor_only);  // no views
    return summary.cameras[0].floor_only;
  };
  // 4 of 5 alike: 7 degrees off still faces alike, 13 does not.
  EXPECT_TRUE(floor_only({0.0, 0.0, 0.0, 7.0, 30.0}));
  EXPECT_FALSE(floor_only({0.0, 0.0, 0.0, 13.0, 30.0}));
  // Gathered about the normal that gathers most, not the first one's.
  EXPECT_TRUE(floor_only({30.0, 0.0, 0.0, 0.0, 7.0}));
  // Within 10 degrees of one of them, so two can be 14 apart.
  EXPECT_TRUE(floor_only({-7.0, 0.0, 0.0, 7.0, 30.0}));
  // Not before 5 usable views.
  EXPECT_FALSE(floor_only({0.0, 0.0, 0.0, 0.0}));
  SessionOptions options;
  options.warn_from = 4;
  EXPECT_TRUE(floor_only({0.0, 0.0, 0.0, 0.0}, options));
  options = {};
  options.floor_share = 0.6;
  EXPECT_TRUE(floor_only({0.0, 0.0, 0.0, 13.0, 30.0}, options));  // 3 of 5
  options = {};
  options.floor_deg = 14.0;
  EXPECT_TRUE(floor_only({0.0, 0.0, 0.0, 13.0, 30.0}, options));
}

// The rig's board fixes a pose far worse face-on: a camera most of whose
// usable views are frontal (tilted under 20 degrees) is warned, once it has
// 5 usable views.
TEST(SessionChecks, WarnsWhenMostOfACamerasViewsAreFrontal) {
  const Scene scene;
  const camera::Mat4d p = scene.everyone;
  const auto session = [&](const std::vector<double>& tilts,
                           const SessionOptions& options = {}) {
    SessionChecks checks = scene.checks(0, options);
    for (std::size_t i = 0; i < tilts.size(); ++i) {
      add(checks, static_cast<int>(i),
          scene.capture(scene.tilted_to(p, 0, tilts[i]), {0}));
    }
    const SessionSummary summary = summary_of(checks);
    EXPECT_EQ(summary.cameras[0].usable_views, static_cast<int>(tilts.size()));
    EXPECT_FALSE(summary.cameras[1].mostly_frontal);  // no views
    return summary.cameras[0];
  };
  CameraSummary cam = session({10.0, 12.0, 15.0, 40.0, 45.0});
  EXPECT_EQ(cam.frontal_views, 3);
  EXPECT_TRUE(cam.mostly_frontal);
  cam = session({10.0, 12.0, 15.0, 40.0, 45.0, 50.0});
  EXPECT_EQ(cam.frontal_views, 3);
  EXPECT_FALSE(cam.mostly_frontal);  // half is not most
  // Not before 5 usable views.
  EXPECT_FALSE(session({10.0, 12.0, 15.0, 40.0}).mostly_frontal);
  SessionOptions options;
  options.warn_from = 4;
  EXPECT_TRUE(session({10.0, 12.0, 15.0, 40.0}, options).mostly_frontal);

  options = {};
  options.view.frontal_deg = 46.0;
  cam = session({10.0, 40.0, 45.0, 50.0, 55.0}, options);
  EXPECT_EQ(cam.frontal_views, 3);
  EXPECT_TRUE(cam.mostly_frontal);
  options.frontal_share = 0.9;
  EXPECT_FALSE(session({10.0, 40.0, 45.0, 50.0, 55.0}, options).mostly_frontal);
}

TEST(SessionChecks, DropsACaptureAndRefusesWhatItCannotKeep) {
  const Scene scene;
  SessionChecks checks = scene.checks();
  add(checks, 4, scene.capture(scene.everyone, {0, 1}));
  const auto refused = [&](int index,
                           const std::vector<Detection>& detections) {
    core::Status added;
    EXPECT_NO_THROW(added = checks.add(index, detections));
    EXPECT_EQ(added.domain(), Code::InvalidArgument);
    return added.message();
  };
  const std::vector<Detection> good = scene.capture(scene.everyone, {0, 1});
  EXPECT_EQ(refused(4, good), "SessionChecks::add: capture 4 is added already");
  EXPECT_EQ(refused(-1, good),
            "SessionChecks::add: capture index -1 is negative");
  EXPECT_EQ(refused(5, std::vector<Detection>(good.begin(), good.end() - 1)),
            "SessionChecks::add: 3 detections for 4 cameras");
  std::vector<Detection> smaller = good;
  smaller[2].image_size = {1920, 1080};
  EXPECT_EQ(refused(5, smaller),
            "SessionChecks::add: camera SYN2: check_view: the detection is of "
            "a 1920x1080 image, the camera's is 3840x2160");
  EXPECT_EQ(summary_of(checks).kept, 1);

  core::Status dropped = checks.drop(3);
  EXPECT_EQ(dropped.domain(), Code::NotFound);
  EXPECT_EQ(dropped.message(), "SessionChecks::drop: no capture 3");
  ASSERT_TRUE(checks.drop(4).ok());
  dropped = checks.drop(4);
  EXPECT_EQ(dropped.domain(), Code::InvalidArgument);
  EXPECT_EQ(dropped.message(),
            "SessionChecks::drop: capture 4 is dropped already");
  const SessionSummary summary = summary_of(checks);
  EXPECT_EQ(summary.kept, 0);
  EXPECT_EQ(summary.dropped, 1);
  EXPECT_EQ(summary.cameras[0].usable_views, 0);
  EXPECT_EQ(summary.covisibility[0][1], 0);
  EXPECT_EQ(refused(4, good), "SessionChecks::add: capture 4 is added already");
}

TEST(SessionChecks, RefusesARigOrOptionsItCannotCheck) {
  const Scene scene;
  const auto refused = [&](const std::vector<solve::RigCamera>& cameras,
                           std::size_t reference,
                           const SessionOptions& options) {
    core::Result<SessionChecks> checks = core::Status::invalid_argument("");
    EXPECT_NO_THROW(checks = SessionChecks::create(cameras, scene.board,
                                                   reference, options));
    EXPECT_EQ(checks.status().domain(), Code::InvalidArgument);
    return checks.status().message();
  };
  EXPECT_EQ(refused({}, 0, {}), "SessionChecks: the rig has no cameras");
  EXPECT_EQ(refused(scene.cameras, 4, {}),
            "SessionChecks: reference 4 is not one of the rig's 4 cameras");
  std::vector<solve::RigCamera> cameras = scene.cameras;
  cameras[2].id = "";
  EXPECT_EQ(refused(cameras, 0, {}), "SessionChecks: camera 2 has no id");
  cameras[2].id = cameras[1].id;
  EXPECT_EQ(refused(cameras, 0, {}),
            "SessionChecks: camera id \"SYN1\" appears twice");
  cameras = scene.cameras;
  cameras[3].model.intrinsics.fx = 0.0;
  EXPECT_EQ(refused(cameras, 0, {})
                .rfind("SessionChecks: camera SYN3: cv_camera: ", 0),
            0u);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto option_refused = [&](const SessionOptions& options) {
    return refused(scene.cameras, 0, options);
  };
  SessionOptions options;
  options.view.far_m = 0.0;
  EXPECT_EQ(option_refused(options),
            "SessionChecks: options.view.far_m must be positive and finite");
  for (const auto& [field, value, rule] :
       {std::tuple{&SessionOptions::duplicate_m, 0.0, "positive and finite"},
        std::tuple{&SessionOptions::duplicate_m, nan, "positive and finite"},
        std::tuple{&SessionOptions::duplicate_deg, 0.0, "in (0, 180]"},
        std::tuple{&SessionOptions::floor_share, 0.0, "in (0, 1]"},
        std::tuple{&SessionOptions::floor_share, 1.1, "in (0, 1]"},
        std::tuple{&SessionOptions::floor_deg, -1.0, "in [0, 180]"},
        std::tuple{&SessionOptions::frontal_share, 1.0, "in [0, 1)"},
        std::tuple{&SessionOptions::frontal_share, nan, "in [0, 1)"}}) {
    options = {};
    options.*field = value;
    const std::string message = option_refused(options);
    EXPECT_NE(message.find(rule), std::string::npos) << message;
  }
  options = {};
  options.solve.min_ambiguity = 0.5;
  EXPECT_EQ(option_refused(options),
            "SessionChecks: options.solve.min_ambiguity must be at least 1 and "
            "finite");
  options = {};
  options.warn_from = 0;
  EXPECT_EQ(option_refused(options),
            "SessionChecks: options.warn_from must be at least 1");
  options = {};
  options.strong_from = 4;
  EXPECT_EQ(
      option_refused(options),
      "SessionChecks: options.strong_from must be at least solve.weak_below");
  options = {};
  options.ready_from = 0;
  EXPECT_EQ(option_refused(options),
            "SessionChecks: options.ready_from must be at least 1");
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
