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
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"

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

  // Each camera's exact detection of the board at `board_to_world`: those in
  // `seeing` find all of it, the others nothing.
  std::vector<Detection> capture(const camera::Mat4d& board_to_world,
                                 const std::vector<std::size_t>& seeing) const {
    test::ObserveOptions exact;
    exact.noise_px = 0.0;
    const std::vector<Observation> views = test::observe(
        rig, board, {test::Placement{board_to_world, seeing}}, exact);
    std::vector<Detection> detections(rig.size());
    for (std::size_t c = 0; c < rig.size(); ++c) {
      detections[c].image_size = rig[c].model.size;
    }
    for (std::size_t i = 0; i < seeing.size(); ++i) {
      detections[seeing[i]].observation = views[i];
    }
    return detections;
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
    const cv::Point3d m = board.centre();
    const camera::Vec3d c(board_to_world * glm::dvec4(m.x, m.y, m.z, 1.0));
    const camera::Mat3d turn = camera::rotation_from_rodrigues(
        glm::normalize(axis) * (deg * kPi / 180.0));
    camera::Mat4d out(turn * camera::Mat3d(board_to_world));
    out[3] = glm::dvec4(
        turn * (camera::Vec3d(board_to_world[3]) - c) + c + shift, 1.0);
    return out;
  }

  // `board_to_world` turned about its centre so its printed face is
  // `tilt_deg` from facing camera `c`.
  camera::Mat4d tilted_to(const camera::Mat4d& board_to_world, std::size_t c,
                          double tilt_deg) const {
    const cv::Point3d m = board.centre();
    const camera::Vec3d centre(board_to_world * glm::dvec4(m.x, m.y, m.z, 1.0));
    const camera::Vec3d to_camera =
        camera::Vec3d(camera::rigid_inverse(rig[c].world_to_camera)[3]) -
        centre;
    const camera::Vec3d face = -camera::Vec3d(board_to_world[2]);
    return moved(board_to_world, glm::cross(face, to_camera),
                 compute::angle_deg(face, to_camera) - tilt_deg);
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

// Over a plan of placements: each camera's usable views and the captures each
// pair shares are what the solve counts, and each usable corner lands in its
// cell of the coverage grid.
TEST(SessionChecks, CountsViewsSharesAndCoverageAsTheSolveDoes) {
  const Scene scene;
  test::PlanOptions options;
  options.count = 30;
  const std::vector<test::Placement> plan =
      test::synthetic_plan(scene.rig, scene.board, options);
  SessionChecks checks = scene.checks();
  std::vector<std::vector<std::size_t>> seen_by;
  std::vector<std::array<int, kCoverageCols * kCoverageRows>> cells(4);
  for (std::size_t i = 0; i < plan.size(); ++i) {
    const std::vector<Detection> detections =
        scene.capture(plan[i].board_to_world, plan[i].cameras);
    add(checks, static_cast<int>(i), detections);
    seen_by.push_back(plan[i].cameras);
    for (const std::size_t c : plan[i].cameras) {
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
  const solve::Covisibility expected = solve::covisibility(4, seen_by).value();
  EXPECT_EQ(summary.covisibility, expected);
  ASSERT_EQ(summary.cameras.size(), 4u);
  for (std::size_t c = 0; c < 4; ++c) {
    const CameraSummary& cam = summary.cameras[c];
    EXPECT_EQ(cam.usable_views, expected[c][c]) << c;
    EXPECT_EQ(cam.coverage, cells[c]) << c;
    int filled = 0;
    for (const int n : cells[c]) filled += n > 0 ? 1 : 0;
    EXPECT_EQ(cam.coverage_filled, filled / 48.0) << c;
    EXPECT_GT(cam.coverage_filled, 0.0) << c;
  }
}

// Links of 10 shared captures or more are strong, 5-9 fair, fewer weak; and
// Calibrate may run once each shares 3.
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

  // The tree grows from the reference.
  const SessionChecks from_two = [&] {
    SessionChecks other = scene.checks(2);
    for (int i = 0; i < index - 1; ++i) {
      add(other, i,
          scene.capture(scene.everyone, i < 10 ? std::vector<std::size_t>{0, 1}
                                        : i < 16
                                            ? std::vector<std::size_t>{1, 2}
                                            : std::vector<std::size_t>{0, 3}));
    }
    return other;
  }();
  summary = summary_of(from_two);
  EXPECT_FALSE(summary.cameras[2].link.has_value());
  expect_link(1, 2, 6, LinkStrength::Fair);
  expect_link(0, 1, 10, LinkStrength::Strong);
  expect_link(3, 0, 2, LinkStrength::Weak);
}

// A camera the reference's tree does not reach is unlinked, and Calibrate
// waits for it; a rig of one camera has nothing to calibrate.
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

// Floor-only: 80 % of a camera's boards' normals within 10 degrees of one of
// them. Boards turned about their own normal all face alike.
TEST(SessionChecks, WarnsWhenACamerasBoardsNearlyAllFaceAlike) {
  const Scene scene;
  SessionChecks checks = scene.checks();
  const camera::Mat4d p = scene.everyone;
  const camera::Vec3d normal(p[2]);
  int index = 0;
  for (const double turn : {0.0, 8.0, -8.0, 16.0}) {
    add(checks, index++, scene.capture(scene.moved(p, normal, turn), {0, 1}));
  }
  // Within 10 degrees still faces alike; beyond it, not.
  add(checks, index++, scene.capture(scene.tilted_to(p, 0, 0.0), {0}));
  SessionSummary summary = summary_of(checks);
  ASSERT_EQ(summary.cameras[0].usable_views, 5);
  EXPECT_TRUE(summary.cameras[0].floor_only);   // 4 of 5
  EXPECT_TRUE(summary.cameras[1].floor_only);   // 4 of 4
  EXPECT_FALSE(summary.cameras[2].floor_only);  // no views
  add(checks, index++,
      scene.capture(scene.moved(scene.tilted_to(p, 0, 0.0), normal, 20.0),
                    {0}));
  summary = summary_of(checks);
  ASSERT_EQ(summary.cameras[0].usable_views, 6);
  EXPECT_FALSE(summary.cameras[0].floor_only);  // 4 of 6
  SessionOptions options;
  options.floor_share = 0.6;
  SessionChecks lenient = scene.checks(0, options);
  for (int i = 0; i < 4; ++i) {
    add(lenient, i, scene.capture(scene.moved(p, normal, 8.0 * i), {0}));
  }
  add(lenient, 4, scene.capture(scene.tilted_to(p, 0, 0.0), {0}));
  add(lenient, 5, scene.capture(scene.tilted_to(p, 0, 5.0), {0}));
  EXPECT_TRUE(summary_of(lenient).cameras[0].floor_only);  // 4 of 6
}

// The rig's board fixes a pose far worse face-on: a camera most of whose
// views are frontal (tilted under 20 degrees) is warned.
TEST(SessionChecks, WarnsWhenMostOfACamerasViewsAreFrontal) {
  const Scene scene;
  SessionChecks checks = scene.checks();
  const camera::Mat4d p = scene.everyone;
  int index = 0;
  for (const double tilt : {10.0, 12.0, 15.0, 40.0, 45.0}) {
    add(checks, index++, scene.capture(scene.tilted_to(p, 0, tilt), {0}));
  }
  SessionSummary summary = summary_of(checks);
  ASSERT_EQ(summary.cameras[0].usable_views, 5);
  EXPECT_EQ(summary.cameras[0].frontal_views, 3);
  EXPECT_TRUE(summary.cameras[0].mostly_frontal);
  add(checks, index++, scene.capture(scene.tilted_to(p, 0, 50.0), {0}));
  summary = summary_of(checks);
  EXPECT_EQ(summary.cameras[0].frontal_views, 3);
  EXPECT_FALSE(summary.cameras[0].mostly_frontal);  // half is not most
  EXPECT_FALSE(summary.cameras[1].mostly_frontal);  // no views

  SessionOptions options;
  options.view.frontal_deg = 46.0;
  options.frontal_share = 0.9;
  SessionChecks other = scene.checks(0, options);
  for (const double tilt : {10.0, 40.0, 45.0, 50.0}) {
    add(other, index++, scene.capture(scene.tilted_to(p, 0, tilt), {0}));
  }
  summary = summary_of(other);
  EXPECT_EQ(summary.cameras[0].frontal_views, 3);
  EXPECT_FALSE(summary.cameras[0].mostly_frontal);
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
  EXPECT_EQ(refused({}, 0, {}), "SessionChecks: no cameras");
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
  EXPECT_EQ(refused(cameras, 0, {}).rfind("SessionChecks: camera SYN3: ", 0),
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
  options.fair_from = 0;
  EXPECT_EQ(option_refused(options),
            "SessionChecks: options.fair_from must be at least 1");
  options = {};
  options.strong_from = 4;
  EXPECT_EQ(option_refused(options),
            "SessionChecks: options.strong_from must be at least fair_from");
  options = {};
  options.ready_from = 0;
  EXPECT_EQ(option_refused(options),
            "SessionChecks: options.ready_from must be at least 1");
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
