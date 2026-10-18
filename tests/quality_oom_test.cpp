// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The quality tier runs out of memory anywhere and returns OutOfMemory,
// never throwing; the capture rule and the session's checks are then as they
// were. The one exception is OpenCV's own: solvePnPGeneric swallows what IPPE
// throws, which view_pose reports as a Backend error naming it; in the
// transfer, that fails the metrics.

#include <chrono>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "allocation_failure.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/quality/calibration_metrics.hpp"
#include "volumetric_kit/calib/quality/capture_rule.hpp"
#include "volumetric_kit/calib/quality/depth_checks.hpp"
#include "volumetric_kit/calib/quality/live_checks.hpp"
#include "volumetric_kit/calib/quality/results.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;
using Code = core::Status::Code;
using test::expect_oom_anywhere;

const core::Status& status_of(const core::Status& status) { return status; }
template <class T>
const core::Status& status_of(const core::Result<T>& result) {
  return result.status();
}

// `fn` on a copy of `base`, made before allocation fails, swept: each run that
// fails one returns OutOfMemory, and the run failing none returns what an
// unhindered run does.
template <class T, class Fn>
void expect_oom_on_a_copy(const T& base, const Fn& fn) {
  T fresh = base;
  const auto original = fn(fresh);
  std::size_t oom = 0;
  test::sweep_allocations([&](std::size_t after) {
    T copy = base;
    std::optional<decltype(fn(copy))> result;
    EXPECT_NO_THROW({
      const test::FailAllocation failure(after);
      result.emplace(fn(copy));
    });
    ASSERT_TRUE(result.has_value());
    if (!test::allocation_failure.injected) {
      EXPECT_EQ(status_of(*result).domain(), status_of(original).domain());
      EXPECT_EQ(status_of(*result).message(), status_of(original).message());
      return;
    }
    ASSERT_EQ(status_of(*result).domain(), Code::OutOfMemory) << after;
    ++oom;
  });
  EXPECT_GT(oom, 0u);
}

bool ippe_failed(const core::Status& status) {
  return status.domain() == Code::Backend &&
         status.message().find("solvePnPGeneric") != std::string::npos;
}

struct Scene {
  compute::Board board = test::femto_board();
  std::vector<test::SyntheticCamera> rig = test::synthetic_rig();
  std::vector<solve::RigCamera> cameras;
  std::vector<std::vector<Detection>> captures;

  Scene() {
    for (const test::SyntheticCamera& cam : rig) {
      cameras.push_back({cam.id, cam.model});
    }
    test::PlanOptions options;
    options.count = 3;
    options.min_cameras = 4;
    const std::vector<test::Placement> plan =
        test::synthetic_plan(rig, board, options);
    const std::vector<Observation> views = test::observe(rig, board, plan);
    for (std::size_t p = 0; p < plan.size(); ++p) {
      std::vector<Detection>& capture = captures.emplace_back(rig.size());
      for (std::size_t c = 0; c < rig.size(); ++c) {
        capture[c].image_size = rig[c].model.size;
        capture[c].observation = views[p * rig.size() + c];
      }
    }
  }
};

TEST(QualityOom, LiveChecks) {
  const Scene scene;
  const compute::CvCamera cam =
      compute::cv_camera(scene.cameras[0].model).value();
  const Detection& view = scene.captures[0][0];
  expect_oom_anywhere([&] { return check_view(cam, scene.board, view); },
                      [](const core::Result<CheckedView>& r) {
                        return ippe_failed(r.status());
                      });
  Detection smaller = view;
  smaller.image_size = {64, 64};
  expect_oom_anywhere([&] { return check_view(cam, scene.board, smaller); });

  const std::vector<Detection>& a = scene.captures[0];
  const std::vector<Detection>& b = scene.captures[1];
  const std::vector<Detection> one = {b[0]};
  expect_oom_anywhere([&] { return corner_shift(a, b); });
  expect_oom_anywhere([&] { return corner_shift(a, one); });
}

// Each path of the rule -- waiting, taking with warnings, failing -- out of
// memory leaves the rule as it was: run again, it answers as it would have.
TEST(QualityOom, CaptureRuleIsUnchangedByAFailure) {
  using std::chrono::milliseconds;
  const CaptureRule::Clock::time_point press{};
  CaptureRule armed =
      CaptureRule::create({"CL2A141000N", "CL2A14100A4", "CL2A14100A7"})
          .value();
  armed.arm(press);
  LiveSet incomplete;
  incomplete.present = {true, false, false};
  incomplete.checks.resize(3);
  incomplete.motion_px = 0.1;
  LiveSet warned = incomplete;
  warned.present.assign(3, true);
  warned.spread_ms = 12.5;
  // Fixed formatting must grow its buffer on both libc++ and libstdc++.
  LiveSet large_spread = warned;
  large_spread.spread_ms = 1.8e30;
  LiveSet large_motion = warned;
  large_motion.motion_px = 1.8e30;
  // Waiting on cameras whose names outgrow a short string's buffer.
  CaptureRule waiting = armed;
  ASSERT_TRUE(waiting.update(incomplete, press).ok());

  struct Case {
    const CaptureRule* rule;
    const LiveSet* set;
    CaptureRule::Clock::time_point now;
  };
  for (const Case& c :
       {Case{&armed, &incomplete, press}, Case{&armed, &warned, press},
        Case{&waiting, &large_spread, press},
        Case{&waiting, &large_motion, press},
        Case{&waiting, &warned, press + milliseconds(3000)}}) {
    CaptureRule fresh = *c.rule;
    const CaptureDecision expected = fresh.update(*c.set, c.now).value();
    SCOPED_TRACE(expected.message);
    std::size_t oom = 0;
    test::sweep_allocations([&](std::size_t after) {
      SCOPED_TRACE(after);
      CaptureRule rule = *c.rule;
      std::optional<core::Result<CaptureDecision>> result;
      EXPECT_NO_THROW({
        const test::FailAllocation failure(after);
        result.emplace(rule.update(*c.set, c.now));
      });
      ASSERT_TRUE(result.has_value());
      if (!test::allocation_failure.injected) {
        ASSERT_TRUE(result->ok());
        EXPECT_EQ(result->value().state, expected.state);
        EXPECT_EQ(result->value().hold, expected.hold);
        EXPECT_EQ(result->value().message, expected.message);
        EXPECT_EQ(rule.armed(), fresh.armed());
        return;
      }
      ASSERT_EQ(result->status().domain(), Code::OutOfMemory)
          << result->status().message();
      ++oom;
      EXPECT_TRUE(rule.armed());
      // Its last hold as it was, and its answer to the set as it would have.
      CaptureRule untouched = *c.rule;
      const CaptureDecision held = rule.tick(press).value();
      const CaptureDecision was = untouched.tick(press).value();
      EXPECT_EQ(held.hold, was.hold);
      EXPECT_EQ(held.message, was.message);
      const CaptureDecision again = rule.update(*c.set, c.now).value();
      EXPECT_EQ(again.state, expected.state);
      EXPECT_EQ(again.hold, expected.hold);
      EXPECT_EQ(again.message, expected.message);
      EXPECT_EQ(rule.armed(), fresh.armed());
    });
    EXPECT_GT(oom, 0u);
  }
  expect_oom_on_a_copy(waiting, [&](CaptureRule& rule) {
    return rule.tick(press + milliseconds(2000));
  });
  expect_oom_on_a_copy(waiting, [&](CaptureRule& rule) {
    return rule.tick(press + milliseconds(3000));
  });
  const std::vector<std::string> fine = {"A", "B"};
  const std::vector<std::string> twice = {"A", "A"};
  expect_oom_anywhere([&] { return CaptureRule::create(fine); });
  expect_oom_anywhere([&] { return CaptureRule::create(twice); });
}

// The session's checks: made, added to, summed up and dropped from, out of
// memory, they are as they were.
TEST(QualityOom, SessionChecksAreUnchangedByAFailure) {
  const Scene scene;
  expect_oom_anywhere(
      [&] { return SessionChecks::create(scene.cameras, scene.board, 0); });
  expect_oom_anywhere(
      [&] { return SessionChecks::create(scene.cameras, scene.board, 9); });

  SessionChecks base =
      SessionChecks::create(scene.cameras, scene.board, 0).value();
  ASSERT_TRUE(base.add(0, scene.captures[0]).ok());
  ASSERT_TRUE(base.add(1, scene.captures[1]).ok());
  std::size_t oom = 0;
  test::sweep_allocations([&](std::size_t after) {
    SessionChecks checks = base;
    core::Status added;
    EXPECT_NO_THROW({
      const test::FailAllocation failure(after);
      added = checks.add(2, scene.captures[2]);
    });
    if (!test::allocation_failure.injected) {
      EXPECT_TRUE(added.ok()) << added.message();
      return;
    }
    if (added.domain() == Code::OutOfMemory) {
      ++oom;
    } else {
      ASSERT_TRUE(ippe_failed(added)) << added.message();
    }
    EXPECT_EQ(checks.summary().value().kept, 2) << after;
    EXPECT_TRUE(checks.add(2, scene.captures[2]).ok()) << after;
  });
  EXPECT_GT(oom, 0u);
  expect_oom_anywhere([&] { return base.summary(); });
  expect_oom_on_a_copy(base,
                       [](SessionChecks& checks) { return checks.drop(7); });
}

// A solved rig of the synthetic rig's first `count` cameras and 3 captures,
// every camera seeing each, on exact data, so each re-solve takes few
// iterations; each link needs 2 shared captures, so each capture can be left
// out.
struct Solved {
  compute::Board board = test::femto_board();
  std::vector<test::SyntheticCamera> rig = test::synthetic_rig();
  std::vector<solve::RigCamera> cameras;
  std::vector<Observation> views;
  solve::InitialRig start;
  MetricsOptions options;
  solve::RigSolution solution;
  CalibrationMetrics metrics;

  explicit Solved(std::size_t count = 4) {
    rig.resize(count);
    for (const test::SyntheticCamera& cam : rig) {
      cameras.push_back({cam.id, cam.model});
    }
    test::PlanOptions plan;
    plan.count = 3;
    plan.min_cameras = count;
    views = test::observe(rig, board, test::synthetic_plan(rig, board, plan),
                          {0.0, 1});
    options.bundle_adjust.min_shared = 2;
    start = solve::initialise(cameras, board, views, 0).value();
    solution = solve::bundle_adjust(cameras, board, views, start,
                                    options.bundle_adjust)
                   .value();
    metrics =
        calibration_metrics(cameras, board, views, start, solution, options)
            .value();
  }
};

TEST(QualityOom, CrossCameraTransfer) {
  const Solved s;
  expect_oom_anywhere(
      [&] {
        return cross_camera_transfer(s.cameras, s.board, s.views,
                                     s.solution.world_to_camera);
      },
      [](const core::Result<Transfer>& r) { return ippe_failed(r.status()); });
}

// Out of memory anywhere, a re-solve's included, fails the whole, as IPPE's
// failure in the transfer does. On two cameras it makes every call stack it
// makes on four, with about half the allocations.
TEST(QualityOom, CalibrationMetrics) {
  const Solved s(2);
  ASSERT_EQ(s.metrics.leave_one_out.captures.size(), 3u);
  for (const LeftOut& left : s.metrics.leave_one_out.captures) {
    ASSERT_TRUE(left.refused.empty()) << left.refused;
  }
  expect_oom_anywhere(
      [&] {
        return calibration_metrics(s.cameras, s.board, s.views, s.start,
                                   s.solution, s.options);
      },
      [](const core::Result<CalibrationMetrics>& r) {
        return ippe_failed(r.status());
      });
}

// One view's depth check of the solved rig's first capture, on its board's
// own plane, and a summary of it with a pair in that capture.
struct DepthChecked {
  const Solved& s;
  BoardRegion region;
  DepthPlane plane;
  BoardDepthCheck check;
  DepthSummary summary;
  std::vector<PairAgreement> pairs;

  explicit DepthChecked(const Solved& solved) : s(solved) {
    const camera::Mat4d board_to_color =
        camera::matrix_from_rodrigues(s.solution.world_to_camera[1]) *
        camera::matrix_from_rodrigues(s.solution.boards[0].board_to_world);
    camera::Mat4d depth_to_color(1.0);
    depth_to_color[3] = glm::dvec4(0.032, 0.0, 0.004, 1.0);
    region = board_region(s.board, board_to_color, depth_to_color).value();
    plane.valid = true;
    plane.point = camera::Vec3d(0.3, 0.2, 0.001);
    plane.normal = camera::Vec3d(0.0, 0.01, 1.0);
    plane.members = 100;
    plane.inliers = 90;
    check =
        board_depth_check(s.cameras, s.board, view(), region, plane).value();
    PairAgreement pair;
    pair.from = 1;
    pair.to = 0;
    pair.capture = check.capture;
    pair.overlap = 2000;
    pair.in_window = 1990;
    pairs = {pair};
    summary = summarize_depth_checks(s.cameras.size(), {check}, pairs).value();
  }

  // Camera 1's view of the first capture adjusted.
  const Observation& view() const {
    for (const Observation& v : s.views) {
      if (v.camera_id == s.cameras[1].id &&
          v.frame_id == s.solution.boards[0].capture) {
        return v;
      }
    }
    throw std::runtime_error("no view of the first capture by camera 1");
  }
};

TEST(QualityOom, DepthChecks) {
  const Solved s;
  const DepthChecked d(s);
  ASSERT_FALSE(d.check.ratios.empty());
  camera::Mat4d skewed(1.0);
  skewed[0][1] = 0.5;
  expect_oom_anywhere(
      [&] { return board_region(s.board, skewed, camera::Mat4d(1.0)); });
  expect_oom_anywhere([&] {
    return board_depth_check(s.cameras, s.board, d.view(), d.region, d.plane);
  });
  Observation stranger = d.view();
  stranger.camera_id = "nobody";
  expect_oom_anywhere([&] {
    return board_depth_check(s.cameras, s.board, stranger, d.region, d.plane);
  });
  // The arguments are made before allocation fails.
  const std::vector<BoardDepthCheck> one = {d.check};
  const std::vector<BoardDepthCheck> twice = {d.check, d.check};
  const std::vector<PairAgreement> none;
  expect_oom_anywhere(
      [&] { return summarize_depth_checks(s.cameras.size(), one, d.pairs); });
  expect_oom_anywhere(
      [&] { return summarize_depth_checks(s.cameras.size(), twice, none); });
}

TEST(QualityOom, FormatResults) {
  const Solved s;
  expect_oom_anywhere(
      [&] { return format_results(s.cameras, s.solution, s.metrics); });
  std::vector<solve::RigCamera> twice = s.cameras;
  twice[1].id = twice[0].id;
  expect_oom_anywhere(
      [&] { return format_results(twice, s.solution, s.metrics); });
  const DepthChecked d(s);
  expect_oom_anywhere([&] {
    return format_results(s.cameras, s.solution, s.metrics, &d.summary);
  });
  DepthSummary misfit = d.summary;
  misfit.cameras.pop_back();
  expect_oom_anywhere([&] {
    return format_results(s.cameras, s.solution, s.metrics, &misfit);
  });
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
