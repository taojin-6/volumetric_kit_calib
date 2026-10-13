// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The solve runs out of memory anywhere -- building a message, adding
// context, inside OpenCV -- and returns OutOfMemory, never throwing, and
// never a rig short of a view. The one exception is OpenCV's own:
// solvePnPGeneric swallows whatever IPPE throws, leaving the view with no
// pose or half of one, and view_pose reports either as a Backend error naming
// solvePnPGeneric.

#include <cstddef>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "allocation_failure.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/calib/solve/spanning_tree.hpp"
#include "volumetric_kit/calib/solve/view_pose.hpp"

namespace volumetric_kit::calib::solve {
namespace {

using Code = core::Status::Code;

// Fail the n-th allocation of `fn`, for n = 0, 1, ... until a run allocates
// fewer: each run that failed one returns OutOfMemory, or what `swallowed`
// accepts as a failure OpenCV swallowed; the first that did not returns what
// an unhindered run does.
template <class Fn, class Swallowed>
void expect_oom_anywhere(const Fn& fn, const Swallowed& swallowed) {
  const auto original = fn();
  std::size_t oom = 0;
  for (std::size_t after = 0; after < 100000; ++after) {
    std::optional<std::invoke_result_t<const Fn&>> result;
    EXPECT_NO_THROW({
      const test::FailAllocation failure(after);
      result.emplace(fn());
    });
    ASSERT_TRUE(result.has_value());
    if (!test::allocation_failure.injected) {
      EXPECT_GT(oom, 0u);
      EXPECT_EQ(result->status().domain(), original.status().domain());
      EXPECT_EQ(result->status().message(), original.status().message());
      return;
    }
    if (result->status().domain() == Code::OutOfMemory) {
      ++oom;
    } else {
      ASSERT_TRUE(swallowed(*result))
          << "allocation " << after << ": " << result->status().message();
    }
  }
  FAIL() << "did not finish without an injected allocation failure";
}

template <class Fn>
void expect_oom_anywhere(const Fn& fn) {
  expect_oom_anywhere(fn, [](const auto&) { return false; });
}

struct Small {
  compute::Board board = compute::Board::create(legacy_femto_board()).value();
  std::vector<test::SyntheticCamera> rig = test::synthetic_rig();
  std::vector<Observation> views;
  std::vector<RigCamera> cameras;

  Small() {
    test::PlanOptions options;
    options.count = 4;
    options.min_cameras = 4;
    views =
        test::observe(rig, board, test::synthetic_plan(rig, board, options));
    for (const test::SyntheticCamera& cam : rig) {
      cameras.push_back({cam.id, cam.model});
    }
  }
};

// What view_pose reports when solvePnPGeneric swallowed IPPE's failure.
bool ippe_failed(const core::Status& status) {
  return status.domain() == Code::Backend &&
         status.message().find("solvePnPGeneric") != std::string::npos;
}

TEST(SolveOom, Initialise) {
  const Small scene;
  ASSERT_EQ(scene.views.size(), 16u);
  expect_oom_anywhere(
      [&] { return initialise(scene.cameras, scene.board, scene.views, 0); },
      [](const core::Result<InitialRig>& rig) {
        return !rig.ok() && ippe_failed(rig.status());
      });
}

TEST(SolveOom, InitialiseRefusingAView) {
  Small scene;
  scene.views[5].camera_id = "nobody";
  const std::string refusal =
      initialise(scene.cameras, scene.board, scene.views, 0).status().message();
  expect_oom_anywhere(
      [&] { return initialise(scene.cameras, scene.board, scene.views, 0); },
      [&](const core::Result<InitialRig>& rig) {
        return rig.status().message() == refusal || ippe_failed(rig.status());
      });
}

TEST(SolveOom, ViewPose) {
  const Small scene;
  const compute::CvCamera cam =
      compute::cv_camera(scene.cameras[0].model).value();
  expect_oom_anywhere(
      [&] { return view_pose(cam, scene.board, scene.views[0]); },
      [](const core::Result<ViewPose>& pose) {
        return ippe_failed(pose.status());
      });
  Observation few = scene.views[0];
  few.points.resize(3);
  expect_oom_anywhere([&] { return view_pose(cam, scene.board, few); });
}

// The inputs are built before allocation can fail: only the call is tested.
TEST(SolveOom, TreeAndCovisibility) {
  const std::vector<std::string> ids = {"A", "B", "C"};
  const Covisibility apart = {{2, 2, 0}, {2, 2, 0}, {0, 0, 1}};
  const Covisibility linked = {{2, 2, 1}, {2, 2, 0}, {1, 0, 1}};
  expect_oom_anywhere([&] { return spanning_tree(apart, ids, 0, 5); });
  expect_oom_anywhere([&] { return spanning_tree(linked, ids, 1, 5); });
  const std::vector<std::vector<std::size_t>> twice = {{0, 1}, {1, 2, 2}};
  const std::vector<std::vector<std::size_t>> fine = {{0, 1}, {1, 2}};
  expect_oom_anywhere([&] { return covisibility(3, twice); });
  expect_oom_anywhere([&] { return covisibility(3, fine); });
}

}  // namespace
}  // namespace volumetric_kit::calib::solve
