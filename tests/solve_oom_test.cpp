// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The solve runs out of memory anywhere -- building a message, adding
// context, inside OpenCV and cv::LevMarq -- and returns OutOfMemory, never
// throwing, and never a rig short of a view. The one exception is OpenCV's own:
// solvePnPGeneric swallows whatever IPPE throws, leaving the view with no
// pose or half of one, and view_pose reports either as a Backend error naming
// solvePnPGeneric.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "allocation_failure.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/calib/solve/spanning_tree.hpp"
#include "volumetric_kit/calib/solve/view_pose.hpp"
#include "volumetric_kit/calib/solve/world_tag.hpp"

namespace volumetric_kit::calib::solve {
namespace {

using Code = core::Status::Code;

using test::expect_oom_anywhere;

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

TEST(SolveOom, BundleAdjust) {
  const Small scene;
  const InitialRig rig =
      initialise(scene.cameras, scene.board, scene.views, 0).value();
  expect_oom_anywhere([&] {
    return bundle_adjust(scene.cameras, scene.board, scene.views, rig);
  });
  // Refusing an option, and a start that does not fit.
  BundleAdjustOptions refused;
  refused.huber_px = 0.0;
  expect_oom_anywhere([&] {
    return bundle_adjust(scene.cameras, scene.board, scene.views, rig, refused);
  });
  InitialRig short_of = rig;
  short_of.views.pop_back();
  expect_oom_anywhere([&] {
    return bundle_adjust(scene.cameras, scene.board, scene.views, short_of);
  });
}

// The small scene's solution and what its cameras see of the origin tag:
// every camera in 2 captures, the second's views of the tag moved, so the
// location drops them in an outlier round and solves again.
struct Tagged {
  Small scene;
  RigSolution solution;
  std::vector<TagView> views;

  Tagged() {
    const InitialRig rig =
        initialise(scene.cameras, scene.board, scene.views, 0).value();
    solution =
        bundle_adjust(scene.cameras, scene.board, scene.views, rig).value();
    const auto add = [&](const test::FloorTag& tag, std::int64_t capture) {
      const test::ObserveOptions noise{0.3,
                                       static_cast<std::uint32_t>(capture + 1)};
      for (const test::MarkerSighting& s :
           test::observe_marker(scene.rig, kOriginTag.id,
                                test::floor_tag_corners(tag), 1, noise)) {
        views.push_back({scene.rig[s.camera].id, capture,
                         scene.rig[s.camera].model.size, s.marker});
      }
    };
    add({}, 0);
    add({}, 1);
    add({0.15, {0.05, 0.03, 0.0}, 4.0}, 2);
  }
};

TEST(SolveOom, OriginTagFrame) {
  const Tagged tagged;
  ASSERT_EQ(tagged.views.size(), 12u);
  const core::Result<WorldOrigin> origin = origin_tag_frame(
      tagged.scene.cameras, tagged.solution, tagged.views, kOriginTag);
  ASSERT_TRUE(origin.ok() && origin->tag && origin->tag->solves == 2);
  expect_oom_anywhere([&] {
    return origin_tag_frame(tagged.scene.cameras, tagged.solution, tagged.views,
                            kOriginTag);
  });
  // A miss, and a refusal.
  const std::vector<TagView> one(tagged.views.begin(),
                                 tagged.views.begin() + 1);
  expect_oom_anywhere([&] {
    return origin_tag_frame(tagged.scene.cameras, tagged.solution, one,
                            kOriginTag);
  });
  std::vector<TagView> other = tagged.views;
  other[5].marker.id = 3;
  expect_oom_anywhere([&] {
    return origin_tag_frame(tagged.scene.cameras, tagged.solution, other,
                            kOriginTag);
  });
  // With no outlier round the moved views stay, and the world is refused for
  // its fit.
  TagFrameOptions keep;
  keep.outlier_rounds = 0;
  const core::Result<WorldOrigin> refused = origin_tag_frame(
      tagged.scene.cameras, tagged.solution, tagged.views, kOriginTag, keep);
  ASSERT_TRUE(refused.ok() && refused->miss == TagMiss::PoorFit);
  expect_oom_anywhere([&] {
    return origin_tag_frame(tagged.scene.cameras, tagged.solution, tagged.views,
                            kOriginTag, keep);
  });
  // Two cameras 44 px apart: cv::LevMarq stops at its cap, and the world is
  // refused for the fit there.
  std::vector<TagView> apart(tagged.views.begin(), tagged.views.begin() + 2);
  ASSERT_EQ(apart[1].capture, apart[0].capture);
  for (recon::camera::Vec2d& c : apart[0].marker.corners) {
    c += recon::camera::Vec2d(44.0, 22.0);
  }
  const core::Result<WorldOrigin> stopped = origin_tag_frame(
      tagged.scene.cameras, tagged.solution, apart, kOriginTag);
  ASSERT_TRUE(stopped.ok() && stopped->miss == TagMiss::PoorFit &&
              stopped->reason.find("stopped short of converging") !=
                  std::string::npos)
      << (stopped.ok() ? stopped->reason : stopped.status().message());
  expect_oom_anywhere([&] {
    return origin_tag_frame(tagged.scene.cameras, tagged.solution, apart,
                            kOriginTag);
  });
}

TEST(SolveOom, AnchorRig) {
  const Tagged tagged;
  const WorldOrigin origin =
      origin_tag_frame(tagged.scene.cameras, tagged.solution, tagged.views,
                       kOriginTag)
          .value();
  ASSERT_TRUE(origin.tag);
  expect_oom_anywhere([&] { return anchor_rig(tagged.solution, origin); });
  expect_oom_anywhere([&] { return anchor_rig(tagged.solution, {}); });
  expect_oom_anywhere([&] { return anchor_rig(RigSolution{}, origin); });
  // A tag recon's file cannot name.
  WorldOrigin aruco = origin;
  aruco.sought.dictionary = MarkerDictionary::ArUco4x4_50;
  expect_oom_anywhere([&] { return anchor_rig(tagged.solution, aruco); });
}

TEST(SolveOom, ViewResiduals) {
  const Small scene;
  const compute::CvCamera cam =
      compute::cv_camera(scene.cameras[0].model).value();
  expect_oom_anywhere(
      [&] { return view_residuals(cam, scene.board, scene.views[0], {}, {}); });
  Observation unknown = scene.views[0];
  unknown.points[0].id = 99;
  expect_oom_anywhere(
      [&] { return view_residuals(cam, scene.board, unknown, {}, {}); });
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
  expect_oom_anywhere([&] { return reached_from(apart, 0); });
  expect_oom_anywhere([&] { return reached_from(apart, 3); });
}

// Checking a rig keeps cv_camera's OutOfMemory, never recoding it.
TEST(SolveOom, RigChecks) {
  Small scene;
  expect_oom_anywhere([&] { return check_rig(scene.cameras, 0); });
  expect_oom_anywhere([&] { return check_rig(scene.cameras, 4); });
  // Refusals, as Results (a Status has no status()).
  const std::vector<std::string> twice = {"A", "B", "A"};
  expect_oom_anywhere(
      [&] { return core::Result<bool>(check_camera_ids(twice)); });
  scene.cameras[3].model.intrinsics.fx = 0.0;
  expect_oom_anywhere([&] { return check_rig(scene.cameras, 0); });
  InitialiseOptions options;
  options.min_ambiguity = 0.5;
  expect_oom_anywhere(
      [&] { return core::Result<bool>(check_initialise_options(options)); });
}

// A literal prefix outgrows the inline buffer on both libc++ and libstdc++.
// Its conversion at the call site must not allocate before the error guard.
TEST(SolveOom, OptionsWithALongLiteralPrefix) {
  InitialiseOptions options;
  options.min_ambiguity = 0.5;
  expect_oom_anywhere([&] {
    return core::Result<bool>(check_initialise_options(
        options, "calibration application: initialise options."));
  });
  BundleAdjustOptions adjust;
  adjust.huber_px = 0.0;
  expect_oom_anywhere([&] {
    return core::Result<bool>(check_bundle_adjust_options(
        adjust, "calibration application: bundle adjust options."));
  });
}

}  // namespace
}  // namespace volumetric_kit::calib::solve
