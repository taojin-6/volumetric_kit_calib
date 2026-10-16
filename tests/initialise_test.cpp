// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The first rig on the synthetic rig: 60 placements at 0.3 px, partial and
// collinear views, poor and ambiguous views, an outlier capture, weak and
// missing links, any reference and any order.

#include "volumetric_kit/calib/solve/initialise.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "synthetic_scene.hpp"

namespace volumetric_kit::calib::solve {
namespace {

namespace camera = recon::camera;
using Code = core::Status::Code;

using test::angle_deg;
using test::centre_of;
using test::index_of;
using test::make_scene;
using test::Scene;

core::Result<InitialRig> run(const std::vector<RigCamera>& cameras,
                             const compute::Board& board,
                             const std::vector<Observation>& views,
                             std::size_t reference,
                             const InitialiseOptions& options = {}) {
  core::Result<InitialRig> rig = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(rig = initialise(cameras, board, views, reference, options));
  return rig;
}

bool finite(const camera::Mat4d& m) {
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (!std::isfinite(m[c][r])) return false;
    }
  }
  return true;
}

// Every camera of `rig` within `max_deg` and `max_m` (its centre) of the
// truth in the world of the camera named `reference`; and every board within
// twice that.
void expect_near_truth(const Scene& scene, const std::vector<RigCamera>& order,
                       const InitialRig& rig, const std::string& reference,
                       double max_deg, double max_m) {
  const std::size_t r = index_of(scene.cameras, reference);
  const std::vector<camera::Mat4d> truth = test::truths_in_world_of(scene, r);
  const camera::Mat4d& world_to_reference = scene.rig[r].world_to_camera;
  ASSERT_EQ(rig.world_to_camera.size(), order.size());
  for (std::size_t c = 0; c < order.size(); ++c) {
    const camera::Mat4d& expected = truth[index_of(scene.cameras, order[c].id)];
    const camera::Mat4d& got = rig.world_to_camera[c];
    EXPECT_LT(angle_deg(got, expected), max_deg) << order[c].id;
    EXPECT_LT(glm::length(centre_of(got) - centre_of(expected)), max_m)
        << order[c].id;
  }
  for (const BoardPose& board : rig.boards) {
    const camera::Mat4d expected =
        world_to_reference *
        scene.plan[static_cast<std::size_t>(board.capture)].board_to_world;
    EXPECT_LT(angle_deg(board.board_to_world, expected), 2.0 * max_deg)
        << "capture " << board.capture;
    EXPECT_LT(glm::length(camera::Vec3d(board.board_to_world[3] - expected[3])),
              2.0 * max_m)
        << "capture " << board.capture;
  }
}

// The same rig: each camera's pose, each board, the tree and the records,
// exactly, with cameras matched by id.
void expect_same_rig(const InitialRig& a, const std::vector<RigCamera>& a_cams,
                     const InitialRig& b, const std::vector<RigCamera>& b_cams,
                     bool compare_views = true) {
  ASSERT_EQ(a.world_to_camera.size(), b.world_to_camera.size());
  for (std::size_t c = 0; c < a_cams.size(); ++c) {
    const std::size_t d = index_of(b_cams, a_cams[c].id);
    EXPECT_EQ(a.world_to_camera[c], b.world_to_camera[d]) << a_cams[c].id;
  }
  EXPECT_EQ(a_cams[a.reference].id, b_cams[b.reference].id);
  ASSERT_EQ(a.boards.size(), b.boards.size());
  for (std::size_t i = 0; i < a.boards.size(); ++i) {
    EXPECT_EQ(a.boards[i].capture, b.boards[i].capture);
    EXPECT_EQ(a.boards[i].board_to_world, b.boards[i].board_to_world)
        << "capture " << a.boards[i].capture;
  }
  ASSERT_EQ(a.links.size(), b.links.size());
  for (std::size_t i = 0; i < a.links.size(); ++i) {
    const PlacedLink& x = a.links[i];
    const PlacedLink& y = b.links[i];
    EXPECT_EQ(a_cams[x.camera].id, b_cams[y.camera].id);
    EXPECT_EQ(a_cams[x.parent].id, b_cams[y.parent].id);
    EXPECT_EQ(x.shared, y.shared);
    EXPECT_EQ(x.weak, y.weak);
    EXPECT_EQ(x.inliers, y.inliers);
  }
  if (!compare_views) return;
  ASSERT_EQ(a.views.size(), b.views.size());
  for (std::size_t i = 0; i < a.views.size(); ++i) {
    EXPECT_EQ(a_cams[a.views[i].camera].id, b_cams[b.views[i].camera].id);
    EXPECT_EQ(a.views[i].capture, b.views[i].capture);
    EXPECT_EQ(a.views[i].fate, b.views[i].fate);
    EXPECT_EQ(a.views[i].pnp_rms_px, b.views[i].pnp_rms_px);
  }
}

// The record of camera `id`'s view of `capture`.
const ViewRecord* record_of(const InitialRig& rig,
                            const std::vector<RigCamera>& cameras,
                            const std::string& id, std::int64_t capture) {
  for (const ViewRecord& view : rig.views) {
    if (cameras[view.camera].id == id && view.capture == capture) return &view;
  }
  return nullptr;
}

// Camera `c`'s view of `capture` replaced by its view of the board at
// `board_to_world` instead, without noise: the view fits its own pose better
// than any other, so only the other views can outvote it.
void replace_view(Scene& scene, std::size_t c, std::int64_t capture,
                  const camera::Mat4d& board_to_world) {
  test::Placement moved;
  moved.board_to_world = board_to_world;
  moved.cameras = {c};
  const std::vector<Observation> seen = test::observe(
      scene.rig, scene.board, {moved}, test::ObserveOptions{0.0, 5});
  for (Observation& view : scene.views) {
    if (view.camera_id == scene.cameras[c].id && view.frame_id == capture) {
      view.points = seen.front().points;
    }
  }
}

// The board turned `deg` degrees about its middle, in its own plane.
camera::Mat4d turned(const camera::Mat4d& board_to_world, double deg) {
  const camera::Vec3d middle(0.375, 0.225, 0.0);
  camera::Mat4d turn(camera::rotation_from_rodrigues(
      camera::Vec3d(0.0, 0.0, glm::radians(deg))));
  turn[3] = glm::dvec4(middle - camera::Mat3d(turn) * middle, 1.0);
  return board_to_world * turn;
}

// The board's pose in capture `capture`, and its truth in the world of the
// camera named `reference`.
std::pair<camera::Mat4d, camera::Mat4d> board_and_truth(
    const Scene& scene, const InitialRig& rig, std::int64_t capture,
    const std::string& reference) {
  camera::Mat4d truth(1.0);
  for (const test::SyntheticCamera& cam : scene.rig) {
    if (cam.id == reference) {
      truth = cam.world_to_camera *
              scene.plan[static_cast<std::size_t>(capture)].board_to_world;
    }
  }
  for (const BoardPose& board : rig.boards) {
    if (board.capture == capture) return {board.board_to_world, truth};
  }
  ADD_FAILURE() << "no board for capture " << capture;
  return {camera::Mat4d(1.0), truth};
}

TEST(Initialise, FirstRigIsWithinADegreeAndFiveCentimetres) {
  const Scene scene = make_scene();
  ASSERT_EQ(scene.plan.size(), 60u);
  const core::Result<InitialRig> rig =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  EXPECT_EQ(rig->reference, 0u);
  EXPECT_EQ(rig->world_to_camera[0], camera::Mat4d(1.0));
  expect_near_truth(scene, scene.cameras, *rig, "SYN0", 1.0, 0.05);

  ASSERT_EQ(rig->views.size(), scene.views.size());
  for (const ViewRecord& view : rig->views) {
    EXPECT_EQ(view.fate, ViewFate::Used) << view.capture;
  }
  EXPECT_EQ(rig->boards.size(), scene.plan.size());
  ASSERT_EQ(rig->links.size(), 3u);
  for (const PlacedLink& link : rig->links) {
    EXPECT_FALSE(link.weak);
    EXPECT_EQ(link.inliers, link.shared);
  }
}

// The solver's state (DESIGN conventions): T_cw takes the world into a
// camera, the world is the reference's colour camera, and T_wb takes the
// board into the world, so each corner projects through T_cw T_wb to where
// its camera saw it.
TEST(Initialise, WorldIsTheReferencesColourCamera) {
  const Scene scene = make_scene(20, 0.0);
  const core::Result<InitialRig> rig =
      run(scene.cameras, scene.board, scene.views, 1);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  EXPECT_EQ(rig->world_to_camera[1], camera::Mat4d(1.0));
  std::map<std::int64_t, camera::Mat4d> boards;
  for (const BoardPose& board : rig->boards) {
    boards[board.capture] = board.board_to_world;
  }
  for (const Observation& view : scene.views) {
    const std::size_t c = index_of(scene.cameras, view.camera_id);
    std::vector<cv::Point3d> corners;
    for (const DetectedPoint& p : view.points) {
      corners.push_back(
          scene.board.object_points()[static_cast<std::size_t>(p.id)]);
    }
    const std::vector<cv::Point2d> pixels = test::project(
        scene.cameras[c].model,
        rig->world_to_camera[c] * boards.at(view.frame_id), corners);
    for (std::size_t i = 0; i < pixels.size(); ++i) {
      EXPECT_LT(std::hypot(pixels[i].x - view.points[i].pixel.x,
                           pixels[i].y - view.points[i].pixel.y),
                1e-6)
          << view.camera_id << " capture " << view.frame_id;
    }
  }
  // Noise-free: the first rig is the truth, in the reference's frame.
  expect_near_truth(scene, scene.cameras, *rig, "SYN1", 1e-7, 1e-9);
}

TEST(Initialise, AnyCameraCanBeTheReference) {
  const Scene scene = make_scene();
  for (const std::size_t reference : {2u, 3u}) {
    const core::Result<InitialRig> rig =
        run(scene.cameras, scene.board, scene.views, reference);
    ASSERT_TRUE(rig.ok()) << rig.status().message();
    EXPECT_EQ(rig->reference, reference);
    EXPECT_EQ(rig->world_to_camera[reference], camera::Mat4d(1.0));
    expect_near_truth(scene, scene.cameras, *rig, scene.cameras[reference].id,
                      1.0, 0.05);
  }
}

// The cameras and the observations in any order give the same rig, bit for
// bit, and so does a second run.
TEST(Initialise, IsTheSameForAnyOrderAndEveryRun) {
  const Scene scene = make_scene();
  const core::Result<InitialRig> rig =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  const core::Result<InitialRig> again =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(again.ok());
  expect_same_rig(*rig, scene.cameras, *again, scene.cameras);

  std::vector<RigCamera> reversed(scene.cameras.rbegin(), scene.cameras.rend());
  std::vector<Observation> shuffled = scene.views;
  std::shuffle(shuffled.begin(), shuffled.end(), std::mt19937(3));
  const core::Result<InitialRig> other =
      run(reversed, scene.board, shuffled, 3);  // SYN0, now last
  ASSERT_TRUE(other.ok()) << other.status().message();
  expect_same_rig(*rig, scene.cameras, *other, reversed,
                  /*compare_views=*/false);
  // Each view's record follows its observation.
  for (std::size_t i = 0; i < shuffled.size(); ++i) {
    EXPECT_EQ(reversed[other->views[i].camera].id, shuffled[i].camera_id);
    EXPECT_EQ(other->views[i].capture, shuffled[i].frame_id);
  }

  // Every corner capped: every board candidate scores the same, and the tie
  // goes to the view whose camera's id sorts first, in either order.
  InitialiseOptions tied;
  tied.truncation_px = 1e-9;
  const core::Result<InitialRig> tie =
      run(scene.cameras, scene.board, scene.views, 0, tied);
  const core::Result<InitialRig> other_tie =
      run(reversed, scene.board, shuffled, 3, tied);
  ASSERT_TRUE(tie.ok() && other_tie.ok());
  expect_same_rig(*tie, scene.cameras, *other_tie, reversed,
                  /*compare_views=*/false);
}

// A quarter of the views partial: some corners lost, a row of them alone
// (collinear), or fewer than 4; and a few corners not finite. Nothing is NaN
// and the rig still holds.
TEST(Initialise, PartialAndCollinearViewsGiveNoNaN) {
  Scene scene = make_scene();
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  std::map<std::size_t, compute::ViewUse> expected;
  for (std::size_t i = 0; i < scene.views.size(); ++i) {
    if (uniform(rng) >= 0.25) continue;
    std::vector<DetectedPoint>& points = scene.views[i].points;
    const double kind = uniform(rng);
    if (kind < 0.4) {
      // Lose 1 to 4 corners at random.
      std::shuffle(points.begin(), points.end(), rng);
      const std::size_t lose =
          1 + static_cast<std::size_t>(4.0 * uniform(rng)) % 4;
      points.resize(points.size() - lose);
      continue;  // usable or collinear: whatever usable_for_pose says
    }
    if (kind < 0.7) {
      // One row of corners alone.
      const int row = uniform(rng) < 0.5 ? 0 : 1;
      std::vector<DetectedPoint> kept;
      for (const DetectedPoint& p : points) {
        if (p.id / 4 == row) kept.push_back(p);
      }
      points = kept;
      expected[i] = compute::ViewUse::Collinear;
    } else if (kind < 0.9) {
      points.resize(3);
      expected[i] = compute::ViewUse::TooFewCorners;
    } else {
      points[2].pixel.x = std::numeric_limits<double>::quiet_NaN();
      expected[i] = compute::ViewUse::OutsideImage;
    }
  }
  ASSERT_GE(expected.size(), 25u);

  const core::Result<InitialRig> rig =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  for (const camera::Mat4d& m : rig->world_to_camera) EXPECT_TRUE(finite(m));
  for (const BoardPose& board : rig->boards) {
    EXPECT_TRUE(finite(board.board_to_world)) << board.capture;
  }
  std::size_t partial_used = 0;
  for (std::size_t i = 0; i < rig->views.size(); ++i) {
    const ViewRecord& view = rig->views[i];
    for (const std::optional<double>& value :
         {view.pnp_rms_px, view.ambiguity, view.tilt_deg}) {
      if (value) EXPECT_FALSE(std::isnan(*value)) << i;
    }
    const auto known = expected.find(i);
    if (known != expected.end()) {
      EXPECT_EQ(view.fate, ViewFate::NotUsable) << i;
      EXPECT_EQ(view.use, known->second) << i;
      EXPECT_FALSE(view.pnp_rms_px.has_value()) << i;
    } else if (scene.views[i].points.size() < 8 &&
               view.fate == ViewFate::Used) {
      ++partial_used;
    }
  }
  EXPECT_GT(partial_used, 0u);
  expect_near_truth(scene, scene.cameras, *rig, "SYN0", 1.0, 0.05);
}

// The record of camera `id`'s view of `capture`, and the rig without it.
std::vector<Observation> without(const std::vector<Observation>& views,
                                 const std::string& id, std::int64_t capture) {
  std::vector<Observation> kept;
  for (const Observation& view : views) {
    if (view.camera_id != id || view.frame_id != capture) kept.push_back(view);
  }
  return kept;
}

// A view that does not fit its own pose is left out, listed with its RMS,
// and initialises nothing.
TEST(Initialise, LeavesOutAndListsAViewThatFitsPoorly) {
  Scene scene = make_scene();
  Observation& bad = scene.views[17];
  // One corner 40 px out.
  bad.points[3].pixel += camera::Vec2d(40.0, -15.0);
  const core::Result<InitialRig> rig =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  const ViewRecord& record = rig->views[17];
  EXPECT_EQ(record.fate, ViewFate::PoorFit);
  ASSERT_TRUE(record.pnp_rms_px.has_value());
  EXPECT_GT(*record.pnp_rms_px, 5.0);

  const core::Result<InitialRig> without_it =
      run(scene.cameras, scene.board,
          without(scene.views, bad.camera_id, bad.frame_id), 0);
  ASSERT_TRUE(without_it.ok());
  expect_same_rig(*rig, scene.cameras, *without_it, scene.cameras,
                  /*compare_views=*/false);

  // A limit at its RMS, or looser, uses it.
  for (const double limit : {*record.pnp_rms_px, 2.0 * *record.pnp_rms_px}) {
    InitialiseOptions loose;
    loose.max_pnp_rms_px = limit;
    const core::Result<InitialRig> used =
        run(scene.cameras, scene.board, scene.views, 0, loose);
    ASSERT_TRUE(used.ok());
    EXPECT_NE(used->views[17].fate, ViewFate::PoorFit) << limit;
  }
}

// A view whose two poses fit too alike is kept, with its measures, but
// initialises nothing: the rig is the one without it.
TEST(Initialise, KeepsAnAmbiguousViewButDoesNotInitialiseFromIt) {
  const Scene scene = make_scene();
  const core::Result<InitialRig> first =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(first.ok()) << first.status().message();
  // The least and next least ambiguous views.
  std::vector<std::pair<double, std::size_t>> ambiguity;
  for (std::size_t i = 0; i < first->views.size(); ++i) {
    ambiguity.emplace_back(*first->views[i].ambiguity, i);
  }
  std::sort(ambiguity.begin(), ambiguity.end());
  ASSERT_LT(ambiguity[0].first, ambiguity[1].first);
  const std::size_t i = ambiguity[0].second;

  InitialiseOptions options;
  options.min_ambiguity = 0.5 * (ambiguity[0].first + ambiguity[1].first);
  const core::Result<InitialRig> rig =
      run(scene.cameras, scene.board, scene.views, 0, options);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  EXPECT_EQ(rig->views[i].fate, ViewFate::Ambiguous);
  EXPECT_EQ(rig->views[i].ambiguity, first->views[i].ambiguity);
  EXPECT_EQ(rig->views[i].pnp_rms_px, first->views[i].pnp_rms_px);
  for (std::size_t j = 0; j < rig->views.size(); ++j) {
    if (j != i) EXPECT_EQ(rig->views[j].fate, ViewFate::Used) << j;
  }

  const core::Result<InitialRig> without_it = run(
      scene.cameras, scene.board,
      without(scene.views, scene.views[i].camera_id, scene.views[i].frame_id),
      0, options);
  ASSERT_TRUE(without_it.ok());
  expect_same_rig(*rig, scene.cameras, *without_it, scene.cameras,
                  /*compare_views=*/false);

  // A limit at its ambiguity uses it.
  options.min_ambiguity = ambiguity[0].first;
  const core::Result<InitialRig> at =
      run(scene.cameras, scene.board, scene.views, 0, options);
  ASSERT_TRUE(at.ok());
  EXPECT_EQ(at->views[i].fate, ViewFate::Used);
}

// A view tilted past oblique_deg is flagged and still used.
TEST(Initialise, FlagsObliqueViewsAndUsesThem) {
  const Scene scene = make_scene();
  InitialiseOptions options;
  options.oblique_deg = 40.0;
  const core::Result<InitialRig> rig =
      run(scene.cameras, scene.board, scene.views, 0, options);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  std::size_t oblique = 0;
  for (const ViewRecord& view : rig->views) {
    ASSERT_TRUE(view.tilt_deg.has_value());
    EXPECT_EQ(view.oblique, *view.tilt_deg > 40.0);
    EXPECT_EQ(view.fate, ViewFate::Used);
    oblique += view.oblique ? 1 : 0;
  }
  EXPECT_GT(oblique, 0u);
  EXPECT_LT(oblique, rig->views.size());

  // A view tilted exactly oblique_deg is not oblique.
  const double tilt = *rig->views[5].tilt_deg;
  options.oblique_deg = tilt;
  const core::Result<InitialRig> at =
      run(scene.cameras, scene.board, scene.views, 0, options);
  ASSERT_TRUE(at.ok());
  EXPECT_EQ(*at->views[5].tilt_deg, tilt);
  EXPECT_FALSE(at->views[5].oblique);
}

// One capture where a camera saw the board turned 10 degrees from where the
// others saw it: its relative pose is not an inlier, and the camera is placed
// as well as before.
TEST(Initialise, AnOutlierCaptureDoesNotPlaceTheCamera) {
  Scene scene = make_scene();
  const core::Result<InitialRig> clean =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(clean.ok()) << clean.status().message();
  const PlacedLink link = clean->links.back();
  const std::string& child = scene.cameras[link.camera].id;
  const std::string& parent = scene.cameras[link.parent].id;

  // The first capture both see, with a third camera to place its board: the
  // child's view of the board turned 10 degrees about its middle.
  std::int64_t capture = -1;
  for (std::size_t p = 0; p < scene.plan.size() && capture < 0; ++p) {
    const std::vector<std::size_t>& seen = scene.plan[p].cameras;
    if (seen.size() >= 3 && std::count(seen.begin(), seen.end(), link.camera) &&
        std::count(seen.begin(), seen.end(), link.parent)) {
      capture = static_cast<std::int64_t>(p);
    }
  }
  ASSERT_GE(capture, 0);
  replace_view(
      scene, link.camera, capture,
      turned(scene.plan[static_cast<std::size_t>(capture)].board_to_world,
             10.0));

  const core::Result<InitialRig> rig =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  ASSERT_EQ(rig->links.size(), clean->links.size());
  ASSERT_EQ(rig->links.back().camera, link.camera);
  EXPECT_EQ(rig->links.back().inliers, rig->links.back().shared - 1);
  EXPECT_EQ(record_of(*rig, scene.cameras, child, capture)->fate,
            ViewFate::Used);
  EXPECT_LT(angle_deg(rig->world_to_camera[link.camera],
                      clean->world_to_camera[link.camera]),
            0.1)
      << child << " from " << parent;
  expect_near_truth(scene, scene.cameras, *rig, "SYN0", 1.0, 0.05);

  // Within 15 degrees, the turned capture is an inlier.
  InitialiseOptions wide;
  wide.inlier_deg = 15.0;
  const core::Result<InitialRig> loose =
      run(scene.cameras, scene.board, scene.views, 0, wide);
  ASSERT_TRUE(loose.ok()) << loose.status().message();
  ASSERT_EQ(loose->links.back().camera, link.camera);
  EXPECT_EQ(loose->links.back().inliers, loose->links.back().shared);
}

// A link whose shared captures' relative rotations are spread evenly about
// the truth, a degree either way: their chordal mean is the truth, and the
// median of their translations within a millimetre of it, while each one,
// the medoid included, is a degree and centimetres off.
TEST(Initialise, PlacesACameraByTheMeanOfItsInliers) {
  Scene scene = make_scene(60, 0.0);
  const core::Result<InitialRig> clean =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(clean.ok()) << clean.status().message();
  const PlacedLink link = clean->links.back();  // the last placed: a leaf
  const camera::Mat4d world_to_child = scene.rig[link.camera].world_to_camera;
  std::vector<std::size_t> shared;
  for (std::size_t p = 0; p < scene.plan.size(); ++p) {
    const std::vector<std::size_t>& seen = scene.plan[p].cameras;
    if (std::count(seen.begin(), seen.end(), link.camera) &&
        std::count(seen.begin(), seen.end(), link.parent)) {
      shared.push_back(p);
    }
  }
  ASSERT_EQ(shared.size(), static_cast<std::size_t>(link.shared));
  ASSERT_GE(shared.size(), 4u);
  // The child rolled a degree one way, then the other, in pairs; an odd one
  // out is left as it was.
  for (std::size_t i = 0; i + 1 < shared.size(); i += 2) {
    for (const std::size_t j : {i, i + 1}) {
      const double deg = j == i ? 1.0 : -1.0;
      const camera::Mat4d roll(camera::rotation_from_rodrigues(
          camera::Vec3d(0.0, 0.0, glm::radians(deg))));
      replace_view(scene, link.camera, static_cast<std::int64_t>(shared[j]),
                   camera::rigid_inverse(world_to_child) * roll *
                       world_to_child * scene.plan[shared[j]].board_to_world);
    }
  }

  const core::Result<InitialRig> rig =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  for (const ViewRecord& view : rig->views) {
    EXPECT_EQ(view.fate, ViewFate::Used) << view.capture;
  }
  ASSERT_EQ(rig->links.back().camera, link.camera);
  EXPECT_EQ(rig->links.back().inliers, link.shared);
  const camera::Mat4d expected =
      world_to_child * camera::rigid_inverse(scene.rig[0].world_to_camera);
  const camera::Mat4d& got = rig->world_to_camera[link.camera];
  EXPECT_LT(angle_deg(got, expected), 1e-6);
  EXPECT_LT(glm::length(centre_of(got) - centre_of(expected)), 1e-3);
}

// A capture whose first view, in id order, saw the board turned 10 degrees
// from where the others saw it: the board is the candidate the most views
// agree with, not the first.
TEST(Initialise, EachBoardIsTheCandidateThatFitsItsCaptureBest) {
  Scene scene = make_scene();
  std::int64_t capture = -1;
  for (std::size_t p = 0; p < scene.plan.size() && capture < 0; ++p) {
    const std::vector<std::size_t>& seen = scene.plan[p].cameras;
    if (seen.size() >= 3 && seen.front() == 0) {
      capture = static_cast<std::int64_t>(p);
    }
  }
  ASSERT_GE(capture, 0);
  replace_view(
      scene, 0, capture,
      turned(scene.plan[static_cast<std::size_t>(capture)].board_to_world,
             10.0));
  const core::Result<InitialRig> rig =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  const auto [board, truth] = board_and_truth(scene, *rig, capture, "SYN0");
  EXPECT_LT(angle_deg(board, truth), 1.0);
  expect_near_truth(scene, scene.cameras, *rig, "SYN0", 1.0, 0.05);
}

// A capture whose views are all ambiguous takes its board from them: of
// their poses, the one that fits them all best.
TEST(Initialise, PlacesTheBoardOfACaptureWhoseViewsAreAllAmbiguous) {
  const Scene scene = make_scene();
  const core::Result<InitialRig> plain =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(plain.ok()) << plain.status().message();
  // Each capture's least ambiguous view; the capture where it is least.
  std::map<std::int64_t, double> clearest;
  for (const ViewRecord& view : plain->views) {
    double& a = clearest.try_emplace(view.capture, 0.0).first->second;
    a = std::max(a, *view.ambiguity);
  }
  std::vector<std::pair<double, std::int64_t>> order;
  for (const auto& [capture, a] : clearest) order.emplace_back(a, capture);
  std::sort(order.begin(), order.end());
  const std::int64_t capture = order[0].second;
  InitialiseOptions options;
  options.min_ambiguity = 0.5 * (order[0].first + order[1].first);
  const core::Result<InitialRig> rig =
      run(scene.cameras, scene.board, scene.views, 0, options);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  std::size_t ambiguous = 0;
  for (const ViewRecord& view : rig->views) {
    if (view.capture != capture) continue;
    EXPECT_EQ(view.fate, ViewFate::Ambiguous);
    ++ambiguous;
  }
  ASSERT_GE(ambiguous, 2u);
  EXPECT_EQ(rig->boards.size(), scene.plan.size());
  const auto [board, truth] = board_and_truth(scene, *rig, capture, "SYN0");
  EXPECT_LT(angle_deg(board, truth), 1.0);
  EXPECT_LT(glm::length(camera::Vec3d(board[3] - truth[3])), 0.05);
}

// RMS distance between two sets of pixels.
double rms_px(const std::vector<cv::Point2d>& a,
              const std::vector<cv::Point2d>& b) {
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    sum += (a[i].x - b[i].x) * (a[i].x - b[i].x) +
           (a[i].y - b[i].y) * (a[i].y - b[i].y);
  }
  return std::sqrt(sum / static_cast<double>(a.size()));
}

// One camera of three saw the board moved: by more than the cap in every
// camera, but in its own image by more than in the other two together. An
// uncapped score would follow that one view; with each corner capped, the two
// agreeing views place the board.
TEST(Initialise, CapsEachCornersErrorWhenScoringABoard) {
  Scene scene = make_scene();
  const std::vector<cv::Point3d>& corners = scene.board.object_points();
  // The capture of three cameras, the camera, and the move that does it best.
  std::int64_t capture = -1;
  std::size_t moved_in = 0;
  camera::Vec3d move(0.0);
  double best = 0.0;
  for (std::size_t p = 0; p < scene.plan.size(); ++p) {
    const std::vector<std::size_t>& seen = scene.plan[p].cameras;
    if (seen.size() != 3) continue;
    const camera::Mat4d& board_to_world = scene.plan[p].board_to_world;
    for (int i = 0; i < 400; ++i) {
      // Directions spread over the sphere (a Fibonacci lattice).
      const double z = 1.0 - (i + 0.5) / 200.0;
      const double a = 2.39996322972865332 * i;
      const camera::Vec3d d(std::sqrt(1.0 - z * z) * std::cos(a),
                            std::sqrt(1.0 - z * z) * std::sin(a), z);
      camera::Mat4d shift(1.0);
      shift[3] = glm::dvec4(0.05 * d, 1.0);
      std::vector<double> moved_px;
      for (const std::size_t c : seen) {
        const test::SyntheticCamera& cam = scene.rig[c];
        moved_px.push_back(
            rms_px(test::project(cam.model,
                                 cam.world_to_camera * board_to_world, corners),
                   test::project(cam.model,
                                 cam.world_to_camera * shift * board_to_world,
                                 corners)));
      }
      for (std::size_t k = 0; k < 3; ++k) {
        const double a2 = moved_px[(k + 1) % 3];
        const double b2 = moved_px[(k + 2) % 3];
        const double ratio = moved_px[k] * moved_px[k] / (a2 * a2 + b2 * b2);
        if (std::min(a2, b2) > 15.0 && ratio > best) {
          best = ratio;
          capture = static_cast<std::int64_t>(p);
          moved_in = seen[k];
          move = 0.05 * d;
        }
      }
    }
  }
  ASSERT_GE(capture, 0);
  ASSERT_GT(best, 1.5);
  camera::Mat4d shift(1.0);
  shift[3] = glm::dvec4(move, 1.0);
  replace_view(
      scene, moved_in, capture,
      shift * scene.plan[static_cast<std::size_t>(capture)].board_to_world);

  const core::Result<InitialRig> rig =
      run(scene.cameras, scene.board, scene.views, 0);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  const auto [board, truth] = board_and_truth(scene, *rig, capture, "SYN0");
  EXPECT_LT(glm::length(camera::Vec3d(board[3] - truth[3])), 0.02)
      << scene.cameras[moved_in].id << ", ratio " << best;

  // Uncapped, the one view places the board where it saw it.
  InitialiseOptions uncapped;
  uncapped.truncation_px = 1e6;
  const core::Result<InitialRig> loose =
      run(scene.cameras, scene.board, scene.views, 0, uncapped);
  ASSERT_TRUE(loose.ok()) << loose.status().message();
  const camera::Mat4d followed =
      board_and_truth(scene, *loose, capture, "SYN0").first;
  const camera::Mat4d moved_to =
      scene.rig[0].world_to_camera * shift *
      scene.plan[static_cast<std::size_t>(capture)].board_to_world;
  EXPECT_LT(glm::length(camera::Vec3d(followed[3] - moved_to[3])), 0.01);
}

// A camera that shares no capture with the others' tree is named.
TEST(Initialise, NamesACameraOutsideTheTree) {
  const Scene scene = make_scene();
  // SYN3 alone in the captures it sees; the others keep the rest.
  std::set<std::int64_t> syn3;
  for (const Observation& view : scene.views) {
    if (view.camera_id == "SYN3") syn3.insert(view.frame_id);
  }
  ASSERT_FALSE(syn3.empty());
  std::vector<Observation> apart;
  for (const Observation& view : scene.views) {
    if (view.camera_id == "SYN3" || syn3.count(view.frame_id) == 0) {
      apart.push_back(view);
    }
  }
  core::Result<InitialRig> rig = run(scene.cameras, scene.board, apart, 0);
  EXPECT_EQ(rig.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(rig.status().message(),
            "initialise: spanning_tree: SYN3 shares no capture with the tree "
            "of the reference, SYN0");

  // With no views at all.
  std::vector<Observation> none;
  for (const Observation& view : scene.views) {
    if (view.camera_id != "SYN3") none.push_back(view);
  }
  rig = run(scene.cameras, scene.board, none, 0);
  EXPECT_EQ(rig.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(rig.status().message(),
            "initialise: spanning_tree: SYN3 shares no capture with the tree "
            "of the reference, SYN0; SYN3 has no view");

  // With every view of it left out: the message says why.
  std::vector<Observation> poor = scene.views;
  std::size_t left_out = 0;
  for (Observation& view : poor) {
    if (view.camera_id != "SYN3") continue;
    view.points[3].pixel += camera::Vec2d(40.0, -15.0);
    ++left_out;
  }
  // Another camera's view left out too, which the message does not list.
  std::find_if(poor.begin(), poor.end(), [](const Observation& view) {
    return view.camera_id != "SYN3";
  })->points.resize(3);
  rig = run(scene.cameras, scene.board, poor, 0);
  EXPECT_EQ(rig.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(rig.status().message(),
            "initialise: spanning_tree: SYN3 shares no capture with the tree "
            "of the reference, SYN0; of SYN3's " +
                std::to_string(left_out) + " views none is used: " +
                std::to_string(left_out) + " PoorFit");
}

// A camera sharing 3 captures with the others still links, weakly.
TEST(Initialise, AWeakLinkWarns) {
  const Scene scene = make_scene();
  std::vector<Observation> few;
  int kept = 0;
  for (const Observation& view : scene.views) {
    if (view.camera_id == "SYN2" && kept++ >= 3) continue;
    few.push_back(view);
  }
  const core::Result<InitialRig> rig = run(scene.cameras, scene.board, few, 0);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  for (const PlacedLink& link : rig->links) {
    if (scene.cameras[link.camera].id == "SYN2") {
      EXPECT_TRUE(link.weak);
      EXPECT_LE(link.shared, 3);
    } else {
      EXPECT_FALSE(link.weak) << scene.cameras[link.camera].id;
    }
  }
  expect_near_truth(scene, scene.cameras, *rig, "SYN0", 1.0, 0.05);

  // weak_below as given: 0 flags none, 100 every link.
  for (const int weak_below : {0, 100}) {
    InitialiseOptions options;
    options.weak_below = weak_below;
    const core::Result<InitialRig> other =
        run(scene.cameras, scene.board, few, 0, options);
    ASSERT_TRUE(other.ok()) << other.status().message();
    for (const PlacedLink& link : other->links) {
      EXPECT_EQ(link.weak, weak_below == 100) << weak_below;
    }
  }
}

TEST(Initialise, ARigOfOneCameraPlacesItsBoards) {
  const Scene scene = make_scene(20);
  const std::vector<RigCamera> one = {scene.cameras[1]};
  std::vector<Observation> views;
  for (const Observation& view : scene.views) {
    if (view.camera_id == one[0].id) views.push_back(view);
  }
  ASSERT_FALSE(views.empty());
  const core::Result<InitialRig> rig = run(one, scene.board, views, 0);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  EXPECT_EQ(rig->world_to_camera,
            std::vector<camera::Mat4d>{camera::Mat4d(1.0)});
  EXPECT_TRUE(rig->links.empty());
  EXPECT_EQ(rig->boards.size(), views.size());
  expect_near_truth(scene, one, *rig, one[0].id, 1.0, 0.05);
}

TEST(Initialise, RefusesWhatItCannotSolve) {
  const Scene scene = make_scene(10);
  std::vector<RigCamera> repeated = scene.cameras;
  repeated[2].id = repeated[0].id;
  std::vector<RigCamera> unnamed = scene.cameras;
  unnamed[1].id.clear();
  std::vector<RigCamera> bad_lens = scene.cameras;
  bad_lens[3].model.intrinsics.fx = 0.0;
  std::vector<Observation> stranger = scene.views;
  stranger[4].camera_id = "SYN9";
  std::vector<Observation> twice = scene.views;
  twice.push_back(scene.views[6]);
  const struct {
    const char* name;
    std::vector<RigCamera> cameras;
    std::vector<Observation> views;
    std::size_t reference;
    const char* says;
  } cases[] = {
      {"no cameras", {}, {}, 0, "no cameras"},
      {"a reference outside", scene.cameras, scene.views, 4, "reference 4"},
      {"a repeated id", repeated, scene.views, 0, "appears twice"},
      {"an empty id", unnamed, scene.views, 0, "has no id"},
      {"a bad lens", bad_lens, scene.views, 0, "camera SYN3: cv_camera: "},
      {"an unknown camera", scene.cameras, stranger, 0, "\"SYN9\""},
      {"two views of a capture", scene.cameras, twice, 0, "two views"},
  };
  for (const auto& c : cases) {
    const core::Result<InitialRig> rig =
        run(c.cameras, scene.board, c.views, c.reference);
    EXPECT_EQ(rig.status().domain(), Code::InvalidArgument) << c.name;
    EXPECT_EQ(rig.status().message().rfind("initialise: ", 0), 0u)
        << c.name << ": " << rig.status().message();
    EXPECT_NE(rig.status().message().find(c.says), std::string::npos)
        << c.name << ": " << rig.status().message();
  }
}

TEST(Initialise, RefusesOptionsOutOfRange) {
  const Scene scene = make_scene(10);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const struct {
    const char* field;
    void (*set)(InitialiseOptions&, double);
    std::vector<double> refused;
  } cases[] = {
      {"max_pnp_rms_px",
       [](InitialiseOptions& o, double v) { o.max_pnp_rms_px = v; },
       {0.0, -1.0, nan, inf}},
      {"min_ambiguity",
       [](InitialiseOptions& o, double v) { o.min_ambiguity = v; },
       {0.99, nan, inf}},
      {"oblique_deg",
       [](InitialiseOptions& o, double v) { o.oblique_deg = v; },
       {0.0, 180.5, nan}},
      {"weak_below",
       [](InitialiseOptions& o, double v) {
         o.weak_below = static_cast<int>(v);
       },
       {-1.0}},
      {"inlier_deg",
       [](InitialiseOptions& o, double v) { o.inlier_deg = v; },
       {0.0, 90.0, nan}},
      {"truncation_px",
       [](InitialiseOptions& o, double v) { o.truncation_px = v; },
       {0.0, nan, inf}},
  };
  for (const auto& c : cases) {
    for (const double value : c.refused) {
      InitialiseOptions options;
      c.set(options, value);
      const core::Result<InitialRig> rig =
          run(scene.cameras, scene.board, scene.views, 0, options);
      EXPECT_EQ(rig.status().domain(), Code::InvalidArgument)
          << c.field << " = " << value;
      EXPECT_EQ(rig.status().message().rfind(
                    std::string("initialise: options.") + c.field + " ", 0),
                0u)
          << rig.status().message();
    }
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::solve
