// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The bundle adjustment on the synthetic rig: accuracy over seeds, outliers,
// the Jacobian at rotations near 0 and pi, rotations brought within pi, the
// covariance against Monte Carlo and the dense inverse, the sensitivity
// against a re-solve, the reference, recon's calibration, failures, OpenCV's
// calibrateMultiview, order, and each option.

#include "volumetric_kit/calib/solve/bundle_adjust.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <glm/gtc/constants.hpp>
#include <opencv2/calib.hpp>

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

const double kPi = glm::pi<double>();

// The first rig, or the test fails here: GoogleTest reports the exception
// and goes on to the next test.
InitialRig first_rig(const Scene& scene, std::size_t reference = 0,
                     const InitialiseOptions& options = {}) {
  core::Result<InitialRig> rig =
      initialise(scene.cameras, scene.board, scene.views, reference, options);
  if (!rig.ok()) throw std::runtime_error(rig.status().message());
  return *std::move(rig);
}

core::Result<RigSolution> adjust(const Scene& scene, const InitialRig& start,
                                 const BundleAdjustOptions& options = {}) {
  core::Result<RigSolution> solution = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(solution = bundle_adjust(scene.cameras, scene.board,
                                           scene.views, start, options));
  return solution;
}

// The solution, or the test fails here, as first_rig does.
RigSolution solved(const Scene& scene, const InitialRig& start,
                   const BundleAdjustOptions& options = {}) {
  core::Result<RigSolution> solution = adjust(scene, start, options);
  if (!solution.ok()) throw std::runtime_error(solution.status().message());
  return *std::move(solution);
}

camera::Mat4d matrix_of(const camera::RodriguesTransform& t) {
  return camera::matrix_from_rodrigues(t);
}

// The worst camera's rotation (degrees) and centre (metres) error.
std::pair<double, double> worst_error(const Scene& scene,
                                      const RigSolution& solution) {
  const std::vector<camera::Mat4d> truth =
      test::truths_in_world_of(scene, solution.reference);
  double deg = 0.0;
  double m = 0.0;
  for (std::size_t c = 0; c < truth.size(); ++c) {
    const camera::Mat4d got = matrix_of(solution.world_to_camera[c]);
    deg = std::max(deg, angle_deg(got, truth[c]));
    m = std::max(m, glm::length(centre_of(got) - centre_of(truth[c])));
  }
  return {deg, m};
}

double huber(double s, double delta) {
  return s <= delta ? s * s : 2.0 * delta * s - delta * delta;
}

const SolvedBoard* board_of(const RigSolution& solution, std::int64_t capture) {
  for (const SolvedBoard& board : solution.boards) {
    if (board.capture == capture) return &board;
  }
  return nullptr;
}

// A rejection's key: camera index, capture, corner id (-1 for a view).
using Key = std::tuple<std::size_t, std::int64_t, int>;

std::set<Key> rejected(const RigSolution& solution, Dropped reason) {
  std::set<Key> keys;
  for (const Rejection& r : solution.rejections) {
    if (r.reason == reason) keys.insert({r.camera, r.capture, r.corner});
  }
  return keys;
}

// J^T W J and J^T W e over every corner of the scene's views the solution
// keeps, assembled from view_residuals: W each corner's weight times its
// Huber weight at `delta`, the parameters the cameras but the reference
// (camera 0), then the boards, each in the solution's order.
struct NormalEquations {
  cv::Mat_<double> h;
  cv::Mat_<double> g;
};

NormalEquations normal_equations(const Scene& scene,
                                 const RigSolution& solution, double delta) {
  const int cameras = 6 * (static_cast<int>(scene.cameras.size()) - 1);
  const int n = cameras + 6 * static_cast<int>(solution.boards.size());
  NormalEquations out{cv::Mat_<double>(n, n, 0.0), cv::Mat_<double>(n, 1, 0.0)};
  std::set<Key> dropped;
  for (const CornerResidual& r : solution.residuals) {
    if (r.dropped) dropped.insert({r.camera, r.capture, r.corner});
  }
  for (const Observation& view : scene.views) {
    const std::size_t c = index_of(scene.cameras, view.camera_id);
    const auto kept = [&](const DetectedPoint& point) {
      return dropped.count({c, view.frame_id, point.id}) == 0;
    };
    if (std::none_of(view.points.begin(), view.points.end(), kept)) continue;
    const SolvedBoard* board = board_of(solution, view.frame_id);
    if (board == nullptr) {
      ADD_FAILURE() << "no board for capture " << view.frame_id;
      return out;
    }
    const int ob =
        cameras + 6 * static_cast<int>(board - solution.boards.data());
    const int oc = c == 0 ? -1 : 6 * (static_cast<int>(c) - 1);
    const ViewResiduals at =
        view_residuals(compute::cv_camera(scene.cameras[c].model).value(),
                       scene.board, view, solution.world_to_camera[c],
                       board->board_to_world)
            .value();
    for (std::size_t i = 0; i < view.points.size(); ++i) {
      if (!kept(view.points[i])) continue;
      const double s = glm::length(at.residuals[i]);
      const double w = view.points[i].weight * (s <= delta ? 1.0 : delta / s);
      cv::Mat_<double> j(2, n, 0.0);
      for (int a = 0; a < 2; ++a) {
        for (int k = 0; k < 6; ++k) {
          if (oc >= 0) j(a, oc + k) = at.camera_jacobian[i](a, k);
          j(a, ob + k) = at.board_jacobian[i](a, k);
        }
      }
      cv::Mat_<double> e(2, 1);
      e(0) = at.residuals[i].x;
      e(1) = at.residuals[i].y;
      out.h += w * j.t() * j;
      out.g += w * j.t() * e;
    }
  }
  return out;
}

// Newton's step, by the Huber cost's gradient over the corners `solution`
// keeps: nil at the cost's minimum.
double newton_step(const Scene& scene, const RigSolution& solution,
                   double delta) {
  const NormalEquations normal = normal_equations(scene, solution, delta);
  cv::Mat_<double> step;
  EXPECT_TRUE(cv::solve(normal.h, normal.g, step, cv::DECOMP_CHOLESKY));
  return cv::norm(step, cv::NORM_INF);
}

// A start for `scene` with every view used: the cameras at
// `world_to_camera`, each capture's board at `boards`, in the captures'
// order, and camera 1 linked to camera 0 by every capture.
InitialRig start_with(const Scene& scene,
                      std::vector<camera::Mat4d> world_to_camera,
                      const std::vector<camera::Mat4d>& boards) {
  InitialRig start;
  start.world_to_camera = std::move(world_to_camera);
  for (std::size_t p = 0; p < boards.size(); ++p) {
    start.boards.push_back({static_cast<std::int64_t>(p), boards[p]});
  }
  const int shared = static_cast<int>(boards.size());
  start.links = {PlacedLink{{1, 0, shared, false}, shared}};
  for (const Observation& view : scene.views) {
    ViewRecord record;
    record.camera = index_of(scene.cameras, view.camera_id);
    record.capture = view.frame_id;
    record.fate = ViewFate::Used;
    record.use = compute::ViewUse::Usable;
    start.views.push_back(record);
  }
  return start;
}

// Two cameras looking the same way, FAR a metre behind NEAR (the reference)
// and 0.2 m to its side, both seeing the board at each of `boards` (T_cb in
// NEAR, which is the world) at 0.3 px; and a start at the truth.
struct Pair {
  Scene scene;
  InitialRig start;
};

Pair facing_pair(const std::vector<camera::Mat4d>& boards) {
  camera::Mat4d far(1.0);
  far[3] = glm::dvec4(0.2, 0.0, 1.0, 1.0);
  Scene scene{{{"NEAR", test::color_like_lens(), camera::Mat4d(1.0)},
               {"FAR", test::color_like_lens(), far}},
              test::femto_board(),
              {},
              {},
              {}};
  for (const camera::Mat4d& board : boards)
    scene.plan.push_back({board, {0, 1}});
  scene.views = test::observe(scene.rig, scene.board, scene.plan,
                              test::ObserveOptions{0.3, 1});
  scene.cameras = test::rig_cameras(scene.rig);
  InitialRig start = start_with(scene, {camera::Mat4d(1.0), far}, boards);
  return {std::move(scene), std::move(start)};
}

// Four boards facing NEAR 2.5-3.1 m away, capture 0's tilted 60 degrees so
// its rows are 13 cm apart in depth, at 2.435 and 2.565 m.
std::vector<camera::Mat4d> pair_boards() {
  const compute::Board board = test::femto_board();
  return {test::board_facing_camera(board, {0.0, 0.0, 2.5}, 60.0, 0.0),
          test::board_facing_camera(board, {-0.2, 0.1, 3.0}, 25.0, 30.0),
          test::board_facing_camera(board, {0.15, -0.1, 3.1}, -20.0, -60.0),
          test::board_facing_camera(board, {0.1, 0.15, 2.9}, 10.0, 100.0)};
}

// A corner's depth (metres) in its camera, z of T_cw T_wb X.
double depth_of(const compute::Board& board,
                const camera::Mat4d& world_to_camera,
                const camera::Mat4d& board_to_world, int corner) {
  const cv::Point3d& x =
      board.object_points()[static_cast<std::size_t>(corner)];
  return (world_to_camera * board_to_world * glm::dvec4(x.x, x.y, x.z, 1.0)).z;
}

TEST(BundleAdjust, TwentySeedsWithinAFiftiethOfADegreeAndAMillimetre) {
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    const Scene scene = make_scene(60, 0.3, seed, seed);
    const RigSolution solution = solved(scene, first_rig(scene));
    const auto [deg, m] = worst_error(scene, solution);
    EXPECT_LT(deg, 0.02) << "seed " << seed;
    EXPECT_LT(m, 0.001) << "seed " << seed;
    EXPECT_TRUE(solution.rejections.empty()) << "seed " << seed;
    EXPECT_EQ(solution.solves, 1) << "seed " << seed;
    EXPECT_LT(solution.final_energy, solution.initial_energy);
  }
}

// 2 % of the corners 5-12 px off, which the first rig still uses, and one
// view 6 px off as a whole: the outlier rounds drop exactly those, and the
// rig is within a millimetre.
TEST(BundleAdjust, DropsOutliersAndAShiftedView) {
  Scene scene = make_scene();
  const std::size_t shifted = 7;
  for (DetectedPoint& point : scene.views[shifted].points) {
    point.pixel.x += 6.0;
  }
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  std::set<Key> injected;
  std::size_t corners = 0;
  for (std::size_t i = 0; i < scene.views.size(); ++i) {
    for (DetectedPoint& point : scene.views[i].points) {
      ++corners;
      if (i == shifted || uniform(rng) >= 0.02) continue;
      const double angle = 2.0 * kPi * uniform(rng);
      const double size = 5.0 + 7.0 * uniform(rng);
      point.pixel += size * camera::Vec2d(std::cos(angle), std::sin(angle));
      injected.insert({index_of(scene.cameras, scene.views[i].camera_id),
                       scene.views[i].frame_id, point.id});
    }
  }
  ASSERT_GT(injected.size(), corners / 100);
  const InitialRig start = first_rig(scene);
  // The injected corners in the views the first rig left for adjustment.
  std::set<Key> adjusted;
  for (const Key& key : injected) {
    for (const ViewRecord& record : start.views) {
      if (record.camera == std::get<0>(key) &&
          record.capture == std::get<1>(key) &&
          (record.fate == ViewFate::Used ||
           record.fate == ViewFate::Ambiguous)) {
        adjusted.insert(key);
      }
    }
  }
  ASSERT_GT(adjusted.size(), injected.size() / 2);

  const RigSolution solution = solved(scene, start);
  const auto [deg, m] = worst_error(scene, solution);
  EXPECT_LT(m, 0.001);
  EXPECT_LT(deg, 0.02);
  const Key view{index_of(scene.cameras, scene.views[shifted].camera_id),
                 scene.views[shifted].frame_id, -1};
  EXPECT_EQ(rejected(solution, Dropped::OutlierView), std::set<Key>{view});
  EXPECT_EQ(rejected(solution, Dropped::Outlier), adjusted);
  for (const Rejection& r : solution.rejections) {
    EXPECT_GT(r.value, r.bound);
    EXPECT_GE(r.round, 1);
  }
  EXPECT_GE(solution.solves, 2);
  for (const CornerResidual& r : solution.residuals) {
    const bool out =
        adjusted.count({r.camera, r.capture, r.corner}) > 0 ||
        (r.camera == std::get<0>(view) && r.capture == std::get<1>(view));
    EXPECT_EQ(r.dropped, out)
        << r.camera << " " << r.capture << " " << r.corner;
  }

  // With no rounds, nothing is dropped.
  BundleAdjustOptions none;
  none.outlier_rounds = 0;
  const RigSolution kept = solved(scene, start, none);
  EXPECT_TRUE(kept.rejections.empty());
  EXPECT_EQ(kept.solves, 1);
  for (const CornerResidual& r : kept.residuals) EXPECT_FALSE(r.dropped);
}

// A view whose kept corners no longer fix a pose once an outlier goes is
// dropped whole: 4 corners of a row and 2 of the next, one of them 6 px off.
TEST(BundleAdjust, DropsAViewItsOutliersLeaveUnusable) {
  Scene scene = make_scene();
  Observation& view = scene.views[11];
  std::vector<DetectedPoint> kept;
  for (const DetectedPoint& point : view.points) {
    if (point.id <= 5) kept.push_back(point);
  }
  ASSERT_EQ(kept.size(), 6u);
  kept.back().pixel.y += 6.0;  // id 5
  view.points = kept;
  const InitialRig start = first_rig(scene);
  ASSERT_EQ(start.views[11].fate, ViewFate::Used);
  const RigSolution solution = solved(scene, start);
  const std::size_t c = index_of(scene.cameras, view.camera_id);
  EXPECT_EQ(rejected(solution, Dropped::Outlier),
            (std::set<Key>{{c, view.frame_id, 5}}));
  EXPECT_EQ(rejected(solution, Dropped::Unusable),
            (std::set<Key>{{c, view.frame_id, -1}}));
  for (const CornerResidual& r : solution.residuals) {
    EXPECT_EQ(r.dropped, r.camera == c && r.capture == view.frame_id);
  }
}

// The analytic blocks against central differences of the residuals, and the
// depths against recon's matrices, with the camera's and the board's
// rotations 1e-7, 0.8 and pi - 1e-6 rad.
TEST(ViewResiduals, JacobiansMatchCentralDifferences) {
  const compute::Board board = test::femto_board();
  const compute::CvCamera lens =
      compute::cv_camera(test::color_like_lens()).value();
  const camera::Vec3d camera_axis =
      glm::normalize(camera::Vec3d(0.3, -0.5, 0.8));
  const camera::Vec3d board_axis =
      glm::normalize(camera::Vec3d(-0.6, 0.2, 0.7));
  const cv::Point3f corner = board.charuco().getRightBottomCorner();
  const camera::Vec3d middle(0.5 * corner.x, 0.5 * corner.y, 0.0);
  Observation view;
  for (int id = 0; id < 8; ++id) {
    view.points.push_back({id, {1900.0 + 10.0 * id, 1000.0 - 7.0 * id}, 1.0});
  }
  for (const double camera_angle : {1e-7, 0.8, kPi - 1e-6}) {
    for (const double board_angle : {1e-7, 0.8, kPi - 1e-6}) {
      const camera::RodriguesTransform board_to_world{board_angle * board_axis,
                                                      {0.1, -0.2, 0.3}};
      camera::RodriguesTransform world_to_camera{camera_angle * camera_axis,
                                                 {}};
      // The board's middle 2 m in front of the camera.
      const camera::Vec3d in_world =
          camera::rotation_from_rodrigues(board_to_world.rvec) * middle +
          board_to_world.tvec;
      world_to_camera.tvec =
          camera::Vec3d(0.05, -0.03, 2.0) -
          camera::rotation_from_rodrigues(world_to_camera.rvec) * in_world;
      const ViewResiduals at =
          view_residuals(lens, board, view, world_to_camera, board_to_world)
              .value();
      ASSERT_EQ(at.residuals.size(), 8u);
      ASSERT_EQ(at.depths.size(), 8u);
      for (int id = 0; id < 8; ++id) {
        const double depth =
            depth_of(board, camera::matrix_from_rodrigues(world_to_camera),
                     camera::matrix_from_rodrigues(board_to_world), id);
        EXPECT_GT(depth, 1.5);
        EXPECT_NEAR(at.depths[static_cast<std::size_t>(id)], depth, 1e-12)
            << "camera " << camera_angle << " rad, board " << board_angle
            << " rad, corner " << id;
      }
      for (int k = 0; k < 12; ++k) {
        const auto moved = [&](double h) {
          camera::RodriguesTransform cam = world_to_camera;
          camera::RodriguesTransform brd = board_to_world;
          camera::RodriguesTransform& t = k < 6 ? cam : brd;
          const int i = k % 6;
          (i < 3 ? t.rvec : t.tvec)[i % 3] += h;
          return view_residuals(lens, board, view, cam, brd).value().residuals;
        };
        const double h = 1e-6;
        const std::vector<camera::Vec2d> plus = moved(h);
        const std::vector<camera::Vec2d> minus = moved(-h);
        double scale = 0.0;
        double error = 0.0;
        for (std::size_t i = 0; i < 8; ++i) {
          for (int a = 0; a < 2; ++a) {
            const double numeric = (plus[i][a] - minus[i][a]) / (2.0 * h);
            const double analytic = k < 6 ? at.camera_jacobian[i](a, k)
                                          : at.board_jacobian[i](a, k - 6);
            scale = std::max(scale, std::abs(numeric));
            error = std::max(error, std::abs(analytic - numeric));
          }
        }
        EXPECT_GT(scale, 10.0);
        EXPECT_LT(error, 1e-6 * scale)
            << "camera " << camera_angle << " rad, board " << board_angle
            << " rad, parameter " << k;
      }
    }
  }
}

TEST(ViewResiduals, RefusesACornerTheBoardLacks) {
  const compute::Board board = test::femto_board();
  const compute::CvCamera lens =
      compute::cv_camera(test::color_like_lens()).value();
  for (const int id : {-1, 8}) {
    Observation view;
    view.points.push_back({id, {10.0, 10.0}, 1.0});
    core::Result<ViewResiduals> at = core::Status::invalid_argument("unset");
    EXPECT_NO_THROW(at = view_residuals(lens, board, view, {}, {}));
    EXPECT_EQ(at.status().domain(), Code::InvalidArgument) << id;
    EXPECT_EQ(at.status().message().rfind("view_residuals: ", 0), 0u);
  }
}

// A pixel not a number gives its corner a residual not a number; a transform
// not finite is refused. Neither throws.
TEST(ViewResiduals, TakesAPixelOrATransformNotANumber) {
  const compute::Board board = test::femto_board();
  const compute::CvCamera lens =
      compute::cv_camera(test::color_like_lens()).value();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  Observation view;
  for (int id = 0; id < 8; ++id) {
    view.points.push_back({id, {1900.0 + 10.0 * id, 1000.0 - 7.0 * id}, 1.0});
  }
  view.points[3].pixel.x = nan;
  const camera::RodriguesTransform world_to_camera{{0.1, -0.2, 0.05},
                                                   {-0.3, -0.2, 2.0}};
  const camera::RodriguesTransform board_to_world{{0.0, 0.0, 0.0},
                                                  {0.0, 0.0, 0.0}};
  core::Result<ViewResiduals> at = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(
      at = view_residuals(lens, board, view, world_to_camera, board_to_world));
  ASSERT_TRUE(at.ok()) << at.status().message();
  for (std::size_t i = 0; i < 8; ++i) {
    EXPECT_EQ(std::isnan(at->residuals[i].x), i == 3) << i;
    EXPECT_TRUE(std::isfinite(at->residuals[i].y)) << i;
    EXPECT_TRUE(std::isfinite(at->depths[i])) << i;
  }
  for (int k = 0; k < 4; ++k) {
    camera::RodriguesTransform cam = world_to_camera;
    camera::RodriguesTransform brd = board_to_world;
    camera::RodriguesTransform& t = k < 2 ? cam : brd;
    (k % 2 == 0 ? t.rvec : t.tvec).y = nan;
    EXPECT_NO_THROW(at = view_residuals(lens, board, view, cam, brd));
    EXPECT_EQ(at.status().domain(), Code::InvalidArgument) << k;
    EXPECT_EQ(at.status().message(),
              "view_residuals: a transform is not finite")
        << k;
  }
}

// A camera facing the reference across the boards, and a board upside down
// to the reference: both a little short of a half turn, started a little past
// it the same way, so the solve's vectors end longer than pi. Each comes back
// within pi, about the other way.
TEST(BundleAdjust, BringsEachRotationWithinPi) {
  const double short_of = 1e-3;
  const double past = 1e-4;
  const auto facing_back = [](double angle) {
    camera::Mat4d m(camera::rotation_from_rodrigues({0.0, angle, 0.0}));
    m[3] = glm::dvec4(-camera::Mat3d(m) * camera::Vec3d(0.0, 0.0, 5.0), 1.0);
    return m;
  };
  Scene scene{{{"FRONT", test::color_like_lens(), camera::Mat4d(1.0)},
               {"BACK", test::color_like_lens(), facing_back(kPi - short_of)}},
              test::femto_board(),
              {},
              {},
              {}};
  const auto upside_down = [&](double angle) {
    return test::board_facing_camera(scene.board, {0.1, -0.05, 2.5}, 0.0,
                                     glm::degrees(angle));
  };
  scene.plan = {
      {upside_down(kPi - short_of), {0, 1}},
      {test::board_facing_camera(scene.board, {-0.2, 0.1, 2.4}, 25.0, 30.0),
       {0, 1}},
      {test::board_facing_camera(scene.board, {0.15, 0.2, 2.6}, -20.0, -60.0),
       {0, 1}}};
  scene.views = test::observe(scene.rig, scene.board, scene.plan,
                              test::ObserveOptions{0.0, 1});
  scene.cameras = test::rig_cameras(scene.rig);
  const InitialRig start =
      start_with(scene, {camera::Mat4d(1.0), facing_back(kPi + past)},
                 {upside_down(kPi + past), scene.plan[1].board_to_world,
                  scene.plan[2].board_to_world});
  // The start's vectors turn the other way, short of pi.
  EXPECT_LT(camera::rodrigues_from_matrix(start.world_to_camera[1]).rvec.y,
            0.0);

  const RigSolution solution = solved(scene, start);
  const camera::Vec3d& back = solution.world_to_camera[1].rvec;
  EXPECT_NEAR(back.x, 0.0, 1e-9);
  EXPECT_NEAR(back.y, kPi - short_of, 1e-9);
  EXPECT_NEAR(back.z, 0.0, 1e-9);
  const SolvedBoard* flipped = board_of(solution, 0);
  ASSERT_NE(flipped, nullptr);
  EXPECT_NEAR(flipped->board_to_world.rvec.x, 0.0, 1e-9);
  EXPECT_NEAR(flipped->board_to_world.rvec.y, 0.0, 1e-9);
  EXPECT_NEAR(flipped->board_to_world.rvec.z, kPi - short_of, 1e-9);
  for (const SolvedBoard& board : solution.boards) {
    EXPECT_LE(glm::length(board.board_to_world.rvec), kPi);
  }
  const auto [deg, m] = worst_error(scene, solution);
  EXPECT_LT(deg, 1e-7);
  EXPECT_LT(m, 1e-8);
}

// The noise's covariance against the spread of 40 solves of the same plan
// under fresh noise, each corner weighted 1, as the detector weights them:
// the residuals' variance gives the scale.
TEST(BundleAdjust, CovarianceIsWithinThirtyPercentOfMonteCarlo) {
  constexpr int kRuns = 40;
  std::vector<double> rotation(4, 0.0);
  std::vector<double> centre(4, 0.0);
  std::vector<double> predicted_rotation(4, 0.0);
  std::vector<double> predicted_centre(4, 0.0);
  for (int run = 0; run < kRuns; ++run) {
    Scene scene = make_scene(60, 0.3, 1, 100 + run);
    for (Observation& view : scene.views) {
      for (DetectedPoint& point : view.points) point.weight = 1.0;
    }
    const RigSolution solution = solved(scene, first_rig(scene));
    EXPECT_NEAR(solution.variance, 0.09, 0.01);  // 0.3 px on each axis
    ASSERT_EQ(solution.covariance.size(), 4u);
    const std::vector<camera::Mat4d> truth = test::truths_in_world_of(scene, 0);
    for (std::size_t c = 0; c < 4; ++c) {
      const camera::Mat4d got = matrix_of(solution.world_to_camera[c]);
      rotation[c] += std::pow(angle_deg(got, truth[c]), 2) / kRuns;
      centre[c] +=
          std::pow(1000.0 * glm::length(centre_of(got) - centre_of(truth[c])),
                   2) /
          kRuns;
      predicted_rotation[c] += solution.covariance[c].rotation_deg / kRuns;
      predicted_centre[c] += solution.covariance[c].centre_mm / kRuns;
    }
  }
  EXPECT_EQ(predicted_rotation[0], 0.0);
  EXPECT_EQ(predicted_centre[0], 0.0);
  for (std::size_t c = 1; c < 4; ++c) {
    const double deg = std::sqrt(rotation[c]);
    const double mm = std::sqrt(centre[c]);
    EXPECT_NEAR(predicted_rotation[c], deg, 0.3 * deg) << c;
    EXPECT_NEAR(predicted_centre[c], mm, 0.3 * mm) << c;
  }
}

// On a small rig, each camera's covariance is the variance times its block
// of the whole normal matrix's inverse, assembled here from
// view_residuals.
TEST(BundleAdjust, SchurComplementEqualsTheDenseInverse) {
  const Scene scene = make_scene(6);
  const RigSolution solution = solved(scene, first_rig(scene));
  ASSERT_EQ(solution.boards.size(), 6u);
  const cv::Mat_<double> h = normal_equations(scene, solution, 1.0).h;
  cv::Mat_<double> inverse;
  ASSERT_GT(cv::invert(h, inverse, cv::DECOMP_SVD), 0.0);
  for (std::size_t c = 1; c < 4; ++c) {
    const cv::Matx66d& schur = solution.covariance[c].covariance;
    double scale = 0.0;
    double error = 0.0;
    for (int a = 0; a < 6; ++a) {
      for (int b = 0; b < 6; ++b) {
        const int oc = 6 * (static_cast<int>(c) - 1);
        const double dense = solution.variance * inverse(oc + a, oc + b);
        scale = std::max(scale, std::abs(dense));
        error = std::max(error, std::abs(dense - schur(a, b)));
      }
    }
    EXPECT_GT(scale, 0.0);
    EXPECT_LT(error, 1e-6 * scale) << c;
  }
}

// Each prediction against a re-solve with that camera's intrinsics off: the
// reference's (which moves the others) and another's.
TEST(BundleAdjust, SensitivityMatchesAResolveWithTheIntrinsicsOff) {
  const Scene scene = make_scene();
  const InitialRig start = first_rig(scene);
  const RigSolution base = solved(scene, start);
  ASSERT_EQ(base.sensitivity.size(), 4u);
  for (const std::size_t j : {std::size_t{0}, std::size_t{2}}) {
    for (int kind = 0; kind < 3; ++kind) {
      Scene off = scene;
      camera::PinholeIntrinsics& k = off.cameras[j].model.intrinsics;
      const std::vector<PoseShift>* predicted = nullptr;
      if (kind == 0) {
        k.fx *= 1.003;
        k.fy *= 1.003;
        predicted = &base.sensitivity[j].focal;
      } else if (kind == 1) {
        k.cx += 2.0;
        predicted = &base.sensitivity[j].principal_x;
      } else {
        k.cy += 2.0;
        predicted = &base.sensitivity[j].principal_y;
      }
      const RigSolution moved = solved(off, start);
      double largest = 0.0;
      for (std::size_t c = 0; c < 4; ++c) {
        const camera::Mat4d a = matrix_of(base.world_to_camera[c]);
        const camera::Mat4d b = matrix_of(moved.world_to_camera[c]);
        const camera::Vec3d shift = 1000.0 * (centre_of(b) - centre_of(a));
        const double deg = angle_deg(a, b);
        largest = std::max(largest, glm::length(shift));
        const PoseShift& p = (*predicted)[c];
        EXPECT_LT(glm::length(p.centre_mm - shift),
                  0.05 * glm::length(shift) + 1e-3)
            << "camera " << j << " kind " << kind << " shifts " << c;
        EXPECT_NEAR(p.rotation_deg, deg, 0.05 * deg + 1e-5)
            << "camera " << j << " kind " << kind << " turns " << c;
      }
      EXPECT_GT(largest, 0.1) << "camera " << j << " kind " << kind;
    }
  }
}

TEST(BundleAdjust, TheReferenceIsExactlyTheIdentity) {
  const Scene scene = make_scene();
  const RigSolution solution = solved(scene, first_rig(scene, 2));
  EXPECT_EQ(solution.reference, 2u);
  EXPECT_EQ(solution.world_to_camera[2].rvec, camera::Vec3d(0.0));
  EXPECT_EQ(solution.world_to_camera[2].tvec, camera::Vec3d(0.0));
  EXPECT_EQ(solution.calibration.world.sensor, "SYN2");
  const camera::SensorCalibration* sensor =
      camera::find_sensor(solution.calibration, "SYN2");
  ASSERT_NE(sensor, nullptr);
  ASSERT_TRUE(sensor->color_to_world.has_value());
  EXPECT_EQ(*sensor->color_to_world, camera::Mat4d(1.0));
  const cv::Matx66d& zero = solution.covariance[2].covariance;
  EXPECT_EQ(cv::norm(zero, cv::NORM_INF), 0.0);
  const auto [deg, m] = worst_error(scene, solution);
  EXPECT_LT(deg, 0.02);
  EXPECT_LT(m, 0.001);
}

// The solution as recon's calibration: valid, the poses' inverses, the
// lenses as given; and it reads back through recon's writer and reader.
TEST(BundleAdjust, ItsCalibrationIsRecons) {
  const Scene scene = make_scene(20);
  const RigSolution solution = solved(scene, first_rig(scene));
  const camera::ArrayCalibration& calibration = solution.calibration;
  EXPECT_TRUE(camera::validate_array_calibration(calibration).ok());
  EXPECT_EQ(calibration.world.sensor, "SYN0");
  ASSERT_EQ(calibration.sensors.size(), 4u);
  for (std::size_t c = 0; c < 4; ++c) {
    const camera::SensorCalibration& sensor = calibration.sensors[c];
    EXPECT_EQ(sensor.id, scene.cameras[c].id);  // in id order
    EXPECT_EQ(*sensor.color_to_world,
              camera::rigid_inverse(matrix_of(solution.world_to_camera[c])));
    ASSERT_TRUE(sensor.color.has_value());
    EXPECT_EQ(sensor.color->source, camera::IntrinsicsSource::Factory);
    EXPECT_EQ(sensor.color->model.intrinsics.fx,
              scene.cameras[c].model.intrinsics.fx);
    EXPECT_EQ(sensor.color->model.distortion.k6,
              scene.cameras[c].model.distortion.k6);
    EXPECT_FALSE(sensor.depth.has_value());
  }
  const std::string text =
      camera::format_array_calibration(calibration).value();
  const camera::ArrayCalibration back =
      camera::parse_array_calibration(text).value();
  EXPECT_EQ(back.world.sensor, "SYN0");
  ASSERT_EQ(back.sensors.size(), 4u);
  for (std::size_t c = 0; c < 4; ++c) {
    const camera::Mat4d& a = *calibration.sensors[c].color_to_world;
    const camera::Mat4d& b = *back.sensors[c].color_to_world;
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        EXPECT_NEAR(a[col][row], b[col][row], 1e-12) << c;
      }
    }
  }
}

TEST(BundleAdjust, NotConvergingOrNotFiniteIsNumerical) {
  const Scene scene = make_scene(20);
  const InitialRig start = first_rig(scene);
  BundleAdjustOptions once;
  once.max_iterations = 1;
  core::Result<RigSolution> solution = adjust(scene, start, once);
  EXPECT_EQ(solution.status().domain(), Code::Numerical);
  EXPECT_NE(solution.status().message().find("did not converge"),
            std::string::npos)
      << solution.status().message();
  // A lens so long that the residuals overflow.
  Scene far = scene;
  far.cameras[2].model.intrinsics.fx = 1e200;
  far.cameras[2].model.intrinsics.fy = 1e200;
  solution = adjust(far, start);
  EXPECT_EQ(solution.status().domain(), Code::Numerical);
  EXPECT_NE(solution.status().message().find("not finite"), std::string::npos)
      << solution.status().message();
  EXPECT_EQ(solution.status().message().rfind("bundle_adjust: ", 0), 0u);
}

// OpenCV's own multi-camera adjustment, its intrinsics held, on views of the
// whole board: the two rigs agree.
TEST(BundleAdjust, AgreesWithCalibrateMultiviewOnFullViews) {
  const Scene scene = make_scene();
  const RigSolution solution = solved(scene, first_rig(scene));
  const int k = 4;
  const int frames = static_cast<int>(scene.plan.size());
  cv::Mat object;  // 8 x 3, as calibrateMultiview takes it
  cv::Mat(scene.board.object_points()).reshape(1, 8).convertTo(object, CV_32F);
  const std::vector<cv::Mat> objects(static_cast<std::size_t>(frames), object);
  std::vector<std::vector<cv::Mat>> images(
      k, std::vector<cv::Mat>(static_cast<std::size_t>(frames),
                              cv::Mat(8, 2, CV_32F, cv::Scalar(-1.0))));
  cv::Mat mask(k, frames, CV_8U, cv::Scalar(0));
  for (const Observation& view : scene.views) {
    const int c = static_cast<int>(index_of(scene.cameras, view.camera_id));
    ASSERT_EQ(view.points.size(), 8u);
    cv::Mat pixels(8, 2, CV_32F);  // as calibrateMultiview takes them
    for (int i = 0; i < 8; ++i) {
      pixels.at<float>(i, 0) = static_cast<float>(view.points[i].pixel.x);
      pixels.at<float>(i, 1) = static_cast<float>(view.points[i].pixel.y);
    }
    images[c][static_cast<std::size_t>(view.frame_id)] = pixels;
    mask.at<std::uint8_t>(c, static_cast<int>(view.frame_id)) = 1;
  }
  std::vector<cv::Size> sizes;
  std::vector<cv::Mat> ks;
  std::vector<cv::Mat> distortions;
  for (const RigCamera& cam : scene.cameras) {
    const compute::CvCamera lens = compute::cv_camera(cam.model).value();
    sizes.push_back(lens.size);
    ks.push_back(cv::Mat(lens.camera_matrix).clone());
    distortions.push_back(cv::Mat(lens.distortion).reshape(1, 1).clone());
  }
  const cv::Mat models(k, 1, CV_8U, cv::Scalar(cv::CALIB_MODEL_PINHOLE));
  std::vector<cv::Mat> rs;
  std::vector<cv::Mat> ts;
  cv::calibrateMultiview(objects, images, sizes, mask, models, ks, distortions,
                         rs, ts, cv::noArray(), cv::CALIB_USE_INTRINSIC_GUESS);
  for (int c = 1; c < k; ++c) {
    const camera::Mat4d theirs = camera::matrix_from_rodrigues(
        {{rs[c].at<double>(0), rs[c].at<double>(1), rs[c].at<double>(2)},
         {ts[c].at<double>(0), ts[c].at<double>(1), ts[c].at<double>(2)}});
    const camera::Mat4d ours = matrix_of(solution.world_to_camera[c]);
    EXPECT_LT(angle_deg(ours, theirs), 1e-3) << c;
    EXPECT_LT(glm::length(centre_of(ours) - centre_of(theirs)), 5e-5) << c;
  }
}

// The cameras and the observations in any order give the same solution, bit
// for bit, and so does a second run.
TEST(BundleAdjust, IsTheSameForAnyOrderAndEveryRun) {
  const Scene scene = make_scene();
  const RigSolution a = solved(scene, first_rig(scene));
  const RigSolution again = solved(scene, first_rig(scene));
  Scene other = scene;
  std::reverse(other.cameras.begin(), other.cameras.end());
  std::shuffle(other.views.begin(), other.views.end(), std::mt19937(3));
  const RigSolution b = solved(other, first_rig(other, 3));  // SYN0 last
  for (const RigSolution* x : {&again, &b}) {
    const std::vector<RigCamera>& cams =
        x == &b ? other.cameras : scene.cameras;
    for (std::size_t c = 0; c < 4; ++c) {
      const std::size_t d = index_of(cams, scene.cameras[c].id);
      EXPECT_EQ(a.world_to_camera[c].rvec, x->world_to_camera[d].rvec);
      EXPECT_EQ(a.world_to_camera[c].tvec, x->world_to_camera[d].tvec);
      EXPECT_EQ(cv::norm(a.covariance[c].covariance,
                         x->covariance[d].covariance, cv::NORM_INF),
                0.0);
      EXPECT_EQ(a.sensitivity[c].focal[c].centre_mm,
                x->sensitivity[d].focal[d].centre_mm);
    }
    ASSERT_EQ(a.boards.size(), x->boards.size());
    for (std::size_t i = 0; i < a.boards.size(); ++i) {
      EXPECT_EQ(a.boards[i].capture, x->boards[i].capture);
      EXPECT_EQ(a.boards[i].board_to_world.rvec,
                x->boards[i].board_to_world.rvec);
    }
    EXPECT_EQ(a.final_energy, x->final_energy);
    EXPECT_EQ(a.iterations, x->iterations);
    EXPECT_EQ(a.variance, x->variance);
    EXPECT_EQ(a.calibration.sensors.size(), x->calibration.sensors.size());
  }
}

// A quarter of the views partial: corners lost, or a row alone. The views
// the first rig uses are adjusted, and nothing is NaN.
TEST(BundleAdjust, PartialViewsGiveNoNaN) {
  Scene scene = make_scene();
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  for (Observation& view : scene.views) {
    if (uniform(rng) >= 0.25) continue;
    std::vector<DetectedPoint>& points = view.points;
    if (uniform(rng) < 0.6) {
      std::shuffle(points.begin(), points.end(), rng);
      points.resize(points.size() - 1 -
                    static_cast<std::size_t>(4.0 * uniform(rng)) % 4);
    } else {
      points.resize(4);  // one row: collinear
    }
  }
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  for (const camera::RodriguesTransform& t : solution.world_to_camera) {
    for (int i = 0; i < 3; ++i) {
      EXPECT_TRUE(std::isfinite(t.rvec[i]) && std::isfinite(t.tvec[i]));
    }
  }
  for (const CornerResidual& r : solution.residuals) {
    EXPECT_TRUE(std::isfinite(r.residual.x) && std::isfinite(r.residual.y));
  }
  for (const PoseCovariance& cov : solution.covariance) {
    EXPECT_TRUE(std::isfinite(cov.rotation_deg) &&
                std::isfinite(cov.centre_mm));
  }
  std::size_t partial = 0;
  for (std::size_t i = 0; i < scene.views.size(); ++i) {
    if (start.views[i].fate == ViewFate::Used &&
        scene.views[i].points.size() < 8) {
      ++partial;
    }
  }
  EXPECT_GT(partial, 0u);
  const auto [deg, m] = worst_error(scene, solution);
  EXPECT_LT(deg, 0.02);
  EXPECT_LT(m, 0.001);
}

// A capture whose views are all ambiguous has a board, and its views are
// adjusted.
TEST(BundleAdjust, AdjustsAmbiguousViews) {
  const Scene scene = make_scene();
  const InitialRig plain = first_rig(scene);
  // Each capture's least ambiguous view; the capture where it is least.
  std::map<std::int64_t, double> clearest;
  for (const ViewRecord& view : plain.views) {
    double& a = clearest.try_emplace(view.capture, 0.0).first->second;
    a = std::max(a, *view.ambiguity);
  }
  std::vector<std::pair<double, std::int64_t>> order;
  for (const auto& [capture, a] : clearest) order.emplace_back(a, capture);
  std::sort(order.begin(), order.end());
  InitialiseOptions options;
  options.min_ambiguity = 0.5 * (order[0].first + order[1].first);
  const std::int64_t capture = order[0].second;
  const InitialRig start = first_rig(scene, 0, options);
  std::size_t ambiguous = 0;
  for (const ViewRecord& view : start.views) {
    if (view.capture != capture) continue;
    EXPECT_EQ(view.fate, ViewFate::Ambiguous);
    ++ambiguous;
  }
  ASSERT_GE(ambiguous, 2u);
  const RigSolution solution = solved(scene, start);
  ASSERT_NE(board_of(solution, capture), nullptr);
  std::size_t corners = 0;
  for (const CornerResidual& r : solution.residuals) {
    if (r.capture != capture) continue;
    EXPECT_FALSE(r.dropped);
    EXPECT_LT(glm::length(r.residual), 1.5);
    ++corners;
  }
  EXPECT_EQ(corners, 8 * ambiguous);
}

// The energy is the Huber cost at huber_px, each corner's Huber weight is
// its IRLS weight, and the solution is the cost's minimum: Newton's step
// from it, assembled here from view_residuals, is nil.
TEST(BundleAdjust, MinimisesTheHuberCostAtHuberPx) {
  const Scene scene = make_scene(20);
  const InitialRig start = first_rig(scene);
  for (const double delta : {1.0, 0.25, 1e6}) {
    BundleAdjustOptions options;
    options.huber_px = delta;
    const RigSolution solution = solved(scene, start, options);
    double energy = 0.0;
    std::size_t beyond = 0;
    for (const CornerResidual& r : solution.residuals) {
      if (r.dropped) continue;
      const double s = glm::length(r.residual);
      energy += r.weight * huber(s, delta);
      EXPECT_DOUBLE_EQ(r.huber_weight, s <= delta ? 1.0 : delta / s);
      beyond += s > delta ? 1 : 0;
    }
    EXPECT_NEAR(solution.final_energy, energy, 1e-9 * energy) << delta;
    if (delta == 0.25) {
      EXPECT_GT(beyond, solution.residuals.size() / 4);
    }
    if (delta == 1e6) {
      EXPECT_EQ(beyond, 0u);
    }

    EXPECT_LT(newton_step(scene, solution, delta), 1e-6) << delta;
  }
}

// Every corner a solution keeps at least `min_depth_m` from its camera.
void expect_kept_at_least(const Scene& scene, const RigSolution& solution,
                          double min_depth_m) {
  for (const CornerResidual& r : solution.residuals) {
    if (r.dropped) continue;
    const SolvedBoard* board = board_of(solution, r.capture);
    ASSERT_NE(board, nullptr);
    EXPECT_GE(
        depth_of(scene.board, matrix_of(solution.world_to_camera[r.camera]),
                 matrix_of(board->board_to_world), r.corner),
        min_depth_m)
        << r.camera << " " << r.capture << " " << r.corner;
  }
}

// Corners nearer their cameras than min_depth_m as the first solve begins
// are dropped, and one exactly that near is kept; no step brings a kept
// corner nearer.
TEST(BundleAdjust, DropsCornersNearerThanMinDepth) {
  const Scene scene = make_scene();
  const InitialRig start = first_rig(scene);
  std::map<std::int64_t, camera::Mat4d> boards;
  for (const BoardPose& b : start.boards) boards[b.capture] = b.board_to_world;
  std::vector<std::pair<double, Key>> depths;
  for (std::size_t v = 0; v < scene.views.size(); ++v) {
    const Observation& view = scene.views[v];
    const std::size_t c = start.views[v].camera;
    for (const DetectedPoint& point : view.points) {
      depths.push_back({depth_of(scene.board, start.world_to_camera[c],
                                 boards.at(view.frame_id), point.id),
                        {c, view.frame_id, point.id}});
    }
  }
  std::sort(depths.begin(), depths.end());
  BundleAdjustOptions options;
  options.min_depth_m = 0.5 * (depths[4].first + depths[5].first);
  const RigSolution solution = solved(scene, start, options);
  std::set<Key> nearest;
  for (std::size_t i = 0; i < 5; ++i) nearest.insert(depths[i].second);
  EXPECT_EQ(rejected(solution, Dropped::TooNear), nearest);
  for (const Rejection& r : solution.rejections) {
    if (r.reason != Dropped::TooNear) continue;
    EXPECT_EQ(r.round, 0);
    EXPECT_LT(r.value, r.bound);
    EXPECT_EQ(r.bound, options.min_depth_m);
  }
  expect_kept_at_least(scene, solution, options.min_depth_m);
  options.min_depth_m = depths[0].first;
  const RigSolution all = solved(scene, start, options);
  EXPECT_TRUE(rejected(all, Dropped::TooNear).empty());
  expect_kept_at_least(scene, all, options.min_depth_m);
}

// A view left with one row once its corners nearer than min_depth_m go is
// dropped whole as the first solve begins: capture 0's rows, either side of
// min_depth_m in NEAR alone. FAR's view still places the board.
TEST(BundleAdjust, DropsAViewItsTooNearCornersLeaveUnusable) {
  const Pair pair = facing_pair(pair_boards());
  BundleAdjustOptions options;
  options.min_depth_m = 2.5;
  std::set<Key> near;
  for (const Observation& view : pair.scene.views) {
    const std::size_t c = index_of(pair.scene.cameras, view.camera_id);
    for (const DetectedPoint& point : view.points) {
      if (depth_of(pair.scene.board, pair.scene.rig[c].world_to_camera,
                   pair.scene.plan[static_cast<std::size_t>(view.frame_id)]
                       .board_to_world,
                   point.id) < options.min_depth_m) {
        near.insert({c, view.frame_id, point.id});
      }
    }
  }
  ASSERT_EQ(near, (std::set<Key>{{0, 0, 0}, {0, 0, 1}, {0, 0, 2}, {0, 0, 3}}));
  const RigSolution solution = solved(pair.scene, pair.start, options);
  EXPECT_EQ(rejected(solution, Dropped::TooNear), near);
  EXPECT_EQ(rejected(solution, Dropped::Unusable), (std::set<Key>{{0, 0, -1}}));
  EXPECT_EQ(solution.rejections.size(), 5u);
  for (const Rejection& r : solution.rejections) EXPECT_EQ(r.round, 0);
  for (const CornerResidual& r : solution.residuals) {
    EXPECT_EQ(r.dropped, r.camera == 0 && r.capture == 0)
        << r.camera << " " << r.capture << " " << r.corner;
  }
  EXPECT_NE(board_of(solution, 0), nullptr);
}

// A start that puts capture 3's board behind both cameras: every corner of it
// is too near, so neither view fixes a pose, its board is not adjusted, and
// its corners' residuals are not numbers.
TEST(BundleAdjust, LeavesOutABoardStartedBehindItsCameras) {
  Pair pair = facing_pair(pair_boards());
  pair.start.boards[3].board_to_world[3][2] = -2.0;  // NEAR's frame
  const RigSolution solution = solved(pair.scene, pair.start);
  EXPECT_EQ(board_of(solution, 3), nullptr);
  EXPECT_EQ(solution.boards.size(), 3u);
  EXPECT_EQ(rejected(solution, Dropped::Unusable),
            (std::set<Key>{{0, 3, -1}, {1, 3, -1}}));
  EXPECT_EQ(rejected(solution, Dropped::TooNear).size(), 16u);
  std::size_t behind = 0;
  for (const CornerResidual& r : solution.residuals) {
    if (r.capture != 3) {
      EXPECT_FALSE(r.dropped);
      EXPECT_LT(glm::length(r.residual), 1.5);
      continue;
    }
    EXPECT_TRUE(r.dropped);
    EXPECT_TRUE(std::isnan(r.residual.x) && std::isnan(r.residual.y));
    ++behind;
  }
  EXPECT_EQ(behind, 16u);
}

// A corner weighted 0 adds nothing to the fit, so the variance does not count
// its residuals: a view weighted 0 throughout.
TEST(BundleAdjust, VarianceCountsTheWeightedResiduals) {
  Scene scene = make_scene(20);
  const std::size_t v = 5;
  for (DetectedPoint& point : scene.views[v].points) point.weight = 0.0;
  const InitialRig start = first_rig(scene);
  ASSERT_EQ(start.views[v].fate, ViewFate::Used);
  const RigSolution solution = solved(scene, start);
  double weighted = 0.0;
  int m = 0;
  std::size_t unweighted = 0;
  for (const CornerResidual& r : solution.residuals) {
    if (r.dropped) continue;
    weighted += r.weight * r.huber_weight * glm::dot(r.residual, r.residual);
    m += r.weight > 0.0 ? 2 : 0;
    unweighted += r.weight > 0.0 ? 0 : 1;
  }
  ASSERT_EQ(unweighted, 8u);
  const int n = 6 * 3 + 6 * static_cast<int>(solution.boards.size());
  EXPECT_NEAR(solution.variance, weighted / (m - n), 1e-12 * weighted / m);
}

// One corner 2 px off, inside the default bound: outlier_px below its
// residual drops it, and at its residual keeps it. outlier_mads alone (a
// tiny floor) drops exactly the corners beyond that many robust deviations.
TEST(BundleAdjust, HonoursTheCornerBounds) {
  Scene scene = make_scene(30);
  scene.views[4].points[2].pixel.x += 2.0;
  const InitialRig start = first_rig(scene);
  BundleAdjustOptions none;
  none.outlier_rounds = 0;
  const RigSolution first = solved(scene, start, none);
  const std::size_t c = start.views[4].camera;
  const std::int64_t capture = scene.views[4].frame_id;
  const int id = scene.views[4].points[2].id;
  double residual = 0.0;
  std::vector<double> components;
  for (const CornerResidual& r : first.residuals) {
    components.push_back(r.residual.x);
    components.push_back(r.residual.y);
    if (r.camera == c && r.capture == capture && r.corner == id) {
      residual = glm::length(r.residual);
    }
  }
  ASSERT_GT(residual, 1.5);
  EXPECT_TRUE(solved(scene, start).rejections.empty());  // below 3 px
  BundleAdjustOptions options;
  options.outlier_px = residual * (1.0 - 1e-9);
  EXPECT_EQ(rejected(solved(scene, start, options), Dropped::Outlier),
            std::set<Key>{Key(c, capture, id)});
  options.outlier_px = residual;
  EXPECT_TRUE(solved(scene, start, options).rejections.empty());

  // The robust deviation: 1.4826 times the median absolute deviation of the
  // residuals' components from their median.
  const auto median = [](std::vector<double> v) {
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
  };
  const double centre = median(components);
  for (double& x : components) x = std::abs(x - centre);
  const double sigma = 1.4826 * median(components);
  options.outlier_px = 1e-9;
  options.outlier_mads = 2.5;
  options.outlier_rounds = 1;
  std::set<Key> beyond;
  for (const CornerResidual& r : first.residuals) {
    if (glm::length(r.residual) > 2.5 * sigma) {
      beyond.insert({r.camera, r.capture, r.corner});
    }
  }
  ASSERT_GT(beyond.size(), 1u);
  const RigSolution robust = solved(scene, start, options);
  EXPECT_EQ(rejected(robust, Dropped::Outlier), beyond);
  for (const Rejection& r : robust.rejections) {
    EXPECT_NEAR(r.bound, 2.5 * sigma, 1e-12);
  }
}

// One view 2.5 px off as a whole, in a capture every camera sees, each
// corner inside the corner bound: the view bound drops it, and the rig is
// solved again without it; unless outlier_view_ratio or outlier_view_px puts
// the bound at or above its RMS.
TEST(BundleAdjust, HonoursTheViewBounds) {
  Scene scene = make_scene(30);
  std::size_t v = 0;
  while (scene.plan[static_cast<std::size_t>(scene.views[v].frame_id)]
             .cameras.size() < 4) {
    ++v;
  }
  for (DetectedPoint& point : scene.views[v].points) point.pixel.y -= 2.5;
  const InitialRig start = first_rig(scene);
  const std::size_t c = start.views[v].camera;
  const std::int64_t capture = scene.views[v].frame_id;
  const RigSolution solution = solved(scene, start);
  EXPECT_EQ(rejected(solution, Dropped::OutlierView),
            std::set<Key>{Key(c, capture, -1)});
  ASSERT_EQ(solution.rejections.size(), 1u);
  EXPECT_EQ(solution.solves, 2);
  EXPECT_LT(newton_step(scene, solution, 1.0), 1e-6);
  const double rms = solution.rejections[0].value;
  const double median = solution.rejections[0].bound / 3.0;
  ASSERT_GT(3.0 * median, 1.0);  // the ratio's bound, not the floor
  BundleAdjustOptions options;
  // Division then multiplication can round below rms: check ratios just
  // above and below the boundary, beyond that rounding error.
  options.outlier_view_ratio = (rms / median) * (1.0 + 1e-9);
  EXPECT_TRUE(solved(scene, start, options).rejections.empty());
  options.outlier_view_ratio = (rms / median) * (1.0 - 1e-9);
  EXPECT_EQ(rejected(solved(scene, start, options), Dropped::OutlierView),
            std::set<Key>{Key(c, capture, -1)});
  options.outlier_view_ratio = 3.0;
  options.outlier_view_px = rms;
  EXPECT_TRUE(solved(scene, start, options).rejections.empty());
  options.outlier_view_px = rms * (1.0 - 1e-9);
  EXPECT_EQ(rejected(solved(scene, start, options), Dropped::OutlierView),
            std::set<Key>{Key(c, capture, -1)});
}

// SYN2 left with `views` views, each of a capture its parent in the tree
// sees too; and that parent's id.
std::pair<Scene, std::string> syn2_with(int views) {
  const Scene full = make_scene();
  Scene scene = full;
  scene.views.clear();
  int kept = 0;
  for (const Observation& view : full.views) {
    if (view.camera_id == "SYN2" && kept++ >= views) continue;
    scene.views.push_back(view);
  }
  std::string parent;
  for (const PlacedLink& link : first_rig(scene).links) {
    if (scene.cameras[link.camera].id != "SYN2") continue;
    EXPECT_EQ(link.shared, views);
    parent = scene.cameras[link.parent].id;
  }
  return {std::move(scene), parent};
}

// A camera left with 3 shared captures: min_shared 3 holds, 4 refuses it,
// naming the link's cameras and their captures; so does a link short once a
// view is dropped, or from the start.
TEST(BundleAdjust, HonoursMinShared) {
  {
    const auto [scene, parent] = syn2_with(3);
    const InitialRig start = first_rig(scene);
    BundleAdjustOptions options;
    EXPECT_TRUE(adjust(scene, start, options).ok());
    options.min_shared = 4;
    const core::Result<RigSolution> refused = adjust(scene, start, options);
    EXPECT_EQ(refused.status().domain(), Code::InvalidArgument);
    EXPECT_EQ(refused.status().message(),
              "bundle_adjust: SYN2 and " + parent +
                  " share 3 captures in which both keep a view, fewer than "
                  "min_shared (4)");
  }
  {
    // SYN2's view 6 px off in a capture every camera sees, whose board the
    // others hold: SYN2's pose takes up some of it, and the round drops two
    // of its three views.
    auto [scene, parent] = syn2_with(3);
    for (Observation& view : scene.views) {
      const std::size_t capture = static_cast<std::size_t>(view.frame_id);
      if (view.camera_id != "SYN2" || scene.plan[capture].cameras.size() < 4) {
        continue;
      }
      for (DetectedPoint& point : view.points) point.pixel.x += 6.0;
      break;
    }
    const core::Result<RigSolution> refused = adjust(scene, first_rig(scene));
    EXPECT_EQ(refused.status().message(),
              "bundle_adjust: SYN2 and " + parent +
                  " share 1 capture in which both keep a view (3 before "
                  "views were dropped), fewer than min_shared (3)");
  }
  {
    const auto [scene, parent] = syn2_with(1);
    const core::Result<RigSolution> refused = adjust(scene, first_rig(scene));
    EXPECT_EQ(refused.status().message(),
              "bundle_adjust: SYN2 and " + parent +
                  " share 1 capture in which both keep a view, fewer than "
                  "min_shared (3)");
  }
}

// Zero-weight corners cannot make a view count toward min_shared: the
// positive-weight corners alone must fix a pose in each shared capture.
TEST(BundleAdjust, SharedCapturesNeedPositiveWeightPoseSupport) {
  for (const std::vector<int>& positive : std::vector<std::vector<int>>{
           {}, {0, 1, 4}, {0, 1, 2, 3}, {0, 1, 4, 5}}) {
    SCOPED_TRACE(testing::PrintToString(positive));
    Pair pair = facing_pair(pair_boards());
    pair.scene.views =
        test::observe(pair.scene.rig, pair.scene.board, pair.scene.plan,
                      test::ObserveOptions{0.0, 1});
    for (Observation& view : pair.scene.views) {
      if (view.camera_id != "FAR" || view.frame_id == 0) continue;
      for (DetectedPoint& point : view.points) {
        point.weight = std::find(positive.begin(), positive.end(), point.id) !=
                               positive.end()
                           ? 0.5
                           : 0.0;
      }
    }
    const core::Result<RigSolution> solution = adjust(pair.scene, pair.start);
    if (positive == std::vector<int>({0, 1, 4, 5})) {
      EXPECT_TRUE(solution.ok()) << solution.status().message();
    } else {
      EXPECT_EQ(solution.status().domain(), Code::InvalidArgument);
      EXPECT_EQ(solution.status().message(),
                "bundle_adjust: FAR and NEAR share 1 capture in which both "
                "keep a view, fewer than min_shared (3)");
    }
  }
}

TEST(BundleAdjust, HonoursMaxConditionAndMaxCaptures) {
  const Scene scene = make_scene(20);
  const InitialRig start = first_rig(scene);
  BundleAdjustOptions options;
  options.max_condition = 1.0 + 1e-9;
  core::Result<RigSolution> solution = adjust(scene, start, options);
  EXPECT_EQ(solution.status().domain(), Code::Numerical);
  EXPECT_NE(solution.status().message().find("camera SYN1 is poorly "
                                             "constrained"),
            std::string::npos)
      << solution.status().message();
  options.max_condition = 1e300;
  EXPECT_TRUE(adjust(scene, start, options).ok());

  options = {};
  options.max_captures = 19;
  solution = adjust(scene, start, options);
  EXPECT_EQ(solution.status().domain(), Code::InvalidArgument);
  EXPECT_NE(solution.status().message().find("20 captures"), std::string::npos)
      << solution.status().message();
  options.max_captures = 20;
  EXPECT_TRUE(adjust(scene, start, options).ok());
}

// The sensitivity is linear in the amounts it is asked for.
TEST(BundleAdjust, HonoursTheSensitivityAmounts) {
  const Scene scene = make_scene(20);
  const InitialRig start = first_rig(scene);
  const RigSolution base = solved(scene, start);
  BundleAdjustOptions options;
  options.focal_fraction = 0.006;
  options.principal_px = 0.5;
  const RigSolution other = solved(scene, start, options);
  for (std::size_t j = 0; j < 4; ++j) {
    for (std::size_t c = 1; c < 4; ++c) {
      const auto expect_scaled = [&](const PoseShift& a, const PoseShift& b,
                                     double scale) {
        EXPECT_NEAR(b.rotation_deg, scale * a.rotation_deg,
                    1e-9 * a.rotation_deg);
        EXPECT_LT(glm::length(b.centre_mm - scale * a.centre_mm),
                  1e-9 * glm::length(a.centre_mm));
      };
      expect_scaled(base.sensitivity[j].focal[c], other.sensitivity[j].focal[c],
                    2.0);
      expect_scaled(base.sensitivity[j].principal_x[c],
                    other.sensitivity[j].principal_x[c], 0.25);
      expect_scaled(base.sensitivity[j].principal_y[c],
                    other.sensitivity[j].principal_y[c], 0.25);
    }
    EXPECT_EQ(base.sensitivity[j].focal[0].rotation_deg, 0.0);  // reference
  }
}

// Finite sensitivity amounts may overflow the linear prediction or its
// rotation norm. Such diagnostics must not escape in a successful result.
TEST(BundleAdjust, NonFiniteSensitivityIsNumerical) {
  const Scene scene = make_scene(20);
  const InitialRig start = first_rig(scene);
  for (const double amount : {1e160, 1e308}) {
    SCOPED_TRACE(amount);
    for (const bool focal : {false, true}) {
      SCOPED_TRACE(focal ? "focal_fraction" : "principal_px");
      BundleAdjustOptions options;
      if (focal) {
        options.focal_fraction = amount;
      } else {
        options.principal_px = amount;
      }
      const core::Result<RigSolution> solution = adjust(scene, start, options);
      EXPECT_EQ(solution.status().domain(), Code::Numerical);
      EXPECT_NE(solution.status().message().find("sensitivity"),
                std::string::npos)
          << solution.status().message();
      EXPECT_NE(solution.status().message().find("not finite"),
                std::string::npos)
          << solution.status().message();
    }
  }
}

TEST(BundleAdjust, RefusesWhatItCannotSolve) {
  const Scene scene = make_scene(10);
  const InitialRig start = first_rig(scene);
  struct Case {
    const char* name;
    std::vector<RigCamera> cameras;
    std::vector<Observation> views;
    InitialRig start;
    const char* says;
  };
  std::vector<Case> cases;
  const auto add = [&](const char* name, const char* says) -> Case& {
    return cases.emplace_back(
        Case{name, scene.cameras, scene.views, start, says});
  };
  add("no cameras", "no cameras").cameras.clear();
  add("an empty id", "has no id").cameras[1].id.clear();
  add("a repeated id", "appears twice").cameras[2].id = "SYN0";
  add("a bad lens", "camera SYN3: cv_camera: ").cameras[3].model.intrinsics.fx =
      0.0;
  add("a reference outside", "start.reference 4").start.reference = 4;
  add("a camera short", "places 3 cameras").start.world_to_camera.pop_back();
  add("a reference moved", "not exactly at the identity")
      .start.world_to_camera[0][3][0] = 1e-12;
  add("a camera not rigid", "start's camera SYN1")
      .start.world_to_camera[1][0][0] = 2.0;
  add("a record short", "view records").start.views.pop_back();
  add("a record of another capture", "not of its camera")
      .start.views[3]
      .capture += 1000;
  add("a link to no camera", "link to no camera").start.links[0].parent = 9;
  add("two boards of a capture", "two boards")
      .start.boards.push_back(start.boards[0]);
  {
    Case& c = add("a board missing", "places no board");
    c.start.boards.erase(c.start.boards.begin());
  }
  add("a board not rigid", "start's board")
      .start.boards[0]
      .board_to_world[1][1] = 3.0;
  add("a view unusable", "fixes no pose").views[2].points.resize(3);
  add("a pixel not a number", "fixes no pose").views[2].points[4].pixel.y =
      std::numeric_limits<double>::quiet_NaN();
  add("a negative weight", "weight").views[5].points[1].weight = -1.0;
  add("a weight not finite", "weight").views[5].points[1].weight =
      std::numeric_limits<double>::quiet_NaN();
  {
    Case& c = add("two views of a capture", "two views");
    c.views.push_back(c.views[0]);
    c.start.views.push_back(c.start.views[0]);
  }
  {
    Case& c = add("no view used", "uses no view");
    for (ViewRecord& record : c.start.views) record.fate = ViewFate::PoorFit;
  }
  for (const Case& c : cases) {
    core::Result<RigSolution> solution =
        core::Status::invalid_argument("unset");
    EXPECT_NO_THROW(
        solution = bundle_adjust(c.cameras, scene.board, c.views, c.start, {}));
    EXPECT_EQ(solution.status().domain(), Code::InvalidArgument) << c.name;
    EXPECT_EQ(solution.status().message().rfind("bundle_adjust: ", 0), 0u)
        << c.name << ": " << solution.status().message();
    EXPECT_NE(solution.status().message().find(c.says), std::string::npos)
        << c.name << ": " << solution.status().message();
  }
}

TEST(BundleAdjust, RefusesOptionsOutOfRange) {
  const Scene scene = make_scene(10);
  const InitialRig start = first_rig(scene);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const struct {
    const char* field;
    void (*set)(BundleAdjustOptions&, double);
    std::vector<double> refused;
  } cases[] = {
      {"huber_px",
       [](BundleAdjustOptions& o, double v) { o.huber_px = v; },
       {0.0, -1.0, nan, inf}},
      {"min_depth_m",
       [](BundleAdjustOptions& o, double v) { o.min_depth_m = v; },
       {0.0, nan, inf}},
      {"max_iterations",
       [](BundleAdjustOptions& o, double v) {
         o.max_iterations = static_cast<int>(v);
       },
       {0.0, -1.0}},
      {"outlier_rounds",
       [](BundleAdjustOptions& o, double v) {
         o.outlier_rounds = static_cast<int>(v);
       },
       {-1.0}},
      {"outlier_px",
       [](BundleAdjustOptions& o, double v) { o.outlier_px = v; },
       {0.0, nan, inf}},
      {"outlier_mads",
       [](BundleAdjustOptions& o, double v) { o.outlier_mads = v; },
       {0.0, nan, inf}},
      {"outlier_view_ratio",
       [](BundleAdjustOptions& o, double v) { o.outlier_view_ratio = v; },
       {0.0, nan, inf}},
      {"outlier_view_px",
       [](BundleAdjustOptions& o, double v) { o.outlier_view_px = v; },
       {0.0, nan, inf}},
      {"min_shared",
       [](BundleAdjustOptions& o, double v) {
         o.min_shared = static_cast<int>(v);
       },
       {0.0}},
      {"max_condition",
       [](BundleAdjustOptions& o, double v) { o.max_condition = v; },
       {1.0, nan}},
      {"max_captures",
       [](BundleAdjustOptions& o, double v) {
         o.max_captures = static_cast<std::size_t>(v);
       },
       {0.0}},
      {"focal_fraction",
       [](BundleAdjustOptions& o, double v) { o.focal_fraction = v; },
       {0.0, nan, inf}},
      {"principal_px",
       [](BundleAdjustOptions& o, double v) { o.principal_px = v; },
       {0.0, nan, inf}},
  };
  for (const auto& c : cases) {
    for (const double value : c.refused) {
      BundleAdjustOptions options;
      c.set(options, value);
      const core::Result<RigSolution> solution = adjust(scene, start, options);
      EXPECT_EQ(solution.status().domain(), Code::InvalidArgument)
          << c.field << " = " << value;
      EXPECT_EQ(solution.status().message().rfind(
                    std::string("bundle_adjust: options.") + c.field + " ", 0),
                0u)
          << solution.status().message();
      // The same check on its own, with the caller's prefix.
      const core::Status checked =
          check_bundle_adjust_options(options, "app: adjust.");
      EXPECT_EQ(checked.domain(), Code::InvalidArgument);
      EXPECT_EQ(checked.message().rfind(
                    std::string("app: adjust.") + c.field + " must be ", 0),
                0u)
          << checked.message();
    }
  }
  EXPECT_TRUE(check_bundle_adjust_options({}).ok());
}

// The views the adjustment takes: those initialise used or found ambiguous.
TEST(BundleAdjust, AdjustsTheViewsUsedOrAmbiguous) {
  ViewRecord record;
  for (const auto& [fate, adjusted] :
       {std::pair{ViewFate::Used, true}, std::pair{ViewFate::Ambiguous, true},
        std::pair{ViewFate::NotUsable, false},
        std::pair{ViewFate::NoPose, false},
        std::pair{ViewFate::PoorFit, false}}) {
    record.fate = fate;
    EXPECT_EQ(adjusts(record), adjusted) << to_string(fate);
  }
}

TEST(BundleAdjust, NamesEachReason) {
  EXPECT_STREQ(to_string(Dropped::TooNear), "too_near");
  EXPECT_STREQ(to_string(Dropped::Outlier), "outlier");
  EXPECT_STREQ(to_string(Dropped::OutlierView), "outlier_view");
  EXPECT_STREQ(to_string(Dropped::Unusable), "unusable");
  EXPECT_STREQ(to_string(static_cast<Dropped>(9)), "unknown");
}

}  // namespace
}  // namespace volumetric_kit::calib::solve
