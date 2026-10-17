// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The solved rig's metrics on the synthetic rig: zero on exact data, the
// legacy RMSE, the groups of errors, the transfer against verify_rig.py's
// definition, leave one out against re-solves run here, flagging a bad
// capture and recording refused ones, the principal-point A/B test both ways
// and against a re-solve run here, its verdict, the geometry, the views left
// out, the rejections, and refusals.

#include "volumetric_kit/calib/quality/calibration_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/geometry.hpp>

#include "result_checks.hpp"
#include "synthetic_scene.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/cv_pose.hpp"
#include "volumetric_kit/calib/compute/usable_for_pose.hpp"
#include "volumetric_kit/calib/solve/view_pose.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;
using Code = core::Status::Code;
using solve::BundleAdjustOptions;
using solve::InitialRig;
using solve::RigSolution;
using test::index_of;
using test::make_scene;
using test::Scene;
using test::value_of;

const double kInfinity = std::numeric_limits<double>::infinity();

// The first rig, or the test fails here: GoogleTest reports the exception
// and goes on to the next test.
InitialRig first_rig(const Scene& scene,
                     const solve::InitialiseOptions& options = {}) {
  core::Result<InitialRig> rig =
      solve::initialise(scene.cameras, scene.board, scene.views, 0, options);
  if (!rig.ok()) throw std::runtime_error(rig.status().message());
  return *std::move(rig);
}

RigSolution solved(const Scene& scene, const InitialRig& start,
                   const BundleAdjustOptions& options = {}) {
  core::Result<RigSolution> solution = solve::bundle_adjust(
      scene.cameras, scene.board, scene.views, start, options);
  if (!solution.ok()) throw std::runtime_error(solution.status().message());
  return *std::move(solution);
}

CalibrationMetrics measured(const Scene& scene, const InitialRig& start,
                            const RigSolution& solution,
                            const MetricsOptions& options = {}) {
  core::Result<CalibrationMetrics> metrics =
      core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(metrics = calibration_metrics(scene.cameras, scene.board,
                                                scene.views, start, solution,
                                                options));
  return value_of(std::move(metrics));
}

// numpy's percentile, linear: the definition the metrics follow.
double percentile(std::vector<double> values, double q) {
  std::sort(values.begin(), values.end());
  const double at = q * static_cast<double>(values.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(at));
  const std::size_t hi = std::min(lo + 1, values.size() - 1);
  return values[lo] +
         (at - static_cast<double>(lo)) * (values[hi] - values[lo]);
}

// The kept corners' residuals at the solution.
std::vector<camera::Vec2d> kept_residuals(const RigSolution& solution) {
  std::vector<camera::Vec2d> out;
  for (const solve::CornerResidual& r : solution.residuals) {
    if (!r.dropped) out.push_back(r.residual);
  }
  return out;
}

double rms_of(const std::vector<camera::Vec2d>& residuals) {
  double squared = 0.0;
  for (const camera::Vec2d& e : residuals) squared += e.x * e.x + e.y * e.y;
  return std::sqrt(squared / static_cast<double>(residuals.size()));
}

// Where a re-solve starts: the solution's cameras, the reference exactly at
// the identity, and its boards (the start's for a capture it has none of),
// with the start's tree and view records.
InitialRig warm_from(const InitialRig& start, const RigSolution& solution) {
  InitialRig warm;
  warm.reference = solution.reference;
  for (std::size_t c = 0; c < solution.world_to_camera.size(); ++c) {
    warm.world_to_camera.push_back(
        c == solution.reference
            ? camera::Mat4d(1.0)
            : camera::matrix_from_rodrigues(solution.world_to_camera[c]));
  }
  std::map<std::int64_t, camera::Mat4d> boards;
  for (const solve::SolvedBoard& b : solution.boards) {
    boards[b.capture] = camera::matrix_from_rodrigues(b.board_to_world);
  }
  for (const solve::BoardPose& b : start.boards) {
    const auto it = boards.find(b.capture);
    warm.boards.push_back(
        {b.capture, it != boards.end() ? it->second : b.board_to_world});
  }
  warm.links = start.links;
  warm.views = start.views;
  return warm;
}

TEST(CalibrationMetrics, ExactDataGivesZeroRmsTransferAndLeaveOneOut) {
  const Scene scene = make_scene(12, 0.0);
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  const CalibrationMetrics m = measured(scene, start, solution);
  const ReprojectionStats& s = m.reprojection.solved;
  std::size_t corners = 0;
  for (const Observation& view : scene.views) corners += view.points.size();
  EXPECT_EQ(s.overall.count, corners);
  EXPECT_LT(s.overall.rms_px, 1e-6);
  EXPECT_LT(s.overall.max_px, 1e-6);
  EXPECT_LT(m.reprojection.initial.overall.rms_px, 1e-6);
  ASSERT_FALSE(m.transfer.errors.empty());
  EXPECT_LT(m.transfer.worst_px, 1e-6);
  ASSERT_EQ(m.leave_one_out.captures.size(), scene.plan.size());
  for (const LeftOut& left : m.leave_one_out.captures) {
    EXPECT_TRUE(left.refused.empty()) << left.refused;
  }
  for (const CameraStability& c : m.leave_one_out.cameras) {
    EXPECT_LT(c.centre_mm, 1e-6);
    EXPECT_LT(c.rotation_deg, 1e-6);
  }
  EXPECT_TRUE(m.rejections.empty());
}

// The legacy RMSE is the root of the mean squared residual component, as the
// legacy solve computes it from its residual vector, at the first rig and at
// the solution, over the same corners; it is the RMS over the root of 2.
TEST(CalibrationMetrics, LegacyRmseIsTheRmsOverTheRootOfTwo) {
  const Scene scene = make_scene(15);
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  const CalibrationMetrics m = measured(scene, start, solution);

  const auto legacy = [](const std::vector<camera::Vec2d>& residuals) {
    std::vector<double> components;
    for (const camera::Vec2d& e : residuals) {
      components.push_back(e.x);
      components.push_back(e.y);
    }
    double sum = 0.0;
    for (const double c : components) sum += c * c;
    return std::sqrt(sum / static_cast<double>(components.size()));
  };
  // The first rig's residuals, projected afresh.
  std::map<std::int64_t, camera::Mat4d> boards;
  for (const solve::BoardPose& b : start.boards) {
    boards[b.capture] = b.board_to_world;
  }
  std::vector<camera::Vec2d> initial;
  std::size_t r = 0;
  for (std::size_t i = 0; i < scene.views.size(); ++i) {
    if (start.views[i].fate != solve::ViewFate::Used &&
        start.views[i].fate != solve::ViewFate::Ambiguous) {
      continue;
    }
    const Observation& view = scene.views[i];
    const std::size_t c = start.views[i].camera;
    const std::vector<cv::Point2d> projected =
        test::project(scene.cameras[c].model,
                      start.world_to_camera[c] * boards.at(view.frame_id), [&] {
                        std::vector<cv::Point3d> x;
                        for (const DetectedPoint& p : view.points) {
                          x.push_back(scene.board.object_points()[p.id]);
                        }
                        return x;
                      }());
    for (std::size_t j = 0; j < view.points.size(); ++j, ++r) {
      if (solution.residuals[r].dropped) continue;
      initial.push_back({projected[j].x - view.points[j].pixel.x,
                         projected[j].y - view.points[j].pixel.y});
    }
  }
  const std::vector<camera::Vec2d> final_residuals = kept_residuals(solution);
  ASSERT_EQ(initial.size(), final_residuals.size());
  const ReprojectionStats& before = m.reprojection.initial;
  const ReprojectionStats& after = m.reprojection.solved;
  EXPECT_NEAR(after.legacy_rmse_px, legacy(final_residuals),
              1e-12 * after.legacy_rmse_px);
  EXPECT_NEAR(before.legacy_rmse_px, legacy(initial),
              1e-9 * before.legacy_rmse_px);
  EXPECT_NEAR(after.legacy_rmse_px, after.overall.rms_px / std::sqrt(2.0),
              1e-12 * after.legacy_rmse_px);
  EXPECT_NEAR(before.legacy_rmse_px, before.overall.rms_px / std::sqrt(2.0),
              1e-12 * before.legacy_rmse_px);
  EXPECT_LT(after.legacy_rmse_px, before.legacy_rmse_px);
  EXPECT_NEAR(m.reprojection.legacy_reduction_percent,
              100.0 * (1.0 - after.legacy_rmse_px / before.legacy_rmse_px),
              1e-12);
  EXPECT_GT(m.reprojection.legacy_reduction_percent, 0.0);
}

// Every group holds the corners it names: per camera, per capture and per
// view, with and without the oblique views; each group's RMS, p90 (numpy's),
// largest and mean from its own corners; and the histogram counts them all.
TEST(CalibrationMetrics, GroupsTheErrors) {
  const Scene scene = make_scene(15);
  solve::InitialiseOptions flag;
  flag.oblique_deg = 35.0;  // some views oblique
  const InitialRig start = first_rig(scene, flag);
  const RigSolution solution = solved(scene, start);
  MetricsOptions options;
  options.initialise = flag;
  options.histogram_bin_px = 0.05;
  options.histogram_bins = 12;
  const CalibrationMetrics m = measured(scene, start, solution, options);
  const ReprojectionStats& s = m.reprojection.solved;
  // The settings, as measured with.
  EXPECT_EQ(m.options.initialise.oblique_deg, 35.0);
  EXPECT_EQ(m.options.histogram_bin_px, 0.05);
  EXPECT_EQ(m.options.histogram_bins, 12);

  std::map<std::size_t, std::vector<camera::Vec2d>> by_camera;
  std::map<std::int64_t, std::vector<camera::Vec2d>> by_capture;
  std::vector<camera::Vec2d> all;
  std::vector<camera::Vec2d> not_oblique;
  std::size_t oblique_views = 0;
  std::size_t r = 0;
  std::size_t v = 0;
  for (std::size_t i = 0; i < scene.views.size(); ++i) {
    const solve::ViewRecord& record = start.views[i];
    if (record.fate != solve::ViewFate::Used &&
        record.fate != solve::ViewFate::Ambiguous) {
      continue;
    }
    ASSERT_LT(v, s.views.size());
    const ViewErrors& view = s.views[v++];
    EXPECT_EQ(view.camera, record.camera);
    EXPECT_EQ(view.capture, record.capture);
    EXPECT_EQ(view.oblique, record.oblique);
    oblique_views += record.oblique ? 1 : 0;
    std::vector<camera::Vec2d> mine;
    for (std::size_t j = 0; j < scene.views[i].points.size(); ++j, ++r) {
      const solve::CornerResidual& c = solution.residuals[r];
      if (c.dropped) continue;
      mine.push_back(c.residual);
      all.push_back(c.residual);
      if (!record.oblique) not_oblique.push_back(c.residual);
      by_camera[c.camera].push_back(c.residual);
      by_capture[c.capture].push_back(c.residual);
    }
    EXPECT_EQ(view.errors.count, mine.size());
  }
  EXPECT_EQ(v, s.views.size());
  ASSERT_GT(oblique_views, 0u);
  ASSERT_LT(not_oblique.size(), all.size());

  const auto expect_stats = [](const ErrorStats& got,
                               const std::vector<camera::Vec2d>& e) {
    ASSERT_EQ(got.count, e.size());
    std::vector<double> norms;
    double squared = 0.0;
    camera::Vec2d sum(0.0);
    for (const camera::Vec2d& x : e) {
      norms.push_back(std::hypot(x.x, x.y));
      squared += x.x * x.x + x.y * x.y;
      sum += x;
    }
    const double n = static_cast<double>(e.size());
    EXPECT_NEAR(got.rms_px, std::sqrt(squared / n), 1e-12);
    EXPECT_NEAR(got.p90_px, percentile(norms, 0.9), 1e-12);
    EXPECT_NEAR(got.max_px, *std::max_element(norms.begin(), norms.end()),
                1e-12);
    EXPECT_NEAR(got.mean_px.x, sum.x / n, 1e-12);
    EXPECT_NEAR(got.mean_px.y, sum.y / n, 1e-12);
  };
  expect_stats(s.overall, all);
  expect_stats(s.without_oblique, not_oblique);
  ASSERT_EQ(s.cameras.size(), scene.cameras.size());
  for (std::size_t c = 0; c < scene.cameras.size(); ++c) {
    expect_stats(s.cameras[c], by_camera[c]);
  }
  ASSERT_EQ(s.captures.size(), by_capture.size());
  std::size_t at = 0;
  for (const auto& [capture, e] : by_capture) {
    EXPECT_EQ(s.captures[at].capture, capture);  // ascending
    expect_stats(s.captures[at++].errors, e);
  }

  // The histogram: 12 bins of 0.05 px, then everything beyond.
  ASSERT_EQ(s.histogram.counts.size(), 13u);
  EXPECT_EQ(s.histogram.bin_px, 0.05);
  std::vector<std::size_t> counts(13, 0);
  for (const camera::Vec2d& e : all) {
    const double norm = std::hypot(e.x, e.y);
    ++counts[std::min<std::size_t>(static_cast<std::size_t>(norm / 0.05), 12)];
  }
  EXPECT_EQ(s.histogram.counts, counts);
  EXPECT_GT(counts.back(), 0u);
  EXPECT_EQ(m.reprojection.initial.histogram.counts.size(), 13u);
}

// verify_rig.py, in OpenCV directly: each view's board by solvePnP (IPPE)
// with the factory lens, carried through the poses into every other camera
// that sees it, projected there, and the mean distance to its corners.
TEST(CrossCameraTransfer, IsVerifyRigsCheck) {
  const Scene scene = make_scene(15);
  // Poses a little off, so the errors are more than the noise's.
  std::vector<camera::Mat4d> truth = test::truths_in_world_of(scene, 0);
  truth[2] = truth[2] * glm::dmat4(glm::rotate(glm::dmat4(1.0), 0.004,
                                               glm::dvec3(0.3, 1.0, 0.1)));
  truth[3][3] += glm::dvec4(0.004, 0.0, -0.002, 0.0);
  std::vector<camera::RodriguesTransform> poses;
  for (const camera::Mat4d& t : truth) {
    poses.push_back(camera::rodrigues_from_matrix(t));
  }
  const Transfer t = value_of(
      cross_camera_transfer(scene.cameras, scene.board, scene.views, poses));

  // The legacy check.
  const std::size_t k = scene.cameras.size();
  std::map<std::int64_t, std::map<std::size_t, const Observation*>> sets;
  for (const Observation& view : scene.views) {
    sets[view.frame_id][index_of(scene.cameras, view.camera_id)] = &view;
  }
  std::vector<std::tuple<std::int64_t, std::size_t, std::size_t, double>> want;
  for (const auto& [capture, views] : sets) {
    std::map<std::size_t, cv::Matx44d> board_in;
    for (const auto& [c, view] : views) {
      if (view->points.size() < 4) continue;
      std::vector<cv::Point3d> object;
      std::vector<cv::Point2d> pixels;
      for (const DetectedPoint& p : view->points) {
        object.push_back(scene.board.object_points()[p.id]);
        pixels.emplace_back(p.pixel.x, p.pixel.y);
      }
      const compute::CvCamera lens =
          compute::cv_camera(scene.cameras[c].model).value();
      cv::Vec3d rvec;
      cv::Vec3d tvec;
      ASSERT_TRUE(cv::solvePnP(object, pixels, lens.camera_matrix,
                               lens.distortion, rvec, tvec, false,
                               cv::SOLVEPNP_IPPE));
      board_in[c] = compute::cv_matrix(
          compute::matrix_from_cv_pose(compute::CvPose{rvec, tvec}));
    }
    for (const auto& [a, a_board] : board_in) {
      for (const auto& [b, b_board] : board_in) {
        if (a == b) continue;
        const cv::Matx44d carried = compute::cv_matrix(truth[b]) *
                                    compute::cv_matrix(truth[a]).inv() *
                                    a_board;
        const Observation& view = *views.at(b);
        std::vector<cv::Point3d> in_b;
        std::vector<cv::Point2d> pixels;
        for (const DetectedPoint& p : view.points) {
          const cv::Point3d& x = scene.board.object_points()[p.id];
          const cv::Vec4d y = carried * cv::Vec4d(x.x, x.y, x.z, 1.0);
          in_b.emplace_back(y[0], y[1], y[2]);
        }
        const compute::CvCamera lens =
            compute::cv_camera(scene.cameras[b].model).value();
        cv::projectPoints(in_b, cv::Vec3d(), cv::Vec3d(), lens.camera_matrix,
                          lens.distortion, pixels);
        double sum = 0.0;
        for (std::size_t j = 0; j < pixels.size(); ++j) {
          sum += std::hypot(pixels[j].x - view.points[j].pixel.x,
                            pixels[j].y - view.points[j].pixel.y);
        }
        want.emplace_back(capture, a, b,
                          sum / static_cast<double>(pixels.size()));
      }
    }
  }
  ASSERT_EQ(t.errors.size(), want.size());
  std::vector<double> errors;
  std::map<std::pair<std::size_t, std::size_t>, std::vector<double>> by_pair;
  for (std::size_t i = 0; i < want.size(); ++i) {
    const auto& [capture, a, b, e] = want[i];
    EXPECT_EQ(t.errors[i].capture, capture);
    EXPECT_EQ(t.errors[i].from, a);
    EXPECT_EQ(t.errors[i].to, b);
    EXPECT_NEAR(t.errors[i].error_px, e, 1e-6) << capture << " " << a << b;
    errors.push_back(e);
    by_pair[{a, b}].push_back(e);
  }
  EXPECT_GT(percentile(errors, 0.5), 1.0);  // the poses are off
  EXPECT_NEAR(t.median_px, percentile(errors, 0.5), 1e-6);
  EXPECT_NEAR(t.p90_px, percentile(errors, 0.9), 1e-6);
  EXPECT_NEAR(t.worst_px, *std::max_element(errors.begin(), errors.end()),
              1e-6);
  ASSERT_EQ(t.pairs.size(), k);
  for (std::size_t a = 0; a < k; ++a) {
    ASSERT_EQ(t.pairs[a].size(), k);
    for (std::size_t b = 0; b < k; ++b) {
      const auto it = by_pair.find({a, b});
      if (it == by_pair.end()) {
        EXPECT_EQ(t.pairs[a][b].count, 0u);
        EXPECT_TRUE(std::isnan(t.pairs[a][b].mean_px));
        continue;
      }
      double sum = 0.0;
      for (const double e : it->second) sum += e;
      EXPECT_EQ(t.pairs[a][b].count, it->second.size());
      EXPECT_NEAR(t.pairs[a][b].mean_px,
                  sum / static_cast<double>(it->second.size()), 1e-6);
    }
  }
}

// A board carried behind a camera errs infinitely; a view that fixes no
// pose poses nothing, and is not carried into.
TEST(CrossCameraTransfer, BehindACameraIsInfiniteAndAnUnusableViewIsSkipped) {
  Scene scene = make_scene(6);
  std::vector<camera::RodriguesTransform> poses;
  for (const camera::Mat4d& t : test::truths_in_world_of(scene, 0)) {
    poses.push_back(camera::rodrigues_from_matrix(t));
  }
  // Camera 1 turned to face away.
  camera::Mat4d away = camera::matrix_from_rodrigues(poses[1]);
  away = glm::dmat4(glm::rotate(glm::dmat4(1.0), glm::pi<double>(),
                                glm::dvec3(0.0, 1.0, 0.0))) *
         away;
  poses[1] = camera::rodrigues_from_matrix(away);
  // One view's corners on a line: it fixes no pose.
  std::size_t line = 0;
  for (; line < scene.views.size(); ++line) {
    if (scene.views[line].camera_id == scene.cameras[2].id) break;
  }
  ASSERT_LT(line, scene.views.size());
  std::vector<DetectedPoint> row;
  for (const DetectedPoint& p : scene.views[line].points) {
    if (p.id < 4) row.push_back(p);
  }
  scene.views[line].points = row;
  scene.views[line].points[0].pixel.x =
      std::numeric_limits<double>::quiet_NaN();

  core::Result<Transfer> t = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(t = cross_camera_transfer(scene.cameras, scene.board,
                                            scene.views, poses));
  ASSERT_TRUE(t.ok()) << t.status().message();
  bool into_one = false;
  for (const TransferError& e : t->errors) {
    if (e.to == 1) {
      into_one = true;
      EXPECT_EQ(e.error_px, kInfinity);
    } else if (e.from != 1) {
      EXPECT_LT(e.error_px, 20.0);  // the views' own poses' error
    }
    EXPECT_FALSE(e.capture == scene.views[line].frame_id &&
                 (e.from == 2 || e.to == 2));
  }
  EXPECT_TRUE(into_one);
  // A quarter of the errors are infinite: the 90th percentile falls between
  // two of them.
  EXPECT_EQ(t->p90_px, kInfinity);
  EXPECT_EQ(t->worst_px, kInfinity);
  EXPECT_EQ(t->pairs[0][1].mean_px, kInfinity);
}

// A usable view whose every IPPE pose puts a corner behind its camera (a
// board straddling the camera, seen through a pinhole) poses nothing: its
// capture's pairs are skipped, not an error.
TEST(CrossCameraTransfer, SkipsAViewWithNoPoseInFront) {
  const compute::Board board = test::femto_board();
  const camera::CameraModel pinhole{
      {3840, 2160}, {2240.0, 2240.0, 1919.5, 1079.5}, {}};
  const std::vector<solve::RigCamera> cameras = {{"A", pinhole},
                                                 {"B", pinhole}};
  camera::Mat4d to_b(1.0);  // B 0.3 m right of A, the world
  to_b[3] = glm::dvec4(-0.3, 0.0, 0.0, 1.0);
  const std::vector<camera::RodriguesTransform> poses = {
      camera::rodrigues_from_matrix(camera::Mat4d(1.0)),
      camera::rodrigues_from_matrix(to_b)};
  const auto view_of = [&](const char* id, std::int64_t capture,
                           const camera::Mat4d& board_to_camera,
                           const std::vector<int>& ids) {
    const std::vector<cv::Point2d> pixels =
        test::project(pinhole, board_to_camera, board.object_points());
    Observation view;
    view.camera_id = id;
    view.frame_id = capture;
    for (const int i : ids) {
      const cv::Point2d& p = pixels[static_cast<std::size_t>(i)];
      view.points.push_back({i, {p.x, p.y}, 1.0});
    }
    return view;
  };
  const std::vector<int> all = {0, 1, 2, 3, 4, 5, 6, 7};
  const camera::Mat4d in_front =
      test::board_facing_camera(board, {0.0, 0.0, 2.5}, 20.0, 10.0);
  // The board's x along the camera's -z, its y along the camera's x.
  camera::Mat4d straddling(0.0);
  straddling[0] = glm::dvec4(0.0, 0.0, -1.0, 0.0);
  straddling[1] = glm::dvec4(1.0, 0.0, 0.0, 0.0);
  straddling[2] = glm::dvec4(0.0, -1.0, 0.0, 0.0);
  straddling[3] = glm::dvec4(-0.225, 0.05, 0.3, 1.0);
  const std::vector<Observation> views = {
      view_of("A", 1, in_front, all), view_of("B", 1, to_b * in_front, all),
      view_of("A", 2, straddling, {0, 3, 4, 7}),
      view_of("B", 2, to_b * in_front, all)};
  const compute::CvCamera lens = compute::cv_camera(pinhole).value();
  ASSERT_EQ(compute::usable_for_pose(board, views[2], lens.size).value(),
            compute::ViewUse::Usable);
  ASSERT_EQ(solve::view_pose(lens, board, views[2]).status().domain(),
            Code::Numerical);

  core::Result<Transfer> t = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(t = cross_camera_transfer(cameras, board, views, poses));
  ASSERT_TRUE(t.ok()) << t.status().message();
  ASSERT_EQ(t->errors.size(), 2u);
  for (const TransferError& e : t->errors) {
    EXPECT_EQ(e.capture, 1);
    EXPECT_LT(e.error_px, 1e-6);
  }
  EXPECT_EQ(t->pairs[0][1].count, 1u);
  EXPECT_EQ(t->pairs[1][0].count, 1u);
}

TEST(CrossCameraTransfer, RefusesWhatItCannotCheck) {
  const Scene scene = make_scene(4);
  std::vector<camera::RodriguesTransform> poses(scene.cameras.size());
  for (std::size_t c = 0; c < poses.size(); ++c) {
    poses[c] =
        camera::rodrigues_from_matrix(test::truths_in_world_of(scene, 0)[c]);
  }
  const auto refused = [&](const std::vector<solve::RigCamera>& cameras,
                           const std::vector<Observation>& views,
                           const std::vector<camera::RodriguesTransform>& p,
                           const std::string& says) {
    core::Result<Transfer> t = Transfer{};
    EXPECT_NO_THROW(t = cross_camera_transfer(cameras, scene.board, views, p));
    EXPECT_EQ(t.status().domain(), Code::InvalidArgument) << says;
    EXPECT_EQ(t.status().message().rfind("cross_camera_transfer: ", 0), 0u)
        << t.status().message();
    EXPECT_NE(t.status().message().find(says), std::string::npos)
        << t.status().message();
  };
  refused({}, scene.views, {}, "no cameras");
  refused(scene.cameras, scene.views, {poses.begin(), poses.end() - 1},
          "3 poses for 4 cameras");
  std::vector<camera::RodriguesTransform> nan = poses;
  nan[2].tvec.y = std::numeric_limits<double>::quiet_NaN();
  refused(scene.cameras, scene.views, nan, "not finite");
  std::vector<Observation> stranger = scene.views;
  stranger[1].camera_id = "nobody";
  refused(scene.cameras, stranger, poses, "nobody");
  std::vector<Observation> twice = scene.views;
  twice.push_back(scene.views[0]);
  refused(scene.cameras, twice, poses, "two views");
}

// A capture with one camera's corners 1.1 px off -- too little for the
// outlier rounds to drop, enough to pull the camera -- moves that camera the
// most when left out, more than twice as far as any other capture.
TEST(LeaveOneOut, FlagsABadCapture) {
  Scene scene = make_scene(24, 0.05);
  const std::size_t c = 2;
  std::int64_t bad = -1;
  for (Observation& view : scene.views) {
    if (view.camera_id != scene.cameras[c].id || view.frame_id < 5) continue;
    bad = view.frame_id;
    for (DetectedPoint& p : view.points) p.pixel += camera::Vec2d(1.0, -0.5);
    break;
  }
  ASSERT_GE(bad, 0);
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  for (const solve::Rejection& r : solution.rejections) {
    EXPECT_FALSE(r.camera == c && r.capture == bad) << "the bad view went";
  }
  core::Result<LeaveOneOut> loo = LeaveOneOut{};
  EXPECT_NO_THROW(loo = leave_one_out(scene.cameras, scene.board, scene.views,
                                      start, solution));
  ASSERT_TRUE(loo.ok()) << loo.status().message();
  const CameraStability& moved = loo->cameras[c];
  ASSERT_TRUE(moved.centre_capture.has_value());
  EXPECT_EQ(*moved.centre_capture, bad);
  ASSERT_TRUE(moved.rotation_capture.has_value());
  EXPECT_EQ(*moved.rotation_capture, bad);
  double next_centre = 0.0;
  double next_rotation = 0.0;
  for (const LeftOut& left : loo->captures) {
    ASSERT_EQ(left.cameras.size(), scene.cameras.size()) << left.refused;
    if (left.capture == bad) continue;
    next_centre = std::max(next_centre, left.cameras[c].centre_mm);
    next_rotation = std::max(next_rotation, left.cameras[c].rotation_deg);
  }
  EXPECT_GT(moved.centre_mm, 2.0 * next_centre);
  EXPECT_GT(moved.rotation_deg, 2.0 * next_rotation);
  // The reference never moves.
  EXPECT_EQ(loo->cameras[0].centre_mm, 0.0);
  EXPECT_FALSE(loo->cameras[0].centre_capture.has_value());
  EXPECT_FALSE(loo->cameras[0].rotation_capture.has_value());
  // The largest of each camera's moves.
  for (std::size_t cam = 0; cam < scene.cameras.size(); ++cam) {
    double most = 0.0;
    for (const LeftOut& left : loo->captures) {
      most = std::max(most, left.cameras[cam].centre_mm);
    }
    EXPECT_EQ(loo->cameras[cam].centre_mm, most);
  }
}

// Each re-solve, run here as leave_one_out describes it, moves each camera's
// centre, -R^T t, by the millimetres reported, turns it by the degrees
// reported, and fits its kept corners by the RMS reported. On this rig a
// camera's centre and its translation move by clearly different amounts.
TEST(LeaveOneOut, EachMoveIsTheCentresAndTheRotations) {
  const Scene scene = make_scene(10);
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  const LeaveOneOut loo = value_of(
      leave_one_out(scene.cameras, scene.board, scene.views, start, solution));
  const InitialRig warm = warm_from(start, solution);
  const std::size_t k = scene.cameras.size();
  ASSERT_EQ(loo.captures.size(), solution.boards.size());
  double apart = 0.0;  // the most a translation's move and a centre's differ
  for (const LeftOut& left : loo.captures) {
    SCOPED_TRACE(left.capture);
    ASSERT_TRUE(left.refused.empty()) << left.refused;
    ASSERT_EQ(left.cameras.size(), k);
    std::vector<Observation> views;
    InitialRig less = warm;
    less.views.clear();
    less.boards.clear();
    for (std::size_t i = 0; i < scene.views.size(); ++i) {
      if (scene.views[i].frame_id == left.capture) continue;
      views.push_back(scene.views[i]);
      less.views.push_back(warm.views[i]);
    }
    for (const solve::BoardPose& b : warm.boards) {
      if (b.capture != left.capture) less.boards.push_back(b);
    }
    const RigSolution again =
        value_of(solve::bundle_adjust(scene.cameras, scene.board, views, less));
    EXPECT_NEAR(left.rms_px, rms_of(kept_residuals(again)), 1e-12);
    EXPECT_NEAR(left.legacy_rmse_px, left.rms_px / std::sqrt(2.0), 1e-12);
    for (std::size_t c = 0; c < k; ++c) {
      const camera::Mat4d before =
          camera::matrix_from_rodrigues(solution.world_to_camera[c]);
      const camera::Mat4d after =
          camera::matrix_from_rodrigues(again.world_to_camera[c]);
      const double centre_mm = 1000.0 * glm::length(test::centre_of(after) -
                                                    test::centre_of(before));
      EXPECT_NEAR(left.cameras[c].centre_mm, centre_mm, 1e-9) << c;
      EXPECT_NEAR(left.cameras[c].rotation_deg, test::angle_deg(before, after),
                  1e-9)
          << c;
      const double translation_mm =
          1000.0 * glm::length(camera::Vec3d(after[3] - before[3]));
      if (centre_mm > 0.01) {
        apart =
            std::max(apart, std::abs(translation_mm - centre_mm) / centre_mm);
      }
    }
  }
  EXPECT_GT(apart, 0.2);
}

// A capture without which a link shares fewer than min_shared captures is
// refused, naming why, and moves nothing; progress counts every re-solve.
TEST(LeaveOneOut, RecordsARefusedCaptureAndReportsProgress) {
  const Scene scene = make_scene(8);
  const InitialRig start = first_rig(scene);
  int fewest = std::numeric_limits<int>::max();
  for (const solve::PlacedLink& link : start.links) {
    fewest = std::min(fewest, link.shared);
  }
  BundleAdjustOptions options;
  options.min_shared = fewest;
  const RigSolution solution = solved(scene, start, options);
  std::vector<std::pair<std::size_t, std::size_t>> calls;
  core::Result<LeaveOneOut> loo = LeaveOneOut{};
  EXPECT_NO_THROW(loo = leave_one_out(scene.cameras, scene.board, scene.views,
                                      start, solution, options,
                                      [&](std::size_t done, std::size_t total) {
                                        calls.emplace_back(done, total);
                                      }));
  ASSERT_TRUE(loo.ok()) << loo.status().message();
  ASSERT_EQ(loo->captures.size(), solution.boards.size());
  std::size_t refused = 0;
  for (const LeftOut& left : loo->captures) {
    if (left.refused.empty()) {
      EXPECT_EQ(left.cameras.size(), scene.cameras.size());
      continue;
    }
    ++refused;
    EXPECT_TRUE(left.cameras.empty());
    EXPECT_NE(left.refused.find("min_shared"), std::string::npos)
        << left.refused;
  }
  EXPECT_GT(refused, 0u);
  EXPECT_LT(refused, loo->captures.size());
  ASSERT_EQ(calls.size(), loo->captures.size());
  for (std::size_t i = 0; i < calls.size(); ++i) {
    EXPECT_EQ(calls[i].first, i + 1);
    EXPECT_EQ(calls[i].second, calls.size());
  }
}

// A re-solve bundle_adjust refuses as Numerical -- every camera poorly
// constrained, under a condition bound just above 1 -- is refused too.
TEST(LeaveOneOut, RecordsANumericalRefusal) {
  const Scene scene = make_scene(6);
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  BundleAdjustOptions strict;
  strict.max_condition = 1.0 + 1e-9;
  core::Result<LeaveOneOut> loo = LeaveOneOut{};
  EXPECT_NO_THROW(loo = leave_one_out(scene.cameras, scene.board, scene.views,
                                      start, solution, strict));
  ASSERT_TRUE(loo.ok()) << loo.status().message();
  ASSERT_FALSE(loo->captures.empty());
  for (const LeftOut& left : loo->captures) {
    EXPECT_NE(left.refused.find("poorly constrained"), std::string::npos)
        << left.refused;
    EXPECT_TRUE(left.cameras.empty());
    EXPECT_TRUE(std::isnan(left.rms_px));
  }
  for (const CameraStability& c : loo->cameras) {
    EXPECT_FALSE(c.centre_capture.has_value());
  }
}

// What progress throws comes back as a Status.
TEST(LeaveOneOut, ReturnsWhatProgressThrows) {
  const Scene scene = make_scene(6);
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  core::Result<LeaveOneOut> loo = LeaveOneOut{};
  EXPECT_NO_THROW(
      loo = leave_one_out(
          scene.cameras, scene.board, scene.views, start, solution, {},
          [](std::size_t, std::size_t) { throw std::runtime_error("stop"); }));
  EXPECT_EQ(loo.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(loo.status().message(), "leave_one_out: stop");
}

// The truth's principal points 2.5 px above the lenses given: the A/B test
// prefers them shifted; and with the truth as given, it does not. With 0.1 px
// of noise, the transfer's median tells the sides apart whatever the draw; at
// 0.3 px it misses on about one in eight.
TEST(PrincipalPointAB, PrefersTheShiftWhenTheTruthHasIt) {
  for (const bool truth_shifted : {true, false}) {
    SCOPED_TRACE(truth_shifted);
    Scene scene = make_scene(20, 0.1);
    if (truth_shifted) {
      for (solve::RigCamera& cam : scene.cameras) {
        cam.model.intrinsics.cx -= 2.5;
        cam.model.intrinsics.cy -= 2.5;
      }
    }
    const InitialRig start = first_rig(scene);
    const RigSolution solution = solved(scene, start);
    core::Result<PrincipalPointAB> ab = PrincipalPointAB{};
    EXPECT_NO_THROW(ab = principal_point_ab(scene.cameras, scene.board,
                                            scene.views, start, solution));
    ASSERT_TRUE(ab.ok()) << ab.status().message();
    EXPECT_EQ(ab->shift_px, 2.5);
    EXPECT_GT(ab->corners, 0u);
    const PrincipalPointArm& near = truth_shifted ? ab->shifted : ab->as_given;
    const PrincipalPointArm& far = truth_shifted ? ab->as_given : ab->shifted;
    EXPECT_LT(near.rms_px, far.rms_px);
    EXPECT_LT(near.transfer_median_px, far.transfer_median_px);
    EXPECT_NEAR(near.legacy_rmse_px, near.rms_px / std::sqrt(2.0), 1e-12);
    EXPECT_EQ(ab->verdict, truth_shifted ? PrincipalPointVerdict::Shifted
                                         : PrincipalPointVerdict::AsGiven);
    // The given side's transfer is the solution's.
    const Transfer given = value_of(cross_camera_transfer(
        scene.cameras, scene.board, scene.views, solution.world_to_camera));
    EXPECT_EQ(ab->as_given.transfer_median_px, given.median_px);
    EXPECT_EQ(ab->as_given.transfer_p90_px, given.p90_px);
  }
}

// The sides' RMS is over the corners both adjustments kept: re-solved here
// as principal_point_ab describes it, with outlier rounds strict enough that
// the shifted solve drops corners the solution kept.
TEST(PrincipalPointAB, ComparesTheCornersBothKeep) {
  const Scene scene = make_scene(12);
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  BundleAdjustOptions strict;
  strict.outlier_px = 0.01;
  strict.outlier_mads = 2.5;
  core::Result<PrincipalPointAB> ab = PrincipalPointAB{};
  EXPECT_NO_THROW(ab =
                      principal_point_ab(scene.cameras, scene.board,
                                         scene.views, start, solution, strict));
  ASSERT_TRUE(ab.ok()) << ab.status().message();
  ASSERT_TRUE(ab->refused.empty()) << ab->refused;

  std::vector<solve::RigCamera> shifted = scene.cameras;
  for (solve::RigCamera& cam : shifted) {
    cam.model.intrinsics.cx += 2.5;
    cam.model.intrinsics.cy += 2.5;
  }
  const RigSolution again = value_of(solve::bundle_adjust(
      shifted, scene.board, scene.views, warm_from(start, solution), strict));
  ASSERT_EQ(again.residuals.size(), solution.residuals.size());
  std::vector<camera::Vec2d> given;
  std::vector<camera::Vec2d> moved;
  std::size_t kept = 0;
  for (std::size_t j = 0; j < solution.residuals.size(); ++j) {
    kept += solution.residuals[j].dropped ? 0 : 1;
    if (solution.residuals[j].dropped || again.residuals[j].dropped) continue;
    given.push_back(solution.residuals[j].residual);
    moved.push_back(again.residuals[j].residual);
  }
  EXPECT_LT(given.size(), kept);  // the shifted solve dropped some
  EXPECT_EQ(ab->corners, given.size());
  EXPECT_NEAR(ab->as_given.rms_px, rms_of(given), 1e-12);
  EXPECT_NEAR(ab->shifted.rms_px, rms_of(moved), 1e-12);
  const Transfer t = value_of(cross_camera_transfer(
      shifted, scene.board, scene.views, again.world_to_camera));
  EXPECT_EQ(ab->shifted.transfer_median_px, t.median_px);
  EXPECT_EQ(ab->shifted.transfer_p90_px, t.p90_px);
}

// A shifted re-solve bundle_adjust refuses -- here one that does not
// converge, every principal point 1000 px off -- is recorded as refused, not
// an error, and the other metrics are measured all the same.
TEST(PrincipalPointAB, RecordsARefusedReSolve) {
  const Scene scene = make_scene(6);
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  core::Result<PrincipalPointAB> ab = PrincipalPointAB{};
  EXPECT_NO_THROW(ab = principal_point_ab(scene.cameras, scene.board,
                                          scene.views, start, solution, {},
                                          1000.0));
  ASSERT_TRUE(ab.ok()) << ab.status().message();
  EXPECT_EQ(ab->shift_px, 1000.0);
  EXPECT_EQ(ab->refused.rfind("bundle_adjust: ", 0), 0u) << ab->refused;
  EXPECT_EQ(ab->corners, 0u);
  EXPECT_TRUE(std::isnan(ab->as_given.rms_px));
  EXPECT_TRUE(std::isnan(ab->shifted.transfer_median_px));
  EXPECT_EQ(ab->verdict, PrincipalPointVerdict::Mixed);

  MetricsOptions options;
  options.principal_shift_px = 1000.0;
  const CalibrationMetrics m = measured(scene, start, solution, options);
  EXPECT_EQ(m.principal_point.refused, ab->refused);
  EXPECT_EQ(m.leave_one_out.captures.size(), solution.boards.size());
  EXPECT_FALSE(m.transfer.errors.empty());
}

// A verdict needs both the RMS and the transfer's median to agree; a tie or
// a figure not a number is mixed.
TEST(PrincipalPointAB, AVerdictNeedsBothFiguresToAgree) {
  const auto arm = [](double rms_px, double median_px) {
    PrincipalPointArm a;
    a.rms_px = rms_px;
    a.transfer_median_px = median_px;
    return a;
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  using V = PrincipalPointVerdict;
  EXPECT_EQ(principal_point_verdict(arm(1.0, 2.0), arm(0.9, 1.9)), V::Shifted);
  EXPECT_EQ(principal_point_verdict(arm(0.9, 1.9), arm(1.0, 2.0)), V::AsGiven);
  EXPECT_EQ(principal_point_verdict(arm(1.0, 2.0), arm(0.9, 2.1)), V::Mixed);
  EXPECT_EQ(principal_point_verdict(arm(0.9, 2.1), arm(1.0, 2.0)), V::Mixed);
  EXPECT_EQ(principal_point_verdict(arm(1.0, 2.0), arm(1.0, 1.9)), V::Mixed);
  EXPECT_EQ(principal_point_verdict(arm(1.0, nan), arm(0.9, 1.9)), V::Mixed);
  EXPECT_EQ(principal_point_verdict(arm(0.9, 1.9), arm(nan, nan)), V::Mixed);
}

TEST(PrincipalPointAB, RefusesAShiftOfNothing) {
  const Scene scene = make_scene(6);
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  for (const double shift :
       {0.0, std::numeric_limits<double>::quiet_NaN(), kInfinity}) {
    core::Result<PrincipalPointAB> ab = PrincipalPointAB{};
    EXPECT_NO_THROW(ab = principal_point_ab(scene.cameras, scene.board,
                                            scene.views, start, solution, {},
                                            shift));
    EXPECT_EQ(ab.status().domain(), Code::InvalidArgument);
    EXPECT_EQ(ab.status().message().rfind("principal_point_ab: ", 0), 0u);
  }
}

TEST(CalibrationMetrics, GeometryIsTheCentresAndBaselines) {
  const Scene scene = make_scene(12);
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  const CalibrationMetrics m = measured(scene, start, solution);
  const std::vector<camera::Mat4d> truth = test::truths_in_world_of(scene, 0);
  const std::size_t k = scene.cameras.size();
  ASSERT_EQ(m.geometry.centres_m.size(), k);
  ASSERT_EQ(m.geometry.baselines_m.size(), k);
  for (std::size_t a = 0; a < k; ++a) {
    const camera::Vec3d centre = test::centre_of(
        camera::matrix_from_rodrigues(solution.world_to_camera[a]));
    EXPECT_NEAR(glm::length(m.geometry.centres_m[a] - centre), 0.0, 1e-12);
    EXPECT_LT(glm::length(centre - test::centre_of(truth[a])), 0.002);
    ASSERT_EQ(m.geometry.baselines_m[a].size(), k);
    for (std::size_t b = 0; b < k; ++b) {
      EXPECT_EQ(m.geometry.baselines_m[a][b], m.geometry.baselines_m[b][a]);
      EXPECT_NEAR(
          m.geometry.baselines_m[a][b],
          glm::length(m.geometry.centres_m[a] - m.geometry.centres_m[b]),
          1e-12);
    }
  }
  EXPECT_EQ(m.geometry.centres_m[0], camera::Vec3d(0.0));
  EXPECT_GT(m.geometry.baselines_m[0][1], 1.0);
}

// The views initialise left out are listed, in the observations' order,
// each as initialise recorded it.
TEST(CalibrationMetrics, ListsTheViewsInitialiseLeftOut) {
  Scene scene = make_scene(12);
  // View 6's corners 9 px either way in turn: its own pose fits poorly.
  for (std::size_t j = 0; j < scene.views[6].points.size(); ++j) {
    scene.views[6].points[j].pixel.x += j % 2 == 0 ? 9.0 : -9.0;
  }
  const InitialRig start = first_rig(scene);
  ASSERT_EQ(start.views[6].fate, solve::ViewFate::PoorFit);
  const RigSolution solution = solved(scene, start);
  const CalibrationMetrics m = measured(scene, start, solution);
  std::vector<const solve::ViewRecord*> want;
  for (const solve::ViewRecord& record : start.views) {
    if (!solve::adjusts(record)) want.push_back(&record);
  }
  ASSERT_EQ(m.views_left_out.size(), want.size());
  bool poor = false;
  for (std::size_t i = 0; i < want.size(); ++i) {
    const solve::ViewRecord& got = m.views_left_out[i];
    EXPECT_EQ(got.camera, want[i]->camera);
    EXPECT_EQ(got.capture, want[i]->capture);
    EXPECT_EQ(got.fate, want[i]->fate);
    EXPECT_EQ(got.use, want[i]->use);
    EXPECT_EQ(got.pnp_rms_px, want[i]->pnp_rms_px);
    EXPECT_EQ(got.ambiguity, want[i]->ambiguity);
    EXPECT_EQ(got.tilt_deg, want[i]->tilt_deg);
    poor =
        poor || (got.capture == start.views[6].capture &&
                 got.camera == start.views[6].camera && got.pnp_rms_px > 5.0);
  }
  EXPECT_TRUE(poor);
}

// Outliers injected: the rejections counted by round and reason, corners and
// views apart.
TEST(CalibrationMetrics, CountsTheRejectionsByRoundAndReason) {
  Scene scene = make_scene(12);
  for (DetectedPoint& point : scene.views[5].points) point.pixel.x += 6.0;
  scene.views[9].points[3].pixel.y += 9.0;
  scene.views[14].points[6].pixel.x -= 8.0;
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  ASSERT_FALSE(solution.rejections.empty());
  const CalibrationMetrics m = measured(scene, start, solution);
  std::map<std::pair<int, solve::Dropped>, std::pair<std::size_t, std::size_t>>
      want;
  for (const solve::Rejection& r : solution.rejections) {
    auto& [corners, views] = want[{r.round, r.reason}];
    ++(r.corner < 0 ? views : corners);
  }
  ASSERT_EQ(m.rejections.size(), want.size());
  std::size_t i = 0;
  for (const auto& [key, count] : want) {
    EXPECT_EQ(m.rejections[i].round, key.first);
    EXPECT_EQ(m.rejections[i].reason, key.second);
    EXPECT_EQ(m.rejections[i].corners, count.first);
    EXPECT_EQ(m.rejections[i].views, count.second);
    ++i;
  }
  // The dropped corners are in no group.
  std::size_t kept = kept_residuals(solution).size();
  EXPECT_EQ(m.reprojection.solved.overall.count, kept);
  EXPECT_LT(kept, solution.residuals.size());
}

TEST(CalibrationMetrics, RefusesOptionsAndASolutionThatDoNotFit) {
  const Scene scene = make_scene(6);
  const InitialRig start = first_rig(scene);
  const RigSolution solution = solved(scene, start);
  const auto refused = [&](const InitialRig& s, const RigSolution& sol,
                           const MetricsOptions& options,
                           const std::string& says) {
    core::Result<CalibrationMetrics> m = CalibrationMetrics{};
    EXPECT_NO_THROW(m = calibration_metrics(scene.cameras, scene.board,
                                            scene.views, s, sol, options));
    EXPECT_EQ(m.status().domain(), Code::InvalidArgument) << says;
    EXPECT_EQ(m.status().message().rfind("calibration_metrics: " + says, 0), 0u)
        << m.status().message();
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  MetricsOptions o;
  o.principal_shift_px = 0.0;
  refused(start, solution, o, "options.principal_shift_px");
  o = {};
  o.principal_shift_px = nan;
  refused(start, solution, o, "options.principal_shift_px");
  o = {};
  o.histogram_bin_px = 0.0;
  refused(start, solution, o, "options.histogram_bin_px");
  o = {};
  o.histogram_bin_px = kInfinity;
  refused(start, solution, o, "options.histogram_bin_px");
  o = {};
  o.histogram_bins = 0;
  refused(start, solution, o, "options.histogram_bins");
  o = {};
  o.initialise.oblique_deg = 0.0;
  refused(start, solution, o, "options.initialise.oblique_deg");
  o = {};
  o.bundle_adjust.huber_px = -1.0;
  refused(start, solution, o, "options.bundle_adjust.huber_px");

  RigSolution fewer = solution;
  fewer.world_to_camera.pop_back();
  refused(start, fewer, {}, "the solution places 3 cameras of 4");
  RigSolution off = solution;
  off.residuals[3].corner += 1;
  refused(start, off, {}, "the solution's residuals do not fit");
  RigSolution more = solution;
  more.residuals.push_back(more.residuals.back());
  refused(start, more, {}, "the solution has");
  RigSolution nan_pose = solution;
  nan_pose.world_to_camera[1].rvec.x = nan;
  refused(start, nan_pose, {}, "the solution's camera");
  InitialRig short_start = start;
  short_start.views.pop_back();
  refused(short_start, solution, {}, "the start has");
  InitialRig swapped = start;
  std::swap(swapped.views[0], swapped.views[1]);
  ASSERT_NE(swapped.views[0].camera, start.views[0].camera);
  refused(swapped, solution, {}, "the start's record of observation 0 ");
  InitialRig moved = start;
  moved.views[2].capture += 100;
  refused(moved, solution, {}, "the start's record of observation 2 ");
  InitialRig other = start;
  other.reference = 1;
  refused(other, solution, {}, "the start's reference");
}

TEST(CalibrationMetrics, NamesEachVerdict) {
  EXPECT_STREQ(to_string(PrincipalPointVerdict::AsGiven), "as_given");
  EXPECT_STREQ(to_string(PrincipalPointVerdict::Shifted), "shifted");
  EXPECT_STREQ(to_string(PrincipalPointVerdict::Mixed), "mixed");
  EXPECT_STREQ(to_string(static_cast<PrincipalPointVerdict>(9)), "unknown");
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
