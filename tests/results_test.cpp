// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A solved rig's results files: recon's calibration file posed by each
// camera's T_cw and read back by recon; the report parsed back with
// nlohmann, every section, member and number as the metrics and the solution
// hold it; the residuals CSV, every corner; the world at the origin tag in
// both files, and the reference's for each miss, its file to the byte the
// solution's own; and refusals.

#include "volumetric_kit/calib/quality/results.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "result_checks.hpp"
#include "synthetic_scene.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;
using test::value_of;
using Code = core::Status::Code;
using json = nlohmann::json;

// One solved scene, with outliers so that something is left out by the
// adjustment and by initialise, measured with options not the defaults.
struct Solved {
  test::Scene scene = test::make_scene(10);
  MetricsOptions options;
  solve::InitialRig start;
  solve::RigSolution solution;
  CalibrationMetrics metrics;

  Solved() {
    for (DetectedPoint& p : scene.views[4].points) p.pixel.x += 6.0;
    scene.views[7].points[2].pixel.y += 9.0;
    for (std::size_t j = 0; j < scene.views[9].points.size(); ++j) {
      scene.views[9].points[j].pixel.x += j % 2 == 0 ? 9.0 : -9.0;
    }
    options.histogram_bins = 8;
    options.principal_shift_px = 1.5;
    options.principal_margin_fraction = 0.25;
    options.principal_margin_px = 0.125;
    start =
        value_of(solve::initialise(scene.cameras, scene.board, scene.views, 0));
    solution = value_of(
        solve::bundle_adjust(scene.cameras, scene.board, scene.views, start));
    metrics = value_of(calibration_metrics(
        scene.cameras, scene.board, scene.views, start, solution, options));
  }
};

const Solved& solved() {
  static const Solved once;
  return once;
}

ResultFiles formatted(const std::vector<solve::RigCamera>& cameras,
                      const solve::RigSolution& solution,
                      const CalibrationMetrics& metrics) {
  core::Result<ResultFiles> files = ResultFiles{};
  EXPECT_NO_THROW(files = format_results(cameras, solution, metrics));
  return value_of(std::move(files));
}

ResultFiles formatted(const Solved& s) {
  return formatted(s.scene.cameras, s.solution, s.metrics);
}

// The file's pose is each camera's T_cw, world to camera; recon's reader
// gives its inverse as color_to_world.
TEST(Results, TheCalibrationFilePosesEachCameraByItsTcw) {
  const Solved& s = solved();
  const ResultFiles files = formatted(s);
  const json doc = json::parse(files.array_calibration);
  EXPECT_EQ(doc["format"], "volumetric_kit.array_calibration");
  EXPECT_EQ(doc["version"], 2);
  EXPECT_EQ(doc["world"]["sensor"], s.scene.cameras[0].id);
  const camera::ArrayCalibration back =
      value_of(camera::parse_array_calibration(files.array_calibration));
  for (std::size_t c = 0; c < s.scene.cameras.size(); ++c) {
    const std::string& id = s.scene.cameras[c].id;
    const camera::RodriguesTransform& t = s.solution.world_to_camera[c];
    const json& pose = doc["device_calibration"][id]["pose"];
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(pose["rvec"][i].get<double>(), t.rvec[i], 1e-12) << id;
      EXPECT_NEAR(pose["tvec"][i].get<double>(), t.tvec[i], 1e-12) << id;
    }
    const camera::SensorCalibration* sensor = camera::find_sensor(back, id);
    ASSERT_NE(sensor, nullptr);
    ASSERT_TRUE(sensor->color_to_world.has_value());
    // color_to_world T_cw is the identity: the reader inverted the pose.
    const camera::Mat4d product =
        *sensor->color_to_world * camera::matrix_from_rodrigues(t);
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        EXPECT_NEAR(product[col][row], col == row ? 1.0 : 0.0, 1e-12) << id;
      }
    }
    ASSERT_TRUE(sensor->color.has_value());
    EXPECT_EQ(sensor->color->model.intrinsics.cx,
              s.scene.cameras[c].model.intrinsics.cx);
    EXPECT_EQ(sensor->color->source, camera::IntrinsicsSource::Factory);
  }
  // A camera off the reference is posed away from the origin, so the check
  // above tells T_cw from its inverse.
  EXPECT_GT(glm::length(s.solution.world_to_camera[1].tvec), 0.5);
}

// A number as the report writes it: itself, or null when not finite.
void expect_number(const json& j, double v) {
  if (std::isfinite(v)) {
    ASSERT_TRUE(j.is_number()) << j;
    EXPECT_EQ(j.get<double>(), v);
  } else {
    EXPECT_TRUE(j.is_null()) << j;
  }
}

template <class T>
void expect_optional(const json& j, const std::optional<T>& v) {
  if (v) {
    EXPECT_EQ(j.get<T>(), *v);
  } else {
    EXPECT_TRUE(j.is_null()) << j;
  }
}

// An object's keys, in the order written.
std::vector<std::string> keys_of(const nlohmann::ordered_json& j) {
  std::vector<std::string> keys;
  for (const auto& item : j.items()) keys.push_back(item.key());
  return keys;
}

void expect_stats(const json& j, const ErrorStats& s) {
  EXPECT_EQ(j["count"].get<std::size_t>(), s.count);
  expect_number(j["rms_px"], s.rms_px);
  expect_number(j["p90_px"], s.p90_px);
  expect_number(j["max_px"], s.max_px);
  expect_number(j["mean_px"][0], s.mean_px.x);
  expect_number(j["mean_px"][1], s.mean_px.y);
}

// Every section, in order, every member exactly the metrics' and the
// solution's; cameras by id; each figure once. The solution and the metrics
// are given values that differ wherever two members could be swapped.
TEST(Results, TheReportHasItsSchema) {
  const Solved& s = solved();
  const std::vector<solve::RigCamera>& cams = s.scene.cameras;
  const std::size_t k = cams.size();
  solve::RigSolution solution = s.solution;
  ASSERT_GE(solution.links.size(), 2u);
  solution.links[0].weak = !solution.links[0].weak;
  solution.links[1].weak = !solution.links[0].weak;
  CalibrationMetrics m = s.metrics;
  m.leave_one_out.cameras[1].centre_capture = 3;
  m.leave_one_out.cameras[1].rotation_capture = 5;
  m.leave_one_out.cameras[2].centre_mm = 0.25;
  m.leave_one_out.cameras[2].rotation_deg = 0.0625;
  m.rejections.push_back({7, solve::Dropped::TooNear, 3, 5});
  const ResultFiles files = formatted(cams, solution, m);
  const json doc = json::parse(files.report);
  const nlohmann::ordered_json ordered =
      nlohmann::ordered_json::parse(files.report);
  EXPECT_EQ(keys_of(ordered),
            (std::vector<std::string>{
                "format", "version", "settings", "solver", "tree",
                "reprojection", "rejections", "transfer", "leave_one_out",
                "covariance", "sensitivity", "principal_point_ab", "geometry",
                "world", "depth", "depth_unchecked"}));
  EXPECT_EQ(doc["format"], "volumetric_kit_calib.report");
  EXPECT_EQ(doc["version"], 1);
  EXPECT_TRUE(doc["depth"].is_null());  // not checked, nor asked to be
  EXPECT_TRUE(doc["depth_unchecked"].is_null());

  // The settings the metrics were measured with.
  const json& settings = doc["settings"];
  EXPECT_EQ(keys_of(ordered["settings"]),
            (std::vector<std::string>{"reference", "cameras", "initialise",
                                      "bundle_adjust", "metrics"}));
  EXPECT_EQ(settings["reference"], cams[0].id);
  ASSERT_EQ(settings["cameras"].size(), k);
  for (std::size_t c = 0; c < k; ++c) {
    EXPECT_EQ(settings["cameras"][c], cams[c].id);
  }
  const solve::InitialiseOptions& init = m.options.initialise;
  const json& i = settings["initialise"];
  EXPECT_EQ(i.size(), 6u);
  EXPECT_EQ(i["max_pnp_rms_px"].get<double>(), init.max_pnp_rms_px);
  EXPECT_EQ(i["min_ambiguity"].get<double>(), init.min_ambiguity);
  EXPECT_EQ(i["oblique_deg"].get<double>(), init.oblique_deg);
  EXPECT_EQ(i["weak_below"], init.weak_below);
  EXPECT_EQ(i["inlier_deg"].get<double>(), init.inlier_deg);
  EXPECT_EQ(i["truncation_px"].get<double>(), init.truncation_px);
  const solve::BundleAdjustOptions& adjust = m.options.bundle_adjust;
  const json& a = settings["bundle_adjust"];
  EXPECT_EQ(a.size(), 13u);
  EXPECT_EQ(a["huber_px"].get<double>(), adjust.huber_px);
  EXPECT_EQ(a["min_depth_m"].get<double>(), adjust.min_depth_m);
  EXPECT_EQ(a["max_iterations"], adjust.max_iterations);
  EXPECT_EQ(a["outlier_rounds"], adjust.outlier_rounds);
  EXPECT_EQ(a["outlier_px"].get<double>(), adjust.outlier_px);
  EXPECT_EQ(a["outlier_mads"].get<double>(), adjust.outlier_mads);
  EXPECT_EQ(a["outlier_view_ratio"].get<double>(), adjust.outlier_view_ratio);
  EXPECT_EQ(a["outlier_view_px"].get<double>(), adjust.outlier_view_px);
  EXPECT_EQ(a["min_shared"], adjust.min_shared);
  EXPECT_EQ(a["max_condition"].get<double>(), adjust.max_condition);
  EXPECT_EQ(a["max_captures"].get<std::size_t>(), adjust.max_captures);
  EXPECT_EQ(a["focal_fraction"].get<double>(), adjust.focal_fraction);
  EXPECT_EQ(a["principal_px"].get<double>(), adjust.principal_px);
  EXPECT_EQ(settings["metrics"], json::parse(R"({"principal_shift_px": 1.5,
                            "principal_margin_fraction": 0.25,
                            "principal_margin_px": 0.125,
                            "histogram_bin_px": 0.1, "histogram_bins": 8})"));

  const json& solver = doc["solver"];
  EXPECT_EQ(solver.size(), 6u);
  EXPECT_EQ(solver["solves"], solution.solves);
  EXPECT_EQ(solver["iterations"], solution.iterations);
  EXPECT_EQ(solver["initial_energy"].get<double>(), solution.initial_energy);
  EXPECT_EQ(solver["final_energy"].get<double>(), solution.final_energy);
  EXPECT_EQ(solver["variance"].get<double>(), solution.variance);
  EXPECT_NEAR(solver["seconds"].get<double>(), solution.seconds, 5e-7);
  ASSERT_EQ(doc["tree"].size(), solution.links.size());
  for (std::size_t l = 0; l < solution.links.size(); ++l) {
    const solve::PlacedLink& link = solution.links[l];
    const json& j = doc["tree"][l];
    EXPECT_EQ(j.size(), 5u);
    EXPECT_EQ(j["camera"], cams[link.camera].id);
    EXPECT_EQ(j["parent"], cams[link.parent].id);
    EXPECT_EQ(j["shared"], link.shared);
    EXPECT_EQ(j["inliers"], link.inliers);
    EXPECT_EQ(j["weak"], link.weak);
  }

  const json& reprojection = doc["reprojection"];
  for (const char* state : {"initial", "solved"}) {
    SCOPED_TRACE(state);
    const ReprojectionStats& r = std::string(state) == "initial"
                                     ? m.reprojection.initial
                                     : m.reprojection.solved;
    const json& j = reprojection[state];
    EXPECT_EQ(keys_of(ordered["reprojection"][state]),
              (std::vector<std::string>{"overall", "without_oblique",
                                        "legacy_rmse_px", "histogram",
                                        "cameras", "captures", "views"}));
    expect_stats(j["overall"], r.overall);
    expect_stats(j["without_oblique"], r.without_oblique);
    expect_number(j["legacy_rmse_px"], r.legacy_rmse_px);
    EXPECT_EQ(j["histogram"].get<std::vector<std::size_t>>(),
              r.histogram.counts);
    ASSERT_EQ(j["cameras"].size(), k);
    for (std::size_t c = 0; c < k; ++c) {
      expect_stats(j["cameras"][cams[c].id], r.cameras[c]);
    }
    ASSERT_EQ(j["captures"].size(), r.captures.size());
    for (std::size_t c = 0; c < r.captures.size(); ++c) {
      EXPECT_EQ(j["captures"][c]["capture"], r.captures[c].capture);
      expect_stats(j["captures"][c], r.captures[c].errors);
    }
    ASSERT_EQ(j["views"].size(), r.views.size());
    for (std::size_t v = 0; v < r.views.size(); ++v) {
      EXPECT_EQ(j["views"][v]["camera"], cams[r.views[v].camera].id);
      EXPECT_EQ(j["views"][v]["capture"], r.views[v].capture);
      EXPECT_EQ(j["views"][v]["oblique"], r.views[v].oblique);
      expect_stats(j["views"][v], r.views[v].errors);
    }
  }
  expect_number(reprojection["legacy_reduction_percent"],
                m.reprojection.legacy_reduction_percent);

  const json& rejections = doc["rejections"];
  EXPECT_EQ(keys_of(ordered["rejections"]),
            (std::vector<std::string>{"views_left_out", "counts", "list"}));
  ASSERT_FALSE(m.views_left_out.empty());
  ASSERT_EQ(rejections["views_left_out"].size(), m.views_left_out.size());
  bool poor = false;
  for (std::size_t v = 0; v < m.views_left_out.size(); ++v) {
    const solve::ViewRecord& view = m.views_left_out[v];
    const json& j = rejections["views_left_out"][v];
    EXPECT_EQ(j.size(), 7u);
    EXPECT_EQ(j["camera"], cams[view.camera].id);
    EXPECT_EQ(j["capture"], view.capture);
    EXPECT_EQ(j["fate"], solve::to_string(view.fate));
    EXPECT_EQ(j["use"], compute::to_string(view.use));
    expect_optional(j["pnp_rms_px"], view.pnp_rms_px);
    expect_optional(j["ambiguity"], view.ambiguity);
    expect_optional(j["tilt_deg"], view.tilt_deg);
    poor = poor || j["fate"] == "poor_fit";
  }
  EXPECT_TRUE(poor);
  ASSERT_EQ(rejections["counts"].size(), m.rejections.size());
  for (std::size_t c = 0; c < m.rejections.size(); ++c) {
    const RejectionCount& count = m.rejections[c];
    const json& j = rejections["counts"][c];
    EXPECT_EQ(j["round"], count.round);
    EXPECT_EQ(j["reason"], solve::to_string(count.reason));
    EXPECT_EQ(j["corners"], count.corners);
    EXPECT_EQ(j["views"], count.views);
  }
  ASSERT_FALSE(solution.rejections.empty());
  ASSERT_EQ(rejections["list"].size(), solution.rejections.size());
  for (std::size_t r = 0; r < solution.rejections.size(); ++r) {
    const solve::Rejection& rejection = solution.rejections[r];
    const json& j = rejections["list"][r];
    EXPECT_EQ(j["camera"], cams[rejection.camera].id);
    EXPECT_EQ(j["capture"], rejection.capture);
    EXPECT_EQ(j["corner"], rejection.corner);
    EXPECT_EQ(j["round"], rejection.round);
    EXPECT_EQ(j["reason"], solve::to_string(rejection.reason));
    EXPECT_EQ(j["value"].get<double>(), rejection.value);
    EXPECT_EQ(j["bound"].get<double>(), rejection.bound);
  }

  const Transfer& t = m.transfer;
  const json& transfer = doc["transfer"];
  expect_number(transfer["median_px"], t.median_px);
  expect_number(transfer["p90_px"], t.p90_px);
  expect_number(transfer["worst_px"], t.worst_px);
  ASSERT_EQ(transfer["errors"].size(), t.errors.size());
  for (std::size_t e = 0; e < t.errors.size(); ++e) {
    const json& j = transfer["errors"][e];
    EXPECT_EQ(j["capture"], t.errors[e].capture);
    EXPECT_EQ(j["from"], cams[t.errors[e].from].id);
    EXPECT_EQ(j["to"], cams[t.errors[e].to].id);
    expect_number(j["error_px"], t.errors[e].error_px);
  }
  std::size_t pairs = 0;
  for (const json& p : transfer["pairs"]) {
    const std::size_t from = test::index_of(cams, p["from"].get<std::string>());
    const std::size_t to = test::index_of(cams, p["to"].get<std::string>());
    EXPECT_EQ(p["count"], t.pairs[from][to].count);
    expect_number(p["mean_px"], t.pairs[from][to].mean_px);
    ++pairs;
  }
  std::size_t want_pairs = 0;
  for (const auto& row : t.pairs) {
    for (const PairTransfer& p : row) want_pairs += p.count > 0 ? 1 : 0;
  }
  EXPECT_EQ(pairs, want_pairs);

  const LeaveOneOut& loo = m.leave_one_out;
  const json& left_out = doc["leave_one_out"];
  for (std::size_t c = 0; c < k; ++c) {
    const json& j = left_out["cameras"][cams[c].id];
    EXPECT_EQ(j["centre_mm"].get<double>(), loo.cameras[c].centre_mm);
    expect_optional(j["centre_capture"], loo.cameras[c].centre_capture);
    EXPECT_EQ(j["rotation_deg"].get<double>(), loo.cameras[c].rotation_deg);
    expect_optional(j["rotation_capture"], loo.cameras[c].rotation_capture);
  }
  EXPECT_TRUE(left_out["cameras"][cams[0].id]["rotation_capture"].is_null());
  ASSERT_EQ(left_out["captures"].size(), loo.captures.size());
  EXPECT_EQ(keys_of(ordered["leave_one_out"]["captures"][0]),
            (std::vector<std::string>{"capture", "refused", "rms_px",
                                      "legacy_rmse_px", "cameras"}));
  for (std::size_t l = 0; l < loo.captures.size(); ++l) {
    const LeftOut& left = loo.captures[l];
    const json& j = left_out["captures"][l];
    EXPECT_EQ(j["capture"], left.capture);
    EXPECT_TRUE(j["refused"].is_null());
    expect_number(j["rms_px"], left.rms_px);
    expect_number(j["legacy_rmse_px"], left.legacy_rmse_px);
    ASSERT_EQ(j["cameras"].size(), left.cameras.size());
    for (std::size_t c = 0; c < left.cameras.size(); ++c) {
      EXPECT_EQ(j["cameras"][cams[c].id]["centre_mm"].get<double>(),
                left.cameras[c].centre_mm);
      EXPECT_EQ(j["cameras"][cams[c].id]["rotation_deg"].get<double>(),
                left.cameras[c].rotation_deg);
    }
  }

  for (std::size_t c = 0; c < k; ++c) {
    const json& j = doc["covariance"][cams[c].id];
    const solve::PoseCovariance& cov = solution.covariance[c];
    EXPECT_EQ(j["rotation_deg"].get<double>(), cov.rotation_deg);
    EXPECT_EQ(j["centre_mm"].get<double>(), cov.centre_mm);
    ASSERT_EQ(j["matrix"].size(), 6u);
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        EXPECT_EQ(j["matrix"][row][col].get<double>(),
                  cov.covariance(row, col));
      }
    }
  }

  // Each camera whose intrinsics are off, each intrinsic, each camera moved.
  const json& sensitivity = doc["sensitivity"];
  ASSERT_EQ(sensitivity.size(), k);
  for (std::size_t off = 0; off < k; ++off) {
    const solve::IntrinsicsSensitivity& by = solution.sensitivity[off];
    EXPECT_EQ(
        keys_of(ordered["sensitivity"][cams[off].id]),
        (std::vector<std::string>{"focal", "principal_x", "principal_y"}));
    for (const auto& [name, shifts] :
         {std::pair{"focal", &by.focal},
          std::pair{"principal_x", &by.principal_x},
          std::pair{"principal_y", &by.principal_y}}) {
      for (std::size_t c = 0; c < k; ++c) {
        const json& j = sensitivity[cams[off].id][name][cams[c].id];
        const solve::PoseShift& shift = (*shifts)[c];
        EXPECT_EQ(j["rotation_deg"].get<double>(), shift.rotation_deg);
        EXPECT_EQ(j["centre_mm"][0].get<double>(), shift.centre_mm.x);
        EXPECT_EQ(j["centre_mm"][1].get<double>(), shift.centre_mm.y);
        EXPECT_EQ(j["centre_mm"][2].get<double>(), shift.centre_mm.z);
      }
    }
  }

  const PrincipalPointAB& ab = m.principal_point;
  const json& principal = doc["principal_point_ab"];
  EXPECT_EQ(keys_of(ordered["principal_point_ab"]),
            (std::vector<std::string>{"refused", "corners", "as_given",
                                      "shifted", "verdict"}));
  EXPECT_EQ(ab.shift_px, 1.5);  // as settings.metrics has it
  EXPECT_TRUE(principal["refused"].is_null());
  EXPECT_EQ(principal["corners"], ab.corners);
  for (const auto& [name, side] : {std::pair{"as_given", &ab.as_given},
                                   std::pair{"shifted", &ab.shifted}}) {
    const json& j = principal[name];
    EXPECT_EQ(j.size(), 4u);
    expect_number(j["rms_px"], side->rms_px);
    expect_number(j["legacy_rmse_px"], side->legacy_rmse_px);
    expect_number(j["transfer_median_px"], side->transfer_median_px);
    expect_number(j["transfer_p90_px"], side->transfer_p90_px);
  }
  // Within the margins of settings.metrics.
  EXPECT_EQ(ab.verdict, PrincipalPointVerdict::Inconclusive);
  EXPECT_EQ(principal["verdict"], "inconclusive");

  const json& geometry = doc["geometry"];
  for (std::size_t c = 0; c < k; ++c) {
    const json& centre = geometry["centres_m"][cams[c].id];
    EXPECT_EQ(centre[0].get<double>(), m.geometry.centres_m[c].x);
    EXPECT_EQ(centre[1].get<double>(), m.geometry.centres_m[c].y);
    EXPECT_EQ(centre[2].get<double>(), m.geometry.centres_m[c].z);
  }
  ASSERT_EQ(geometry["baselines_m"].size(), k * (k - 1) / 2);
  std::size_t b = 0;
  for (std::size_t from = 0; from < k; ++from) {
    for (std::size_t to = from + 1; to < k; ++to, ++b) {
      const json& j = geometry["baselines_m"][b];
      EXPECT_EQ(j["a"], cams[from].id);
      EXPECT_EQ(j["b"], cams[to].id);
      EXPECT_EQ(j["m"].get<double>(), m.geometry.baselines_m[from][to]);
    }
  }
}

// Depth checks of the solved rig's 4 cameras, with options not the defaults
// and values that differ wherever two members could be swapped: views of two
// captures, one plane not valid; pairs pooled and of one capture.
DepthSummary depth_checks() {
  DepthCheckOptions options;
  options.inset_m = 0.025;
  options.seed_mm = 25.0;
  options.min_gate_mm = 4.5;
  options.sigma_gate = 2.5;
  options.min_inliers = 250;
  options.window_mm = 50.0;
  options.bin_mm = 0.25;
  options.edge_ratio = 0.04;
  options.agreement_stride = 2;
  options.min_overlap = 900;
  options.max_scale_error = 0.003;
  options.max_scatter_mm = 8.5;
  options.max_angle_deg = 1.25;
  options.max_signed_median_mm = 4.5;
  options.max_median_abs_mm = 9.5;
  std::vector<BoardDepthCheck> views(3);
  views[0].camera = 2;
  views[0].capture = 5;
  views[0].plane = {
      true, {0.3, 0.2, 0.001}, {0.0, 0.01, 1.0}, 0.00125, 9100, 210, 8700};
  views[0].ratios = {1.0021, 1.0019, 1.0024};
  views[0].offset_mm = 2.75;
  views[0].axial_offset_mm = 4.125;
  views[0].angle_deg = 0.375;
  views[1].camera = 0;
  views[1].capture = 5;
  views[1].plane.rms_m = 0.5;  // not valid: means nothing
  views[1].plane.members = 30;
  views[1].plane.out_of_range = 25;
  views[1].plane.inliers = 4;
  views[2].camera = 1;
  views[2].capture = 3;
  views[2].plane = {
      true, {0.3, 0.2, -0.002}, {0.02, 0.0, -1.0}, 0.0021, 7300, 64, 7001};
  views[2].ratios = {0.9985};
  views[2].offset_mm = -11.5;
  views[2].axial_offset_mm = -23.25;
  views[2].angle_deg = 1.375;
  std::vector<PairAgreement> pairs(2);
  pairs[0].from = 3;
  pairs[0].to = 1;
  pairs[0].overlap = 41000;
  pairs[0].in_window = 40500;
  pairs[0].signed_median_mm = -0.75;
  pairs[0].median_abs_mm = 1.625;
  pairs[0].p90_abs_mm = 4.25;
  pairs[1] = pairs[0];
  pairs[1].capture = 5;
  pairs[1].overlap = 880;
  pairs[1].in_window = 870;
  return value_of(summarize_depth_checks(4, views, pairs, options));
}

void expect_pairs(const json& j, const std::vector<PairCheck>& pairs,
                  const std::vector<solve::RigCamera>& cams) {
  ASSERT_EQ(j.size(), pairs.size());
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const PairAgreement& p = pairs[i].agreement;
    const json& got = j[i];
    EXPECT_EQ(got.size(), 8u);
    EXPECT_EQ(got["from"], cams[p.from].id);
    EXPECT_EQ(got["to"], cams[p.to].id);
    EXPECT_EQ(got["overlap"], p.overlap);
    EXPECT_EQ(got["in_window"], p.in_window);
    expect_number(got["signed_median_mm"], p.signed_median_mm);
    expect_number(got["median_abs_mm"], p.median_abs_mm);
    expect_number(got["p90_abs_mm"], p.p90_abs_mm);
    EXPECT_EQ(got["verdict"], to_string(pairs[i].verdict));
  }
}

void expect_ratio(const json& j, const RatioStats& r) {
  EXPECT_EQ(j.size(), 3u);
  EXPECT_EQ(j["count"], r.count);
  expect_number(j["mean"], r.mean);
  expect_number(j["sd"], r.sd);
}

// The depth section: every member as the summary holds it, cameras by id;
// an invalid plane's RMS is null.
TEST(Results, TheReportHasItsDepthSection) {
  const Solved& s = solved();
  const std::vector<solve::RigCamera>& cams = s.scene.cameras;
  const DepthSummary d = depth_checks();
  const ResultFiles files =
      value_of(format_results(cams, s.solution, s.metrics, &d));
  const json doc = json::parse(files.report);
  const nlohmann::ordered_json ordered =
      nlohmann::ordered_json::parse(files.report)["depth"];
  const json& depth = doc["depth"];
  EXPECT_EQ(keys_of(ordered),
            (std::vector<std::string>{"settings", "ratio", "scale", "cameras",
                                      "views", "pairs", "captures"}));
  EXPECT_EQ(depth["settings"],
            json::parse(R"({"inset_m": 0.025, "seed_mm": 25.0,
                            "min_gate_mm": 4.5, "sigma_gate": 2.5,
                            "min_inliers": 250, "window_mm": 50.0,
                            "bin_mm": 0.25, "edge_ratio": 0.04,
                            "agreement_stride": 2, "min_overlap": 900,
                            "max_scale_error": 0.003, "max_scatter_mm": 8.5,
                            "max_angle_deg": 1.25,
                            "max_signed_median_mm": 4.5,
                            "max_median_abs_mm": 9.5})"));
  expect_ratio(depth["ratio"], d.ratio);
  EXPECT_EQ(depth["ratio"]["count"], 4u);
  EXPECT_EQ(depth["scale"], to_string(d.scale));
  ASSERT_EQ(depth["cameras"].size(), cams.size());
  for (std::size_t c = 0; c < cams.size(); ++c) {
    const CameraDepth& cam = d.cameras[c];
    const json& j = depth["cameras"][cams[c].id];
    EXPECT_EQ(keys_of(ordered["cameras"][cams[c].id]),
              (std::vector<std::string>{"views", "ratio", "scale", "offset_mm",
                                        "angle_deg", "depth_offset_m",
                                        "scatter_mm", "plane", "verdict"}));
    EXPECT_EQ(j["views"], cam.views);
    expect_ratio(j["ratio"], cam.ratio);
    EXPECT_EQ(j["scale"], to_string(cam.scale));
    expect_number(j["offset_mm"], cam.offset_mm);
    expect_number(j["angle_deg"], cam.angle_deg);
    expect_number(j["depth_offset_m"], cam.depth_offset_m);
    expect_number(j["scatter_mm"], cam.scatter_mm);
    EXPECT_EQ(j["plane"], to_string(cam.plane));
    EXPECT_EQ(j["verdict"], to_string(cam.verdict));
  }
  EXPECT_EQ(depth["cameras"][cams[1].id]["plane"], "off");
  EXPECT_EQ(depth["cameras"][cams[1].id]["verdict"], "off");
  EXPECT_EQ(depth["cameras"][cams[2].id]["scale"], "ok");
  EXPECT_EQ(depth["cameras"][cams[2].id]["depth_offset_m"], 0.004125);
  EXPECT_EQ(depth["cameras"][cams[2].id]["plane"], "ok");
  EXPECT_TRUE(depth["cameras"][cams[0].id]["depth_offset_m"].is_null());
  EXPECT_EQ(depth["cameras"][cams[0].id]["plane"], "insufficient");
  EXPECT_EQ(depth["cameras"][cams[0].id]["verdict"], "insufficient");
  EXPECT_TRUE(doc["depth_unchecked"].is_null());
  ASSERT_EQ(depth["views"].size(), d.views.size());
  for (std::size_t v = 0; v < d.views.size(); ++v) {
    const BoardDepthCheck& view = d.views[v];
    const json& j = depth["views"][v];
    EXPECT_EQ(keys_of(ordered["views"][v]),
              (std::vector<std::string>{
                  "camera", "capture", "valid", "members", "out_of_range",
                  "inliers", "rms_mm", "ratios", "offset_mm", "axial_offset_mm",
                  "angle_deg"}));
    EXPECT_EQ(j["camera"], cams[view.camera].id);
    EXPECT_EQ(j["capture"], view.capture);
    EXPECT_EQ(j["valid"], view.plane.valid);
    EXPECT_EQ(j["members"], view.plane.members);
    EXPECT_EQ(j["out_of_range"], view.plane.out_of_range);
    EXPECT_EQ(j["inliers"], view.plane.inliers);
    if (view.plane.valid) {
      EXPECT_EQ(j["rms_mm"].get<double>(), 1000.0 * view.plane.rms_m);
    } else {
      EXPECT_TRUE(j["rms_mm"].is_null());
    }
    EXPECT_EQ(j["ratios"].get<std::vector<double>>(), view.ratios);
    expect_number(j["offset_mm"], view.offset_mm);
    expect_number(j["axial_offset_mm"], view.axial_offset_mm);
    expect_number(j["angle_deg"], view.angle_deg);
  }
  EXPECT_EQ(depth["views"][0]["capture"], 3);
  EXPECT_FALSE(depth["views"][1]["valid"].get<bool>());
  expect_pairs(depth["pairs"], d.pairs, cams);
  ASSERT_EQ(depth["pairs"].size(), 12u);
  ASSERT_EQ(depth["captures"].size(), 1u);
  EXPECT_EQ(keys_of(ordered["captures"][0]),
            (std::vector<std::string>{"capture", "pairs"}));
  EXPECT_EQ(depth["captures"][0]["capture"], 5);
  expect_pairs(depth["captures"][0]["pairs"], d.captures[0].pairs, cams);
  // Camera 3 to camera 1, pooled: ok; in capture 5, too few samples.
  EXPECT_EQ(depth["pairs"][10]["verdict"], "ok");
  EXPECT_EQ(depth["captures"][0]["pairs"][10]["verdict"], "insufficient");
  EXPECT_EQ(depth["captures"][0]["pairs"][10]["overlap"], 880);
}

// Depth not checked, though it was to be: `depth` is null and
// `depth_unchecked` says why; a reason given with the checks is not written.
TEST(Results, TheReportSaysWhyTheDepthWasNotChecked) {
  const Solved& s = solved();
  const std::string why = "depth checks: capture 3: fit_planes: lost";
  const json unchecked =
      json::parse(value_of(format_results(s.scene.cameras, s.solution,
                                          s.metrics, nullptr, why))
                      .report);
  EXPECT_TRUE(unchecked["depth"].is_null());
  EXPECT_EQ(unchecked["depth_unchecked"], why);
  const DepthSummary d = depth_checks();
  const json checked = json::parse(
      value_of(format_results(s.scene.cameras, s.solution, s.metrics, &d, why))
          .report);
  EXPECT_TRUE(checked["depth"].is_object());
  EXPECT_TRUE(checked["depth_unchecked"].is_null());
}

TEST(Results, RefusesDepthChecksThatDoNotFit) {
  const Solved& s = solved();
  const auto refused = [&](const DepthSummary& d) {
    core::Result<ResultFiles> files = ResultFiles{};
    EXPECT_NO_THROW(
        files = format_results(s.scene.cameras, s.solution, s.metrics, &d));
    EXPECT_EQ(files.status().domain(), Code::InvalidArgument);
    EXPECT_EQ(files.status().message(),
              "format_results: the depth checks do not fit the rig's 4 "
              "cameras");
  };
  DepthSummary d = depth_checks();
  d.cameras.pop_back();
  refused(d);
  d = depth_checks();
  d.views[2].camera = 4;
  refused(d);
  d = depth_checks();
  d.pairs[3].agreement.to = 4;
  refused(d);
  d = depth_checks();
  d.captures[0].pairs[0].agreement.from = 7;
  refused(d);
}

// A number not finite is null; a refused re-solve says why.
TEST(Results, TheReportWritesNullForANumberNotFiniteAndARefusal) {
  const Solved& s = solved();
  CalibrationMetrics metrics = s.metrics;
  metrics.reprojection.legacy_reduction_percent =
      std::numeric_limits<double>::quiet_NaN();
  metrics.transfer.worst_px = std::numeric_limits<double>::infinity();
  metrics.leave_one_out.captures[0].refused = "bundle_adjust: too few";
  metrics.leave_one_out.captures[0].cameras.clear();
  metrics.leave_one_out.captures[0].rms_px =
      std::numeric_limits<double>::quiet_NaN();
  metrics.principal_point.refused = "bundle_adjust: did not converge";
  metrics.views_left_out[0].pnp_rms_px.reset();
  const ResultFiles files =
      value_of(format_results(s.scene.cameras, s.solution, metrics));
  const json doc = json::parse(files.report);
  EXPECT_TRUE(doc["reprojection"]["legacy_reduction_percent"].is_null());
  EXPECT_TRUE(doc["transfer"]["worst_px"].is_null());
  const json& left = doc["leave_one_out"]["captures"][0];
  EXPECT_EQ(left["refused"], "bundle_adjust: too few");
  EXPECT_TRUE(left["rms_px"].is_null());
  EXPECT_TRUE(left["cameras"].empty());
  EXPECT_EQ(doc["principal_point_ab"]["refused"],
            "bundle_adjust: did not converge");
  EXPECT_TRUE(doc["rejections"]["views_left_out"][0]["pnp_rms_px"].is_null());
}

// The solve's time is written to the microsecond with six decimals, so a
// report is as long however long its solve took under 10 s, and a run's
// allocations do not depend on it (Calibrate.OutOfMemoryAnywhereFailsTheRun).
// A time negative, not finite or of a billion seconds is null.
TEST(Results, TheSolveTimeHasSixDecimals) {
  const Solved& s = solved();
  const auto report_at = [&](double seconds) {
    solve::RigSolution solution = s.solution;
    solution.seconds = seconds;
    return value_of(format_results(s.scene.cameras, solution, s.metrics))
        .report;
  };
  const auto written = [](const std::string& report) {
    const std::string key = "\"seconds\": ";
    const std::size_t at = report.find(key) + key.size();
    return report.substr(at, report.find('\n', at) - at);
  };
  const std::string report = report_at(0.000233791);
  EXPECT_EQ(written(report), "0.000234");
  for (const double seconds : {0.0, 0.00015, 0.0000004, 1.5, 9.9999994}) {
    SCOPED_TRACE(seconds);
    const std::string other = report_at(seconds);
    EXPECT_EQ(other.size(), report.size()) << written(other);
  }
  EXPECT_EQ(written(report_at(0.00015)), "0.000150");
  EXPECT_EQ(written(report_at(0.0000004)), "0.000000");
  EXPECT_EQ(written(report_at(12.25)), "12.250000");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const double bad :
       {-0.5, nan, std::numeric_limits<double>::infinity(), 1e9}) {
    EXPECT_EQ(written(report_at(bad)), "null") << bad;
  }
}

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char ch = line[i];
    if (quoted) {
      if (ch == '"' && i + 1 < line.size() && line[i + 1] == '"') {
        field += '"';
        ++i;
      } else if (ch == '"') {
        quoted = false;
      } else {
        field += ch;
      }
    } else if (ch == '"') {
      quoted = true;
    } else if (ch == ',') {
      fields.push_back(field);
      field.clear();
    } else {
      field += ch;
    }
  }
  fields.push_back(field);
  return fields;
}

// The header, then one row per corner adjusted, each number reading back to
// the same double; a residual behind its camera is nan; an id with a comma
// is quoted.
TEST(Results, TheResidualsListEveryCorner) {
  const Solved& s = solved();
  std::vector<solve::RigCamera> cameras = s.scene.cameras;
  cameras[1].id = "SYN,\"1\"";
  solve::RigSolution solution = s.solution;
  solution.residuals[2].residual =
      camera::Vec2d(std::numeric_limits<double>::quiet_NaN());
  // The calibration names its sensors by id, in id order.
  for (camera::SensorCalibration& sensor : solution.calibration.sensors) {
    if (sensor.id == s.scene.cameras[1].id) sensor.id = cameras[1].id;
  }
  const ResultFiles files =
      value_of(format_results(cameras, solution, s.metrics));
  std::istringstream in(files.residuals);
  std::string line;
  ASSERT_TRUE(std::getline(in, line));
  EXPECT_EQ(line, "capture,serial,corner_id,u,v,du,dv,weight,dropped");
  std::size_t row = 0;
  std::size_t dropped = 0;
  while (std::getline(in, line)) {
    ASSERT_LT(row, solution.residuals.size());
    const solve::CornerResidual& r = solution.residuals[row];
    const std::vector<std::string> f = split(line);
    ASSERT_EQ(f.size(), 9u) << line;
    EXPECT_EQ(std::stoll(f[0]), r.capture);
    EXPECT_EQ(f[1], cameras[r.camera].id);
    EXPECT_EQ(std::stoi(f[2]), r.corner);
    EXPECT_EQ(std::strtod(f[3].c_str(), nullptr), r.pixel.x);
    EXPECT_EQ(std::strtod(f[4].c_str(), nullptr), r.pixel.y);
    if (row == 2) {
      EXPECT_EQ(f[5], "nan");
      EXPECT_EQ(f[6], "nan");
    } else {
      EXPECT_EQ(std::strtod(f[5].c_str(), nullptr), r.residual.x);
      EXPECT_EQ(std::strtod(f[6].c_str(), nullptr), r.residual.y);
    }
    EXPECT_EQ(std::strtod(f[7].c_str(), nullptr), r.weight);
    EXPECT_EQ(f[8], r.dropped ? "1" : "0");
    dropped += r.dropped ? 1 : 0;
    ++row;
  }
  EXPECT_EQ(row, solution.residuals.size());
  EXPECT_GT(dropped, 0u);
  EXPECT_NE(files.residuals.find("\"SYN,\"\"1\"\"\""), std::string::npos);
}

TEST(Results, RefusesWhatDoesNotFit) {
  const Solved& s = solved();
  const auto refused = [&](const std::vector<solve::RigCamera>& cameras,
                           const solve::RigSolution& solution,
                           const CalibrationMetrics& metrics,
                           const std::string& says) {
    core::Result<ResultFiles> files = ResultFiles{};
    EXPECT_NO_THROW(files = format_results(cameras, solution, metrics));
    EXPECT_EQ(files.status().domain(), Code::InvalidArgument) << says;
    EXPECT_EQ(files.status().message().rfind("format_results: ", 0), 0u);
    EXPECT_NE(files.status().message().find(says), std::string::npos)
        << files.status().message();
  };
  refused({}, s.solution, s.metrics, "no cameras");
  std::vector<solve::RigCamera> twice = s.scene.cameras;
  twice[2].id = twice[1].id;
  refused(twice, s.solution, s.metrics, "appears twice");
  std::vector<solve::RigCamera> fewer = s.scene.cameras;
  fewer.pop_back();
  refused(fewer, s.solution, s.metrics, "the solution's poses");
  solve::RigSolution bad = s.solution;
  bad.residuals[0].camera = 9;
  refused(s.scene.cameras, bad, s.metrics, "the solution's residuals");
  bad = s.solution;
  bad.sensitivity[1].focal.pop_back();
  refused(s.scene.cameras, bad, s.metrics, "the solution's sensitivities");
  CalibrationMetrics m = s.metrics;
  m.transfer.pairs[2].pop_back();
  refused(s.scene.cameras, s.solution, m, "the metrics' transfers");
  m = s.metrics;
  m.leave_one_out.captures[1].cameras.pop_back();
  refused(s.scene.cameras, s.solution, m, "leave-one-out");
  m = s.metrics;
  m.reprojection.solved.views[0].camera = 4;
  refused(s.scene.cameras, s.solution, m, "the metrics' reprojections");
  m = s.metrics;
  m.geometry.baselines_m.pop_back();
  refused(s.scene.cameras, s.solution, m, "the metrics' geometries");
  m = s.metrics;
  m.views_left_out[0].camera = 4;
  refused(s.scene.cameras, s.solution, m, "the metrics' views left out");
  // The calibration file names other cameras than the report.
  bad = s.solution;
  bad.calibration.sensors[1].id = "SYN-other";
  refused(s.scene.cameras, bad, s.metrics,
          "the solution's calibration's sensors");
  bad = s.solution;
  bad.calibration.sensors.pop_back();
  refused(s.scene.cameras, bad, s.metrics,
          "the solution's calibration's sensors");
  // recon's writer refuses a calibration whose world is no sensor.
  bad = s.solution;
  bad.calibration.world.sensor = "nobody";
  core::Result<ResultFiles> files =
      format_results(s.scene.cameras, bad, s.metrics);
  EXPECT_EQ(files.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(files.status().message().rfind("format_results: ", 0), 0u);
  EXPECT_NE(files.status().message().find("nobody"), std::string::npos)
      << files.status().message();
}

// --- The world
// ----------------------------------------------------------------

// The solved scene's world at the floor tag, which its cameras see in 3
// captures at 0.3 px, the outputs anchored to it, and the boards on its floor.
struct Tagged {
  ResultWorld world;
  solve::AnchoredRig rig;

  Tagged() {
    const Solved& s = solved();
    std::vector<solve::TagView> views;
    for (const test::MarkerSighting& m :
         test::observe_marker(s.scene.rig, kOriginTag.id,
                              test::floor_tag_corners(), 3, {0.3, 7})) {
      const test::SyntheticCamera& cam = s.scene.rig[m.camera];
      views.push_back({cam.id, m.capture, cam.model.size, m.marker});
    }
    world.origin = value_of(solve::origin_tag_frame(s.scene.cameras, s.solution,
                                                    views, kOriginTag));
    rig = value_of(solve::anchor_rig(s.solution, world.origin));
    world.floor_boards = value_of(
        floor_boards(s.scene.board, s.solution.boards, rig.boards_to_world));
  }
};

const Tagged& tagged() {
  static const Tagged once;
  return once;
}

json rows_of(const camera::Mat4d& m) {
  json rows = json::array();
  for (int row = 0; row < 4; ++row) {
    rows.push_back({m[0][row], m[1][row], m[2][row], m[3][row]});
  }
  return rows;
}

// The rig's tag, as the report names the tag sought, and how detect_tag
// refines its corners.
const json kRigsTag = json::parse(R"({"dictionary": "DICT_APRILTAG_36h11",
    "family": "tag36h11", "id": 0, "edge_mm": 150.0, "refinement": "subpix"})");

// A tag world: the calibration file names the tag and poses each camera in
// its world, as recon reads it back; the report's world section holds the
// tag world's every figure, each camera's height and the boards on the floor,
// and says its uncertainties are the reference camera's.
TEST(Results, TheTagWorldIsTheFilesAndTheReportsWorld) {
  const Solved& s = solved();
  const Tagged& t = tagged();
  ASSERT_TRUE(t.world.origin.tag) << t.world.origin.reason;
  const solve::TagWorld& w = *t.world.origin.tag;
  ASSERT_FALSE(t.world.floor_boards.empty());
  const ResultFiles files = value_of(format_results(
      s.scene.cameras, s.solution, s.metrics, nullptr, {}, t.world));

  const camera::ArrayCalibration back =
      value_of(camera::parse_array_calibration(files.array_calibration));
  EXPECT_TRUE(back.world.sensor.empty());
  ASSERT_TRUE(back.world.apriltag);
  EXPECT_EQ(back.world.apriltag->family, "tag36h11");
  EXPECT_EQ(back.world.apriltag->id, 0);
  EXPECT_EQ(back.world.apriltag->size_m, 0.15);
  for (std::size_t c = 0; c < s.scene.cameras.size(); ++c) {
    const std::string& id = s.scene.cameras[c].id;
    const camera::SensorCalibration* sensor = camera::find_sensor(back, id);
    ASSERT_NE(sensor, nullptr) << id;
    // T_cw in the world, as anchor_rig gives it, to the Rodrigues round-off.
    const camera::Mat4d read = camera::rigid_inverse(*sensor->color_to_world);
    const camera::Mat4d anchored =
        camera::rigid_inverse(t.rig.color_to_world[c]);
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        EXPECT_NEAR(read[col][row], anchored[col][row], 1e-12) << id;
      }
    }
  }

  const json doc = json::parse(files.report);
  const nlohmann::ordered_json ordered =
      nlohmann::ordered_json::parse(files.report);
  const json& world = doc["world"];
  EXPECT_EQ(
      keys_of(ordered["world"]),
      (std::vector<std::string>{
          "origin", "reference", "uncertainty_frame", "reference_to_world",
          "tag", "miss", "miss_camera", "reason", "located", "floor_boards"}));
  EXPECT_EQ(world["origin"], "tag");
  EXPECT_EQ(world["reference"], s.scene.cameras[0].id);
  EXPECT_EQ(world["uncertainty_frame"], "reference");
  EXPECT_EQ(world["reference_to_world"], rows_of(w.reference_to_world));
  EXPECT_EQ(world["tag"], kRigsTag);
  EXPECT_TRUE(world["miss"].is_null());
  EXPECT_TRUE(world["miss_camera"].is_null());
  EXPECT_TRUE(world["reason"].is_null());

  const json& located = world["located"];
  EXPECT_EQ(keys_of(ordered["world"]["located"]),
            (std::vector<std::string>{
                "rms_px", "max_px", "solves", "edges_mm", "mean_edge_mm",
                "edge_error_percent", "edge_warning", "diagonals_mm",
                "out_of_plane_mm", "corners_m", "cameras", "views"}));
  EXPECT_EQ(located["rms_px"].get<double>(), w.rms_px);
  EXPECT_EQ(located["max_px"].get<double>(), w.worst_px);
  EXPECT_EQ(located["solves"], w.solves);
  ASSERT_EQ(located["edges_mm"].size(), 4u);
  for (std::size_t e = 0; e < 4; ++e) {
    EXPECT_EQ(located["edges_mm"][e].get<double>(), 1000.0 * w.edges_m[e]);
  }
  EXPECT_EQ(located["mean_edge_mm"].get<double>(), 1000.0 * w.edge_m);
  EXPECT_NEAR(located["mean_edge_mm"].get<double>(), 150.0, 1.0);
  EXPECT_EQ(located["edge_error_percent"].get<double>(), w.edge_error_percent);
  EXPECT_EQ(located["edge_warning"], w.edge_warning);
  ASSERT_EQ(located["diagonals_mm"].size(), 2u);
  EXPECT_EQ(located["diagonals_mm"][1].get<double>(),
            1000.0 * w.diagonals_m[1]);
  EXPECT_EQ(located["out_of_plane_mm"].get<double>(),
            1000.0 * w.out_of_plane_m);
  ASSERT_EQ(located["corners_m"].size(), 4u);
  EXPECT_EQ(located["corners_m"][2][0].get<double>(), w.corners_m[2].x);
  ASSERT_EQ(located["cameras"].size(), s.scene.cameras.size());
  for (const solve::TagCamera& c : w.cameras) {
    EXPECT_EQ(
        keys_of(ordered["world"]["located"]["cameras"][c.id]),
        (std::vector<std::string>{"views", "used", "dropped", "height_m"}));
    const json& j = located["cameras"][c.id];
    EXPECT_EQ(j["views"], c.views);
    EXPECT_EQ(j["used"], c.used);
    EXPECT_EQ(j["dropped"], c.views - c.used);
    EXPECT_EQ(j["height_m"].get<double>(), c.height_m);
    EXPECT_GT(c.height_m, 1.0) << c.id;  // the rig's cameras are 2.5 m up
  }
  ASSERT_EQ(located["views"].size(), w.views.size());
  for (std::size_t v = 0; v < w.views.size(); ++v) {
    const solve::TagViewFit& fit = w.views[v];
    const json& j = located["views"][v];
    EXPECT_EQ(keys_of(ordered["world"]["located"]["views"][v]),
              (std::vector<std::string>{"camera", "capture", "used", "rms_px",
                                        "worst_px", "round", "bound_px"}));
    EXPECT_EQ(j["camera"], fit.camera);
    EXPECT_EQ(j["capture"], fit.capture);
    EXPECT_EQ(j["used"], fit.used);
    EXPECT_EQ(j["rms_px"].get<double>(), fit.rms_px);
    EXPECT_EQ(j["worst_px"].get<double>(), fit.worst_px);
    EXPECT_EQ(j["round"], fit.round);
    EXPECT_EQ(j["bound_px"].get<double>(), fit.bound_px);
  }

  ASSERT_EQ(world["floor_boards"].size(), t.world.floor_boards.size());
  for (std::size_t b = 0; b < t.world.floor_boards.size(); ++b) {
    const FloorBoard& board = t.world.floor_boards[b];
    const json& j = world["floor_boards"][b];
    EXPECT_EQ(j.size(), 3u);
    EXPECT_EQ(j["capture"], board.capture);
    EXPECT_EQ(j["angle_deg"].get<double>(), board.angle_deg);
    EXPECT_EQ(j["height_mm"].get<double>(), 1000.0 * board.height_m);
  }
  // The uncertainties and the centres stay in the reference's gauge.
  EXPECT_EQ(doc["geometry"], json::parse(formatted(s).report)["geometry"]);
  EXPECT_EQ(doc["covariance"], json::parse(formatted(s).report)["covariance"]);
}

// Without a tag world, for each miss: the reference camera is the world, the
// report says why, and the calibration file is the solution's own, to the
// byte, as it was before the world was named.
TEST(Results, WithoutATagWorldTheReferenceStaysAndTheReportSaysWhy) {
  const Solved& s = solved();
  const std::string own =
      value_of(camera::format_array_calibration(s.solution.calibration));
  struct Miss {
    solve::TagMiss miss;
    const char* name;
    std::string camera;
  };
  const std::string id = s.scene.cameras[2].id;
  for (const Miss& m :
       {Miss{solve::TagMiss::NotSeen, "not_seen", ""},
        Miss{solve::TagMiss::OneCamera, "one_camera", id},
        Miss{solve::TagMiss::FewCameras, "few_cameras", ""},
        Miss{solve::TagMiss::CameraBelowTag, "camera_below_tag", id},
        Miss{solve::TagMiss::Numerical, "numerical", ""},
        Miss{solve::TagMiss::PoorFit, "poor_fit", ""}}) {
    SCOPED_TRACE(m.name);
    ResultWorld world;
    world.origin.sought = kOriginTag;
    world.origin.miss = m.miss;
    world.origin.camera = m.camera;
    world.origin.reason = std::string("because ") + m.name;
    const ResultFiles files = value_of(format_results(
        s.scene.cameras, s.solution, s.metrics, nullptr, {}, world));
    EXPECT_EQ(files.array_calibration, own);
    const json report = json::parse(files.report)["world"];
    EXPECT_EQ(report["origin"], "reference");
    EXPECT_EQ(report["reference"], s.scene.cameras[0].id);
    EXPECT_EQ(report["uncertainty_frame"], "reference");
    EXPECT_EQ(report["reference_to_world"], rows_of(camera::Mat4d(1.0)));
    EXPECT_EQ(report["tag"], kRigsTag);
    EXPECT_EQ(report["miss"], m.name);
    if (m.camera.empty()) {
      EXPECT_TRUE(report["miss_camera"].is_null());
    } else {
      EXPECT_EQ(report["miss_camera"], m.camera);
    }
    EXPECT_EQ(report["reason"], std::string("because ") + m.name);
    EXPECT_TRUE(report["located"].is_null());
    EXPECT_TRUE(report["floor_boards"].empty());
  }

  // The default world: no tag sought, and the same file.
  const ResultFiles files = formatted(s);
  EXPECT_EQ(files.array_calibration, own);
  const json report = json::parse(files.report)["world"];
  EXPECT_EQ(report["origin"], "reference");
  EXPECT_TRUE(report["tag"].is_null());
  EXPECT_EQ(report["miss"], "not_seen");
  EXPECT_TRUE(report["reason"].is_null());
}

// A tag world of another rig, or one recon's file cannot name, is refused.
TEST(Results, RefusesATagWorldThatDoesNotFit) {
  const Solved& s = solved();
  const auto refused = [&](const ResultWorld& world, const std::string& says) {
    SCOPED_TRACE(says);
    core::Result<ResultFiles> files = ResultFiles{};
    EXPECT_NO_THROW(files = format_results(s.scene.cameras, s.solution,
                                           s.metrics, nullptr, {}, world));
    test::expect_refused(files.status(), "format_results", says);
  };
  ResultWorld fewer = tagged().world;
  fewer.origin.tag->cameras.pop_back();
  refused(fewer, "the tag world does not fit the rig's 4 cameras");
  ResultWorld renamed = tagged().world;
  renamed.origin.tag->cameras[1].id = "SYN-other";
  refused(renamed, "the tag world does not fit the rig's 4 cameras");
  ResultWorld stranger = tagged().world;
  stranger.origin.tag->views[0].camera = "SYN-other";
  refused(stranger, "the tag world does not fit the rig's 4 cameras");
  ResultWorld aruco = tagged().world;
  aruco.origin.sought.dictionary = MarkerDictionary::ArUco4x4_50;
  refused(aruco, "anchor_rig: the tag world's tag sought, of DICT_4X4_50");
}

// Each camera's depth offset whose plane verdict passed is in the calibration
// file, as recon reads it back, version 3; none for a camera whose plane did
// not pass, or whose offset is not a number; none at all, the file as without
// the checks (version 2), when no camera passed. In a tag world as in the
// reference's.
TEST(Results, TheCalibrationFileCarriesEachDepthOffsetThatPassed) {
  const Solved& s = solved();
  const std::vector<solve::RigCamera>& cams = s.scene.cameras;
  ASSERT_EQ(cams.size(), 4u);
  DepthSummary d = depth_checks();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::pair<double, DepthVerdict>> each = {
      {0.00417, DepthVerdict::Ok},
      {-0.00096, DepthVerdict::Ok},
      {0.0123, DepthVerdict::Off},
      {nan, DepthVerdict::Ok}};
  for (std::size_t c = 0; c < cams.size(); ++c) {
    d.cameras[c].depth_offset_m = each[c].first;
    d.cameras[c].plane = each[c].second;
  }
  const ResultWorld reference;
  for (const ResultWorld* world : {&reference, &tagged().world}) {
    SCOPED_TRACE(world->origin.tag ? "tag world" : "reference world");
    const ResultFiles files = value_of(
        format_results(cams, s.solution, s.metrics, &d, {}, *world));
    const json doc = json::parse(files.array_calibration);
    EXPECT_EQ(doc["version"], 3);
    const camera::ArrayCalibration back =
        value_of(camera::parse_array_calibration(files.array_calibration));
    for (std::size_t c = 0; c < cams.size(); ++c) {
      const camera::SensorCalibration* sensor =
          camera::find_sensor(back, cams[c].id);
      ASSERT_NE(sensor, nullptr);
      const bool written = c < 2;
      EXPECT_EQ(sensor->depth_offset_m, written ? each[c].first : 0.0)
          << cams[c].id;
      EXPECT_EQ(doc["device_calibration"][cams[c].id].contains(
                    "depth_offset_m"),
                written)
          << cams[c].id;
    }
  }
  for (CameraDepth& cam : d.cameras) cam.plane = DepthVerdict::Off;
  const ResultFiles none =
      value_of(format_results(cams, s.solution, s.metrics, &d));
  EXPECT_EQ(json::parse(none.array_calibration)["version"], 2);
  EXPECT_EQ(none.array_calibration,
            value_of(format_results(cams, s.solution, s.metrics))
                .array_calibration);
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
