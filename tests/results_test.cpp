// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A solved rig's results files: recon's calibration file posed by each
// camera's T_cw and read back by recon; the report parsed back with
// nlohmann, every section, member and number as the metrics and the solution
// hold it; the residuals CSV, every corner; and refusals.

#include "volumetric_kit/calib/quality/results.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "synthetic_scene.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;
using Code = core::Status::Code;
using json = nlohmann::json;

template <class T>
T value_of(core::Result<T> result) {
  if (!result.ok()) throw std::runtime_error(result.status().message());
  return *std::move(result);
}

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
  EXPECT_EQ(
      keys_of(ordered),
      (std::vector<std::string>{
          "format", "version", "settings", "solver", "tree", "reprojection",
          "rejections", "transfer", "leave_one_out", "covariance",
          "sensitivity", "principal_point_ab", "geometry"}));
  EXPECT_EQ(doc["format"], "volumetric_kit_calib.report");
  EXPECT_EQ(doc["version"], 1);

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
  EXPECT_EQ(settings["metrics"],
            json::parse(R"({"principal_shift_px": 1.5, "histogram_bin_px": 0.1,
                            "histogram_bins": 8})"));

  const json& solver = doc["solver"];
  EXPECT_EQ(solver.size(), 6u);
  EXPECT_EQ(solver["solves"], solution.solves);
  EXPECT_EQ(solver["iterations"], solution.iterations);
  EXPECT_EQ(solver["initial_energy"].get<double>(), solution.initial_energy);
  EXPECT_EQ(solver["final_energy"].get<double>(), solution.final_energy);
  EXPECT_EQ(solver["variance"].get<double>(), solution.variance);
  EXPECT_EQ(solver["seconds"].get<double>(), solution.seconds);
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
  EXPECT_EQ(principal["verdict"], to_string(ab.verdict));

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

}  // namespace
}  // namespace volumetric_kit::calib::quality
