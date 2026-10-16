// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A solved rig's results files: recon's calibration file posed by each
// camera's T_cw and read back by recon; the report parsed back with
// nlohmann, every section and number as the metrics hold it; the residuals
// CSV, every corner; and refusals.

#include "volumetric_kit/calib/quality/results.hpp"

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
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

// One solved scene, with outliers so that something is left out, measured.
struct Solved {
  test::Scene scene = test::make_scene(10);
  MetricsOptions options;
  solve::InitialRig start;
  solve::RigSolution solution;
  CalibrationMetrics metrics;

  Solved() {
    for (DetectedPoint& p : scene.views[4].points) p.pixel.x += 6.0;
    scene.views[7].points[2].pixel.y += 9.0;
    options.histogram_bins = 8;
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

ResultFiles formatted(const Solved& s) {
  core::Result<ResultFiles> files = ResultFiles{};
  EXPECT_NO_THROW(files = format_results(s.scene.cameras, s.solution, s.metrics,
                                         s.options));
  return value_of(std::move(files));
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

void expect_stats(const json& j, const ErrorStats& s) {
  EXPECT_EQ(j["count"].get<std::size_t>(), s.count);
  EXPECT_EQ(j["rms_px"].get<double>(), s.rms_px);
  EXPECT_EQ(j["p90_px"].get<double>(), s.p90_px);
  EXPECT_EQ(j["max_px"].get<double>(), s.max_px);
  EXPECT_EQ(j["mean_px"][0].get<double>(), s.mean_px.x);
  EXPECT_EQ(j["mean_px"][1].get<double>(), s.mean_px.y);
}

// Every section, in order, its numbers exactly the metrics' and the
// solution's; cameras by id.
TEST(Results, TheReportHasItsSchema) {
  const Solved& s = solved();
  const ResultFiles files = formatted(s);
  const json doc = json::parse(files.report);
  const std::vector<std::string> sections = {
      "format",        "version",      "settings",    "solver",
      "tree",          "reprojection", "rejections",  "transfer",
      "leave_one_out", "covariance",   "sensitivity", "principal_point_ab",
      "geometry"};
  const nlohmann::ordered_json ordered =
      nlohmann::ordered_json::parse(files.report);
  std::vector<std::string> keys;
  for (const auto& item : ordered.items()) keys.push_back(item.key());
  EXPECT_EQ(keys, sections);
  EXPECT_EQ(doc["format"], "volumetric_kit_calib.report");
  EXPECT_EQ(doc["version"], 1);
  const std::vector<solve::RigCamera>& cams = s.scene.cameras;
  const std::size_t k = cams.size();

  const json& settings = doc["settings"];
  EXPECT_EQ(settings["reference"], cams[0].id);
  ASSERT_EQ(settings["cameras"].size(), k);
  EXPECT_EQ(settings["cameras"][2], cams[2].id);
  EXPECT_EQ(settings["initialise"]["oblique_deg"].get<double>(),
            s.options.initialise.oblique_deg);
  EXPECT_EQ(settings["bundle_adjust"]["huber_px"].get<double>(),
            s.options.bundle_adjust.huber_px);
  EXPECT_EQ(settings["bundle_adjust"]["max_captures"].get<std::size_t>(),
            s.options.bundle_adjust.max_captures);
  EXPECT_EQ(settings["metrics"]["histogram_bins"], 8);
  EXPECT_EQ(settings["metrics"]["principal_shift_px"].get<double>(), 2.5);

  const json& solver = doc["solver"];
  EXPECT_EQ(solver["solves"], s.solution.solves);
  EXPECT_EQ(solver["iterations"], s.solution.iterations);
  EXPECT_EQ(solver["final_energy"].get<double>(), s.solution.final_energy);
  EXPECT_EQ(solver["variance"].get<double>(), s.solution.variance);
  ASSERT_EQ(doc["tree"].size(), s.solution.links.size());
  EXPECT_EQ(doc["tree"][0]["camera"], cams[s.solution.links[0].camera].id);
  EXPECT_EQ(doc["tree"][0]["parent"], cams[s.solution.links[0].parent].id);
  EXPECT_EQ(doc["tree"][0]["shared"], s.solution.links[0].shared);
  EXPECT_EQ(doc["tree"][0]["inliers"], s.solution.links[0].inliers);

  const json& reprojection = doc["reprojection"];
  for (const char* state : {"initial", "solved"}) {
    SCOPED_TRACE(state);
    const ReprojectionStats& r = std::string(state) == "initial"
                                     ? s.metrics.reprojection.initial
                                     : s.metrics.reprojection.solved;
    const json& j = reprojection[state];
    expect_stats(j["overall"], r.overall);
    expect_stats(j["without_oblique"], r.without_oblique);
    EXPECT_EQ(j["legacy_rmse_px"].get<double>(), r.legacy_rmse_px);
    EXPECT_EQ(j["histogram"]["bin_px"].get<double>(), r.histogram.bin_px);
    EXPECT_EQ(j["histogram"]["counts"].get<std::vector<std::size_t>>(),
              r.histogram.counts);
    for (std::size_t c = 0; c < k; ++c) {
      expect_stats(j["cameras"][cams[c].id], r.cameras[c]);
    }
    ASSERT_EQ(j["captures"].size(), r.captures.size());
    EXPECT_EQ(j["captures"][1]["capture"], r.captures[1].capture);
    expect_stats(j["captures"][1], r.captures[1].errors);
    ASSERT_EQ(j["views"].size(), r.views.size());
    EXPECT_EQ(j["views"][3]["camera"], cams[r.views[3].camera].id);
    EXPECT_EQ(j["views"][3]["capture"], r.views[3].capture);
    EXPECT_EQ(j["views"][3]["oblique"], r.views[3].oblique);
    expect_stats(j["views"][3], r.views[3].errors);
  }
  EXPECT_EQ(reprojection["legacy_reduction_percent"].get<double>(),
            s.metrics.reprojection.legacy_reduction_percent);

  const json& rejections = doc["rejections"];
  ASSERT_FALSE(s.solution.rejections.empty());
  ASSERT_EQ(rejections["counts"].size(), s.metrics.rejections.size());
  EXPECT_EQ(rejections["counts"][0]["reason"],
            solve::to_string(s.metrics.rejections[0].reason));
  EXPECT_EQ(rejections["counts"][0]["corners"],
            s.metrics.rejections[0].corners);
  ASSERT_EQ(rejections["list"].size(), s.solution.rejections.size());
  for (std::size_t i = 0; i < s.solution.rejections.size(); ++i) {
    const solve::Rejection& r = s.solution.rejections[i];
    const json& j = rejections["list"][i];
    EXPECT_EQ(j["camera"], cams[r.camera].id);
    EXPECT_EQ(j["capture"], r.capture);
    EXPECT_EQ(j["corner"], r.corner);
    EXPECT_EQ(j["round"], r.round);
    EXPECT_EQ(j["reason"], solve::to_string(r.reason));
    EXPECT_EQ(j["value"].get<double>(), r.value);
    EXPECT_EQ(j["bound"].get<double>(), r.bound);
  }

  const Transfer& t = s.metrics.transfer;
  const json& transfer = doc["transfer"];
  EXPECT_EQ(transfer["median_px"].get<double>(), t.median_px);
  EXPECT_EQ(transfer["p90_px"].get<double>(), t.p90_px);
  EXPECT_EQ(transfer["worst_px"].get<double>(), t.worst_px);
  ASSERT_EQ(transfer["errors"].size(), t.errors.size());
  EXPECT_EQ(transfer["errors"][5]["capture"], t.errors[5].capture);
  EXPECT_EQ(transfer["errors"][5]["from"], cams[t.errors[5].from].id);
  EXPECT_EQ(transfer["errors"][5]["to"], cams[t.errors[5].to].id);
  EXPECT_EQ(transfer["errors"][5]["error_px"].get<double>(),
            t.errors[5].error_px);
  std::size_t pairs = 0;
  for (const json& p : transfer["pairs"]) {
    const std::size_t from = test::index_of(cams, p["from"].get<std::string>());
    const std::size_t to = test::index_of(cams, p["to"].get<std::string>());
    EXPECT_EQ(p["count"], t.pairs[from][to].count);
    EXPECT_EQ(p["mean_px"].get<double>(), t.pairs[from][to].mean_px);
    ++pairs;
  }
  std::size_t want_pairs = 0;
  for (const auto& row : t.pairs) {
    for (const PairTransfer& p : row) want_pairs += p.count > 0 ? 1 : 0;
  }
  EXPECT_EQ(pairs, want_pairs);

  const LeaveOneOut& loo = s.metrics.leave_one_out;
  const json& left_out = doc["leave_one_out"];
  for (std::size_t c = 0; c < k; ++c) {
    const json& j = left_out["cameras"][cams[c].id];
    EXPECT_EQ(j["centre_mm"].get<double>(), loo.cameras[c].centre_mm);
    EXPECT_EQ(j["rotation_deg"].get<double>(), loo.cameras[c].rotation_deg);
    if (loo.cameras[c].centre_capture) {
      EXPECT_EQ(j["centre_capture"], *loo.cameras[c].centre_capture);
    } else {
      EXPECT_TRUE(j["centre_capture"].is_null());
    }
  }
  EXPECT_TRUE(left_out["cameras"][cams[0].id]["rotation_capture"].is_null());
  ASSERT_EQ(left_out["captures"].size(), loo.captures.size());
  const json& first = left_out["captures"][0];
  EXPECT_EQ(first["capture"], loo.captures[0].capture);
  EXPECT_TRUE(first["refused"].is_null());
  EXPECT_EQ(first["cameras"][cams[1].id]["centre_mm"].get<double>(),
            loo.captures[0].cameras[1].centre_mm);

  for (std::size_t c = 0; c < k; ++c) {
    const json& j = doc["covariance"][cams[c].id];
    const solve::PoseCovariance& cov = s.solution.covariance[c];
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
  const json& sensitivity = doc["sensitivity"];
  EXPECT_EQ(sensitivity["focal_fraction"].get<double>(), 0.003);
  EXPECT_EQ(sensitivity["principal_px"].get<double>(), 2.0);
  const solve::PoseShift& shift = s.solution.sensitivity[2].principal_y[1];
  const json& j = sensitivity["cameras"][cams[2].id]["principal_y"][cams[1].id];
  EXPECT_EQ(j["rotation_deg"].get<double>(), shift.rotation_deg);
  EXPECT_EQ(j["centre_mm"][2].get<double>(), shift.centre_mm.z);

  const PrincipalPointAB& ab = s.metrics.principal_point;
  const json& principal = doc["principal_point_ab"];
  EXPECT_EQ(principal["shift_px"].get<double>(), ab.shift_px);
  EXPECT_EQ(principal["corners"], ab.corners);
  EXPECT_EQ(principal["as_given"]["rms_px"].get<double>(), ab.as_given.rms_px);
  EXPECT_EQ(principal["shifted"]["transfer_p90_px"].get<double>(),
            ab.shifted.transfer_p90_px);
  EXPECT_EQ(principal["verdict"], to_string(ab.verdict));

  const json& geometry = doc["geometry"];
  EXPECT_EQ(geometry["centres_m"][cams[3].id][1].get<double>(),
            s.metrics.geometry.centres_m[3].y);
  ASSERT_EQ(geometry["baselines_m"].size(), k * (k - 1) / 2);
  EXPECT_EQ(geometry["baselines_m"][0]["a"], cams[0].id);
  EXPECT_EQ(geometry["baselines_m"][0]["b"], cams[1].id);
  EXPECT_EQ(geometry["baselines_m"][0]["m"].get<double>(),
            s.metrics.geometry.baselines_m[0][1]);
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
  const ResultFiles files =
      value_of(format_results(s.scene.cameras, s.solution, metrics, s.options));
  const json doc = json::parse(files.report);
  EXPECT_TRUE(doc["reprojection"]["legacy_reduction_percent"].is_null());
  EXPECT_TRUE(doc["transfer"]["worst_px"].is_null());
  EXPECT_EQ(doc["leave_one_out"]["captures"][0]["refused"],
            "bundle_adjust: too few");
  EXPECT_TRUE(doc["leave_one_out"]["captures"][0]["cameras"].empty());
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
  solution.calibration.sensors[1].id = cameras[1].id;
  const ResultFiles files =
      value_of(format_results(cameras, solution, s.metrics, s.options));
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
  // recon's writer refuses a calibration with a sensor twice.
  bad = s.solution;
  bad.calibration.sensors[1].id = bad.calibration.sensors[0].id;
  core::Result<ResultFiles> files =
      format_results(s.scene.cameras, bad, s.metrics);
  EXPECT_EQ(files.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(files.status().message().rfind("format_results: ", 0), 0u);
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
