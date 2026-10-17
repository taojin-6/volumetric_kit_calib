// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/results.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "../core/json_text.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/usable_for_pose.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;
using Writer = detail::JsonWriter;

// Refuses a solution or metrics that index cameras the rig does not have,
// or hold another count of them, or a calibration of other sensors.
core::Status check_fits(const std::vector<std::string>& ids,
                        const solve::RigSolution& s,
                        const CalibrationMetrics& m) {
  const std::size_t k = ids.size();
  const auto refuse = [&](const char* what) {
    return core::Status::invalid_argument(std::string(what) +
                                          " do not fit the rig's " +
                                          std::to_string(k) + " cameras");
  };
  const auto square = [&](const auto& rows) {
    if (rows.size() != k) return false;
    for (const auto& row : rows) {
      if (row.size() != k) return false;
    }
    return true;
  };
  if (s.reference >= k || s.world_to_camera.size() != k) {
    return refuse("the solution's poses");
  }
  if (s.covariance.size() != k) return refuse("the solution's covariances");
  if (s.sensitivity.size() != k) return refuse("the solution's sensitivities");
  for (const solve::IntrinsicsSensitivity& off : s.sensitivity) {
    if (off.focal.size() != k || off.principal_x.size() != k ||
        off.principal_y.size() != k) {
      return refuse("the solution's sensitivities");
    }
  }
  for (const solve::CornerResidual& r : s.residuals) {
    if (r.camera >= k) return refuse("the solution's residuals");
  }
  for (const solve::Rejection& r : s.rejections) {
    if (r.camera >= k) return refuse("the solution's rejections");
  }
  for (const solve::PlacedLink& link : s.links) {
    if (link.camera >= k || link.parent >= k) {
      return refuse("the solution's links");
    }
  }
  std::set<std::string> sensors;
  for (const camera::SensorCalibration& sensor : s.calibration.sensors) {
    sensors.insert(sensor.id);
  }
  if (s.calibration.sensors.size() != k ||
      sensors != std::set<std::string>(ids.begin(), ids.end())) {
    return refuse("the solution's calibration's sensors");
  }
  for (const solve::ViewRecord& view : m.views_left_out) {
    if (view.camera >= k) return refuse("the metrics' views left out");
  }
  for (const ReprojectionStats* at :
       {&m.reprojection.initial, &m.reprojection.solved}) {
    if (at->cameras.size() != k) return refuse("the metrics' reprojections");
    for (const ViewErrors& view : at->views) {
      if (view.camera >= k) return refuse("the metrics' reprojections");
    }
  }
  if (!square(m.transfer.pairs)) return refuse("the metrics' transfers");
  for (const TransferError& e : m.transfer.errors) {
    if (e.from >= k || e.to >= k) return refuse("the metrics' transfers");
  }
  if (m.leave_one_out.cameras.size() != k) {
    return refuse("the metrics' leave-one-out re-solves");
  }
  for (const LeftOut& left : m.leave_one_out.captures) {
    if (!left.cameras.empty() && left.cameras.size() != k) {
      return refuse("the metrics' leave-one-out re-solves");
    }
  }
  if (m.geometry.centres_m.size() != k || !square(m.geometry.baselines_m)) {
    return refuse("the metrics' geometries");
  }
  return {};
}

// Refuses depth checks that index cameras the rig does not have, or hold
// another count of them.
core::Status check_depth_fits(std::size_t k, const DepthSummary& d) {
  const auto pairs_fit = [&](const std::vector<PairCheck>& pairs) {
    for (const PairCheck& p : pairs) {
      if (p.agreement.from >= k || p.agreement.to >= k) return false;
    }
    return true;
  };
  bool fits = d.cameras.size() == k && pairs_fit(d.pairs);
  for (const BoardDepthCheck& view : d.views) fits = fits && view.camera < k;
  for (const CapturePairs& capture : d.captures) {
    fits = fits && pairs_fit(capture.pairs);
  }
  if (fits) return {};
  return core::Status::invalid_argument(
      "the depth checks do not fit the rig's " + std::to_string(k) +
      " cameras");
}

void pair(Writer& w, const camera::Vec2d& v) {
  w.open_array(true);
  w.value(v.x);
  w.value(v.y);
  w.close_array();
}

void triple(Writer& w, const camera::Vec3d& v) {
  w.open_array(true);
  w.value(v.x);
  w.value(v.y);
  w.value(v.z);
  w.close_array();
}

template <class T>
void value_or_null(Writer& w, const std::optional<T>& value) {
  if (value) {
    w.value(*value);
  } else {
    w.null();
  }
}

// An ErrorStats' members, in an object already open.
void stats_members(Writer& w, const ErrorStats& s) {
  w.key("count").value(s.count);
  w.key("rms_px").value(s.rms_px);
  w.key("p90_px").value(s.p90_px);
  w.key("max_px").value(s.max_px);
  pair(w.key("mean_px"), s.mean_px);
}

void stats(Writer& w, const ErrorStats& s) {
  w.open_object();
  stats_members(w, s);
  w.close_object();
}

void reprojection(Writer& w, const ReprojectionStats& s,
                  const std::vector<std::string>& ids) {
  w.open_object();
  stats(w.key("overall"), s.overall);
  stats(w.key("without_oblique"), s.without_oblique);
  w.key("legacy_rmse_px").value(s.legacy_rmse_px);
  w.key("histogram").open_array(true);
  for (const std::size_t count : s.histogram.counts) w.value(count);
  w.close_array();
  w.key("cameras").open_object();
  for (std::size_t c = 0; c < ids.size(); ++c)
    stats(w.key(ids[c]), s.cameras[c]);
  w.close_object();
  w.key("captures").open_array();
  for (const CaptureErrors& capture : s.captures) {
    w.open_object();
    w.key("capture").value(capture.capture);
    stats_members(w, capture.errors);
    w.close_object();
  }
  w.close_array();
  w.key("views").open_array();
  for (const ViewErrors& view : s.views) {
    w.open_object();
    w.key("camera").value(ids[view.camera]);
    w.key("capture").value(view.capture);
    w.key("oblique").value(view.oblique);
    stats_members(w, view.errors);
    w.close_object();
  }
  w.close_array();
  w.close_object();
}

void settings(Writer& w, const std::vector<std::string>& ids,
              const solve::RigSolution& solution,
              const MetricsOptions& options) {
  w.open_object();
  w.key("reference").value(ids[solution.reference]);
  w.key("cameras").open_array(true);
  for (const std::string& id : ids) w.value(id);
  w.close_array();
  const solve::InitialiseOptions& init = options.initialise;
  w.key("initialise").open_object();
  w.key("max_pnp_rms_px").value(init.max_pnp_rms_px);
  w.key("min_ambiguity").value(init.min_ambiguity);
  w.key("oblique_deg").value(init.oblique_deg);
  w.key("weak_below").value(init.weak_below);
  w.key("inlier_deg").value(init.inlier_deg);
  w.key("truncation_px").value(init.truncation_px);
  w.close_object();
  const solve::BundleAdjustOptions& adjust = options.bundle_adjust;
  w.key("bundle_adjust").open_object();
  w.key("huber_px").value(adjust.huber_px);
  w.key("min_depth_m").value(adjust.min_depth_m);
  w.key("max_iterations").value(adjust.max_iterations);
  w.key("outlier_rounds").value(adjust.outlier_rounds);
  w.key("outlier_px").value(adjust.outlier_px);
  w.key("outlier_mads").value(adjust.outlier_mads);
  w.key("outlier_view_ratio").value(adjust.outlier_view_ratio);
  w.key("outlier_view_px").value(adjust.outlier_view_px);
  w.key("min_shared").value(adjust.min_shared);
  w.key("max_condition").value(adjust.max_condition);
  w.key("max_captures").value(adjust.max_captures);
  w.key("focal_fraction").value(adjust.focal_fraction);
  w.key("principal_px").value(adjust.principal_px);
  w.close_object();
  w.key("metrics").open_object();
  w.key("principal_shift_px").value(options.principal_shift_px);
  w.key("histogram_bin_px").value(options.histogram_bin_px);
  w.key("histogram_bins").value(options.histogram_bins);
  w.close_object();
  w.close_object();
}

// A re-solve's refusal: null when it ran.
void refused(Writer& w, const std::string& why) {
  if (why.empty()) {
    w.null();
  } else {
    w.value(why);
  }
}

void ratio_stats(Writer& w, const RatioStats& r) {
  w.open_object();
  w.key("count").value(r.count);
  w.key("mean").value(r.mean);
  w.key("sd").value(r.sd);
  w.close_object();
}

void pair_checks(Writer& w, const std::vector<PairCheck>& pairs,
                 const std::vector<std::string>& ids) {
  w.open_array();
  for (const PairCheck& check : pairs) {
    const PairAgreement& p = check.agreement;
    w.open_object();
    w.key("from").value(ids[p.from]);
    w.key("to").value(ids[p.to]);
    w.key("overlap").value(p.overlap);
    w.key("in_window").value(p.in_window);
    w.key("signed_median_mm").value(p.signed_median_mm);
    w.key("median_abs_mm").value(p.median_abs_mm);
    w.key("p90_abs_mm").value(p.p90_abs_mm);
    w.key("verdict").value(to_string(check.verdict));
    w.close_object();
  }
  w.close_array();
}

void depth_section(Writer& w, const DepthSummary& d,
                   const std::vector<std::string>& ids) {
  const DepthCheckOptions& o = d.options;
  w.open_object();
  w.key("settings").open_object();
  w.key("inset_m").value(o.inset_m);
  w.key("seed_mm").value(o.seed_mm);
  w.key("min_gate_mm").value(o.min_gate_mm);
  w.key("sigma_gate").value(o.sigma_gate);
  w.key("min_inliers").value(o.min_inliers);
  w.key("window_mm").value(o.window_mm);
  w.key("bin_mm").value(o.bin_mm);
  w.key("edge_ratio").value(o.edge_ratio);
  w.key("agreement_stride").value(std::size_t{o.agreement_stride});
  w.key("min_overlap").value(o.min_overlap);
  w.key("max_scale_error").value(o.max_scale_error);
  w.key("max_offset_mm").value(o.max_offset_mm);
  w.key("max_angle_deg").value(o.max_angle_deg);
  w.key("max_signed_median_mm").value(o.max_signed_median_mm);
  w.key("max_median_abs_mm").value(o.max_median_abs_mm);
  w.close_object();
  ratio_stats(w.key("ratio"), d.ratio);
  w.key("scale").value(to_string(d.scale));
  w.key("cameras").open_object();
  for (std::size_t c = 0; c < ids.size(); ++c) {
    const CameraDepth& cam = d.cameras[c];
    w.key(ids[c]).open_object();
    w.key("views").value(cam.views);
    ratio_stats(w.key("ratio"), cam.ratio);
    w.key("scale").value(to_string(cam.scale));
    w.key("offset_mm").value(cam.offset_mm);
    w.key("angle_deg").value(cam.angle_deg);
    w.key("plane").value(to_string(cam.plane));
    w.close_object();
  }
  w.close_object();
  w.key("views").open_array();
  for (const BoardDepthCheck& view : d.views) {
    const DepthPlane& plane = view.plane;
    w.open_object();
    w.key("camera").value(ids[view.camera]);
    w.key("capture").value(view.capture);
    w.key("valid").value(plane.valid);
    w.key("members").value(plane.members);
    w.key("out_of_range").value(plane.out_of_range);
    w.key("inliers").value(plane.inliers);
    if (plane.valid) {
      w.key("rms_mm").value(1000.0 * plane.rms_m);
    } else {
      w.key("rms_mm").null();
    }
    w.key("ratios").open_array(true);
    for (const double r : view.ratios) w.value(r);
    w.close_array();
    w.key("offset_mm").value(view.offset_mm);
    w.key("angle_deg").value(view.angle_deg);
    w.close_object();
  }
  w.close_array();
  pair_checks(w.key("pairs"), d.pairs, ids);
  w.key("captures").open_array();
  for (const CapturePairs& capture : d.captures) {
    w.open_object();
    w.key("capture").value(capture.capture);
    pair_checks(w.key("pairs"), capture.pairs, ids);
    w.close_object();
  }
  w.close_array();
  w.close_object();
}

void arm(Writer& w, const PrincipalPointArm& a) {
  w.open_object();
  w.key("rms_px").value(a.rms_px);
  w.key("legacy_rmse_px").value(a.legacy_rmse_px);
  w.key("transfer_median_px").value(a.transfer_median_px);
  w.key("transfer_p90_px").value(a.transfer_p90_px);
  w.close_object();
}

std::string report_text(const std::vector<std::string>& ids,
                        const solve::RigSolution& solution,
                        const CalibrationMetrics& metrics,
                        const DepthSummary* depth_checks) {
  const std::size_t k = ids.size();
  Writer w;
  w.open_object();
  w.key("format").value("volumetric_kit_calib.report");
  w.key("version").value(1);
  settings(w.key("settings"), ids, solution, metrics.options);

  w.key("solver").open_object();
  w.key("solves").value(solution.solves);
  w.key("iterations").value(solution.iterations);
  w.key("initial_energy").value(solution.initial_energy);
  w.key("final_energy").value(solution.final_energy);
  w.key("variance").value(solution.variance);
  w.key("seconds").value(solution.seconds);
  w.close_object();
  w.key("tree").open_array();
  for (const solve::PlacedLink& link : solution.links) {
    w.open_object();
    w.key("camera").value(ids[link.camera]);
    w.key("parent").value(ids[link.parent]);
    w.key("shared").value(link.shared);
    w.key("inliers").value(link.inliers);
    w.key("weak").value(link.weak);
    w.close_object();
  }
  w.close_array();

  w.key("reprojection").open_object();
  reprojection(w.key("initial"), metrics.reprojection.initial, ids);
  reprojection(w.key("solved"), metrics.reprojection.solved, ids);
  w.key("legacy_reduction_percent")
      .value(metrics.reprojection.legacy_reduction_percent);
  w.close_object();

  w.key("rejections").open_object();
  w.key("views_left_out").open_array();
  for (const solve::ViewRecord& view : metrics.views_left_out) {
    w.open_object();
    w.key("camera").value(ids[view.camera]);
    w.key("capture").value(view.capture);
    w.key("fate").value(solve::to_string(view.fate));
    w.key("use").value(compute::to_string(view.use));
    value_or_null(w.key("pnp_rms_px"), view.pnp_rms_px);
    value_or_null(w.key("ambiguity"), view.ambiguity);
    value_or_null(w.key("tilt_deg"), view.tilt_deg);
    w.close_object();
  }
  w.close_array();
  w.key("counts").open_array();
  for (const RejectionCount& count : metrics.rejections) {
    w.open_object();
    w.key("round").value(count.round);
    w.key("reason").value(solve::to_string(count.reason));
    w.key("corners").value(count.corners);
    w.key("views").value(count.views);
    w.close_object();
  }
  w.close_array();
  w.key("list").open_array();
  for (const solve::Rejection& r : solution.rejections) {
    w.open_object();
    w.key("camera").value(ids[r.camera]);
    w.key("capture").value(r.capture);
    w.key("corner").value(r.corner);
    w.key("round").value(r.round);
    w.key("reason").value(solve::to_string(r.reason));
    w.key("value").value(r.value);
    w.key("bound").value(r.bound);
    w.close_object();
  }
  w.close_array();
  w.close_object();

  const Transfer& t = metrics.transfer;
  w.key("transfer").open_object();
  w.key("median_px").value(t.median_px);
  w.key("p90_px").value(t.p90_px);
  w.key("worst_px").value(t.worst_px);
  w.key("pairs").open_array();
  for (std::size_t from = 0; from < k; ++from) {
    for (std::size_t to = 0; to < k; ++to) {
      const PairTransfer& p = t.pairs[from][to];
      if (p.count == 0) continue;
      w.open_object();
      w.key("from").value(ids[from]);
      w.key("to").value(ids[to]);
      w.key("count").value(p.count);
      w.key("mean_px").value(p.mean_px);
      w.close_object();
    }
  }
  w.close_array();
  w.key("errors").open_array();
  for (const TransferError& e : t.errors) {
    w.open_object();
    w.key("capture").value(e.capture);
    w.key("from").value(ids[e.from]);
    w.key("to").value(ids[e.to]);
    w.key("error_px").value(e.error_px);
    w.close_object();
  }
  w.close_array();
  w.close_object();

  const LeaveOneOut& loo = metrics.leave_one_out;
  w.key("leave_one_out").open_object();
  w.key("cameras").open_object();
  for (std::size_t c = 0; c < k; ++c) {
    const CameraStability& s = loo.cameras[c];
    w.key(ids[c]).open_object();
    w.key("centre_mm").value(s.centre_mm);
    value_or_null(w.key("centre_capture"), s.centre_capture);
    w.key("rotation_deg").value(s.rotation_deg);
    value_or_null(w.key("rotation_capture"), s.rotation_capture);
    w.close_object();
  }
  w.close_object();
  w.key("captures").open_array();
  for (const LeftOut& left : loo.captures) {
    w.open_object();
    w.key("capture").value(left.capture);
    refused(w.key("refused"), left.refused);
    w.key("rms_px").value(left.rms_px);
    w.key("legacy_rmse_px").value(left.legacy_rmse_px);
    w.key("cameras").open_object();
    for (std::size_t c = 0; c < left.cameras.size(); ++c) {
      w.key(ids[c]).open_object();
      w.key("centre_mm").value(left.cameras[c].centre_mm);
      w.key("rotation_deg").value(left.cameras[c].rotation_deg);
      w.close_object();
    }
    w.close_object();
    w.close_object();
  }
  w.close_array();
  w.close_object();

  w.key("covariance").open_object();
  for (std::size_t c = 0; c < k; ++c) {
    const solve::PoseCovariance& cov = solution.covariance[c];
    w.key(ids[c]).open_object();
    w.key("rotation_deg").value(cov.rotation_deg);
    w.key("centre_mm").value(cov.centre_mm);
    w.key("matrix").open_array();
    for (int row = 0; row < 6; ++row) {
      w.open_array(true);
      for (int col = 0; col < 6; ++col) w.value(cov.covariance(row, col));
      w.close_array();
    }
    w.close_array();
    w.close_object();
  }
  w.close_object();

  w.key("sensitivity").open_object();
  for (std::size_t j = 0; j < k; ++j) {
    const solve::IntrinsicsSensitivity& s = solution.sensitivity[j];
    w.key(ids[j]).open_object();
    for (const auto& [name, shifts] :
         {std::pair{"focal", &s.focal},
          std::pair{"principal_x", &s.principal_x},
          std::pair{"principal_y", &s.principal_y}}) {
      w.key(name).open_object();
      for (std::size_t c = 0; c < k; ++c) {
        w.key(ids[c]).open_object();
        w.key("rotation_deg").value((*shifts)[c].rotation_deg);
        triple(w.key("centre_mm"), (*shifts)[c].centre_mm);
        w.close_object();
      }
      w.close_object();
    }
    w.close_object();
  }
  w.close_object();

  const PrincipalPointAB& ab = metrics.principal_point;
  w.key("principal_point_ab").open_object();
  refused(w.key("refused"), ab.refused);
  w.key("corners").value(ab.corners);
  arm(w.key("as_given"), ab.as_given);
  arm(w.key("shifted"), ab.shifted);
  w.key("verdict").value(to_string(ab.verdict));
  w.close_object();

  w.key("geometry").open_object();
  w.key("centres_m").open_object();
  for (std::size_t c = 0; c < k; ++c) {
    triple(w.key(ids[c]), metrics.geometry.centres_m[c]);
  }
  w.close_object();
  w.key("baselines_m").open_array();
  for (std::size_t a = 0; a < k; ++a) {
    for (std::size_t b = a + 1; b < k; ++b) {
      w.open_object();
      w.key("a").value(ids[a]);
      w.key("b").value(ids[b]);
      w.key("m").value(metrics.geometry.baselines_m[a][b]);
      w.close_object();
    }
  }
  w.close_array();
  w.close_object();
  if (depth_checks) {
    depth_section(w.key("depth"), *depth_checks, ids);
  } else {
    w.key("depth").null();
  }
  w.close_object();
  return std::move(w).text();
}

// A double as the report writes it; `nan`, `inf` or `-inf` for one not
// finite.
std::string csv_number(double v) {
  if (std::isnan(v)) return "nan";
  if (std::isinf(v)) return v > 0.0 ? "inf" : "-inf";
  return detail::json_number(v);
}

// A text field, quoted when it holds a comma, a quote or a line break.
std::string csv_text(const std::string& text) {
  if (text.find_first_of(",\"\r\n") == std::string::npos) return text;
  std::string quoted = "\"";
  for (const char ch : text) {
    if (ch == '"') quoted += '"';
    quoted += ch;
  }
  return quoted + "\"";
}

std::string residuals_text(const std::vector<std::string>& ids,
                           const solve::RigSolution& solution) {
  std::string text = "capture,serial,corner_id,u,v,du,dv,weight,dropped\n";
  for (const solve::CornerResidual& r : solution.residuals) {
    text += std::to_string(r.capture);
    text += ',';
    text += csv_text(ids[r.camera]);
    text += ',';
    text += std::to_string(r.corner);
    for (const double v :
         {r.pixel.x, r.pixel.y, r.residual.x, r.residual.y, r.weight}) {
      text += ',';
      text += csv_number(v);
    }
    text += r.dropped ? ",1\n" : ",0\n";
  }
  return text;
}

}  // namespace

core::Result<ResultFiles> format_results(
    const std::vector<solve::RigCamera>& cameras,
    const solve::RigSolution& solution, const CalibrationMetrics& metrics,
    const DepthSummary* depth) {
  constexpr const char* kWhat = "format_results";
  return compute::guarded(kWhat, [&]() -> core::Result<ResultFiles> {
    std::vector<std::string> ids;
    ids.reserve(cameras.size());
    for (const solve::RigCamera& cam : cameras) ids.push_back(cam.id);
    core::Status checked = solve::check_camera_ids(ids);
    if (checked.ok()) checked = check_fits(ids, solution, metrics);
    if (checked.ok() && depth) checked = check_depth_fits(ids.size(), *depth);
    if (!checked.ok()) return std::move(checked).with_context(kWhat);
    core::Result<std::string> calibration =
        camera::format_array_calibration(solution.calibration);
    if (!calibration.ok()) return calibration.status().with_context(kWhat);
    ResultFiles files;
    files.array_calibration = *std::move(calibration);
    files.report = report_text(ids, solution, metrics, depth);
    files.residuals = residuals_text(ids, solution);
    return files;
  });
}

}  // namespace volumetric_kit::calib::quality
