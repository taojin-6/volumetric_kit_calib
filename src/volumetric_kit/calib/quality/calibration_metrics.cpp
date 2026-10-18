// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/calibration_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>

#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/compute/usable_for_pose.hpp"
#include "volumetric_kit/calib/solve/view_pose.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;
using Code = core::Status::Code;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();

ErrorStats stats_of(const std::vector<camera::Vec2d>& residuals) {
  ErrorStats s;
  s.count = residuals.size();
  if (residuals.empty()) return s;
  std::vector<double> norms;
  norms.reserve(residuals.size());
  double squared = 0.0;
  camera::Vec2d sum(0.0);
  for (const camera::Vec2d& e : residuals) {
    norms.push_back(glm::length(e));
    squared += glm::dot(e, e);
    sum += e;
  }
  const auto n = static_cast<double>(residuals.size());
  s.rms_px = std::sqrt(squared / n);
  s.p90_px = compute::quantile(norms, 0.9);
  s.max_px = compute::quantile(std::move(norms), 1.0);
  s.mean_px = sum / n;
  return s;
}

// The legacy solve's RMSE: the root of the mean squared residual component,
// x and y alike.
double legacy_rmse(const std::vector<camera::Vec2d>& residuals) {
  if (residuals.empty()) return kNaN;
  double squared = 0.0;
  for (const camera::Vec2d& e : residuals) squared += e.x * e.x + e.y * e.y;
  return std::sqrt(squared / (2.0 * static_cast<double>(residuals.size())));
}

// The RMS and the legacy RMSE of a solution's kept corners.
std::pair<double, double> kept_rms(const solve::RigSolution& solution) {
  std::vector<camera::Vec2d> kept;
  for (const solve::CornerResidual& r : solution.residuals) {
    if (!r.dropped) kept.push_back(r.residual);
  }
  return {stats_of(kept).rms_px, legacy_rmse(kept)};
}

Histogram histogram_of(const std::vector<camera::Vec2d>& residuals,
                       double bin_px, int bins) {
  Histogram h;
  h.bin_px = bin_px;
  const auto last = static_cast<std::size_t>(bins);
  h.counts.assign(last + 1, 0);
  for (const camera::Vec2d& e : residuals) {
    const double at = glm::length(e) / bin_px;
    // Written so a NaN lands in the last bin.
    ++h.counts[at >= 0.0 && at < static_cast<double>(bins)
                   ? static_cast<std::size_t>(at)
                   : last];
  }
  return h;
}

// A camera's centre, `-R^T t` of its T_cw.
camera::Vec3d centre_of(const camera::Mat4d& world_to_camera) {
  return camera::Vec3d(camera::rigid_inverse(world_to_camera)[3]);
}

camera::Vec3d centre_of(const camera::RodriguesTransform& world_to_camera) {
  return centre_of(camera::matrix_from_rodrigues(world_to_camera));
}

bool finite(const camera::RodriguesTransform& t) noexcept {
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(t.rvec[i]) || !std::isfinite(t.tvec[i])) return false;
  }
  return true;
}

std::string capture_name(std::int64_t capture) {
  return "capture " + std::to_string(capture);
}

// The rig's cameras checked, and where each observation's corners sit in the
// solution's residuals.
struct Fit {
  solve::RigLenses rig;
  // Each observation's first residual; kNone for one not adjusted.
  std::vector<std::size_t> first;
};

core::Result<Fit> read_fit(const std::vector<solve::RigCamera>& cameras,
                           const std::vector<Observation>& views,
                           const solve::InitialRig& start,
                           const solve::RigSolution& solution) {
  VKC_ASSIGN(solve::RigLenses rig,
             solve::check_rig(cameras, solution.reference));
  const std::size_t k = cameras.size();
  if (solution.world_to_camera.size() != k) {
    return core::Status::invalid_argument(
        "the solution places " +
        std::to_string(solution.world_to_camera.size()) + " cameras of " +
        std::to_string(k));
  }
  for (std::size_t c = 0; c < k; ++c) {
    if (!finite(solution.world_to_camera[c])) {
      return core::Status::invalid_argument("the solution's camera " +
                                            rig.ids[c] + " is not finite");
    }
  }
  if (start.reference != solution.reference ||
      start.world_to_camera.size() != k) {
    return core::Status::invalid_argument(
        "the start's reference or cameras are not the solution's");
  }
  if (start.views.size() != views.size()) {
    return core::Status::invalid_argument(
        "the start has " + std::to_string(start.views.size()) +
        " view records for " + std::to_string(views.size()) + " observations");
  }
  Fit fit;
  fit.first.assign(views.size(), kNone);
  std::size_t next = 0;
  for (std::size_t i = 0; i < views.size(); ++i) {
    const solve::ViewRecord& record = start.views[i];
    const std::string what = "observation " + std::to_string(i);
    if (record.camera >= k || rig.ids[record.camera] != views[i].camera_id ||
        record.capture != views[i].frame_id) {
      return core::Status::invalid_argument(
          "the start's record of " + what +
          " is not of its camera and capture");
    }
    if (!solve::adjusts(record)) continue;
    fit.first[i] = next;
    for (const DetectedPoint& point : views[i].points) {
      if (next >= solution.residuals.size() ||
          solution.residuals[next].camera != record.camera ||
          solution.residuals[next].capture != record.capture ||
          solution.residuals[next].corner != point.id) {
        return core::Status::invalid_argument(
            "the solution's residuals do not fit " + what);
      }
      ++next;
    }
  }
  if (next != solution.residuals.size()) {
    return core::Status::invalid_argument(
        "the solution has " + std::to_string(solution.residuals.size()) +
        " residuals for " + std::to_string(next) + " corners adjusted");
  }
  fit.rig = std::move(rig);
  return fit;
}

// One corner the adjustment kept, at the first rig and at the solution.
struct Kept {
  std::size_t view = 0;  // its observation
  std::size_t camera = 0;
  std::int64_t capture = 0;
  bool oblique = false;
  camera::Vec2d initial{0.0};
  camera::Vec2d solved{0.0};
};

ReprojectionStats stats_at(const std::vector<Kept>& kept, std::size_t cameras,
                           camera::Vec2d Kept::* at,
                           const MetricsOptions& options) {
  ReprojectionStats s;
  std::vector<camera::Vec2d> all;
  std::vector<camera::Vec2d> not_oblique;
  std::vector<std::vector<camera::Vec2d>> by_camera(cameras);
  std::map<std::int64_t, std::vector<camera::Vec2d>> by_capture;
  for (std::size_t a = 0; a < kept.size();) {
    std::vector<camera::Vec2d> view;
    std::size_t b = a;
    for (; b < kept.size() && kept[b].view == kept[a].view; ++b) {
      view.push_back(kept[b].*at);
    }
    s.views.push_back(
        {kept[a].camera, kept[a].capture, kept[a].oblique, stats_of(view)});
    a = b;
  }
  for (const Kept& corner : kept) {
    const camera::Vec2d& e = corner.*at;
    all.push_back(e);
    if (!corner.oblique) not_oblique.push_back(e);
    by_camera[corner.camera].push_back(e);
    by_capture[corner.capture].push_back(e);
  }
  s.overall = stats_of(all);
  s.without_oblique = stats_of(not_oblique);
  s.legacy_rmse_px = legacy_rmse(all);
  for (const std::vector<camera::Vec2d>& e : by_camera) {
    s.cameras.push_back(stats_of(e));
  }
  for (const auto& [capture, e] : by_capture) {
    s.captures.push_back({capture, stats_of(e)});
  }
  s.histogram =
      histogram_of(all, options.histogram_bin_px, options.histogram_bins);
  return s;
}

core::Result<Reprojection> reprojection_of(
    const Fit& fit, const compute::Board& board,
    const std::vector<Observation>& views, const solve::InitialRig& start,
    const solve::RigSolution& solution, const MetricsOptions& options) {
  const std::size_t k = fit.rig.ids.size();
  std::vector<camera::RodriguesTransform> start_cameras;
  for (std::size_t c = 0; c < k; ++c) {
    core::Status rigid = camera::check_rigid(start.world_to_camera[c]);
    if (!rigid.ok()) {
      return rigid.with_context("the start's camera " + fit.rig.ids[c]);
    }
    start_cameras.push_back(
        camera::rodrigues_from_matrix(start.world_to_camera[c]));
  }
  std::map<std::int64_t, const camera::Mat4d*> start_boards;
  for (const solve::BoardPose& b : start.boards) {
    start_boards.emplace(b.capture, &b.board_to_world);
  }
  std::vector<Kept> kept;
  for (std::size_t i = 0; i < views.size(); ++i) {
    const std::size_t first = fit.first[i];
    if (first == kNone) continue;
    const Observation& view = views[i];
    const std::size_t n = view.points.size();
    bool any = false;
    for (std::size_t j = 0; j < n; ++j) {
      any = any || !solution.residuals[first + j].dropped;
    }
    if (!any) continue;
    const auto board_to_world = start_boards.find(view.frame_id);
    if (board_to_world == start_boards.end()) {
      return core::Status::invalid_argument("the start places no board for " +
                                            capture_name(view.frame_id));
    }
    core::Status rigid = camera::check_rigid(*board_to_world->second);
    if (!rigid.ok()) {
      return rigid.with_context("the start's board of " +
                                capture_name(view.frame_id));
    }
    const solve::ViewRecord& record = start.views[i];
    VKC_ASSIGN(const solve::ViewResiduals at,
               solve::view_residuals(
                   fit.rig.lenses[record.camera], board, view,
                   start_cameras[record.camera],
                   camera::rodrigues_from_matrix(*board_to_world->second)));
    for (std::size_t j = 0; j < n; ++j) {
      const solve::CornerResidual& r = solution.residuals[first + j];
      if (r.dropped) continue;
      kept.push_back({i, record.camera, view.frame_id, record.oblique,
                      at.residuals[j], r.residual});
    }
  }
  Reprojection out;
  out.initial = stats_at(kept, k, &Kept::initial, options);
  out.solved = stats_at(kept, k, &Kept::solved, options);
  if (out.initial.legacy_rmse_px > 0.0) {
    out.legacy_reduction_percent =
        100.0 * (1.0 - out.solved.legacy_rmse_px / out.initial.legacy_rmse_px);
  }
  return out;
}

std::vector<RejectionCount> rejection_counts(
    const solve::RigSolution& solution) {
  std::map<std::pair<int, int>, RejectionCount> counts;
  for (const solve::Rejection& r : solution.rejections) {
    RejectionCount& count = counts[{r.round, static_cast<int>(r.reason)}];
    count.round = r.round;
    count.reason = r.reason;
    ++(r.corner < 0 ? count.views : count.corners);
  }
  std::vector<RejectionCount> out;
  for (const auto& entry : counts) out.push_back(entry.second);
  return out;
}

// The transfer, as cross_camera_transfer describes it, of cameras already
// checked.
core::Result<Transfer> transfer_of(
    const solve::RigLenses& rig, const compute::Board& board,
    const std::vector<Observation>& views,
    const std::vector<camera::RodriguesTransform>& world_to_camera) {
  const std::size_t k = rig.ids.size();
  if (world_to_camera.size() != k) {
    return core::Status::invalid_argument(
        std::to_string(world_to_camera.size()) + " poses for " +
        std::to_string(k) + " cameras");
  }
  std::vector<camera::Mat4d> to_camera;
  std::vector<camera::Mat4d> to_world;
  for (std::size_t c = 0; c < k; ++c) {
    if (!finite(world_to_camera[c])) {
      return core::Status::invalid_argument("the pose of camera " + rig.ids[c] +
                                            " is not finite");
    }
    to_camera.push_back(camera::matrix_from_rodrigues(world_to_camera[c]));
    to_world.push_back(camera::rigid_inverse(to_camera.back()));
  }
  std::map<std::string, std::size_t> by_id;
  for (std::size_t c = 0; c < k; ++c) by_id.emplace(rig.ids[c], c);
  std::map<std::int64_t, std::vector<std::size_t>> by_capture;
  std::set<std::pair<std::int64_t, std::size_t>> seen;
  for (std::size_t i = 0; i < views.size(); ++i) {
    const auto c = by_id.find(views[i].camera_id);
    if (c == by_id.end()) {
      return core::Status::invalid_argument(
          "observation " + std::to_string(i) + " names camera " +
          views[i].camera_id + ", which the rig does not have");
    }
    if (!seen.emplace(views[i].frame_id, c->second).second) {
      return core::Status::invalid_argument("camera " + views[i].camera_id +
                                            " has two views of " +
                                            capture_name(views[i].frame_id));
    }
    by_capture[views[i].frame_id].push_back(i);
  }

  Transfer out;
  out.pairs.assign(k, std::vector<PairTransfer>(k));
  std::vector<std::vector<double>> sums(k, std::vector<double>(k, 0.0));
  const std::vector<cv::Point3d>& corners = board.object_points();
  for (const auto& [capture, list] : by_capture) {
    // Each camera's own pose of the board (T_cb), and the view it is from.
    std::vector<camera::Mat4d> posed(k);
    std::vector<const Observation*> view_of(k, nullptr);
    for (const std::size_t i : list) {
      const std::size_t c = by_id.at(views[i].camera_id);
      VKC_ASSIGN(const compute::ViewUse use,
                 compute::usable_for_pose(board, views[i], rig.lenses[c].size));
      if (use != compute::ViewUse::Usable) continue;
      core::Result<solve::ViewPose> pose =
          solve::view_pose(rig.lenses[c], board, views[i]);
      if (!pose.ok()) {
        if (pose.status().domain() == Code::Numerical) continue;
        return pose.status();
      }
      posed[c] = pose->best.board_to_camera;
      view_of[c] = &views[i];
    }
    for (std::size_t from = 0; from < k; ++from) {
      if (view_of[from] == nullptr) continue;
      for (std::size_t to = 0; to < k; ++to) {
        if (to == from || view_of[to] == nullptr) continue;
        // T_cb in `to`: T_cw(to) T_cw(from)^-1 T_cb(from).
        const camera::Mat4d carried =
            to_camera[to] * to_world[from] * posed[from];
        double sum = 0.0;
        for (const DetectedPoint& point : view_of[to]->points) {
          const cv::Point3d& x = corners[static_cast<std::size_t>(point.id)];
          const std::optional<camera::Vec2d> pixel = compute::project(
              rig.lenses[to],
              camera::Vec3d(carried * glm::dvec4(x.x, x.y, x.z, 1.0)));
          if (!pixel) {
            sum = kInfinity;
            break;
          }
          sum += glm::length(*pixel - point.pixel);
        }
        const double error =
            sum / static_cast<double>(view_of[to]->points.size());
        out.errors.push_back({capture, from, to, error});
        ++out.pairs[from][to].count;
        sums[from][to] += error;
      }
    }
  }
  std::vector<double> errors;
  errors.reserve(out.errors.size());
  for (const TransferError& e : out.errors) errors.push_back(e.error_px);
  out.median_px = compute::median(errors);
  out.p90_px = compute::quantile(errors, 0.9);
  out.worst_px = compute::quantile(std::move(errors), 1.0);
  for (std::size_t from = 0; from < k; ++from) {
    for (std::size_t to = 0; to < k; ++to) {
      PairTransfer& pair = out.pairs[from][to];
      if (pair.count > 0) {
        pair.mean_px = sums[from][to] / static_cast<double>(pair.count);
      }
    }
  }
  return out;
}

// The rig @p solution reached, as a start for a warm re-solve: its cameras
// and boards (the start's, for a capture whose views were all dropped), and
// @p start's tree and view records.
solve::InitialRig warm_start(const solve::InitialRig& start,
                             const solve::RigSolution& solution) {
  solve::InitialRig warm;
  warm.reference = solution.reference;
  for (std::size_t c = 0; c < solution.world_to_camera.size(); ++c) {
    warm.world_to_camera.push_back(
        c == solution.reference
            ? camera::Mat4d(1.0)
            : camera::matrix_from_rodrigues(solution.world_to_camera[c]));
  }
  std::map<std::int64_t, const camera::RodriguesTransform*> solved;
  for (const solve::SolvedBoard& b : solution.boards) {
    solved.emplace(b.capture, &b.board_to_world);
  }
  for (const solve::BoardPose& b : start.boards) {
    const auto it = solved.find(b.capture);
    warm.boards.push_back(
        {b.capture, it != solved.end()
                        ? camera::matrix_from_rodrigues(*it->second)
                        : b.board_to_world});
  }
  warm.links = start.links;
  warm.views = start.views;
  return warm;
}

core::Result<LeaveOneOut> leave_one_out_of(
    const Fit& fit, const std::vector<solve::RigCamera>& cameras,
    const compute::Board& board, const std::vector<Observation>& views,
    const solve::InitialRig& start, const solve::RigSolution& solution,
    const solve::BundleAdjustOptions& options, const Progress& progress) {
  const std::size_t k = cameras.size();
  std::set<std::int64_t> captures;
  for (std::size_t i = 0; i < views.size(); ++i) {
    if (fit.first[i] != kNone) captures.insert(views[i].frame_id);
  }
  const solve::InitialRig warm = warm_start(start, solution);
  std::vector<camera::Mat4d> solved;
  for (const camera::RodriguesTransform& t : solution.world_to_camera) {
    solved.push_back(camera::matrix_from_rodrigues(t));
  }
  LeaveOneOut out;
  out.cameras.assign(k, CameraStability{});
  std::size_t done = 0;
  for (const std::int64_t capture : captures) {
    std::vector<Observation> less_views;
    solve::InitialRig less = warm;
    less.views.clear();
    less.boards.clear();
    for (std::size_t i = 0; i < views.size(); ++i) {
      if (views[i].frame_id == capture) continue;
      less_views.push_back(views[i]);
      less.views.push_back(warm.views[i]);
    }
    for (const solve::BoardPose& b : warm.boards) {
      if (b.capture != capture) less.boards.push_back(b);
    }
    LeftOut& left = out.captures.emplace_back();
    left.capture = capture;
    core::Result<solve::RigSolution> again =
        solve::bundle_adjust(cameras, board, less_views, less, options);
    if (!again.ok()) {
      const Code code = again.status().domain();
      if (code != Code::InvalidArgument && code != Code::Numerical) {
        return again.status().with_context(capture_name(capture) + " left out");
      }
      left.refused = again.status().message();
    } else {
      std::tie(left.rms_px, left.legacy_rmse_px) = kept_rms(*again);
      for (std::size_t c = 0; c < k; ++c) {
        const camera::Mat4d moved =
            camera::matrix_from_rodrigues(again->world_to_camera[c]);
        const PoseChange change{
            1000.0 * glm::length(centre_of(moved) - centre_of(solved[c])),
            compute::rotation_angle_deg(camera::Mat3d(solved[c]),
                                        camera::Mat3d(moved))};
        left.cameras.push_back(change);
        CameraStability& most = out.cameras[c];
        if (change.centre_mm > most.centre_mm) {
          most.centre_mm = change.centre_mm;
          most.centre_capture = capture;
        }
        if (change.rotation_deg > most.rotation_deg) {
          most.rotation_deg = change.rotation_deg;
          most.rotation_capture = capture;
        }
      }
    }
    ++done;
    if (progress) progress(done, captures.size());
  }
  return out;
}

// Refuses a shift not finite or 0, naming it `name`.
core::Status check_shift(double shift_px, std::string_view name) {
  if (!std::isfinite(shift_px) || shift_px == 0.0) {
    return core::Status::invalid_argument(std::string(name) +
                                          " must be finite and not 0");
  }
  return {};
}

// Refuses an A/B margin out of range: a fraction not in (0, 1), named
// `fraction_name`, or a cap not positive and finite, named `px_name`.
core::Status check_margin(double fraction, double px,
                          std::string_view fraction_name,
                          std::string_view px_name) {
  if (!(fraction > 0.0 && fraction < 1.0)) {
    return core::Status::invalid_argument(std::string(fraction_name) +
                                          " must be in (0, 1)");
  }
  if (!(px > 0.0) || !std::isfinite(px)) {
    return core::Status::invalid_argument(std::string(px_name) +
                                          " must be positive and finite");
  }
  return {};
}

core::Result<PrincipalPointAB> principal_point_of(
    const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
    const std::vector<Observation>& views, const solve::InitialRig& start,
    const solve::RigSolution& solution,
    const solve::BundleAdjustOptions& options, double shift_px,
    double margin_fraction, double margin_px, const Transfer& as_given) {
  std::vector<solve::RigCamera> shifted = cameras;
  for (solve::RigCamera& cam : shifted) {
    cam.model.intrinsics.cx += shift_px;
    cam.model.intrinsics.cy += shift_px;
  }
  core::Result<solve::RigLenses> lenses =
      solve::check_rig(shifted, solution.reference);
  if (!lenses.ok()) return lenses.status().with_context("shifted");
  PrincipalPointAB out;
  out.shift_px = shift_px;
  core::Result<solve::RigSolution> again = solve::bundle_adjust(
      shifted, board, views, warm_start(start, solution), options);
  if (!again.ok()) {
    const Code code = again.status().domain();
    if (code != Code::InvalidArgument && code != Code::Numerical) {
      return again.status().with_context("shifted");
    }
    out.refused = again.status().message();
    return out;
  }
  // The corners both keep: the same views were adjusted, so the residuals
  // are in the same order.
  std::vector<camera::Vec2d> given_residuals;
  std::vector<camera::Vec2d> shifted_residuals;
  if (again->residuals.size() != solution.residuals.size()) {
    return core::Status::invalid_argument(
        "the shifted solve adjusted other corners");
  }
  for (std::size_t j = 0; j < solution.residuals.size(); ++j) {
    const solve::CornerResidual& a = solution.residuals[j];
    const solve::CornerResidual& b = again->residuals[j];
    if (a.camera != b.camera || a.capture != b.capture ||
        a.corner != b.corner) {
      return core::Status::invalid_argument(
          "the shifted solve adjusted other corners");
    }
    if (a.dropped || b.dropped) continue;
    given_residuals.push_back(a.residual);
    shifted_residuals.push_back(b.residual);
  }
  core::Result<Transfer> moved =
      transfer_of(*lenses, board, views, again->world_to_camera);
  if (!moved.ok()) return moved.status().with_context("shifted");

  out.corners = given_residuals.size();
  const auto arm = [](const std::vector<camera::Vec2d>& residuals,
                      const Transfer& transfer) {
    PrincipalPointArm a;
    a.rms_px = stats_of(residuals).rms_px;
    a.legacy_rmse_px = legacy_rmse(residuals);
    a.transfer_median_px = transfer.median_px;
    a.transfer_p90_px = transfer.p90_px;
    return a;
  };
  out.as_given = arm(given_residuals, as_given);
  out.shifted = arm(shifted_residuals, *moved);
  out.verdict = principal_point_verdict(out.as_given, out.shifted,
                                        margin_fraction, margin_px);
  return out;
}

RigGeometry geometry_of(const solve::RigSolution& solution) {
  RigGeometry out;
  for (const camera::RodriguesTransform& t : solution.world_to_camera) {
    out.centres_m.push_back(centre_of(t));
  }
  const std::size_t k = out.centres_m.size();
  out.baselines_m.assign(k, std::vector<double>(k, 0.0));
  for (std::size_t a = 0; a < k; ++a) {
    for (std::size_t b = 0; b < k; ++b) {
      out.baselines_m[a][b] = glm::length(out.centres_m[a] - out.centres_m[b]);
    }
  }
  return out;
}

core::Status check_options(const MetricsOptions& options) {
  VKC_TRY(solve::check_initialise_options(options.initialise,
                                          "options.initialise."));
  VKC_TRY(solve::check_bundle_adjust_options(options.bundle_adjust,
                                             "options.bundle_adjust."));
  VKC_TRY(
      check_shift(options.principal_shift_px, "options.principal_shift_px"));
  VKC_TRY(check_margin(
      options.principal_margin_fraction, options.principal_margin_px,
      "options.principal_margin_fraction", "options.principal_margin_px"));
  if (!(options.histogram_bin_px > 0.0) ||
      !std::isfinite(options.histogram_bin_px)) {
    return core::Status::invalid_argument(
        "options.histogram_bin_px must be positive and finite");
  }
  if (options.histogram_bins < 1) {
    return core::Status::invalid_argument(
        "options.histogram_bins must be at least 1");
  }
  return {};
}

}  // namespace

PrincipalPointVerdict principal_point_verdict(const PrincipalPointArm& as_given,
                                              const PrincipalPointArm& shifted,
                                              double margin_fraction,
                                              double margin_px) noexcept {
  const double a = as_given.rms_px;
  const double s = shifted.rms_px;
  if (!std::isfinite(a) || !std::isfinite(s)) {
    return PrincipalPointVerdict::Inconclusive;
  }
  const double margin = std::min(margin_fraction * std::max(a, s), margin_px);
  // A tie is never decisive, even at both RMS 0, where the margin is 0.
  if (a - s > 0.0 && a - s >= margin) return PrincipalPointVerdict::Shifted;
  if (s - a > 0.0 && s - a >= margin) return PrincipalPointVerdict::AsGiven;
  return PrincipalPointVerdict::Inconclusive;
}

double principal_point_gap(const PrincipalPointArm& as_given,
                           const PrincipalPointArm& shifted) noexcept {
  const double a = as_given.rms_px;
  const double s = shifted.rms_px;
  const double larger = std::max(a, s);
  if (!std::isfinite(a) || !std::isfinite(s) || !(larger > 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (a - s) / larger;
}

const char* to_string(PrincipalPointVerdict verdict) noexcept {
  switch (verdict) {
    case PrincipalPointVerdict::AsGiven:
      return "as_given";
    case PrincipalPointVerdict::Shifted:
      return "shifted";
    case PrincipalPointVerdict::Inconclusive:
      return "inconclusive";
  }
  return "unknown";
}

core::Result<Transfer> cross_camera_transfer(
    const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
    const std::vector<Observation>& views,
    const std::vector<camera::RodriguesTransform>& world_to_camera) {
  constexpr const char* kWhat = "cross_camera_transfer";
  return compute::guarded(kWhat, [&]() -> core::Result<Transfer> {
    core::Result<solve::RigLenses> rig = solve::check_rig(cameras, 0);
    if (!rig.ok()) return rig.status().with_context(kWhat);
    core::Result<Transfer> transfer =
        transfer_of(*rig, board, views, world_to_camera);
    if (!transfer.ok()) return transfer.status().with_context(kWhat);
    return transfer;
  });
}

core::Result<LeaveOneOut> leave_one_out(
    const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
    const std::vector<Observation>& views, const solve::InitialRig& start,
    const solve::RigSolution& solution,
    const solve::BundleAdjustOptions& options, const Progress& progress) {
  constexpr const char* kWhat = "leave_one_out";
  return compute::guarded(kWhat, [&]() -> core::Result<LeaveOneOut> {
    VKC_TRY(
        solve::check_bundle_adjust_options(options, "leave_one_out: options."));
    core::Result<Fit> fit = read_fit(cameras, views, start, solution);
    if (!fit.ok()) return fit.status().with_context(kWhat);
    core::Result<LeaveOneOut> out = leave_one_out_of(
        *fit, cameras, board, views, start, solution, options, progress);
    if (!out.ok()) return out.status().with_context(kWhat);
    return out;
  });
}

core::Result<PrincipalPointAB> principal_point_ab(
    const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
    const std::vector<Observation>& views, const solve::InitialRig& start,
    const solve::RigSolution& solution,
    const solve::BundleAdjustOptions& options, double shift_px,
    double margin_fraction, double margin_px) {
  constexpr const char* kWhat = "principal_point_ab";
  return compute::guarded(kWhat, [&]() -> core::Result<PrincipalPointAB> {
    VKC_TRY(solve::check_bundle_adjust_options(options,
                                               "principal_point_ab: options."));
    core::Status shift = check_shift(shift_px, "shift_px");
    if (!shift.ok()) return shift.with_context(kWhat);
    core::Status margin = check_margin(margin_fraction, margin_px,
                                       "margin_fraction", "margin_px");
    if (!margin.ok()) return margin.with_context(kWhat);
    core::Result<Fit> fit = read_fit(cameras, views, start, solution);
    if (!fit.ok()) return fit.status().with_context(kWhat);
    core::Result<Transfer> as_given =
        transfer_of(fit->rig, board, views, solution.world_to_camera);
    if (!as_given.ok()) return as_given.status().with_context(kWhat);
    core::Result<PrincipalPointAB> out =
        principal_point_of(cameras, board, views, start, solution, options,
                           shift_px, margin_fraction, margin_px, *as_given);
    if (!out.ok()) return out.status().with_context(kWhat);
    return out;
  });
}

core::Result<CalibrationMetrics> calibration_metrics(
    const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
    const std::vector<Observation>& views, const solve::InitialRig& start,
    const solve::RigSolution& solution, const MetricsOptions& options,
    const Progress& progress) {
  constexpr const char* kWhat = "calibration_metrics";
  return compute::guarded(kWhat, [&]() -> core::Result<CalibrationMetrics> {
    const auto in = [&](const char* part, core::Status status) {
      return std::move(status).with_context(std::string(kWhat) + ": " + part);
    };
    core::Status checked = check_options(options);
    if (!checked.ok()) return std::move(checked).with_context(kWhat);
    core::Result<Fit> fit = read_fit(cameras, views, start, solution);
    if (!fit.ok()) return fit.status().with_context(kWhat);

    CalibrationMetrics metrics;
    metrics.options = options;
    core::Result<Reprojection> reprojection =
        reprojection_of(*fit, board, views, start, solution, options);
    if (!reprojection.ok()) return in("reprojection", reprojection.status());
    metrics.reprojection = *std::move(reprojection);
    for (const solve::ViewRecord& record : start.views) {
      if (!solve::adjusts(record)) metrics.views_left_out.push_back(record);
    }
    metrics.rejections = rejection_counts(solution);
    core::Result<Transfer> transfer =
        transfer_of(fit->rig, board, views, solution.world_to_camera);
    if (!transfer.ok()) return in("cross_camera_transfer", transfer.status());
    metrics.transfer = *std::move(transfer);
    core::Result<LeaveOneOut> loo =
        leave_one_out_of(*fit, cameras, board, views, start, solution,
                         options.bundle_adjust, progress);
    if (!loo.ok()) return in("leave_one_out", loo.status());
    metrics.leave_one_out = *std::move(loo);
    core::Result<PrincipalPointAB> ab = principal_point_of(
        cameras, board, views, start, solution, options.bundle_adjust,
        options.principal_shift_px, options.principal_margin_fraction,
        options.principal_margin_px, metrics.transfer);
    if (!ab.ok()) return in("principal_point_ab", ab.status());
    metrics.principal_point = *std::move(ab);
    metrics.geometry = geometry_of(solution);
    return metrics;
  });
}

}  // namespace volumetric_kit::calib::quality
