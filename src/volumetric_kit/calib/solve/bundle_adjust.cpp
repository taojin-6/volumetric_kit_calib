// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/solve/bundle_adjust.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <glm/gtc/constants.hpp>
#include <glm/trigonometric.hpp>
#include <opencv2/geometry.hpp>

#include "common.hpp"
#include "reprojection.hpp"
#include "volumetric_kit/calib/compute/cv_pose.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/usable_for_pose.hpp"

namespace volumetric_kit::calib::solve {
namespace {

namespace camera = recon::camera;
using detail::Derivatives;
using detail::huber;
using detail::huber_weight;
using detail::Pose;
using detail::Projection;

constexpr const char* kWhat = "bundle_adjust: ";
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// cv::LevMarq stops when the energy's relative change, or the step's squared
// norm, falls below this.
constexpr double kTolerance = 1e-12;

Pose pose_of(const camera::RodriguesTransform& t) noexcept {
  return {t.rvec.x, t.rvec.y, t.rvec.z, t.tvec.x, t.tvec.y, t.tvec.z};
}

Pose pose_of(const compute::CvPose& t) noexcept {
  return {t.rvec[0], t.rvec[1], t.rvec[2], t.tvec[0], t.tvec[1], t.tvec[2]};
}

camera::RodriguesTransform transform_of(const Pose& p) noexcept {
  return {{p[0], p[1], p[2]}, {p[3], p[4], p[5]}};
}

// The same rotation with its angle in [0, pi]: a vector longer than pi turns
// the other way about its axis, by what it lacks of a whole turn.
Pose within_pi(Pose p) noexcept {
  const double pi = glm::pi<double>();
  const double angle = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
  if (!(angle > pi)) return p;
  const double scale =
      (angle - 2.0 * pi * std::round(angle / (2.0 * pi))) / angle;
  for (int i = 0; i < 3; ++i) p[i] *= scale;
  return p;
}

std::string number(double v) {
  char text[32];
  std::snprintf(text, sizeof text, "%.6g", v);
  return text;
}

bool positive(double v) noexcept { return v > 0.0 && std::isfinite(v); }

// One view the adjustment takes.
struct View {
  std::size_t observation = 0;
  std::size_t camera = 0;  // in the rig's order
  std::size_t board = 0;   // in Problem::captures
  std::vector<int> ids;
  std::vector<cv::Point3d> object;
  std::vector<cv::Point2d> pixels;
  std::vector<double> weights;
  std::vector<bool> kept;  // each corner
  bool whole = true;       // false once the view is dropped
};

bool counts(const View& view) {
  return view.whole &&
         std::find(view.kept.begin(), view.kept.end(), true) != view.kept.end();
}

// The adjustment's data and state.
struct Problem {
  std::vector<compute::CvCamera> lenses;
  std::vector<std::string> ids;
  std::vector<std::size_t> by_rank;  // the cameras in id order
  std::size_t reference = 0;
  std::vector<std::int64_t> captures;  // each board's, ascending
  std::vector<View> views;  // in their boards' order, then their cameras' ids'
  std::vector<Pose> cameras;  // T_cw, in the rig's order
  std::vector<Pose> boards;   // T_wb, in captures' order
};

// Where each transform's parameters sit in LevMarq's vector: the cameras but
// the reference, in id order, then the boards with a view kept.
struct Layout {
  std::vector<int> camera;  // -1: fixed
  std::vector<int> board;   // -1: not adjusted
  int cameras = 0;          // the cameras' parameters, which come first
  int size = 0;
};

Layout layout_of(const Problem& p) {
  Layout layout;
  layout.camera.assign(p.cameras.size(), -1);
  layout.board.assign(p.boards.size(), -1);
  for (const std::size_t c : p.by_rank) {
    if (c == p.reference) continue;
    layout.camera[c] = layout.size;
    layout.size += 6;
  }
  layout.cameras = layout.size;
  std::vector<bool> seen(p.boards.size(), false);
  for (const View& view : p.views) {
    if (counts(view)) seen[view.board] = true;
  }
  for (std::size_t b = 0; b < p.boards.size(); ++b) {
    if (!seen[b]) continue;
    layout.board[b] = layout.size;
    layout.size += 6;
  }
  return layout;
}

// One view's kept corners, as a solve takes them.
struct Active {
  const View* view = nullptr;
  std::vector<cv::Point3d> object;
  std::vector<cv::Point2d> pixels;
  std::vector<double> weights;
};

std::vector<Active> active_of(const Problem& p) {
  std::vector<Active> active;
  for (const View& view : p.views) {
    if (!counts(view)) continue;
    Active& a = active.emplace_back();
    a.view = &view;
    for (std::size_t i = 0; i < view.kept.size(); ++i) {
      if (!view.kept[i]) continue;
      a.object.push_back(view.object[i]);
      a.pixels.push_back(view.pixels[i]);
      a.weights.push_back(view.weights[i]);
    }
  }
  return active;
}

void add_block(cv::Mat_<double>& m, int row, int col, const cv::Matx66d& b) {
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) m(row + i, col + j) += b(i, j);
  }
}

void add_vector(cv::Mat_<double>& v, int row, const cv::Vec6d& b) {
  for (int i = 0; i < 6; ++i) v(row + i) += b[i];
}

enum class Outcome { Ok, TooNear, NotFinite };

// The energy, sum weight * huber(|e|), over the kept corners at `cameras` and
// `boards`; and, given `jtj` and `jte` (zeroed, `layout.size` square and
// long), J^T W J and J^T W e, W each corner's weight times its Huber weight.
Outcome evaluate(const Problem& p, const std::vector<Active>& active,
                 const std::vector<Pose>& cameras,
                 const std::vector<Pose>& boards, const Layout& layout,
                 const BundleAdjustOptions& options, double& energy,
                 cv::Mat_<double>* jtj, cv::Mat_<double>* jte) {
  energy = 0.0;
  Projection at;
  for (const Active& a : active) {
    const View& view = *a.view;
    detail::project(
        p.lenses[view.camera], cameras[view.camera], boards[view.board],
        a.object, &a.pixels,
        jtj != nullptr ? Derivatives::Extrinsics : Derivatives::None, at);
    cv::Matx66d hcc = cv::Matx66d::zeros();
    cv::Matx66d hcb = cv::Matx66d::zeros();
    cv::Matx66d hbb = cv::Matx66d::zeros();
    cv::Vec6d gc = cv::Vec6d::all(0.0);
    cv::Vec6d gb = cv::Vec6d::all(0.0);
    for (std::size_t i = 0; i < a.object.size(); ++i) {
      if (!(at.depths[i] >= options.min_depth_m)) {
        return std::isnan(at.depths[i]) ? Outcome::NotFinite : Outcome::TooNear;
      }
      const cv::Vec2d& e = at.residuals[i];
      const double s = std::sqrt(e.dot(e));
      if (!std::isfinite(s)) return Outcome::NotFinite;
      energy += a.weights[i] * huber(s, options.huber_px);
      if (jtj == nullptr) continue;
      const double w = a.weights[i] * huber_weight(s, options.huber_px);
      const CornerJacobian& jc = at.by_camera[i];
      const CornerJacobian& jb = at.by_board[i];
      hcc += w * (jc.t() * jc);
      hcb += w * (jc.t() * jb);
      hbb += w * (jb.t() * jb);
      gc += w * (jc.t() * e);
      gb += w * (jb.t() * e);
    }
    if (jtj == nullptr) continue;
    const int oc = layout.camera[view.camera];
    const int ob = layout.board[view.board];
    if (oc >= 0) {
      add_block(*jtj, oc, oc, hcc);
      add_block(*jtj, oc, ob, hcb);
      add_block(*jtj, ob, oc, hcb.t());
      add_vector(*jte, oc, gc);
    }
    add_block(*jtj, ob, ob, hbb);
    add_vector(*jte, ob, gb);
  }
  return std::isfinite(energy) ? Outcome::Ok : Outcome::NotFinite;
}

// The state's parameters as LevMarq's vector, and back; false for a value
// that is not finite.
cv::Mat_<double> pack(const Problem& p, const Layout& layout) {
  cv::Mat_<double> x(layout.size, 1);
  for (std::size_t c = 0; c < p.cameras.size(); ++c) {
    if (layout.camera[c] < 0) continue;
    for (int k = 0; k < 6; ++k) x(layout.camera[c] + k) = p.cameras[c][k];
  }
  for (std::size_t b = 0; b < p.boards.size(); ++b) {
    if (layout.board[b] < 0) continue;
    for (int k = 0; k < 6; ++k) x(layout.board[b] + k) = p.boards[b][k];
  }
  return x;
}

bool unpack(const cv::Mat& x, const Layout& layout, std::vector<Pose>& cameras,
            std::vector<Pose>& boards) {
  const double* v = x.ptr<double>();
  for (int i = 0; i < layout.size; ++i) {
    if (!std::isfinite(v[i])) return false;
  }
  for (std::size_t c = 0; c < cameras.size(); ++c) {
    if (layout.camera[c] < 0) continue;
    for (int k = 0; k < 6; ++k) cameras[c][k] = v[layout.camera[c] + k];
  }
  for (std::size_t b = 0; b < boards.size(); ++b) {
    if (layout.board[b] < 0) continue;
    for (int k = 0; k < 6; ++k) boards[b][k] = v[layout.board[b] + k];
  }
  return true;
}

// Solve `solve` (1 the first) of the kept corners, from the current state.
core::Status solve_once(Problem& p, const BundleAdjustOptions& options,
                        int solve, RigSolution& out) {
  const Layout layout = layout_of(p);
  const std::vector<Active> active = active_of(p);
  const std::string where = "bundle_adjust: solve " + std::to_string(solve);

  double energy = 0.0;
  if (evaluate(p, active, p.cameras, p.boards, layout, options, energy, nullptr,
               nullptr) != Outcome::Ok) {
    return core::Status::numerical(where + ": the energy is not finite");
  }
  if (solve == 1) out.initial_energy = energy;

  Outcome failure = Outcome::Ok;
  std::vector<Pose> cameras = p.cameras;
  std::vector<Pose> boards = p.boards;
  const cv::LevMarq::NormalCallback callback =
      [&](cv::InputOutputArray param, cv::OutputArray jte_out,
          cv::OutputArray jtj_out, double& errnorm) -> bool {
    if (!unpack(param.getMat(), layout, cameras, boards)) {
      failure = Outcome::NotFinite;
      return false;
    }
    const bool linearise = jtj_out.needed();
    cv::Mat_<double> jtj;
    cv::Mat_<double> jte;
    if (linearise) {
      jte_out.create(layout.size, 1, CV_64F);
      jtj_out.create(layout.size, layout.size, CV_64F);
      jte = jte_out.getMat();
      jtj = jtj_out.getMat();
      jte.setTo(0.0);
      jtj.setTo(0.0);
    }
    const Outcome outcome =
        evaluate(p, active, cameras, boards, layout, options, errnorm,
                 linearise ? &jtj : nullptr, linearise ? &jte : nullptr);
    if (outcome == Outcome::Ok) return true;
    if (linearise) {
      failure = outcome;
      return false;
    }
    // A step that brings a corner too near its camera, or out of range, is
    // refused: LevMarq tries a shorter one.
    errnorm = kInfinity;
    return true;
  };
  cv::Mat_<double> x = pack(p, layout);
  cv::LevMarq solver(x, callback,
                     cv::LevMarq::Settings()
                         .setMaxIterations(options.max_iterations)
                         .setRelEnergyDeltaTolerance(kTolerance)
                         .setStepNormTolerance(kTolerance)
                         .setCheckMinGradient(false),
                     cv::noArray(), cv::MatrixType::DENSE,
                     cv::VariableType::LINEAR, false, cv::DECOMP_CHOLESKY);
  const cv::LevMarq::Report report = solver.optimize();
  out.iterations += report.iters;
  out.solves = solve;
  const std::string after =
      " after " + std::to_string(report.iters) + " iterations";
  if (failure == Outcome::TooNear) {
    return core::Status::numerical(
        where + ": a corner came nearer its camera than min_depth_m" + after);
  }
  if (failure == Outcome::NotFinite ||
      !unpack(x, layout, p.cameras, p.boards)) {
    return core::Status::numerical(where + ": a value is not finite" + after);
  }
  if (!report.found || !std::isfinite(report.energy)) {
    return core::Status::numerical(
        where + ": cv::LevMarq did not converge in " +
        std::to_string(report.iters) + " iterations (energy " + number(energy) +
        " to " + number(report.energy) + ")");
  }
  for (Pose& pose : p.cameras) pose = within_pi(pose);
  for (Pose& pose : p.boards) pose = within_pi(pose);
  out.final_energy = report.energy;
  return {};
}

// Drops whole each view `touched` whose kept corners no longer fix a pose.
core::Status drop_unusable(Problem& p, const compute::Board& board,
                           const std::vector<bool>& touched, int round,
                           RigSolution& out) {
  for (std::size_t v = 0; v < p.views.size(); ++v) {
    View& view = p.views[v];
    if (!touched[v] || !view.whole) continue;
    Observation kept;
    for (std::size_t i = 0; i < view.ids.size(); ++i) {
      if (!view.kept[i]) continue;
      kept.points.push_back(
          {view.ids[i], {view.pixels[i].x, view.pixels[i].y}, view.weights[i]});
    }
    VKC_ASSIGN(
        const compute::ViewUse use,
        compute::usable_for_pose(board, kept, p.lenses[view.camera].size));
    if (use == compute::ViewUse::Usable) continue;
    view.whole = false;
    out.rejections.push_back({view.camera, p.captures[view.board], -1, round,
                              Dropped::Unusable, 0.0, 0.0});
  }
  return {};
}

// Drops the kept corners nearer their cameras than min_depth_m.
core::Status drop_too_near(Problem& p, const compute::Board& board,
                           const BundleAdjustOptions& options, int round,
                           RigSolution& out) {
  std::vector<bool> touched(p.views.size(), false);
  Projection at;
  for (std::size_t v = 0; v < p.views.size(); ++v) {
    View& view = p.views[v];
    if (!counts(view)) continue;
    detail::project(p.lenses[view.camera], p.cameras[view.camera],
                    p.boards[view.board], view.object, nullptr,
                    Derivatives::None, at);
    for (std::size_t i = 0; i < view.object.size(); ++i) {
      if (!view.kept[i] || at.depths[i] >= options.min_depth_m) continue;
      view.kept[i] = false;
      touched[v] = true;
      out.rejections.push_back({view.camera, p.captures[view.board],
                                view.ids[i], round, Dropped::TooNear,
                                at.depths[i], options.min_depth_m});
    }
  }
  return drop_unusable(p, board, touched, round, out);
}

// Drops the outlier corners and views at the current state; true if any.
// The corners' bound comes first: a view's RMS is over its corners within it,
// or over all its corners when none is, so one bad corner costs its view
// nothing more, and a view off as a whole goes as one.
core::Result<bool> drop_outliers(Problem& p, const compute::Board& board,
                                 const BundleAdjustOptions& options, int round,
                                 RigSolution& out) {
  struct Fit {
    std::size_t view = 0;
    std::vector<double> norms;  // each kept corner's residual; NaN if not kept
    double rms = 0.0;
  };
  std::vector<Fit> fits;
  std::vector<double> components;
  Projection at;
  for (std::size_t v = 0; v < p.views.size(); ++v) {
    const View& view = p.views[v];
    if (!counts(view)) continue;
    detail::project(p.lenses[view.camera], p.cameras[view.camera],
                    p.boards[view.board], view.object, &view.pixels,
                    Derivatives::None, at);
    Fit& fit = fits.emplace_back();
    fit.view = v;
    fit.norms.assign(view.object.size(), kNaN);
    for (std::size_t i = 0; i < view.object.size(); ++i) {
      if (!view.kept[i]) continue;
      const cv::Vec2d& e = at.residuals[i];
      components.push_back(e[0]);
      components.push_back(e[1]);
      fit.norms[i] = std::sqrt(e.dot(e));
    }
  }
  if (fits.empty()) return false;
  const double corner_bound = detail::outlier_bound(
      std::move(components), options.outlier_px, options.outlier_mads);
  std::vector<double> rms;
  for (Fit& fit : fits) {
    double within = 0.0;
    double all = 0.0;
    int n_within = 0;
    int n_all = 0;
    for (const double e : fit.norms) {
      if (std::isnan(e)) continue;
      all += e * e;
      ++n_all;
      if (e > corner_bound) continue;
      within += e * e;
      ++n_within;
    }
    fit.rms =
        n_within > 0 ? std::sqrt(within / n_within) : std::sqrt(all / n_all);
    rms.push_back(fit.rms);
  }
  const double view_bound =
      std::max(options.outlier_view_ratio * detail::median(rms),
               options.outlier_view_px);

  std::vector<bool> touched(p.views.size(), false);
  bool any = false;
  for (const Fit& fit : fits) {
    View& view = p.views[fit.view];
    if (fit.rms > view_bound) {
      view.whole = false;
      any = true;
      out.rejections.push_back({view.camera, p.captures[view.board], -1, round,
                                Dropped::OutlierView, fit.rms, view_bound});
      continue;
    }
    for (std::size_t i = 0; i < view.object.size(); ++i) {
      if (!(fit.norms[i] > corner_bound)) continue;
      view.kept[i] = false;
      touched[fit.view] = true;
      any = true;
      out.rejections.push_back({view.camera, p.captures[view.board],
                                view.ids[i], round, Dropped::Outlier,
                                fit.norms[i], corner_bound});
    }
  }
  VKC_TRY(drop_unusable(p, board, touched, round, out));
  return any;
}

// The problem @p start poses: its cameras by id, its views adjusted (those
// it uses or left as ambiguous) in canonical order, and its state.
core::Result<Problem> read_problem(const std::vector<RigCamera>& cameras,
                                   const compute::Board& board,
                                   const std::vector<Observation>& views,
                                   const InitialRig& start,
                                   const BundleAdjustOptions& options) {
  VKC_ASSIGN(detail::RigCameras rig, detail::read_rig_cameras(cameras, kWhat));
  const std::size_t k = cameras.size();
  Problem p;
  p.ids = std::move(rig.ids);
  p.lenses = std::move(rig.lenses);
  p.by_rank = std::move(rig.by_rank);
  const std::vector<std::size_t>& rank = rig.rank;
  VKC_TRY(detail::check_one_view_each(views, kWhat));

  // The start's reference and cameras.
  if (start.reference >= k) {
    return core::Status::invalid_argument(
        "bundle_adjust: start.reference " + std::to_string(start.reference) +
        " is not one of the rig's " + std::to_string(k) + " cameras");
  }
  p.reference = start.reference;
  if (start.world_to_camera.size() != k) {
    return core::Status::invalid_argument(
        "bundle_adjust: start places " +
        std::to_string(start.world_to_camera.size()) + " cameras of " +
        std::to_string(k));
  }
  if (start.world_to_camera[p.reference] != camera::Mat4d(1.0)) {
    return core::Status::invalid_argument("bundle_adjust: start's reference, " +
                                          p.ids[p.reference] +
                                          ", is not exactly at the identity");
  }
  p.cameras.assign(k, Pose::all(0.0));
  for (std::size_t c = 0; c < k; ++c) {
    if (c == p.reference) continue;
    core::Result<compute::CvPose> pose =
        compute::cv_pose(start.world_to_camera[c]);
    if (!pose.ok()) {
      return pose.status().with_context("bundle_adjust: start's camera " +
                                        p.ids[c]);
    }
    p.cameras[c] = pose_of(*pose);
  }
  std::map<std::int64_t, const camera::Mat4d*> start_boards;
  for (const BoardPose& b : start.boards) {
    if (!start_boards.emplace(b.capture, &b.board_to_world).second) {
      return core::Status::invalid_argument(
          "bundle_adjust: start has two boards for capture " +
          std::to_string(b.capture));
    }
  }
  for (const PlacedLink& link : start.links) {
    if (link.camera >= k || link.parent >= k) {
      return core::Status::invalid_argument(
          "bundle_adjust: start has a link to no camera");
    }
  }
  if (start.views.size() != views.size()) {
    return core::Status::invalid_argument(
        "bundle_adjust: start has " + std::to_string(start.views.size()) +
        " view records for " + std::to_string(views.size()) + " observations");
  }

  // The views adjusted, and the captures they see.
  std::map<std::int64_t, std::size_t> board_of;
  const std::vector<cv::Point3d>& corners = board.object_points();
  for (std::size_t i = 0; i < views.size(); ++i) {
    const Observation& view = views[i];
    const ViewRecord& record = start.views[i];
    const std::string what = "observation " + std::to_string(i);
    if (record.camera >= k || p.ids[record.camera] != view.camera_id ||
        record.capture != view.frame_id) {
      return core::Status::invalid_argument(
          "bundle_adjust: start's record of " + what +
          " is not of its camera and capture");
    }
    if (!adjusts(record)) continue;
    if (start_boards.count(view.frame_id) == 0) {
      return core::Status::invalid_argument(
          "bundle_adjust: start places no board for capture " +
          std::to_string(view.frame_id) + ", which " + what + " sees");
    }
    VKC_ASSIGN(
        const compute::ViewUse use,
        compute::usable_for_pose(board, view, p.lenses[record.camera].size));
    if (use != compute::ViewUse::Usable) {
      return core::Status::invalid_argument(
          "bundle_adjust: " + what +
          " fixes no pose: " + compute::describe(use));
    }
    View& v = p.views.emplace_back();
    v.observation = i;
    v.camera = record.camera;
    for (const DetectedPoint& point : view.points) {
      if (!(point.weight >= 0.0) || !std::isfinite(point.weight)) {
        return core::Status::invalid_argument(
            "bundle_adjust: " + what + " has a weight negative or not finite");
      }
      v.ids.push_back(point.id);
      v.object.push_back(corners[static_cast<std::size_t>(point.id)]);
      v.pixels.emplace_back(point.pixel.x, point.pixel.y);
      v.weights.push_back(point.weight);
    }
    v.kept.assign(v.ids.size(), true);
    board_of.emplace(view.frame_id, 0);
  }
  if (p.views.empty()) {
    return core::Status::invalid_argument("bundle_adjust: start uses no view");
  }
  if (board_of.size() > options.max_captures) {
    return core::Status::invalid_argument(
        "bundle_adjust: " + std::to_string(board_of.size()) +
        " captures, more than max_captures (" +
        std::to_string(options.max_captures) + ")");
  }
  for (auto& [capture, b] : board_of) {
    b = p.captures.size();
    p.captures.push_back(capture);
    core::Result<compute::CvPose> pose =
        compute::cv_pose(*start_boards.at(capture));
    if (!pose.ok()) {
      return pose.status().with_context(
          "bundle_adjust: start's board of capture " + std::to_string(capture));
    }
    p.boards.push_back(pose_of(*pose));
  }
  for (View& v : p.views) v.board = board_of.at(views[v.observation].frame_id);
  std::sort(p.views.begin(), p.views.end(), [&](const View& a, const View& b) {
    return a.board != b.board ? a.board < b.board
                              : rank[a.camera] < rank[b.camera];
  });
  return p;
}

// Each link of the start's tree keeps min_shared captures in which both
// cameras' kept, positive-weight corners still fix a pose.
core::Status check_links(const Problem& p, const compute::Board& board,
                         const InitialRig& start,
                         const BundleAdjustOptions& options) {
  const std::size_t k = p.cameras.size();
  const std::size_t n = p.captures.size();
  std::vector<std::vector<bool>> adjusted(k, std::vector<bool>(n, false));
  std::vector<std::vector<bool>> kept(k, std::vector<bool>(n, false));
  for (const View& view : p.views) {
    Observation weighted;
    Observation remaining;
    for (std::size_t i = 0; i < view.ids.size(); ++i) {
      if (view.weights[i] == 0.0) continue;
      const DetectedPoint point{
          view.ids[i], {view.pixels[i].x, view.pixels[i].y}, view.weights[i]};
      weighted.points.push_back(point);
      if (view.kept[i]) remaining.points.push_back(point);
    }
    VKC_ASSIGN(
        const compute::ViewUse before,
        compute::usable_for_pose(board, weighted, p.lenses[view.camera].size));
    adjusted[view.camera][view.board] = before == compute::ViewUse::Usable;
    if (!counts(view)) continue;
    VKC_ASSIGN(
        const compute::ViewUse use,
        compute::usable_for_pose(board, remaining, p.lenses[view.camera].size));
    kept[view.camera][view.board] = use == compute::ViewUse::Usable;
  }
  for (const PlacedLink& link : start.links) {
    int shared = 0;
    int before = 0;  // as the adjustment began
    for (std::size_t b = 0; b < n; ++b) {
      shared += kept[link.camera][b] && kept[link.parent][b] ? 1 : 0;
      before += adjusted[link.camera][b] && adjusted[link.parent][b] ? 1 : 0;
    }
    if (shared >= options.min_shared) continue;
    std::string why = "bundle_adjust: " + p.ids[link.camera] + " and " +
                      p.ids[link.parent] + " share " + std::to_string(shared) +
                      (shared == 1 ? " capture" : " captures") +
                      " in which both keep a view";
    if (shared < before) {
      why += " (" + std::to_string(before) + " before views were dropped)";
    }
    return core::Status::invalid_argument(why + ", fewer than min_shared (" +
                                          std::to_string(options.min_shared) +
                                          ")");
  }
  return {};
}

// The left-perturbation derivative of R(r) (dtheta = J dr, where
// R + dR = [dtheta]x R), and that of the camera's centre, C = -R^T t, by
// (r, t).
struct PoseDerivatives {
  cv::Matx33d rotation;
  cv::Matx<double, 3, 6> centre;
};

PoseDerivatives pose_derivatives(const Pose& world_to_camera) {
  cv::Matx33d r;
  cv::Matx<double, 3, 9> d;
  cv::Rodrigues(
      cv::Vec3d(world_to_camera[0], world_to_camera[1], world_to_camera[2]), r,
      d);
  const cv::Vec3d t(world_to_camera[3], world_to_camera[4], world_to_camera[5]);
  PoseDerivatives out;
  for (int k = 0; k < 3; ++k) {
    cv::Matx33d dr;
    for (int i = 0; i < 9; ++i) dr.val[i] = d(k, i);
    const cv::Matx33d skew = dr * r.t();
    out.rotation(0, k) = 0.5 * (skew(2, 1) - skew(1, 2));
    out.rotation(1, k) = 0.5 * (skew(0, 2) - skew(2, 0));
    out.rotation(2, k) = 0.5 * (skew(1, 0) - skew(0, 1));
    const cv::Vec3d dc = -(dr.t() * t);
    for (int a = 0; a < 3; ++a) out.centre(a, k) = dc[a];
  }
  for (int a = 0; a < 3; ++a) {
    for (int k = 0; k < 3; ++k) out.centre(a, k + 3) = -r(k, a);
  }
  return out;
}

PoseShift shift_of(const PoseDerivatives& d, const cv::Vec6d& step) {
  const cv::Vec3d turn = d.rotation * cv::Vec3d(step[0], step[1], step[2]);
  const cv::Vec3d moved = d.centre * step;
  return {glm::degrees(std::sqrt(turn.dot(turn))),
          1000.0 * camera::Vec3d(moved[0], moved[1], moved[2])};
}

// The guard on each camera's conditioning, the covariance and the
// sensitivity, from the normal matrix at the solution with the boards
// eliminated: S = A - sum_b B_b D_b^-1 B_b^T.
core::Status uncertainty(const Problem& p, const BundleAdjustOptions& options,
                         RigSolution& out) {
  const std::size_t k = p.cameras.size();
  const Layout layout = layout_of(p);
  const int nc = layout.cameras;

  // Each kept corner's derivatives and weight, W = weight * Huber weight.
  struct Line {
    const View* view = nullptr;
    std::vector<double> w;
    Projection at;
  };
  std::vector<Line> lines;
  double weighted = 0.0;
  int residuals = 0;  // those weighted: a corner of weight 0 adds nothing
  for (const Active& a : active_of(p)) {
    Line& line = lines.emplace_back();
    line.view = a.view;
    detail::project(p.lenses[a.view->camera], p.cameras[a.view->camera],
                    p.boards[a.view->board], a.object, &a.pixels,
                    Derivatives::All, line.at);
    for (std::size_t i = 0; i < a.object.size(); ++i) {
      const cv::Vec2d& e = line.at.residuals[i];
      const double s = std::sqrt(e.dot(e));
      const double w = a.weights[i] * huber_weight(s, options.huber_px);
      line.w.push_back(w);
      weighted += w * s * s;
      residuals += a.weights[i] > 0.0 ? 2 : 0;
    }
  }
  if (residuals <= layout.size) {
    return core::Status::numerical(
        "bundle_adjust: " + std::to_string(residuals) +
        " weighted residuals for " + std::to_string(layout.size) +
        " parameters");
  }
  cv::Mat_<double> h(layout.size, layout.size, 0.0);
  for (const Line& line : lines) {
    const int oc = layout.camera[line.view->camera];
    const int ob = layout.board[line.view->board];
    for (std::size_t i = 0; i < line.w.size(); ++i) {
      const CornerJacobian& jc = line.at.by_camera[i];
      const CornerJacobian& jb = line.at.by_board[i];
      if (oc >= 0) {
        add_block(h, oc, oc, line.w[i] * (jc.t() * jc));
        add_block(h, oc, ob, line.w[i] * (jc.t() * jb));
        add_block(h, ob, oc, line.w[i] * (jb.t() * jc));
      }
      add_block(h, ob, ob, line.w[i] * (jb.t() * jb));
    }
  }
  cv::Mat_<double> s = h(cv::Rect(0, 0, nc, nc)).clone();
  std::vector<cv::Mat_<double>> d_inverse(p.boards.size());
  for (std::size_t b = 0; b < p.boards.size(); ++b) {
    const int ob = layout.board[b];
    if (ob < 0) continue;
    if (cv::invert(h(cv::Rect(ob, ob, 6, 6)), d_inverse[b],
                   cv::DECOMP_CHOLESKY) == 0.0) {
      return core::Status::numerical("bundle_adjust: the board of capture " +
                                     std::to_string(p.captures[b]) +
                                     " is poorly constrained");
    }
    const cv::Mat_<double> bb = h(cv::Rect(ob, 0, 6, nc));
    if (nc > 0) s -= bb * d_inverse[b] * bb.t();
  }
  cv::Mat_<double> s_inverse;
  if (nc > 0) {
    for (const std::size_t c : p.by_rank) {
      const int oc = layout.camera[c];
      if (oc < 0) continue;
      // TODO: the plan's max_condition, 1e12, stops only a block all but
      // singular: on the synthetic rig the blocks' condition numbers are 3e2
      // to 4e3, a camera tied to the rig by 3 shared captures included, and
      // they mix radians and metres. Revisit it with H3's and H5's data, or
      // scale the translations by the boards' depth to make it unitless.
      const cv::Mat_<double> block = s(cv::Rect(oc, oc, 6, 6));
      cv::Mat values;  // descending
      cv::eigen(0.5 * (block + block.t()), values);
      const double least = values.at<double>(5);
      const double condition =
          least > 0.0 ? values.at<double>(0) / least : kInfinity;
      if (!(condition < options.max_condition)) {
        return core::Status::numerical(
            "bundle_adjust: camera " + p.ids[c] +
            " is poorly constrained: its block's condition number is " +
            number(condition) + ", not below max_condition (" +
            number(options.max_condition) + ")");
      }
    }
    if (cv::invert(s, s_inverse, cv::DECOMP_CHOLESKY) == 0.0) {
      return core::Status::numerical(
          "bundle_adjust: the cameras are poorly constrained together");
    }
  }

  // The covariance: the variance times S^-1's blocks.
  out.variance = weighted / static_cast<double>(residuals - layout.size);
  out.covariance.assign(k, PoseCovariance{});
  std::vector<PoseDerivatives> derivatives;
  for (std::size_t c = 0; c < k; ++c) {
    derivatives.push_back(pose_derivatives(p.cameras[c]));
    const int oc = layout.camera[c];
    if (oc < 0) continue;
    PoseCovariance& cov = out.covariance[c];
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        cov.covariance(i, j) = out.variance * s_inverse(oc + i, oc + j);
      }
    }
    const PoseDerivatives& d = derivatives[c];
    const cv::Matx33d rotation =
        d.rotation * cov.covariance.get_minor<3, 3>(0, 0) * d.rotation.t();
    const cv::Matx33d centre = d.centre * cov.covariance * d.centre.t();
    cov.rotation_deg =
        glm::degrees(std::sqrt(std::max(0.0, cv::trace(rotation))));
    cov.centre_mm = std::sqrt(std::max(0.0, cv::trace(centre))) * 1000.0;
  }

  // The sensitivity: for each camera's J^T W J_int d_int on the right, each
  // camera's step -S^-1 (g_c - sum_b B_b D_b^-1 g_b).
  const auto shifts =
      [&](const cv::Mat_<double>& g) -> core::Result<std::vector<PoseShift>> {
    std::vector<PoseShift> shift(k);
    if (nc == 0) return shift;
    cv::Mat_<double> reduced = g.rowRange(0, nc).clone();
    for (std::size_t b = 0; b < p.boards.size(); ++b) {
      const int ob = layout.board[b];
      if (ob < 0) continue;
      reduced -=
          h(cv::Rect(ob, 0, 6, nc)) * d_inverse[b] * g.rowRange(ob, ob + 6);
    }
    const cv::Mat_<double> step = -(s_inverse * reduced);
    for (std::size_t c = 0; c < k; ++c) {
      const int oc = layout.camera[c];
      if (oc < 0) continue;
      shift[c] = shift_of(derivatives[c],
                          cv::Vec6d(step(oc), step(oc + 1), step(oc + 2),
                                    step(oc + 3), step(oc + 4), step(oc + 5)));
      if (!std::isfinite(shift[c].rotation_deg) ||
          !std::isfinite(shift[c].centre_mm.x) ||
          !std::isfinite(shift[c].centre_mm.y) ||
          !std::isfinite(shift[c].centre_mm.z)) {
        return core::Status::numerical("bundle_adjust: sensitivity of camera " +
                                       p.ids[c] + " is not finite");
      }
    }
    return shift;
  };
  out.sensitivity.assign(k, IntrinsicsSensitivity{});
  for (std::size_t j = 0; j < k; ++j) {
    const cv::Matx33d& m = p.lenses[j].camera_matrix;
    const cv::Vec2d focal(options.focal_fraction * m(0, 0),
                          options.focal_fraction * m(1, 1));
    const cv::Vec2d along_x(options.principal_px, 0.0);
    const cv::Vec2d along_y(0.0, options.principal_px);
    cv::Mat_<double> g_focal(layout.size, 1, 0.0);
    cv::Mat_<double> g_x(layout.size, 1, 0.0);
    cv::Mat_<double> g_y(layout.size, 1, 0.0);
    for (const Line& line : lines) {
      if (line.view->camera != j) continue;
      const int oc = layout.camera[j];
      const int ob = layout.board[line.view->board];
      for (std::size_t i = 0; i < line.w.size(); ++i) {
        const CornerJacobian& jc = line.at.by_camera[i];
        const CornerJacobian& jb = line.at.by_board[i];
        const std::pair<cv::Mat_<double>*, cv::Vec2d> moves[] = {
            {&g_focal, line.at.by_focal[i] * focal},
            {&g_x, line.at.by_centre[i] * along_x},
            {&g_y, line.at.by_centre[i] * along_y}};
        for (const auto& [g, de] : moves) {
          if (oc >= 0) add_vector(*g, oc, line.w[i] * (jc.t() * de));
          add_vector(*g, ob, line.w[i] * (jb.t() * de));
        }
      }
    }
    VKC_ASSIGN(out.sensitivity[j].focal, shifts(g_focal));
    VKC_ASSIGN(out.sensitivity[j].principal_x, shifts(g_x));
    VKC_ASSIGN(out.sensitivity[j].principal_y, shifts(g_y));
  }
  return {};
}

// The poses, recon's calibration, the boards and every adjusted corner's
// residual.
void write_solution(const Problem& p, const std::vector<RigCamera>& cameras,
                    const std::size_t observations, const InitialRig& start,
                    const BundleAdjustOptions& options, RigSolution& out) {
  out.reference = p.reference;
  for (const Pose& pose : p.cameras) {
    out.world_to_camera.push_back(transform_of(pose));
  }
  out.calibration.world.sensor = p.ids[p.reference];
  for (const std::size_t c : p.by_rank) {
    camera::SensorCalibration& sensor = out.calibration.sensors.emplace_back();
    sensor.id = p.ids[c];
    sensor.color_to_world = camera::rigid_inverse(
        camera::matrix_from_rodrigues(out.world_to_camera[c]));
    sensor.color = camera::CameraCalibration{cameras[c].model,
                                             camera::IntrinsicsSource::Factory};
  }
  const Layout layout = layout_of(p);
  for (std::size_t b = 0; b < p.boards.size(); ++b) {
    if (layout.board[b] < 0) continue;
    out.boards.push_back({p.captures[b], transform_of(p.boards[b])});
  }
  std::vector<const View*> by_observation(observations, nullptr);
  for (const View& view : p.views) by_observation[view.observation] = &view;
  Projection at;
  for (const View* view : by_observation) {
    if (view == nullptr) continue;
    detail::project(p.lenses[view->camera], p.cameras[view->camera],
                    p.boards[view->board], view->object, &view->pixels,
                    Derivatives::None, at);
    for (std::size_t i = 0; i < view->object.size(); ++i) {
      CornerResidual& r = out.residuals.emplace_back();
      r.camera = view->camera;
      r.capture = p.captures[view->board];
      r.corner = view->ids[i];
      r.pixel = {view->pixels[i].x, view->pixels[i].y};
      r.residual = at.depths[i] > 0.0
                       ? camera::Vec2d(at.residuals[i][0], at.residuals[i][1])
                       : camera::Vec2d(kNaN);
      r.weight = view->weights[i];
      r.huber_weight = huber_weight(glm::length(r.residual), options.huber_px);
      r.dropped = !view->whole || !view->kept[i];
    }
  }
  out.links = start.links;
}

}  // namespace

core::Status check_bundle_adjust_options(const BundleAdjustOptions& o,
                                         std::string_view prefix) {
  return compute::guarded("check_bundle_adjust_options", [&]() -> core::Status {
    const auto refuse = [&](const char* name, const char* rule) {
      return detail::refuse_option(prefix, name, rule);
    };
    const struct {
      const char* name;
      double value;
    } positives[] = {{"huber_px", o.huber_px},
                     {"min_depth_m", o.min_depth_m},
                     {"outlier_px", o.outlier_px},
                     {"outlier_mads", o.outlier_mads},
                     {"outlier_view_ratio", o.outlier_view_ratio},
                     {"outlier_view_px", o.outlier_view_px},
                     {"focal_fraction", o.focal_fraction},
                     {"principal_px", o.principal_px}};
    for (const auto& option : positives) {
      if (!positive(option.value)) {
        return refuse(option.name, "positive and finite");
      }
    }
    if (o.max_iterations < 1) return refuse("max_iterations", "at least 1");
    if (o.outlier_rounds < 0) return refuse("outlier_rounds", "at least 0");
    if (o.min_shared < 1) return refuse("min_shared", "at least 1");
    if (!(o.max_condition > 1.0)) return refuse("max_condition", "above 1");
    if (o.max_captures < 1) return refuse("max_captures", "at least 1");
    return {};
  });
}

bool adjusts(const ViewRecord& record) noexcept {
  return record.fate == ViewFate::Used || record.fate == ViewFate::Ambiguous;
}

const char* to_string(Dropped reason) noexcept {
  switch (reason) {
    case Dropped::TooNear:
      return "too_near";
    case Dropped::Outlier:
      return "outlier";
    case Dropped::OutlierView:
      return "outlier_view";
    case Dropped::Unusable:
      return "unusable";
  }
  return "unknown";
}

core::Result<ViewResiduals> view_residuals(
    const compute::CvCamera& camera, const compute::Board& board,
    const Observation& view, const camera::RodriguesTransform& world_to_camera,
    const camera::RodriguesTransform& board_to_world) {
  return compute::guarded(
      "view_residuals", [&]() -> core::Result<ViewResiduals> {
        const std::vector<cv::Point3d>& corners = board.object_points();
        std::vector<cv::Point3d> object;
        std::vector<cv::Point2d> detected;
        object.reserve(view.points.size());
        detected.reserve(view.points.size());
        for (const DetectedPoint& point : view.points) {
          if (point.id < 0 ||
              static_cast<std::size_t>(point.id) >= corners.size()) {
            return core::Status::invalid_argument("view_residuals: corner " +
                                                  std::to_string(point.id) +
                                                  " is not one of the board's");
          }
          object.push_back(corners[static_cast<std::size_t>(point.id)]);
          detected.emplace_back(point.pixel.x, point.pixel.y);
        }
        const Pose camera_pose = pose_of(world_to_camera);
        const Pose board_pose = pose_of(board_to_world);
        for (int k = 0; k < 6; ++k) {
          if (!std::isfinite(camera_pose[k]) || !std::isfinite(board_pose[k])) {
            return core::Status::invalid_argument(
                "view_residuals: a transform is not finite");
          }
        }
        Projection at;
        detail::project(camera, camera_pose, board_pose, object, &detected,
                        Derivatives::Extrinsics, at);
        ViewResiduals out;
        out.residuals.reserve(object.size());
        for (const cv::Vec2d& e : at.residuals) {
          out.residuals.emplace_back(e[0], e[1]);
        }
        out.depths = std::move(at.depths);
        out.camera_jacobian = std::move(at.by_camera);
        out.board_jacobian = std::move(at.by_board);
        return out;
      });
}

core::Result<RigSolution> bundle_adjust(const std::vector<RigCamera>& cameras,
                                        const compute::Board& board,
                                        const std::vector<Observation>& views,
                                        const InitialRig& start,
                                        const BundleAdjustOptions& options) {
  return compute::guarded("bundle_adjust", [&]() -> core::Result<RigSolution> {
    const auto began = std::chrono::steady_clock::now();
    VKC_TRY(check_bundle_adjust_options(options));
    VKC_ASSIGN(Problem p, read_problem(cameras, board, views, start, options));
    RigSolution out;
    // The solves, with corners too near dropped before each, and outliers
    // after each but the last.
    for (int round = 0;; ++round) {
      VKC_TRY(drop_too_near(p, board, options, round, out));
      VKC_TRY(solve_once(p, options, round + 1, out));
      if (round == options.outlier_rounds) break;
      VKC_ASSIGN(const bool dropped,
                 drop_outliers(p, board, options, round + 1, out));
      if (!dropped) break;
    }
    VKC_TRY(check_links(p, board, start, options));
    VKC_TRY(uncertainty(p, options, out));
    write_solution(p, cameras, views.size(), start, options, out);
    out.seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count();
    return out;
  });
}

}  // namespace volumetric_kit::calib::solve
