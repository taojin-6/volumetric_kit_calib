// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/solve/world_tag.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/geometry.hpp>

#include "common.hpp"
#include "reprojection.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/cv_pose.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"

namespace volumetric_kit::calib::solve {
namespace {

namespace camera = sensor::camera;
using compute::finite;
using detail::Derivatives;
using detail::kLevMarqTolerance;
using detail::number;
using detail::Pose;
using detail::pose_of;
using detail::Projection;

constexpr const char* kWhat = "origin_tag_frame: ";
constexpr double kInfinity = std::numeric_limits<double>::infinity();
// The corners, 4 of 3 coordinates, as cv::LevMarq's parameters.
constexpr int kParameters = 12;

// One view of the tag.
struct View {
  std::size_t camera = 0;  // in the rig's order
  std::int64_t capture = 0;
  std::vector<cv::Point2d> pixels;  // the corners, DetectedMarker's order
  std::vector<cv::Point2d> rays;    // their normalized coordinates
  bool used = true;
  int round = 0;
  double bound = 0.0;
};

// The location's data and state.
struct Problem {
  detail::RigCameras rig;
  std::vector<Pose> poses;             // T_cw, in the rig's order
  std::vector<cv::Matx34d> extrinsic;  // [R | t] of each T_cw
  std::vector<View> views;             // by capture, then camera id
};

std::string view_name(const Problem& p, const View& v) {
  return "camera " + p.rig.ids[v.camera] + ", capture " +
         std::to_string(v.capture);
}

std::size_t cameras_used(const Problem& p) {
  std::set<std::size_t> used;
  for (const View& v : p.views) {
    if (v.used) used.insert(v.camera);
  }
  return used.size();
}

core::Status check_options(const TagFrameOptions& o) {
  constexpr const char* kPrefix = "origin_tag_frame: options.";
  const struct {
    const char* name;
    double value;
  } positives[] = {{"outlier_px", o.outlier_px},
                   {"outlier_mads", o.outlier_mads},
                   {"edge_warn_fraction", o.edge_warn_fraction}};
  for (const auto& option : positives) {
    if (!(option.value > 0.0) || !std::isfinite(option.value)) {
      return detail::refuse_option(kPrefix, option.name, "positive and finite");
    }
  }
  if (o.outlier_rounds < 0) {
    return detail::refuse_option(kPrefix, "outlier_rounds", "at least 0");
  }
  if (o.min_cameras < 2) {
    return detail::refuse_option(kPrefix, "min_cameras", "at least 2");
  }
  return {};
}

// The rig, the solution's poses and the views, checked, the views in order.
core::Result<Problem> read_problem(const std::vector<RigCamera>& cameras,
                                   const RigSolution& solution,
                                   const std::vector<TagView>& views,
                                   const OriginTag& tag) {
  Problem p;
  VKC_ASSIGN(p.rig, detail::read_rig_cameras(cameras, kWhat));
  const std::size_t k = cameras.size();
  if (solution.world_to_camera.size() != k) {
    return core::Status::invalid_argument(
        std::string(kWhat) + "the solution places " +
        std::to_string(solution.world_to_camera.size()) + " cameras of " +
        std::to_string(k));
  }
  if (solution.reference >= k) {
    return core::Status::invalid_argument(
        std::string(kWhat) + "the solution's reference " +
        std::to_string(solution.reference) + " is not one of the rig's " +
        std::to_string(k) + " cameras");
  }
  for (std::size_t c = 0; c < k; ++c) {
    const camera::RodriguesTransform& t = solution.world_to_camera[c];
    if (!finite(t)) {
      return core::Status::invalid_argument(std::string(kWhat) +
                                            "the solution's pose of camera " +
                                            p.rig.ids[c] + " is not finite");
    }
    p.poses.push_back(pose_of(t));
    const camera::Mat4d m = camera::matrix_from_rodrigues(t);
    cv::Matx34d e;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 4; ++col) e(row, col) = m[col][row];
    }
    p.extrinsic.push_back(e);
  }

  std::set<std::pair<std::int64_t, std::size_t>> seen;
  for (std::size_t i = 0; i < views.size(); ++i) {
    const TagView& view = views[i];
    const std::string what = std::string(kWhat) + "view " + std::to_string(i);
    const auto found = p.rig.by_id.find(view.camera);
    if (found == p.rig.by_id.end()) {
      return core::Status::invalid_argument(
          what + " names camera \"" + view.camera + "\", not one of the rig's");
    }
    const std::size_t c = found->second;
    const std::string of = what + " (camera " + view.camera + ", capture " +
                           std::to_string(view.capture) + ")";
    if (view.marker.id != tag.id) {
      return core::Status::invalid_argument(
          of + " is of marker " + std::to_string(view.marker.id) +
          ", not the tag's id " + std::to_string(tag.id));
    }
    const camera::ImageSize& lens = cameras[c].model.size;
    if (view.image_size.width != lens.width ||
        view.image_size.height != lens.height) {
      return core::Status::invalid_argument(
          of + " is of a " + std::to_string(view.image_size.width) + "x" +
          std::to_string(view.image_size.height) + " image, not its lens's " +
          std::to_string(lens.width) + "x" + std::to_string(lens.height));
    }
    for (const camera::Vec2d& corner : view.marker.corners) {
      if (!std::isfinite(corner.x) || !std::isfinite(corner.y)) {
        return core::Status::invalid_argument(of + " has a corner not finite");
      }
    }
    if (!seen.emplace(view.capture, c).second) {
      return core::Status::invalid_argument(
          std::string(kWhat) + "camera " + view.camera +
          " has two views of capture " + std::to_string(view.capture));
    }
    View& v = p.views.emplace_back();
    v.camera = c;
    v.capture = view.capture;
    for (const camera::Vec2d& corner : view.marker.corners) {
      v.pixels.emplace_back(corner.x, corner.y);
    }
  }
  std::sort(p.views.begin(), p.views.end(), [&](const View& a, const View& b) {
    return a.capture != b.capture ? a.capture < b.capture
                                  : p.rig.rank[a.camera] < p.rig.rank[b.camera];
  });
  for (View& v : p.views) {
    VKC_ASSIGN(v.rays,
               compute::undistort_points(p.rig.lenses[v.camera], v.pixels));
  }
  return p;
}

enum class Outcome { Ok, Behind, NotFinite };

// The energy, sum |e|^2, over the used views' corners at `corners`; and,
// given `jtj` and `jte` (zeroed, 12 square and long), J^T J and J^T e. Each
// corner's residual depends on that corner alone: dp/dX_w is the board
// block's translation part, the board at the identity.
Outcome evaluate(const Problem& p, const std::vector<cv::Point3d>& corners,
                 double& energy, cv::Mat_<double>* jtj, cv::Mat_<double>* jte) {
  energy = 0.0;
  Projection at;
  for (const View& v : p.views) {
    if (!v.used) continue;
    detail::project(
        p.rig.lenses[v.camera], p.poses[v.camera], Pose::all(0.0), corners,
        &v.pixels, jtj != nullptr ? Derivatives::Extrinsics : Derivatives::None,
        at);
    for (std::size_t k = 0; k < corners.size(); ++k) {
      if (!(at.depths[k] > 0.0)) {
        return std::isnan(at.depths[k]) ? Outcome::NotFinite : Outcome::Behind;
      }
      const cv::Vec2d& e = at.residuals[k];
      const double s2 = e.dot(e);
      if (!std::isfinite(s2)) return Outcome::NotFinite;
      energy += s2;
      if (jtj == nullptr) continue;
      const cv::Matx23d j = at.by_board[k].get_minor<2, 3>(0, 3);
      const cv::Matx33d h = j.t() * j;
      const cv::Vec3d g = j.t() * e;
      const int o = 3 * static_cast<int>(k);
      for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) (*jtj)(o + a, o + b) += h(a, b);
        (*jte)(o + a) += g[a];
      }
    }
  }
  return std::isfinite(energy) ? Outcome::Ok : Outcome::NotFinite;
}

// Each corner from the used views' rays by a linear N-view DLT: the null
// vector of the rows x P_3 - P_1 and y P_3 - P_2, P each view's [R | t].
// Empty, or why it failed (a Numerical miss).
std::string triangulate(const Problem& p, std::vector<cv::Point3d>& corners) {
  int rows = 0;
  for (const View& v : p.views) rows += v.used ? 2 : 0;
  corners.assign(4, cv::Point3d());
  for (std::size_t k = 0; k < corners.size(); ++k) {
    cv::Mat_<double> a(rows, 4);
    int row = 0;
    for (const View& v : p.views) {
      if (!v.used) continue;
      const cv::Matx34d& e = p.extrinsic[v.camera];
      const cv::Point2d& ray = v.rays[k];
      for (int col = 0; col < 4; ++col) {
        a(row, col) = ray.x * e(2, col) - e(0, col);
        a(row + 1, col) = ray.y * e(2, col) - e(1, col);
      }
      row += 2;
    }
    cv::Mat_<double> x;
    cv::SVD::solveZ(a, x);
    // A unit null vector whose w is this small puts the corner farther than
    // 1e9 m: the rays meet at no point.
    if (!(std::abs(x(3)) > 1e-9)) {
      return "the rays to corner " + std::to_string(k) + " meet at no point";
    }
    corners[k] = cv::Point3d(x(0) / x(3), x(1) / x(3), x(2) / x(3));
  }
  for (const View& v : p.views) {
    if (!v.used) continue;
    const cv::Matx34d& e = p.extrinsic[v.camera];
    for (std::size_t k = 0; k < corners.size(); ++k) {
      const cv::Point3d& x = corners[k];
      const double z = e(2, 0) * x.x + e(2, 1) * x.y + e(2, 2) * x.z + e(2, 3);
      if (!(z > 0.0)) {
        return "corner " + std::to_string(k) + " lies behind " +
               view_name(p, v);
      }
    }
  }
  return {};
}

// Places the corners from the used views: the DLT, then least squares on
// cv::LevMarq, a few steps from it. Empty, or why it failed (a Numerical
// miss).
std::string solve_once(const Problem& p, std::vector<cv::Point3d>& corners) {
  std::string why = triangulate(p, corners);
  if (!why.empty()) return why;
  double energy = 0.0;
  if (evaluate(p, corners, energy, nullptr, nullptr) != Outcome::Ok) {
    return "the energy at the linear corners is not finite";
  }

  Outcome failure = Outcome::Ok;
  std::vector<cv::Point3d> probe = corners;
  const auto unpack = [&](const cv::Mat& x) {
    const double* v = x.ptr<double>();
    for (int i = 0; i < kParameters; ++i) {
      if (!std::isfinite(v[i])) return false;
    }
    for (std::size_t k = 0; k < probe.size(); ++k) {
      probe[k] = cv::Point3d(v[3 * k], v[3 * k + 1], v[3 * k + 2]);
    }
    return true;
  };
  const cv::LevMarq::NormalCallback callback =
      [&](cv::InputOutputArray param, cv::OutputArray jte_out,
          cv::OutputArray jtj_out, double& errnorm) -> bool {
    if (!unpack(param.getMat())) {
      failure = Outcome::NotFinite;
      return false;
    }
    const bool linearise = jtj_out.needed();
    cv::Mat_<double> jtj;
    cv::Mat_<double> jte;
    if (linearise) {
      jte_out.create(kParameters, 1, CV_64F);
      jtj_out.create(kParameters, kParameters, CV_64F);
      jte = jte_out.getMat();
      jtj = jtj_out.getMat();
      jte.setTo(0.0);
      jtj.setTo(0.0);
    }
    const Outcome outcome =
        evaluate(p, probe, errnorm, linearise ? &jtj : nullptr,
                 linearise ? &jte : nullptr);
    if (outcome == Outcome::Ok) return true;
    if (linearise) {
      failure = outcome;
      return false;
    }
    // A step that takes a corner behind a camera, or out of range, is
    // refused: cv::LevMarq tries a shorter one.
    errnorm = kInfinity;
    return true;
  };
  cv::Mat_<double> x(kParameters, 1);
  for (std::size_t k = 0; k < corners.size(); ++k) {
    x(3 * static_cast<int>(k)) = corners[k].x;
    x(3 * static_cast<int>(k) + 1) = corners[k].y;
    x(3 * static_cast<int>(k) + 2) = corners[k].z;
  }
  cv::LevMarq solver(x, callback,
                     cv::LevMarq::Settings()
                         .setRelEnergyDeltaTolerance(kLevMarqTolerance)
                         .setStepNormTolerance(kLevMarqTolerance)
                         .setCheckMinGradient(false),
                     cv::noArray(), cv::MatrixType::DENSE,
                     cv::VariableType::LINEAR, false, cv::DECOMP_CHOLESKY);
  const cv::LevMarq::Report report = solver.optimize();
  if (failure != Outcome::Ok || !unpack(x)) {
    return "a corner went behind a camera or out of range after " +
           std::to_string(report.iters) + " iterations";
  }
  if (!report.found || !std::isfinite(report.energy)) {
    return "cv::LevMarq did not converge in " + std::to_string(report.iters) +
           " iterations (energy " + number(energy) + " to " +
           number(report.energy) + ")";
  }
  corners = probe;
  return {};
}

// Each view's residuals at `corners`: its RMS and worst (pixels), and the
// components, which `components` gains for a view used.
struct Fit {
  double rms = 0.0;
  double worst = 0.0;
};

Fit fit_of(const Problem& p, const View& v,
           const std::vector<cv::Point3d>& corners,
           std::vector<double>* components) {
  Projection at;
  detail::project(p.rig.lenses[v.camera], p.poses[v.camera], Pose::all(0.0),
                  corners, &v.pixels, Derivatives::None, at);
  Fit fit;
  double sum = 0.0;
  for (const cv::Vec2d& e : at.residuals) {
    const double s2 = e.dot(e);
    sum += s2;
    fit.worst = std::max(fit.worst, std::sqrt(s2));
    if (components != nullptr) {
      components->push_back(e[0]);
      components->push_back(e[1]);
    }
  }
  fit.rms = std::sqrt(sum / static_cast<double>(at.residuals.size()));
  return fit;
}

camera::Vec3d vec(const cv::Point3d& p) noexcept { return {p.x, p.y, p.z}; }

WorldOrigin miss(TagMiss why, std::string camera_id, std::string reason) {
  WorldOrigin origin;
  origin.miss = why;
  origin.camera = std::move(camera_id);
  origin.reason = std::move(reason);
  return origin;
}

core::Result<WorldOrigin> locate(const std::vector<RigCamera>& cameras,
                                 const RigSolution& solution,
                                 const std::vector<TagView>& views,
                                 const OriginTag& tag,
                                 const TagFrameOptions& options) {
  VKC_TRY(check_options(options));
  VKC_TRY(check_origin_tag(tag).with_context("origin_tag_frame"));
  VKC_ASSIGN(Problem p, read_problem(cameras, solution, views, tag));
  const std::size_t k = cameras.size();
  const auto min_cameras = static_cast<std::size_t>(options.min_cameras);

  // Who saw it.
  std::vector<int> seen(k, 0);
  for (const View& v : p.views) ++seen[v.camera];
  const std::size_t seeing = cameras_used(p);
  if (seeing == 0) {
    return miss(TagMiss::NotSeen, {}, "no camera saw the tag");
  }
  if (seeing == 1) {
    const std::string& id = p.rig.ids[p.views.front().camera];
    return miss(TagMiss::OneCamera, id, "camera " + id + " alone saw the tag");
  }
  if (seeing < min_cameras) {
    return miss(TagMiss::FewCameras, {},
                std::to_string(seeing) + " cameras saw the tag, fewer than " +
                    "min_cameras (" + std::to_string(min_cameras) + ")");
  }

  // The corners, with outliers dropped after each solve but the last.
  std::vector<cv::Point3d> corners;
  int solves = 0;
  for (int round = 0;; ++round) {
    const std::string why = solve_once(p, corners);
    ++solves;
    if (!why.empty()) {
      return miss(TagMiss::Numerical, {},
                  "solve " + std::to_string(solves) + ": " + why);
    }
    if (round == options.outlier_rounds) break;
    std::vector<double> components;
    std::vector<double> worst(p.views.size(), 0.0);
    for (std::size_t i = 0; i < p.views.size(); ++i) {
      if (p.views[i].used) {
        worst[i] = fit_of(p, p.views[i], corners, &components).worst;
      }
    }
    const double bound = detail::outlier_bound(
        std::move(components), options.outlier_px, options.outlier_mads);
    bool outliers = false;
    std::set<std::size_t> left;  // the cameras with a view within the bound
    for (std::size_t i = 0; i < p.views.size(); ++i) {
      if (!p.views[i].used) continue;
      if (worst[i] > bound) {
        outliers = true;
      } else {
        left.insert(p.views[i].camera);
      }
    }
    // Dropping them would leave too few cameras to tell the wrong views by,
    // as with two cameras that disagree: none is dropped, and the fit
    // decides.
    if (!outliers || left.size() < min_cameras) break;
    for (std::size_t i = 0; i < p.views.size(); ++i) {
      View& v = p.views[i];
      if (!v.used || !(worst[i] > bound)) continue;
      v.used = false;
      v.round = solves;
      v.bound = bound;
    }
  }

  // The frame: a square of the mean edge fitted to the corners.
  TagWorld world;
  world.solves = solves;
  double edges = 0.0;
  for (std::size_t c = 0; c < 4; ++c) {
    world.edges_m[c] = cv::norm(corners[(c + 1) % 4] - corners[c]);
    edges += world.edges_m[c];
  }
  world.diagonals_m = {cv::norm(corners[2] - corners[0]),
                       cv::norm(corners[3] - corners[1])};
  world.edge_m = edges / 4.0;
  if (!(world.edge_m > 0.0) || !std::isfinite(world.edge_m)) {
    return miss(TagMiss::Numerical, {}, "the corners make no square");
  }
  const double h = 0.5 * world.edge_m;
  const cv::Vec3d model[4] = {
      {-h, h, 0.0}, {h, h, 0.0}, {h, -h, 0.0}, {-h, -h, 0.0}};
  cv::Vec3d centre(0.0, 0.0, 0.0);
  for (const cv::Point3d& x : corners) centre += cv::Vec3d(x.x, x.y, x.z);
  centre *= 0.25;
  cv::Matx33d cross = cv::Matx33d::zeros();  // sum model (X - centre)^T
  for (std::size_t c = 0; c < 4; ++c) {
    const cv::Vec3d x =
        cv::Vec3d(corners[c].x, corners[c].y, corners[c].z) - centre;
    cross += model[c] * x.t();
  }
  cv::Matx31d sigma;
  cv::Matx33d u;
  cv::Matx33d vt;
  cv::SVD::compute(cross, sigma, u, vt);
  // Corners on a line fix no turn about it.
  if (!(sigma(1) > 1e-9 * sigma(0))) {
    return miss(TagMiss::Numerical, {}, "the corners make no square");
  }
  // The proper rotation nearest: a reflection is turned into one by the
  // least singular vector's sign.
  const double turn = cv::determinant(vt.t() * u.t()) < 0.0 ? -1.0 : 1.0;
  const cv::Matx33d rotation =
      vt.t() * cv::Matx33d(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, turn) *
      u.t();  // tag -> reference
  cv::Matx44d tag_to_reference = cv::Matx44d::eye();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      tag_to_reference(row, col) = rotation(row, col);
    }
    tag_to_reference(row, 3) = centre[row];
  }
  world.reference_to_world =
      camera::rigid_inverse(compute::matrix_from_cv(tag_to_reference));
  const camera::Mat4d& w = world.reference_to_world;
  for (std::size_t c = 0; c < 4; ++c) {
    world.corners_m[c] = camera::Vec3d(w * glm::dvec4(vec(corners[c]), 1.0));
    world.out_of_plane_m =
        std::max(world.out_of_plane_m, std::abs(world.corners_m[c].z));
  }
  const double ratio = world.edge_m / tag.edge_m - 1.0;
  world.edge_error_percent = 100.0 * ratio;
  world.edge_warning = std::abs(ratio) > options.edge_warn_fraction;

  // Each camera's height, and the guard on those that saw the tag.
  for (std::size_t c = 0; c < k; ++c) {
    TagCamera& cam = world.cameras.emplace_back();
    cam.id = p.rig.ids[c];
    cam.views = seen[c];
    const camera::Mat4d color_to_world =
        w * camera::rigid_inverse(
                camera::matrix_from_rodrigues(solution.world_to_camera[c]));
    cam.height_m = color_to_world[3].z;
  }
  for (const View& v : p.views) {
    if (v.used) ++world.cameras[v.camera].used;
  }
  for (const std::size_t c : p.rig.by_rank) {
    const TagCamera& cam = world.cameras[c];
    if (cam.views == 0 || cam.height_m > 0.0) continue;
    return miss(TagMiss::CameraBelowTag, cam.id,
                "camera " + cam.id + " is " + number(0.0 - cam.height_m) +
                    " m below the tag's plane: the tag lies face down, or "
                    "its corners are mirrored");
  }

  // The fit.
  double sum = 0.0;
  int used = 0;
  for (const View& v : p.views) {
    const Fit fit = fit_of(p, v, corners, nullptr);
    world.views.push_back({p.rig.ids[v.camera], v.capture, fit.rms, fit.worst,
                           v.used, v.round, v.bound});
    if (!v.used) continue;
    sum += fit.rms * fit.rms;
    ++used;
    world.worst_px = std::max(world.worst_px, fit.worst);
  }
  world.rms_px = std::sqrt(sum / used);
  if (world.rms_px > kMaxTagWorldRmsPx) {
    return miss(TagMiss::PoorFit, {},
                "the tag's corners fit its views at " + number(world.rms_px) +
                    " px RMS, above the " + number(kMaxTagWorldRmsPx) +
                    " px a world takes: the cameras disagree on where it is");
  }
  WorldOrigin origin;
  origin.tag = std::move(world);
  return origin;
}

}  // namespace

const char* to_string(TagMiss miss) noexcept {
  switch (miss) {
    case TagMiss::NotSeen:
      return "not_seen";
    case TagMiss::OneCamera:
      return "one_camera";
    case TagMiss::FewCameras:
      return "few_cameras";
    case TagMiss::CameraBelowTag:
      return "camera_below_tag";
    case TagMiss::Numerical:
      return "numerical";
    case TagMiss::PoorFit:
      return "poor_fit";
  }
  return "unknown";
}

core::Result<WorldOrigin> origin_tag_frame(
    const std::vector<RigCamera>& cameras, const RigSolution& solution,
    const std::vector<TagView>& views, const OriginTag& tag,
    const TagFrameOptions& options) {
  return compute::guarded(
      "origin_tag_frame", [&]() -> core::Result<WorldOrigin> {
        VKC_ASSIGN(WorldOrigin origin,
                   locate(cameras, solution, views, tag, options));
        origin.sought = tag;
        return origin;
      });
}

core::Result<AnchoredRig> anchor_rig(const RigSolution& solution,
                                     const WorldOrigin& origin) {
  return compute::guarded("anchor_rig", [&]() -> core::Result<AnchoredRig> {
    constexpr const char* kAnchor = "anchor_rig: ";
    if (solution.world_to_camera.empty()) {
      return core::Status::invalid_argument(std::string(kAnchor) +
                                            "the solution places no camera");
    }
    const camera::Mat4d w = origin.reference_to_world();
    VKC_TRY(camera::check_rigid(w).with_context(
        "anchor_rig: the origin's reference_to_world"));
    // W moves a frame into the world; the identity leaves it as it is.
    const bool moved = w != camera::Mat4d(1.0);
    const auto in_world = [&](const camera::Mat4d& m) {
      return moved ? w * m : m;
    };

    AnchoredRig out;
    out.calibration = solution.calibration;
    if (origin.tag) {
      const OriginTag& tag = origin.sought;
      VKC_TRY(check_origin_tag(tag).with_context(
          "anchor_rig: the tag world's tag sought"));
      const std::string family = apriltag_family(tag.dictionary);
      if (family.empty()) {
        return core::Status::invalid_argument(
            std::string(kAnchor) + "the tag world's tag sought, of " +
            to_string(tag.dictionary) +
            ", has no AprilTag family for the calibration's world");
      }
      out.calibration.world.sensor.clear();
      out.calibration.world.apriltag =
          camera::AprilTagWorld{family, tag.id, tag.edge_m};
    }
    for (camera::SensorCalibration& sensor : out.calibration.sensors) {
      if (!sensor.color_to_world) {
        return core::Status::invalid_argument(std::string(kAnchor) +
                                              "the solution's sensor " +
                                              sensor.id + " is not posed");
      }
      sensor.color_to_world = in_world(*sensor.color_to_world);
    }
    for (std::size_t c = 0; c < solution.world_to_camera.size(); ++c) {
      const camera::RodriguesTransform& t = solution.world_to_camera[c];
      if (!finite(t)) {
        return core::Status::invalid_argument(
            std::string(kAnchor) + "the solution's pose " + std::to_string(c) +
            " is not finite");
      }
      out.color_to_world.push_back(
          in_world(camera::rigid_inverse(camera::matrix_from_rodrigues(t))));
    }
    for (const SolvedBoard& board : solution.boards) {
      if (!finite(board.board_to_world)) {
        return core::Status::invalid_argument(
            std::string(kAnchor) + "the solution's board of capture " +
            std::to_string(board.capture) + " is not finite");
      }
      out.boards_to_world.push_back(
          in_world(camera::matrix_from_rodrigues(board.board_to_world)));
    }
    return out;
  });
}

}  // namespace volumetric_kit::calib::solve
