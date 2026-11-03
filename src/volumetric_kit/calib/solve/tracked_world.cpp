// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/solve/tracked_world.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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
using detail::Derivatives;
using detail::kLevMarqTolerance;
using detail::number;
using detail::Pose;
using detail::Projection;

constexpr const char* kWhat = "tracked_world_from_tag: ";
constexpr const char* kRegister = "register_tracked: ";
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kPi = 3.14159265358979323846;
// The yaw, then the translation, as cv::LevMarq's parameters.
constexpr int kParameters = 4;
using Parameters = cv::Vec4d;

// One still, read, in the tracking world turned so its up is +z
// (y = A x_tracking), of which the tag's world is a yaw and a shift.
struct Still {
  std::string name;
  compute::CvCamera lens;
  cv::Matx33d to_camera;  // (A R_c)^T: the turned frame's axes to its camera's
  cv::Vec3d centre;       // its camera's centre in the turned frame, A p_c
  std::vector<cv::Point2d> pixels;  // the corners, DetectedMarker's order
  std::vector<cv::Point2d> rays;    // their normalized coordinates
};

struct Problem {
  cv::Matx33d align;                // A: the tracking world's up onto +z
  std::array<cv::Vec3d, 4> square;  // the corners in the tag's frame
  std::vector<Still> stills;        // by name
};

cv::Matx33d rz(double yaw) noexcept {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0};
}

// The largest of `v`'s components' magnitudes.
double largest(const camera::Vec3d& v) noexcept {
  return std::max({std::abs(v.x), std::abs(v.y), std::abs(v.z)});
}

// The rotation that turns `up` (finite, not zero) onto +z the shortest way:
// a half turn about x for -z. Scaled by its largest component first, so its
// length neither overflows nor underflows.
cv::Matx33d alignment(const camera::Vec3d& up) {
  const double scale = largest(up);
  const cv::Vec3d u =
      cv::normalize(cv::Vec3d(up.x / scale, up.y / scale, up.z / scale));
  const cv::Vec3d axis = u.cross(cv::Vec3d(0.0, 0.0, 1.0));
  const double s = cv::norm(axis);
  if (!(s > 0.0)) {
    return u[2] > 0.0
               ? cv::Matx33d::eye()
               : cv::Matx33d(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
  }
  cv::Matx33d r;
  cv::Rodrigues(axis * (std::atan2(s, u[2]) / s), r);
  return r;
}

// `tracking_to_world` at `x`: [Rz(yaw) A | t].
cv::Matx44d transform_at(const Problem& p, const Parameters& x) {
  const cv::Matx33d r = rz(x[0]) * p.align;
  cv::Matx44d m = cv::Matx44d::eye();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) m(row, col) = r(row, col);
    m(row, 3) = x[1 + row];
  }
  return m;
}

// Still `s`'s camera centre in the tag's world at `x`: Rz(yaw) A p_c + t.
cv::Vec3d centre_at(const Still& s, const Parameters& x) {
  return rz(x[0]) * s.centre + cv::Vec3d(x[1], x[2], x[3]);
}

std::string still_name(const std::string& name) { return '"' + name + '"'; }

// The views, checked, by still name; the tag's square of `edge_m`.
core::Result<Problem> read_problem(const std::vector<TrackedTagView>& views,
                                   const OriginTag& tag, double edge_m,
                                   const TrackedWorldOptions& options) {
  const camera::Vec3d& up = options.up;
  if (!std::isfinite(up.x) || !std::isfinite(up.y) || !std::isfinite(up.z) ||
      !(largest(up) > 0.0)) {
    return core::Status::invalid_argument(
        std::string(kWhat) + "options.up must be finite and not zero");
  }
  if (!(edge_m > 0.0) || !std::isfinite(edge_m)) {
    return core::Status::invalid_argument(
        std::string(kWhat) + "edge_m must be finite and above 0, not " +
        number(edge_m));
  }
  VKC_TRY(check_origin_tag(tag).with_context("tracked_world_from_tag"));

  Problem p;
  p.align = alignment(up);
  const double h = 0.5 * edge_m;
  p.square = {cv::Vec3d(-h, h, 0.0), cv::Vec3d(h, h, 0.0),
              cv::Vec3d(h, -h, 0.0), cv::Vec3d(-h, -h, 0.0)};
  std::set<std::string> named;
  for (std::size_t i = 0; i < views.size(); ++i) {
    const TrackedTagView& view = views[i];
    const std::string of = std::string(kWhat) + "view " + std::to_string(i) +
                           " (still " + still_name(view.still) + ")";
    if (!named.insert(view.still).second) {
      return core::Status::invalid_argument(of + " names a still named before");
    }
    VKC_TRY(camera::check_camera_model(view.model).with_context(of));
    const camera::ImageSize& size = view.model.size;
    if (view.image_size.width != size.width ||
        view.image_size.height != size.height) {
      return core::Status::invalid_argument(
          of + " is of a " + std::to_string(view.image_size.width) + "x" +
          std::to_string(view.image_size.height) + " image, not its model's " +
          std::to_string(size.width) + "x" + std::to_string(size.height));
    }
    VKC_TRY(camera::check_rigid(view.cam_to_tracking)
                .with_context(of + ": its cam_to_tracking"));
    if (view.marker.id != tag.id) {
      return core::Status::invalid_argument(
          of + " is of marker " + std::to_string(view.marker.id) +
          ", not the tag's id " + std::to_string(tag.id));
    }
    for (const camera::Vec2d& corner : view.marker.corners) {
      if (!std::isfinite(corner.x) || !std::isfinite(corner.y)) {
        return core::Status::invalid_argument(of + " has a corner not finite");
      }
    }
    Still& s = p.stills.emplace_back();
    s.name = view.still;
    VKC_ASSIGN(s.lens, compute::cv_camera(view.model));
    const camera::Mat4d& m = view.cam_to_tracking;
    cv::Matx33d rotation;  // R_c, camera to tracking
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) rotation(row, col) = m[col][row];
    }
    s.to_camera = (p.align * rotation).t();
    s.centre = p.align * cv::Vec3d(m[3].x, m[3].y, m[3].z);
    for (const camera::Vec2d& corner : view.marker.corners) {
      s.pixels.emplace_back(corner.x, corner.y);
    }
  }
  std::sort(p.stills.begin(), p.stills.end(),
            [](const Still& a, const Still& b) { return a.name < b.name; });
  for (Still& s : p.stills) {
    VKC_ASSIGN(s.rays, compute::undistort_points(s.lens, s.pixels));
  }
  return p;
}

// One still's yaw and translation alone, in closed form. In the turned
// frame the tag's plane is level, so corner k's ray from the camera's centre
// c meets it at c + H e_k, e_k the ray's level part over its rise and H the
// plane's height above the camera. The square's corners are then a turn and
// a scale of the e_k and a shift: H Rz(yaw) e_k + Rz(yaw) c + t, a
// similarity fitted by least squares. Empty, or why it failed (a Numerical
// miss).
std::string seed_of(const Problem& p, const Still& s, Parameters& seed) {
  std::array<cv::Vec2d, 4> e;
  int falling = 0;
  for (std::size_t k = 0; k < 4; ++k) {
    const cv::Vec3d d =
        s.to_camera.t() * cv::Vec3d(s.rays[k].x, s.rays[k].y, 1.0);
    if (!(std::abs(d[2]) > 1e-12 * cv::norm(d))) {
      return "the ray to its corner " + std::to_string(k) + " is level";
    }
    falling += d[2] < 0.0 ? 1 : 0;
    e[k] = cv::Vec2d(d[0] / d[2], d[1] / d[2]);
  }
  if (falling != 0 && falling != 4) {
    return "its corners' rays meet the tag's plane on both sides of its "
           "camera";
  }
  // A square seen anywhere is a convex quadrilateral on its plane: each turn
  // along its outline the same way, by more than round-off.
  double turning = 0.0;
  for (std::size_t k = 0; k < 4; ++k) {
    const cv::Vec2d a = e[(k + 1) % 4] - e[k];
    const cv::Vec2d b = e[(k + 2) % 4] - e[(k + 1) % 4];
    const double cross = a[0] * b[1] - a[1] * b[0];
    const double sine = cross / (cv::norm(a) * cv::norm(b));
    if (!(std::abs(sine) > 1e-9) || sine * turning < 0.0) {
      return "its corners make no convex quadrilateral";
    }
    turning = sine;
  }
  cv::Vec2d e_mean(0.0, 0.0);
  cv::Vec2d x_mean(0.0, 0.0);
  for (std::size_t k = 0; k < 4; ++k) {
    e_mean += 0.25 * e[k];
    x_mean += 0.25 * cv::Vec2d(p.square[k][0], p.square[k][1]);
  }
  double along = 0.0;
  double across = 0.0;
  double spread = 0.0;
  for (std::size_t k = 0; k < 4; ++k) {
    const cv::Vec2d a = e[k] - e_mean;
    const cv::Vec2d b = cv::Vec2d(p.square[k][0], p.square[k][1]) - x_mean;
    along += a.dot(b);
    across += a[0] * b[1] - a[1] * b[0];
    spread += a.dot(a);
  }
  const double c = along / spread;   // H cos(yaw)
  const double n = across / spread;  // H sin(yaw)
  // The plane is below a camera whose rays fall to it, above one whose rays
  // rise.
  const double height = (falling == 4 ? -1.0 : 1.0) * std::hypot(c, n);
  if (!(std::abs(height) > 0.0) || !std::isfinite(height)) {
    return "its corners fix no scale";
  }
  seed[0] = std::atan2(n / height, c / height);
  const cv::Vec2d shift(x_mean[0] - (c * e_mean[0] - n * e_mean[1]),
                        x_mean[1] - (n * e_mean[0] + c * e_mean[1]));
  const cv::Vec3d turned = rz(seed[0]) * s.centre;
  seed[1] = shift[0] - turned[0];
  seed[2] = shift[1] - turned[1];
  seed[3] = -height - s.centre[2];
  return {};
}

// The seed: each still's, their median, the yaw's about the first still's.
std::string seed_all(const Problem& p, Parameters& seed) {
  std::vector<double> yaw;
  std::vector<double> t[3];
  for (const Still& s : p.stills) {
    Parameters one;
    const std::string why = seed_of(p, s, one);
    if (!why.empty()) return "still " + still_name(s.name) + ": " + why;
    // Within half a turn of the first, so a median across +-pi holds.
    yaw.push_back(yaw.empty()
                      ? one[0]
                      : yaw[0] + std::remainder(one[0] - yaw[0], 2.0 * kPi));
    for (int a = 0; a < 3; ++a) t[a].push_back(one[1 + a]);
  }
  seed[0] = compute::median(yaw);
  for (int a = 0; a < 3; ++a) seed[1 + a] = compute::median(t[a]);
  return {};
}

// Still `s`'s corners in its camera at `x`, X_c = M (Rz^T (X - t) - c), and,
// given `by`, each one's derivatives by the yaw and t.
void in_camera(const Problem& p, const Still& s, const Parameters& x,
               std::vector<cv::Point3d>& points,
               std::array<cv::Matx34d, 4>* by) {
  const double c = std::cos(x[0]);
  const double sn = std::sin(x[0]);
  const cv::Matx33d back(c, sn, 0.0, -sn, c, 0.0, 0.0, 0.0, 1.0);  // Rz^T
  const cv::Matx33d back_by_yaw(-sn, c, 0.0, -c, -sn, 0.0, 0.0, 0.0, 0.0);
  const cv::Vec3d t(x[1], x[2], x[3]);
  points.resize(4);
  const cv::Matx33d by_t = -1.0 * (s.to_camera * back);
  for (std::size_t k = 0; k < 4; ++k) {
    const cv::Vec3d xc = s.to_camera * (back * (p.square[k] - t) - s.centre);
    points[k] = cv::Point3d(xc[0], xc[1], xc[2]);
    if (by == nullptr) continue;
    const cv::Vec3d by_yaw = s.to_camera * (back_by_yaw * (p.square[k] - t));
    for (int row = 0; row < 3; ++row) {
      (*by)[k](row, 0) = by_yaw[row];
      for (int col = 0; col < 3; ++col) (*by)[k](row, 1 + col) = by_t(row, col);
    }
  }
}

enum class Outcome { Ok, Behind, NotFinite };

// The energy, sum |e|^2 over every still's corners at `x`; and, given `jtj`
// and `jte` (zeroed, 4 square and long), J^T J and J^T e.
Outcome evaluate(const Problem& p, const Parameters& x, double& energy,
                 cv::Mat_<double>* jtj, cv::Mat_<double>* jte) {
  energy = 0.0;
  std::vector<cv::Point3d> points;
  std::array<cv::Matx34d, 4> by;
  Projection at;
  for (const Still& s : p.stills) {
    in_camera(p, s, x, points, jtj != nullptr ? &by : nullptr);
    detail::project(
        s.lens, Pose::all(0.0), Pose::all(0.0), points, &s.pixels,
        jtj != nullptr ? Derivatives::Extrinsics : Derivatives::None, at);
    for (std::size_t k = 0; k < points.size(); ++k) {
      if (!(at.depths[k] > 0.0)) {
        return std::isnan(at.depths[k]) ? Outcome::NotFinite : Outcome::Behind;
      }
      const cv::Vec2d& e = at.residuals[k];
      const double s2 = e.dot(e);
      if (!std::isfinite(s2)) return Outcome::NotFinite;
      energy += s2;
      if (jtj == nullptr) continue;
      // dp/dX_c is dp/dtvec, the camera at the identity.
      const cv::Matx<double, 2, 4> j =
          at.by_camera[k].get_minor<2, 3>(0, 3) * by[k];
      const cv::Matx44d h = j.t() * j;
      const cv::Vec4d g = j.t() * e;
      for (int a = 0; a < kParameters; ++a) {
        for (int b = 0; b < kParameters; ++b) (*jtj)(a, b) += h(a, b);
        (*jte)(a) += g[a];
      }
    }
  }
  return std::isfinite(energy) ? Outcome::Ok : Outcome::NotFinite;
}

// The yaw and t by least squares on cv::LevMarq, from `x`. Empty, or why it
// failed (a Numerical miss).
// TODO: one home for this cv::LevMarq normal-equations driver, shared with
// bundle_adjust.cpp's and world_tag.cpp's.
std::string solve(const Problem& p, Parameters& x) {
  double energy = 0.0;
  if (evaluate(p, x, energy, nullptr, nullptr) != Outcome::Ok) {
    return "at the seed a corner lies behind a still's camera, or out of "
           "range";
  }
  Outcome failure = Outcome::Ok;
  Parameters probe = x;
  const auto unpack = [&](const cv::Mat& m) {
    const double* v = m.ptr<double>();
    for (int i = 0; i < kParameters; ++i) {
      if (!std::isfinite(v[i])) return false;
      probe[i] = v[i];
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
  cv::Mat_<double> m(kParameters, 1);
  for (int i = 0; i < kParameters; ++i) m(i) = x[i];
  cv::LevMarq solver(m, callback,
                     cv::LevMarq::Settings()
                         .setRelEnergyDeltaTolerance(kLevMarqTolerance)
                         .setStepNormTolerance(kLevMarqTolerance)
                         .setCheckMinGradient(false),
                     cv::noArray(), cv::MatrixType::DENSE,
                     cv::VariableType::LINEAR, false, cv::DECOMP_CHOLESKY);
  const cv::LevMarq::Report report = solver.optimize();
  if (failure != Outcome::Ok || !unpack(m)) {
    return "a corner went behind a still's camera or out of range after " +
           std::to_string(report.iters) + " iterations";
  }
  if (!report.found || !std::isfinite(report.energy)) {
    return "cv::LevMarq did not converge in " + std::to_string(report.iters) +
           " iterations (energy " + number(energy) + " to " +
           number(report.energy) + ")";
  }
  x = probe;
  return {};
}

// Still `s`'s fit at `x`.
TrackedStillFit fit_of(const Problem& p, const Still& s, const Parameters& x) {
  std::vector<cv::Point3d> points;
  in_camera(p, s, x, points, nullptr);
  Projection at;
  detail::project(s.lens, Pose::all(0.0), Pose::all(0.0), points, &s.pixels,
                  Derivatives::None, at);
  TrackedStillFit fit;
  fit.still = s.name;
  double sum = 0.0;
  for (const cv::Vec2d& e : at.residuals) {
    const double s2 = e.dot(e);
    sum += s2;
    fit.worst_px = std::max(fit.worst_px, std::sqrt(s2));
  }
  fit.rms_px = std::sqrt(sum / static_cast<double>(at.residuals.size()));
  const cv::Vec3d c = centre_at(s, x);
  fit.range_m = cv::norm(c);
  fit.off_normal_deg =
      compute::angle_deg(camera::Vec3d(c[0], c[1], c[2]), {0.0, 0.0, 1.0});
  return fit;
}

TrackedWorld miss(TagMiss why, std::string still, std::string reason) {
  TrackedWorld found;
  found.miss = why;
  found.still = std::move(still);
  found.reason = std::move(reason);
  return found;
}

core::Result<TrackedWorld> place(const std::vector<TrackedTagView>& views,
                                 const OriginTag& tag, double edge_m,
                                 const TrackedWorldOptions& options) {
  VKC_ASSIGN(const Problem p, read_problem(views, tag, edge_m, options));
  if (p.stills.empty()) {
    return miss(TagMiss::NotSeen, {}, "no still saw the tag");
  }
  Parameters seed;
  std::string why = seed_all(p, seed);
  Parameters x = seed;
  if (why.empty()) why = solve(p, x);
  if (!why.empty()) return miss(TagMiss::Numerical, {}, why);

  // The guard: every camera above the tag's plane.
  for (const Still& s : p.stills) {
    const double height = centre_at(s, x)[2];
    if (height > 0.0) continue;
    return miss(TagMiss::CameraBelowTag, s.name,
                "still " + still_name(s.name) + "'s camera is " +
                    number(0.0 - height) +
                    " m below the tag's plane: are the stills' poses in "
                    "sensor's camera axes (+z forward, +y down; "
                    "cv_from_gl_camera for ARKit's), and is options.up the "
                    "tracking world's up?");
  }

  TrackedTagFit fit;
  fit.tracking_to_world = compute::matrix_from_cv(transform_at(p, x));
  fit.edge_m = edge_m;
  double sum = 0.0;
  for (const Still& s : p.stills) {
    TrackedStillFit& one = fit.stills.emplace_back(fit_of(p, s, x));
    sum += one.rms_px * one.rms_px;
    fit.worst_px = std::max(fit.worst_px, one.worst_px);
  }
  fit.rms_px = std::sqrt(sum / static_cast<double>(p.stills.size()));
  if (fit.rms_px > kMaxTrackedRmsPx) {
    return miss(TagMiss::PoorFit, {},
                "the tag's corners fit the stills at " + number(fit.rms_px) +
                    " px RMS, above the " + number(kMaxTrackedRmsPx) +
                    " px a fit takes: a still's pose is off, or the tag "
                    "moved between stills");
  }
  // The gate in metres: the pixels barely see a camera moved along its line
  // of sight, which a still whose pose is off can pull the fit to.
  double gap = 0.0;
  const Still* farthest = nullptr;
  for (const Still& s : p.stills) {
    const double d = cv::norm(centre_at(s, x) - centre_at(s, seed));
    if (farthest != nullptr && !(d > gap)) continue;
    gap = d;
    farthest = &s;
  }
  if (!(gap <= kMaxTrackedSeedGapM)) {
    return miss(TagMiss::PoorFit, {},
                "the fit places still " + still_name(farthest->name) +
                    "'s camera " + number(gap) +
                    " m from where the stills' median puts it, above the " +
                    number(kMaxTrackedSeedGapM) +
                    " m a fit takes: a still's pose is off, or the tag "
                    "moved between stills");
  }
  TrackedWorld found;
  found.fit = std::move(fit);
  return found;
}

// What differs between the array's world and the tag sought, or empty.
std::string world_difference(const camera::WorldFrame& world,
                             const OriginTag& sought) {
  if (!world.apriltag) {
    return world.sensor.empty()
               ? "the array's world names no tag"
               : "the array's world is sensor \"" + world.sensor + "\"";
  }
  if (!world.sensor.empty()) {
    return "the array's world names sensor \"" + world.sensor +
           "\" beside its tag";
  }
  const camera::AprilTagWorld& t = *world.apriltag;
  const std::string family = apriltag_family(sought.dictionary);
  std::string differs;
  const auto add = [&](const std::string& what) {
    differs += (differs.empty() ? "" : ", ") + what;
  };
  if (t.family != family) {
    add("its family " + t.family + " is not the tag sought's " +
        (family.empty() ? std::string(to_string(sought.dictionary)) : family));
  }
  if (t.id != sought.id) {
    add("its id " + std::to_string(t.id) + " is not " +
        std::to_string(sought.id));
  }
  if (t.size_m != sought.edge_m) {
    add("its size " + number(t.size_m) + " m is not the printed " +
        number(sought.edge_m) + " m");
  }
  return differs.empty()
             ? differs
             : "the array's world is not the tag sought: " + differs;
}

}  // namespace

core::Result<TrackedWorld> tracked_world_from_tag(
    const std::vector<TrackedTagView>& views, const OriginTag& tag,
    double edge_m, const TrackedWorldOptions& options) {
  return compute::guarded(
      "tracked_world_from_tag", [&]() -> core::Result<TrackedWorld> {
        VKC_ASSIGN(TrackedWorld found, place(views, tag, edge_m, options));
        found.sought = tag;
        return found;
      });
}

core::Result<camera::TrackedRegistration> register_tracked(
    const camera::TrackedStills& stills, const camera::WorldFrame& world,
    const TrackedWorld& found) {
  return compute::guarded(
      "register_tracked", [&]() -> core::Result<camera::TrackedRegistration> {
        if (!found.fit) {
          return core::Status::invalid_argument(
              std::string(kRegister) + "the tracking world was not placed (" +
              to_string(found.miss) + "): " + found.reason);
        }
        const std::string differs = world_difference(world, found.sought);
        if (!differs.empty()) {
          return core::Status::invalid_argument(kRegister + differs);
        }
        VKC_TRY(camera::validate_tracked_stills(stills).with_context(
            "register_tracked"));
        std::set<std::string> images;
        for (const camera::TrackedStill& s : stills.stills) {
          images.insert(s.image);
        }
        for (const TrackedStillFit& s : found.fit->stills) {
          if (images.count(s.still) == 0) {
            return core::Status::invalid_argument(
                kRegister + std::string("the fit's still ") +
                still_name(s.still) + " is none of the stills' images");
          }
        }
        camera::TrackedRegistration reg;
        reg.sensor = stills.sensor;
        reg.session = stills.session;
        reg.world_epoch = stills.world_epoch;
        reg.world = world;
        reg.tracking_to_world = found.fit->tracking_to_world;
        reg.rms_px = found.fit->rms_px;
        reg.views = static_cast<std::uint32_t>(found.fit->stills.size());
        reg.edge_m = found.fit->edge_m;
        VKC_TRY(camera::validate_tracked_registration(reg).with_context(
            "register_tracked"));
        return reg;
      });
}

}  // namespace volumetric_kit::calib::solve
