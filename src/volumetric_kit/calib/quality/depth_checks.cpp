// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/depth_checks.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;
using camera::Mat3d;
using camera::Mat4d;
using camera::Vec2d;
using camera::Vec3d;

bool finite(const Vec3d& v) noexcept {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// TODO: Give a point's rigid transform one home (recon's camera geometry),
// for this and the copies in the tests, the solve and the metrics.
Vec3d apply(const Mat4d& m, const Vec3d& p) noexcept {
  return Vec3d(m * glm::dvec4(p, 1.0));
}

// The largest of `v`'s magnitudes. Divided by it first, a vector of any
// finite length but 0 has a length from 1 to sqrt(3), which neither
// overflows nor underflows.
double largest(const Vec3d& v) noexcept {
  return std::max({std::abs(v.x), std::abs(v.y), std::abs(v.z)});
}

// How far along `direction` from the origin a ray meets the plane through
// `point` with `normal`; empty when it meets it behind the origin, never
// (along the plane) or not finitely.
std::optional<double> meet(const Vec3d& direction, const Vec3d& point,
                           const Vec3d& normal) noexcept {
  const double s = glm::dot(normal, point) / glm::dot(normal, direction);
  if (!(s > 0.0) || !std::isfinite(s)) return std::nullopt;
  return s;
}

core::Status check_region(const BoardRegion& region) {
  core::Status rigid = camera::check_rigid(region.board_to_color);
  if (!rigid.ok()) {
    return std::move(rigid).with_context("the region's board_to_color");
  }
  rigid = camera::check_rigid(region.depth_to_board);
  if (!rigid.ok()) {
    return std::move(rigid).with_context("the region's depth_to_board");
  }
  // Written so a NaN fails it; an infinity fails the size's finiteness.
  if (!(region.min.x < region.max.x && region.min.y < region.max.y) ||
      !std::isfinite(region.max.x - region.min.x) ||
      !std::isfinite(region.max.y - region.min.y)) {
    return core::Status::invalid_argument(
        "the region's rectangle must be finite and not empty");
  }
  return {};
}

core::Status check_plane(const DepthPlane& plane) {
  if (plane.inliers > plane.members ||
      plane.out_of_range > plane.members - plane.inliers) {
    return core::Status::invalid_argument(
        "the plane's inliers and out-of-range pixels exceed its members");
  }
  if (plane.valid && (!finite(plane.point) || !finite(plane.normal) ||
                      !(largest(plane.normal) > 0.0))) {
    return core::Status::invalid_argument(
        "a valid plane needs a finite point and a finite, nonzero normal");
  }
  return {};
}

RatioStats stats_of(const std::vector<double>& values) noexcept {
  RatioStats out;
  out.count = values.size();
  if (values.empty()) return out;
  double sum = 0.0;
  for (const double v : values) sum += v;
  out.mean = sum / static_cast<double>(values.size());
  if (values.size() < 2) return out;
  double squares = 0.0;
  for (const double v : values) squares += (v - out.mean) * (v - out.mean);
  out.sd = std::sqrt(squares / static_cast<double>(values.size() - 1));
  return out;
}

// The comparisons are written so a NaN gives `Off`.
DepthVerdict scale_verdict(const RatioStats& ratio,
                           const DepthCheckOptions& options) noexcept {
  if (ratio.count == 0) return DepthVerdict::Insufficient;
  return std::abs(ratio.mean - 1.0) <= options.max_scale_error
             ? DepthVerdict::Ok
             : DepthVerdict::Off;
}

DepthVerdict plane_verdict(const CameraDepth& camera,
                           const DepthCheckOptions& options) noexcept {
  if (camera.views == 0) return DepthVerdict::Insufficient;
  return std::abs(camera.offset_mm) <= options.max_offset_mm &&
                 camera.angle_deg <= options.max_angle_deg
             ? DepthVerdict::Ok
             : DepthVerdict::Off;
}

DepthVerdict pair_verdict(const PairAgreement& pair,
                          const DepthCheckOptions& options) noexcept {
  if (pair.overlap < options.min_overlap) return DepthVerdict::Insufficient;
  // Fewer than half the samples in the window: the median magnitude over
  // every sample is beyond it.
  if (pair.in_window < pair.overlap - pair.in_window) return DepthVerdict::Off;
  return std::abs(pair.signed_median_mm) <= options.max_signed_median_mm &&
                 pair.median_abs_mm <= options.max_median_abs_mm
             ? DepthVerdict::Ok
             : DepthVerdict::Off;
}

core::Status check_options(const DepthCheckOptions& options) {
  if (!(options.inset_m >= 0.0) || !std::isfinite(options.inset_m)) {
    return core::Status::invalid_argument(
        "options.inset_m must be finite and not negative");
  }
  if (options.min_overlap < 1) {
    return core::Status::invalid_argument(
        "options.min_overlap must be at least 1");
  }
  for (const auto& [name, bound] :
       {std::pair{"max_scale_error", options.max_scale_error},
        std::pair{"max_offset_mm", options.max_offset_mm},
        std::pair{"max_angle_deg", options.max_angle_deg},
        std::pair{"max_signed_median_mm", options.max_signed_median_mm},
        std::pair{"max_median_abs_mm", options.max_median_abs_mm}}) {
    if (!(bound > 0.0) || !std::isfinite(bound)) {
      return core::Status::invalid_argument(std::string("options.") + name +
                                            " must be positive and finite");
    }
  }
  return {};
}

std::string pair_name(const PairAgreement& pair) {
  std::string name = "the pair from camera " + std::to_string(pair.from) +
                     " to camera " + std::to_string(pair.to);
  if (pair.capture) name += " in capture " + std::to_string(*pair.capture);
  return name;
}

// Every ordered pair of `k` cameras for one capture (or pooled): as given, or
// with an overlap of 0 when not given; each with its verdict.
std::vector<PairCheck> every_pair(
    std::size_t k, const std::optional<std::int64_t>& capture,
    const std::map<
        std::tuple<std::optional<std::int64_t>, std::size_t, std::size_t>,
        const PairAgreement*>& given,
    const DepthCheckOptions& options) {
  std::vector<PairCheck> out;
  out.reserve(k * (k - 1));
  for (std::size_t from = 0; from < k; ++from) {
    for (std::size_t to = 0; to < k; ++to) {
      if (from == to) continue;
      PairCheck check;
      const auto found = given.find({capture, from, to});
      if (found != given.end()) {
        check.agreement = *found->second;
      } else {
        check.agreement.from = from;
        check.agreement.to = to;
        check.agreement.capture = capture;
      }
      check.verdict = pair_verdict(check.agreement, options);
      out.push_back(std::move(check));
    }
  }
  return out;
}

}  // namespace

core::Result<BoardRegion> board_region(const compute::Board& board,
                                       const Mat4d& board_to_color,
                                       const Mat4d& depth_to_color,
                                       const DepthCheckOptions& options) {
  constexpr const char* kWhat = "board_region";
  return compute::guarded(kWhat, [&]() -> core::Result<BoardRegion> {
    core::Status rigid = camera::check_rigid(board_to_color);
    if (!rigid.ok()) {
      return std::move(rigid).with_context(std::string(kWhat) +
                                           ": board_to_color");
    }
    rigid = camera::check_rigid(depth_to_color);
    if (!rigid.ok()) {
      return std::move(rigid).with_context(std::string(kWhat) +
                                           ": depth_to_color");
    }
    core::Status checked = check_options(options);
    if (!checked.ok()) return std::move(checked).with_context(kWhat);
    // The chessboard spans twice its centre, from the board frame's origin:
    // an inset leaves some of it while short of the centre on either side.
    const cv::Point3d centre = board.centre();
    const double inset_m = options.inset_m;
    if (!(inset_m < std::min(centre.x, centre.y))) {
      return core::Status::invalid_argument(
          std::string(kWhat) +
          ": options.inset_m must leave some of the board");
    }
    BoardRegion region;
    region.board_to_color = board_to_color;
    region.depth_to_board =
        camera::rigid_inverse(board_to_color) * depth_to_color;
    region.min = Vec2d(inset_m);
    region.max = Vec2d(2.0 * centre.x - inset_m, 2.0 * centre.y - inset_m);
    return region;
  });
}

bool in_board_region(const BoardRegion& region, const Vec3d& ray) noexcept {
  // The ray from the depth camera's centre, in the board frame, meets z = 0.
  const Vec3d origin(region.depth_to_board[3]);
  const Vec3d direction = Mat3d(region.depth_to_board) * ray;
  const std::optional<double> s =
      meet(direction, -origin, Vec3d(0.0, 0.0, 1.0));
  if (!s) return false;
  const Vec3d at = origin + *s * direction;
  return at.x >= region.min.x && at.x <= region.max.x && at.y >= region.min.y &&
         at.y <= region.max.y;
}

core::Result<BoardDepthCheck> board_depth_check(
    const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
    const Observation& view, const BoardRegion& region,
    const DepthPlane& plane) {
  constexpr const char* kWhat = "board_depth_check";
  return compute::guarded(kWhat, [&]() -> core::Result<BoardDepthCheck> {
    core::Result<solve::RigLenses> rig = solve::check_rig(cameras, 0);
    if (!rig.ok()) return rig.status().with_context(kWhat);
    const auto named =
        std::find(rig->ids.begin(), rig->ids.end(), view.camera_id);
    if (named == rig->ids.end()) {
      return core::Status::invalid_argument(
          std::string(kWhat) + ": the view names camera " + view.camera_id +
          ", which the rig does not have");
    }
    const auto c = static_cast<std::size_t>(named - rig->ids.begin());
    core::Status checked = check_region(region);
    if (checked.ok()) checked = check_plane(plane);
    if (!checked.ok()) return std::move(checked).with_context(kWhat);
    const std::size_t corners = board.object_points().size();
    std::vector<bool> seen(corners, false);
    for (const DetectedPoint& p : view.points) {
      if (p.id < 0 || static_cast<std::size_t>(p.id) >= corners ||
          seen[static_cast<std::size_t>(p.id)]) {
        return core::Status::invalid_argument(
            std::string(kWhat) + ": corner " + std::to_string(p.id) +
            " is not on the board, or is detected twice");
      }
      seen[static_cast<std::size_t>(p.id)] = true;
    }

    BoardDepthCheck out;
    out.camera = c;
    out.capture = view.frame_id;
    out.plane = plane;
    if (!plane.valid) return out;

    // The plane against the solved board, z = 0, in the board frame.
    const Vec3d normal = glm::normalize(plane.normal / largest(plane.normal));
    const cv::Point3d centre = board.centre();
    const double turn = compute::angle_deg(normal, Vec3d(0.0, 0.0, 1.0));
    out.angle_deg = std::min(turn, 180.0 - turn);
    if (normal.z != 0.0) {
      const double z = plane.point.z - (normal.x * (centre.x - plane.point.x) +
                                        normal.y * (centre.y - plane.point.y)) /
                                           normal.z;
      out.offset_mm = 1000.0 * z;
    }

    if (view.points.empty()) return out;
    // Each corner's ray in the colour camera met with the plane there.
    const Vec3d normal_c = Mat3d(region.board_to_color) * normal;
    const Vec3d point_c = apply(region.board_to_color, plane.point);
    std::vector<cv::Point2d> pixels;
    pixels.reserve(view.points.size());
    for (const DetectedPoint& p : view.points) {
      pixels.emplace_back(p.pixel.x, p.pixel.y);
    }
    core::Result<std::vector<cv::Point2d>> rays =
        compute::undistort_points(rig->lenses[c], pixels);
    if (!rays.ok()) return rays.status().with_context(kWhat);
    std::vector<std::optional<Vec3d>> hit(corners);
    for (std::size_t i = 0; i < view.points.size(); ++i) {
      const Vec3d ray((*rays)[i].x, (*rays)[i].y, 1.0);
      if (const std::optional<double> s = meet(ray, point_c, normal_c)) {
        hit[static_cast<std::size_t>(view.points[i].id)] = *s * ray;
      }
    }
    // Ids are row-major, `cols` to a row (OpenCV's ChArUco order).
    const auto cols = static_cast<std::size_t>(board.target().cols);
    const double square = board.target().square_size_m;
    for (std::size_t a = 0; a < corners; ++a) {
      if (!hit[a]) continue;
      for (const std::size_t b :
           {a % cols + 1 < cols ? a + 1 : corners, a + cols}) {
        if (b < corners && hit[b]) {
          out.ratios.push_back(glm::length(*hit[b] - *hit[a]) / square);
        }
      }
    }
    return out;
  });
}

const char* to_string(DepthVerdict verdict) noexcept {
  switch (verdict) {
    case DepthVerdict::Ok:
      return "ok";
    case DepthVerdict::Off:
      return "off";
    case DepthVerdict::Insufficient:
      return "insufficient";
  }
  return "unknown";
}

core::Result<DepthSummary> summarize_depth_checks(
    std::size_t cameras, const std::vector<BoardDepthCheck>& views,
    const std::vector<PairAgreement>& pairs, const DepthCheckOptions& options) {
  constexpr const char* kWhat = "summarize_depth_checks";
  return compute::guarded(kWhat, [&]() -> core::Result<DepthSummary> {
    const auto refuse = [&](const std::string& why) {
      return core::Status::invalid_argument(std::string(kWhat) + ": " + why);
    };
    if (cameras == 0) return refuse("the rig has no cameras");
    core::Status checked = check_options(options);
    if (!checked.ok()) return std::move(checked).with_context(kWhat);

    DepthSummary out;
    out.options = options;
    out.views = views;
    std::sort(out.views.begin(), out.views.end(),
              [](const BoardDepthCheck& a, const BoardDepthCheck& b) {
                return std::tie(a.capture, a.camera) <
                       std::tie(b.capture, b.camera);
              });
    for (std::size_t v = 0; v < out.views.size(); ++v) {
      const BoardDepthCheck& view = out.views[v];
      if (view.camera >= cameras) {
        return refuse("a view names camera " + std::to_string(view.camera) +
                      ", which the rig does not have");
      }
      if (v > 0 && out.views[v - 1].camera == view.camera &&
          out.views[v - 1].capture == view.capture) {
        return refuse("camera " + std::to_string(view.camera) +
                      " has two views of capture " +
                      std::to_string(view.capture));
      }
    }
    std::map<std::tuple<std::optional<std::int64_t>, std::size_t, std::size_t>,
             const PairAgreement*>
        given;
    std::set<std::int64_t> captures;
    for (const PairAgreement& pair : pairs) {
      if (pair.from >= cameras || pair.to >= cameras || pair.from == pair.to) {
        return refuse(pair_name(pair) + " is not of two of the rig's cameras");
      }
      if (pair.in_window > pair.overlap) {
        return refuse(pair_name(pair) +
                      " has more samples in the window than overlap");
      }
      if (!given.emplace(std::tuple{pair.capture, pair.from, pair.to}, &pair)
               .second) {
        return refuse(pair_name(pair) + " appears twice");
      }
      if (pair.capture) captures.insert(*pair.capture);
    }

    out.cameras.resize(cameras);
    std::vector<std::vector<double>> ratios(cameras);
    std::vector<std::vector<double>> offsets(cameras);
    std::vector<std::vector<double>> angles(cameras);
    std::vector<double> every_ratio;
    for (const BoardDepthCheck& view : out.views) {
      if (!view.plane.valid) continue;
      const std::size_t c = view.camera;
      ++out.cameras[c].views;
      ratios[c].insert(ratios[c].end(), view.ratios.begin(), view.ratios.end());
      every_ratio.insert(every_ratio.end(), view.ratios.begin(),
                         view.ratios.end());
      if (!std::isnan(view.offset_mm)) offsets[c].push_back(view.offset_mm);
      if (!std::isnan(view.angle_deg)) angles[c].push_back(view.angle_deg);
    }
    for (std::size_t c = 0; c < cameras; ++c) {
      CameraDepth& camera = out.cameras[c];
      camera.ratio = stats_of(ratios[c]);
      camera.scale = scale_verdict(camera.ratio, options);
      camera.offset_mm = compute::median(offsets[c]);
      camera.angle_deg = compute::median(angles[c]);
      camera.plane = plane_verdict(camera, options);
    }
    out.ratio = stats_of(every_ratio);
    out.scale = scale_verdict(out.ratio, options);
    out.pairs = every_pair(cameras, std::nullopt, given, options);
    for (const std::int64_t capture : captures) {
      out.captures.push_back(
          {capture, every_pair(cameras, capture, given, options)});
    }
    return out;
  });
}

}  // namespace volumetric_kit::calib::quality
