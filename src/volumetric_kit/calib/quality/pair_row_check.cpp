// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/pair_row_check.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <utility>

#include <opencv2/core.hpp>

#include "checks.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"
#include "volumetric_kit/sensor/camera/stereo_rectification.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = sensor::camera;
using detail::number;
using detail::student_t;
using Verdict = PairRowCheck::Verdict;

constexpr const char* kWho = "check_pair_rows";
constexpr int kColumns = 3;
constexpr std::array<const char*, kPairRowCells> kCellNames = {
    "top-left", "top",         "top-right", "left",        "centre",
    "right",    "bottom-left", "bottom",    "bottom-right"};

core::Status check_options(const PairRowCheckOptions& o) {
  const auto refuse = [](const char* name, const char* rule) {
    return core::Status::invalid_argument(std::string(kWho) + ": options." +
                                          name + " must be " + rule);
  };
  if (o.rectified.width == 0 || o.rectified.height == 0) {
    return refuse("rectified", "the runtime's size, neither side 0");
  }
  if (!(o.alpha >= 0.0 && o.alpha <= 1.0)) return refuse("alpha", "in [0, 1]");
  if (!(o.max_cell_mean_px > 0.0) || !std::isfinite(o.max_cell_mean_px)) {
    return refuse("max_cell_mean_px", "positive and finite");
  }
  if (o.min_cell_corners < 2) return refuse("min_cell_corners", "at least 2");
  if (o.min_cells < 1 || o.min_cells > kPairRowCells) {
    return refuse("min_cells", "in [1, 9]");
  }
  return {};
}

// Each capture's view by the left and the right camera; InvalidArgument for a
// camera with two views of one capture.
using CaptureViews = std::map<std::int64_t, std::array<const Observation*, 2>>;

core::Result<CaptureViews> pair_views(const std::vector<Observation>& views,
                                      std::string_view left,
                                      std::string_view right) {
  CaptureViews out;
  for (const Observation& view : views) {
    const int side = view.camera_id == left    ? 0
                     : view.camera_id == right ? 1
                                               : -1;
    if (side < 0) continue;
    const Observation*& slot =
        out[view.frame_id][static_cast<std::size_t>(side)];
    if (slot != nullptr) {
      return core::Status::invalid_argument(
          std::string(kWho) + ": camera \"" + view.camera_id +
          "\" has two views of capture " + std::to_string(view.frame_id));
    }
    slot = &view;
  }
  return out;
}

// Where `pixels` of one camera land in its rectified view: undistorted to
// convergence, turned by `rotation` (R1 or R2), projected by `k`. NaN for a
// ray that does not reach the view's front.
core::Result<std::vector<camera::Vec2d>> rectify_pixels(
    const camera::CameraModel& model, const camera::Mat3d& rotation,
    const camera::PinholeIntrinsics& k,
    const std::vector<cv::Point2d>& pixels) {
  VKC_ASSIGN(const compute::CvCamera lens, compute::cv_camera(model));
  VKC_ASSIGN(const std::vector<cv::Point2d> rays,
             compute::undistort_points(lens, pixels));
  std::vector<camera::Vec2d> out;
  out.reserve(rays.size());
  for (const cv::Point2d& ray : rays) {
    const camera::Vec3d q = rotation * camera::Vec3d(ray.x, ray.y, 1.0);
    if (!(q.z > 0.0)) {
      out.emplace_back(std::numeric_limits<double>::quiet_NaN());
      continue;
    }
    out.emplace_back(k.fx * q.x / q.z + k.cx, k.fy * q.y / q.z + k.cy);
  }
  return out;
}

// The gate over the cells' Δy: fills the cells, the verdict and its reason.
void judge_cells(const std::array<std::vector<double>, kPairRowCells>& cells,
                 PairRowCheck& out) {
  const PairRowCheckOptions& o = out.options;
  int judged = 0;
  std::array<bool, kColumns> column_judged{};
  std::string failed;
  for (std::size_t c = 0; c < cells.size(); ++c) {
    const std::vector<double>& dy = cells[c];
    const double n = static_cast<double>(dy.size());
    out.cell_corners[c] = dy.size();
    if (dy.size() < o.min_cell_corners) continue;
    double sum = 0.0;
    for (const double d : dy) sum += d;
    const double mean = sum / n;
    double squares = 0.0;
    for (const double d : dy) squares += (d - mean) * (d - mean);
    const double standard_error = std::sqrt(squares / (n - 1.0) / n);
    const double limit =
        std::max(o.max_cell_mean_px, student_t(dy.size()) * standard_error);
    out.cell_mean_px[c] = mean;
    out.cell_limit_px[c] = limit;
    ++judged;
    column_judged[c % kColumns] = true;
    if (std::fabs(mean) <= limit) continue;
    failed += failed.empty() ? "" : "; ";
    failed += number(mean) + " px in the view's " + kCellNames[c] +
              ", beyond " + number(limit) + " px";
  }
  if (!failed.empty()) {
    out.verdict = Verdict::Fail;
    out.reason = "the rows differ on average by " + failed;
    return;
  }
  if (judged < o.min_cells || !column_judged[0] ||
      !column_judged[kColumns - 1]) {
    std::string short_cells;
    for (std::size_t c = 0; c < cells.size(); ++c) {
      if (out.cell_corners[c] >= o.min_cell_corners) continue;
      short_cells += short_cells.empty() ? "" : ", ";
      short_cells += std::string(kCellNames[c]) + " (" +
                     std::to_string(out.cell_corners[c]) + ")";
    }
    out.verdict = Verdict::Insufficient;
    out.reason = "too few corners seen by both cameras in the view's " +
                 short_cells + ": " + std::to_string(o.min_cell_corners) +
                 " in each of " + std::to_string(o.min_cells) +
                 " of its 9 parts, both sides among them; capture the board "
                 "there";
    return;
  }
  out.verdict = Verdict::Pass;
}

core::Result<PairRowCheck> check(const camera::ArrayCalibration& solved,
                                 std::string_view left, std::string_view right,
                                 const std::vector<Observation>& views,
                                 const PairRowCheckOptions& options) {
  VKC_TRY(check_options(options));
  VKC_ASSIGN(const CaptureViews captures, pair_views(views, left, right));
  PairRowCheck out;
  out.options = options;
  out.left = std::string(left);
  out.right = std::string(right);

  // stereo_calibration's refusals are the caller's (an id, a lens, a pose);
  // rectify_stereo's, past the options checked above, are the pair's.
  const core::Result<sensor::StereoCalibration> pair =
      sensor::stereo_calibration(solved, left, right);
  if (!pair.ok()) return pair.status().with_context(kWho);
  const core::Result<sensor::StereoRectification> rectified =
      sensor::rectify_stereo(*pair, options.rectified, options.alpha);
  const auto refused = [](const core::Status& s) {
    return s.domain() == core::Status::Code::InvalidArgument;
  };
  if (!rectified.ok()) {
    if (!refused(rectified.status())) {
      return rectified.status().with_context(kWho);
    }
    // Named the other way round: is only the order wrong, or the pair?
    const core::Result<sensor::StereoCalibration> reversed =
        sensor::stereo_calibration(solved, right, left);
    if (!reversed.ok()) return reversed.status().with_context(kWho);
    const core::Result<sensor::StereoRectification> other =
        sensor::rectify_stereo(*reversed, options.rectified, options.alpha);
    if (!other.ok() && !refused(other.status())) {
      return other.status().with_context(kWho);
    }
    if (other.ok()) {
      out.verdict = Verdict::Swapped;
      out.reason = "\"" + out.right + "\" is the left camera: name it first, " +
                   "then \"" + out.left + "\"";
    } else {
      out.verdict = Verdict::Refused;
      out.reason = rectified.status().message();
    }
    return out;
  }
  const sensor::StereoRectification& rect = *rectified;
  out.baseline_m = rect.rig.baseline;

  std::array<std::vector<cv::Point2d>, 2> pixels;
  std::vector<std::int64_t> capture_of;
  for (const auto& [capture, seen] : captures) {
    if (seen[0] == nullptr || seen[1] == nullptr) continue;
    std::map<int, camera::Vec2d> right_corners;
    for (const DetectedPoint& p : seen[1]->points) {
      right_corners.emplace(p.id, p.pixel);
    }
    for (const DetectedPoint& p : seen[0]->points) {
      const auto match = right_corners.find(p.id);
      if (match == right_corners.end()) continue;
      pixels[0].emplace_back(p.pixel.x, p.pixel.y);
      pixels[1].emplace_back(match->second.x, match->second.y);
      capture_of.push_back(capture);
    }
  }

  const camera::PinholeIntrinsics& k = rect.rig.camera.intrinsics;
  const camera::Mat3d r1 =
      glm::transpose(camera::Mat3d(rect.rig.rectified_to_left));
  const core::Result<std::vector<camera::Vec2d>> in_left =
      rectify_pixels(pair->left, r1, k, pixels[0]);
  if (!in_left.ok()) return in_left.status().with_context(kWho);
  const core::Result<std::vector<camera::Vec2d>> in_right =
      rectify_pixels(pair->right, rect.right_to_rectified, k, pixels[1]);
  if (!in_right.ok()) return in_right.status().with_context(kWho);

  const int w = static_cast<int>(options.rectified.width);
  const int h = static_cast<int>(options.rectified.height);
  std::array<std::vector<double>, kPairRowCells> cells;
  std::vector<double> magnitudes;
  magnitudes.reserve(in_left->size());
  std::optional<std::int64_t> last_capture;
  for (std::size_t i = 0; i < in_left->size(); ++i) {
    const camera::Vec2d& l = (*in_left)[i];
    const camera::Vec2d& r = (*in_right)[i];
    const std::optional<int> cell =
        compute::grid_cell(l.x, l.y, w, h, kColumns, kColumns);
    if (!cell || !compute::grid_cell(r.x, r.y, w, h, kColumns, kColumns)) {
      continue;
    }
    const double dy = l.y - r.y;
    cells[static_cast<std::size_t>(*cell)].push_back(dy);
    magnitudes.push_back(std::fabs(dy));
    if (last_capture != capture_of[i]) ++out.captures;
    last_capture = capture_of[i];
  }
  out.corners = magnitudes.size();
  out.dy_median_px = compute::median(magnitudes);
  out.dy_p95_px = compute::quantile(std::move(magnitudes), 0.95);
  judge_cells(cells, out);
  return out;
}

}  // namespace

const char* to_string(PairRowCheck::Verdict verdict) noexcept {
  switch (verdict) {
    case Verdict::Pass:
      return "pass";
    case Verdict::Fail:
      return "fail";
    case Verdict::Insufficient:
      return "insufficient";
    case Verdict::Swapped:
      return "swapped";
    case Verdict::Refused:
      return "refused";
  }
  return "unknown";
}

const char* pair_row_cell_name(std::size_t cell) noexcept {
  return cell < kCellNames.size() ? kCellNames[cell] : "none";
}

core::Result<PairRowCheck> check_pair_rows(
    const camera::ArrayCalibration& solved, std::string_view left,
    std::string_view right, const std::vector<Observation>& views,
    const PairRowCheckOptions& options) {
  return compute::guarded(kWho, [&]() -> core::Result<PairRowCheck> {
    return check(solved, left, right, views, options);
  });
}

}  // namespace volumetric_kit::calib::quality
