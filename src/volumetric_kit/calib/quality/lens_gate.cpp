// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/lens_gate.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>

#include <glm/trigonometric.hpp>

#include "checks.hpp"
#include "lens_cells.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"
#include "volumetric_kit/sensor/camera/projection.hpp"
#include "volumetric_kit/sensor/camera/stereo_rectification.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = sensor::camera;
using detail::number;
using Use = solve::LensView::Use;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

core::Status check_options(const LensGateOptions& o) {
  const auto refuse = [](const char* name, const char* rule) {
    return core::Status::invalid_argument(std::string("judge_lens: options.") +
                                          name + " must be " + rule);
  };
  if (!(o.max_holdout_rms_px > 0.0) || !std::isfinite(o.max_holdout_rms_px)) {
    return refuse("max_holdout_rms_px", "positive and finite");
  }
  if (!(o.max_holdout_ratio > 0.0) || !std::isfinite(o.max_holdout_ratio)) {
    return refuse("max_holdout_ratio", "positive and finite");
  }
  if (!(o.max_cell_bias_px > 0.0) || !std::isfinite(o.max_cell_bias_px)) {
    return refuse("max_cell_bias_px", "positive and finite");
  }
  if (o.min_cell_corners < 2) return refuse("min_cell_corners", "at least 2");
  if (!(o.min_reach >= 0.0 && o.min_reach <= 1.0)) {
    return refuse("min_reach", "in [0, 1]");
  }
  if (!(o.min_ratio_holdout_rms_px >= 0.0) ||
      !std::isfinite(o.min_ratio_holdout_rms_px)) {
    return refuse("min_ratio_holdout_rms_px", "at least 0 and finite");
  }
  if (!(o.max_holdout_dropped >= 0.0 && o.max_holdout_dropped <= 1.0)) {
    return refuse("max_holdout_dropped", "in [0, 1]");
  }
  return {};
}

// The fitted corners kept; counts the held-out ones into `out`.
core::Result<std::vector<const solve::CornerResidual*>> fitted_corners(
    LensVerdict& out) {
  const solve::LensFit& fit = out.fit;
  std::vector<const solve::CornerResidual*> corners;
  corners.reserve(fit.residuals.size());
  std::map<std::int64_t, Use> uses;
  for (const solve::LensView& view : fit.views) {
    uses.emplace(view.frame_id, view.use);
  }
  for (const solve::CornerResidual& r : fit.residuals) {
    const auto use = uses.find(r.capture);
    if (use == uses.end()) {
      return core::Status::invalid_argument(
          "judge_lens: a residual of capture " + std::to_string(r.capture) +
          ", which has no view");
    }
    if (use->second == Use::Fitted && !r.dropped) corners.push_back(&r);
    if (use->second == Use::HeldOut) {
      ++out.holdout_corners;
      out.holdout_dropped += r.dropped ? 1 : 0;
    }
  }
  return corners;
}

// The gates on the held-out views: fills `failed` for each missed.
void judge_holdout(const LensGateOptions& options, LensVerdict& out) {
  const solve::LensFit& judged = out.fit;
  const camera::ImageSize& size = judged.camera.model.size;
  const double holdout = judged.holdout_rms_px;
  const double most = detail::at_width(options.max_holdout_rms_px, size);
  if (std::isnan(holdout)) {
    out.failed.push_back("no view was held out");
    return;
  }
  if (!(holdout <= most)) {
    out.failed.push_back("the held-out RMS is " + number(holdout) +
                         " px, above " + number(most) + " px");
  }
  const double ratio = holdout / judged.rms_px;
  if (holdout > detail::at_width(options.min_ratio_holdout_rms_px, size) &&
      !(ratio <= options.max_holdout_ratio)) {
    out.failed.push_back("the held-out RMS is " + number(ratio) +
                         " times the fit's, above " +
                         number(options.max_holdout_ratio));
  }
  if (out.holdout_dropped > options.max_holdout_dropped * out.holdout_corners) {
    out.failed.push_back(std::to_string(out.holdout_dropped) + " of " +
                         std::to_string(out.holdout_corners) +
                         " held-out corners are past the fit's corner bound, "
                         "above " +
                         number(100.0 * options.max_holdout_dropped) + "%");
  }
}

// The bias gate: fills `worst_cell_bias_px`, `worst_cell`, and `failed` for
// the cell past its limit by the most.
void judge_bias(const std::vector<const solve::CornerResidual*>& corners,
                const LensGateOptions& options, LensVerdict& out) {
  const std::array<LensCell, kLensCells> cells = detail::lens_cells(
      corners, out.fit.camera.model.size, options.max_cell_bias_px,
      options.min_cell_corners, detail::CellErrors::Three);
  double worst_over = 1.0;
  std::string failure;
  for (std::size_t c = 0; c < cells.size(); ++c) {
    const LensCell& cell = cells[c];
    if (cell.corners < options.min_cell_corners) continue;
    const double bias = glm::length(cell.mean_px);
    const double limit = cell.limit_px;
    if (bias >= out.worst_cell_bias_px) {
      out.worst_cell_bias_px = bias;
      out.worst_cell = {static_cast<int>(c) % kCoverageCols,
                        static_cast<int>(c) / kCoverageCols};
    }
    if (bias / limit > worst_over) {
      worst_over = bias / limit;
      failure = "the mean residual in cell " +
                std::to_string(c % kCoverageCols) + ", " +
                std::to_string(c / kCoverageCols) + " of the 8 x 6 grid is " +
                number(bias) + " px, above " + number(limit) + " px";
    }
  }
  if (!failure.empty()) out.failed.push_back(std::move(failure));
}

// The largest normalized distorted radius a lens reaches: at its fold,
// `radius2` the undistorted radius squared there. Infinite at the
// denominator's pole.
double fold_radius(const camera::RationalDistortion& d, double radius2) {
  const double s = radius2;
  if (!(1.0 + s * (d.k4 + s * (d.k5 + s * d.k6)) > 0.0)) return kInfinity;
  const double radial_only[] = {d.k1, d.k2, 0.0, 0.0, d.k3, d.k4, d.k5, d.k6};
  double x = 0.0;
  double y = 0.0;
  camera::distort_rational(radial_only, std::sqrt(s), 0.0, &x, &y);
  return x;
}

// The fold gate: fills `fold_deg` and `frame_folds`, and `failed` when an
// image corner lies past the fold.
core::Status judge_fold(const compute::CvCamera& lens, LensVerdict& out) {
  const camera::CameraModel& model = out.fit.camera.model;
  const float fold = sensor::fold_radius2(model.distortion);
  if (fold >= std::numeric_limits<float>::max()) return {};
  const double radius2 = static_cast<double>(fold);
  out.fold_deg = glm::degrees(std::atan(std::sqrt(radius2)));
  const double reached = fold_radius(model.distortion, radius2);
  const camera::PinholeIntrinsics& k = model.intrinsics;
  std::vector<cv::Point2d> frame;
  double farthest = 0.0;
  for (const double u : {-0.5, model.size.width - 0.5}) {
    for (const double v : {-0.5, model.size.height - 0.5}) {
      frame.emplace_back(u, v);
      farthest =
          std::max(farthest, std::hypot((u - k.cx) / k.fx, (v - k.cy) / k.fy));
    }
  }
  if (!(farthest < reached)) {
    out.frame_folds = true;
    out.failed.push_back("the lens folds " + number(out.fold_deg) +
                         " degrees off axis, inside the frame: an image corner "
                         "lies at a radius of " +
                         number(farthest) + ", past its largest, " +
                         number(reached));
    return {};
  }
  // Past a pole the fit's curve can run on: an image corner then reads past
  // the fold, or as no ray at all (NaN, taken as 90 degrees).
  VKC_ASSIGN(const std::vector<cv::Point2d> rays,
             compute::undistort_points(lens, frame));
  double off_axis = 0.0;
  for (const cv::Point2d& ray : rays) {
    const double deg = glm::degrees(std::atan(std::hypot(ray.x, ray.y)));
    off_axis = std::isnan(deg) ? 90.0 : std::max(off_axis, deg);
  }
  if (off_axis >= out.fold_deg) {
    out.frame_folds = true;
    out.failed.push_back("the lens folds " + number(out.fold_deg) +
                         " degrees off axis, inside the frame: an image corner "
                         "reads " +
                         number(off_axis) + " degrees through it");
  }
  return {};
}

// Each direction's farthest fitted corner, as a reach and off axis through the
// lens.
core::Status measure_reach(
    const std::vector<const solve::CornerResidual*>& corners,
    const compute::CvCamera& lens, LensVerdict& out) {
  const camera::ImageSize& size = out.fit.camera.model.size;
  solve::LensReach& reach = out.reach;
  std::vector<cv::Point2d> farthest(reach.size());
  std::vector<bool> found(reach.size(), false);
  for (const solve::CornerResidual* r : corners) {
    const solve::LensReach here = solve::pixel_reach(size, r->pixel);
    for (std::size_t d = 0; d < reach.size(); ++d) {
      if (!(here[d] > reach[d])) continue;
      reach[d] = here[d];
      farthest[d] = {r->pixel.x, r->pixel.y};
      found[d] = true;
    }
  }
  VKC_ASSIGN(const std::vector<cv::Point2d> rays,
             compute::undistort_points(lens, farthest));
  for (std::size_t d = 0; d < reach.size(); ++d) {
    if (!found[d]) continue;
    const double off_axis =
        glm::degrees(std::atan(std::hypot(rays[d].x, rays[d].y)));
    if (d < out.corner_reach_deg.size()) {
      out.corner_reach_deg[d] = off_axis;
    } else {
      out.side_reach_deg[d - out.corner_reach_deg.size()] = off_axis;
    }
  }
  return {};
}

// The reach gate: fills `failed` for each direction short of `min_reach`.
void judge_reach(const LensGateOptions& options, LensVerdict& out) {
  for (std::size_t d = 0; d < out.reach.size(); ++d) {
    if (out.reach[d] >= options.min_reach) continue;
    out.failed.push_back("the fitted corners reach " + number(out.reach[d]) +
                         " of the way toward the " + detail::kReachToward[d] +
                         ", short of " + number(options.min_reach));
  }
}

}  // namespace

core::Result<LensVerdict> judge_lens(solve::LensFit fit,
                                     const LensGateOptions& options) {
  return compute::guarded("judge_lens", [&]() -> core::Result<LensVerdict> {
    VKC_TRY(check_options(options));
    core::Result<compute::CvCamera> lens = compute::cv_camera(fit.camera.model);
    if (!lens.ok()) return lens.status().with_context("judge_lens");

    LensVerdict verdict;
    verdict.fit = std::move(fit);
    VKC_ASSIGN(const std::vector<const solve::CornerResidual*> corners,
               fitted_corners(verdict));
    judge_holdout(options, verdict);
    judge_bias(corners, options, verdict);
    VKC_TRY(judge_fold(*lens, verdict));
    VKC_TRY(measure_reach(corners, *lens, verdict));
    judge_reach(options, verdict);
    verdict.pass = verdict.failed.empty();
    return verdict;
  });
}

}  // namespace volumetric_kit::calib::quality
