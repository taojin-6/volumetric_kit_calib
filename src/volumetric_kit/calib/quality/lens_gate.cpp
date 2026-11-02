// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/lens_gate.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <optional>
#include <utility>

#include <glm/trigonometric.hpp>

#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"
#include "volumetric_kit/sensor/camera/stereo_rectification.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = sensor::camera;
using Use = solve::LensView::Use;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// `v` in a gate's message: "%.3g".
std::string number(double v) {
  char text[32];
  std::snprintf(text, sizeof text, "%.3g", v);
  return text;
}

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
  return {};
}

// The fitted corners kept.
core::Result<std::vector<const solve::CornerResidual*>> fitted_corners(
    const solve::LensFit& fit) {
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
  }
  return corners;
}

// One grid cell's fitted residuals.
struct Cell {
  int n = 0;
  camera::Vec2d sum{0.0};
  double squares = 0.0;  // of both components
};

// The bias gate: fills `worst_cell_bias_px`, and `failed` for the cell past
// its limit by the most.
void judge_bias(const std::vector<const solve::CornerResidual*>& corners,
                const LensGateOptions& options, LensVerdict& out) {
  const camera::ImageSize& size = out.fit.camera.model.size;
  std::array<Cell, kCoverageCols * kCoverageRows> cells;
  for (const solve::CornerResidual* r : corners) {
    if (!std::isfinite(r->residual.x) || !std::isfinite(r->residual.y))
      continue;
    const std::optional<int> at = compute::grid_cell(
        r->pixel.x, r->pixel.y, static_cast<int>(size.width),
        static_cast<int>(size.height), kCoverageCols, kCoverageRows);
    if (!at) continue;
    Cell& cell = cells[static_cast<std::size_t>(*at)];
    ++cell.n;
    cell.sum += r->residual;
    cell.squares += glm::dot(r->residual, r->residual);
  }
  double worst_over = 1.0;
  std::string failure;
  for (std::size_t c = 0; c < cells.size(); ++c) {
    const Cell& cell = cells[c];
    if (cell.n < options.min_cell_corners) continue;
    const double n = cell.n;
    const camera::Vec2d mean = cell.sum / n;
    const double bias = glm::length(mean);
    // The standard error of the mean, as a vector: the sample variance about
    // it, of both components, over n.
    const double variance =
        std::max(0.0, (cell.squares - n * glm::dot(mean, mean)) / (n - 1.0));
    const double limit =
        std::max(options.max_cell_bias_px, 3.0 * std::sqrt(variance / n));
    out.worst_cell_bias_px = std::max(out.worst_cell_bias_px, bias);
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
  const double numerator = 1.0 + s * (d.k1 + s * (d.k2 + s * d.k3));
  const double denominator = 1.0 + s * (d.k4 + s * (d.k5 + s * d.k6));
  if (!(denominator > 0.0)) return kInfinity;
  return std::sqrt(s) * numerator / denominator;
}

// The fold gate: fills `fold_deg` and `frame_folds`, and `failed` when an
// image corner lies past the fold.
void judge_fold(LensVerdict& out) {
  const camera::CameraModel& model = out.fit.camera.model;
  const float fold = sensor::fold_radius2(model.distortion);
  if (fold >= std::numeric_limits<float>::max()) return;
  const double radius2 = static_cast<double>(fold);
  out.fold_deg = glm::degrees(std::atan(std::sqrt(radius2)));
  const double reached = fold_radius(model.distortion, radius2);
  const camera::PinholeIntrinsics& k = model.intrinsics;
  double farthest = 0.0;
  for (const double u : {-0.5, model.size.width - 0.5}) {
    for (const double v : {-0.5, model.size.height - 0.5}) {
      farthest =
          std::max(farthest, std::hypot((u - k.cx) / k.fx, (v - k.cy) / k.fy));
    }
  }
  out.frame_folds = !(farthest < reached);
  if (!out.frame_folds) return;
  out.failed.push_back("the lens folds " + number(out.fold_deg) +
                       " degrees off axis, inside the frame: an image corner "
                       "lies at a radius of " +
                       number(farthest) + ", past its largest, " +
                       number(reached));
}

// Each direction's farthest fitted corner, off axis through the lens.
core::Status measure_reach(
    const std::vector<const solve::CornerResidual*>& corners,
    const compute::CvCamera& lens, LensVerdict& out) {
  const camera::ImageSize& size = out.fit.camera.model.size;
  solve::LensReach reach{};
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

}  // namespace

core::Result<LensVerdict> judge_lens(solve::LensFit fit,
                                     const LensGateOptions& options) {
  return compute::guarded("judge_lens", [&]() -> core::Result<LensVerdict> {
    VKC_TRY(check_options(options));
    core::Result<compute::CvCamera> lens = compute::cv_camera(fit.camera.model);
    if (!lens.ok()) return lens.status().with_context("judge_lens");

    LensVerdict verdict;
    verdict.fit = std::move(fit);
    const solve::LensFit& judged = verdict.fit;

    const double holdout = judged.holdout_rms_px;
    const double most =
        options.max_holdout_rms_px * judged.camera.model.size.width / 3840.0;
    if (std::isnan(holdout)) {
      verdict.failed.push_back("no view was held out");
    } else if (!(holdout <= most)) {
      verdict.failed.push_back("the held-out RMS is " + number(holdout) +
                               " px, above " + number(most) + " px");
    }
    const double ratio = holdout / judged.rms_px;
    if (!std::isnan(holdout) && !(ratio <= options.max_holdout_ratio)) {
      verdict.failed.push_back("the held-out RMS is " + number(ratio) +
                               " times the fit's, above " +
                               number(options.max_holdout_ratio));
    }

    VKC_ASSIGN(const std::vector<const solve::CornerResidual*> corners,
               fitted_corners(judged));
    judge_bias(corners, options, verdict);
    judge_fold(verdict);
    VKC_TRY(measure_reach(corners, *lens, verdict));
    verdict.pass = verdict.failed.empty();
    return verdict;
  });
}

}  // namespace volumetric_kit::calib::quality
