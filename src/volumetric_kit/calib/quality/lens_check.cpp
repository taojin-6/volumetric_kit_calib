// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/lens_check.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

#include "checks.hpp"
#include "lens_cells.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/quality/pair_row_check.hpp"
#include "volumetric_kit/calib/solve/lens_poses.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = sensor::camera;
using detail::number;
using Verdict = LensCheck::Verdict;

constexpr const char* kWho = "check_lens";
constexpr std::size_t kParts = 3;  // the image's parts on each axis

core::Status check_options(const LensCheckOptions& o) {
  const auto refuse = [](const char* name, const char* rule) {
    return core::Status::invalid_argument(std::string(kWho) + ": options." +
                                          name + " must be " + rule);
  };
  if (!(o.max_rms_px > 0.0) || !std::isfinite(o.max_rms_px)) {
    return refuse("max_rms_px", "positive and finite");
  }
  if (!(o.max_dropped >= 0.0 && o.max_dropped <= 1.0)) {
    return refuse("max_dropped", "in [0, 1]");
  }
  if (!(o.max_cell_bias_px > 0.0) || !std::isfinite(o.max_cell_bias_px)) {
    return refuse("max_cell_bias_px", "positive and finite");
  }
  if (o.min_cell_corners < 2) return refuse("min_cell_corners", "at least 2");
  if (o.min_corners < 4) return refuse("min_corners", "at least 4");
  if (o.outlier_rounds < 0) return refuse("outlier_rounds", "at least 0");
  return {};
}

// The part of the image's 3 x 3 split that a cell of the 8 x 6 grid lies in,
// by its centre, row-major from the top left: 3, 2 and 3 columns across, 2
// rows each down.
std::size_t part_of(std::size_t cell) {
  constexpr auto kCols = static_cast<std::size_t>(kCoverageCols);
  constexpr auto kRows = static_cast<std::size_t>(kCoverageRows);
  const std::size_t col = cell % kCols;
  const std::size_t row = cell / kCols;
  return (2 * row + 1) * kParts / (2 * kRows) * kParts +
         (2 * col + 1) * kParts / (2 * kCols);
}

// The limits over `out`'s figures: its verdict and reason.
void judge(camera::ImageSize size, LensCheck& out) {
  const LensCheckOptions& o = out.options;
  std::string passed;
  const auto add = [&passed](const std::string& why) {
    passed += passed.empty() ? "" : "; ";
    passed += why;
  };
  const double most = detail::at_width(o.max_rms_px, size);
  if (out.corners > 0 && !(out.rms_px <= most)) {
    add("the RMS through the lens is " + number(out.rms_px) + " px, above " +
        number(most) + " px");
  }
  const std::size_t all = out.corners + out.dropped;
  if (static_cast<double>(out.dropped) >
      o.max_dropped * static_cast<double>(all)) {
    add(std::to_string(out.dropped) + " of " + std::to_string(all) +
        " corners are past the bound (" + number(out.bound_px) +
        " px) or in a view with no pose, above " +
        number(100.0 * o.max_dropped) + "%");
  }

  std::array<bool, kParts * kParts> judged{};
  std::array<int, kParts * kParts> fullest{};
  std::string cells;
  for (std::size_t c = 0; c < out.cells.size(); ++c) {
    const LensCell& cell = out.cells[c];
    const std::size_t part = part_of(c);
    fullest[part] = std::max(fullest[part], cell.corners);
    if (cell.corners < o.min_cell_corners) continue;
    judged[part] = true;
    const double bias = glm::length(cell.mean_px);
    if (bias <= cell.limit_px) continue;
    cells += cells.empty() ? "" : "; ";
    cells += number(bias) + " px in the image's " + pair_row_cell_name(part) +
             " (cell " + std::to_string(c % kCoverageCols) + ", " +
             std::to_string(c / kCoverageCols) + "), above " +
             number(cell.limit_px) + " px";
  }
  if (!cells.empty()) add("the mean residual is " + cells);
  if (!passed.empty()) {
    out.verdict = Verdict::Suspect;
    out.reason = std::move(passed);
    return;
  }

  std::string short_parts;
  for (std::size_t p = 0; p < judged.size(); ++p) {
    if (judged[p]) continue;
    short_parts += short_parts.empty() ? "" : ", ";
    short_parts += std::string(pair_row_cell_name(p)) + " (" +
                   std::to_string(fullest[p]) + ")";
  }
  if (!short_parts.empty()) {
    out.verdict = Verdict::Insufficient;
    out.reason = "too few corners in the image's " + short_parts +
                 ", the most in one of its cells: each of its 9 parts needs " +
                 std::to_string(o.min_cell_corners) +
                 " in a cell of the 8 x 6 grid; capture the board there";
    return;
  }
  out.verdict = Verdict::Good;
}

core::Result<LensCheck> check(std::string_view camera_id,
                              const camera::CameraModel& lens,
                              const compute::Board& board,
                              const std::vector<Observation>& views,
                              const LensCheckOptions& options) {
  VKC_TRY(check_options(options));
  solve::LensOptions posing;
  posing.min_corners = options.min_corners;
  posing.outlier_rounds = options.outlier_rounds;
  const core::Result<solve::PosedLensViews> posed =
      solve::pose_lens_views(camera_id, lens, board, views, posing);
  if (!posed.ok()) return posed.status().with_context(kWho);

  LensCheck out;
  out.options = options;
  out.camera = std::string(camera_id);
  out.bound_px = posed->bound_px;
  for (const solve::LensView& view : posed->views) {
    if (view.use == solve::LensView::Use::HeldOut) ++out.views;
  }
  std::vector<const solve::CornerResidual*> kept;
  std::vector<double> lengths;
  double squares = 0.0;
  for (const solve::CornerResidual& r : posed->residuals) {
    if (r.dropped || !std::isfinite(r.residual.x) ||
        !std::isfinite(r.residual.y)) {
      ++out.dropped;
      continue;
    }
    kept.push_back(&r);
    const double square = glm::dot(r.residual, r.residual);
    squares += square;
    lengths.push_back(std::sqrt(square));
  }
  out.corners = kept.size();
  if (!kept.empty()) {
    out.rms_px = std::sqrt(squares / static_cast<double>(kept.size()));
    out.p95_px = compute::quantile(std::move(lengths), 0.95);
  }
  out.cells = detail::lens_cells(kept, lens.size, options.max_cell_bias_px,
                                 options.min_cell_corners);
  judge(lens.size, out);
  return out;
}

}  // namespace

const char* to_string(LensCheck::Verdict verdict) noexcept {
  switch (verdict) {
    case Verdict::Good:
      return "good";
    case Verdict::Suspect:
      return "suspect";
    case Verdict::Insufficient:
      return "insufficient";
  }
  return "unknown";
}

core::Result<LensCheck> check_lens(std::string_view camera_id,
                                   const camera::CameraModel& lens,
                                   const compute::Board& board,
                                   const std::vector<Observation>& views,
                                   const LensCheckOptions& options) {
  return compute::guarded(kWho, [&]() -> core::Result<LensCheck> {
    return check(camera_id, lens, board, views, options);
  });
}

}  // namespace volumetric_kit::calib::quality
