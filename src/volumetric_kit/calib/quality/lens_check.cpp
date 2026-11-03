// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/lens_check.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
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
constexpr const char* kResidualsWho = "check_lens_residuals";
constexpr const char* kRigWho = "check_rig_lens";
constexpr std::size_t kParts = 3;  // the image's parts on each axis

core::Status check_options(const LensCheckOptions& o, const char* who) {
  const auto refuse = [who](const char* name, const char* rule) {
    return core::Status::invalid_argument(std::string(who) + ": options." +
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

using Parts = std::array<int, kParts * kParts>;

// The limits over `out`'s figures: its verdict and reason. `dropped_as` says
// what became of the corners dropped, `dropped_in` how many lie in each part.
void judge(camera::ImageSize size, const std::string& dropped_as,
           const Parts& dropped_in, LensCheck& out) {
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
    std::string where;
    for (std::size_t p = 0; p < dropped_in.size(); ++p) {
      if (dropped_in[p] == 0) continue;
      where += where.empty() ? ", in the image's " : ", ";
      where += std::string(pair_row_cell_name(p)) + " (" +
               std::to_string(dropped_in[p]) + ")";
    }
    add(std::to_string(out.dropped) + " of " + std::to_string(all) +
        " corners " + dropped_as + ", above " + number(100.0 * o.max_dropped) +
        "%" + where);
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

// `out`'s figures over `residuals` of an image of `size`, then its verdict.
void measure(const std::vector<solve::CornerResidual>& residuals,
             camera::ImageSize size, const std::string& dropped_as,
             LensCheck& out) {
  std::vector<const solve::CornerResidual*> kept;
  std::vector<double> lengths;
  double squares = 0.0;
  Parts dropped_in{};
  for (const solve::CornerResidual& r : residuals) {
    if (r.dropped || !std::isfinite(r.residual.x) ||
        !std::isfinite(r.residual.y)) {
      ++out.dropped;
      const std::optional<int> at = compute::grid_cell(
          r.pixel.x, r.pixel.y, static_cast<int>(size.width),
          static_cast<int>(size.height), kCoverageCols, kCoverageRows);
      if (at) ++dropped_in[part_of(static_cast<std::size_t>(*at))];
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
  out.cells = detail::lens_cells(kept, size, out.options.max_cell_bias_px,
                                 out.options.min_cell_corners,
                                 detail::CellErrors::StudentT);
  judge(size, dropped_as, dropped_in, out);
}

core::Result<LensCheck> check_views(std::string_view camera_id,
                                    const camera::CameraModel& lens,
                                    const compute::Board& board,
                                    const std::vector<Observation>& views,
                                    const LensCheckOptions& options) {
  VKC_TRY(check_options(options, kWho));
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
  measure(posed->residuals, lens.size,
          "are past the bound (" + number(out.bound_px) +
              " px) or in a view with no pose",
          out);
  return out;
}

core::Result<LensCheck> check_residuals(
    const char* who, std::string_view camera_id, camera::ImageSize size,
    const std::vector<solve::CornerResidual>& residuals,
    const LensCheckOptions& options) {
  VKC_TRY(check_options(options, who));
  if (camera_id.empty()) {
    return core::Status::invalid_argument(std::string(who) + ": no id");
  }
  if (size.width == 0 || size.height == 0) {
    return core::Status::invalid_argument(
        std::string(who) + ": the size " + std::to_string(size.width) + " x " +
        std::to_string(size.height) + " has a side 0");
  }
  std::set<std::int64_t> captures;
  for (const solve::CornerResidual& r : residuals) {
    const camera::Vec2d& p = r.pixel;
    // In the image, a NaN not: the grid's own test, on one cell.
    if (!compute::grid_cell(p.x, p.y, static_cast<int>(size.width),
                            static_cast<int>(size.height), 1, 1)) {
      return core::Status::invalid_argument(
          std::string(who) + ": a corner of capture " +
          std::to_string(r.capture) + " at (" + number(p.x) + ", " +
          number(p.y) + ") is not finite or outside the " +
          std::to_string(size.width) + " x " + std::to_string(size.height) +
          " image");
    }
    captures.insert(r.capture);
  }

  LensCheck out;
  out.options = options;
  out.camera = std::string(camera_id);
  out.views = captures.size();
  measure(residuals, size, "were dropped by the solve or behind the camera",
          out);
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

core::Result<LensCheck> check_lens_residuals(
    std::string_view camera_id, camera::ImageSize size,
    const std::vector<solve::CornerResidual>& residuals,
    const LensCheckOptions& options) {
  return compute::guarded(kResidualsWho, [&]() -> core::Result<LensCheck> {
    return check_residuals(kResidualsWho, camera_id, size, residuals, options);
  });
}

core::Result<LensCheck> check_rig_lens(
    std::string_view camera_id, const std::vector<solve::RigCamera>& cameras,
    const solve::RigSolution& solution, const LensCheckOptions& options) {
  return compute::guarded(kRigWho, [&]() -> core::Result<LensCheck> {
    const auto refuse = [](const std::string& why) {
      return core::Status::invalid_argument(std::string(kRigWho) + ": " + why);
    };
    const auto at = std::find_if(
        cameras.begin(), cameras.end(),
        [camera_id](const solve::RigCamera& c) { return c.id == camera_id; });
    if (at == cameras.end()) {
      return refuse("the rig has no camera \"" + std::string(camera_id) + "\"");
    }
    if (solution.world_to_camera.size() != cameras.size()) {
      return refuse("the solution has " +
                    std::to_string(solution.world_to_camera.size()) +
                    " cameras, the rig " + std::to_string(cameras.size()));
    }
    const auto index = static_cast<std::size_t>(at - cameras.begin());
    std::vector<solve::CornerResidual> mine;
    for (const solve::CornerResidual& r : solution.residuals) {
      if (r.camera >= cameras.size()) {
        return refuse("a residual of camera " + std::to_string(r.camera) +
                      ", which the rig does not have");
      }
      if (r.camera == index) mine.push_back(r);
    }
    return check_residuals(kRigWho, camera_id, at->model.size, mine, options);
  });
}

core::Result<LensCheck> check_lens(std::string_view camera_id,
                                   const camera::CameraModel& lens,
                                   const compute::Board& board,
                                   const std::vector<Observation>& views,
                                   const LensCheckOptions& options) {
  return compute::guarded(kWho, [&]() -> core::Result<LensCheck> {
    return check_views(camera_id, lens, board, views, options);
  });
}

}  // namespace volumetric_kit::calib::quality
