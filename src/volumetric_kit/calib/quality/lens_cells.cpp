// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "lens_cells.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>

#include "checks.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"

namespace volumetric_kit::calib::quality::detail {
namespace {

namespace camera = sensor::camera;

// One cell's residuals.
struct Sums {
  int n = 0;
  camera::Vec2d sum{0.0};
  double squares = 0.0;  // of both components
};

}  // namespace

std::array<LensCell, kLensCells> lens_cells(
    const std::vector<const solve::CornerResidual*>& corners,
    camera::ImageSize size, double max_bias_px, int min_corners,
    CellErrors errors) {
  std::array<Sums, kLensCells> sums;
  for (const solve::CornerResidual* r : corners) {
    if (!std::isfinite(r->residual.x) || !std::isfinite(r->residual.y))
      continue;
    const std::optional<int> at = compute::grid_cell(
        r->pixel.x, r->pixel.y, static_cast<int>(size.width),
        static_cast<int>(size.height), kCoverageCols, kCoverageRows);
    if (!at) continue;
    Sums& cell = sums[static_cast<std::size_t>(*at)];
    ++cell.n;
    cell.sum += r->residual;
    cell.squares += glm::dot(r->residual, r->residual);
  }
  std::array<LensCell, kLensCells> cells;
  for (std::size_t c = 0; c < cells.size(); ++c) {
    const Sums& cell = sums[c];
    cells[c].corners = cell.n;
    if (cell.n < min_corners) continue;
    const double n = cell.n;
    const camera::Vec2d mean = cell.sum / n;
    // The standard error of the mean, as a vector: the sample variance about
    // it, of both components, over n.
    const double variance =
        std::max(0.0, (cell.squares - n * glm::dot(mean, mean)) / (n - 1.0));
    const double t = errors == CellErrors::Three
                         ? 3.0
                         : student_t(static_cast<std::size_t>(cell.n));
    cells[c].mean_px = mean;
    cells[c].limit_px = std::max(max_bias_px, t * std::sqrt(variance / n));
  }
  return cells;
}

}  // namespace volumetric_kit::calib::quality::detail
