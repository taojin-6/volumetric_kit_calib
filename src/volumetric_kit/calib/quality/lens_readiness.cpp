// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/lens_readiness.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <string>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"

namespace volumetric_kit::calib::quality {
namespace {

core::Status check_options(const LensReadyOptions& o) {
  const auto refuse = [](const char* name, const char* rule) {
    return core::Status::invalid_argument(
        std::string("lens_readiness: options.") + name + " must be " + rule);
  };
  if (o.min_views < 1) return refuse("min_views", "at least 1");
  if (!(o.min_cells >= 0.0 && o.min_cells <= 1.0)) {
    return refuse("min_cells", "in [0, 1]");
  }
  if (!(o.min_reach >= 0.0 && o.min_reach <= 1.0)) {
    return refuse("min_reach", "in [0, 1]");
  }
  if (!(o.max_frontal >= 0.0 && o.max_frontal <= 1.0)) {
    return refuse("max_frontal", "in [0, 1]");
  }
  return {};
}

// What `state` lacks of `options`, in words: the first of its views, cells,
// reach and frontal share that falls short; empty for none.
std::string shortfall(const LensReadiness& state,
                      const LensReadyOptions& options) {
  const auto percent = [](double share) {
    return std::to_string(static_cast<int>(std::lround(100.0 * share))) + "%";
  };
  if (state.views < options.min_views) {
    return std::to_string(state.views) + " views of " +
           std::to_string(options.min_views);
  }
  if (state.cells < options.min_cells) {
    return percent(state.cells) + " of the cells, of " +
           percent(options.min_cells);
  }
  if (*std::min_element(state.reach.begin(), state.reach.end()) <
      options.min_reach) {
    return "its corners reach too little of the way to an image corner or "
           "side";
  }
  if (state.frontal && *state.frontal > options.max_frontal) {
    return percent(*state.frontal) + " of its views frontal, over " +
           percent(options.max_frontal);
  }
  return {};
}

}  // namespace

core::Result<LensReadiness> lens_readiness(
    std::string_view camera_id, sensor::camera::ImageSize size,
    const compute::Board& board, const std::vector<Observation>& views,
    const std::vector<std::int64_t>* frontal, const LensReadyOptions& options) {
  return compute::guarded(
      "lens_readiness", [&]() -> core::Result<LensReadiness> {
        VKC_TRY(check_options(options));
        core::Result<std::vector<solve::LensView>> plan =
            solve::plan_lens_views(camera_id, size, board, views, options.lens);
        if (!plan.ok()) return plan.status().with_context("lens_readiness");
        std::map<std::int64_t, const Observation*> by_capture;
        for (const Observation& view : views) {
          if (view.camera_id == camera_id) {
            by_capture.emplace(view.frame_id, &view);
          }
        }

        LensReadiness state;
        int frontal_views = 0;
        std::array<bool, kCoverageCols * kCoverageRows> held{};
        for (const solve::LensView& use : *plan) {
          if (use.use != solve::LensView::Use::Fitted) continue;
          ++state.views;
          if (frontal != nullptr && std::find(frontal->begin(), frontal->end(),
                                              use.frame_id) != frontal->end()) {
            ++frontal_views;
          }
          for (const DetectedPoint& point : by_capture[use.frame_id]->points) {
            const std::optional<int> cell = compute::grid_cell(
                point.pixel.x, point.pixel.y, static_cast<int>(size.width),
                static_cast<int>(size.height), kCoverageCols, kCoverageRows);
            if (cell) held[static_cast<std::size_t>(*cell)] = true;
            const solve::LensReach here = solve::pixel_reach(size, point.pixel);
            for (std::size_t d = 0; d < here.size(); ++d) {
              state.reach[d] = std::max(state.reach[d], here[d]);
            }
          }
        }
        state.cells =
            static_cast<double>(std::count(held.begin(), held.end(), true)) /
            static_cast<double>(held.size());
        if (frontal != nullptr && state.views > 0) {
          state.frontal = static_cast<double>(frontal_views) / state.views;
        }
        state.unmet = shortfall(state, options);
        state.ready = state.unmet.empty();
        return state;
      });
}

}  // namespace volumetric_kit::calib::quality
