// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/viz/overlay.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/gfx/ui/image_overlay.hpp"

namespace volumetric_kit::calib::viz {
namespace {

// The heat map's alpha for one corner, and from `coverage_full` on.
constexpr float kFirstAlpha = 48.0f;
constexpr float kFullAlpha = 160.0f;

glm::vec2 to_mark(const recon::camera::Vec2d& pixel) noexcept {
  return {static_cast<float>(pixel.x), static_cast<float>(pixel.y)};
}

}  // namespace

ImU32 coverage_fill(int corners, const OverlayStyle& style) noexcept {
  if (corners <= 0) return 0;
  const float t =
      style.coverage_full > 1
          ? std::min(1.0f, static_cast<float>(corners - 1) /
                               static_cast<float>(style.coverage_full - 1))
          : 1.0f;
  const auto alpha = static_cast<ImU32>(
      std::lround(kFirstAlpha + (kFullAlpha - kFirstAlpha) * t));
  return (style.coverage_color & ~IM_COL32_A_MASK) |
         (alpha << IM_COL32_A_SHIFT);
}

core::Result<TileMarks> tile_marks(const TileOverlay& tile,
                                   const quality::SessionOptions& thresholds,
                                   const OverlayStyle& style) {
  return compute::guarded("tile_marks", [&]() -> core::Result<TileMarks> {
    TileMarks marks;
    const ImU32 tone = tone_color(verdict_tone(tile.check, thresholds));

    if (style.coverage && tile.session != nullptr) {
      marks.grid_columns = quality::kCoverageCols;
      marks.grid_rows = quality::kCoverageRows;
      marks.grid_fills.reserve(tile.session->coverage.size());
      for (const int corners : tile.session->coverage) {
        marks.grid_fills.push_back(coverage_fill(corners, style));
      }
      marks.grid_line = style.grid_line;
    }

    if (!tile.id.empty()) {
      marks.title.text =
          std::string(tile.id) + ": " + to_string(tile.check.verdict);
      marks.title.color = tone;
      marks.title.background = style.label_background;
    }

    if (tile.detection != nullptr) {
      if (style.markers) {
        marks.outlines.reserve(tile.detection->markers.size());
        for (const DetectedMarker& marker : tile.detection->markers) {
          MarkOutline outline;
          for (std::size_t k = 0; k < outline.corners.size(); ++k) {
            outline.corners[k] = to_mark(marker.corners[k]);
          }
          outline.color = style.marker_color;
          outline.thickness = style.marker_thickness;
          marks.outlines.push_back(outline);
        }
      }
      const std::vector<DetectedPoint>& points =
          tile.detection->observation.points;
      marks.points.reserve(points.size());
      for (const DetectedPoint& point : points) {
        marks.points.push_back(
            {to_mark(point.pixel), tone, style.corner_radius});
        if (style.corner_ids) {
          MarkLabel id;
          id.at = to_mark(point.pixel);
          id.text = std::to_string(point.id);
          id.color = style.label_color;
          id.background = style.label_background;
          id.offset = style.id_offset;
          marks.labels.push_back(std::move(id));
        }
      }
    }
    return marks;
  });
}

void draw_tile_marks(const gfx::camera::ImageView2D& view,
                     const gfx::ui::OverlayRegion& region,
                     const TileMarks& marks, ImDrawList* draw_list) {
  const gfx::ui::ImageOverlay overlay(view, gfx::ui::PixelCenters::Integer,
                                      region, draw_list);
  if (marks.grid_columns > 0 && marks.grid_rows > 0) {
    overlay.grid(marks.grid_columns, marks.grid_rows, marks.grid_fills.data(),
                 marks.grid_fills.size(), marks.grid_line);
  }
  for (const MarkOutline& outline : marks.outlines) {
    overlay.polyline(outline.corners.data(), outline.corners.size(),
                     outline.color, outline.thickness, true);
  }
  for (const MarkPoint& point : marks.points) {
    overlay.point(point.at, point.color, point.radius);
  }
  for (const MarkLabel& label : marks.labels) {
    overlay.label(label.at, label.text.c_str(), label.color, label.background,
                  label.offset);
  }
  overlay.caption(marks.title.text.c_str(), marks.title.color,
                  marks.title.background);
}

}  // namespace volumetric_kit::calib::viz
