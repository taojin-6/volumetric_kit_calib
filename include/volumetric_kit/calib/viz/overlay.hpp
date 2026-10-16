// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file overlay.hpp
/// @brief What the live view marks on a camera's tile: the corners and
///        markers found, each corner's id, the camera and its view's verdict,
///        and the coverage of its kept views as a heat map.
///
/// @ref tile_marks decides the marks, from host data alone; @ref
/// draw_tile_marks draws them through gfx's `ImageOverlay`, which maps
/// OpenCV's pixels (centres on integers) onto the tile however its view pans
/// and zooms, and onto a high-DPI display.

#include <array>
#include <string>
#include <string_view>
#include <vector>

#include <glm/vec2.hpp>

#include "imgui.h"
#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/core/view_check.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"
#include "volumetric_kit/calib/viz/export.hpp"
#include "volumetric_kit/calib/viz/tone.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/gfx/camera/image_view_2d.hpp"

namespace volumetric_kit::calib::viz {

/// @brief How the marks look. Sizes are in display pixels, so marks keep
///        their size as a tile zooms.
///
/// @code
/// OverlayStyle style;
/// style.corner_ids = false;  // corners alone, for a crowded view
/// @endcode
struct OverlayStyle {
  float corner_radius = 3.0f;  ///< A corner's dot.
  bool corner_ids = true;      ///< Label each corner with its id.
  /// From a corner to its id's top-left.
  glm::vec2 id_offset{4.0f, -16.0f};
  ImU32 label_color = IM_COL32_WHITE;               ///< Labels' text.
  ImU32 label_background = IM_COL32(0, 0, 0, 160);  ///< Behind labels.
  bool markers = true;                              ///< Outline each marker.
  ImU32 marker_color = IM_COL32(255, 220, 0, 255);  ///< The outlines.
  float marker_thickness = 1.5f;                    ///< Their width.
  bool coverage = true;  ///< Fill the coverage grid's cells.
  /// The heat map's colour; its alpha is set by each cell's corners.
  ImU32 coverage_color = IM_COL32(0, 160, 255, 0);
  /// The fewest corners in a cell that give it the strongest fill.
  int coverage_full = 20;
  ImU32 grid_line = IM_COL32(255, 255, 255, 48);  ///< The grid's borders.
};

/// @brief A dot centred on a pixel.
struct MarkPoint {
  glm::vec2 at{0.0f, 0.0f};  ///< The pixel (OpenCV's).
  ImU32 color = 0;           ///< Its colour.
  float radius = 0.0f;       ///< Its radius (display pixels).
};

/// @brief A closed outline through four pixels: a marker's.
struct MarkOutline {
  std::array<glm::vec2, 4> corners{};  ///< The pixels, in order.
  ImU32 color = 0;                     ///< Its colour.
  float thickness = 0.0f;              ///< Its width (display pixels).
};

/// @brief Text by a pixel.
struct MarkLabel {
  glm::vec2 at{0.0f, 0.0f};      ///< The anchor pixel.
  std::string text;              ///< The text.
  ImU32 color = 0;               ///< Its colour.
  ImU32 background = 0;          ///< Behind it; 0 for none.
  glm::vec2 offset{0.0f, 0.0f};  ///< From the anchor to its top-left.
};

/// @brief The marks over one tile, in OpenCV's pixels, drawn in this order:
///        the grid, the outlines, the dots, then the labels.
///
/// @code
/// VKC_ASSIGN(const TileMarks marks, tile_marks(tile));
/// draw_tile_marks(tiles->view(i), marks);
/// @endcode
struct TileMarks {
  int grid_columns = 0;  ///< The grid's cells across; 0 for no grid.
  int grid_rows = 0;     ///< Its cells down.
  /// Each cell's fill, row-major from the top left; 0 leaves it clear.
  std::vector<ImU32> grid_fills;
  ImU32 grid_line = 0;                ///< Its borders' colour.
  std::vector<MarkOutline> outlines;  ///< The markers'.
  std::vector<MarkPoint> points;      ///< The corners'.
  /// The camera's title at the image's top-left corner, then the corners'
  /// ids.
  std::vector<MarkLabel> labels;
};

/// @brief What one tile shows, from the live view's host data.
///
/// @code
/// TileOverlay tile;
/// tile.id = serials[i];
/// tile.check = views[i].check;      // quality::check_view's
/// tile.detection = &detections[i];  // live (hybrid)
/// tile.session = &summary.cameras[i];
/// @endcode
struct TileOverlay {
  std::string_view id;  ///< The camera's id; empty for no title.
  /// Its view's live check: the title's verdict and the corners' tone.
  ViewCheck check;
  /// What the live detector found; null for nothing.
  const Detection* detection = nullptr;
  /// The camera's part of the session, whose coverage the grid shows; null
  /// for no grid.
  const quality::CameraSummary* session = nullptr;
};

/// @brief A coverage cell's fill: the heat map's colour, more opaque with
///        more corners.
/// @param corners  The cell's corners (@ref quality::CameraSummary::coverage).
/// @param style    The heat map's colour and `coverage_full`.
/// @return Clear (0) for no corner; else the colour at an alpha from 48,
///         for one corner, up to 160 at `coverage_full` corners and above.
///
/// @code
/// const ImU32 fill = coverage_fill(summary.coverage[cell], style);
/// @endcode
VC_VIZ_API ImU32 coverage_fill(int corners, const OverlayStyle& style) noexcept;

/// @brief The marks over one tile.
///
/// - **Grid.** The @ref quality::kCoverageCols x @ref quality::kCoverageRows
///   coverage grid, each cell filled by @ref coverage_fill, when
///   `style.coverage` and the tile has a session.
/// - **Outlines.** Each marker found, when `style.markers`.
/// - **Dots.** Each corner found, in its view's @ref verdict_tone: green for a
///   usable view, amber or red for a flagged one, grey for one with no pose.
/// - **Labels.** The title, `"<id>: <verdict>"` in the same tone, at the
///   image's top-left corner; then each corner's id, when `style.corner_ids`.
/// @param tile        What the tile shows.
/// @param thresholds  The session's thresholds (@ref verdict_tone).
/// @param style       How the marks look.
/// @return The marks; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const TileMarks marks, tile_marks(tile, session_options));
/// @endcode
VC_VIZ_API core::Result<TileMarks> tile_marks(
    const TileOverlay& tile, const quality::SessionOptions& thresholds = {},
    const OverlayStyle& style = {});

/// @brief Draw @p marks over the tile @p view shows, in @ref TileMarks'
///        order, through gfx's `ImageOverlay` with OpenCV's pixel centres
///        (`gfx::ui::PixelCenters::Integer`).
/// @param view       The tile's view (`Tiles::view`).
/// @param marks      The marks.
/// @param draw_list  The ImGui list to draw into; null for the background
///                   list, behind every ImGui window.
/// @pre An ImGui frame is active, with its context current.
///
/// @code
/// draw_tile_marks(tiles->view(i), marks);
/// @endcode
VC_VIZ_API void draw_tile_marks(const gfx::camera::ImageView2D& view,
                                const TileMarks& marks,
                                ImDrawList* draw_list = nullptr);

}  // namespace volumetric_kit::calib::viz
