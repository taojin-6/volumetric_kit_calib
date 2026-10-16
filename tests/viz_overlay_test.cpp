// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The marks over a tile, decided from host data: the corners and markers
// found, the ids, the title and the coverage heat map, each in its tone; no
// device needed (viz_render_test draws them over a tile of the mosaic).

#include "volumetric_kit/calib/viz/overlay.hpp"

#include <string>

#include <gtest/gtest.h>

namespace volumetric_kit::calib::viz {
namespace {

// A live detection of two corners and one marker.
Detection detection() {
  Detection d;
  d.image_size = {3840, 2160};
  d.observation.points.push_back({0, {812.25, 390.5}, 1.0});
  d.observation.points.push_back({5, {1012.0, 391.75}, 1.0});
  DetectedMarker marker;
  marker.id = 3;
  marker.corners = {recon::camera::Vec2d{700.0, 300.0},
                    {780.0, 300.5},
                    {779.5, 380.0},
                    {699.5, 379.5}};
  d.markers.push_back(marker);
  return d;
}

ViewCheck check_of(ViewVerdict verdict, double tilt_deg = 10.0) {
  ViewCheck check;
  check.verdict = verdict;
  check.tilt_deg = tilt_deg;
  return check;
}

TileMarks marks_of(const TileOverlay& tile, const OverlayStyle& style = {}) {
  const core::Result<TileMarks> marks = tile_marks(tile, {}, style);
  EXPECT_TRUE(marks.ok()) << marks.status().message();
  return marks.ok() ? marks.value() : TileMarks{};
}

TEST(CoverageFill, IsClearWithoutCornersAndDeepensToFull) {
  OverlayStyle style;
  style.coverage_color = IM_COL32(10, 20, 30, 0);
  style.coverage_full = 5;
  EXPECT_EQ(coverage_fill(0, style), 0u);
  EXPECT_EQ(coverage_fill(-3, style), 0u);
  EXPECT_EQ(coverage_fill(1, style), IM_COL32(10, 20, 30, 48));
  EXPECT_EQ(coverage_fill(3, style), IM_COL32(10, 20, 30, 104));
  EXPECT_EQ(coverage_fill(5, style), IM_COL32(10, 20, 30, 160));
  EXPECT_EQ(coverage_fill(500, style), IM_COL32(10, 20, 30, 160));
  style.coverage_full = 1;
  EXPECT_EQ(coverage_fill(1, style), IM_COL32(10, 20, 30, 160));
}

// Each corner is a dot on its pixel, OpenCV's (marks are drawn with integer
// pixel centres), labelled with its id; each marker a closed outline.
TEST(TileMarks, MarksEachCornerAndMarkerOnItsPixels) {
  const Detection found = detection();
  TileOverlay tile;
  tile.check = check_of(ViewVerdict::Good);
  tile.detection = &found;
  OverlayStyle style;
  style.corner_radius = 2.5f;
  const TileMarks marks = marks_of(tile, style);

  ASSERT_EQ(marks.points.size(), 2u);
  EXPECT_EQ(marks.points[0].at, glm::vec2(812.25f, 390.5f));
  EXPECT_EQ(marks.points[1].at, glm::vec2(1012.0f, 391.75f));
  EXPECT_EQ(marks.points[0].radius, 2.5f);

  ASSERT_EQ(marks.labels.size(), 2u);
  EXPECT_TRUE(marks.title.text.empty());  // no id, so no title
  EXPECT_EQ(marks.labels[0].text, "0");
  EXPECT_EQ(marks.labels[1].text, "5");
  EXPECT_EQ(marks.labels[1].at, glm::vec2(1012.0f, 391.75f));
  EXPECT_EQ(marks.labels[1].offset, style.id_offset);
  EXPECT_EQ(marks.labels[1].color, style.label_color);
  EXPECT_EQ(marks.labels[1].background, style.label_background);

  ASSERT_EQ(marks.outlines.size(), 1u);
  EXPECT_EQ(marks.outlines[0].corners[0], glm::vec2(700.0f, 300.0f));
  EXPECT_EQ(marks.outlines[0].corners[2], glm::vec2(779.5f, 380.0f));
  EXPECT_EQ(marks.outlines[0].color, style.marker_color);
  EXPECT_EQ(marks.outlines[0].thickness, style.marker_thickness);
  EXPECT_EQ(marks.grid_columns, 0);  // no session, no grid
}

// The corners take their view's tone: green when usable, amber when
// flagged, red past the solve's tilt or collinear, grey with no pose.
TEST(TileMarks, TonesTheCornersByTheirViewsVerdict) {
  const Detection found = detection();
  const struct {
    ViewCheck check;
    Tone tone;
  } cases[] = {
      {check_of(ViewVerdict::Good), Tone::Good},
      {check_of(ViewVerdict::Partial), Tone::Good},
      {check_of(ViewVerdict::Far), Tone::Warn},
      {check_of(ViewVerdict::Oblique, 65.0), Tone::Warn},
      {check_of(ViewVerdict::Oblique, 71.0), Tone::Bad},
      {check_of(ViewVerdict::Collinear), Tone::Bad},
      {check_of(ViewVerdict::None), Tone::Muted},
  };
  for (const auto& c : cases) {
    TileOverlay tile;
    tile.check = c.check;
    tile.detection = &found;
    const TileMarks marks = marks_of(tile);
    ASSERT_EQ(marks.points.size(), 2u);
    EXPECT_EQ(marks.points[1].color, tone_color(c.tone))
        << to_string(c.check.verdict);
  }
}

// The title names the camera and its verdict, in the corners' tone, over
// the labels' background.
TEST(TileMarks, TitlesTheTileWithItsCameraAndVerdict) {
  TileOverlay tile;
  tile.id = "CL2A141000N";
  tile.check = check_of(ViewVerdict::Far);
  OverlayStyle style;
  style.label_background = IM_COL32(1, 2, 3, 4);
  const TileMarks marks = marks_of(tile, style);
  EXPECT_EQ(marks.title.text, "CL2A141000N: far");
  EXPECT_EQ(marks.title.color, tone_color(Tone::Warn));
  EXPECT_EQ(marks.title.background, IM_COL32(1, 2, 3, 4));
  EXPECT_TRUE(marks.labels.empty());
  EXPECT_TRUE(marks.points.empty());
  EXPECT_TRUE(marks.outlines.empty());
}

// The grid is the session's 8 x 6 coverage, each cell filled by its corners.
TEST(TileMarks, ShowsTheSessionsCoverage) {
  quality::CameraSummary session;
  session.coverage[0] = 1;
  session.coverage[9] = 40;  // row 1, column 1
  TileOverlay tile;
  tile.session = &session;
  OverlayStyle style;
  const TileMarks marks = marks_of(tile, style);
  EXPECT_EQ(marks.grid_columns, quality::kCoverageCols);
  EXPECT_EQ(marks.grid_rows, quality::kCoverageRows);
  ASSERT_EQ(marks.grid_fills.size(), session.coverage.size());
  EXPECT_EQ(marks.grid_fills[0], coverage_fill(1, style));
  EXPECT_EQ(marks.grid_fills[9], coverage_fill(40, style));
  EXPECT_EQ(marks.grid_fills[1], 0u);
  EXPECT_EQ(marks.grid_line, style.grid_line);
}

TEST(TileMarks, LeavesOutWhatTheStyleTurnsOff) {
  const Detection found = detection();
  const quality::CameraSummary session;
  TileOverlay tile;
  tile.id = "A";
  tile.detection = &found;
  tile.session = &session;
  OverlayStyle style;
  style.corner_ids = false;
  style.markers = false;
  style.coverage = false;
  const TileMarks marks = marks_of(tile, style);
  EXPECT_EQ(marks.points.size(), 2u);
  EXPECT_TRUE(marks.labels.empty());
  EXPECT_EQ(marks.title.text, "A: none");
  EXPECT_TRUE(marks.outlines.empty());
  EXPECT_EQ(marks.grid_columns, 0);
  EXPECT_TRUE(marks.grid_fills.empty());
}

}  // namespace
}  // namespace volumetric_kit::calib::viz
