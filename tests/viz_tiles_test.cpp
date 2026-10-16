// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The tiles' layout, and a decoder's picture as gfx's update of a tile; no
// device needed (viz_render_test draws them).

#include "volumetric_kit/calib/viz/tiles.hpp"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/image.hpp"

namespace volumetric_kit::calib::viz {
namespace {

using Code = core::Status::Code;
namespace pipelines = gfx::pipelines;
namespace sensor = recon::sensor;

void expect_rect(const TileRect& rect, float x, float y, float w, float h) {
  EXPECT_EQ(rect.origin.x, x);
  EXPECT_EQ(rect.origin.y, y);
  EXPECT_EQ(rect.size.x, w);
  EXPECT_EQ(rect.size.y, h);
}

TEST(TileGrid, IsAsSquareAsItCanBeAndNeverTallerThanWide) {
  struct Case {
    std::size_t count;
    int columns;
    int rows;
  };
  for (const Case& c :
       {Case{0, 0, 0}, Case{1, 1, 1}, Case{2, 2, 1}, Case{3, 2, 2},
        Case{4, 2, 2}, Case{5, 3, 2}, Case{6, 3, 2}, Case{7, 3, 3},
        Case{9, 3, 3}, Case{10, 4, 3}}) {
    const TileGrid grid = tile_grid(c.count);
    EXPECT_EQ(grid.columns, c.columns) << c.count;
    EXPECT_EQ(grid.rows, c.rows) << c.count;
  }
}

// Up to INT_MAX tiles, TileGrid's ints hold the grid; past it there is none,
// and no tile has a place.
TEST(TileGrid, LaysOutAtMostIntMaxTiles) {
  constexpr auto kMax =
      static_cast<std::size_t>(std::numeric_limits<int>::max());
  const TileGrid most = tile_grid(kMax);
  EXPECT_EQ(most.columns, 46341);  // 46340^2 < INT_MAX <= 46341^2
  EXPECT_EQ(most.rows, 46341);
  for (const std::size_t count : {kMax + 1, std::size_t{1} << 62,
                                  std::numeric_limits<std::size_t>::max()}) {
    const TileGrid none = tile_grid(count);
    ASSERT_EQ(none.columns, 0) << count;  // stop before a count that hangs
    EXPECT_EQ(none.rows, 0) << count;
    expect_rect(tile_rect({{5.0f, 6.0f}, {200.0f, 100.0f}}, count, 0), 5, 6, 0,
                0);
  }
}

// The rig's four cameras fill a 2 x 2 grid, row-major, the gap between
// neighbours and none at the area's edges.
TEST(TileRect, LaysTheRigOutTwoByTwo) {
  const TileRect area{{10.0f, 20.0f}, {1284.0f, 724.0f}};
  expect_rect(tile_rect(area, 4, 0, 4.0f), 10, 20, 640, 360);
  expect_rect(tile_rect(area, 4, 1, 4.0f), 654, 20, 640, 360);
  expect_rect(tile_rect(area, 4, 2, 4.0f), 10, 384, 640, 360);
  expect_rect(tile_rect(area, 4, 3, 4.0f), 654, 384, 640, 360);
}

// An area that does not divide evenly keeps whole-pixel edges, the gap
// between neighbours exact, and the last tile ending at the area's edge.
TEST(TileRect, RoundsEdgesToWholePixels) {
  const TileRect area{{0.0f, 0.0f}, {101.0f, 50.0f}};
  const TileRect left = tile_rect(area, 2, 0, 2.0f);
  const TileRect right = tile_rect(area, 2, 1, 2.0f);
  EXPECT_EQ(left.origin.x, 0.0f);
  EXPECT_EQ(left.origin.x + left.size.x + 2.0f, right.origin.x);
  EXPECT_EQ(right.origin.x + right.size.x, 101.0f);
  EXPECT_NEAR(left.size.x, right.size.x, 1.0f);
  EXPECT_EQ(left.size.y, 50.0f);
}

// Three tiles take a 2 x 2 grid and leave its last cell empty.
TEST(TileRect, FillsRowsFromTheTopLeft) {
  const TileRect area{{0.0f, 0.0f}, {200.0f, 100.0f}};
  expect_rect(tile_rect(area, 3, 2), 0, 50, 100, 50);
  expect_rect(tile_rect(area, 1, 0), 0, 0, 200, 100);
}

TEST(TileRect, IsEmptyPastTheLastTileOrForAnEmptyArea) {
  const TileRect area{{5.0f, 6.0f}, {200.0f, 100.0f}};
  expect_rect(tile_rect(area, 4, 4), 5, 6, 0, 0);
  expect_rect(tile_rect(area, 0, 0), 5, 6, 0, 0);
  expect_rect(tile_rect({{5.0f, 6.0f}, {0.0f, 100.0f}}, 4, 0), 5, 6, 0, 0);
  expect_rect(tile_rect({{5.0f, 6.0f}, {-10.0f, 100.0f}}, 4, 0), 5, 6, 0, 0);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const TileRect not_finite = tile_rect({{nan, 6.0f}, {200.0f, 100.0f}}, 4, 0);
  EXPECT_EQ(not_finite.size.x, 0.0f);
  EXPECT_EQ(not_finite.size.y, 0.0f);
}

// A gap wider than the area leaves the tiles empty; a bad gap counts as 0.
TEST(TileRect, TakesNoBadGap) {
  const TileRect area{{0.0f, 0.0f}, {200.0f, 100.0f}};
  expect_rect(tile_rect(area, 2, 1, -5.0f), 100, 0, 100, 100);
  expect_rect(tile_rect(area, 2, 1, std::numeric_limits<float>::infinity()),
              100, 0, 100, 100);
  EXPECT_EQ(tile_rect(area, 2, 0, 500.0f).size.x, 0.0f);
}

// A VideoToolbox MJPEG picture: NV12 in two images, BT.601 full range,
// centred chroma.
sensor::YuvImage mjpeg_picture() {
  sensor::YuvImage picture;
  picture.layout = sensor::YuvLayout::Nv12;
  picture.image[0] = std::make_shared<core::Image>();
  picture.image[1] = std::make_shared<core::Image>();
  picture.width = 3840;
  picture.height = 2160;
  picture.kr = 0.299f;
  picture.kb = 0.114f;
  picture.full_range = true;
  picture.chroma_location = sensor::ChromaLocation::Center;
  return picture;
}

TEST(TileUpdate, ReadsTheDecodersPlanesInPlace) {
  const sensor::YuvImage picture = mjpeg_picture();
  const core::Result<pipelines::ImageUpdate> update = tile_update(picture);
  ASSERT_TRUE(update.ok()) << update.status().message();
  EXPECT_EQ(update.value().planes[0].image, picture.image[0].get());
  EXPECT_EQ(update.value().planes[1].image, picture.image[1].get());
  EXPECT_EQ(update.value().planes[0].buffer, nullptr);
  EXPECT_EQ(update.value().planes[1].buffer, nullptr);
  EXPECT_EQ(update.value().kr, 0.299f);
  EXPECT_EQ(update.value().kb, 0.114f);
  EXPECT_TRUE(update.value().full_range);
  EXPECT_EQ(update.value().chroma_siting, pipelines::ChromaSiting::Center);
}

// An HEVC picture: BT.709, limited range, left-sited chroma.
TEST(TileUpdate, KeepsThePicturesMatrixRangeAndSiting) {
  sensor::YuvImage picture = mjpeg_picture();
  const sensor::YcbcrWeights bt709 =
      sensor::ycbcr_weights(sensor::VideoColorMatrix::Bt709);
  picture.kr = bt709.kr;
  picture.kb = bt709.kb;
  picture.full_range = false;
  picture.chroma_location = sensor::ChromaLocation::Left;
  const core::Result<pipelines::ImageUpdate> update = tile_update(picture);
  ASSERT_TRUE(update.ok()) << update.status().message();
  EXPECT_EQ(update.value().kr, bt709.kr);
  EXPECT_EQ(update.value().kb, bt709.kb);
  EXPECT_FALSE(update.value().full_range);
  EXPECT_EQ(update.value().chroma_siting, pipelines::ChromaSiting::Left);
}

TEST(TileUpdate, RefusesWhatATileCannotDraw) {
  sensor::YuvImage i420 = mjpeg_picture();
  i420.layout = sensor::YuvLayout::I420;
  EXPECT_EQ(tile_update(i420).status().domain(), Code::Unsupported);

  sensor::YuvImage buffer = mjpeg_picture();
  buffer.image[0].reset();
  buffer.image[1].reset();
  buffer.device = std::make_shared<core::Buffer>();
  EXPECT_EQ(tile_update(buffer).status().domain(), Code::Unsupported);

  for (const sensor::ChromaLocation location :
       {sensor::ChromaLocation::TopLeft, sensor::ChromaLocation::Top,
        sensor::ChromaLocation::BottomLeft, sensor::ChromaLocation::Bottom}) {
    sensor::YuvImage sited = mjpeg_picture();
    sited.chroma_location = location;
    EXPECT_EQ(tile_update(sited).status().domain(), Code::Unsupported)
        << static_cast<int>(location);
  }

  sensor::YuvImage no_chroma = mjpeg_picture();
  no_chroma.image[1].reset();
  EXPECT_EQ(tile_update(no_chroma).status().domain(), Code::InvalidArgument);
  sensor::YuvImage none;
  none.layout = sensor::YuvLayout::Nv12;
  EXPECT_EQ(tile_update(none).status().domain(), Code::InvalidArgument);
  sensor::YuvImage empty = mjpeg_picture();
  empty.height = 0;
  EXPECT_EQ(tile_update(empty).status().domain(), Code::InvalidArgument);
}

}  // namespace
}  // namespace volumetric_kit::calib::viz
