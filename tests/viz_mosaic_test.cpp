// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The mosaic's layout -- any number of cameras, of mixed sizes, paged when one
// texture cannot hold it -- and a decoder's picture as gfx's write of its tile
// and as the mosaic's conversion; no device needed (viz_render_test draws it).

#include "volumetric_kit/calib/viz/mosaic.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/image.hpp"

namespace volumetric_kit::calib::viz {
namespace {

using Code = core::Status::Code;
using recon::camera::ImageSize;
namespace pipelines = gfx::pipelines;
namespace sensor = recon::sensor;

MosaicLayout layout_of(const std::vector<ImageSize>& sizes,
                       std::uint32_t gutter = kMosaicGutter,
                       const MosaicLimits& limits = {}) {
  const core::Result<MosaicLayout> layout =
      mosaic_layout(sizes, gutter, limits);
  EXPECT_TRUE(layout.ok()) << layout.status().message();
  return layout.ok() ? layout.value() : MosaicLayout{};
}

void expect_offset(VkOffset2D offset, std::int32_t x, std::int32_t y) {
  EXPECT_EQ(offset.x, x);
  EXPECT_EQ(offset.y, y);
}

void expect_extent(VkExtent2D extent, std::uint32_t width,
                   std::uint32_t height) {
  EXPECT_EQ(extent.width, width);
  EXPECT_EQ(extent.height, height);
}

// What every layout keeps: each page within the limits and a multiple of 16,
// every tile inside its page at the place the page's origin gives it, and no
// two tiles' cells overlapping.
void expect_well_formed(const MosaicLayout& layout, const MosaicLimits& limits,
                        std::size_t cameras) {
  ASSERT_EQ(layout.tiles.size(), cameras);
  ASSERT_FALSE(layout.pages.empty());
  EXPECT_EQ(layout.extent.width % 16, 0u);
  EXPECT_EQ(layout.extent.height % 16, 0u);
  EXPECT_EQ(layout.pitch.width % 16, 0u);
  EXPECT_EQ(layout.pitch.height % 16, 0u);
  for (const MosaicPage& page : layout.pages) {
    EXPECT_EQ(page.extent.width % 16, 0u);
    EXPECT_EQ(page.extent.height % 16, 0u);
    EXPECT_LE(page.extent.width, limits.max_extent.width);
    EXPECT_LE(page.extent.height, limits.max_extent.height);
    EXPECT_LE(std::uint64_t{page.extent.width} * page.extent.height,
              limits.max_texels);
  }
  for (std::size_t i = 0; i < layout.tiles.size(); ++i) {
    SCOPED_TRACE(i);
    const MosaicTile& tile = layout.tiles[i];
    ASSERT_LT(tile.page, layout.pages.size());
    const MosaicPage& page = layout.pages[tile.page];
    EXPECT_EQ(tile.at.x, tile.origin.x - page.origin.x);
    EXPECT_EQ(tile.at.y, tile.origin.y - page.origin.y);
    EXPECT_GE(tile.at.x, 0);
    EXPECT_GE(tile.at.y, 0);
    EXPECT_LE(tile.at.x + static_cast<std::int64_t>(tile.extent.width),
              static_cast<std::int64_t>(page.extent.width));
    EXPECT_LE(tile.at.y + static_cast<std::int64_t>(tile.extent.height),
              static_cast<std::int64_t>(page.extent.height));
    EXPECT_EQ(tile.origin.x % 16, 0);
    EXPECT_EQ(tile.origin.y % 16, 0);
    // Row-major cells, a pitch apart.
    expect_offset(
        tile.origin,
        static_cast<std::int32_t>(i % layout.columns * layout.pitch.width),
        static_cast<std::int32_t>(i / layout.columns * layout.pitch.height));
  }
}

// The rig's four 4K cameras: 2 x 2 cells with a 16-texel gutter, on one page.
TEST(MosaicLayout, LaysTheRigOutTwoByTwoAt4K) {
  const std::vector<ImageSize> sizes(4, ImageSize{3840, 2160});
  const MosaicLayout layout = layout_of(sizes);
  expect_well_formed(layout, {}, 4);
  EXPECT_EQ(layout.columns, 2u);
  EXPECT_EQ(layout.rows, 2u);
  expect_extent(layout.cell, 3840, 2160);
  expect_extent(layout.pitch, 3856, 2176);
  expect_extent(layout.extent, 7696, 4336);
  ASSERT_EQ(layout.pages.size(), 1u);
  expect_offset(layout.pages[0].origin, 0, 0);
  expect_extent(layout.pages[0].extent, 7696, 4336);
  expect_offset(layout.tiles[0].origin, 0, 0);
  expect_offset(layout.tiles[1].origin, 3856, 0);
  expect_offset(layout.tiles[2].origin, 0, 2176);
  expect_offset(layout.tiles[3].origin, 3856, 2176);
  for (const MosaicTile& tile : layout.tiles) {
    expect_extent(tile.extent, 3840, 2160);
    EXPECT_EQ(tile.page, 0u);
  }
}

// Any number of cameras: the grid as square as it can be, never taller than
// wide, the mosaic ending 16-aligned past its last cell.
TEST(MosaicLayout, TakesAnyNumberOfCameras) {
  struct Case {
    std::size_t cameras;
    std::uint32_t columns;
    std::uint32_t rows;
    std::uint32_t width;
    std::uint32_t height;
  };
  // 64 x 48 pictures: cells 80 x 64 apart.
  for (const Case& c : {Case{1, 1, 1, 64, 48}, Case{2, 2, 1, 144, 48},
                        Case{3, 2, 2, 144, 112}, Case{4, 2, 2, 144, 112},
                        Case{5, 3, 2, 224, 112}, Case{6, 3, 2, 224, 112},
                        Case{7, 3, 3, 224, 176}, Case{10, 4, 3, 304, 176}}) {
    SCOPED_TRACE(c.cameras);
    const MosaicLayout layout =
        layout_of(std::vector<ImageSize>(c.cameras, ImageSize{64, 48}));
    expect_well_formed(layout, {}, c.cameras);
    EXPECT_EQ(layout.columns, c.columns);
    EXPECT_EQ(layout.rows, c.rows);
    expect_extent(layout.pitch, 80, 64);
    expect_extent(layout.extent, c.width, c.height);
    ASSERT_EQ(layout.pages.size(), 1u);
    expect_extent(layout.pages[0].extent, c.width, c.height);
  }
}

// Mixed sizes: every cell is the largest picture's, each picture at its
// cell's top-left at its own size.
TEST(MosaicLayout, SizesCellsToTheLargestPicture) {
  const MosaicLayout layout =
      layout_of({{640, 576}, {3840, 2160}, {1280, 720}});
  expect_well_formed(layout, {}, 3);
  expect_extent(layout.cell, 3840, 2160);
  expect_extent(layout.pitch, 3856, 2176);
  expect_extent(layout.extent, 7696, 4336);
  expect_offset(layout.tiles[0].origin, 0, 0);
  expect_extent(layout.tiles[0].extent, 640, 576);
  expect_offset(layout.tiles[1].origin, 3856, 0);
  expect_extent(layout.tiles[1].extent, 3840, 2160);
  expect_offset(layout.tiles[2].origin, 0, 2176);
  expect_extent(layout.tiles[2].extent, 1280, 720);

  // The largest width and the largest height need not be one picture's.
  const MosaicLayout crossed = layout_of({{100, 20}, {20, 60}});
  expect_extent(crossed.cell, 100, 60);
  expect_extent(crossed.pitch, 128, 80);
  expect_extent(crossed.extent, 240, 64);
}

// The pitch is the cell and the gutter rounded up to 16, so the gutter is the
// fewest texels between cells, and the mosaic's extent a multiple of 16.
TEST(MosaicLayout, AlignsThePitchAndTheExtentTo16) {
  const MosaicLayout odd_gutter = layout_of({{100, 50}, {100, 50}}, 5);
  expect_extent(odd_gutter.pitch, 112, 64);
  expect_extent(odd_gutter.extent, 224, 64);
  const MosaicLayout no_gutter = layout_of({{64, 48}, {64, 48}}, 0);
  expect_extent(no_gutter.pitch, 64, 48);
  expect_extent(no_gutter.extent, 128, 48);
  const MosaicLayout wide_gutter = layout_of({{64, 48}, {64, 48}}, 40);
  expect_extent(wide_gutter.pitch, 112, 96);
  expect_extent(wide_gutter.extent, 176, 48);
}

// Beyond a page's extent, the mosaic is paged in blocks of whole cells: each
// tile keeps its place in the mosaic, and a page holds the tiles of its
// block at their places relative to its origin.
TEST(MosaicLayout, PagesAMosaicWiderThanAPage) {
  const std::vector<ImageSize> sizes(5, ImageSize{64, 48});
  MosaicLimits limits;
  limits.max_extent = {144, 112};  // 2 x 2 cells
  const MosaicLayout layout = layout_of(sizes, kMosaicGutter, limits);
  expect_well_formed(layout, limits, 5);
  const MosaicLayout whole = layout_of(sizes);
  expect_extent(layout.extent, whole.extent.width, whole.extent.height);
  for (std::size_t i = 0; i < sizes.size(); ++i) {
    expect_offset(layout.tiles[i].origin, whole.tiles[i].origin.x,
                  whole.tiles[i].origin.y);
  }
  // Cells 0, 1 / 3, 4 on the first page; cell 2 alone on the second.
  ASSERT_EQ(layout.pages.size(), 2u);
  expect_offset(layout.pages[0].origin, 0, 0);
  expect_extent(layout.pages[0].extent, 144, 112);
  expect_offset(layout.pages[1].origin, 160, 0);
  expect_extent(layout.pages[1].extent, 64, 48);
  EXPECT_EQ(layout.tiles[2].page, 1u);
  expect_offset(layout.tiles[2].at, 0, 0);
  EXPECT_EQ(layout.tiles[4].page, 0u);
  expect_offset(layout.tiles[4].at, 80, 64);
}

// Over the memory budget, a page gives up a column or a row from its longer
// side until it fits, and a row once it is one column wide.
TEST(MosaicLayout, PagesAMosaicOverTheTexelBudget) {
  const std::vector<ImageSize> sizes(4, ImageSize{64, 48});
  MosaicLimits limits;
  limits.max_texels = 64 * 112;  // one column of two cells
  const MosaicLayout layout = layout_of(sizes, kMosaicGutter, limits);
  expect_well_formed(layout, limits, 4);
  ASSERT_EQ(layout.pages.size(), 2u);
  expect_offset(layout.pages[0].origin, 0, 0);
  expect_extent(layout.pages[0].extent, 64, 112);
  expect_offset(layout.pages[1].origin, 80, 0);
  expect_extent(layout.pages[1].extent, 64, 112);
  EXPECT_EQ(layout.tiles[0].page, 0u);
  EXPECT_EQ(layout.tiles[1].page, 1u);
  EXPECT_EQ(layout.tiles[2].page, 0u);
  EXPECT_EQ(layout.tiles[3].page, 1u);
  expect_offset(layout.tiles[3].at, 0, 64);

  // One cell a page: three cameras take three pages, not four.
  limits.max_texels = 64 * 48;
  const MosaicLayout single = layout_of(
      std::vector<ImageSize>(3, ImageSize{64, 48}), kMosaicGutter, limits);
  expect_well_formed(single, limits, 3);
  ASSERT_EQ(single.pages.size(), 3u);
  expect_offset(single.pages[2].origin, 0, 64);

  // Wide, short cells: a block one column wide and still wider than tall
  // gives up its second row.
  limits.max_texels = 64 * 16;
  const MosaicLayout flat = layout_of(
      std::vector<ImageSize>(4, ImageSize{64, 2}), kMosaicGutter, limits);
  expect_well_formed(flat, limits, 4);
  ASSERT_EQ(flat.pages.size(), 4u);
  for (std::size_t i = 0; i < flat.tiles.size(); ++i) {
    EXPECT_EQ(flat.tiles[i].page, i);
    expect_extent(flat.pages[i].extent, 64, 16);
  }
}

// Sixteen 4K cameras: a page of 4 x 4 cells is over the default budget, so
// two pages of 2 x 4 hold them.
TEST(MosaicLayout, PagesSixteen4KCamerasWithinTheDefaultBudget) {
  const MosaicLayout layout =
      layout_of(std::vector<ImageSize>(16, ImageSize{3840, 2160}));
  expect_well_formed(layout, {}, 16);
  expect_extent(layout.extent, 15408, 8688);
  ASSERT_EQ(layout.pages.size(), 2u);
  expect_extent(layout.pages[0].extent, 7696, 8688);
  expect_offset(layout.pages[1].origin, 7712, 0);
}

TEST(MosaicLayout, RefusesWhatNoMosaicHolds) {
  EXPECT_EQ(mosaic_layout({}).status().domain(), Code::InvalidArgument);
  for (const ImageSize bad : {ImageSize{0, 48}, ImageSize{64, 0},
                              ImageSize{65, 48}, ImageSize{64, 47}}) {
    SCOPED_TRACE(::testing::Message() << bad.width << "x" << bad.height);
    EXPECT_EQ(mosaic_layout({{64, 48}, bad}).status().domain(),
              Code::InvalidArgument);
  }
  // A cell wider, or taller, than a page.
  MosaicLimits limits;
  for (const VkExtent2D page : {VkExtent2D{63, 48}, VkExtent2D{64, 47}}) {
    limits.max_extent = page;
    EXPECT_EQ(
        mosaic_layout({{64, 48}}, kMosaicGutter, limits).status().domain(),
        Code::InvalidArgument)
        << page.width << "x" << page.height;
  }
  limits = {};
  limits.max_texels = 64 * 48 - 1;
  EXPECT_EQ(mosaic_layout({{64, 48}}, kMosaicGutter, limits).status().domain(),
            Code::InvalidArgument);
  // Beyond 2^24 texels across or down, whatever the limits.
  limits.max_extent = {~0u, ~0u};
  limits.max_texels = ~std::uint64_t{0};
  EXPECT_TRUE(mosaic_layout({{1u << 23, 2}}, 0, limits).ok());
  EXPECT_EQ(mosaic_layout({{1u << 23, 2}, {1u << 23, 2}}, 16, limits)
                .status()
                .domain(),
            Code::InvalidArgument);
  EXPECT_TRUE(mosaic_layout({{2, 1u << 23}, {2, 1u << 23}}, 16, limits).ok());
  EXPECT_EQ(mosaic_layout(std::vector<ImageSize>(3, ImageSize{2, 1u << 23}), 16,
                          limits)
                .status()
                .domain(),
            Code::InvalidArgument);
}

// A VideoToolbox MJPEG picture: NV12 in two images, BT.601 full range,
// centred chroma.
sensor::YuvImage mjpeg_picture(std::uint32_t width = 3840,
                               std::uint32_t height = 2160) {
  sensor::YuvImage picture;
  picture.layout = sensor::YuvLayout::Nv12;
  picture.image[0] = std::make_shared<core::Image>();
  picture.image[1] = std::make_shared<core::Image>();
  picture.width = width;
  picture.height = height;
  picture.kr = 0.299f;
  picture.kb = 0.114f;
  picture.full_range = true;
  picture.chroma_location = sensor::ChromaLocation::Center;
  return picture;
}

TEST(TileWrite, CopiesTheDecodersPlanesInPlaceToItsTile) {
  const sensor::YuvImage picture = mjpeg_picture();
  const core::Result<pipelines::ImageWrite> write =
      tile_write(picture, {3840, 2160}, {3856, 2176});
  ASSERT_TRUE(write.ok()) << write.status().message();
  EXPECT_EQ(write.value().planes[0].image, picture.image[0].get());
  EXPECT_EQ(write.value().planes[1].image, picture.image[1].get());
  EXPECT_EQ(write.value().planes[0].buffer, nullptr);
  EXPECT_EQ(write.value().planes[1].buffer, nullptr);
  expect_offset(write.value().offset, 3856, 2176);
  expect_extent(write.value().extent, 3840, 2160);
  // Images a readback took over onto a family of the device.
  sensor::YuvImage taken = mjpeg_picture();
  taken.queue_family = 0;
  EXPECT_TRUE(tile_write(taken, {3840, 2160}, {}).ok());
}

TEST(TileWrite, RefusesWhatATileCannotTake) {
  const VkExtent2D tile{3840, 2160};
  sensor::YuvImage i420 = mjpeg_picture();
  i420.layout = sensor::YuvLayout::I420;
  EXPECT_EQ(tile_write(i420, tile, {}).status().domain(), Code::Unsupported);

  sensor::YuvImage buffer = mjpeg_picture();
  buffer.image[0].reset();
  buffer.image[1].reset();
  buffer.device = std::make_shared<core::Buffer>();
  EXPECT_EQ(tile_write(buffer, tile, {}).status().domain(), Code::Unsupported);
  // NVDEC's images, which CUDA wrote, until a readback takes them over.
  sensor::YuvImage external = mjpeg_picture();
  external.queue_family = sensor::kQueueFamilyExternal;
  EXPECT_EQ(tile_write(external, tile, {}).status().domain(),
            Code::Unsupported);

  sensor::YuvImage no_chroma = mjpeg_picture();
  no_chroma.image[1].reset();
  EXPECT_EQ(tile_write(no_chroma, tile, {}).status().domain(),
            Code::InvalidArgument);
  sensor::YuvImage none;
  none.layout = sensor::YuvLayout::Nv12;
  EXPECT_EQ(tile_write(none, tile, {}).status().domain(),
            Code::InvalidArgument);
  EXPECT_EQ(tile_write(mjpeg_picture(0, 0), {0, 0}, {}).status().domain(),
            Code::InvalidArgument);

  // Another size than its tile's: gfx would copy a corner, or read past it.
  for (const VkExtent2D other : {VkExtent2D{1920, 1080}, VkExtent2D{3840, 2176},
                                 VkExtent2D{3856, 2160}}) {
    const core::Result<pipelines::ImageWrite> write =
        tile_write(mjpeg_picture(), other, {});
    EXPECT_EQ(write.status().domain(), Code::InvalidArgument);
    EXPECT_NE(write.status().message().find("3840x2160"), std::string::npos)
        << write.status().message();
  }
}

TEST(TileConversion, KeepsThePicturesMatrixRangeAndSiting) {
  const sensor::YuvImage mjpeg = mjpeg_picture();
  core::Result<pipelines::ImageConversion> conversion =
      tile_conversion({&mjpeg, nullptr, &mjpeg});
  ASSERT_TRUE(conversion.ok()) << conversion.status().message();
  EXPECT_EQ(conversion.value().kr, 0.299f);
  EXPECT_EQ(conversion.value().kb, 0.114f);
  EXPECT_TRUE(conversion.value().full_range);
  EXPECT_EQ(conversion.value().chroma_siting, pipelines::ChromaSiting::Center);
  EXPECT_EQ(conversion.value().encoding, pipelines::ImageEncoding::Srgb);

  // An HEVC picture: BT.709, limited range, left-sited chroma.
  sensor::YuvImage hevc = mjpeg_picture();
  const sensor::YcbcrWeights bt709 =
      sensor::ycbcr_weights(sensor::VideoColorMatrix::Bt709);
  hevc.kr = bt709.kr;
  hevc.kb = bt709.kb;
  hevc.full_range = false;
  hevc.chroma_location = sensor::ChromaLocation::Left;
  conversion = tile_conversion({nullptr, &hevc});
  ASSERT_TRUE(conversion.ok()) << conversion.status().message();
  EXPECT_EQ(conversion.value().kr, bt709.kr);
  EXPECT_EQ(conversion.value().kb, bt709.kb);
  EXPECT_FALSE(conversion.value().full_range);
  EXPECT_EQ(conversion.value().chroma_siting, pipelines::ChromaSiting::Left);
}

TEST(TileConversion, RefusesWhatOnePageCannotConvert) {
  const sensor::YuvImage mjpeg = mjpeg_picture();
  sensor::YuvImage matrix = mjpeg_picture();
  matrix.kr = 0.2126f;
  sensor::YuvImage range = mjpeg_picture();
  range.full_range = false;
  sensor::YuvImage siting = mjpeg_picture();
  siting.chroma_location = sensor::ChromaLocation::Left;
  for (const sensor::YuvImage* other : {&matrix, &range, &siting}) {
    EXPECT_EQ(tile_conversion({&mjpeg, nullptr, other}).status().domain(),
              Code::InvalidArgument);
  }

  for (const sensor::ChromaLocation location :
       {sensor::ChromaLocation::TopLeft, sensor::ChromaLocation::Top,
        sensor::ChromaLocation::BottomLeft, sensor::ChromaLocation::Bottom}) {
    sensor::YuvImage sited = mjpeg_picture();
    sited.chroma_location = location;
    EXPECT_EQ(tile_conversion({&sited}).status().domain(), Code::Unsupported)
        << static_cast<int>(location);
  }

  // Weights that are no Y'CbCr matrix's, which gfx would not convert.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (const auto& [kr, kb] : {std::pair{0.0f, 0.114f}, std::pair{0.299f, 0.0f},
                               std::pair{0.5f, 0.5f}, std::pair{nan, 0.114f}}) {
    sensor::YuvImage weighted = mjpeg_picture();
    weighted.kr = kr;
    weighted.kb = kb;
    EXPECT_EQ(tile_conversion({&weighted}).status().domain(),
              Code::InvalidArgument)
        << kr << ", " << kb;
  }

  EXPECT_EQ(tile_conversion({}).status().domain(), Code::InvalidArgument);
  EXPECT_EQ(tile_conversion({nullptr, nullptr}).status().domain(),
            Code::InvalidArgument);
}

}  // namespace
}  // namespace volumetric_kit::calib::viz
