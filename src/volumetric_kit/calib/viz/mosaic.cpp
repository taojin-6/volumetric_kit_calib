// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/viz/mosaic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"

namespace volumetric_kit::calib::viz {
namespace {

namespace pipelines = gfx::pipelines;
namespace sensor = recon::sensor;

constexpr std::uint64_t kAlign = 16;
// As far as a float -- the view's image coordinates -- holds every integer.
constexpr std::uint64_t kMaxMosaic = std::uint64_t{1} << 24;

std::uint64_t align_up(std::uint64_t value) noexcept {
  return (value + kAlign - 1) / kAlign * kAlign;
}

std::string size_text(std::uint64_t width, std::uint64_t height) {
  return std::to_string(width) + "x" + std::to_string(height);
}

// The smallest c with c * c >= n, for n >= 1.
std::uint64_t ceil_sqrt(std::uint64_t n) noexcept {
  auto c = static_cast<std::uint64_t>(std::sqrt(static_cast<double>(n)));
  while (c * c < n) ++c;
  while (c > 1 && (c - 1) * (c - 1) >= n) --c;
  return c;
}

// Cuts `rect` short at `end`, its right and bottom edges rounded to whole
// pixels, as the rasterizer covers pixel centres; a NaN cuts it to nothing.
void cut_at(VkRect2D& rect, glm::vec2 end) noexcept {
  const auto cut = [](std::int32_t from, std::uint32_t& size, float at) {
    const double lo = from;
    const double hi = lo + size;
    const double edge = std::round(static_cast<double>(at));
    const double kept = edge >= hi ? hi : (edge > lo ? edge : lo);
    size = static_cast<std::uint32_t>(kept - lo);
  };
  cut(rect.offset.x, rect.extent.width, end.x);
  cut(rect.offset.y, rect.extent.height, end.y);
}

bool same_coding(const pipelines::ImageConversion& a,
                 const pipelines::ImageConversion& b) noexcept {
  return a.kr == b.kr && a.kb == b.kb && a.full_range == b.full_range &&
         a.chroma_siting == b.chroma_siting;
}

core::Result<pipelines::ImageConversion> conversion_of(
    const sensor::YuvImage& picture) {
  pipelines::ImageConversion conversion;
  switch (picture.chroma_location) {
    case sensor::ChromaLocation::Left:
      conversion.chroma_siting = pipelines::ChromaSiting::Left;
      break;
    case sensor::ChromaLocation::Center:
      conversion.chroma_siting = pipelines::ChromaSiting::Center;
      break;
    default:
      return core::Status::unsupported(
          "chroma location " +
          std::to_string(static_cast<int>(picture.chroma_location)) +
          " is neither Left nor Center, the sitings gfx draws");
  }
  // As gfx's convert takes them, so a page written is never left unconverted.
  if (!(picture.kr > 0.0f && picture.kb > 0.0f &&
        picture.kr + picture.kb < 1.0f)) {
    return core::Status::invalid_argument(
        "matrix weights kr " + std::to_string(picture.kr) + " and kb " +
        std::to_string(picture.kb) + " are not a Y'CbCr matrix's");
  }
  conversion.kr = picture.kr;
  conversion.kb = picture.kb;
  conversion.full_range = picture.full_range;
  return conversion;
}

}  // namespace

core::Result<MosaicLayout> mosaic_layout(
    const std::vector<recon::camera::ImageSize>& sizes, std::uint32_t gutter,
    const MosaicLimits& limits) {
  return compute::guarded("mosaic_layout", [&]() -> core::Result<MosaicLayout> {
    if (sizes.empty()) {
      return core::Status::invalid_argument("mosaic_layout: no picture");
    }
    std::uint64_t cell_w = 0;
    std::uint64_t cell_h = 0;
    for (std::size_t i = 0; i < sizes.size(); ++i) {
      const recon::camera::ImageSize& size = sizes[i];
      if (size.width == 0 || size.height == 0 || size.width % 2 != 0 ||
          size.height % 2 != 0) {
        return core::Status::invalid_argument(
            "mosaic_layout: picture " + std::to_string(i) + " is " +
            size_text(size.width, size.height) +
            "; NV12 tiles are non-zero and even");
      }
      cell_w = std::max<std::uint64_t>(cell_w, size.width);
      cell_h = std::max<std::uint64_t>(cell_h, size.height);
    }
    const std::uint64_t n = sizes.size();
    const std::uint64_t columns = ceil_sqrt(n);
    const std::uint64_t rows = (n + columns - 1) / columns;
    const std::uint64_t pitch_x = align_up(cell_w + gutter);
    const std::uint64_t pitch_y = align_up(cell_h + gutter);
    // A block of c x r cells, from its first cell's origin: the last cell's
    // end, rounded up to a multiple of 16.
    const auto block_w = [&](std::uint64_t c) {
      return (c - 1) * pitch_x + align_up(cell_w);
    };
    const auto block_h = [&](std::uint64_t r) {
      return (r - 1) * pitch_y + align_up(cell_h);
    };
    const std::uint64_t width = block_w(columns);
    const std::uint64_t height = block_h(rows);
    if (width > kMaxMosaic || height > kMaxMosaic) {
      return core::Status::invalid_argument(
          "mosaic_layout: a " + size_text(width, height) +
          " mosaic is over 2^24 texels across or down");
    }
    const std::uint64_t max_w = limits.max_extent.width;
    const std::uint64_t max_h = limits.max_extent.height;
    if (block_w(1) > max_w || block_h(1) > max_h ||
        block_w(1) * block_h(1) > limits.max_texels) {
      return core::Status::invalid_argument(
          "mosaic_layout: a " + size_text(block_w(1), block_h(1)) +
          " cell is beyond a page's " + size_text(max_w, max_h) + " and " +
          std::to_string(limits.max_texels) + " texels");
    }
    std::uint64_t block_columns =
        std::min(columns, (max_w - block_w(1)) / pitch_x + 1);
    std::uint64_t block_rows =
        std::min(rows, (max_h - block_h(1)) / pitch_y + 1);
    while (block_w(block_columns) * block_h(block_rows) > limits.max_texels) {
      const bool wider = block_w(block_columns) >= block_h(block_rows);
      if ((wider && block_columns > 1) || block_rows == 1) {
        --block_columns;
      } else {
        --block_rows;
      }
    }

    MosaicLayout layout;
    layout.columns = static_cast<std::uint32_t>(columns);
    layout.rows = static_cast<std::uint32_t>(rows);
    layout.cell = {static_cast<std::uint32_t>(cell_w),
                   static_cast<std::uint32_t>(cell_h)};
    layout.pitch = {static_cast<std::uint32_t>(pitch_x),
                    static_cast<std::uint32_t>(pitch_y)};
    layout.extent = {static_cast<std::uint32_t>(width),
                     static_cast<std::uint32_t>(height)};
    layout.tiles.resize(sizes.size());
    // Row-major tiles meet their blocks in row-major order, a block's first
    // tile in its first row, so pages are numbered as they are met.
    const std::uint64_t blocks_across =
        (columns + block_columns - 1) / block_columns;
    constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> page_of_block(
        blocks_across * ((rows + block_rows - 1) / block_rows), kNone);
    for (std::size_t i = 0; i < sizes.size(); ++i) {
      const std::uint64_t column = i % columns;
      const std::uint64_t row = i / columns;
      const std::uint64_t first_column = column / block_columns * block_columns;
      const std::uint64_t first_row = row / block_rows * block_rows;
      std::size_t& page = page_of_block[(row / block_rows) * blocks_across +
                                        column / block_columns];
      if (page == kNone) {
        page = layout.pages.size();
        MosaicPage block;
        block.origin = {static_cast<std::int32_t>(first_column * pitch_x),
                        static_cast<std::int32_t>(first_row * pitch_y)};
        layout.pages.push_back(block);
      }
      MosaicPage& block = layout.pages[page];
      block.extent.width = std::max(
          block.extent.width,
          static_cast<std::uint32_t>(block_w(column - first_column + 1)));
      block.extent.height =
          std::max(block.extent.height,
                   static_cast<std::uint32_t>(block_h(row - first_row + 1)));
      MosaicTile& tile = layout.tiles[i];
      tile.origin = {static_cast<std::int32_t>(column * pitch_x),
                     static_cast<std::int32_t>(row * pitch_y)};
      tile.extent = {sizes[i].width, sizes[i].height};
      tile.page = page;
      tile.at = {tile.origin.x - block.origin.x,
                 tile.origin.y - block.origin.y};
    }
    return layout;
  });
}

core::Result<pipelines::ImageWrite> tile_write(const sensor::YuvImage& picture,
                                               VkExtent2D expected,
                                               VkOffset2D at) {
  return compute::guarded(
      "tile_write", [&]() -> core::Result<pipelines::ImageWrite> {
        if (picture.layout != sensor::YuvLayout::Nv12) {
          return core::Status::unsupported(
              "tile_write: an I420 picture; tiles take NV12");
        }
        if (picture.image[0] == nullptr && picture.image[1] == nullptr &&
            picture.device != nullptr) {
          return core::Status::unsupported(
              "tile_write: planes in a buffer; tiles take image planes");
        }
        if (picture.queue_family == sensor::kQueueFamilyExternal) {
          return core::Status::unsupported(
              "tile_write: images still owned outside Vulkan; tiles read "
              "them in place");
        }
        if (picture.image[0] == nullptr || picture.image[1] == nullptr) {
          return core::Status::invalid_argument(
              "tile_write: the picture lacks its luma or chroma image");
        }
        if (picture.width == 0 || picture.height == 0 ||
            picture.width != expected.width ||
            picture.height != expected.height) {
          return core::Status::invalid_argument(
              "tile_write: a " + size_text(picture.width, picture.height) +
              " picture for a " + size_text(expected.width, expected.height) +
              " tile");
        }
        pipelines::ImageWrite write;
        write.planes[0].image = picture.image[0].get();
        write.planes[1].image = picture.image[1].get();
        write.offset = at;
        write.extent = expected;
        return write;
      });
}

core::Result<pipelines::ImageConversion> tile_conversion(
    const std::vector<const sensor::YuvImage*>& pictures) {
  return compute::guarded(
      "tile_conversion", [&]() -> core::Result<pipelines::ImageConversion> {
        std::optional<pipelines::ImageConversion> shared;
        std::size_t first = 0;
        for (std::size_t i = 0; i < pictures.size(); ++i) {
          if (pictures[i] == nullptr) continue;
          core::Result<pipelines::ImageConversion> conversion =
              conversion_of(*pictures[i]);
          if (!conversion.ok()) {
            return std::move(conversion)
                .status()
                .with_context("tile_conversion: picture " + std::to_string(i));
          }
          if (!shared) {
            shared = conversion.value();
            first = i;
          } else if (!same_coding(*shared, conversion.value())) {
            return core::Status::invalid_argument(
                "tile_conversion: picture " + std::to_string(i) +
                "'s matrix, range or siting is not picture " +
                std::to_string(first) + "'s; a page converts with one");
          }
        }
        if (!shared) {
          return core::Status::invalid_argument("tile_conversion: no picture");
        }
        return *shared;
      });
}

RigMosaic::~RigMosaic() = default;

core::Result<std::unique_ptr<RigMosaic>> RigMosaic::create(
    const core::Device& device, core::Allocator& allocator,
    const gfx::RenderTargetLayout& layout,
    const std::vector<recon::camera::ImageSize>& pictures,
    const MosaicLimits& limits) {
  return compute::guarded(
      "RigMosaic::create", [&]() -> core::Result<std::unique_ptr<RigMosaic>> {
        std::unique_ptr<RigMosaic> mosaic(new RigMosaic());
        VKC_ASSIGN(mosaic->pipeline_,
                   pipelines::ImagePipeline::create(device, layout));
        const VkExtent2D device_max = mosaic->pipeline_.max_texture_extent();
        MosaicLimits page = limits;
        page.max_extent = {
            std::min(limits.max_extent.width, device_max.width),
            std::min(limits.max_extent.height, device_max.height)};
        VKC_ASSIGN(mosaic->layout_,
                   mosaic_layout(pictures, kMosaicGutter, page));
        const MosaicLayout& placed = mosaic->layout_;

        mosaic->textures_.reserve(placed.pages.size());
        for (const MosaicPage& block : placed.pages) {
          pipelines::ImageTextureDesc desc;
          desc.extent = block.extent;
          desc.mapping = pipelines::ImageMapping::Nv12;
          VKC_ASSIGN(pipelines::ImageTexture texture,
                     pipelines::ImageTexture::create(mosaic->pipeline_,
                                                     allocator, desc));
          mosaic->textures_.push_back(std::move(texture));
        }
        if (placed.pages.size() > 1) {
          pipelines::ImageTextureDesc desc;
          desc.extent = {1, 1};
          desc.mapping = pipelines::ImageMapping::Grey;
          desc.format = VK_FORMAT_R8_UNORM;
          VKC_ASSIGN(mosaic->backdrop_,
                     pipelines::ImageTexture::create(mosaic->pipeline_,
                                                     allocator, desc));
          core::BufferDesc black;
          black.size = 1;
          black.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
          black.memory = core::MemoryUsage::Staging;
          VKC_ASSIGN(mosaic->black_, allocator.create_buffer(black));
          *static_cast<std::uint8_t*>(mosaic->black_.mapped()) = 0;
        }
        mosaic->codings_.resize(placed.pages.size());
        mosaic->since_.resize(placed.tiles.size());
        mosaic->refused_.reserve(placed.tiles.size());
        mosaic->page_first_.resize(placed.pages.size());
        mosaic->page_count_.assign(placed.pages.size(), 0);
        for (const MosaicTile& tile : placed.tiles) {
          ++mosaic->page_count_[tile.page];
        }
        std::size_t first = 0;
        for (std::size_t p = 0; p < placed.pages.size(); ++p) {
          mosaic->page_first_[p] = first;
          first += mosaic->page_count_[p];
        }
        mosaic->writes_.resize(placed.tiles.size());
        mosaic->write_tile_.resize(placed.tiles.size());
        mosaic->page_pictures_.reserve(placed.tiles.size());
        mosaic->page_conversion_.resize(placed.pages.size());
        mosaic->draws_.resize(placed.pages.size() +
                              (mosaic->backdrop_.valid() ? 1 : 0));
        // An empty viewport until the caller sets one: nothing is drawn
        // before.
        mosaic->view_ = gfx::camera::ImageView2D(
            glm::vec2(static_cast<float>(placed.extent.width),
                      static_cast<float>(placed.extent.height)),
            glm::vec2(0.0f), glm::vec2(0.0f));
        return mosaic;
      });
}

core::Status RigMosaic::record(
    VkCommandBuffer cmd, const std::vector<const sensor::YuvImage*>& pictures) {
  return compute::guarded("RigMosaic::record", [&]() -> core::Status {
    bool started = false;
    try {
      core::Status recorded = record_pages(cmd, pictures, started);
      if (!recorded.ok()) refused_.clear();
      return recorded;
    } catch (...) {
      // What it recorded may never run: forget it all.
      if (started) discard();
      refused_.clear();
      throw;
    }
  });
}

core::Status RigMosaic::record_pages(
    VkCommandBuffer cmd, const std::vector<const sensor::YuvImage*>& pictures,
    bool& started) {
  const std::size_t tiles = layout_.tiles.size();
  if (pictures.size() != tiles) {
    return core::Status::invalid_argument(
        "RigMosaic::record: " + std::to_string(pictures.size()) +
        " pictures for " + std::to_string(tiles) + " tiles");
  }
  refused_.clear();
  // Check each picture, and group the writes of those taken by page, before
  // recording anything.
  std::fill(page_count_.begin(), page_count_.end(), 0);
  for (std::size_t i = 0; i < tiles; ++i) {
    if (pictures[i] == nullptr) continue;
    const MosaicTile& tile = layout_.tiles[i];
    core::Result<pipelines::ImageWrite> write =
        tile_write(*pictures[i], tile.extent, tile.at);
    if (!write.ok()) {
      refused_.push_back({i, std::move(write).status()});
      continue;
    }
    core::Result<pipelines::ImageConversion> coding =
        conversion_of(*pictures[i]);
    if (!coding.ok()) {
      refused_.push_back(
          {i, std::move(coding).status().with_context("tile_conversion")});
      continue;
    }
    const std::size_t slot = page_first_[tile.page] + page_count_[tile.page];
    ++page_count_[tile.page];
    writes_[slot] = write.value();
    write_tile_[slot] = i;
  }
  // A page's pictures share one coding, as gfx converts a page whole.
  bool any = false;
  for (std::size_t p = 0; p < textures_.size(); ++p) {
    if (page_count_[p] == 0) continue;
    page_pictures_.assign(tiles, nullptr);
    for (std::size_t slot = page_first_[p];
         slot < page_first_[p] + page_count_[p]; ++slot) {
      page_pictures_[write_tile_[slot]] = pictures[write_tile_[slot]];
    }
    const core::Result<pipelines::ImageConversion> conversion =
        tile_conversion(page_pictures_);
    if (!conversion.ok()) {
      refuse_page(p, conversion.status());
      continue;
    }
    page_conversion_[p] = conversion.value();
    any = true;
  }

  if (any) {
    started = true;
    if (backdrop_.valid() && !backdrop_.has_picture()) {
      pipelines::ImageWrite black;
      black.planes[0].buffer = &black_;
      black.extent = {1, 1};
      VKC_CHECK(pipeline_.record_write(cmd, backdrop_, &black, 1).ok(),
                "RigMosaic::record: the backdrop's write");
      VKC_CHECK(pipeline_.record_convert(cmd, backdrop_, {}).ok(),
                "RigMosaic::record: the backdrop's convert");
    }
  }
  for (std::size_t p = 0; p < textures_.size(); ++p) {
    if (page_count_[p] == 0) continue;
    const pipelines::ImageConversion& conversion = page_conversion_[p];
    core::Status written = pipeline_.record_write(
        cmd, textures_[p], writes_.data() + page_first_[p],
        static_cast<std::uint32_t>(page_count_[p]));
    if (!written.ok()) {
      refuse_page(p, std::move(written).with_context(
                         "RigMosaic::record: page " + std::to_string(p)));
      continue;
    }
    if (codings_[p] && !same_coding(*codings_[p], conversion)) {
      // gfx checks every write before recording any. Only after it accepts
      // this set may we forget the old coding's tiles. Re-record the checked
      // writes after discarding so the uncovered tiles are cleared black.
      forget_page(p);
      VKC_CHECK(
          pipeline_
              .record_write(cmd, textures_[p], writes_.data() + page_first_[p],
                            static_cast<std::uint32_t>(page_count_[p]))
              .ok(),
          "RigMosaic::record: a checked replacement write");
    }
    codings_[p] = conversion;  // what its planes now hold
    // A page written is converted: tile_conversion took only weights the
    // convert takes.
    VKC_CHECK(pipeline_.record_convert(cmd, textures_[p], conversion).ok(),
              "RigMosaic::record: a page's convert");
  }
  // The set ages every picture; the ones written start again.
  for (std::optional<std::uint64_t>& since : since_) {
    if (since) ++*since;
  }
  for (std::size_t p = 0; p < textures_.size(); ++p) {
    for (std::size_t slot = page_first_[p];
         slot < page_first_[p] + page_count_[p]; ++slot) {
      since_[write_tile_[slot]] = 0;
    }
  }
  return core::Status();
}

void RigMosaic::refuse_page(std::size_t page, const core::Status& status) {
  for (std::size_t slot = page_first_[page];
       slot < page_first_[page] + page_count_[page]; ++slot) {
    refused_.push_back({write_tile_[slot], status});
  }
  page_count_[page] = 0;
}

void RigMosaic::forget_page(std::size_t page) noexcept {
  textures_[page].discard();
  codings_[page].reset();
  for (std::size_t i = 0; i < layout_.tiles.size(); ++i) {
    if (layout_.tiles[i].page == page) since_[i].reset();
  }
}

void RigMosaic::discard() noexcept {
  backdrop_.discard();
  for (pipelines::ImageTexture& texture : textures_) texture.discard();
  for (std::optional<pipelines::ImageConversion>& coding : codings_) {
    coding.reset();
  }
  for (std::optional<std::uint64_t>& since : since_) since.reset();
}

void RigMosaic::draw(VkCommandBuffer cmd, VkExtent2D extent,
                     pipelines::ImageFilter magnify) noexcept {
  const float scale = view_.scale();
  std::size_t count = 0;
  if (backdrop_.valid()) {
    // One black texel as large as the mosaic's longer side, from its top-left
    // corner, cut at its right and bottom edges.
    pipelines::ImageDraw& backdrop = draws_[count++];
    backdrop = pipelines::image_draw(backdrop_, view_);
    const glm::vec2 size(static_cast<float>(layout_.extent.width),
                         static_cast<float>(layout_.extent.height));
    backdrop.scale = scale * std::max(size.x, size.y);
    cut_at(backdrop.viewport, view_.image_to_target(size));
  }
  for (std::size_t p = 0; p < textures_.size(); ++p) {
    pipelines::ImageDraw& draw = draws_[count++];
    draw = pipelines::image_draw(textures_[p], view_, magnify);
    const VkOffset2D origin = layout_.pages[p].origin;
    draw.origin +=
        glm::vec2(static_cast<float>(origin.x), static_cast<float>(origin.y)) *
        scale;
  }
  pipeline_.submit(cmd,
                   {extent, draws_.data(), static_cast<std::uint32_t>(count)});
}

gfx::ui::OverlayRegion RigMosaic::tile_region(std::size_t index) const {
  VKC_CHECK(index < layout_.tiles.size(), "RigMosaic::tile_region: no tile");
  const MosaicTile& tile = layout_.tiles[index];
  return {
      {static_cast<float>(tile.origin.x), static_cast<float>(tile.origin.y)},
      {static_cast<float>(tile.extent.width),
       static_cast<float>(tile.extent.height)}};
}

std::optional<std::size_t> RigMosaic::tile_at(
    glm::vec2 target_point) const noexcept {
  const glm::vec2 lo = view_.viewport_origin();
  const glm::vec2 hi = lo + view_.viewport_size();
  if (!(target_point.x >= lo.x && target_point.x < hi.x &&
        target_point.y >= lo.y && target_point.y < hi.y)) {
    return std::nullopt;
  }
  const glm::vec2 p = view_.target_to_image(target_point);
  if (!(p.x >= 0.0f && p.y >= 0.0f &&
        p.x < static_cast<float>(layout_.extent.width) &&
        p.y < static_cast<float>(layout_.extent.height))) {
    return std::nullopt;
  }
  const auto column = static_cast<std::uint64_t>(p.x / layout_.pitch.width);
  const auto row = static_cast<std::uint64_t>(p.y / layout_.pitch.height);
  if (column >= layout_.columns || row >= layout_.rows) return std::nullopt;
  const std::uint64_t index = row * layout_.columns + column;
  if (index >= layout_.tiles.size()) return std::nullopt;
  const MosaicTile& tile = layout_.tiles[index];
  if (p.x - static_cast<float>(tile.origin.x) >=
          static_cast<float>(tile.extent.width) ||
      p.y - static_cast<float>(tile.origin.y) >=
          static_cast<float>(tile.extent.height)) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(index);
}

std::optional<std::uint64_t> RigMosaic::sets_since_picture(
    std::size_t index) const {
  VKC_CHECK(index < since_.size(), "RigMosaic::sets_since_picture: no tile");
  return since_[index];
}

}  // namespace volumetric_kit::calib::viz
