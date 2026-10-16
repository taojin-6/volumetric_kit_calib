// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/viz/tiles.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

#include <glm/common.hpp>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/core/base/check.hpp"

namespace volumetric_kit::calib::viz {
namespace {

namespace pipelines = gfx::pipelines;

// One axis of a tile: the start and length of cell `cell` of `cells` over
// [origin, origin + size), `gap` apart, with edges on whole pixels.
void cell_span(float origin, float size, int cells, std::size_t cell, float gap,
               float& start, float& length) noexcept {
  const float usable =
      std::max(0.0f, size - gap * static_cast<float>(cells - 1));
  const float step = usable / static_cast<float>(cells);
  const float lo = origin + static_cast<float>(cell) * (step + gap);
  start = std::round(lo);
  length = std::max(0.0f, std::round(lo + step) - start);
}

// A tile's rectangle as a draw's viewport, clamped to non-negative pixels.
VkRect2D to_viewport(const TileRect& rect) noexcept {
  const glm::vec2 lo = glm::max(rect.origin, glm::vec2(0.0f));
  const glm::vec2 hi = glm::max(rect.origin + rect.size, lo);
  VkRect2D viewport{};
  viewport.offset = {static_cast<std::int32_t>(lo.x),
                     static_cast<std::int32_t>(lo.y)};
  viewport.extent = {static_cast<std::uint32_t>(hi.x - lo.x),
                     static_cast<std::uint32_t>(hi.y - lo.y)};
  return viewport;
}

}  // namespace

TileGrid tile_grid(std::size_t count) noexcept {
  if (count == 0) return {};
  std::size_t columns = 1;
  while (columns * columns < count) ++columns;
  const std::size_t rows = (count + columns - 1) / columns;
  return {static_cast<int>(columns), static_cast<int>(rows)};
}

TileRect tile_rect(const TileRect& area, std::size_t count, std::size_t index,
                   float gap) noexcept {
  TileRect rect{area.origin, {0.0f, 0.0f}};
  const bool finite = std::isfinite(area.origin.x) &&
                      std::isfinite(area.origin.y) &&
                      std::isfinite(area.size.x) && std::isfinite(area.size.y);
  if (index >= count || !finite || !(area.size.x > 0.0f) ||
      !(area.size.y > 0.0f)) {
    return rect;
  }
  if (!(gap >= 0.0f) || !std::isfinite(gap)) gap = 0.0f;
  const TileGrid grid = tile_grid(count);
  const auto columns = static_cast<std::size_t>(grid.columns);
  cell_span(area.origin.x, area.size.x, grid.columns, index % columns, gap,
            rect.origin.x, rect.size.x);
  cell_span(area.origin.y, area.size.y, grid.rows, index / columns, gap,
            rect.origin.y, rect.size.y);
  return rect;
}

core::Result<pipelines::ImageUpdate> tile_update(
    const recon::sensor::YuvImage& picture) {
  return compute::guarded(
      "tile_update", [&]() -> core::Result<pipelines::ImageUpdate> {
        if (picture.layout != recon::sensor::YuvLayout::Nv12) {
          return core::Status::unsupported(
              "tile_update: an I420 picture; tiles draw NV12");
        }
        if (picture.image[0] == nullptr && picture.image[1] == nullptr &&
            picture.device != nullptr) {
          return core::Status::unsupported(
              "tile_update: planes in a buffer; tiles read image planes");
        }
        if (picture.image[0] == nullptr || picture.image[1] == nullptr) {
          return core::Status::invalid_argument(
              "tile_update: the picture lacks its luma or chroma image");
        }
        if (picture.width == 0 || picture.height == 0) {
          return core::Status::invalid_argument(
              "tile_update: a picture of zero size");
        }
        pipelines::ImageUpdate update;
        switch (picture.chroma_location) {
          case recon::sensor::ChromaLocation::Left:
            update.chroma_siting = pipelines::ChromaSiting::Left;
            break;
          case recon::sensor::ChromaLocation::Center:
            update.chroma_siting = pipelines::ChromaSiting::Center;
            break;
          default:
            return core::Status::unsupported(
                "tile_update: chroma location " +
                std::to_string(static_cast<int>(picture.chroma_location)) +
                " is neither Left nor Center, the sitings tiles draw");
        }
        update.planes[0].image = picture.image[0].get();
        update.planes[1].image = picture.image[1].get();
        update.kr = picture.kr;
        update.kb = picture.kb;
        update.full_range = picture.full_range;
        return update;
      });
}

Tiles::~Tiles() = default;

core::Result<std::unique_ptr<Tiles>> Tiles::create(
    const core::Device& device, core::Allocator& allocator,
    const gfx::RenderTargetLayout& layout,
    const std::vector<recon::camera::ImageSize>& pictures) {
  return compute::guarded(
      "Tiles::create", [&]() -> core::Result<std::unique_ptr<Tiles>> {
        if (pictures.empty()) {
          return core::Status::invalid_argument("Tiles::create: no picture");
        }
        for (std::size_t i = 0; i < pictures.size(); ++i) {
          if (pictures[i].width == 0 || pictures[i].height == 0) {
            return core::Status::invalid_argument("Tiles::create: picture " +
                                                  std::to_string(i) +
                                                  " has a zero size");
          }
        }
        std::unique_ptr<Tiles> tiles(new Tiles());
        VKC_ASSIGN(tiles->pipeline_,
                   pipelines::ImagePipeline::create(device, layout));
        tiles->textures_.reserve(pictures.size());
        tiles->views_.reserve(pictures.size());
        for (const recon::camera::ImageSize& size : pictures) {
          pipelines::ImageTextureDesc desc;
          desc.extent = {size.width, size.height};
          desc.mapping = pipelines::ImageMapping::Nv12;
          VKC_ASSIGN(pipelines::ImageTexture texture,
                     pipelines::ImageTexture::create(tiles->pipeline_,
                                                     allocator, desc));
          tiles->textures_.push_back(std::move(texture));
          // An empty viewport until set_area: nothing is drawn before.
          tiles->views_.emplace_back(glm::vec2(static_cast<float>(size.width),
                                               static_cast<float>(size.height)),
                                     glm::vec2(0.0f), glm::vec2(0.0f));
        }
        tiles->rects_.resize(pictures.size());
        tiles->draws_.resize(pictures.size());
        return tiles;
      });
}

void Tiles::set_area(const TileRect& area, float gap) noexcept {
  for (std::size_t i = 0; i < views_.size(); ++i) {
    rects_[i] = tile_rect(area, views_.size(), i, gap);
    views_[i].set_viewport(rects_[i].origin, rects_[i].size);
  }
}

TileRect Tiles::rect(std::size_t index) const {
  VKC_CHECK(index < rects_.size(), "Tiles::rect: no such tile");
  return rects_[index];
}

gfx::camera::ImageView2D& Tiles::view(std::size_t index) {
  VKC_CHECK(index < views_.size(), "Tiles::view: no such tile");
  return views_[index];
}

const gfx::camera::ImageView2D& Tiles::view(std::size_t index) const {
  VKC_CHECK(index < views_.size(), "Tiles::view: no such tile");
  return views_[index];
}

core::Status Tiles::record_update(VkCommandBuffer cmd, std::size_t index,
                                  const recon::sensor::YuvImage& picture) {
  return compute::guarded("Tiles::record_update", [&]() -> core::Status {
    if (index >= textures_.size()) {
      return core::Status::invalid_argument("Tiles::record_update: tile " +
                                            std::to_string(index) + " of " +
                                            std::to_string(textures_.size()));
    }
    const VkExtent2D extent = textures_[index].extent();
    if (picture.width != extent.width || picture.height != extent.height) {
      return core::Status::invalid_argument(
          "Tiles::record_update: tile " + std::to_string(index) + " is " +
          std::to_string(extent.width) + "x" + std::to_string(extent.height) +
          ", its picture " + std::to_string(picture.width) + "x" +
          std::to_string(picture.height));
    }
    VKC_ASSIGN(const pipelines::ImageUpdate update, tile_update(picture));
    return pipeline_.record_update(cmd, textures_[index], update)
        .with_context("Tiles::record_update");
  });
}

void Tiles::submit(VkCommandBuffer cmd, VkExtent2D extent,
                   pipelines::ImageFilter magnify) noexcept {
  for (std::size_t i = 0; i < textures_.size(); ++i) {
    pipelines::ImageDraw& draw = draws_[i];
    draw.texture = &textures_[i];
    draw.viewport = to_viewport(rects_[i]);
    draw.origin = views_[i].origin();
    draw.scale = views_[i].scale();
    draw.magnify = magnify;
  }
  pipeline_.submit(
      cmd, {extent, draws_.data(), static_cast<std::uint32_t>(draws_.size())});
}

}  // namespace volumetric_kit::calib::viz
