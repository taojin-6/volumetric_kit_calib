// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The rig mosaic and its marks drawn offscreen on a device, as the live view
// draws them: synthetic NV12 pictures in device images -- as a decoder leaves
// them, read in place -- each with a bright dot, written into their tiles of
// one mosaic (or of its pages), and marked through the mosaic's view on a
// high-DPI ImGui frame. The dot each tile draws must sit under the mark its
// overlay draws, for 1, 3 and 5 cameras, of mixed sizes, on one page or
// several; the coverage cell, the marker's outline and the title must land
// where their marks say, inside their tile. Then what a set leaves: a missing
// camera's last picture, a black gutter, paged or not, nothing recorded
// without a picture, a picture or a page it cannot take left out while the
// others are recorded, a clean record after a discard. Then every panel and
// the board editor drawn in one frame. Runs under the validation layer when the
// environment asks (VKC_TEST_VALIDATION); skips without a Vulkan device.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/common.hpp>
#include <glm/vec2.hpp>

#include "imgui.h"
#include "volumetric_kit/calib/viz/board_editor.hpp"
#include "volumetric_kit/calib/viz/mosaic.hpp"
#include "volumetric_kit/calib/viz/overlay.hpp"
#include "volumetric_kit/calib/viz/panels.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/core/vulkan/command_buffer.hpp"
#include "volumetric_kit/core/vulkan/command_pool.hpp"
#include "volumetric_kit/core/vulkan/image.hpp"
#include "volumetric_kit/gfx/core/device_requirements.hpp"
#include "volumetric_kit/gfx/core/image_barrier.hpp"
#include "volumetric_kit/gfx/core/offscreen_target.hpp"
#include "volumetric_kit/gfx/core/texture_upload.hpp"
#include "volumetric_kit/gfx/ui/imgui_overlay.hpp"
#include "volumetric_kit/gfx/ui/metrics_panel.hpp"

namespace volumetric_kit::calib::viz {
namespace {

using Code = core::Status::Code;
using sensor::camera::ImageSize;

constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;
constexpr VkExtent2D kExtent{256, 128};
const glm::vec2 kTarget(static_cast<float>(kExtent.width),
                        static_cast<float>(kExtent.height));
// The ImGui frame is half the target's size at twice the scale: a high-DPI
// display, whose window coordinates are half its framebuffer pixels.
constexpr float kFramebufferScale = 2.0f;

gfx::RenderTargetLayout color_layout() {
  gfx::RenderTargetLayout layout;
  layout.color_formats[0] = kFormat;
  layout.color_count = 1;
  return layout;
}

// An NV12 picture of one luma value but for one white texel, in device
// images.
struct Picture {
  std::shared_ptr<core::Image> luma;
  std::shared_ptr<core::Image> chroma;
  glm::vec2 dot{0.0f, 0.0f};  // the white texel, in OpenCV's pixels
  sensor::YuvImage yuv;
};

// A rectangle of target pixels.
struct Rect {
  glm::vec2 origin{0.0f, 0.0f};
  glm::vec2 size{0.0f, 0.0f};
};

// The rendered target's pixels.
struct Pixels {
  std::vector<std::uint8_t> bytes;
  const std::uint8_t* at(std::uint32_t x, std::uint32_t y) const {
    return &bytes[(static_cast<std::size_t>(y) * kExtent.width + x) * 4];
  }
};

// The whole pixels of image rectangle [lo, hi) as the view shows it, `inset`
// pixels in from each side (out, when negative), held inside the viewport
// and the target.
Rect target_rect(const gfx::camera::ImageView2D& view, glm::vec2 lo,
                 glm::vec2 hi, float inset = 0.0f) {
  const glm::vec2 viewport_lo = glm::max(view.viewport_origin(), glm::vec2(0));
  const glm::vec2 viewport_hi =
      glm::min(view.viewport_origin() + view.viewport_size(), kTarget);
  const glm::vec2 a = glm::clamp(glm::floor(view.image_to_target(lo) + inset),
                                 viewport_lo, viewport_hi);
  const glm::vec2 b =
      glm::clamp(glm::ceil(view.image_to_target(hi) - inset), a, viewport_hi);
  return {a, b - a};
}

// Tile `i`'s visible pixels.
Rect visible(const RigMosaic& mosaic, std::size_t i) {
  const gfx::ui::OverlayRegion region = mosaic.tile_region(i);
  return target_rect(mosaic.view(), region.origin, region.origin + region.size,
                     -1.0f);
}

const Rect kWhole{{0.0f, 0.0f}, kTarget};

template <class Visit>
void for_each_pixel(const Rect& rect, Visit visit) {
  const auto x0 = static_cast<std::uint32_t>(rect.origin.x);
  const auto y0 = static_cast<std::uint32_t>(rect.origin.y);
  const auto x1 = static_cast<std::uint32_t>(rect.origin.x + rect.size.x);
  const auto y1 = static_cast<std::uint32_t>(rect.origin.y + rect.size.y);
  for (std::uint32_t y = y0; y < y1; ++y) {
    for (std::uint32_t x = x0; x < x1; ++x) visit(x, y);
  }
}

// The weighted centroid, in target pixels (centres at + 0.5), of the pixels
// in `rect` whose weight is positive.
template <class Weight>
std::optional<glm::vec2> centroid(const Pixels& pixels, const Rect& rect,
                                  Weight weight) {
  double sum = 0.0;
  double sx = 0.0;
  double sy = 0.0;
  for_each_pixel(rect, [&](std::uint32_t x, std::uint32_t y) {
    const double w = weight(pixels.at(x, y));
    if (w <= 0.0) return;
    sum += w;
    sx += w * (x + 0.5);
    sy += w * (y + 0.5);
  });
  if (sum <= 0.0) return std::nullopt;
  return glm::vec2(static_cast<float>(sx / sum), static_cast<float>(sy / sum));
}

// The top-left corner, in target pixels, of the pixels in `rect` whose
// weight is positive.
template <class Weight>
std::optional<glm::vec2> top_left(const Pixels& pixels, const Rect& rect,
                                  Weight weight) {
  std::optional<glm::vec2> corner;
  for_each_pixel(rect, [&](std::uint32_t x, std::uint32_t y) {
    if (weight(pixels.at(x, y)) <= 0.0) return;
    const glm::vec2 p(static_cast<float>(x), static_cast<float>(y));
    corner = corner ? glm::min(*corner, p) : p;
  });
  return corner;
}

// How many pixels in `rect` have a positive weight.
template <class Weight>
int count(const Pixels& pixels, const Rect& rect, Weight weight) {
  int n = 0;
  for_each_pixel(rect, [&](std::uint32_t x, std::uint32_t y) {
    if (weight(pixels.at(x, y)) > 0.0) ++n;
  });
  return n;
}

double white(const std::uint8_t* px) {
  return px[0] > 128 && px[1] > 128 && px[2] > 128 ? px[1] : 0.0;
}
// Brighter than a grey picture's 160.
double bright(const std::uint8_t* px) {
  return px[0] > 220 && px[1] > 220 && px[2] > 220 ? 1.0 : 0.0;
}
double red(const std::uint8_t* px) {
  return px[0] > 128 && px[1] < 96 && px[2] < 96 ? px[0] : 0.0;
}
double blue(const std::uint8_t* px) {
  return px[2] > 64 && px[0] < 32 && px[1] < 32 ? px[2] : 0.0;
}
// Weighted by coverage, faint anti-aliased edges included, so a line's
// centroid is its middle.
double yellow(const std::uint8_t* px) {
  return px[0] > 16 && px[1] > 16 && px[2] < 32 ? px[1] : 0.0;
}
// Lit at all: black, through the NV12 matrix, stays below 8.
double lit(const std::uint8_t* px) {
  return px[0] > 8 || px[1] > 8 || px[2] > 8 ? 1.0 : 0.0;
}
// At full intensity.
double full(const std::uint8_t* px) {
  return px[0] >= 250 && px[1] >= 250 && px[2] >= 250 ? 1.0 : 0.0;
}
// The grey the target is cleared to, where a test asks for it.
double grey(const std::uint8_t* px) {
  const auto near = [](std::uint8_t v) { return v >= 125 && v <= 131; };
  return near(px[0]) && near(px[1]) && near(px[2]) ? 1.0 : 0.0;
}

std::vector<Picture*> all(std::vector<Picture>& pictures) {
  std::vector<Picture*> pointers;
  for (Picture& picture : pictures) pointers.push_back(&picture);
  return pointers;
}

// The mosaic's view fitted to `area` (the whole target by default).
void fit(RigMosaic& mosaic, const Rect& area = kWhole) {
  mosaic.view().set_viewport(area.origin, area.size);
  mosaic.view().fit();
}

class VizRenderTest : public core::test::VulkanDeviceTest {
 protected:
  core::DeviceRequirements requirements() const override {
    return gfx::device_requirements();
  }

  std::unique_ptr<RigMosaic> make_mosaic(const std::vector<ImageSize>& sizes,
                                         const MosaicLimits& limits = {}) {
    core::Result<std::unique_ptr<RigMosaic>> made =
        RigMosaic::create(device(), allocator(), color_layout(), sizes, limits);
    EXPECT_TRUE(made.ok()) << made.status().message();
    return made.ok() ? std::move(made).value() : nullptr;
  }

  // A `width` x `height` picture of luma `fill` with its white texel at
  // `dot`, its planes uploaded and left in GENERAL, as sensor's decoders leave
  // them; BT.601 full range, centred chroma, as MJPEG.
  Picture make_picture(std::uint32_t width, std::uint32_t height, glm::vec2 dot,
                       std::uint8_t fill = 0) {
    Picture picture;
    picture.dot = dot;
    std::vector<std::uint8_t> luma(static_cast<std::size_t>(width) * height,
                                   fill);
    luma[static_cast<std::size_t>(dot.y) * width +
         static_cast<std::size_t>(dot.x)] = 255;
    const std::uint32_t cw = (width + 1) / 2;
    const std::uint32_t ch = (height + 1) / 2;
    const std::vector<std::uint8_t> chroma(
        static_cast<std::size_t>(cw) * ch * 2, 128);
    picture.luma = upload(VK_FORMAT_R8_UNORM, {width, height}, luma);
    picture.chroma = upload(VK_FORMAT_R8G8_UNORM, {cw, ch}, chroma);
    picture.yuv.layout = sensor::YuvLayout::Nv12;
    picture.yuv.image[0] = picture.luma;
    picture.yuv.image[1] = picture.chroma;
    picture.yuv.width = width;
    picture.yuv.height = height;
    picture.yuv.full_range = true;
    picture.yuv.chroma_location = sensor::ChromaLocation::Center;
    return picture;
  }

  std::shared_ptr<core::Image> upload(VkFormat format, VkExtent2D extent,
                                      const std::vector<std::uint8_t>& bytes) {
    gfx::ImageUploadDesc desc;
    desc.extent = extent;
    desc.format = format;
    desc.pixels = bytes.data();
    desc.size = bytes.size();
    core::Result<core::Image> image =
        gfx::upload_texture(device(), allocator(), desc);
    EXPECT_TRUE(image.ok()) << image.status().message();
    if (!image.ok()) return std::make_shared<core::Image>();
    image.value().set_layout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    return std::make_shared<core::Image>(std::move(image).value());
  }

  // Moves a fresh upload to GENERAL, as a decoder leaves its picture.
  static void to_general(VkCommandBuffer cmd, core::Image& image) {
    if (image.layout() == VK_IMAGE_LAYOUT_GENERAL) return;
    gfx::ImageBarrierDesc b;
    b.image = image.handle();
    b.src_stage = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
    b.dst_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    b.dst_access = VK_ACCESS_TRANSFER_READ_BIT;
    b.old_layout = image.layout();
    b.new_layout = VK_IMAGE_LAYOUT_GENERAL;
    gfx::cmd_image_barrier(cmd, b);
    image.set_layout(VK_IMAGE_LAYOUT_GENERAL);
  }

  std::optional<gfx::ui::ImGuiOverlay> make_overlay(
      float framebuffer_scale = kFramebufferScale) {
    gfx::ui::ImGuiOverlayConfig config;
    config.layout = color_layout();
    core::Result<gfx::ui::ImGuiOverlay> overlay =
        gfx::ui::ImGuiOverlay::create(device(), instance().handle(), config);
    EXPECT_TRUE(overlay.ok()) << overlay.status().message();
    if (!overlay.ok()) return std::nullopt;
    ImGui::SetCurrentContext(overlay.value().context());
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(kExtent.width / framebuffer_scale,
                            kExtent.height / framebuffer_scale);
    io.DisplayFramebufferScale = ImVec2(framebuffer_scale, framebuffer_scale);
    return std::move(overlay).value();
  }

  // What render clears its target to: black, unless a test asks otherwise.
  VkClearColorValue clear_{};

  // Records each camera's new picture (null for none) into the mosaic, then
  // draws it (magnified by `magnify`, or draw's default), then the ImGui
  // frame `overlay` built (when given), into a fresh target cleared to
  // `clear_`; its pixels.
  Pixels render(RigMosaic& mosaic, const std::vector<Picture*>& pictures,
                gfx::ui::ImGuiOverlay* overlay = nullptr,
                std::optional<gfx::pipelines::ImageFilter> magnify = {}) {
    gfx::OffscreenTargetDesc desc;
    desc.extent = kExtent;
    desc.color_format = kFormat;
    core::Result<gfx::OffscreenTarget> target =
        gfx::OffscreenTarget::create(allocator(), desc);
    EXPECT_TRUE(target.ok()) << target.status().message();
    core::Result<core::CommandPool> pool =
        core::CommandPool::create(device().handle(), device().queue_family());
    EXPECT_TRUE(pool.ok()) << pool.status().message();
    if (!target.ok() || !pool.ok()) return {};
    core::Result<core::CommandBuffer> cmd = pool.value().allocate_primary();
    EXPECT_TRUE(cmd.ok()) << cmd.status().message();
    if (!cmd.ok()) return {};
    EXPECT_TRUE(
        cmd.value().begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT).ok());
    const VkCommandBuffer raw = cmd.value().handle();

    std::vector<const sensor::YuvImage*> yuv(pictures.size(), nullptr);
    for (std::size_t i = 0; i < pictures.size(); ++i) {
      if (pictures[i] == nullptr) continue;
      to_general(raw, *pictures[i]->luma);
      to_general(raw, *pictures[i]->chroma);
      yuv[i] = &pictures[i]->yuv;
    }
    const core::Status recorded = mosaic.record(raw, yuv);
    EXPECT_TRUE(recorded.ok()) << recorded.message();
    target.value().prepare(raw);
    const gfx::RenderTarget rt = target.value().target();
    gfx::RenderTargetBeginInfo begin;
    begin.clear_color = clear_;
    rt.begin(raw, begin);
    if (magnify) {
      mosaic.draw(raw, kExtent, *magnify);
    } else {
      mosaic.draw(raw, kExtent);
    }
    if (overlay != nullptr) overlay->render(raw);
    rt.end(raw);
    target.value().record_readback(raw);
    EXPECT_TRUE(cmd.value().end().ok());
    const core::Status submitted = device().submit_and_wait(raw);
    EXPECT_TRUE(submitted.ok()) << submitted.message();
    if (!submitted.ok()) return {};
    const auto* px = static_cast<const std::uint8_t*>(target.value().pixels());
    return {std::vector<std::uint8_t>(
        px, px + static_cast<std::size_t>(kExtent.width) * kExtent.height * 4)};
  }
};

// Each camera's white dot, drawn by the mosaic, must lie under the red mark
// its overlay draws over its tile, and both where the view puts the texel's
// centre: OpenCV's (x, y) at the view's tile origin + (x + 0.5, y + 0.5). At
// 2.67 target pixels a texel or more, a mark half a texel off misses by
// 1.33. For 1 camera zoomed off its dot; 3 of mixed sizes, on one page and on
// a page each; and 5, on one page and on two.
TEST_F(VizRenderTest, EachTilesDotSitsUnderItsMark) {
  struct Case {
    const char* name;
    std::vector<ImageSize> sizes;
    MosaicLimits limits;
    std::size_t pages;
  };
  MosaicLimits two_by_two;
  two_by_two.max_extent = {48, 48};
  MosaicLimits one_cell;
  one_cell.max_texels = 16 * 16;
  const std::vector<ImageSize> mixed = {{16, 12}, {8, 6}, {12, 10}};
  const std::vector<ImageSize> five(5, ImageSize{16, 12});
  const Case cases[] = {
      {"1 camera", {{16, 12}}, {}, 1},
      {"3 mixed", mixed, {}, 1},
      {"3 mixed, a page each", mixed, one_cell, 3},
      {"5 cameras", five, {}, 1},
      {"5 cameras on 2 pages", five, two_by_two, 2},
  };
  std::optional<gfx::ui::ImGuiOverlay> overlay = make_overlay();
  ASSERT_TRUE(overlay.has_value());
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    std::unique_ptr<RigMosaic> mosaic = make_mosaic(c.sizes, c.limits);
    ASSERT_NE(mosaic, nullptr);
    ASSERT_EQ(mosaic->layout().pages.size(), c.pages);
    std::vector<Picture> pictures;
    for (std::size_t i = 0; i < c.sizes.size(); ++i) {
      const auto k = static_cast<float>(i);
      const ImageSize& size = c.sizes[i];
      pictures.push_back(make_picture(
          size.width, size.height,
          {std::min(static_cast<float>(size.width) - 2.0f, 3.0f + 2.0f * k),
           std::min(static_cast<float>(size.height) - 2.0f, 2.0f + k)}));
    }
    fit(*mosaic);
    if (c.sizes.size() == 1) mosaic->view().zoom_about({90.0f, 30.0f}, 2.0f);
    ASSERT_GT(mosaic->view().scale(), 2.6f);

    const Pixels plain = render(*mosaic, all(pictures));
    ASSERT_FALSE(plain.bytes.empty());
    overlay->new_frame();
    for (std::size_t i = 0; i < pictures.size(); ++i) {
      Detection found;
      found.observation.points.push_back(
          {0, {pictures[i].dot.x, pictures[i].dot.y}, 1.0});
      TileOverlay tile;
      tile.check.verdict = ViewVerdict::Collinear;  // red dots
      tile.detection = &found;
      OverlayStyle style;
      style.corner_ids = false;
      style.corner_radius = 1.5f;
      const core::Result<TileMarks> marks = tile_marks(tile, {}, style);
      ASSERT_TRUE(marks.ok()) << marks.status().message();
      draw_tile_marks(mosaic->view(), mosaic->tile_region(i), marks.value());
    }
    const Pixels marked = render(*mosaic, all(pictures), &*overlay);
    ASSERT_FALSE(marked.bytes.empty());

    for (std::size_t i = 0; i < pictures.size(); ++i) {
      SCOPED_TRACE(i);
      const glm::vec2 expected = mosaic->view().image_to_target(
          mosaic->tile_region(i).origin + pictures[i].dot + 0.5f);
      const std::optional<glm::vec2> dot =
          centroid(plain, visible(*mosaic, i), white);
      const std::optional<glm::vec2> mark =
          centroid(marked, visible(*mosaic, i), red);
      ASSERT_TRUE(dot.has_value());
      ASSERT_TRUE(mark.has_value());
      EXPECT_NEAR(dot->x, expected.x, 0.75f);
      EXPECT_NEAR(dot->y, expected.y, 0.75f);
      EXPECT_NEAR(mark->x, dot->x, 0.75f);
      EXPECT_NEAR(mark->y, dot->y, 0.75f);
    }
    // Every dot drawn, and nothing else lit.
    EXPECT_EQ(count(plain, kWhole, white), count(plain, kWhole, lit));
  }
}

// Over a grey target, a set without camera 1 keeps its last picture while
// camera 0's changes, and the gutters, the cell past a smaller picture and the
// empty cell stay black. Paged a cell a page, the mosaic draws the very
// pixels it draws on one page: its backdrop is black between the pages and
// in the empty block, and cut at the mosaic's edges, for a wide mosaic and a
// tall one too.
TEST_F(VizRenderTest, KeepsAMissingCamerasLastPictureAndTheGutterBlack) {
  clear_.float32[0] = 0.5f;
  clear_.float32[1] = 0.5f;
  clear_.float32[2] = 0.5f;
  clear_.float32[3] = 1.0f;
  const struct {
    const char* name;
    std::vector<ImageSize> sizes;
    std::uint64_t one_cell;  // a page's texels for one cell a page
  } cases[] = {
      {"mixed", {{16, 12}, {8, 6}, {16, 12}}, 16 * 16},
      {"wide", {{16, 12}, {16, 12}}, 16 * 16},
      {"tall", {{2, 30}, {2, 30}, {2, 30}}, 16 * 32},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.name);
    MosaicLimits one_cell;
    one_cell.max_texels = c.one_cell;
    std::vector<Pixels> seconds;
    for (const MosaicLimits& limits : {MosaicLimits{}, one_cell}) {
      SCOPED_TRACE(limits.max_texels);
      std::unique_ptr<RigMosaic> mosaic = make_mosaic(c.sizes, limits);
      ASSERT_NE(mosaic, nullptr);
      ASSERT_EQ(mosaic->layout().pages.size(),
                seconds.empty() ? 1u : c.sizes.size());
      fit(*mosaic);
      ASSERT_GT(mosaic->view().scale(), 1.0f);  // magnified, nearest
      std::vector<Picture> first;
      for (std::size_t i = 0; i < c.sizes.size(); ++i) {
        const ImageSize& size = c.sizes[i];
        first.push_back(
            make_picture(size.width, size.height,
                         {static_cast<float>((size.width - 1) / 2),
                          static_cast<float>((size.height - 1) / 2 + i)},
                         160));
      }
      ASSERT_FALSE(render(*mosaic, all(first)).bytes.empty());
      Picture moved = make_picture(c.sizes[0].width, c.sizes[0].height,
                                   {static_cast<float>(c.sizes[0].width - 2),
                                    static_cast<float>(c.sizes[0].height - 2)},
                                   160);
      std::vector<Picture*> next = all(first);
      next[0] = &moved;
      next[1] = nullptr;
      seconds.push_back(render(*mosaic, next));
      const Pixels& second = seconds.back();
      ASSERT_FALSE(second.bytes.empty());

      for (std::size_t i = 0; i < c.sizes.size(); ++i) {
        SCOPED_TRACE(i);
        const glm::vec2 dot = i == 0 ? moved.dot : first[i].dot;
        const glm::vec2 expected = mosaic->view().image_to_target(
            mosaic->tile_region(i).origin + dot + 0.5f);
        const std::optional<glm::vec2> found =
            centroid(second, visible(*mosaic, i), bright);
        ASSERT_TRUE(found.has_value());
        EXPECT_NEAR(found->x, expected.x, 0.75f);
        EXPECT_NEAR(found->y, expected.y, 0.75f);
        EXPECT_EQ(mosaic->sets_since_picture(i), i == 1 ? 1u : 0u);
      }
      // The tiles themselves are lit -- their grey and their dots -- and the
      // grey target shows around the mosaic.
      const gfx::camera::ImageView2D& view = mosaic->view();
      for (std::size_t i = 0; i < c.sizes.size(); ++i) {
        const gfx::ui::OverlayRegion region = mosaic->tile_region(i);
        const Rect inside =
            target_rect(view, region.origin, region.origin + region.size, 1.0f);
        EXPECT_EQ(count(second, inside, lit),
                  static_cast<int>(inside.size.x * inside.size.y))
            << i;
      }
      EXPECT_GT(count(second, kWhole, grey), 0);
      if (std::string(c.name) != "mixed") continue;
      // Cells 32 texels apart: the gutters beside and below tile 0, the rest
      // of tile 1's cell, and the empty fourth cell.
      for (const Rect& black :
           {target_rect(view, {16.0f, 0.0f}, {32.0f, 12.0f}, 1.0f),
            target_rect(view, {0.0f, 12.0f}, {16.0f, 32.0f}, 1.0f),
            target_rect(view, {40.0f, 0.0f}, {48.0f, 12.0f}, 1.0f),
            target_rect(view, {32.0f, 6.0f}, {40.0f, 12.0f}, 1.0f),
            target_rect(view, {32.0f, 32.0f}, {48.0f, 44.0f}, 1.0f)}) {
        SCOPED_TRACE(::testing::Message()
                     << black.origin.x << ", " << black.origin.y);
        ASSERT_GT(black.size.x * black.size.y, 0.0f);
        EXPECT_EQ(count(second, black, lit), 0);
      }
    }
    ASSERT_EQ(seconds.size(), 2u);
    EXPECT_TRUE(seconds[0].bytes == seconds[1].bytes);
  }
}

// A set with no new picture records nothing -- no command buffer is touched
// -- and ages the pictures, and so does one whose only picture is left out;
// a set with a picture short or extra is refused, leaving the pictures' ages
// as they were and naming no picture left out.
TEST_F(VizRenderTest, RecordsNothingWithoutANewPicture) {
  std::unique_ptr<RigMosaic> mosaic = make_mosaic({{16, 12}, {16, 12}});
  ASSERT_NE(mosaic, nullptr);
  fit(*mosaic);
  ASSERT_TRUE(mosaic->record(VK_NULL_HANDLE, {nullptr, nullptr}).ok());
  EXPECT_EQ(mosaic->sets_since_picture(0), std::nullopt);

  std::vector<Picture> pictures;
  pictures.push_back(make_picture(16, 12, {3.0f, 2.0f}));
  pictures.push_back(make_picture(16, 12, {9.0f, 7.0f}));
  ASSERT_FALSE(render(*mosaic, all(pictures)).bytes.empty());
  ASSERT_TRUE(mosaic->record(VK_NULL_HANDLE, {nullptr, nullptr}).ok());
  EXPECT_TRUE(mosaic->refused().empty());
  EXPECT_EQ(mosaic->sets_since_picture(0), 1u);
  EXPECT_EQ(mosaic->sets_since_picture(1), 1u);
  Picture small = make_picture(8, 6, {1.0f, 1.0f});
  ASSERT_TRUE(mosaic->record(VK_NULL_HANDLE, {nullptr, &small.yuv}).ok());
  EXPECT_EQ(mosaic->refused().size(), 1u);
  EXPECT_EQ(mosaic->sets_since_picture(0), 2u);
  EXPECT_EQ(mosaic->sets_since_picture(1), 2u);

  const std::vector<const sensor::YuvImage*> short_set = {&pictures[0].yuv};
  const std::vector<const sensor::YuvImage*> extra_set = {
      &pictures[0].yuv, &pictures[1].yuv, &pictures[1].yuv};
  for (const auto* set : {&short_set, &extra_set}) {
    SCOPED_TRACE(set->size());
    const core::Status status = mosaic->record(VK_NULL_HANDLE, *set);
    EXPECT_EQ(status.domain(), Code::InvalidArgument) << status.message();
    EXPECT_TRUE(mosaic->refused().empty());
    EXPECT_EQ(mosaic->sets_since_picture(0), 2u);
    EXPECT_EQ(mosaic->sets_since_picture(1), 2u);
  }

  // The pictures stay drawn.
  const Pixels kept = render(*mosaic, {nullptr, nullptr});
  ASSERT_FALSE(kept.bytes.empty());
  EXPECT_TRUE(centroid(kept, visible(*mosaic, 0), white).has_value());
  EXPECT_TRUE(centroid(kept, visible(*mosaic, 1), white).has_value());
  EXPECT_EQ(mosaic->sets_since_picture(0), 3u);
}

// One camera's fault leaves the other live: a picture the mosaic cannot take
// -- another size, I420, chroma gfx cannot site, or no Y'CbCr matrix -- is
// left out and named, its tile keeping its last picture and ageing, while the
// other camera's new picture is drawn. Two pictures in different codings on
// one page are both left out, their tiles as they were.
TEST_F(VizRenderTest, LeavesOutOnlyThePicturesItCannotTake) {
  std::unique_ptr<RigMosaic> mosaic = make_mosaic({{16, 12}, {16, 12}});
  ASSERT_NE(mosaic, nullptr);
  fit(*mosaic);
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(16, 12, {3.0f, 2.0f}, 160));
  pictures.push_back(make_picture(16, 12, {9.0f, 7.0f}, 160));
  ASSERT_FALSE(render(*mosaic, all(pictures)).bytes.empty());

  Picture small = make_picture(8, 6, {1.0f, 1.0f}, 160);
  Picture i420 = make_picture(16, 12, {1.0f, 1.0f}, 160);
  i420.yuv.layout = sensor::YuvLayout::I420;
  Picture unsited = make_picture(16, 12, {1.0f, 1.0f}, 160);
  unsited.yuv.chroma_location = sensor::ChromaLocation::TopLeft;
  Picture no_matrix = make_picture(16, 12, {1.0f, 1.0f}, 160);
  no_matrix.yuv.kr = 0.0f;
  const struct {
    const char* name;
    Picture* picture;
    Code code;
  } refused[] = {
      {"another size", &small, Code::InvalidArgument},
      {"I420", &i420, Code::Unsupported},
      {"chroma gfx cannot site", &unsited, Code::Unsupported},
      {"no Y'CbCr matrix", &no_matrix, Code::InvalidArgument},
  };
  std::uint64_t age = 0;
  for (const auto& r : refused) {
    SCOPED_TRACE(r.name);
    ++age;
    Picture moved = make_picture(16, 12, {2.0f + age, 9.0f}, 160);
    const Pixels drawn = render(*mosaic, {&moved, r.picture});
    ASSERT_FALSE(drawn.bytes.empty());
    ASSERT_EQ(mosaic->refused().size(), 1u);
    EXPECT_EQ(mosaic->refused()[0].camera, 1u);
    EXPECT_EQ(mosaic->refused()[0].status.domain(), r.code)
        << mosaic->refused()[0].status.message();
    EXPECT_EQ(mosaic->sets_since_picture(0), 0u);
    EXPECT_EQ(mosaic->sets_since_picture(1), age);
    const glm::vec2 dots[] = {moved.dot, pictures[1].dot};
    for (std::size_t i = 0; i < 2; ++i) {
      const glm::vec2 expected = mosaic->view().image_to_target(
          mosaic->tile_region(i).origin + dots[i] + 0.5f);
      const std::optional<glm::vec2> dot =
          centroid(drawn, visible(*mosaic, i), bright);
      ASSERT_TRUE(dot.has_value()) << i;
      EXPECT_NEAR(dot->x, expected.x, 0.75f) << i;
      EXPECT_NEAR(dot->y, expected.y, 0.75f) << i;
    }
  }

  Picture last = make_picture(16, 12, {2.0f + age, 9.0f}, 160);
  Picture bt709 = make_picture(16, 12, {1.0f, 1.0f}, 160);
  bt709.yuv.kr = 0.2126f;
  bt709.yuv.kb = 0.0722f;
  const Pixels mixed = render(*mosaic, {&pictures[0], &bt709});
  ASSERT_FALSE(mixed.bytes.empty());
  ASSERT_EQ(mosaic->refused().size(), 2u);
  for (std::size_t i = 0; i < 2; ++i) {
    const TileRefusal& refusal = mosaic->refused()[i];
    EXPECT_EQ(refusal.camera, i);
    EXPECT_EQ(refusal.status.domain(), Code::InvalidArgument);
    EXPECT_NE(refusal.status.message().find("a page converts with one"),
              std::string::npos)
        << refusal.status.message();
  }
  EXPECT_EQ(mosaic->sets_since_picture(0), 1u);
  EXPECT_EQ(mosaic->sets_since_picture(1), age + 1);
  const glm::vec2 kept[] = {last.dot, pictures[1].dot};
  for (std::size_t i = 0; i < 2; ++i) {
    const glm::vec2 expected = mosaic->view().image_to_target(
        mosaic->tile_region(i).origin + kept[i] + 0.5f);
    const std::optional<glm::vec2> dot =
        centroid(mixed, visible(*mosaic, i), bright);
    ASSERT_TRUE(dot.has_value()) << i;
    EXPECT_NEAR(dot->x, expected.x, 0.75f) << i;
    EXPECT_NEAR(dot->y, expected.y, 0.75f) << i;
  }
}

// Each page converts in its own pictures' coding, so cameras of different
// codings on pages of their own are both drawn.
TEST_F(VizRenderTest, ConvertsEachPageInItsPicturesCoding) {
  MosaicLimits one_cell;
  one_cell.max_texels = 16 * 16;
  std::unique_ptr<RigMosaic> mosaic =
      make_mosaic({{16, 12}, {16, 12}}, one_cell);
  ASSERT_NE(mosaic, nullptr);
  ASSERT_EQ(mosaic->layout().pages.size(), 2u);
  fit(*mosaic);
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(16, 12, {3.0f, 2.0f}, 160));
  pictures.push_back(make_picture(16, 12, {9.0f, 7.0f}, 160));
  pictures[1].yuv.kr = 0.2126f;  // HEVC: BT.709, limited range, left-sited
  pictures[1].yuv.kb = 0.0722f;
  pictures[1].yuv.full_range = false;
  pictures[1].yuv.chroma_location = sensor::ChromaLocation::Left;
  const Pixels drawn = render(*mosaic, all(pictures));
  ASSERT_FALSE(drawn.bytes.empty());
  EXPECT_TRUE(mosaic->refused().empty());
  for (std::size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(mosaic->sets_since_picture(i), 0u);
    EXPECT_TRUE(centroid(drawn, visible(*mosaic, i), bright).has_value()) << i;
  }
}

// A picture calib takes but gfx refuses -- its luma image of the wrong format
// -- leaves out its page alone: the other page is recorded, the refused one
// keeps its picture, and the frame renders under validation. Recorded into a
// command buffer that never runs, then discarded, the mosaic records cleanly
// again.
TEST_F(VizRenderTest, LeavesOutAPageGfxRefuses) {
  MosaicLimits one_cell;
  one_cell.max_texels = 16 * 16;
  std::unique_ptr<RigMosaic> mosaic =
      make_mosaic({{16, 12}, {16, 12}}, one_cell);
  ASSERT_NE(mosaic, nullptr);
  ASSERT_EQ(mosaic->layout().pages.size(), 2u);
  fit(*mosaic);
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(16, 12, {3.0f, 2.0f}, 160));
  pictures.push_back(make_picture(16, 12, {9.0f, 7.0f}, 160));
  ASSERT_FALSE(render(*mosaic, all(pictures)).bytes.empty());

  Picture moved = make_picture(16, 12, {12.0f, 9.0f}, 160);
  Picture wrong = make_picture(16, 12, {1.0f, 1.0f}, 160);
  wrong.luma = upload(VK_FORMAT_R8G8_UNORM, {16, 12},
                      std::vector<std::uint8_t>(16 * 12 * 2, 160));
  wrong.yuv.image[0] = wrong.luma;
  const Pixels drawn = render(*mosaic, {&moved, &wrong});
  ASSERT_FALSE(drawn.bytes.empty());
  ASSERT_EQ(mosaic->refused().size(), 1u);
  EXPECT_EQ(mosaic->refused()[0].camera, 1u);
  EXPECT_EQ(mosaic->refused()[0].status.domain(), Code::InvalidArgument);
  EXPECT_NE(mosaic->refused()[0].status.message().find("page 1"),
            std::string::npos)
      << mosaic->refused()[0].status.message();
  EXPECT_EQ(mosaic->sets_since_picture(0), 0u);
  EXPECT_EQ(mosaic->sets_since_picture(1), 1u);
  const glm::vec2 dots[] = {moved.dot, pictures[1].dot};
  for (std::size_t i = 0; i < 2; ++i) {
    const glm::vec2 expected = mosaic->view().image_to_target(
        mosaic->tile_region(i).origin + dots[i] + 0.5f);
    const std::optional<glm::vec2> dot =
        centroid(drawn, visible(*mosaic, i), bright);
    ASSERT_TRUE(dot.has_value()) << i;
    EXPECT_NEAR(dot->x, expected.x, 0.75f) << i;
    EXPECT_NEAR(dot->y, expected.y, 0.75f) << i;
  }

  {
    core::Result<core::CommandPool> pool =
        core::CommandPool::create(device().handle(), device().queue_family());
    ASSERT_TRUE(pool.ok()) << pool.status().message();
    core::Result<core::CommandBuffer> dropped = pool.value().allocate_primary();
    ASSERT_TRUE(dropped.ok()) << dropped.status().message();
    ASSERT_TRUE(dropped.value()
                    .begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT)
                    .ok());
    ASSERT_TRUE(
        mosaic->record(dropped.value().handle(), {&pictures[0].yuv, &wrong.yuv})
            .ok());
    EXPECT_EQ(mosaic->refused().size(), 1u);
    // Never submitted: the frame failed.
  }
  mosaic->discard();
  const Pixels again = render(*mosaic, all(pictures));
  ASSERT_FALSE(again.bytes.empty());
  EXPECT_TRUE(mosaic->refused().empty());
  for (std::size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(mosaic->sets_since_picture(i), 0u);
    EXPECT_TRUE(centroid(again, visible(*mosaic, i), bright).has_value()) << i;
  }
}

// A new coding must not erase retained pictures until gfx accepts every
// write on its page, including cameras with no new picture. Retrying the old
// coding must also keep them; a valid coding change still blanks old tiles.
TEST_F(VizRenderTest, KeepsPicturesWhenGfxRefusesANewCoding) {
  for (const bool paged : {false, true}) {
    SCOPED_TRACE(paged);
    MosaicLimits limits;
    if (paged) limits.max_extent = {48, 16};
    std::unique_ptr<RigMosaic> mosaic =
        make_mosaic(std::vector<ImageSize>(3, ImageSize{16, 12}), limits);
    ASSERT_NE(mosaic, nullptr);
    ASSERT_EQ(mosaic->layout().pages.size(), paged ? 2u : 1u);
    ASSERT_EQ(mosaic->layout().tiles[1].page, 0u);
    fit(*mosaic);
    std::vector<Picture> pictures;
    pictures.push_back(make_picture(16, 12, {3.0f, 2.0f}, 64));
    pictures.push_back(make_picture(16, 12, {9.0f, 7.0f}, 128));
    pictures.push_back(make_picture(16, 12, {5.0f, 4.0f}, 160));
    const Pixels first = render(*mosaic, all(pictures));
    ASSERT_FALSE(first.bytes.empty());

    Picture changed = make_picture(16, 12, {12.0f, 9.0f}, 160);
    changed.yuv.kr = 0.2126f;
    changed.yuv.kb = 0.0722f;
    Picture wrong = changed;
    wrong.yuv.image[0] = wrong.chroma;  // R8G8 luma: only gfx refuses it.
    const Pixels refused = render(*mosaic, {&wrong, nullptr, nullptr});
    ASSERT_EQ(mosaic->refused().size(), 1u);
    EXPECT_EQ(mosaic->refused()[0].camera, 0u);
    EXPECT_EQ(mosaic->refused()[0].status.domain(), Code::InvalidArgument);
    EXPECT_NE(mosaic->refused()[0].status.message().find("wrong format"),
              std::string::npos);
    EXPECT_EQ(refused.bytes, first.bytes);
    for (std::size_t i = 0; i < pictures.size(); ++i) {
      EXPECT_EQ(mosaic->sets_since_picture(i), 1u) << i;
    }

    // A valid first write followed by an invalid one refuses the whole page.
    const Pixels batch = render(*mosaic, {&changed, &wrong, nullptr});
    ASSERT_EQ(mosaic->refused().size(), 2u);
    for (std::size_t i = 0; i < 2; ++i) {
      EXPECT_EQ(mosaic->refused()[i].camera, i);
      EXPECT_EQ(mosaic->refused()[i].status.domain(), Code::InvalidArgument);
    }
    EXPECT_EQ(batch.bytes, first.bytes);
    for (std::size_t i = 0; i < pictures.size(); ++i) {
      EXPECT_EQ(mosaic->sets_since_picture(i), 2u) << i;
    }

    const Pixels old_coding = render(*mosaic, {&pictures[0], nullptr, nullptr});
    EXPECT_TRUE(mosaic->refused().empty());
    EXPECT_EQ(old_coding.bytes, first.bytes);
    EXPECT_EQ(mosaic->sets_since_picture(0), 0u);
    EXPECT_EQ(mosaic->sets_since_picture(1), 3u);
    EXPECT_EQ(mosaic->sets_since_picture(2), 3u);

    const Pixels switched = render(*mosaic, {&changed, nullptr, nullptr});
    ASSERT_FALSE(switched.bytes.empty());
    EXPECT_TRUE(mosaic->refused().empty());
    EXPECT_GT(count(switched, visible(*mosaic, 0), lit), 0);
    EXPECT_EQ(count(switched, visible(*mosaic, 1), lit), 0);
    EXPECT_EQ(mosaic->sets_since_picture(0), 0u);
    EXPECT_EQ(mosaic->sets_since_picture(1), std::nullopt);
    if (paged) {
      EXPECT_GT(count(switched, visible(*mosaic, 2), lit), 0);
      EXPECT_EQ(mosaic->sets_since_picture(2), 4u);
    } else {
      EXPECT_EQ(count(switched, visible(*mosaic, 2), lit), 0);
      EXPECT_EQ(mosaic->sets_since_picture(2), std::nullopt);
    }
  }
}

// A record whose command buffer never runs, then discard: the mosaic shows
// nothing, and the next record -- of camera 1 alone -- clears the page
// first, under validation, so camera 0's tile is black until its next
// picture. Paged, a first record that never ran leaves the backdrop
// unwritten too.
TEST_F(VizRenderTest, RecordsCleanlyAfterADiscard) {
  std::unique_ptr<RigMosaic> mosaic = make_mosaic({{16, 12}, {16, 12}});
  ASSERT_NE(mosaic, nullptr);
  fit(*mosaic);
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(16, 12, {3.0f, 2.0f}, 160));
  pictures.push_back(make_picture(16, 12, {9.0f, 7.0f}, 160));
  ASSERT_FALSE(render(*mosaic, all(pictures)).bytes.empty());

  {
    core::Result<core::CommandPool> pool =
        core::CommandPool::create(device().handle(), device().queue_family());
    ASSERT_TRUE(pool.ok()) << pool.status().message();
    core::Result<core::CommandBuffer> dropped = pool.value().allocate_primary();
    ASSERT_TRUE(dropped.ok()) << dropped.status().message();
    ASSERT_TRUE(dropped.value()
                    .begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT)
                    .ok());
    ASSERT_TRUE(mosaic
                    ->record(dropped.value().handle(),
                             {&pictures[0].yuv, &pictures[1].yuv})
                    .ok());
    // Never submitted: the frame failed.
  }
  mosaic->discard();
  EXPECT_EQ(mosaic->sets_since_picture(0), std::nullopt);
  EXPECT_EQ(mosaic->sets_since_picture(1), std::nullopt);
  const Pixels blank = render(*mosaic, {nullptr, nullptr});
  ASSERT_FALSE(blank.bytes.empty());
  EXPECT_EQ(count(blank, kWhole, lit), 0);

  const Pixels one = render(*mosaic, {nullptr, &pictures[1]});
  ASSERT_FALSE(one.bytes.empty());
  EXPECT_EQ(count(one, visible(*mosaic, 0), lit), 0);
  EXPECT_GT(count(one, visible(*mosaic, 1), lit), 0);
  EXPECT_TRUE(centroid(one, visible(*mosaic, 1), bright).has_value());
  EXPECT_EQ(mosaic->sets_since_picture(0), std::nullopt);
  EXPECT_EQ(mosaic->sets_since_picture(1), 0u);

  MosaicLimits one_cell;
  one_cell.max_texels = 16 * 16;
  std::unique_ptr<RigMosaic> paged =
      make_mosaic({{16, 12}, {16, 12}}, one_cell);
  ASSERT_NE(paged, nullptr);
  ASSERT_EQ(paged->layout().pages.size(), 2u);
  fit(*paged);
  {
    core::Result<core::CommandPool> pool =
        core::CommandPool::create(device().handle(), device().queue_family());
    ASSERT_TRUE(pool.ok()) << pool.status().message();
    core::Result<core::CommandBuffer> dropped = pool.value().allocate_primary();
    ASSERT_TRUE(dropped.ok()) << dropped.status().message();
    ASSERT_TRUE(dropped.value()
                    .begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT)
                    .ok());
    ASSERT_TRUE(paged
                    ->record(dropped.value().handle(),
                             {&pictures[0].yuv, &pictures[1].yuv})
                    .ok());
  }
  paged->discard();
  const Pixels paged_blank = render(*paged, {nullptr, nullptr});
  ASSERT_FALSE(paged_blank.bytes.empty());
  EXPECT_EQ(count(paged_blank, kWhole, lit), 0);
  const Pixels paged_one = render(*paged, {nullptr, &pictures[1]});
  ASSERT_FALSE(paged_one.bytes.empty());
  EXPECT_EQ(count(paged_one, visible(*paged, 0), lit), 0);
  EXPECT_TRUE(centroid(paged_one, visible(*paged, 1), bright).has_value());
}

// A set in another coding than the page holds blanks the page's other tiles
// rather than convert them wrong, until their own pictures come; on that page
// alone, when the mosaic is paged.
TEST_F(VizRenderTest, BlanksTilesACodingChangeWouldConvertWrong) {
  const auto to_hevc = [](Picture& picture) {
    picture.yuv.kr = 0.2126f;
    picture.yuv.kb = 0.0722f;
    picture.yuv.full_range = false;
    picture.yuv.chroma_location = sensor::ChromaLocation::Left;
  };
  std::unique_ptr<RigMosaic> mosaic = make_mosaic({{16, 12}, {16, 12}});
  ASSERT_NE(mosaic, nullptr);
  fit(*mosaic);
  std::vector<Picture> mjpeg;
  mjpeg.push_back(make_picture(16, 12, {3.0f, 2.0f}, 160));
  mjpeg.push_back(make_picture(16, 12, {9.0f, 7.0f}, 160));
  ASSERT_FALSE(render(*mosaic, all(mjpeg)).bytes.empty());

  std::vector<Picture> hevc;
  hevc.push_back(make_picture(16, 12, {3.0f, 2.0f}, 160));
  hevc.push_back(make_picture(16, 12, {9.0f, 7.0f}, 160));
  for (Picture& picture : hevc) to_hevc(picture);
  const Pixels switched = render(*mosaic, {&hevc[0], nullptr});
  ASSERT_FALSE(switched.bytes.empty());
  EXPECT_GT(count(switched, visible(*mosaic, 0), lit), 0);
  EXPECT_EQ(count(switched, visible(*mosaic, 1), lit), 0);
  EXPECT_EQ(mosaic->sets_since_picture(0), 0u);
  EXPECT_EQ(mosaic->sets_since_picture(1), std::nullopt);

  const Pixels both = render(*mosaic, {nullptr, &hevc[1]});
  ASSERT_FALSE(both.bytes.empty());
  EXPECT_GT(count(both, visible(*mosaic, 0), lit), 0);
  EXPECT_GT(count(both, visible(*mosaic, 1), lit), 0);

  // Paged, cameras 0 and 1 on page 0 and camera 2 on page 1: camera 0's new
  // coding blanks camera 1 alone.
  MosaicLimits row;
  row.max_extent = {48, 16};
  std::unique_ptr<RigMosaic> paged =
      make_mosaic(std::vector<ImageSize>(3, ImageSize{16, 12}), row);
  ASSERT_NE(paged, nullptr);
  ASSERT_EQ(paged->layout().pages.size(), 2u);
  ASSERT_EQ(paged->layout().tiles[1].page, 0u);
  ASSERT_EQ(paged->layout().tiles[2].page, 1u);
  fit(*paged);
  mjpeg.push_back(make_picture(16, 12, {5.0f, 4.0f}, 160));
  ASSERT_FALSE(render(*paged, all(mjpeg)).bytes.empty());
  const Pixels one = render(*paged, {&hevc[0], nullptr, nullptr});
  ASSERT_FALSE(one.bytes.empty());
  EXPECT_GT(count(one, visible(*paged, 0), lit), 0);
  EXPECT_EQ(count(one, visible(*paged, 1), lit), 0);
  EXPECT_TRUE(centroid(one, visible(*paged, 2), bright).has_value());
  EXPECT_EQ(paged->sets_since_picture(0), 0u);
  EXPECT_EQ(paged->sets_since_picture(1), std::nullopt);
  EXPECT_EQ(paged->sets_since_picture(2), 1u);
}

// One coverage cell filled, at row 1, column 6 of the 8 x 6 grid over
// camera 1's tile, and one marker. The fill must cover that cell's texels of
// camera 1's tile -- a grid of 6 columns by 8 rows, or one over the whole
// mosaic, would put it elsewhere -- and the marker's outline must run round
// its quad, closed on all four sides; camera 0's tile stays unmarked.
TEST_F(VizRenderTest, MarksTheCoverageCellAndTheMarkerWhereTheyAre) {
  std::unique_ptr<RigMosaic> mosaic = make_mosaic({{64, 48}, {64, 48}});
  ASSERT_NE(mosaic, nullptr);
  fit(*mosaic);
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(64, 48, {60.0f, 44.0f}));  // out of the way
  pictures.push_back(make_picture(64, 48, {60.0f, 44.0f}));

  constexpr int kRow = 1;
  constexpr int kColumn = 6;
  quality::CameraSummary session;
  session.coverage[kRow * quality::kCoverageCols + kColumn] = 50;
  Detection found;
  DetectedMarker marker;
  marker.corners = {
      sensor::camera::Vec2d{8.0, 8.0}, {24.0, 8.0}, {24.0, 24.0}, {8.0, 24.0}};
  found.markers.push_back(marker);
  TileOverlay tile;
  tile.detection = &found;
  tile.session = &session;
  OverlayStyle style;
  style.coverage_color = IM_COL32(0, 0, 255, 0);
  style.grid_line = 0;  // the fill alone
  style.marker_color = IM_COL32(255, 255, 0, 255);
  const core::Result<TileMarks> marks = tile_marks(tile, {}, style);
  ASSERT_TRUE(marks.ok()) << marks.status().message();

  std::optional<gfx::ui::ImGuiOverlay> overlay = make_overlay();
  ASSERT_TRUE(overlay.has_value());
  overlay->new_frame();
  draw_tile_marks(mosaic->view(), mosaic->tile_region(1), marks.value());
  const Pixels marked = render(*mosaic, all(pictures), &*overlay);
  ASSERT_FALSE(marked.bytes.empty());
  const gfx::camera::ImageView2D& view = mosaic->view();
  const glm::vec2 origin = mosaic->tile_region(1).origin;
  EXPECT_EQ(origin, glm::vec2(80.0f, 0.0f));

  // The cell spans texels [c * w / 8, (c + 1) * w / 8) across and
  // [r * h / 6, (r + 1) * h / 6) down: its middle, in Vulkan's image
  // coordinates.
  const glm::vec2 cell_middle((kColumn + 0.5f) * 64.0f / quality::kCoverageCols,
                              (kRow + 0.5f) * 48.0f / quality::kCoverageRows);
  const glm::vec2 cell = view.image_to_target(origin + cell_middle);
  const std::optional<glm::vec2> fill =
      centroid(marked, visible(*mosaic, 1), blue);
  ASSERT_TRUE(fill.has_value());
  EXPECT_NEAR(fill->x, cell.x, 1.0f);
  EXPECT_NEAR(fill->y, cell.y, 1.0f);

  // OpenCV's corners at their texels' centres; the outline's middle is the
  // quad's only when all four sides are drawn.
  const glm::vec2 lo = view.image_to_target(origin + 8.5f);
  const glm::vec2 hi = view.image_to_target(origin + 24.5f);
  const std::optional<glm::vec2> outline =
      centroid(marked, visible(*mosaic, 1), yellow);
  ASSERT_TRUE(outline.has_value());
  EXPECT_NEAR(outline->x, 0.5f * (lo.x + hi.x), 1.0f);
  EXPECT_NEAR(outline->y, 0.5f * (lo.y + hi.y), 1.0f);
  const glm::vec2 mid = 0.5f * (lo + hi);
  for (const glm::vec2 side :
       {glm::vec2(mid.x, lo.y), glm::vec2(hi.x, mid.y), glm::vec2(mid.x, hi.y),
        glm::vec2(lo.x, mid.y)}) {
    const Rect near{side - 2.0f, glm::vec2(4.0f)};
    EXPECT_GT(count(marked, near, yellow), 0)
        << "no outline near (" << side.x << ", " << side.y << ")";
  }
  EXPECT_EQ(count(marked, visible(*mosaic, 0), blue), 0);
  EXPECT_EQ(count(marked, visible(*mosaic, 0), yellow), 0);
}

// A panned tile reaches beyond the framebuffer. Fractional display scales
// must keep every covered pixel at the far edge and none outside the tile.
TEST_F(VizRenderTest, KeepsMarksAtTheFramebufferEdgeAtFractionalScales) {
  std::unique_ptr<RigMosaic> mosaic = make_mosaic({{256, 128}});
  ASSERT_NE(mosaic, nullptr);
  TileMarks marks;
  marks.grid_columns = 1;
  marks.grid_rows = 1;
  marks.grid_fills = {IM_COL32(0, 0, 255, 255)};
  for (const float scale : {1.0f, 1.1f, 1.75f, 2.0f}) {
    SCOPED_TRACE(scale);
    std::optional<gfx::ui::ImGuiOverlay> overlay = make_overlay(scale);
    ASSERT_TRUE(overlay.has_value());
    fit(*mosaic);
    const float origin = scale == 1.75f ? 227.0f : 147.0f;
    mosaic->view().pan({origin, 0.0f});
    overlay->new_frame();
    draw_tile_marks(mosaic->view(), mosaic->tile_region(0), marks);
    const Pixels marked = render(*mosaic, {nullptr}, &*overlay);
    ASSERT_FALSE(marked.bytes.empty());
    const Rect inside{{origin, 0.0f}, {kTarget.x - origin, kTarget.y}};
    const int expected = static_cast<int>(inside.size.x * inside.size.y);
    EXPECT_EQ(count(marked, inside, blue), expected);
    EXPECT_EQ(count(marked, kWhole, blue), expected);
  }
}

// The title stays inside its tile's visible part, whatever the viewport and
// however the view pans and zooms: at the tile's top-left corner while that
// is in view, at the viewport's once the view zooms past it; drawn whole --
// text and background -- whenever the visible part holds it, and never a
// pixel outside it, into the gutter or the next tile.
TEST_F(VizRenderTest, KeepsTheTitleInsideItsTile) {
  std::unique_ptr<RigMosaic> mosaic = make_mosaic({{64, 48}, {64, 48}});
  ASSERT_NE(mosaic, nullptr);
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(64, 48, {60.0f, 44.0f}));
  pictures.push_back(make_picture(64, 48, {60.0f, 44.0f}));
  // A title short enough for the fitted tile: red on blue.
  TileMarks marks;
  marks.title = {"N", IM_COL32(255, 0, 0, 255), IM_COL32(0, 0, 255, 255)};
  std::optional<gfx::ui::ImGuiOverlay> overlay = make_overlay();
  ASSERT_TRUE(overlay.has_value());
  const auto draw_title = [&]() {
    overlay->new_frame();
    draw_tile_marks(mosaic->view(), mosaic->tile_region(1), marks);
    return render(*mosaic, all(pictures), &*overlay);
  };
  const auto title = [](const std::uint8_t* px) { return red(px) + blue(px); };

  for (const Rect area : {kWhole, Rect{{24.0f, 16.0f}, {216.0f, 96.0f}}}) {
    SCOPED_TRACE(::testing::Message()
                 << "viewport " << area.size.x << " x " << area.size.y);
    fit(*mosaic, area);
    const Pixels fitted = draw_title();
    ASSERT_FALSE(fitted.bytes.empty());
    const Rect inside = visible(*mosaic, 1);
    const int text_pixels = count(fitted, inside, red);
    const int background_pixels = count(fitted, inside, blue);
    ASSERT_GT(text_pixels, 0);
    ASSERT_GT(background_pixels, 0);
    EXPECT_EQ(count(fitted, kWhole, title), count(fitted, inside, title));
    const std::optional<glm::vec2> corner = top_left(fitted, inside, title);
    ASSERT_TRUE(corner.has_value());
    const glm::vec2 tile_corner =
        mosaic->view().image_to_target(mosaic->tile_region(1).origin);
    EXPECT_GE(corner->x, std::floor(tile_corner.x));
    EXPECT_LT(corner->x, tile_corner.x + 12.0f);
    EXPECT_GE(corner->y, std::floor(tile_corner.y));
    EXPECT_LT(corner->y, tile_corner.y + 12.0f);

    // Zoomed past the tile's corner: the title at the viewport's.
    const gfx::ui::OverlayRegion region = mosaic->tile_region(1);
    mosaic->view().zoom_about(
        mosaic->view().image_to_target(region.origin + 0.5f * region.size),
        4.0f);
    ASSERT_LT(mosaic->view().image_to_target(region.origin).x, area.origin.x);
    const Pixels zoomed = draw_title();
    ASSERT_FALSE(zoomed.bytes.empty());
    const Rect zoomed_inside = visible(*mosaic, 1);
    EXPECT_EQ(count(zoomed, zoomed_inside, red), text_pixels);
    EXPECT_EQ(count(zoomed, zoomed_inside, blue), background_pixels);
    EXPECT_EQ(count(zoomed, kWhole, title),
              count(zoomed, zoomed_inside, title));
    const std::optional<glm::vec2> pinned =
        top_left(zoomed, zoomed_inside, title);
    ASSERT_TRUE(pinned.has_value());
    EXPECT_LT(pinned->x, area.origin.x + 12.0f);
    EXPECT_LT(pinned->y, area.origin.y + 12.0f);

    // Panned toward the viewport's far corner: whole while 60 pixels of the
    // tile show, clipped inside it once 10 do, and gone once none do.
    for (const float shown : {60.0f, 10.0f}) {
      SCOPED_TRACE(::testing::Message() << shown << " pixels shown");
      fit(*mosaic, area);
      mosaic->view().pan(area.origin + area.size - shown -
                         mosaic->view().image_to_target(region.origin));
      const Pixels panned = draw_title();
      ASSERT_FALSE(panned.bytes.empty());
      const Rect panned_inside = visible(*mosaic, 1);
      if (shown >= 60.0f) {
        EXPECT_EQ(count(panned, panned_inside, red), text_pixels);
        EXPECT_EQ(count(panned, panned_inside, blue), background_pixels);
      }
      EXPECT_EQ(count(panned, kWhole, title),
                count(panned, panned_inside, title));
    }
    fit(*mosaic, area);
    mosaic->view().pan(-kTarget);
    const Pixels gone = draw_title();
    ASSERT_FALSE(gone.bytes.empty());
    EXPECT_EQ(count(gone, kWhole, title), 0);
  }

  // A viewport too narrow for the title: clipped to the tile.
  fit(*mosaic, {{24.0f, 16.0f}, {64.0f, 96.0f}});
  const Pixels narrow = draw_title();
  ASSERT_FALSE(narrow.bytes.empty());
  EXPECT_GT(count(narrow, visible(*mosaic, 1), title), 0);
  EXPECT_EQ(count(narrow, kWhole, title),
            count(narrow, visible(*mosaic, 1), title));
}

// Zoomed to 4 pixels a texel, the mosaic shows a white texel as a 4 x 4
// square at full intensity by default (Nearest), for checking marks against
// pixels; Linear spreads it over its neighbours.
TEST_F(VizRenderTest, MagnifiesEachTexelAsASquareByDefault) {
  std::unique_ptr<RigMosaic> mosaic = make_mosaic({{32, 24}});
  ASSERT_NE(mosaic, nullptr);
  fit(*mosaic, {{0.0f, 0.0f}, {128.0f, 128.0f}});  // the mosaic is 32 x 32
  ASSERT_EQ(mosaic->view().scale(), 4.0f);
  ASSERT_EQ(mosaic->view().origin(), glm::vec2(0.0f));
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(32, 24, {10.0f, 15.0f}));

  const Pixels nearest = render(*mosaic, all(pictures));
  ASSERT_FALSE(nearest.bytes.empty());
  const Rect texel{{40.0f, 60.0f}, {4.0f, 4.0f}};  // texel (10, 15)
  EXPECT_EQ(count(nearest, kWhole, lit), 16);
  EXPECT_EQ(count(nearest, texel, full), 16);

  const Pixels linear =
      render(*mosaic, {nullptr}, nullptr, gfx::pipelines::ImageFilter::Linear);
  ASSERT_FALSE(linear.bytes.empty());
  EXPECT_GT(count(linear, kWhole, lit), 16);
  EXPECT_EQ(count(linear, kWhole, full), 0);
}

// The tile under a pixel, across pages and mixed sizes, however the view
// pans and zooms: none in a gutter, past a smaller picture in its cell, in
// the empty cell, beyond the mosaic, or outside the viewport.
TEST_F(VizRenderTest, FindsTheTileUnderAPixel) {
  MosaicLimits two_by_two;
  two_by_two.max_extent = {48, 48};
  const std::vector<ImageSize> sizes = {
      {16, 12}, {8, 6}, {16, 12}, {16, 12}, {16, 12}};
  std::unique_ptr<RigMosaic> mosaic = make_mosaic(sizes, two_by_two);
  ASSERT_NE(mosaic, nullptr);
  ASSERT_EQ(mosaic->layout().pages.size(), 2u);
  const gfx::camera::ImageView2D& view = mosaic->view();
  const auto at = [&](glm::vec2 image_point) {
    return mosaic->tile_at(view.image_to_target(image_point));
  };
  EXPECT_EQ(mosaic->tile_at({100.0f, 50.0f}), std::nullopt);  // no viewport

  fit(*mosaic);
  for (std::size_t i = 0; i < sizes.size(); ++i) {
    const gfx::ui::OverlayRegion region = mosaic->tile_region(i);
    EXPECT_EQ(at(region.origin + 0.5f * region.size), i);
    EXPECT_EQ(at(region.origin + 0.05f), i);
    EXPECT_EQ(at(region.origin + region.size - 0.1f), i);
  }
  EXPECT_EQ(at({24.0f, 4.0f}), std::nullopt);   // the gutter
  EXPECT_EQ(at({4.0f, 20.0f}), std::nullopt);   // below tile 0
  EXPECT_EQ(at({44.0f, 3.0f}), std::nullopt);   // past tile 1's picture
  EXPECT_EQ(at({72.0f, 38.0f}), std::nullopt);  // the sixth, empty cell
  EXPECT_EQ(mosaic->tile_at({2.0f, 64.0f}), std::nullopt);  // letterboxed
  EXPECT_EQ(at({-1.0f, 4.0f}), std::nullopt);

  // Zoomed in a smaller viewport, tile 4 is below it, then panned into it.
  mosaic->view().set_viewport({16.0f, 8.0f}, {128.0f, 64.0f});
  mosaic->view().zoom_about({80.0f, 40.0f}, 2.0f);
  const gfx::ui::OverlayRegion fourth = mosaic->tile_region(4);
  const glm::vec2 centre = fourth.origin + 0.5f * fourth.size;
  const glm::vec2 below = view.image_to_target(centre);
  ASSERT_GT(below.y, 72.0f);
  ASSERT_LT(below.y, kTarget.y);
  EXPECT_EQ(mosaic->tile_at(below), std::nullopt);
  mosaic->view().pan({0.0f, -15.0f});
  const glm::vec2 inside = view.image_to_target(centre);
  ASSERT_GT(inside.y, 8.0f);
  ASSERT_LT(inside.y, 72.0f);
  EXPECT_EQ(at(centre), 4u);
}

// Two pictures as wide as the device's largest texture, under limits that
// would allow any page: the device's own limit pages them, one a page.
TEST_F(VizRenderTest, PagesWithinTheDevicesLimits) {
  core::Result<gfx::pipelines::ImagePipeline> pipeline =
      gfx::pipelines::ImagePipeline::create(device(), color_layout());
  ASSERT_TRUE(pipeline.ok()) << pipeline.status().message();
  const std::uint32_t widest = pipeline.value().max_texture_extent().width;
  ASSERT_EQ(widest % 16, 0u);
  MosaicLimits unbounded;
  unbounded.max_extent = {~0u, ~0u};
  unbounded.max_texels = ~std::uint64_t{0};
  std::unique_ptr<RigMosaic> mosaic =
      make_mosaic({{widest, 2}, {widest, 2}}, unbounded);
  ASSERT_NE(mosaic, nullptr);
  ASSERT_EQ(mosaic->layout().pages.size(), 2u);
  EXPECT_EQ(mosaic->layout().pages[1].extent.width, widest);
}

TEST_F(VizRenderTest, RefusesWhatAMosaicCannotTake) {
  EXPECT_EQ(RigMosaic::create(device(), allocator(), color_layout(), {})
                .status()
                .domain(),
            Code::InvalidArgument);
  for (const ImageSize bad : {ImageSize{64, 0}, ImageSize{65, 48}}) {
    EXPECT_EQ(RigMosaic::create(device(), allocator(), color_layout(),
                                {{64, 48}, bad})
                  .status()
                  .domain(),
              Code::InvalidArgument);
  }
  EXPECT_EQ(RigMosaic::create(device(), allocator(), gfx::RenderTargetLayout{},
                              {{64, 48}})
                .status()
                .domain(),
            Code::InvalidArgument);
  MosaicLimits tiny;
  tiny.max_texels = 64;
  EXPECT_EQ(
      RigMosaic::create(device(), allocator(), color_layout(), {{64, 48}}, tiny)
          .status()
          .domain(),
      Code::InvalidArgument);
}

// Every panel and the board editor, locked and not, drawn in one ImGui frame
// over the mosaic: they build, and render under validation.
TEST_F(VizRenderTest, DrawsEveryPanel) {
  std::unique_ptr<RigMosaic> mosaic = make_mosaic({{64, 48}});
  ASSERT_NE(mosaic, nullptr);
  fit(*mosaic);
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(64, 48, {40.0f, 20.0f}));

  CameraLive camera;
  camera.present = true;
  quality::SessionSummary summary;
  summary.cameras.resize(2);
  summary.cameras[0].linked = true;
  summary.cameras[1].linked = true;
  summary.cameras[1].link =
      quality::PreviewLink{0, 6, quality::LinkStrength::Fair};
  summary.covisibility = {{6, 6}, {6, 6}};
  const std::vector<std::string> ids = {"N", "G"};
  const SessionPanel session = session_panel(summary, ids).value();
  const std::vector<PanelLine> lines =
      camera_panel(camera, summary.cameras[0], legacy_femto_board()).value();
  core::StageMetrics stages;
  stages.add_cpu("detect", 9.0);
  const gfx::FrameMetrics metrics = stage_frame_metrics(stages, 25.0).value();
  BoardEditor open = BoardEditor::create(legacy_femto_board()).value();
  BoardEditor locked = BoardEditor::create(legacy_femto_board()).value();
  locked.lock();

  std::optional<gfx::ui::ImGuiOverlay> overlay = make_overlay();
  ASSERT_TRUE(overlay.has_value());
  overlay->new_frame();
  ImGui::Begin("camera");
  draw_panel_lines("camera", lines);
  ImGui::End();
  ImGui::Begin("session");
  draw_session_panel(session);
  ImGui::End();
  ImGui::Begin("board");
  EXPECT_TRUE(draw_board_editor(open).ok());
  EXPECT_TRUE(draw_board_editor(locked).ok());
  ImGui::End();
  gfx::ui::draw_metrics_panel(metrics, "Stages");
  const Pixels drawn = render(*mosaic, all(pictures), &*overlay);
  ASSERT_FALSE(drawn.bytes.empty());
  // The panels' windows cover the black background somewhere.
  EXPECT_TRUE(centroid(drawn, kWhole, [](const std::uint8_t* px) {
    return px[0] + px[1] + px[2] > 30 ? 1.0 : 0.0;
  }));
}

}  // namespace
}  // namespace volumetric_kit::calib::viz
