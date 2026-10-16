// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The tiles and their marks drawn offscreen on a device, as the live view
// draws them: synthetic NV12 pictures in device images -- as a decoder leaves
// them, read in place -- each with a bright dot, laid out, zoomed, and
// marked through each tile's view on a high-DPI ImGui frame. The dot each
// tile draws must sit under the mark its overlay draws; the coverage cell,
// the marker's outline and the title must land where their marks say. Then
// every panel and the board editor drawn in one frame. Runs under the
// validation layer when the environment asks (VKC_TEST_VALIDATION); skips
// without a Vulkan device.

#include <gtest/gtest.h>

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
#include "volumetric_kit/calib/viz/overlay.hpp"
#include "volumetric_kit/calib/viz/panels.hpp"
#include "volumetric_kit/calib/viz/tiles.hpp"
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

namespace sensor = recon::sensor;
using Code = core::Status::Code;

constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;
constexpr VkExtent2D kExtent{256, 128};
// The ImGui frame is half the target's size at twice the scale: a high-DPI
// display, whose window coordinates are half its framebuffer pixels.
constexpr float kFramebufferScale = 2.0f;

gfx::RenderTargetLayout color_layout() {
  gfx::RenderTargetLayout layout;
  layout.color_formats[0] = kFormat;
  layout.color_count = 1;
  return layout;
}

// An NV12 picture, black but for one white texel, in device images.
struct Picture {
  std::shared_ptr<core::Image> luma;
  std::shared_ptr<core::Image> chroma;
  glm::vec2 dot{0.0f, 0.0f};  // the white texel, in OpenCV's pixels
  sensor::YuvImage yuv;
};

// The rendered target's pixels.
struct Pixels {
  std::vector<std::uint8_t> bytes;
  const std::uint8_t* at(std::uint32_t x, std::uint32_t y) const {
    return &bytes[(static_cast<std::size_t>(y) * kExtent.width + x) * 4];
  }
};

// The weighted centroid, in target pixels (centres at + 0.5), of the pixels
// in `rect` whose weight is positive.
template <class Weight>
std::optional<glm::vec2> centroid(const Pixels& pixels, const TileRect& rect,
                                  Weight weight) {
  double sum = 0.0;
  double sx = 0.0;
  double sy = 0.0;
  const auto x0 = static_cast<std::uint32_t>(rect.origin.x);
  const auto y0 = static_cast<std::uint32_t>(rect.origin.y);
  for (std::uint32_t y = y0; y < y0 + static_cast<std::uint32_t>(rect.size.y);
       ++y) {
    for (std::uint32_t x = x0; x < x0 + static_cast<std::uint32_t>(rect.size.x);
         ++x) {
      const double w = weight(pixels.at(x, y));
      if (w <= 0.0) continue;
      sum += w;
      sx += w * (x + 0.5);
      sy += w * (y + 0.5);
    }
  }
  if (sum <= 0.0) return std::nullopt;
  return glm::vec2(static_cast<float>(sx / sum), static_cast<float>(sy / sum));
}

// The top-left corner, in target pixels, of the pixels in `rect` whose
// weight is positive.
template <class Weight>
std::optional<glm::vec2> top_left(const Pixels& pixels, const TileRect& rect,
                                  Weight weight) {
  std::optional<glm::vec2> corner;
  const auto x0 = static_cast<std::uint32_t>(rect.origin.x);
  const auto y0 = static_cast<std::uint32_t>(rect.origin.y);
  for (std::uint32_t y = y0; y < y0 + static_cast<std::uint32_t>(rect.size.y);
       ++y) {
    for (std::uint32_t x = x0; x < x0 + static_cast<std::uint32_t>(rect.size.x);
         ++x) {
      if (weight(pixels.at(x, y)) <= 0.0) continue;
      const glm::vec2 p(static_cast<float>(x), static_cast<float>(y));
      corner = corner ? glm::min(*corner, p) : p;
    }
  }
  return corner;
}

// How many pixels in `rect` have a positive weight.
template <class Weight>
int count(const Pixels& pixels, const TileRect& rect, Weight weight) {
  int n = 0;
  for (std::uint32_t y = static_cast<std::uint32_t>(rect.origin.y);
       y < static_cast<std::uint32_t>(rect.origin.y + rect.size.y); ++y) {
    for (std::uint32_t x = static_cast<std::uint32_t>(rect.origin.x);
         x < static_cast<std::uint32_t>(rect.origin.x + rect.size.x); ++x) {
      if (weight(pixels.at(x, y)) > 0.0) ++n;
    }
  }
  return n;
}

double white(const std::uint8_t* px) {
  return px[0] > 128 && px[1] > 128 && px[2] > 128 ? px[1] : 0.0;
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

class VizRenderTest : public core::test::VulkanDeviceTest {
 protected:
  core::DeviceRequirements requirements() const override {
    return gfx::device_requirements();
  }

  // A `width` x `height` picture with its white texel at `dot`, its planes
  // uploaded and left in GENERAL, as recon's decoders leave them.
  Picture make_picture(std::uint32_t width, std::uint32_t height,
                       glm::vec2 dot) {
    Picture picture;
    picture.dot = dot;
    std::vector<std::uint8_t> luma(static_cast<std::size_t>(width) * height, 0);
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

  std::optional<gfx::ui::ImGuiOverlay> make_overlay() {
    gfx::ui::ImGuiOverlayConfig config;
    config.layout = color_layout();
    core::Result<gfx::ui::ImGuiOverlay> overlay =
        gfx::ui::ImGuiOverlay::create(device(), instance().handle(), config);
    EXPECT_TRUE(overlay.ok()) << overlay.status().message();
    if (!overlay.ok()) return std::nullopt;
    ImGui::SetCurrentContext(overlay.value().context());
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(kExtent.width / kFramebufferScale,
                            kExtent.height / kFramebufferScale);
    io.DisplayFramebufferScale = ImVec2(kFramebufferScale, kFramebufferScale);
    return std::move(overlay).value();
  }

  // Updates every tile from its picture, draws the tiles (magnified by
  // `magnify`, or submit's default), then the ImGui frame `overlay` built
  // (when given), into a fresh target; its pixels.
  Pixels render(Tiles& tiles, std::vector<Picture>& pictures,
                gfx::ui::ImGuiOverlay* overlay,
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

    for (std::size_t i = 0; i < pictures.size(); ++i) {
      to_general(raw, *pictures[i].luma);
      to_general(raw, *pictures[i].chroma);
      const core::Status updated = tiles.record_update(raw, i, pictures[i].yuv);
      EXPECT_TRUE(updated.ok()) << updated.message();
    }
    target.value().prepare(raw);
    const gfx::RenderTarget rt = target.value().target();
    rt.begin(raw, {});  // cleared to black
    if (magnify) {
      tiles.submit(raw, kExtent, *magnify);
    } else {
      tiles.submit(raw, kExtent);
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

// Two tiles side by side; the first zoomed about a point off its dot, so
// the dot moves. Each white dot the tiles draw must lie under the red mark
// its overlay draws, and both where the tile's view puts the texel's centre:
// OpenCV's (x, y) at the view's (x + 0.5, y + 0.5), divided by the
// framebuffer scale for ImGui.
TEST_F(VizRenderTest, EachTilesDotSitsUnderItsMark) {
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(64, 48, {40.0f, 20.0f}));
  pictures.push_back(make_picture(32, 24, {10.0f, 15.0f}));
  core::Result<std::unique_ptr<Tiles>> made = Tiles::create(
      device(), allocator(), color_layout(), {{64, 48}, {32, 24}});
  ASSERT_TRUE(made.ok()) << made.status().message();
  Tiles& tiles = *made.value();
  tiles.set_area({{0.0f, 0.0f}, {256.0f, 128.0f}});
  tiles.view(0).zoom_about({40.0f, 60.0f}, 2.0f);
  ASSERT_GT(tiles.view(0).scale(), 3.5f);  // a half texel is 2 pixels or more

  const Pixels plain = render(tiles, pictures, nullptr);
  ASSERT_FALSE(plain.bytes.empty());

  std::optional<gfx::ui::ImGuiOverlay> overlay = make_overlay();
  ASSERT_TRUE(overlay.has_value());
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
    draw_tile_marks(tiles.view(i), marks.value());
  }
  const Pixels marked = render(tiles, pictures, &*overlay);
  ASSERT_FALSE(marked.bytes.empty());

  for (std::size_t i = 0; i < pictures.size(); ++i) {
    SCOPED_TRACE(i);
    const glm::vec2 expected =
        tiles.view(i).image_to_target(pictures[i].dot + 0.5f);
    const std::optional<glm::vec2> dot = centroid(plain, tiles.rect(i), white);
    const std::optional<glm::vec2> mark = centroid(marked, tiles.rect(i), red);
    ASSERT_TRUE(dot.has_value());
    ASSERT_TRUE(mark.has_value());
    EXPECT_NEAR(dot->x, expected.x, 0.75f);
    EXPECT_NEAR(dot->y, expected.y, 0.75f);
    EXPECT_NEAR(mark->x, dot->x, 0.75f);
    EXPECT_NEAR(mark->y, dot->y, 0.75f);
  }
}

// One coverage cell filled, at row 1, column 6 of the 8 x 6 grid, and one
// marker. The fill must cover that cell's texels -- a grid of 6 columns by 8
// rows would put it elsewhere -- and the marker's outline must run round its
// quad, closed on all four sides.
TEST_F(VizRenderTest, MarksTheCoverageCellAndTheMarkerWhereTheyAre) {
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(64, 48, {60.0f, 44.0f}));  // out of the way
  core::Result<std::unique_ptr<Tiles>> made =
      Tiles::create(device(), allocator(), color_layout(), {{64, 48}});
  ASSERT_TRUE(made.ok()) << made.status().message();
  Tiles& tiles = *made.value();
  tiles.set_area({{0.0f, 0.0f}, {256.0f, 128.0f}});

  constexpr int kRow = 1;
  constexpr int kColumn = 6;
  quality::CameraSummary session;
  session.coverage[kRow * quality::kCoverageCols + kColumn] = 50;
  Detection found;
  DetectedMarker marker;
  marker.corners = {
      recon::camera::Vec2d{8.0, 8.0}, {24.0, 8.0}, {24.0, 24.0}, {8.0, 24.0}};
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
  draw_tile_marks(tiles.view(0), marks.value());
  const Pixels marked = render(tiles, pictures, &*overlay);
  ASSERT_FALSE(marked.bytes.empty());
  const gfx::camera::ImageView2D& view = tiles.view(0);

  // The cell spans texels [c * w / 8, (c + 1) * w / 8) across and
  // [r * h / 6, (r + 1) * h / 6) down: its middle, in Vulkan's image
  // coordinates.
  const glm::vec2 cell_middle((kColumn + 0.5f) * 64.0f / quality::kCoverageCols,
                              (kRow + 0.5f) * 48.0f / quality::kCoverageRows);
  const glm::vec2 cell = view.image_to_target(cell_middle);
  const std::optional<glm::vec2> fill = centroid(marked, tiles.rect(0), blue);
  ASSERT_TRUE(fill.has_value());
  EXPECT_NEAR(fill->x, cell.x, 1.0f);
  EXPECT_NEAR(fill->y, cell.y, 1.0f);

  // OpenCV's corners at their texels' centres; the outline's middle is the
  // quad's only when all four sides are drawn.
  const glm::vec2 lo = view.image_to_target(glm::vec2(8.5f));
  const glm::vec2 hi = view.image_to_target(glm::vec2(24.5f));
  const std::optional<glm::vec2> outline =
      centroid(marked, tiles.rect(0), yellow);
  ASSERT_TRUE(outline.has_value());
  EXPECT_NEAR(outline->x, 0.5f * (lo.x + hi.x), 1.0f);
  EXPECT_NEAR(outline->y, 0.5f * (lo.y + hi.y), 1.0f);
  const glm::vec2 mid = 0.5f * (lo + hi);
  for (const glm::vec2 side :
       {glm::vec2(mid.x, lo.y), glm::vec2(hi.x, mid.y), glm::vec2(mid.x, hi.y),
        glm::vec2(lo.x, mid.y)}) {
    const TileRect near{side - 2.0f, glm::vec2(4.0f)};
    EXPECT_GT(count(marked, near, yellow), 0)
        << "no outline near (" << side.x << ", " << side.y << ")";
  }
}

// The title stays in the tile: at the image's top-left corner while that
// is in view, at the tile's once the view zooms past it.
TEST_F(VizRenderTest, KeepsTheTitleInTheTile) {
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(64, 48, {60.0f, 44.0f}));
  core::Result<std::unique_ptr<Tiles>> made =
      Tiles::create(device(), allocator(), color_layout(), {{64, 48}});
  ASSERT_TRUE(made.ok()) << made.status().message();
  Tiles& tiles = *made.value();
  tiles.set_area({{0.0f, 0.0f}, {256.0f, 128.0f}});
  TileOverlay tile;
  tile.id = "N";
  tile.check.verdict = ViewVerdict::Collinear;  // a red title
  const core::Result<TileMarks> marks = tile_marks(tile);
  ASSERT_TRUE(marks.ok()) << marks.status().message();
  std::optional<gfx::ui::ImGuiOverlay> overlay = make_overlay();
  ASSERT_TRUE(overlay.has_value());
  const auto title = [&]() -> std::optional<glm::vec2> {
    overlay->new_frame();
    draw_tile_marks(tiles.view(0), marks.value());
    const Pixels drawn = render(tiles, pictures, &*overlay);
    if (drawn.bytes.empty()) return std::nullopt;
    return top_left(drawn, tiles.rect(0), red);
  };

  // Fitted, the image is letterboxed: its corner is inside the tile.
  const glm::vec2 corner = tiles.view(0).image_to_target({0.0f, 0.0f});
  ASSERT_GT(corner.x, 32.0f);
  const std::optional<glm::vec2> fitted = title();
  ASSERT_TRUE(fitted.has_value());
  EXPECT_GE(fitted->x, corner.x);
  EXPECT_LT(fitted->x, corner.x + 20.0f);
  EXPECT_LT(fitted->y, 20.0f);

  tiles.view(0).zoom_about({128.0f, 64.0f}, 4.0f);  // H4's 400 %
  ASSERT_LT(tiles.view(0).image_to_target({0.0f, 0.0f}).x, -100.0f);
  const std::optional<glm::vec2> zoomed = title();
  ASSERT_TRUE(zoomed.has_value());
  EXPECT_LT(zoomed->x, 20.0f);
  EXPECT_LT(zoomed->y, 20.0f);
}

// Zoomed to 4 pixels a texel, a tile shows its white texel as a 4 x 4
// square at full intensity by default (Nearest), for checking marks against
// pixels; Linear spreads it over its neighbours.
TEST_F(VizRenderTest, MagnifiesEachTexelAsASquareByDefault) {
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(32, 24, {10.0f, 15.0f}));
  core::Result<std::unique_ptr<Tiles>> made =
      Tiles::create(device(), allocator(), color_layout(), {{32, 24}});
  ASSERT_TRUE(made.ok()) << made.status().message();
  Tiles& tiles = *made.value();
  tiles.set_area({{0.0f, 0.0f}, {128.0f, 96.0f}});
  ASSERT_EQ(tiles.view(0).scale(), 4.0f);
  ASSERT_EQ(tiles.view(0).origin(), glm::vec2(0.0f));

  const Pixels nearest = render(tiles, pictures, nullptr);
  ASSERT_FALSE(nearest.bytes.empty());
  const TileRect texel{{40.0f, 60.0f}, {4.0f, 4.0f}};  // texel (10, 15)
  EXPECT_EQ(count(nearest, tiles.rect(0), lit), 16);
  EXPECT_EQ(count(nearest, texel, full), 16);

  const Pixels linear =
      render(tiles, pictures, nullptr, gfx::pipelines::ImageFilter::Linear);
  ASSERT_FALSE(linear.bytes.empty());
  EXPECT_GT(count(linear, tiles.rect(0), lit), 16);
  EXPECT_EQ(count(linear, tiles.rect(0), full), 0);
}

// An area beyond the target, even past int's range, is drawn clipped to the
// target, the picture where the tile's view puts it.
TEST_F(VizRenderTest, DrawsAnAreaBeyondTheTarget) {
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(64, 48, {40.0f, 20.0f}));
  core::Result<std::unique_ptr<Tiles>> made =
      Tiles::create(device(), allocator(), color_layout(), {{64, 48}});
  ASSERT_TRUE(made.ok()) << made.status().message();
  Tiles& tiles = *made.value();
  tiles.set_area({{-3e9f, -3e9f}, {9e9f, 9e9f}});
  EXPECT_FALSE(render(tiles, pictures, nullptr).bytes.empty());

  tiles.set_area({{-64.0f, -32.0f}, {384.0f, 192.0f}});
  const Pixels drawn = render(tiles, pictures, nullptr);
  ASSERT_FALSE(drawn.bytes.empty());
  const glm::vec2 expected =
      tiles.view(0).image_to_target(pictures[0].dot + 0.5f);
  const std::optional<glm::vec2> dot =
      centroid(drawn, {{0.0f, 0.0f}, {256.0f, 128.0f}}, white);
  ASSERT_TRUE(dot.has_value());
  EXPECT_NEAR(dot->x, expected.x, 0.75f);
  EXPECT_NEAR(dot->y, expected.y, 0.75f);
}

// Laid out again, a tile keeps its zoom and the image point at its centre;
// before any area, nothing is drawn.
TEST_F(VizRenderTest, ATileKeepsItsZoomWhenTheAreaChanges) {
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(64, 48, {40.0f, 20.0f}));
  core::Result<std::unique_ptr<Tiles>> made =
      Tiles::create(device(), allocator(), color_layout(), {{64, 48}});
  ASSERT_TRUE(made.ok()) << made.status().message();
  Tiles& tiles = *made.value();
  ASSERT_EQ(tiles.size(), 1u);
  EXPECT_EQ(tiles.view(0).scale(), 0.0f);
  const Pixels before = render(tiles, pictures, nullptr);
  ASSERT_FALSE(before.bytes.empty());
  EXPECT_FALSE(centroid(before, {{0.0f, 0.0f}, {256.0f, 128.0f}}, white));

  tiles.set_area({{0.0f, 0.0f}, {128.0f, 128.0f}});
  tiles.view(0).zoom_about({30.0f, 50.0f}, 1.5f);
  const float zoom = tiles.view(0).zoom();
  const glm::vec2 centre = tiles.view(0).center();
  tiles.set_area({{16.0f, 8.0f}, {200.0f, 100.0f}}, 4.0f);
  EXPECT_EQ(tiles.view(0).zoom(), zoom);
  EXPECT_EQ(tiles.view(0).center(), centre);
  EXPECT_EQ(tiles.rect(0).origin, glm::vec2(16.0f, 8.0f));
  EXPECT_EQ(tiles.view(0).viewport_size(), glm::vec2(200.0f, 100.0f));
}

TEST_F(VizRenderTest, RefusesWhatATileCannotTake) {
  EXPECT_EQ(Tiles::create(device(), allocator(), color_layout(), {})
                .status()
                .domain(),
            Code::InvalidArgument);
  EXPECT_EQ(Tiles::create(device(), allocator(), color_layout(), {{64, 0}})
                .status()
                .domain(),
            Code::InvalidArgument);
  EXPECT_EQ(Tiles::create(device(), allocator(), gfx::RenderTargetLayout{},
                          {{64, 48}})
                .status()
                .domain(),
            Code::InvalidArgument);

  core::Result<std::unique_ptr<Tiles>> made =
      Tiles::create(device(), allocator(), color_layout(), {{32, 24}});
  ASSERT_TRUE(made.ok()) << made.status().message();
  Picture picture = make_picture(64, 48, {1.0f, 1.0f});
  core::Result<core::CommandPool> pool =
      core::CommandPool::create(device().handle(), device().queue_family());
  ASSERT_TRUE(pool.ok());
  core::Result<core::CommandBuffer> cmd = pool.value().allocate_primary();
  ASSERT_TRUE(cmd.ok());
  ASSERT_TRUE(
      cmd.value().begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT).ok());
  const VkCommandBuffer raw = cmd.value().handle();
  to_general(raw, *picture.luma);
  to_general(raw, *picture.chroma);
  EXPECT_EQ(made.value()->record_update(raw, 1, picture.yuv).domain(),
            Code::InvalidArgument);
  // 64 x 48 for a 32 x 24 tile: gfx would copy its corner.
  EXPECT_EQ(made.value()->record_update(raw, 0, picture.yuv).domain(),
            Code::InvalidArgument);
  picture.yuv.width = 32;
  picture.yuv.height = 24;
  picture.yuv.layout = sensor::YuvLayout::I420;
  EXPECT_EQ(made.value()->record_update(raw, 0, picture.yuv).domain(),
            Code::Unsupported);
  EXPECT_TRUE(cmd.value().end().ok());
}

// Every panel and the board editor, locked and not, drawn in one ImGui frame
// over the tiles: they build, and render under validation.
TEST_F(VizRenderTest, DrawsEveryPanel) {
  std::vector<Picture> pictures;
  pictures.push_back(make_picture(64, 48, {40.0f, 20.0f}));
  core::Result<std::unique_ptr<Tiles>> made =
      Tiles::create(device(), allocator(), color_layout(), {{64, 48}});
  ASSERT_TRUE(made.ok()) << made.status().message();
  made.value()->set_area({{0.0f, 0.0f}, {256.0f, 128.0f}});

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
  const Pixels drawn = render(*made.value(), pictures, &*overlay);
  ASSERT_FALSE(drawn.bytes.empty());
  // The panels' windows cover the black background somewhere.
  EXPECT_TRUE(centroid(drawn, {{0.0f, 0.0f}, {256.0f, 128.0f}},
                       [](const std::uint8_t* px) {
                         return px[0] + px[1] + px[2] > 30 ? 1.0 : 0.0;
                       }));
}

}  // namespace
}  // namespace volumetric_kit::calib::viz
