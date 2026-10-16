// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file mosaic.hpp
/// @brief The live view's rig mosaic: each camera's decoded picture copied on
///        the GPU into its tile of one NV12 picture, converted once per frame
///        with a new picture, and drawn through one pan-and-zoom view.
///
/// No picture passes through the host: gfx's `ImagePipeline::record_write`
/// copies each decoder's planes into its camera's tile, and `record_convert`
/// converts the mosaic for display. Any number of cameras, of mixed even
/// sizes, take equal cells of one grid (@ref mosaic_layout); a mosaic too
/// large for one texture is paged over several, which its view shows as one.
/// Marks drawn over a tile's region (@ref RigMosaic::tile_region,
/// @ref overlay.hpp) land on the texels they name.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <glm/vec2.hpp>

#include "volumetric_kit/calib/viz/export.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/camera/image_view_2d.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/pipelines/image_pipeline.hpp"
#include "volumetric_kit/gfx/ui/image_overlay.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/sensor/yuv_image.hpp"

namespace volumetric_kit::core {
class Allocator;
class Device;
}  // namespace volumetric_kit::core

namespace volumetric_kit::calib::viz {

/// The fewest texels between neighbouring cells of the rig's mosaic.
inline constexpr std::uint32_t kMosaicGutter = 16;

/// @brief How large one page of a mosaic -- one texture -- may be.
///
/// @code
/// MosaicLimits limits;
/// limits.max_texels = std::uint64_t{1} << 24;  // 16 Mi texels a page
/// @endcode
struct MosaicLimits {
  /// The largest page, in texels: by default 16384 x 16384, the common
  /// `maxImageDimension2D`. @ref RigMosaic::create takes the smaller of this
  /// and the device's own (gfx's `ImagePipeline::max_texture_extent`).
  VkExtent2D max_extent{16384, 16384};
  /// The most texels one page holds, its memory budget: by default 2^26, about
  /// 460 MB of gfx's NV12 texture (its `ImageTexture` gives the bytes a texel
  /// takes).
  std::uint64_t max_texels = std::uint64_t{1} << 26;
};

/// @brief Where one camera's picture sits in a mosaic.
///
/// @code
/// const MosaicTile& tile = layout.tiles[1];  // 2 x 2 at 4K: origin (3856, 0)
/// @endcode
struct MosaicTile {
  /// Its top-left texel in the mosaic: in its view's image coordinates.
  VkOffset2D origin{};
  /// Its picture's size, in texels.
  VkExtent2D extent{};
  /// The page that holds it: an index into @ref MosaicLayout::pages.
  std::size_t page = 0;
  /// Its top-left texel in that page's texture, where its picture is written.
  VkOffset2D at{};
};

/// @brief One page of a mosaic: a texture holding a block of its cells.
///
/// @code
/// const MosaicPage& page = layout.pages[layout.tiles[i].page];
/// @endcode
struct MosaicPage {
  VkOffset2D origin{};  ///< Its top-left texel in the mosaic.
  VkExtent2D extent{};  ///< Its texture's size: multiples of 16.
};

/// @brief A mosaic's layout: each camera's picture at the top-left of a cell of
///        one grid, row-major from the top left, the cells paged over
///        textures.
///
/// The grid is as square as it can be, never taller than wide: `columns` is
/// the ceiling of the square root of the cameras. A cell is the largest
/// picture's size, and the cells are @ref pitch apart: a cell and the gutter,
/// rounded up to a multiple of 16. So every tile's origin, and the extent of
/// each page and of the mosaic, are multiples of 16 too. A page is a block of
/// whole cells, so a tile's place in the mosaic is the same however it is
/// paged.
///
/// @code
/// VKC_ASSIGN(const MosaicLayout layout, mosaic_layout(sizes));
/// // Four 3840 x 2160 cameras: 2 x 2 cells 3856 x 2176 apart, origins (0, 0),
/// // (3856, 0), (0, 2176) and (3856, 2176), one page of 7696 x 4336.
/// @endcode
struct MosaicLayout {
  std::uint32_t columns = 0;  ///< Cells across.
  std::uint32_t rows = 0;     ///< Cells down.
  VkExtent2D cell{};          ///< The largest picture's width and height.
  VkExtent2D pitch{};         ///< From a cell's origin to the next one's.
  VkExtent2D extent{};        ///< The whole mosaic's size: its view's image.
  std::vector<MosaicPage> pages;  ///< Its pages, row-major by block.
  std::vector<MosaicTile> tiles;  ///< Each camera's tile, in the rig's order.
};

/// @brief Lay out a mosaic of @p sizes, paged within @p limits.
///
/// One page holds the whole mosaic when it fits @p limits. Otherwise each page
/// holds the same block of cells: as many columns and rows as fit
/// `max_extent`, less a column or a row from its longer side until it fits
/// `max_texels`. A block that holds no tile has no page.
/// @param sizes   Each camera's picture size, in the rig's order: at least
///                one, each non-zero and even, as NV12's chroma needs.
/// @param gutter  The fewest texels between neighbouring cells.
/// @param limits  How large a page may be.
/// @return The layout; `InvalidArgument` for no picture, a size that is zero
///         or odd, a cell larger than a page may be, or a mosaic over 2^24
///         texels across or down (as far as its view's floats hold every
///         texel); or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const MosaicLayout layout,
///            mosaic_layout({{3840, 2160}, {3840, 2160}, {640, 576}}));
/// @endcode
VC_VIZ_API core::Result<MosaicLayout> mosaic_layout(
    const std::vector<recon::camera::ImageSize>& sizes,
    std::uint32_t gutter = kMosaicGutter, const MosaicLimits& limits = {});

/// @brief A decoder's picture as gfx's write of its tile: its two NV12 image
///        planes, read in place, copied to @p at.
///
/// The write borrows the picture's images, so the picture -- the frame that
/// holds it -- must outlive the write's execution on the GPU, not merely its
/// recording. The images must be readable from the renderer's queue family
/// (recon's `color_queue_families`).
/// @param picture   The picture, as `RgbdFrame::color` holds it.
/// @param expected  The tile's size (@ref MosaicTile::extent).
/// @param at        The tile's place in its page (@ref MosaicTile::at).
/// @return The write; `Unsupported` for an I420 picture, or planes in a buffer
///         rather than images; `InvalidArgument` for a picture missing its
///         luma or chroma image, or whose size is zero or not @p expected; or
///         `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const gfx::pipelines::ImageWrite write,
///            tile_write(frame.color, tile.extent, tile.at));
/// @endcode
// TODO: planes in a buffer (NVDEC and nvJPEG on Linux), which need their
// queue family's ownership taken over first, and I420, once gfx maps it.
VC_VIZ_API core::Result<gfx::pipelines::ImageWrite> tile_write(
    const recon::sensor::YuvImage& picture, VkExtent2D expected, VkOffset2D at);

/// @brief How pictures written together become display colour: the Y'CbCr
///        matrix, range and chroma siting they share.
///
/// gfx converts a page whole, with one coding, so the pictures written into
/// one page at once must share it.
/// @param pictures  The pictures; a null entry is none.
/// @return Their conversion; `Unsupported` for a chroma location gfx cannot
///         site (any but `Left` and `Center`); `InvalidArgument` for no
///         picture, matrix weights that are not a Y'CbCr matrix's
///         (`kr > 0`, `kb > 0`, `kr + kb < 1`), or two pictures whose matrix,
///         range or siting differ; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const gfx::pipelines::ImageConversion conversion,
///            tile_conversion(pictures));
/// @endcode
VC_VIZ_API core::Result<gfx::pipelines::ImageConversion> tile_conversion(
    const std::vector<const recon::sensor::YuvImage*>& pictures);

/// @brief A camera's new picture that @ref RigMosaic::record left out, and
///        why.
///
/// @code
/// for (const TileRefusal& refusal : mosaic->refused()) {
///   show_error(refusal.camera, refusal.status.message());
/// }
/// @endcode
struct TileRefusal {
  std::size_t camera = 0;  ///< The camera: its tile's index.
  core::Status status;     ///< Why its picture was left out.
};

/// @brief The rig's mosaic: one NV12 texture per page, written in tiles from
///        each camera's decoded picture and converted on the GPU, and one view
///        of it all.
///
/// Records and draws go into the caller's command buffers on the renderer's
/// graphics queue: @ref record before the frame's rendering scope, @ref draw
/// inside it. A camera with no new picture keeps its last; texels no picture
/// covers -- the gutter, and any empty cell -- are black, paged or not. The
/// view keeps its pan and zoom as its viewport changes. Not thread-safe.
///
/// @warning The device the allocator allocates on must outlive the mosaic, and
///          the GPU must be done with every frame that recorded or drew it
///          before it is destroyed: retire it through gfx's `RetireQueue`, or
///          after `core::Device::wait_idle`.
///
/// @code
/// VKC_ASSIGN(std::unique_ptr<RigMosaic> mosaic,
///            RigMosaic::create(device, allocator, swapchain.layout(), sizes));
/// mosaic->view().set_viewport({0.0f, 0.0f}, framebuffer);  // and on resize
/// // Each frame with a new set, before the rendering scope; the set is held
/// // in gfx's RetireQueue until the frame retires. `pictures` is the caller's
/// // own vector, reused from frame to frame:
/// pictures.assign(set.frames.size(), nullptr);
/// for (std::size_t i = 0; i < pictures.size(); ++i) {
///   if (set.frames[i] && set.frames[i]->has_color()) {
///     pictures[i] = &set.frames[i]->color;
///   }
/// }
/// VKC_TRY(mosaic->record(cmd, pictures));
/// // ... and each of mosaic->refused() shown on its camera's panel.
/// target.begin(cmd, begin_info);
/// mosaic->draw(cmd, extent);
/// // ... each camera's marks, then ImGui:
/// // draw_tile_marks(mosaic->view(), mosaic->tile_region(i), marks[i]);
/// target.end(cmd);
/// // After the frame, if it may not have run:
/// if (!end.ok() && !windowing::swapchain_stale(end)) mosaic->discard();
/// @endcode
class VC_VIZ_API RigMosaic {
 public:
  /// @brief Build the mosaic for the rig's cameras.
  /// @param device     The renderer's device: it enabled gfx's
  ///                   `device_requirements`. It must outlive the mosaic.
  /// @param allocator  Allocates the pages' textures, device-only.
  /// @param layout     The signature of the target the mosaic draws into.
  /// @param pictures   Each camera's picture size, in the rig's order: its
  ///                   colour model's size (@ref mosaic_layout).
  /// @param limits     How large a page may be, within the device's limits.
  /// @return The mosaic, showing nothing until its first @ref record, its view
  ///         fitted to an empty viewport; what @ref mosaic_layout refuses;
  ///         gfx's `ImagePipeline` and `ImageTexture` failures (a layout the
  ///         device cannot draw); or `OutOfMemory`.
  static core::Result<std::unique_ptr<RigMosaic>> create(
      const core::Device& device, core::Allocator& allocator,
      const gfx::RenderTargetLayout& layout,
      const std::vector<recon::camera::ImageSize>& pictures,
      const MosaicLimits& limits = {});

  /// @brief Destroy the textures and the pipeline.
  /// @pre The GPU is done with every frame that recorded or drew the mosaic.
  ~RigMosaic();
  RigMosaic(const RigMosaic&) = delete;
  RigMosaic& operator=(const RigMosaic&) = delete;
  RigMosaic(RigMosaic&&) = delete;
  RigMosaic& operator=(RigMosaic&&) = delete;

  /// @return How many tiles there are: one per camera.
  std::size_t size() const noexcept { return layout_.tiles.size(); }
  /// @return Where each tile and page sits.
  const MosaicLayout& layout() const noexcept { return layout_; }

  /// @brief Record the copy of each new picture into its tile, then one
  ///        convert of each page written.
  ///
  /// A set with no new picture records nothing. Each page is recorded whole
  /// or not at all. One camera's fault leaves the others live: a picture
  /// @ref tile_write or @ref tile_conversion refuses is left out, and so are
  /// a page's pictures when they mix codings or gfx refuses their write;
  /// @ref refused names each, and its tile keeps what it showed. A page whose
  /// other tiles hold another coding than its new pictures' is blanked only
  /// after gfx accepts their writes, so they show black rather than
  /// mis-converted. Refused writes keep the page's pictures and their ages
  /// advance as for a set with no new picture.
  /// @param cmd       A command buffer in the recording state, outside a
  ///                  rendering scope, for the renderer's graphics queue.
  /// @param pictures  Each camera's new picture, in the rig's order; a null
  ///                  entry is none, and keeps its tile's last. Each must
  ///                  outlive the record's execution (@ref tile_write).
  /// @return OK once recorded, whatever it left out; `InvalidArgument` for an
  ///         entry per camera missing or extra, with nothing recorded; or
  ///         `OutOfMemory`, after which the mosaic may have forgotten its
  ///         pictures, as @ref discard does, so it is valid whether or not
  ///         @p cmd runs.
  core::Status record(
      VkCommandBuffer cmd,
      const std::vector<const recon::sensor::YuvImage*>& pictures);

  /// @return The pictures the last @ref record left out, by camera; empty
  ///         after one that returned an error.
  const std::vector<TileRefusal>& refused() const noexcept { return refused_; }

  /// @brief Forget every tile's picture, after a command buffer that recorded
  ///        to the mosaic may not have run.
  ///
  /// Each page then shows nothing until a @ref record writes to it, which first
  /// clears it black. Call it when `windowing::FrameLoop::end_frame` fails and
  /// the swapchain is not stale, as for gfx's `ImageTexture::discard`, or when
  /// a command buffer @ref record recorded into is dropped.
  void discard() noexcept;

  /// @brief Record the mosaic's draw: each page with a picture where the view
  ///        puts it, over a paged mosaic's black backdrop.
  /// @param cmd      A command buffer in the recording state, inside a
  ///                 rendering scope of a target of the layout @ref create
  ///                 took.
  /// @param extent   The target's size, in pixels.
  /// @param magnify  How the mosaic zoomed past one texel per pixel is
  ///                 magnified: `Nearest` (the default) shows each texel as a
  ///                 square, for checking marks against pixels.
  void draw(VkCommandBuffer cmd, VkExtent2D extent,
            gfx::pipelines::ImageFilter magnify =
                gfx::pipelines::ImageFilter::Nearest) noexcept;

  /// @brief The view of the whole mosaic: where it sits in its viewport, for
  ///        panning, zooming and marks.
  /// @return The view.
  gfx::camera::ImageView2D& view() noexcept { return view_; }
  /// @copydoc view
  const gfx::camera::ImageView2D& view() const noexcept { return view_; }

  /// @brief Tile @p index's region of the mosaic, for marks over it.
  /// @param index  The tile.
  /// @return Its origin and its picture's size, in the view's image
  ///         coordinates.
  /// @pre @p index is below @ref size.
  gfx::ui::OverlayRegion tile_region(std::size_t index) const;

  /// @brief The tile a target pixel shows: for picking a camera.
  /// @param target_point  In target pixels.
  /// @return The tile whose picture is under it; empty outside the viewport,
  ///         in the gutter, or in a cell past its tile's picture.
  std::optional<std::size_t> tile_at(glm::vec2 target_point) const noexcept;

  /// @brief How stale tile @p index's picture is.
  /// @param index  The tile.
  /// @return The sets @ref record took since the tile's picture: 0 for the
  ///         latest set's; empty while it shows none (never written, or
  ///         forgotten by @ref discard or a change of coding).
  /// @pre @p index is below @ref size.
  std::optional<std::uint64_t> sets_since_picture(std::size_t index) const;

 private:
  RigMosaic() = default;

  core::Status record_pages(
      VkCommandBuffer cmd,
      const std::vector<const recon::sensor::YuvImage*>& pictures,
      bool& started);
  // Leave out the pictures of page `page`'s writes, for `status`.
  void refuse_page(std::size_t page, const core::Status& status);
  // Forget page `page`'s picture and its tiles', as discard does.
  void forget_page(std::size_t page) noexcept;

  MosaicLayout layout_;
  gfx::pipelines::ImagePipeline pipeline_;
  std::vector<gfx::pipelines::ImageTexture> textures_;  // one per page
  // A paged mosaic's backdrop: one black texel, drawn under the pages across
  // the mosaic, so the gutters between pages and any empty block are black;
  // written from black_, a byte of 0.
  gfx::pipelines::ImageTexture backdrop_;
  core::Buffer black_;
  // The coding each page's planes hold, since its last discard.
  std::vector<std::optional<gfx::pipelines::ImageConversion>> codings_;
  std::vector<std::optional<std::uint64_t>> since_;  // per tile
  std::vector<TileRefusal> refused_;                 // by the last record
  gfx::camera::ImageView2D view_;
  // Reused by record: its writes grouped by page, from page_first_[p], and
  // the tile of each; each page's pictures, by tile, and their conversion.
  std::vector<std::size_t> page_first_;
  std::vector<std::size_t> page_count_;
  std::vector<gfx::pipelines::ImageWrite> writes_;
  std::vector<std::size_t> write_tile_;
  std::vector<const recon::sensor::YuvImage*> page_pictures_;
  std::vector<gfx::pipelines::ImageConversion> page_conversion_;
  std::vector<gfx::pipelines::ImageDraw> draws_;  // reused by draw
};

}  // namespace volumetric_kit::calib::viz
