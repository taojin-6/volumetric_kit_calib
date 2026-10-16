// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tiles.hpp
/// @brief The live view's tiles: each camera's decoded picture drawn in place
///        from the device, laid out in a grid, panned and zoomed by a view of
///        its own.
///
/// No picture passes through the host: a tile's update copies the decoder's
/// planes into the tile's texture on the GPU (gfx's `ImagePipeline`), which
/// then draws it. Each tile's `ImageView2D` maps its image to target pixels,
/// so marks drawn through it (@ref overlay.hpp) land on the texels they name.

#include <cstddef>
#include <memory>
#include <vector>

#include <glm/vec2.hpp>

#include "volumetric_kit/calib/viz/export.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/camera/image_view_2d.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/pipelines/image_pipeline.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/sensor/yuv_image.hpp"

namespace volumetric_kit::core {
class Allocator;
class Device;
}  // namespace volumetric_kit::core

namespace volumetric_kit::calib::viz {

/// @brief A rectangle in target pixels.
///
/// @code
/// const TileRect area{{0.0f, 0.0f}, {1280.0f, 720.0f}};
/// @endcode
struct TileRect {
  glm::vec2 origin{0.0f, 0.0f};  ///< The top-left corner.
  glm::vec2 size{0.0f, 0.0f};    ///< The width and height.
};

/// @brief How many tiles across and down a grid of tiles has.
///
/// @code
/// const TileGrid grid = tile_grid(4);  // 2 x 2
/// @endcode
struct TileGrid {
  int columns = 0;  ///< Tiles across.
  int rows = 0;     ///< Tiles down.
};

/// @brief The grid @p count tiles fill: as square as it can be, never taller
///        than wide.
/// @param count  The tiles.
/// @return The smallest square grid that holds them, less its last row when
///         one fewer suffices: 1 x 1 for 1, 2 x 1 for 2, 2 x 2 for 3 and 4,
///         3 x 2 for 5 and 6; 0 x 0 for none.
///
/// @code
/// const TileGrid grid = tile_grid(cameras.size());
/// @endcode
VC_VIZ_API TileGrid tile_grid(std::size_t count) noexcept;

/// @brief Where tile @p index of @p count sits in @p area: the cells of
///        @ref tile_grid, row-major from the top left, @p gap pixels apart.
///
/// Edges are rounded to whole pixels, so neighbours stay exactly @p gap apart
/// when the area and the gap are whole.
/// @param area   The rectangle the tiles fill.
/// @param count  The tiles.
/// @param index  The tile.
/// @param gap    Pixels between neighbouring tiles; negative or not finite
///               counts as 0.
/// @return The tile's rectangle; an empty one at @p area's origin for an
///         index past the last, or an area that is empty or not finite.
///
/// @code
/// const TileRect rect = tile_rect(area, 4, 3, 4.0f);  // the bottom right
/// @endcode
VC_VIZ_API TileRect tile_rect(const TileRect& area, std::size_t count,
                              std::size_t index, float gap = 0.0f) noexcept;

/// @brief A decoder's picture as gfx's update of a tile: its two NV12 image
///        planes, read in place, with the picture's matrix, range and chroma
///        siting.
///
/// The update borrows the picture's images, so the picture -- the frame that
/// holds it -- must outlive the update's execution on the GPU, not merely its
/// recording. The images must be readable from the renderer's queue family
/// (recon's `color_queue_families`).
/// @param picture  The picture, as `RgbdFrame::color` holds it.
/// @return The update; `Unsupported` for an I420 picture, planes in a buffer
///         rather than images, or a chroma location gfx cannot site (any but
///         `Left` and `Center`); `InvalidArgument` for a picture of zero size
///         or missing a plane image; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const gfx::pipelines::ImageUpdate update,
///            tile_update(frame.color));
/// @endcode
// TODO: planes in a buffer (NVDEC and nvJPEG on Linux), which need their
// queue family's ownership taken over first, and I420, once gfx maps it.
VC_VIZ_API core::Result<gfx::pipelines::ImageUpdate> tile_update(
    const recon::sensor::YuvImage& picture);

/// @brief Every camera's tile: its texture, converted on the GPU from the
///        camera's decoded picture, its place in the grid, and its view.
///
/// Updates and draws go into the caller's command buffers on the renderer's
/// graphics queue. The views keep their pan and zoom when the area changes.
/// Not thread-safe.
///
/// @warning The device the allocator allocates on must outlive the tiles, and
///          the GPU must be done with every frame that updated or drew them
///          before they are destroyed: retire them through gfx's
///          `RetireQueue`, or after `core::Device::wait_idle`.
///
/// @code
/// VKC_ASSIGN(std::unique_ptr<Tiles> tiles,
///            Tiles::create(device, allocator, swapchain.layout(), sizes));
/// tiles->set_area({{0.0f, 0.0f}, framebuffer}, 4.0f);  // and on each resize
/// // per frame, before the rendering scope:
/// for (std::size_t i = 0; i < set.frames.size(); ++i) {
///   if (!set.frames[i]) continue;
///   VKC_TRY(tiles->record_update(cmd, i, set.frames[i]->color));
/// }
/// target.begin(cmd, begin_info);
/// tiles->submit(cmd, extent);
/// // ... overlays drawn through tiles->view(i), then ImGui
/// target.end(cmd);
/// @endcode
class VC_VIZ_API Tiles {
 public:
  /// @brief Build a tile for each camera.
  /// @param device     The renderer's device: it enabled gfx's
  ///                   `device_requirements`. It must outlive the tiles.
  /// @param allocator  Allocates the textures, device-only.
  /// @param layout     The signature of the target the tiles draw into.
  /// @param pictures   Each camera's picture size, in the rig's order: its
  ///                   colour model's size; at least one, none zero.
  /// @return The tiles, laid out in an empty area until @ref set_area;
  ///         `InvalidArgument` for no picture or a zero size; gfx's
  ///         `ImagePipeline` and `ImageTexture` failures (a layout or size
  ///         the device cannot draw); or `OutOfMemory`.
  static core::Result<std::unique_ptr<Tiles>> create(
      const core::Device& device, core::Allocator& allocator,
      const gfx::RenderTargetLayout& layout,
      const std::vector<recon::camera::ImageSize>& pictures);

  ~Tiles();
  Tiles(const Tiles&) = delete;
  Tiles& operator=(const Tiles&) = delete;
  Tiles(Tiles&&) = delete;
  Tiles& operator=(Tiles&&) = delete;

  /// @return How many tiles there are: one per camera.
  std::size_t size() const noexcept { return textures_.size(); }

  /// @brief Lay the tiles out in @p area (@ref tile_rect), each view keeping
  ///        its zoom and the image point at its centre.
  /// @param area  Where the tiles go, in target pixels.
  /// @param gap   Pixels between neighbouring tiles.
  void set_area(const TileRect& area, float gap = 0.0f) noexcept;

  /// @param index  The tile; below @ref size.
  /// @return Its rectangle, as @ref set_area laid it out.
  /// @pre @p index is below @ref size.
  TileRect rect(std::size_t index) const;

  /// @brief Tile @p index's view: where its image sits in its rectangle, for
  ///        panning, zooming and marks.
  /// @param index  The tile.
  /// @return The view.
  /// @pre @p index is below @ref size.
  gfx::camera::ImageView2D& view(std::size_t index);
  /// @copydoc view
  const gfx::camera::ImageView2D& view(std::size_t index) const;

  /// @brief Record tile @p index's update from its camera's decoded picture:
  ///        copied and converted on the GPU (@ref tile_update).
  /// @param cmd      A command buffer in the recording state, outside a
  ///                 rendering scope, for the renderer's graphics queue.
  /// @param index    The tile.
  /// @param picture  The camera's picture; it must outlive the update's
  ///                 execution (@ref tile_update).
  /// @return OK once recorded; `InvalidArgument` for an index past the last
  ///         or a picture of another size than the tile's; what
  ///         @ref tile_update or gfx's `record_update` refuses; or
  ///         `OutOfMemory`. On an error nothing is recorded, and the tile
  ///         keeps its last picture.
  core::Status record_update(VkCommandBuffer cmd, std::size_t index,
                             const recon::sensor::YuvImage& picture);

  /// @brief Record every tile with a picture into its rectangle.
  /// @param cmd      A command buffer in the recording state, inside a
  ///                 rendering scope of a target of the layout @ref create
  ///                 took.
  /// @param extent   The target's size, in pixels.
  /// @param magnify  How a tile zoomed past one texel per pixel is
  ///                 magnified: `Nearest` (the default) shows each texel as a
  ///                 square, for checking marks against pixels.
  void submit(VkCommandBuffer cmd, VkExtent2D extent,
              gfx::pipelines::ImageFilter magnify =
                  gfx::pipelines::ImageFilter::Nearest) noexcept;

 private:
  Tiles() = default;

  gfx::pipelines::ImagePipeline pipeline_;
  std::vector<gfx::pipelines::ImageTexture> textures_;
  std::vector<gfx::camera::ImageView2D> views_;
  std::vector<TileRect> rects_;
  std::vector<gfx::pipelines::ImageDraw> draws_;  // reused by submit
};

}  // namespace volumetric_kit::calib::viz
