// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file point_cloud.hpp
/// @brief The 3D view's points: one camera's prepared depth as gfx's draw,
///        read in place, at the rig's current poses.
///
/// recon's `GpuFramePrep` leaves each camera's undistorted depth and colour in
/// device buffers (`DeviceFrame`), and gfx's `DepthCloudPipeline` draws a
/// point per depth pixel straight from them, with no copy. calib composes the
/// two: the poses are the rig's at draw time, not the frame's, so a frozen or
/// captured cloud follows a new solve; and the buffers are checked readable
/// from the renderer's queue family before anything is drawn from them.

#include <cstdint>

#include <glm/mat4x4.hpp>

#include "volumetric_kit/calib/viz/export.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/pipelines/depth_cloud_pipeline.hpp"
#include "volumetric_kit/gfx/pipelines/primitive_pipeline.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"

namespace volumetric_kit::calib::viz {

/// @brief How one camera's points are drawn, and where.
///
/// @code
/// CloudStyle style;
/// style.mode = gfx::pipelines::CloudColor::TintedLuma;
/// style.tint = gfx::pipelines::pack_rgba8(255, 160, 0);
/// style.viewport = {{0, 0}, extent};
/// style.view_proj = scene.view_proj(style.viewport);
/// style.producer_family = recon_device.queue_family();
/// style.renderer_family = app.device().queue_family();
/// @endcode
struct CloudStyle {
  /// How the points are coloured (gfx's `CloudColor`).
  gfx::pipelines::CloudColor mode = gfx::pipelines::CloudColor::TintedLuma;
  /// The camera's tint, an sRGB-encoded `pack_rgba8` word: every point's in
  /// `Solid`, and the luma's scale in `TintedLuma`.
  std::uint32_t tint = gfx::pipelines::pack_rgba8(255, 255, 255);
  /// Every stride-th depth pixel along each axis; at least 1.
  std::uint32_t stride = 1;
  /// A point's width in pixels, as gfx clamps it to the device's.
  float point_size = 2.0f;
  /// The rectangle drawn into, in target pixels.
  VkRect2D viewport{};
  /// World to Vulkan clip space.
  glm::mat4 view_proj{1.0f};
  /// The queue family that wrote the frame's buffers: recon's.
  std::uint32_t producer_family = 0;
  /// The queue family that draws them: the renderer's.
  std::uint32_t renderer_family = 0;
};

/// @brief One camera's prepared frame as gfx's draw of its points, at the
///        rig's poses now.
///
/// The frame's own poses are ignored: the depth camera is placed at
/// `color_to_world * depth_to_color` and the colour camera at
/// @p color_to_world, so the same frame redraws wherever a new solve puts
/// its camera. Both cameras keep their pinhole intrinsics, with recon's
/// integer pixel centres, which gfx's are. Colour is read only when the
/// style's mode uses it and the frame has it; a frame prepared without colour
/// draws its tint.
/// @param frame           The frame, as `GpuFramePrep` handed it out; its
///                        buffers must outlive the draw's execution.
/// @param color_to_world  The colour camera's pose in the rig's world.
/// @param depth_to_color  The depth camera's frame to the colour camera's
///                        (recon's `SensorInfo::depth_to_color`).
/// @param style           How and where it is drawn, and by which queue
///                        family.
/// @return The draw; or `InvalidArgument`, its message starting
///         `cloud_draw: `, for a frame without depth, a buffer drawn from
///         without `STORAGE_BUFFER` usage, a buffer that is EXCLUSIVE while
///         the producer's and the renderer's queue families differ (the
///         renderer may not read it: recon's
///         `GpuFramePrepConfig::depth_queue_families` and
///         `color_queue_families` must name both), or a pose that is not
///         rigid (recon's `check_rigid`).
///
/// @code
/// VKC_ASSIGN(const gfx::pipelines::DepthCloudDraw draw,
///            cloud_draw(*snapshot.frames[i], color_to_world[i],
///                       info[i].depth_to_color, style));
/// @endcode
VC_VIZ_API core::Result<gfx::pipelines::DepthCloudDraw> cloud_draw(
    const recon::sensor::DeviceFrame& frame,
    const recon::camera::Mat4d& color_to_world,
    const recon::camera::Mat4d& depth_to_color, const CloudStyle& style);

}  // namespace volumetric_kit::calib::viz
