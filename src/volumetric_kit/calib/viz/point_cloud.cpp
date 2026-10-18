// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/viz/point_cloud.hpp"

#include <string>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"

namespace volumetric_kit::calib::viz {

namespace {

namespace pipelines = gfx::pipelines;

// Whether the renderer may read `buffer` where recon left it: as a storage
// buffer, and shared with the renderer's queue family when it is not recon's.
core::Status readable(const core::Buffer& buffer, const char* what,
                      const CloudStyle& style) {
  if ((buffer.usage() & VK_BUFFER_USAGE_STORAGE_BUFFER_BIT) == 0) {
    return core::Status::invalid_argument(
        std::string("cloud_draw: the frame's ") + what +
        " buffer has no STORAGE_BUFFER usage");
  }
  if (style.producer_family != style.renderer_family &&
      buffer.sharing_mode() != VK_SHARING_MODE_CONCURRENT) {
    return core::Status::invalid_argument(
        std::string("cloud_draw: the frame's ") + what +
        " is EXCLUSIVE to queue family " +
        std::to_string(style.producer_family) + ", so family " +
        std::to_string(style.renderer_family) +
        " may not read it; GpuFramePrepConfig must name both families for " +
        what);
  }
  return {};
}

template <class Params>
gfx::camera::PinholeCamera pinhole(const Params& params) noexcept {
  return {params.fx, params.fy, params.cx, params.cy, params.width,
          params.height};
}

}  // namespace

core::Result<pipelines::DepthCloudDraw> cloud_draw(
    const recon::sensor::DeviceFrame& frame,
    const recon::camera::Mat4d& color_to_world,
    const recon::camera::Mat4d& depth_to_color, const CloudStyle& style) {
  return compute::guarded(
      "cloud_draw", [&]() -> core::Result<pipelines::DepthCloudDraw> {
        if (frame.depth == nullptr) {
          return core::Status::invalid_argument(
              "cloud_draw: the frame has no depth");
        }
        VKC_TRY(readable(*frame.depth, "depth", style));
        const bool colour = style.mode != pipelines::CloudColor::Solid &&
                            frame.color != nullptr;
        if (colour) VKC_TRY(readable(*frame.color, "colour", style));
        VKC_TRY(recon::camera::check_rigid(color_to_world)
                    .with_context("cloud_draw: color_to_world"));
        VKC_TRY(recon::camera::check_rigid(depth_to_color)
                    .with_context("cloud_draw: depth_to_color"));

        pipelines::DepthCloudDraw draw;
        draw.depth.buffer = frame.depth.get();
        draw.depth.camera = pinhole(frame.depth_camera);
        draw.depth.to_world = glm::mat4(color_to_world * depth_to_color);
        draw.depth.min_depth = frame.depth_camera.min_depth;
        draw.depth.max_depth = frame.depth_camera.max_depth;
        if (colour) {
          draw.color.buffer = frame.color.get();
          draw.color.camera = pinhole(frame.color_camera);
          draw.color.to_world = glm::mat4(color_to_world);
          draw.color.alpha_is_coverage = true;  // recon's coverage byte
        }
        draw.mode = style.mode;
        draw.tint = style.tint;
        draw.stride = style.stride;
        draw.viewport = style.viewport;
        draw.view_proj = style.view_proj;
        draw.point_size = style.point_size;
        return draw;
      });
}

}  // namespace volumetric_kit::calib::viz
