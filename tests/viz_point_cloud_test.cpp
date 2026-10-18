// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// One camera's prepared frame as gfx's draw of its points: every intrinsic,
// size, depth bound and pose lands in its own field, exactly, at the poses
// given and not the frame's own; colour only where the mode reads it and the
// frame has it; a buffer without storage usage, or EXCLUSIVE while the
// producer's and the renderer's queue families differ, is refused, as is a
// style that leaves either family unset; across two real families, depth,
// or colour the mode reads, EXCLUSIVE is refused and CONCURRENT is not.
// Then each shown, placed
// camera's draw in its tint. On real buffers, through the core's Vulkan test
// fixtures (they skip without a device; the cross-family case also with one
// family).

#include "volumetric_kit/calib/viz/point_cloud.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

#include "volumetric_kit/calib/viz/scene_view.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"

namespace volumetric_kit::calib::viz {
namespace {

namespace camera = recon::camera;
namespace pipelines = gfx::pipelines;
namespace sensor = recon::sensor;
using camera::Mat4d;
using camera::Vec3d;
using Code = core::Status::Code;

constexpr VkBufferUsageFlags kStorage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;

Mat4d pose(double degrees, Vec3d axis, Vec3d translation) {
  return glm::translate(Mat4d(1.0), translation) *
         glm::rotate(Mat4d(1.0), glm::radians(degrees), axis);
}

// Every intrinsic distinct, as the Femto Mega's depth and 4K colour cameras
// are; the frame's own poses a translation the draw must not use.
sensor::DeviceFrame frame_of(std::shared_ptr<const core::Buffer> depth,
                             std::shared_ptr<const core::Buffer> color) {
  sensor::DeviceFrame frame;
  frame.depth = std::move(depth);
  frame.color = std::move(color);
  const recon::Mat4f baked =
      glm::translate(recon::Mat4f(1.0f), glm::vec3(5.0f, 6.0f, 7.0f));
  frame.depth_camera = {500.25f, 499.75f, 321.5f, 287.25f, 0.5f,
                        3.86f,   640,     576,    baked};
  frame.color_camera = {1800.5f, 1799.25f, 1919.75f, 1081.5f,
                        3840,    2160,     baked};
  return frame;
}

CloudStyle style_of(std::uint32_t family) {
  CloudStyle style;
  style.mode = pipelines::CloudColor::TintedLuma;
  style.tint = pipelines::pack_rgba8(12, 34, 56);
  style.stride = 3;
  style.point_size = 2.5f;
  style.viewport = {{10, 20}, {300, 200}};
  style.view_proj =
      glm::mat4(glm::vec4(1, 2, 3, 4), glm::vec4(5, 6, 7, 8),
                glm::vec4(9, 10, 11, 12), glm::vec4(13, 14, 15, 16));
  style.producer_family = family;
  style.renderer_family = family;
  return style;
}

class VizPointCloud : public core::test::VulkanDeviceTest {
 protected:
  std::shared_ptr<const core::Buffer> buffer(VkBufferUsageFlags usage) {
    core::BufferDesc desc;
    desc.size = 256;
    desc.usage = usage;
    core::Result<core::Buffer> made = allocator().create_buffer(desc);
    EXPECT_TRUE(made.ok()) << made.status().message();
    if (!made.ok()) return nullptr;
    return std::make_shared<const core::Buffer>(std::move(made).value());
  }
};

// Each parameter lands in its own field, exactly: fx is not fy, cx is not
// cy, a width is not a height, the depth camera is not the colour camera,
// and the poses are the ones given.
TEST_F(VizPointCloud, MapsEveryParameterToItsOwnFieldExactly) {
  const sensor::DeviceFrame frame =
      frame_of(buffer(kStorage), buffer(kStorage));
  const Mat4d color_to_world = pose(17.0, {0.3, 1.0, 0.2}, {0.4, -0.2, 1.5});
  const Mat4d depth_to_color =
      pose(1.5, {1.0, 0.1, -0.3}, {0.032, -0.002, 0.004});
  const CloudStyle style = style_of(device().queue_family());
  const core::Result<pipelines::DepthCloudDraw> made =
      cloud_draw(frame, color_to_world, depth_to_color, style);
  ASSERT_TRUE(made.ok()) << made.status().message();
  const pipelines::DepthCloudDraw& draw = made.value();

  EXPECT_EQ(draw.depth.buffer, frame.depth.get());
  EXPECT_EQ(draw.depth.offset, 0u);
  EXPECT_EQ(draw.depth.camera.fx, 500.25f);
  EXPECT_EQ(draw.depth.camera.fy, 499.75f);
  EXPECT_EQ(draw.depth.camera.cx, 321.5f);
  EXPECT_EQ(draw.depth.camera.cy, 287.25f);
  EXPECT_EQ(draw.depth.camera.width, 640u);
  EXPECT_EQ(draw.depth.camera.height, 576u);
  EXPECT_EQ(draw.depth.min_depth, 0.5f);
  EXPECT_EQ(draw.depth.max_depth, 3.86f);
  EXPECT_EQ(draw.depth.to_world, glm::mat4(color_to_world * depth_to_color));

  EXPECT_EQ(draw.color.buffer, frame.color.get());
  EXPECT_EQ(draw.color.camera.fx, 1800.5f);
  EXPECT_EQ(draw.color.camera.fy, 1799.25f);
  EXPECT_EQ(draw.color.camera.cx, 1919.75f);
  EXPECT_EQ(draw.color.camera.cy, 1081.5f);
  EXPECT_EQ(draw.color.camera.width, 3840u);
  EXPECT_EQ(draw.color.camera.height, 2160u);
  EXPECT_EQ(draw.color.to_world, glm::mat4(color_to_world));
  EXPECT_TRUE(draw.color.alpha_is_coverage);

  EXPECT_EQ(draw.mode, pipelines::CloudColor::TintedLuma);
  EXPECT_EQ(draw.tint, style.tint);
  EXPECT_EQ(draw.stride, 3u);
  EXPECT_EQ(draw.point_size, 2.5f);
  EXPECT_EQ(draw.viewport.offset.x, 10);
  EXPECT_EQ(draw.viewport.offset.y, 20);
  EXPECT_EQ(draw.viewport.extent.width, 300u);
  EXPECT_EQ(draw.viewport.extent.height, 200u);
  EXPECT_EQ(draw.view_proj, style.view_proj);
}

// Solid reads no colour; nor does a mode that would, on a frame prepared
// without it.
TEST_F(VizPointCloud, ReadsColourOnlyWhereTheModeUsesItAndTheFrameHasIt) {
  const Mat4d identity(1.0);
  CloudStyle style = style_of(device().queue_family());
  style.mode = pipelines::CloudColor::Solid;
  // Solid: the colour buffer is not read, so its usage does not matter.
  const sensor::DeviceFrame solid =
      frame_of(buffer(kStorage), buffer(VK_BUFFER_USAGE_TRANSFER_DST_BIT));
  core::Result<pipelines::DepthCloudDraw> made =
      cloud_draw(solid, identity, identity, style);
  ASSERT_TRUE(made.ok()) << made.status().message();
  EXPECT_EQ(made.value().color.buffer, nullptr);
  EXPECT_EQ(made.value().mode, pipelines::CloudColor::Solid);

  style.mode = pipelines::CloudColor::True;
  const sensor::DeviceFrame depth_only = frame_of(buffer(kStorage), nullptr);
  made = cloud_draw(depth_only, identity, identity, style);
  ASSERT_TRUE(made.ok()) << made.status().message();
  EXPECT_EQ(made.value().color.buffer, nullptr);
  EXPECT_EQ(made.value().mode, pipelines::CloudColor::True);
}

TEST_F(VizPointCloud, RefusesWhatTheRendererCannotRead) {
  const Mat4d identity(1.0);
  const CloudStyle style = style_of(device().queue_family());
  const auto refused = [&](const sensor::DeviceFrame& frame,
                           const CloudStyle& s, const Mat4d& color_to_world,
                           const Mat4d& depth_to_color, const char* why) {
    const core::Status status =
        cloud_draw(frame, color_to_world, depth_to_color, s).status();
    EXPECT_EQ(status.domain(), Code::InvalidArgument) << why;
    EXPECT_EQ(status.message().rfind("cloud_draw: ", 0), 0u)
        << status.message();
    EXPECT_NE(status.message().find(why), std::string::npos)
        << status.message();
  };
  refused(frame_of(nullptr, buffer(kStorage)), style, identity, identity,
          "no depth");
  refused(frame_of(buffer(VK_BUFFER_USAGE_TRANSFER_DST_BIT), nullptr), style,
          identity, identity, "depth buffer has no STORAGE_BUFFER usage");
  refused(frame_of(buffer(kStorage), buffer(VK_BUFFER_USAGE_TRANSFER_DST_BIT)),
          style, identity, identity,
          "colour buffer has no STORAGE_BUFFER usage");
  const sensor::DeviceFrame good = frame_of(buffer(kStorage), nullptr);
  refused(good, style, glm::scale(identity, Vec3d(2.0)), identity,
          "color_to_world");
  refused(good, style, identity, glm::scale(identity, Vec3d(0.5)),
          "depth_to_color");
  // A buffer EXCLUSIVE to recon's family, drawn on another.
  CloudStyle across = style;
  across.renderer_family = style.producer_family + 1;
  refused(good, across, identity, identity, "EXCLUSIVE");
  // A style that leaves a family unset could not tell the two apart.
  const char* unset = "queue family unset";
  refused(good, CloudStyle{}, identity, identity, unset);
  CloudStyle half = style;
  half.renderer_family = VK_QUEUE_FAMILY_IGNORED;
  refused(good, half, identity, identity, unset);
  half = style;
  half.producer_family = VK_QUEUE_FAMILY_IGNORED;
  refused(good, half, identity, identity, unset);
}

// Each camera shown, placed and with a frame is drawn, in its own tint; the
// unplaced, the hidden and the frameless are not.
TEST_F(VizPointCloud, DrawsEachShownPlacedCameraInItsTint) {
  std::vector<SceneCamera> cameras(4);
  std::vector<std::optional<sensor::DeviceFrame>> frames(4);
  for (std::size_t i = 0; i < cameras.size(); ++i) {
    const double k = static_cast<double>(i);
    cameras[i].color_to_world =
        pose(5.0 * k, {0.0, 1.0, 0.0}, {0.3 * k, 0.0, 0.0});
    cameras[i].tint = pipelines::pack_rgba8(static_cast<std::uint8_t>(i), 1, 2);
    if (i != 3) frames[i] = frame_of(buffer(kStorage), buffer(kStorage));
  }
  cameras[1].color_to_world.reset();  // not placed
  cameras[2].shown = false;
  const CloudStyle style = style_of(device().queue_family());
  core::Result<std::vector<pipelines::DepthCloudDraw>> draws =
      cloud_draws(frames, cameras, style);
  ASSERT_TRUE(draws.ok()) << draws.status().message();
  ASSERT_EQ(draws.value().size(), 1u);
  EXPECT_EQ(draws.value()[0].tint, cameras[0].tint);
  EXPECT_EQ(draws.value()[0].depth.buffer, frames[0]->depth.get());

  frames.pop_back();
  EXPECT_EQ(cloud_draws(frames, cameras, style).status().domain(),
            Code::InvalidArgument);
  frames.emplace_back();
  frames[0]->depth = buffer(VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  const core::Status named = cloud_draws(frames, cameras, style).status();
  EXPECT_EQ(named.domain(), Code::InvalidArgument);
  EXPECT_NE(named.message().find("camera 0"), std::string::npos)
      << named.message();
}

// On a device whose compute library and renderer use two queue families: a
// buffer CONCURRENT across both is drawn, one EXCLUSIVE to the compute
// library's is refused where it is read.
class VizPointCloudAcrossFamilies : public core::test::SharedDeviceTest {};

TEST_F(VizPointCloudAcrossFamilies, DrawsConcurrentBuffersAndRefusesExclusive) {
  core::test::TwoFamilyDevice& two = two_family();
  const auto made = [&](bool shared) {
    core::BufferDesc desc;
    desc.size = 256;
    desc.usage = kStorage;
    if (shared) {
      desc.queue_families = two.families().data();
      desc.queue_family_count = core::test::TwoFamilyDevice::kFamilyCount;
    }
    core::Result<core::Buffer> buffer =
        two.compute_allocator().create_buffer(desc);
    EXPECT_TRUE(buffer.ok()) << buffer.status().message();
    return std::make_shared<const core::Buffer>(std::move(buffer).value());
  };
  CloudStyle style = style_of(two.families()[0]);
  style.renderer_family = two.families()[1];
  const Mat4d identity(1.0);
  const sensor::DeviceFrame concurrent = frame_of(made(true), made(true));
  EXPECT_TRUE(cloud_draw(concurrent, identity, identity, style).ok());
  const sensor::DeviceFrame exclusive = frame_of(made(false), made(true));
  const core::Status refused =
      cloud_draw(exclusive, identity, identity, style).status();
  EXPECT_EQ(refused.domain(), Code::InvalidArgument);
  EXPECT_NE(refused.message().find("depth is EXCLUSIVE"), std::string::npos)
      << refused.message();
  // Colour EXCLUSIVE to the compute library's family is refused where the
  // mode reads it, and drawn past where it does not.
  const sensor::DeviceFrame colour_exclusive =
      frame_of(made(true), made(false));
  const core::Status colour_refused =
      cloud_draw(colour_exclusive, identity, identity, style).status();
  EXPECT_EQ(colour_refused.domain(), Code::InvalidArgument);
  EXPECT_NE(colour_refused.message().find("colour is EXCLUSIVE"),
            std::string::npos)
      << colour_refused.message();
  style.mode = pipelines::CloudColor::Solid;
  const core::Result<pipelines::DepthCloudDraw> solid =
      cloud_draw(colour_exclusive, identity, identity, style);
  ASSERT_TRUE(solid.ok()) << solid.status().message();
  EXPECT_EQ(solid.value().color.buffer, nullptr);
}

}  // namespace
}  // namespace volumetric_kit::calib::viz
