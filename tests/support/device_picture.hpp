// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file device_picture.hpp
/// @brief NV12 pictures in device images, as a decoder leaves them (GENERAL,
///        readable by copies), for the app tier's GPU tests, written through
///        the core's `test::write_image`.

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/write_image.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/image.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/recon/sensor/yuv_image.hpp"

namespace volumetric_kit::calib::test {

/// @brief A `width` x `height` image of `format` holding `bytes` (rows
///        packed), with copy usage, written on @p device's queue and left in
///        GENERAL, as a decoder leaves its planes; `CONCURRENT` across
///        @p families when they are given.
/// @return The image; null when it cannot be made.
inline std::shared_ptr<const core::Image> device_plane(
    const core::Device& device, core::Allocator& allocator, VkFormat format,
    std::uint32_t width, std::uint32_t height,
    const std::vector<std::uint8_t>& bytes,
    const std::vector<std::uint32_t>& families = {}) {
  core::ImageDesc desc;
  desc.extent = {width, height};
  desc.format = format;
  desc.usage =
      VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  desc.with_view = false;
  desc.queue_families = families.empty() ? nullptr : families.data();
  desc.queue_family_count = static_cast<std::uint32_t>(families.size());
  core::Result<core::Image> image = allocator.create_image(desc);
  if (!image.ok() ||
      !core::test::write_image(device, allocator, image.value(), bytes.data(),
                               bytes.size(), VK_IMAGE_LAYOUT_GENERAL)
           .ok()) {
    return nullptr;
  }
  return std::make_shared<const core::Image>(std::move(image).value());
}

/// @brief An NV12 picture of @p luma (`width * height` bytes, rows packed)
///        with neutral chroma, coded as an MJPEG camera's: BT.601, full
///        range, centred chroma; its planes as @ref device_plane makes them.
/// @return The picture; its images null when they cannot be made.
inline recon::sensor::YuvImage device_nv12(
    const core::Device& device, core::Allocator& allocator, std::uint32_t width,
    std::uint32_t height, const std::vector<std::uint8_t>& luma,
    const std::vector<std::uint32_t>& families = {}) {
  recon::sensor::YuvImage picture;
  picture.layout = recon::sensor::YuvLayout::Nv12;
  picture.width = width;
  picture.height = height;
  const std::uint32_t cw = (width + 1) / 2;
  const std::uint32_t ch = (height + 1) / 2;
  picture.image[0] = device_plane(device, allocator, VK_FORMAT_R8_UNORM, width,
                                  height, luma, families);
  picture.image[1] = device_plane(
      device, allocator, VK_FORMAT_R8G8_UNORM, cw, ch,
      std::vector<std::uint8_t>(static_cast<std::size_t>(cw) * ch * 2, 128),
      families);
  const recon::sensor::YcbcrWeights weights =
      recon::sensor::ycbcr_weights(recon::sensor::VideoColorMatrix::Bt601);
  picture.kr = weights.kr;
  picture.kb = weights.kb;
  picture.full_range = true;
  picture.chroma_location = recon::sensor::ChromaLocation::Center;
  return picture;
}

}  // namespace volumetric_kit::calib::test
