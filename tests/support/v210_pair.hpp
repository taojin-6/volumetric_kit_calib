// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file v210_pair.hpp
/// @brief V210 pictures on the device, as a DeckLink pair hands them out, for
///        the source and app tiers' tests: 10-bit samples packed six pixels
///        to four words, in a buffer the GPU reads, so no test opens a card.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/sensor/stereo/stereo_sensor.hpp"
#include "volumetric_kit/sensor/v210_rgb.hpp"
#include "volumetric_kit/sensor/yuv_image.hpp"

namespace volumetric_kit::calib::test {

/// @brief A picture's 10-bit samples: Y' `width` a row, Cb and Cr
///        `width / 2` a row (co-sited with the even pixels), every row.
struct V210Samples {
  std::uint32_t width = 0;   ///< Even, and a multiple of 6 here.
  std::uint32_t height = 0;  ///< Rows.
  std::vector<std::uint16_t> y, cb, cr;
};

/// @brief Samples of `width` x `height`, every pixel's from `luma(x, y)` and
///        each pair's chroma from `chroma(x / 2, y)` (Cb, Cr), codes 4..1019.
template <typename Luma, typename Chroma>
V210Samples v210_samples(std::uint32_t width, std::uint32_t height, Luma luma,
                         Chroma chroma) {
  V210Samples s;
  s.width = width;
  s.height = height;
  s.y.resize(std::size_t{width} * height);
  s.cb.resize(std::size_t{width / 2} * height);
  s.cr.resize(s.cb.size());
  for (std::uint32_t r = 0; r < height; ++r) {
    for (std::uint32_t x = 0; x < width; ++x) {
      s.y[std::size_t{r} * width + x] = luma(x, r);
    }
    for (std::uint32_t c = 0; c < width / 2; ++c) {
      const std::pair<std::uint16_t, std::uint16_t> cbcr = chroma(c, r);
      s.cb[std::size_t{r} * (width / 2) + c] = cbcr.first;
      s.cr[std::size_t{r} * (width / 2) + c] = cbcr.second;
    }
  }
  return s;
}

/// @brief @p s packed as V210 rows @p stride bytes apart (at least
///        `v210_row_bytes(width)`): each six pixels four little-endian words,
///        Cb0 Y0 Cr0 | Y1 Cb1 Y2 | Cr1 Y3 Cb2 | Y4 Cr2 Y5, ten bits each from
///        the low end; padding 0xCD.
/// @pre `s.width` is a multiple of 6.
inline std::vector<std::uint8_t> pack_v210(const V210Samples& s,
                                           std::size_t stride) {
  std::vector<std::uint8_t> bytes(stride * s.height, 0xCD);
  const std::size_t half = s.width / 2;
  const auto put = [&bytes](std::size_t at, std::uint32_t word) {
    for (int b = 0; b < 4; ++b) {
      bytes[at + static_cast<std::size_t>(b)] =
          static_cast<std::uint8_t>(word >> (8 * b));
    }
  };
  for (std::uint32_t r = 0; r < s.height; ++r) {
    const std::uint16_t* y = &s.y[std::size_t{r} * s.width];
    const std::uint16_t* cb = &s.cb[std::size_t{r} * half];
    const std::uint16_t* cr = &s.cr[std::size_t{r} * half];
    for (std::uint32_t g = 0; g < s.width / 6; ++g) {
      const std::size_t p = std::size_t{g} * 6;  // first pixel
      const std::size_t c = std::size_t{g} * 3;  // first chroma pair
      const std::size_t at = std::size_t{r} * stride + std::size_t{g} * 16;
      const auto word = [](std::uint32_t a, std::uint32_t b, std::uint32_t d) {
        return a | (b << 10) | (d << 20);
      };
      put(at, word(cb[c], y[p], cr[c]));
      put(at + 4, word(y[p + 1], cb[c + 1], y[p + 2]));
      put(at + 8, word(cr[c + 1], y[p + 3], cb[c + 2]));
      put(at + 12, word(y[p + 4], cr[c + 2], y[p + 5]));
    }
  }
  return bytes;
}

/// @brief A V210 picture of @p width x @p height, rows @p stride apart,
///        described as a DeckLink capture is: BT.709, limited range, chroma
///        co-sited; no planes.
inline sensor::YuvImage device_v210_description(std::uint32_t width,
                                                std::uint32_t height,
                                                std::size_t stride) {
  sensor::YuvImage picture;
  picture.layout = sensor::YuvLayout::V210;
  picture.stride[0] = stride;
  picture.width = width;
  picture.height = height;
  const sensor::YcbcrWeights bt709 =
      sensor::ycbcr_weights(sensor::VideoColorMatrix::Bt709);
  picture.kr = bt709.kr;
  picture.kb = bt709.kb;
  picture.full_range = false;
  picture.chroma_location = sensor::ChromaLocation::Left;
  return picture;
}

/// @brief @p bytes in a buffer on @p device the GPU reads and copies from,
///        described as @ref device_v210_description has it.
inline core::Result<sensor::YuvImage> device_v210(
    const core::Device& device, core::Allocator& allocator,
    const std::vector<std::uint8_t>& bytes, std::uint32_t width,
    std::uint32_t height, std::size_t stride) {
  static_cast<void>(device);
  core::BufferDesc desc;
  desc.size = bytes.size();
  desc.usage =
      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  desc.memory = core::MemoryUsage::DeviceMapped;
  VKC_ASSIGN(core::Buffer buffer, allocator.create_buffer(desc));
  std::memcpy(buffer.mapped(), bytes.data(), bytes.size());
  sensor::YuvImage picture = device_v210_description(width, height, stride);
  picture.device = std::make_shared<const core::Buffer>(std::move(buffer));
  return picture;
}

/// @brief What @p picture's words make as 8-bit R'G'B' on the host
///        (`v210_to_rgb8`), from @p bytes, the words it was made of: the
///        reference a still is held to.
inline core::Result<std::vector<std::uint8_t>> v210_rgb(
    const sensor::YuvImage& picture, const std::vector<std::uint8_t>& bytes) {
  sensor::YuvImage host = picture;
  host.device.reset();
  host.host = bytes.data();
  std::vector<std::uint8_t> rgb(std::size_t{picture.width} * picture.height *
                                3);
  VKC_TRY(sensor::v210_to_rgb8(host, rgb.data(), rgb.size()));
  return rgb;
}

/// @brief A stereo pair of V210 pictures on the device, @p left and
///        @p right, the right one @p skew_ns after the left at
///        @p timestamp_ns; each picture holds its buffer, as a DeckLink's
///        does.
inline sensor::StereoFrame v210_pair(sensor::YuvImage left,
                                     sensor::YuvImage right,
                                     std::uint64_t timestamp_ns,
                                     std::int64_t skew_ns,
                                     std::uint64_t sequence) {
  sensor::StereoFrame frame;
  frame.left = std::move(left);
  frame.right = std::move(right);
  frame.timestamp_ns = timestamp_ns;
  frame.skew_ns = skew_ns;
  frame.sequence = sequence;
  return frame;
}

}  // namespace volumetric_kit::calib::test
