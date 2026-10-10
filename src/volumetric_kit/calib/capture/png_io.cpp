// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The one translation unit that compiles stb's PNG reader and writer, as
// static functions (png_io.hpp says why), and the two wrappers the rest of the
// tier calls. stb does not survive the tier's -Werror surface, so CMake builds
// this file alone with warnings off; keep the code of our own here minimal.
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include "stb_image.h"

#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "png_io.hpp"

#include <cstddef>
#include <string>

namespace volumetric_kit::calib::capture::detail {

core::Status write_png(const std::string& path, const LumaImage& image) {
  const int w = image.size.width;
  if (stbi_write_png(path.c_str(), w, image.size.height, 1, image.pixels.data(),
                     w) == 0) {
    return core::Status::io_error(path + ": PNG write failed");
  }
  return {};
}

core::Result<LumaImage> read_png(const std::string& path) {
  int w = 0;
  int h = 0;
  int channels = 0;
  unsigned char* data = stbi_load(path.c_str(), &w, &h, &channels, 1);
  if (data == nullptr) {
    return core::Status::io_error(path + ": not a PNG it can decode (" +
                                  std::string(stbi_failure_reason()) + ")");
  }
  LumaImage image;
  image.size = {w, h};
  image.pixels.assign(
      data, data + static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
  stbi_image_free(data);
  return image;
}

}  // namespace volumetric_kit::calib::capture::detail
