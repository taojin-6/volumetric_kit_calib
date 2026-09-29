// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Shared by the capture-tier tests: a scratch directory removed on scope exit,
// and cameras and images that are cheap to make and easy to tell apart.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>

#include <gtest/gtest.h>

#include "volumetric_kit/calib/capture/rig_source.hpp"
#include "volumetric_kit/calib/core/camera_model.hpp"

namespace volumetric_kit::calib::capture::testing {

/// A fresh directory under the system temp directory, removed with everything
/// in it when the object goes out of scope.
class ScratchDir {
 public:
  ScratchDir() {
    static std::atomic<int> counter{0};
    const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
    std::string name = "vc_capture_";
    if (info != nullptr) {
      name += std::string(info->test_suite_name()) + "_" + info->name() + "_";
    }
    name += std::to_string(counter++);
    std::error_code ec;
    path_ = std::filesystem::temp_directory_path(ec) / name;
    std::filesystem::remove_all(path_, ec);
  }
  ~ScratchDir() {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }
  ScratchDir(const ScratchDir&) = delete;
  ScratchDir& operator=(const ScratchDir&) = delete;

  /// The directory itself (not created: SessionWriter::create makes it).
  std::string str() const { return path_.string(); }
  /// A path inside it.
  std::string operator/(const std::string& leaf) const {
    return (path_ / leaf).string();
  }

 private:
  std::filesystem::path path_;
};

/// A camera of @p width x @p height with a distinct, lensed model.
inline CameraInfo make_camera(std::string serial, SyncRole role, int width,
                              int height) {
  CameraInfo camera;
  camera.serial = std::move(serial);
  camera.role = role;
  // Values with long binary expansions, so a lossy round trip shows.
  auto lens = PinholeBrownConrady::create(
      {width, height},
      {/*fx=*/0.9 * width + 0.1234567890123, /*fy=*/0.9 * width - 0.0987654321,
       /*cx=*/(width - 1) / 2.0 + 0.3141592653589793,
       /*cy=*/(height - 1) / 2.0 - 0.2718281828459045},
      {/*k1=*/0.0731199458241463, /*k2=*/-0.1004820391535759,
       /*p1=*/-0.00046360507258214056, /*p2=*/-5.1091061322949827e-05,
       /*k3=*/0.040595322847366333});
  EXPECT_TRUE(lens.ok()) << lens.status().message();
  if (lens.ok()) camera.lens = std::move(lens).value();
  return camera;
}

/// An image whose every pixel is @p level.
inline LumaImage flat_image(const ImageSize& size, std::uint8_t level) {
  LumaImage image;
  image.size = size;
  image.pixels.assign(static_cast<std::size_t>(size.width) *
                          static_cast<std::size_t>(size.height),
                      level);
  return image;
}

/// An image whose pixels vary with position and @p seed, so two differ.
inline LumaImage pattern_image(const ImageSize& size, unsigned seed) {
  LumaImage image = flat_image(size, 0);
  for (std::size_t p = 0; p < image.pixels.size(); ++p) {
    image.pixels[p] = static_cast<std::uint8_t>((p * 7 + seed * 31) & 0xFFu);
  }
  return image;
}

}  // namespace volumetric_kit::calib::capture::testing
