// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/preset.hpp"

#include <algorithm>
#include <cstddef>
#include <string>

#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::app {

namespace {

namespace camera = recon::camera;
namespace sensor = recon::sensor;

std::string size_text(const camera::ImageSize& size) {
  return std::to_string(size.width) + "x" + std::to_string(size.height);
}

bool same_size(const camera::ImageSize& a, const camera::ImageSize& b) {
  return a.width == b.width && a.height == b.height;
}

// The Femto Mega's depth modes, as its maker names and ranges them.
struct NamedDepthMode {
  camera::ImageSize size;
  const char* name;
  float min_m;
  float max_m;
};
constexpr NamedDepthMode kDepthModes[] = {
    {{640, 576}, "nfov_unbinned", 0.5f, 3.86f},
    {{320, 288}, "nfov_binned", 0.5f, 5.46f},
    {{1024, 1024}, "wfov_unbinned", 0.25f, 2.21f},
    {{512, 512}, "wfov_binned", 0.25f, 2.88f},
};

}  // namespace

core::Result<capture::DepthMode> depth_mode(camera::ImageSize depth) {
  return compute::guarded(
      "depth_mode", [&]() -> core::Result<capture::DepthMode> {
        for (const NamedDepthMode& mode : kDepthModes) {
          if (!same_size(mode.size, depth)) continue;
          capture::DepthMode out;
          out.name = mode.name;
          out.metres_per_unit = 0.001f;
          out.min_m = mode.min_m;
          out.max_m = mode.max_m;
          return out;
        }
        return core::Status::invalid_argument(
            "source.depth_width/depth_height: " + size_text(depth) +
            " is not a depth mode of the Femto Mega (640x576, 320x288, "
            "1024x1024 or 512x512)");
      });
}

core::Result<RigPreset> rig_preset(const core::config::Source& source) {
  return compute::guarded("rig_preset", [&]() -> core::Result<RigPreset> {
    if (source.color_codec != core::config::ColorCodec::Mjpeg) {
      return core::Status::invalid_argument(
          "source.color_codec: calib_rig streams MJPEG, since a capture saves "
          "each camera's JPEG");
    }
    RigPreset preset;
    preset.color = {source.color_width, source.color_height};
    preset.depth = {source.depth_width, source.depth_height};
    preset.fps = source.fps;
    VKC_ASSIGN(preset.depth_mode, depth_mode(preset.depth));
    return preset;
  });
}

core::Status check_preset(const std::vector<sensor::SensorInfo>& cameras,
                          const RigPreset& preset) {
  return compute::guarded("check_preset", [&]() -> core::Status {
    if (cameras.empty()) {
      return core::Status::invalid_argument(
          "check_preset: the rig has no "
          "cameras");
    }
    for (std::size_t i = 0; i < cameras.size(); ++i) {
      const sensor::SensorInfo& info = cameras[i];
      const std::string who =
          "camera " + (info.id.empty() ? std::to_string(i) : info.id);
      if (!info.color) {
        return core::Status::unsupported(who + " has no colour camera");
      }
      if (!same_size(info.color->size, preset.color)) {
        return core::Status::unsupported(
            who + " streams colour at " + size_text(info.color->size) +
            ", not the scenario's " + size_text(preset.color));
      }
      if (!info.depth) {
        return core::Status::unsupported(who + " has no depth camera");
      }
      if (!same_size(info.depth->size, preset.depth)) {
        return core::Status::unsupported(
            who + " streams depth at " + size_text(info.depth->size) +
            ", not the scenario's " + size_text(preset.depth));
      }
      if (info.fps != preset.fps) {
        return core::Status::unsupported(
            who + " streams at " + std::to_string(info.fps) +
            " fps, not the scenario's " + std::to_string(preset.fps));
      }
    }
    return {};
  });
}

sensor::OrbbecSensor::Options orbbec_options(
    const RigPreset& preset, const core::Device& device,
    core::Allocator& allocator, const std::vector<std::uint32_t>& families) {
  sensor::OrbbecSensor::Options options;
  options.color_width = preset.color.width;
  options.color_height = preset.color.height;
  options.depth_width = preset.depth.width;
  options.depth_height = preset.depth.height;
  options.fps = preset.fps;
  options.color_codec = sensor::OrbbecColorCodec::Mjpeg;
  options.min_depth = preset.depth_mode.min_m;
  options.max_depth = preset.depth_mode.max_m;
  options.device = &device;
  options.allocator = &allocator;
  // A count past the array is kept, for the camera's open to refuse.
  const std::size_t kept =
      std::min<std::size_t>(families.size(), sensor::kMaxColorQueueFamilies);
  for (std::size_t i = 0; i < kept; ++i) {
    options.color_queue_families[i] = families[i];
  }
  options.color_queue_family_count = static_cast<std::uint32_t>(
      std::min<std::size_t>(families.size(), 0xFFFFFFFFu));
  return options;
}

}  // namespace volumetric_kit::calib::app
