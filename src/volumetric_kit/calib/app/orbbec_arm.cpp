// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "orbbec_arm.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <utility>

#include "volumetric_kit/calib/app/calib_mode.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"

namespace volumetric_kit::calib::app {

core::Status check_orbbec(const core::config::Config& config) {
  return compute::guarded("check_scenario", [&]() -> core::Status {
    VKC_TRY(rig_preset(config.source).status());
    // sensor's reader refuses a file that lists no camera.
    core::Result<sensor::OrbbecRigSyncConfig> sync =
        sensor::read_orbbec_sync_config(config.source.sync_config);
    if (!sync.ok()) return sync.status().with_context("source.sync_config");
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
  options.color_codec = preset.color_codec == core::config::ColorCodec::Hevc
                            ? sensor::OrbbecColorCodec::Hevc
                            : sensor::OrbbecColorCodec::Mjpeg;
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

core::Result<capture::SessionCamera> orbbec_camera(
    const sensor::SensorInfo& info, const sensor::OrbbecDeviceInfo& device,
    const sensor::OrbbecRigSyncConfig& sync) {
  return compute::guarded(
      "orbbec_camera", [&]() -> core::Result<capture::SessionCamera> {
        VKC_CHECK(info.color.has_value(), "orbbec_camera: no colour model");
        capture::SessionCamera out;
        out.id = info.id;
        out.model = info.model;
        out.size = info.color->size;
        out.fps = info.fps;
        out.role = sensor::to_string(info.role);
        out.color = info.color;
        out.depth = info.depth;
        out.depth_to_color = info.depth_to_color;
        out.lens = capture::LensOrigin::Factory;
        capture::OrbbecFacts& facts = out.orbbec.emplace();
        facts.firmware = device.firmware_version;
        facts.ip_address = device.ip_address;
        facts.sync_mode = sensor::to_string(device.sync_mode);
        facts.color_lens_model = device.color_lens_model;
        facts.depth_lens_model = device.depth_lens_model;
        for (const sensor::OrbbecSyncDevice& entry : sync.devices) {
          if (entry.serial == out.id) {
            facts.depth_delay_us = entry.sync.depth_delay_us;
          }
        }
        return out;
      });
}

core::Result<OpenedSource> open_orbbec(
    const core::config::Config& config, const RigScenario& scenario,
    const core::Device& device, core::Allocator& allocator,
    const std::vector<std::uint32_t>& families) {
  return compute::guarded("open_source", [&]() -> core::Result<OpenedSource> {
    VKC_ASSIGN(const RigPreset preset, rig_preset(config.source));
    // TODO: read the sync file once, when sensor's open_orbbec_sensors takes
    // a parsed sync configuration: here, and again by the opener below.
    core::Result<sensor::OrbbecRigSyncConfig> sync =
        sensor::read_orbbec_sync_config(config.source.sync_config);
    if (!sync.ok()) return sync.status().with_context("source.sync_config");
    VKC_ASSIGN(auto sensors,
               sensor::open_orbbec_sensors(
                   config.source.sync_config,
                   orbbec_options(preset, device, allocator, families)));
    OpenedSource out;
    std::vector<sensor::SensorInfo> infos;
    for (const std::unique_ptr<sensor::IRgbdSensor>& camera : sensors) {
      infos.push_back(camera->info());
    }
    VKC_TRY(check_preset(infos, preset));
    for (std::size_t i = 0; i < sensors.size(); ++i) {
      // open_orbbec_sensors hands out one OrbbecSensor per camera.
      const auto* orbbec =
          dynamic_cast<const sensor::OrbbecSensor*>(sensors[i].get());
      if (orbbec == nullptr) {
        return core::Status::invalid_argument(
            "open_source: camera " + infos[i].id + " is not an Orbbec sensor");
      }
      VKC_ASSIGN(capture::SessionCamera camera,
                 orbbec_camera(infos[i], orbbec->device_info(), *sync));
      VKC_ASSIGN(const sensor::camera::CameraModel lens,
                 take_lens(scenario.mode, scenario.calibration.get(),
                           config.source.calibration, camera));
      out.cameras.push_back(std::move(camera));
      out.lenses.push_back(lens);
    }
    VKC_ASSIGN(sensor::SensorArray array,
               sensor::SensorArray::open(std::move(sensors), {}));
    out.reference = array.primary();
    VKC_ASSIGN(out.source, source::RigSource::open(std::move(array), device,
                                                   allocator, families));
    out.driver = "orbbec";
    out.sync_path = config.source.sync_config;
    out.depth_mode = preset.depth_mode;
    out.color_codec =
        preset.color_codec == core::config::ColorCodec::Hevc ? "hevc" : "mjpeg";
    return out;
  });
}

}  // namespace volumetric_kit::calib::app
