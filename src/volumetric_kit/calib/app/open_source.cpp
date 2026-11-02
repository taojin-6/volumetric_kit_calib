// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/open_source.hpp"

#include <filesystem>
#include <system_error>
#include <utility>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"
#include "volumetric_kit/sensor/orbbec/orbbec_sensor.hpp"

namespace volumetric_kit::calib::app {

core::Result<RigScenario> check_scenario(const core::config::Config& config) {
  return compute::guarded("check_scenario", [&]() -> core::Result<RigScenario> {
    RigScenario scenario;
    VKC_ASSIGN(scenario.preset, rig_preset(config.source));
    std::error_code ec;
    if (!std::filesystem::is_directory(config.output.path, ec)) {
      return core::Status::not_found("output.path: " + config.output.path +
                                     " is not an existing directory");
    }
    // sensor's reader refuses a file that lists no camera.
    core::Result<sensor::OrbbecRigSyncConfig> sync =
        sensor::read_orbbec_sync_config(config.source.sync_config);
    if (!sync.ok()) return sync.status().with_context("source.sync_config");
    scenario.sync = std::move(sync).value();
    return scenario;
  });
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

core::Result<OpenedSource> open_source(
    const core::config::Config& config, const RigScenario& scenario,
    const core::Device& device, core::Allocator& allocator,
    const std::vector<std::uint32_t>& families) {
  // TODO: the decklink arm, on core's source.driver (A5).
  return compute::guarded("open_source", [&]() -> core::Result<OpenedSource> {
    const RigPreset& preset = scenario.preset;
    // TODO: open the rig from scenario.sync once sensor's open_orbbec_sensors
    // takes a parsed sync configuration, so the file is read once.
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
                 orbbec_camera(infos[i], orbbec->device_info(), scenario.sync));
      out.cameras.push_back(std::move(camera));
      out.lenses.push_back(*infos[i].color);
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
