// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "decklink_arm.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::app {
namespace {

// The camera of `source.cameras` that `field` names.
core::Result<sensor::DecklinkPort> port(const core::config::Source& source,
                                        const std::string& id,
                                        const std::string& field) {
  const auto named = std::find_if(
      source.cameras.begin(), source.cameras.end(),
      [&](const core::config::LiveCamera& camera) { return camera.id == id; });
  if (named == source.cameras.end()) {
    return core::Status::invalid_argument(field + ": " + id +
                                          " is not a camera of source.cameras");
  }
  return sensor::DecklinkPort{named->id, named->input};
}

}  // namespace

core::Result<sensor::DecklinkStereo::Options> decklink_options(
    const core::config::Source& source) {
  return compute::guarded(
      "decklink_options",
      [&]() -> core::Result<sensor::DecklinkStereo::Options> {
        if (source.pairs.size() != 1) {
          return core::Status::invalid_argument(
              "source.pairs: " + std::to_string(source.pairs.size()) +
              " pairs; calib_rig opens exactly one");
        }
        const core::config::StereoPairConfig& pair = source.pairs.front();
        sensor::DecklinkStereo::Options options;
        VKC_ASSIGN(options.left,
                   port(source, pair.left, "source.pairs[0].left"));
        VKC_ASSIGN(options.right,
                   port(source, pair.right, "source.pairs[0].right"));
        options.width = source.color_width;
        options.height = source.color_height;
        options.fps = source.fps;
        // No lens yet: calib makes them.
        options.calibration.reset();
        return options;
      });
}

core::Result<OpenedSource> open_decklink(
    const core::config::Config& config, const RigScenario& scenario,
    const core::Device& device, core::Allocator& allocator,
    const std::vector<std::uint32_t>& families) {
  return compute::guarded("open_source", [&]() -> core::Result<OpenedSource> {
    VKC_ASSIGN(const sensor::DecklinkStereo::Options options,
               decklink_options(config.source));
    VKC_ASSIGN(sensor::DecklinkStereo pair,
               sensor::DecklinkStereo::open(options, device, allocator));
    const std::string model = pair.info().model;
    const sensor::camera::ImageSize size{options.width, options.height};
    OpenedSource out;
    for (const sensor::DecklinkPort* side : {&options.left, &options.right}) {
      capture::SessionCamera camera;
      camera.id = side->id;
      camera.model = model;
      camera.size = size;
      camera.fps = options.fps;
      camera.input = side->input;
      VKC_TRY(add_stereo_camera(std::move(camera), scenario,
                                config.source.calibration, out));
    }
    source::StereoMember member;
    member.sensor = std::make_unique<sensor::DecklinkStereo>(std::move(pair));
    member.ids = {options.left.id, options.right.id};
    // Opened without a calibration, the pair reports its cameras 0 x 0.
    member.sizes = {size, size};
    VKC_ASSIGN(out.source, source::RigSource::open(std::move(member), device,
                                                   allocator, families));
    out.reference = 0;  // the left camera
    out.driver = "decklink";
    return out;
  });
}

}  // namespace volumetric_kit::calib::app
