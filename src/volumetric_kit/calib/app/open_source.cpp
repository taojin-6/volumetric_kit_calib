// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/open_source.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <utility>

#include "volumetric_kit/calib/app/calib_mode.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"

#if defined(VC_WITH_ORBBEC)
#include "orbbec_arm.hpp"
#endif
#if defined(VC_WITH_DECKLINK)
#include "decklink_arm.hpp"
#endif

namespace volumetric_kit::calib::app {

namespace {

constexpr const char* kOrbbec = "orbbec";
constexpr const char* kDecklink = "decklink";

// The driver's own checks (check_scenario's step 5), or why this build has
// no arm for it.
core::Status check_driver(const core::config::Config& config) {
  const std::string& driver = config.source.driver;
  if (driver == kOrbbec) {
#if defined(VC_WITH_ORBBEC)
    return check_orbbec(config);
#else
    return core::Status::invalid_argument(
        "source.driver: orbbec: this calib_rig was built without sensor's "
        "Orbbec driver (configure with -DVS_WITH_ORBBEC=ON)");
#endif
  }
#if defined(VC_WITH_DECKLINK)
  return decklink_options(config.source).status();
#else
  return core::Status::invalid_argument(
      "source.driver: decklink: this calib_rig was built without sensor's "
      "DeckLink driver (configure with -DVS_WITH_DECKLINK=ON, on Linux "
      "x86-64, with the DeckLink SDK)");
#endif
}

}  // namespace

core::Result<RigScenario> check_scenario(const core::config::Config& config) {
  return compute::guarded("check_scenario", [&]() -> core::Result<RigScenario> {
    RigScenario scenario;
    VKC_ASSIGN(scenario.mode, session_mode(config.operation));
    scenario.driver = config.source.driver;
    if (scenario.driver != kOrbbec && scenario.driver != kDecklink) {
      return core::Status::invalid_argument(
          "source.driver: \"" + scenario.driver +
          "\" is not one calib_rig opens (orbbec, decklink)");
    }
    // TODO: a DeckLink pair's poses, with its rectified rows (A6).
    if (scenario.driver == kDecklink &&
        scenario.mode != capture::SessionMode::Intrinsics) {
      return core::Status::invalid_argument(
          "source.driver: decklink with operation calibrate: calib_rig fits a "
          "DeckLink pair's lenses alone (operation calibrate_intrinsics)");
    }
    std::error_code ec;
    if (!std::filesystem::is_directory(config.output.path, ec)) {
      return core::Status::not_found("output.path: " + config.output.path +
                                     " is not an existing directory");
    }
    if (!config.source.calibration.empty()) {
      core::Result<sensor::camera::ArrayCalibration> file =
          sensor::camera::read_array_calibration(config.source.calibration);
      if (!file.ok()) return file.status().with_context("source.calibration");
      scenario.calibration =
          std::make_shared<const sensor::camera::ArrayCalibration>(
              std::move(file).value());
    }
    VKC_TRY(check_driver(config));
    return scenario;
  });
}

core::Result<OpenedSource> open_source(
    const core::config::Config& config, const RigScenario& scenario,
    const core::Device& device, core::Allocator& allocator,
    const std::vector<std::uint32_t>& families) {
#if defined(VC_WITH_ORBBEC)
  if (scenario.driver == kOrbbec) {
    return open_orbbec(config, scenario, device, allocator, families);
  }
#endif
#if defined(VC_WITH_DECKLINK)
  if (scenario.driver == kDecklink) {
    return open_decklink(config, scenario, device, allocator, families);
  }
#endif
  static_cast<void>(config);
  static_cast<void>(device);
  static_cast<void>(allocator);
  static_cast<void>(families);
  return core::Status::invalid_argument(
      "open_source: no arm opens source.driver \"" + scenario.driver +
      "\" (check_scenario refuses it)");
}

core::Status add_stereo_camera(capture::SessionCamera camera,
                               const RigScenario& scenario,
                               const std::string& path, OpenedSource& opened) {
  return compute::guarded("add_stereo_camera", [&]() -> core::Status {
    VKC_ASSIGN(const RigLens lens,
               rig_lens(scenario.mode, camera.id, camera.size, std::nullopt,
                        scenario.calibration.get(), path));
    camera.lens = lens.origin;
    if (lens.origin == capture::LensOrigin::File) camera.lens_file = path;
    opened.cameras.push_back(std::move(camera));
    opened.lenses.push_back(lens.model);
    return {};
  });
}

}  // namespace volumetric_kit::calib::app
