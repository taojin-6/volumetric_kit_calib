// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's preset: the Femto Mega's depth modes, the preset a scenario asks
// for, what calib_rig refuses of a scenario before it opens anything (an
// operation or a driver it does not run, its calibration file, its session
// root and sync file), and the check that every camera of a rig of any size
// opened at the preset, naming the first that did not. Host-only.

#include "volumetric_kit/calib/app/preset.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "scratch_dir.hpp"
#include "volumetric_kit/calib/app/calib_rig.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = sensor::camera;
using Code = core::Status::Code;
using test::ScratchDir;

camera::CameraModel model(std::uint32_t width, std::uint32_t height) {
  camera::CameraModel m;
  m.size = {width, height};
  m.intrinsics = {0.6 * width, 0.6 * width, 0.5 * width, 0.5 * height};
  return m;
}

// The Femto rig's modes: 4K MJPEG colour, NFOV unbinned depth, 25 fps.
RigPreset femto_preset() {
  RigPreset preset;
  preset.color = {3840, 2160};
  preset.depth = {640, 576};
  preset.fps = 25;
  preset.color_codec = core::config::ColorCodec::Mjpeg;
  preset.depth_mode = depth_mode(preset.depth).value();
  return preset;
}

// A camera that opened at femto_preset().
sensor::SensorInfo at_preset(const std::string& id) {
  sensor::SensorInfo info;
  info.id = id;
  info.color = model(3840, 2160);
  info.depth = model(640, 576);
  info.fps = 25;
  return info;
}

std::vector<sensor::SensorInfo> rig(std::size_t cameras) {
  std::vector<sensor::SensorInfo> infos;
  for (std::size_t i = 0; i < cameras; ++i) {
    infos.push_back(at_preset("CL2A14100" + std::to_string(i)));
  }
  return infos;
}

void refused(const core::Status& status, Code code, const std::string& text) {
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.domain(), code) << status.message();
  EXPECT_NE(status.message().find(text), std::string::npos)
      << status.message() << " lacks " << text;
}

TEST(AppPreset, NamesEachFemtoDepthModeAndItsRange) {
  struct Mode {
    std::uint32_t width, height;
    const char* name;
    float min_m, max_m;
  };
  const Mode modes[] = {{640, 576, "nfov_unbinned", 0.5f, 3.86f},
                        {320, 288, "nfov_binned", 0.5f, 5.46f},
                        {1024, 1024, "wfov_unbinned", 0.25f, 2.21f},
                        {512, 512, "wfov_binned", 0.25f, 2.88f}};
  for (const Mode& m : modes) {
    const core::Result<capture::DepthMode> mode =
        depth_mode({m.width, m.height});
    ASSERT_TRUE(mode.ok()) << mode.status().message();
    EXPECT_EQ(mode->name, m.name);
    EXPECT_EQ(mode->min_m, m.min_m) << m.name;
    EXPECT_EQ(mode->max_m, m.max_m) << m.name;
  }
  refused(depth_mode({640, 480}).status(), Code::InvalidArgument,
          "640x480 is not a depth mode");
  refused(depth_mode({576, 640}).status(), Code::InvalidArgument,
          "576x640 is not a depth mode");
}

TEST(AppPreset, TakesTheScenariosModes) {
  core::config::Source source;
  source.color_codec = core::config::ColorCodec::Mjpeg;
  source.color_width = 3840;
  source.color_height = 2160;
  source.depth_width = 640;
  source.depth_height = 576;
  source.fps = 25;
  core::Result<RigPreset> preset = rig_preset(source);
  ASSERT_TRUE(preset.ok()) << preset.status().message();
  EXPECT_EQ(preset->color.width, 3840u);
  EXPECT_EQ(preset->color.height, 2160u);
  EXPECT_EQ(preset->depth.width, 640u);
  EXPECT_EQ(preset->depth.height, 576u);
  EXPECT_EQ(preset->fps, 25u);
  EXPECT_EQ(preset->color_codec, core::config::ColorCodec::Mjpeg);
  EXPECT_EQ(preset->depth_mode.name, "nfov_unbinned");

  source.color_width = 1920;
  source.color_height = 1080;
  source.depth_width = 320;
  source.depth_height = 288;
  source.fps = 15;
  // The coding is the core's to check: whatever the scenario says is taken.
  source.color_codec = core::config::ColorCodec::Hevc;
  preset = rig_preset(source);
  ASSERT_TRUE(preset.ok()) << preset.status().message();
  EXPECT_EQ(preset->color.width, 1920u);
  EXPECT_EQ(preset->depth.height, 288u);
  EXPECT_EQ(preset->fps, 15u);
  EXPECT_EQ(preset->color_codec, core::config::ColorCodec::Hevc);
  EXPECT_EQ(preset->depth_mode.name, "nfov_binned");
}

TEST(AppPreset, RefusesADepthSizeWithNoMode) {
  core::config::Source source;
  source.depth_width = 800;
  source.depth_height = 600;
  refused(rig_preset(source).status(), Code::InvalidArgument,
          "source.depth_width/depth_height: 800x600");
}

// `text` written to `name` in `dir`: its path.
std::string write(const ScratchDir& dir, const std::string& name,
                  const std::string& text) {
  const std::string path = dir / name;
  EXPECT_TRUE(core::write_file(path, text, core::WriteMode::Replace).ok());
  return path;
}

// A scenario of calib_rig's as core's config resolves it: `operation` over
// `driver`, its session root `dir`; an Orbbec rig's sync file of one camera,
// or a DeckLink pair's cameras and modes.
core::config::Config scenario(const ScratchDir& dir,
                              core::config::Operation operation,
                              const std::string& driver) {
  core::config::Config config;
  config.operation = operation;
  config.source.driver = driver;
  config.output.path = dir.path().string();
  if (driver == "decklink") {
    config.source.cameras = {{"bm-1", "SDI 2"}, {"bm-2", "SDI 1"}};
    config.source.pairs = {{"bm-1", "bm-2"}};
    config.source.color_width = 3840;
    config.source.color_height = 2160;
    config.source.depth_width = 960;
    config.source.depth_height = 544;
    config.source.fps = 30;
    return config;
  }
  config.source.color_codec = core::config::ColorCodec::Mjpeg;
  config.source.color_width = 3840;
  config.source.color_height = 2160;
  config.source.depth_width = 640;
  config.source.depth_height = 576;
  config.source.fps = 25;
  config.source.sync_config = write(
      dir, "rig.json",
      R"({"devices": [{"sn": "CL2A141000N", "syncConfig": {)"
      R"("syncMode": "OB_MULTI_DEVICE_SYNC_MODE_PRIMARY", "depthDelayUs": 0,)"
      R"("colorDelayUs": 0, "trigger2ImageDelayUs": 0,)"
      R"("triggerOutEnable": true, "triggerOutDelayUs": 0,)"
      R"("framesPerTrigger": 1}}]})");
  return config;
}

// core 0.40.0 gives every scenario an operation and a driver: calib_rig runs
// what calib supports and refuses the rest before anything opens, naming the
// field and its value, so no scenario runs another solve than it asks for.
TEST(AppPreset, RunsOnlyTheOperationsAndDriversItSupports) {
  using Operation = core::config::Operation;
  const ScratchDir dir;
  // Every other operation, by its number: core's load refuses them for
  // calib_rig, so only a scenario made in code reaches this.
  for (const Operation operation :
       {Operation::Fuse, Operation::Stream, Operation::Codec, Operation::View,
        Operation::Project}) {
    for (const char* driver : {"orbbec", "decklink"}) {
      refused(check_scenario(scenario(dir, operation, driver)).status(),
              Code::InvalidArgument,
              "operation: " + std::to_string(static_cast<int>(operation)) +
                  " is not one calib_rig runs (calibrate, "
                  "calibrate_intrinsics)");
    }
  }
  for (const char* driver : {"", "realsense", "arkit", "Orbbec"}) {
    for (const Operation operation :
         {Operation::Calibrate, Operation::CalibrateIntrinsics}) {
      refused(check_scenario(scenario(dir, operation, driver)).status(),
              Code::InvalidArgument,
              std::string("source.driver: \"") + driver +
                  "\" is not one calib_rig opens (orbbec, decklink)");
    }
  }
  // A DeckLink pair's lenses and its poses, its pair's rows checked at the
  // size its runtime rectifies to, the scenario's depth size (FFS's).
  for (const Operation operation :
       {Operation::Calibrate, Operation::CalibrateIntrinsics}) {
    core::config::Config config = scenario(dir, operation, "decklink");
    config.source.depth_width = 1024;  // not FFS's own, read as given
    config.source.depth_height = 576;
    const core::Result<RigScenario> pair = check_scenario(config);
#if defined(VC_WITH_DECKLINK)
    ASSERT_TRUE(pair.ok()) << pair.status().message();
    EXPECT_EQ(pair->mode, operation == Operation::Calibrate
                              ? capture::SessionMode::Extrinsics
                              : capture::SessionMode::Intrinsics);
    EXPECT_EQ(pair->driver, "decklink");
    ASSERT_EQ(pair->pairs.size(), 1u);
    EXPECT_EQ(pair->pairs[0].left, "bm-1");
    EXPECT_EQ(pair->pairs[0].right, "bm-2");
    EXPECT_EQ(pair->pair_rows.rectified.width, 1024u);
    EXPECT_EQ(pair->pair_rows.rectified.height, 576u);
#else
    refused(pair.status(), Code::InvalidArgument,
            "source.driver: decklink: this calib_rig was built without "
            "sensor's DeckLink driver (configure with -DVS_WITH_DECKLINK=ON");
#endif
  }
  for (const Operation operation :
       {Operation::Calibrate, Operation::CalibrateIntrinsics}) {
    const core::Result<RigScenario> rig =
        check_scenario(scenario(dir, operation, "orbbec"));
#if defined(VC_WITH_ORBBEC)
    ASSERT_TRUE(rig.ok()) << rig.status().message();
    EXPECT_EQ(rig->mode, operation == Operation::Calibrate
                             ? capture::SessionMode::Extrinsics
                             : capture::SessionMode::Intrinsics);
    EXPECT_EQ(rig->driver, "orbbec");
    // The Femto rig has no pair whose rows to check.
    EXPECT_TRUE(rig->pairs.empty());
    EXPECT_EQ(rig->pair_rows.rectified.width, 0u);
#else
    refused(rig.status(), Code::InvalidArgument,
            "source.driver: orbbec: this calib_rig was built without "
            "sensor's Orbbec driver");
#endif
  }
}

// `source.calibration`, when given, is read before anything opens and kept:
// the lenses a session starts from. On whichever driver this build has (an
// app is built with one at least), so every build reads it.
TEST(AppPreset, ReadsTheScenariosCalibrationFile) {
  const ScratchDir dir;
#if defined(VC_WITH_DECKLINK)
  const char* driver = "decklink";
#else
  const char* driver = "orbbec";
#endif
  core::config::Config config =
      scenario(dir, core::config::Operation::CalibrateIntrinsics, driver);
  const core::Result<RigScenario> without = check_scenario(config);
  ASSERT_TRUE(without.ok()) << without.status().message();
  EXPECT_EQ(without->calibration, nullptr);
  config.source.calibration = dir / "missing.json";
  refused(check_scenario(config).status(), Code::NotFound,
          "source.calibration: ");
  config.source.calibration = write(dir, "bad.json", "{\"format\": 1}");
  refused(check_scenario(config).status(), Code::InvalidArgument,
          "source.calibration: ");
  sensor::camera::ArrayCalibration file;
  sensor::camera::SensorCalibration& bm1 = file.sensors.emplace_back();
  bm1.id = "bm-1";
  bm1.color = sensor::camera::CameraCalibration{
      model(3840, 2160), sensor::camera::IntrinsicsSource::Calibrated};
  config.source.calibration =
      write(dir, "array_calibration.json",
            sensor::camera::format_array_calibration(file).value());
  const core::Result<RigScenario> read = check_scenario(config);
  ASSERT_TRUE(read.ok()) << read.status().message();
  ASSERT_NE(read->calibration, nullptr);
  ASSERT_EQ(read->calibration->sensors.size(), 1u);
  EXPECT_EQ(read->calibration->sensors[0].id, "bm-1");
  ASSERT_TRUE(read->calibration->sensors[0].color.has_value());
  EXPECT_EQ(read->calibration->sensors[0].color->model.size.width, 3840u);
}

#if defined(VC_WITH_ORBBEC)
// The session root and the sync file, before anything opens.
TEST(AppPreset, ChecksTheScenarioBeforeOpeningAnything) {
  const ScratchDir dir;
  core::config::Config config =
      scenario(dir, core::config::Operation::Calibrate, "orbbec");
  config.output.path = dir / "no_such_directory";
  refused(
      check_scenario(config).status(), Code::NotFound,
      "output.path: " + config.output.path + " is not an existing directory");

  config.output.path = dir.path().string();
  core::Result<RigScenario> checked = check_scenario(config);
  ASSERT_TRUE(checked.ok()) << checked.status().message();
  EXPECT_EQ(checked->mode, capture::SessionMode::Extrinsics);
  EXPECT_EQ(checked->driver, "orbbec");
  EXPECT_EQ(checked->calibration, nullptr);

  const std::string good = config.source.sync_config;
  config.source.sync_config = dir / "missing.json";
  refused(check_scenario(config).status(), Code::NotFound,
          "source.sync_config: ");
  config.source.sync_config =
      write(dir, "bad.json", R"({"devices": [{"sn": 1}]})");
  refused(check_scenario(config).status(), Code::InvalidArgument,
          "source.sync_config: ");
  config.source.sync_config = write(dir, "empty.json", R"({"devices": []})");
  refused(check_scenario(config).status(), Code::InvalidArgument,
          "source.sync_config: " + config.source.sync_config +
              ": sync config: no \"devices\" array");
  config.source.sync_config = good;
  config.source.depth_width = 800;
  config.source.depth_height = 600;
  refused(check_scenario(config).status(), Code::InvalidArgument,
          "800x600 is not a depth mode");
}
#endif  // VC_WITH_ORBBEC

TEST(AppPreset, AcceptsARigOfAnySizeAtThePreset) {
  const RigPreset preset = femto_preset();
  for (const std::size_t cameras : {1u, 3u, 4u, 5u}) {
    const core::Status checked = check_preset(rig(cameras), preset);
    EXPECT_TRUE(checked.ok()) << cameras << ": " << checked.message();
  }
}

TEST(AppPreset, RefusesTheFirstCameraAtAnotherMode) {
  const RigPreset preset = femto_preset();
  refused(check_preset({}, preset), Code::InvalidArgument, "no cameras");

  std::vector<sensor::SensorInfo> cameras = rig(3);
  cameras[1].color = model(1920, 1080);
  refused(check_preset(cameras, preset), Code::Unsupported,
          "camera CL2A141001 streams colour at 1920x1080, not the "
          "scenario's 3840x2160");

  cameras = rig(3);
  cameras[2].depth = model(320, 288);
  refused(check_preset(cameras, preset), Code::Unsupported,
          "camera CL2A141002 streams depth at 320x288, not the scenario's "
          "640x576");

  cameras = rig(4);
  cameras[3].fps = 15;
  refused(check_preset(cameras, preset), Code::Unsupported,
          "camera CL2A141003 streams at 15 fps, not the scenario's 25");

  cameras = rig(1);
  cameras[0].color.reset();
  refused(check_preset(cameras, preset), Code::Unsupported,
          "camera CL2A141000 has no colour camera");
  cameras = rig(1);
  cameras[0].depth.reset();
  refused(check_preset(cameras, preset), Code::Unsupported,
          "camera CL2A141000 has no depth camera");
  cameras = rig(2);
  cameras[1].id.clear();
  cameras[1].fps = 30;
  refused(check_preset(cameras, preset), Code::Unsupported,
          "camera 1 streams at 30 fps");

  // The first fault is the one named.
  cameras = rig(3);
  cameras[0].fps = 30;
  cameras[2].color = model(1280, 720);
  refused(check_preset(cameras, preset), Code::Unsupported,
          "camera CL2A141000 streams at 30 fps");
}

TEST(AppPreset, HoldsEachCameraToAScenariosOtherModes) {
  RigPreset preset = femto_preset();
  preset.color = {1920, 1080};
  preset.depth = {320, 288};
  preset.fps = 15;
  std::vector<sensor::SensorInfo> cameras = rig(2);
  refused(check_preset(cameras, preset), Code::Unsupported,
          "camera CL2A141000 streams colour at 3840x2160, not the "
          "scenario's 1920x1080");
  for (sensor::SensorInfo& info : cameras) {
    info.color = model(1920, 1080);
    info.depth = model(320, 288);
    info.fps = 15;
  }
  EXPECT_TRUE(check_preset(cameras, preset).ok());
}

}  // namespace
}  // namespace volumetric_kit::calib::app
