// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's preset: the Femto Mega's depth modes, the preset a scenario asks
// for, what calib_rig refuses of a scenario before it opens anything, and the
// check that every camera of a rig of any size opened at the preset, naming
// the first that did not. Host-only.

#include "volumetric_kit/calib/app/preset.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "volumetric_kit/calib/app/calib_rig.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = recon::camera;
namespace sensor = recon::sensor;
using Code = core::Status::Code;

camera::CameraModel model(std::uint32_t width, std::uint32_t height) {
  camera::CameraModel m;
  m.size = {width, height};
  m.intrinsics = {0.6 * width, 0.6 * width, 0.5 * width, 0.5 * height};
  return m;
}

// A camera that opened at the default preset.
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
    EXPECT_EQ(mode->metres_per_unit, 0.001f) << m.name;
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
  EXPECT_EQ(preset->depth_mode.name, "nfov_unbinned");

  source.color_width = 1920;
  source.color_height = 1080;
  source.depth_width = 320;
  source.depth_height = 288;
  source.fps = 15;
  preset = rig_preset(source);
  ASSERT_TRUE(preset.ok()) << preset.status().message();
  EXPECT_EQ(preset->color.width, 1920u);
  EXPECT_EQ(preset->depth.height, 288u);
  EXPECT_EQ(preset->fps, 15u);
  EXPECT_EQ(preset->depth_mode.name, "nfov_binned");
}

TEST(AppPreset, RefusesHevcAndADepthSizeWithNoMode) {
  core::config::Source source;
  source.color_codec = core::config::ColorCodec::Hevc;
  refused(rig_preset(source).status(), Code::InvalidArgument,
          "source.color_codec: calib_rig streams MJPEG");
  source.color_codec = core::config::ColorCodec::Mjpeg;
  source.depth_width = 800;
  source.depth_height = 600;
  refused(rig_preset(source).status(), Code::InvalidArgument,
          "source.depth_width/depth_height: 800x600");
}

TEST(AppPreset, ChecksTheScenarioBeforeOpeningAnything) {
  core::config::Config config;
  config.source.color_codec = core::config::ColorCodec::Mjpeg;
  config.output.path = (std::filesystem::temp_directory_path() /
                        "vc_app_preset_no_such_directory")
                           .string();
  std::error_code ec;
  std::filesystem::remove_all(config.output.path, ec);
  refused(
      check_scenario(config).status(), Code::NotFound,
      "output.path: " + config.output.path + " is not an existing directory");
  config.output.path = std::filesystem::temp_directory_path().string();
  EXPECT_TRUE(check_scenario(config).ok());
  config.source.color_codec = core::config::ColorCodec::Hevc;
  refused(check_scenario(config).status(), Code::InvalidArgument,
          "source.color_codec");
}

TEST(AppPreset, AcceptsARigOfAnySizeAtThePreset) {
  const RigPreset preset;
  for (const std::size_t cameras : {1u, 3u, 4u, 5u}) {
    const core::Status checked = check_preset(rig(cameras), preset);
    EXPECT_TRUE(checked.ok()) << cameras << ": " << checked.message();
  }
}

TEST(AppPreset, RefusesTheFirstCameraAtAnotherMode) {
  const RigPreset preset;
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
  RigPreset preset;
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
