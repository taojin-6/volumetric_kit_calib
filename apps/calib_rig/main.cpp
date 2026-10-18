// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig: the live rig calibration application. Every camera of the rig
// live, the board found in each, and captures saved to a session on disk
// (calib_app's calib_rig.hpp).
//
//   calib_rig --config my_rig.json
//   calib_rig --help

#include <cstdio>
#include <string>

#include "volumetric_kit/calib/app/calib_rig.hpp"
#include "volumetric_kit/core/config/config.hpp"

namespace vkc = volumetric_kit::core;

namespace {

vkc::Status launch(int argc, const char* const* argv) {
  VKC_ASSIGN(const auto arguments, vkc::config::parse_args(argc, argv));
  if (arguments.help) {
    std::printf("%s", vkc::config::usage());
    return {};
  }
  VKC_ASSIGN(const auto config,
             vkc::config::load(arguments.config_path,
                               vkc::config::Application::CalibRig));
  return volumetric_kit::calib::app::run_calib_rig(config);
}

}  // namespace

int main(int argc, char** argv) {
  const vkc::Status status = launch(argc, argv);
  if (!status) {
    std::fprintf(stderr, "error: %s: %s\n",
                 std::string(vkc::to_string(status.domain())).c_str(),
                 status.message().c_str());
    return 1;
  }
  return 0;
}
