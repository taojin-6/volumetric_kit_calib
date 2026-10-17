// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The rig application's use of the capture and depth tiers from an installed
// package: it includes their headers, so they must compile from the install,
// names a session directory and refuses a session with none, so it links
// calib_capture and what it links, and asks for the depth tier's device
// requirements, so it links calib_depth and what it links.

#include <chrono>

#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/calib/depth/depth_checks.hpp"

int main() {
  namespace capture = volumetric_kit::calib::capture;
  const bool named =
      capture::session_stamp(std::chrono::system_clock::now()).ok();
  const bool refused =
      capture::SessionWriter::create(capture::SessionInfo{})
          .status()
          .domain() == volumetric_kit::core::Status::Code::InvalidArgument;
  const bool depth =
      volumetric_kit::calib::depth::DepthChecks::device_requirements()
          .timeline_semaphore;
  return named && refused && depth ? 0 : 1;
}
