// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The rig application's use of the capture tier from an installed package: it
// includes the tier's header, so it must compile from the install, and names
// a session directory and refuses a session with none, so it links
// calib_capture and what it links. With the depth tier, it asks for its
// device requirements, so it links calib_depth and what it links.

#include <chrono>

#include "volumetric_kit/calib/capture/session_writer.hpp"
#ifdef VC_RIG_CONSUMER_DEPTH
#include "volumetric_kit/calib/depth/depth_checks.hpp"
#endif

int main() {
  namespace capture = volumetric_kit::calib::capture;
  const bool named =
      capture::session_stamp(std::chrono::system_clock::now()).ok();
  const bool refused =
      capture::SessionWriter::create(capture::SessionInfo{})
          .status()
          .domain() == volumetric_kit::core::Status::Code::InvalidArgument;
  bool depth = true;
#ifdef VC_RIG_CONSUMER_DEPTH
  depth = volumetric_kit::calib::depth::DepthChecks::device_requirements()
              .timeline_semaphore;
#endif
  return named && refused && depth ? 0 : 1;
}
