// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The rig application's use of the capture tier from an installed package: it
// includes the tier's header, so it must compile from the install, and names
// a session directory and refuses a session with none, so it links
// calib_capture and what it links.

#include <chrono>

#include "volumetric_kit/calib/capture/session_writer.hpp"

int main() {
  namespace capture = volumetric_kit::calib::capture;
  const bool named =
      capture::session_stamp(std::chrono::system_clock::now()).ok();
  const bool refused =
      capture::SessionWriter::create(capture::SessionInfo{})
          .status()
          .domain() == volumetric_kit::core::Status::Code::InvalidArgument;
  return named && refused ? 0 : 1;
}
