// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Internal: the names and the text of a session's files (session_writer.hpp
// shows the layout and both schemas). Each function may throw what an
// allocation, or nlohmann/json, throws; the writer turns it into a Status.

#include <cstdint>
#include <string>
#include <vector>

#include "volumetric_kit/calib/capture/session_writer.hpp"

namespace volumetric_kit::calib::capture::detail {

inline constexpr const char* kSessionFile = "session.json";
inline constexpr const char* kFactoryFile = "factory_calibration.json";
inline constexpr const char* kImagesDir = "images";
inline constexpr const char* kDepthDir = "depth";
inline constexpr const char* kCapturesDir = "captures";

// One capture as session.json lists it.
struct CaptureEntry {
  int index = 0;
  bool dropped = false;
};

// Whether `serial` can name a camera's directories and files: one or more of
// letters, digits, '-', '_' and '.', and not "." or "..".
bool is_plain_serial(const std::string& serial) noexcept;

// `directory`/`name`, with one separator.
std::string join(const std::string& directory, const std::string& name);

// A capture's files, relative to the session's directory.
std::string color_file(const std::string& serial, int index);
std::string depth_file(const std::string& serial, int index);
std::string capture_file(int index);

// `ns` since the epoch as ISO 8601 UTC to the millisecond, e.g.
// "2026-10-13T09:41:07.250Z".
std::string iso_time(std::int64_t ns);

// session.json's text, for an `info` SessionWriter::create accepted.
std::string format_session(const SessionInfo& info, const std::string& created,
                           const std::string& sync_sha256,
                           const std::vector<CaptureEntry>& captures);

// captures/<NNN>.json's text, for `job` with its index set, which
// SessionWriter::submit accepted.
std::string format_capture(const SessionInfo& info, const CaptureJob& job);

}  // namespace volumetric_kit::calib::capture::detail
