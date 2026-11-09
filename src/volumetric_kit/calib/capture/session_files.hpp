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

// What session.json records beside the SessionInfo it was made with: the
// files the writer read, and whether it wrote the factory record.
struct SessionFacts {
  std::string created;      // ISO 8601, at the directory's creation
  std::string sync_sha256;  // the sync file's; empty without one
  // Each camera's lens file's SHA-256, by camera; empty where its lens came
  // from no file.
  std::vector<std::string> lens_sha256;
  bool factory = false;  // factory_calibration.json was written
};

// Whether `id` can name a camera's directories and files: one or more of
// letters, digits, '-', '_' and '.', and not "." or "..".
bool is_plain_serial(const std::string& id) noexcept;

// `directory`/`name`, with one separator.
std::string join(const std::string& directory, const std::string& name);

// What a camera's picture is written as: its JPEG, a PNG still, or its luma
// as a PGM.
enum class PictureKind { Jpeg, Png, Pgm };

// The kind `part` is written as: its JPEG, else its PNG, else its luma.
PictureKind picture_kind(const CameraCapture& part) noexcept;

// A capture's files, relative to the session's directory: the camera's
// picture, named for a calibration's images or, in a scan session, a still's,
// and its depth.
std::string color_file(const std::string& id, int index, PictureKind kind,
                       SessionMode mode);
std::string depth_file(const std::string& id, int index);
std::string capture_file(int index);

// `luma` (`size`'s rows, packed) as a PGM file's bytes: "P5", the size and
// 255, a newline after each, then the rows.
std::string pgm(const std::vector<std::uint8_t>& luma,
                sensor::camera::ImageSize size);

// The bytes `part`'s picture file takes, as `kind` writes it for a camera of
// `size`.
std::size_t picture_bytes(const CameraCapture& part, PictureKind kind,
                          sensor::camera::ImageSize size);

// `ns` since the epoch as ISO 8601 UTC to the millisecond, e.g.
// "2026-10-13T09:41:07.250Z".
std::string iso_time(std::int64_t ns);

// session.json's text, for an `info` SessionWriter::create accepted.
std::string format_session(const SessionInfo& info, const SessionFacts& facts,
                           const std::vector<CaptureEntry>& captures);

// captures/<NNN>.json's text, for `job` with its index set, which
// SessionWriter::submit accepted.
std::string format_capture(const SessionInfo& info, const CaptureJob& job);

}  // namespace volumetric_kit::calib::capture::detail
