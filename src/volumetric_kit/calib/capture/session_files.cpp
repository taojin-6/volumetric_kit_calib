// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "session_files.hpp"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <optional>
#include <utility>

#include <nlohmann/json.hpp>

#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"

namespace volumetric_kit::calib::capture::detail {
namespace {

// TODO: write these files with core/json_text.hpp's JsonWriter, as the
// report is (DECISIONS.md, 2026-10-16): nlohmann's containers built here, on
// the writer thread, allocate in their destructors and so terminate when
// memory runs out.
using json = nlohmann::ordered_json;

// The capture's index as its file names give it: three digits or more.
std::string padded(int index) {
  char digits[16];
  std::snprintf(digits, sizeof digits, "%03d", index);
  return digits;
}

// The double of the shortest decimal that reads back to `value`, which
// nlohmann/json, writing doubles, then writes as that decimal: 0.001 for
// 0.001f, not the float's own double, 0.0010000000474974513.
double shortest(float value) {
  char text[32];
  // Nine significant digits always read back to the float.
  for (int digits = 1; digits <= 9; ++digits) {
    std::snprintf(text, sizeof text, "%.*g", digits,
                  static_cast<double>(value));
    const double parsed = std::strtod(text, nullptr);
    if (static_cast<float>(parsed) == value) return parsed;
  }
  return static_cast<double>(value);
}

json optional_number(const std::optional<double>& value) {
  return value ? json(*value) : json(nullptr);
}

json pixel(const recon::camera::Vec2d& p) { return json::array({p.x, p.y}); }

json detection_json(const Detection& detection) {
  json ids = json::array();
  json corners = json::array();
  json weights = json::array();
  for (const DetectedPoint& point : detection.observation.points) {
    ids.push_back(point.id);
    corners.push_back(pixel(point.pixel));
    weights.push_back(point.weight);
  }
  json marker_ids = json::array();
  json marker_corners = json::array();
  for (const DetectedMarker& marker : detection.markers) {
    marker_ids.push_back(marker.id);
    json outline = json::array();
    for (const recon::camera::Vec2d& corner : marker.corners) {
      outline.push_back(pixel(corner));
    }
    marker_corners.push_back(std::move(outline));
  }
  json j;
  j["method"] = "full";
  j["image_size"] =
      json::array({detection.image_size.width, detection.image_size.height});
  j["pixel_convention"] = "opencv";
  j["ids"] = std::move(ids);
  j["corners_px"] = std::move(corners);
  j["weights"] = std::move(weights);
  j["marker_ids"] = std::move(marker_ids);
  j["marker_corners_px"] = std::move(marker_corners);
  return j;
}

json check_json(const ViewCheck& check) {
  json j;
  j["corners"] = check.corners;
  j["markers"] = check.markers;
  j["collinear"] = check.collinear;
  j["distance_m"] = optional_number(check.distance_m);
  j["tilt_deg"] = optional_number(check.tilt_deg);
  j["pnp_rms_px"] = optional_number(check.pnp_rms_px);
  j["cell"] = check.cell ? json(*check.cell) : json(nullptr);
  j["verdict"] = to_string(check.verdict);
  return j;
}

json origin_tag_json(const OriginTag& tag,
                     const std::optional<DetectedMarker>& found) {
  if (!found) return nullptr;
  json corners = json::array();
  for (const recon::camera::Vec2d& corner : found->corners) {
    corners.push_back(pixel(corner));
  }
  json j;
  j["dictionary"] = to_string(tag.dictionary);
  j["id"] = found->id;
  j["corners_px"] = std::move(corners);
  return j;
}

// Strings from a driver or an application may not be UTF-8: their bad bytes
// become U+FFFD rather than a throw.
std::string text_of(const json& doc) {
  return doc.dump(2, ' ', false, json::error_handler_t::replace) + "\n";
}

}  // namespace

bool is_plain_serial(const std::string& serial) noexcept {
  if (serial.empty() || serial == "." || serial == "..") return false;
  for (const char ch : serial) {
    const bool plain = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                       (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' ||
                       ch == '.';
    if (!plain) return false;
  }
  return true;
}

std::string join(const std::string& directory, const std::string& name) {
  if (!directory.empty() && directory.back() == '/') return directory + name;
  return directory + "/" + name;
}

std::string color_file(const std::string& serial, int index) {
  return std::string(kImagesDir) + "/" + serial + "/calib_img_" + serial + "_" +
         padded(index) + ".jpg";
}

std::string depth_file(const std::string& serial, int index) {
  return std::string(kDepthDir) + "/" + serial + "/calib_depth_" + serial +
         "_" + padded(index) + ".png";
}

std::string capture_file(int index) {
  return std::string(kCapturesDir) + "/" + padded(index) + ".json";
}

std::string iso_time(std::int64_t ns) {
  constexpr std::int64_t kNsPerSecond = 1'000'000'000;
  std::int64_t seconds = ns / kNsPerSecond;
  std::int64_t rest = ns % kNsPerSecond;
  if (rest < 0) {  // before the epoch: round down, not toward zero
    rest += kNsPerSecond;
    --seconds;
  }
  const auto time = static_cast<std::time_t>(seconds);
  std::tm utc{};
  // Every int64_t of nanoseconds is within years 1677-2262.
  VKC_CHECK(::gmtime_r(&time, &utc) != nullptr, "iso_time: gmtime_r failed");
  char text[48];
  const std::size_t date =
      std::strftime(text, sizeof text, "%Y-%m-%dT%H:%M:%S", &utc);
  std::snprintf(text + date, sizeof text - date, ".%03dZ",
                static_cast<int>(rest / 1'000'000));
  return text;
}

std::string format_session(const SessionInfo& info, const std::string& created,
                           const std::string& sync_sha256,
                           const std::vector<CaptureEntry>& captures) {
  json doc;
  doc["format"] = "volumetric_kit_calib.session";
  doc["version"] = 1;
  doc["created"] = created;

  json& software = doc["software"];
  software["calib"] = info.software.calib;
  software["recon"] = info.software.recon;
  software["gfx"] = info.software.gfx;
  software["core"] = info.software.core;
  software["opencv"] = info.software.opencv;
  software["orbbec_sdk"] = info.software.orbbec_sdk;

  json& rig = doc["rig"];
  rig["sync_file"] = info.sync_path;
  rig["sync_sha256"] = sync_sha256;
  rig["reference"] = info.reference;

  const Target& board = info.board;
  json& b = doc["board"];
  b["type"] = to_string(board.kind);
  // OpenCV's CharucoBoard counts squares, x then y: one more than corners.
  b["squares"] = json::array({board.cols + 1, board.rows + 1});
  b["square_m"] = board.square_size_m;
  b["marker_m"] = board.marker_size_m;
  b["dictionary"] = to_string(board.dictionary);
  b["first_marker_id"] = board.first_marker_id;
  b["legacy_pattern"] = board.legacy_pattern;
  b["square_measured"] = info.square_measured;
  b["square_tolerance_m"] = optional_number(board.square_size_tolerance_m);
  b["planarity_bow_m"] = optional_number(board.planarity_bow_m);

  json& tag = doc["origin_tag"];
  tag["dictionary"] = to_string(info.origin_tag.dictionary);
  tag["id"] = info.origin_tag.id;
  tag["edge_m"] = info.origin_tag.edge_m;

  // Every camera streams what the first does.
  const recon::sensor::SensorInfo& first = info.cameras.front().info;
  const DepthMode& mode = info.depth_mode;
  json& streams = doc["streams"];
  json& color = streams["color"];
  color["width"] = first.color->size.width;
  color["height"] = first.color->size.height;
  color["codec"] = "mjpeg";
  json& depth = streams["depth"];
  depth["width"] = first.depth->size.width;
  depth["height"] = first.depth->size.height;
  depth["mode"] = mode.name;
  depth["metres_per_unit"] = shortest(mode.metres_per_unit);
  depth["min_m"] = shortest(mode.min_m);
  depth["max_m"] = shortest(mode.max_m);
  streams["fps"] = first.fps;

  json cameras = json::array();
  for (const SessionCamera& camera : info.cameras) {
    json c;
    c["serial"] = camera.info.id;
    c["model"] = camera.info.model;
    c["firmware"] = camera.firmware;
    c["ip"] = camera.ip_address;
    c["role"] = recon::sensor::to_string(camera.info.role);
    c["sync_mode"] = camera.sync_mode;
    c["depth_delay_us"] = camera.depth_delay_us;
    c["color_lens_model"] = camera.color_lens_model;
    c["depth_lens_model"] = camera.depth_lens_model;
    cameras.push_back(std::move(c));
  }
  doc["cameras"] = std::move(cameras);
  doc["factory_calibration"] = kFactoryFile;

  json listed = json::array();
  for (const CaptureEntry& capture : captures) {
    json c;
    c["index"] = capture.index;
    c["file"] = capture_file(capture.index);
    c["dropped"] = capture.dropped;
    listed.push_back(std::move(c));
  }
  doc["captures"] = std::move(listed);
  return text_of(doc);
}

std::string format_capture(const SessionInfo& info, const CaptureJob& job) {
  const CaptureMeta& meta = job.meta;
  json doc;
  doc["format"] = "volumetric_kit_calib.capture";
  doc["version"] = 1;
  doc["index"] = meta.index;
  doc["time"] = iso_time(meta.time_ns);
  doc["trigger_ns"] = meta.trigger_ns;
  doc["sequence"] = meta.sequence;
  doc["spread_ms"] = meta.spread_ms;
  doc["stillness_px"] = meta.stillness_px;
  doc["live_vs_full_px"] = optional_number(meta.live_vs_full_px);

  json cameras = json::object();
  int usable_views = 0;
  for (std::size_t i = 0; i < info.cameras.size(); ++i) {
    const std::string& serial = info.cameras[i].info.id;
    const CameraCapture& part = job.cameras[i];
    json c;
    c["color"] = color_file(serial, meta.index);
    c["depth"] = depth_file(serial, meta.index);
    c["color_bytes"] = part.jpeg.size();
    c["timestamp_ns"] = part.timestamp_ns;
    c["sequence"] = part.sequence;
    c["detection"] = detection_json(part.detection);
    c["check"] = check_json(part.check);
    c["origin_tag"] = origin_tag_json(info.origin_tag, part.origin_tag);
    cameras[serial] = std::move(c);
    if (is_usable(part.check.verdict)) ++usable_views;
  }
  doc["cameras"] = std::move(cameras);
  doc["usable_views"] = usable_views;
  return text_of(doc);
}

}  // namespace volumetric_kit::calib::capture::detail
