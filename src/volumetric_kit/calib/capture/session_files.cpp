// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "session_files.hpp"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <optional>
#include <utility>

#include "../core/json_text.hpp"
#include "volumetric_kit/core/base/check.hpp"

namespace volumetric_kit::calib::capture::detail {
namespace {

using calib::detail::JsonWriter;

// The capture's index as its file names give it: three digits or more.
std::string padded(int index) {
  char digits[16];
  std::snprintf(digits, sizeof digits, "%03d", index);
  return digits;
}

// A PGM file's header for a picture of `size`.
std::string pgm_header(sensor::camera::ImageSize size) {
  return "P5\n" + std::to_string(size.width) + " " +
         std::to_string(size.height) + "\n255\n";
}

// A PGM file's size for `pixels` bytes of a picture of `size`.
std::size_t pgm_bytes(std::size_t pixels, sensor::camera::ImageSize size) {
  return pgm_header(size).size() + pixels;
}

// The double of the shortest decimal that reads back to `value`, which the
// writer then writes as that decimal: 0.001 for 0.001f, not the float's own
// double, 0.0010000000474974513.
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

// Each writes one value, after its key: `write_check(w.key("check"), check)`.

// [width, height].
void write_size(JsonWriter& w, sensor::camera::ImageSize size) {
  w.open_array();
  w.value(std::size_t{size.width});
  w.value(std::size_t{size.height});
  w.close_array();
}

void write_pixel(JsonWriter& w, const sensor::camera::Vec2d& p) {
  w.open_array();
  w.value(p.x);
  w.value(p.y);
  w.close_array();
}

void write_detection(JsonWriter& w, const Detection& detection) {
  const std::vector<DetectedPoint>& points = detection.observation.points;
  w.open_object();
  w.key("method").value("full");
  write_size(w.key("image_size"), detection.image_size);
  w.key("pixel_convention").value("opencv");
  w.key("ids").open_array();
  for (const DetectedPoint& point : points) w.value(point.id);
  w.close_array();
  w.key("corners_px").open_array();
  for (const DetectedPoint& point : points) write_pixel(w, point.pixel);
  w.close_array();
  w.key("weights").open_array();
  for (const DetectedPoint& point : points) w.value(point.weight);
  w.close_array();
  w.key("marker_ids").open_array();
  for (const DetectedMarker& marker : detection.markers) w.value(marker.id);
  w.close_array();
  w.key("marker_corners_px").open_array();
  for (const DetectedMarker& marker : detection.markers) {
    w.open_array();
    for (const sensor::camera::Vec2d& corner : marker.corners) {
      write_pixel(w, corner);
    }
    w.close_array();
  }
  w.close_array();
  w.close_object();
}

void write_check(JsonWriter& w, const ViewCheck& check) {
  w.open_object();
  w.key("corners").value(check.corners);
  w.key("markers").value(check.markers);
  w.key("collinear").value(check.collinear);
  w.key("distance_m").value_or_null(check.distance_m);
  w.key("tilt_deg").value_or_null(check.tilt_deg);
  w.key("pnp_rms_px").value_or_null(check.pnp_rms_px);
  w.key("cell").value_or_null(check.cell);
  w.key("verdict").value(to_string(check.verdict));
  w.close_object();
}

void write_origin_tag(JsonWriter& w, const OriginTag& tag,
                      const std::optional<DetectedMarker>& found) {
  if (!found) {
    w.null();
    return;
  }
  w.open_object();
  w.key("dictionary").value(to_string(tag.dictionary));
  w.key("id").value(found->id);
  w.key("corners_px").open_array();
  for (const sensor::camera::Vec2d& corner : found->corners) {
    write_pixel(w, corner);
  }
  w.close_array();
  w.close_object();
}

}  // namespace

bool is_plain_serial(const std::string& id) noexcept {
  if (id.empty() || id == "." || id == "..") return false;
  for (const char ch : id) {
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

std::string color_file(const std::string& id, int index, bool jpeg) {
  return std::string(kImagesDir) + "/" + id + "/calib_img_" + id + "_" +
         padded(index) + (jpeg ? ".jpg" : ".pgm");
}

std::string depth_file(const std::string& id, int index) {
  return std::string(kDepthDir) + "/" + id + "/calib_depth_" + id + "_" +
         padded(index) + ".png";
}

std::string capture_file(int index) {
  return std::string(kCapturesDir) + "/" + padded(index) + ".json";
}

std::string pgm(const std::vector<std::uint8_t>& luma,
                sensor::camera::ImageSize size) {
  std::string bytes = pgm_header(size);
  bytes.append(reinterpret_cast<const char*>(luma.data()), luma.size());
  return bytes;
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

std::string format_session(const SessionInfo& info, const SessionFacts& facts,
                           const std::vector<CaptureEntry>& captures) {
  JsonWriter w;
  w.open_object();
  w.key("format").value("volumetric_kit_calib.session");
  w.key("version").value(2);
  w.key("created").value(facts.created);
  w.key("mode").value(info.mode == SessionMode::Intrinsics ? "intrinsics"
                                                           : "extrinsics");

  const Software& software = info.software;
  w.key("software").open_object();
  w.key("calib").value(software.calib);
  w.key("sensor").value(software.sensor);
  w.key("gfx").value(software.gfx);
  w.key("core").value(software.core);
  w.key("opencv").value(software.opencv);
  if (!software.orbbec_sdk.empty()) {
    w.key("orbbec_sdk").value(software.orbbec_sdk);
  }
  w.close_object();

  w.key("rig").open_object();
  w.key("driver").value(info.driver);
  if (!info.sync_path.empty()) {
    w.key("sync_file").value(info.sync_path);
    w.key("sync_sha256").value(facts.sync_sha256);
  }
  w.key("reference").value(info.reference);
  w.close_object();

  const Target& board = info.board;
  w.key("board").open_object();
  w.key("type").value(to_string(board.kind));
  // OpenCV's CharucoBoard counts squares, x then y: one more than corners.
  w.key("squares").open_array();
  w.value(board.cols + 1);
  w.value(board.rows + 1);
  w.close_array();
  w.key("square_m").value(board.square_size_m);
  // A checkerboard has no markers.
  if (board.kind == TargetKind::ChArUco) {
    w.key("marker_m").value(board.marker_size_m);
    w.key("dictionary").value(to_string(board.dictionary));
    w.key("first_marker_id").value(board.first_marker_id);
    w.key("legacy_pattern").value(board.legacy_pattern);
  }
  w.key("square_measured").value(info.square_measured);
  w.key("square_tolerance_m").value_or_null(board.square_size_tolerance_m);
  w.key("planarity_bow_m").value_or_null(board.planarity_bow_m);
  w.close_object();

  w.key("origin_tag").open_object();
  w.key("dictionary").value(to_string(info.origin_tag.dictionary));
  w.key("id").value(info.origin_tag.id);
  w.key("edge_m").value(info.origin_tag.edge_m);
  w.close_object();

  // Every camera streams what the first does; every depth camera, what the
  // first of them does.
  const SessionCamera& first = info.cameras.front();
  w.key("streams").open_object();
  w.key("color").open_object();
  w.key("width").value(std::size_t{first.size.width});
  w.key("height").value(std::size_t{first.size.height});
  if (!info.color_codec.empty()) w.key("codec").value(info.color_codec);
  w.close_object();
  if (info.depth_mode) {
    const DepthMode& mode = *info.depth_mode;
    const SessionCamera* with_depth = nullptr;
    for (const SessionCamera& camera : info.cameras) {
      if (camera.depth && with_depth == nullptr) with_depth = &camera;
    }
    VKC_CHECK(with_depth != nullptr,
              "format_session: a depth mode with no depth camera");
    w.key("depth").open_object();
    w.key("width").value(std::size_t{with_depth->depth->size.width});
    w.key("height").value(std::size_t{with_depth->depth->size.height});
    w.key("mode").value(mode.name);
    w.key("metres_per_unit").value(shortest(mode.metres_per_unit));
    w.key("min_m").value(shortest(mode.min_m));
    w.key("max_m").value(shortest(mode.max_m));
    w.close_object();
  }
  w.key("fps").value(std::size_t{first.fps});
  w.close_object();

  w.key("cameras").open_array();
  for (std::size_t i = 0; i < info.cameras.size(); ++i) {
    const SessionCamera& camera = info.cameras[i];
    w.open_object();
    w.key("id").value(camera.id);
    w.key("model").value(camera.model);
    if (camera.orbbec) {
      w.key("firmware").value(camera.orbbec->firmware);
      w.key("ip").value(camera.orbbec->ip_address);
    }
    if (!camera.role.empty()) w.key("role").value(camera.role);
    if (camera.orbbec) {
      w.key("sync_mode").value(camera.orbbec->sync_mode);
      w.key("depth_delay_us").value(camera.orbbec->depth_delay_us);
      w.key("color_lens_model").value(camera.orbbec->color_lens_model);
      w.key("depth_lens_model").value(camera.orbbec->depth_lens_model);
    }
    if (!camera.input.empty()) w.key("input").value(camera.input);
    w.key("lens").open_object();
    w.key("origin").value(to_string(camera.lens));
    if (camera.lens == LensOrigin::File) {
      w.key("path").value(camera.lens_file);
      w.key("sha256").value(facts.lens_sha256[i]);
    }
    w.close_object();
    w.close_object();
  }
  w.close_array();
  if (facts.factory) w.key("factory_calibration").value(kFactoryFile);

  w.key("captures").open_array();
  for (const CaptureEntry& capture : captures) {
    w.open_object();
    w.key("index").value(capture.index);
    w.key("file").value(capture_file(capture.index));
    w.key("dropped").value(capture.dropped);
    w.close_object();
  }
  w.close_array();
  w.close_object();
  return std::move(w).text();
}

std::string format_capture(const SessionInfo& info, const CaptureJob& job) {
  const CaptureMeta& meta = job.meta;
  JsonWriter w;
  w.open_object();
  w.key("format").value("volumetric_kit_calib.capture");
  w.key("version").value(1);
  w.key("index").value(meta.index);
  w.key("time").value(iso_time(meta.time_ns));
  w.key("trigger_ns").value(meta.trigger_ns);
  w.key("sequence").value(meta.sequence);
  w.key("spread_ms").value(meta.spread_ms);
  w.key("stillness_px").value(meta.stillness_px);
  w.key("live_vs_full_px").value_or_null(meta.live_vs_full_px);

  // Keyed by id, which no two cameras share (SessionWriter::create).
  w.key("cameras").open_object();
  int usable_views = 0;
  for (std::size_t i = 0; i < info.cameras.size(); ++i) {
    const SessionCamera& camera = info.cameras[i];
    const std::string& id = camera.id;
    const CameraCapture& part = job.cameras[i];
    const bool jpeg = !part.jpeg.empty();
    w.key(id).open_object();
    w.key("color").value(color_file(id, meta.index, jpeg));
    if (camera.depth) {
      w.key("depth").value(depth_file(id, meta.index));
    } else {
      w.key("depth").null();
    }
    w.key("color_bytes")
        .value(jpeg ? part.jpeg.size()
                    : pgm_bytes(part.luma.size(), camera.size));
    w.key("timestamp_ns").value(part.timestamp_ns);
    w.key("sequence").value(part.sequence);
    write_detection(w.key("detection"), part.detection);
    write_check(w.key("check"), part.check);
    write_origin_tag(w.key("origin_tag"), info.origin_tag, part.origin_tag);
    w.close_object();
    if (is_usable(part.check.verdict)) ++usable_views;
  }
  w.close_object();
  w.key("usable_views").value(usable_views);
  w.close_object();
  return std::move(w).text();
}

}  // namespace volumetric_kit::calib::capture::detail
