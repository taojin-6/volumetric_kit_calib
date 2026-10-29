// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file session_writer.hpp
/// @brief A calibration session on disk: the rig, and each capture's RGB-D
///        set, detections and checks, written on a thread of its own and
///        never overwritten.
///
/// A session is a directory, created at its first capture:
///
/// @code{.txt}
/// session.json                 the rig, the board, the captures
/// factory_calibration.json     sensor's calibration file, unposed
/// images/<serial>/calib_img_<serial>_<NNN>.jpg
///                              the camera's JPEG, byte for byte
/// depth/<serial>/calib_depth_<serial>_<NNN>.png
///                              16-bit grey, the sensor's units
/// captures/<NNN>.json          times, detections and checks
/// @endcode
///
/// - `<NNN>` is the capture's index, zero-padded to three digits. The image
///   names are the ones the legacy `camera_calibration` scripts glob for.
/// - `factory_calibration.json` is sensor's version-2 calibration file
///   (`sensor/camera/array_calibration.hpp`): each camera's factory colour and
///   depth models at the streamed mode, and its `depth_to_color`, with no pose.
/// - Depth is as the sensor captured it: raw, not undistorted.
/// - `streams` records the cameras' colour and depth sizes and rate, which
///   every camera shares, and the @ref DepthMode.
/// - **Never overwritten.** The directory is new, and every file in it but
///   `session.json` is created once, refusing one already there.
///   `session.json`, which lists the captures, is replaced atomically after
///   each capture and each drop, so it never lists a capture whose files are
///   not all written. A capture that fails is not listed, and its index is
///   not used again.
/// - Numbers are written in the shortest form that reads back exactly; a
///   `float` in the shortest form that reads back to the same `float`.
/// - Nothing reads a session back (DECISIONS.md, 2026-10-13).
///
/// `session.json`:
///
/// @code{.json}
/// {"format": "volumetric_kit_calib.session", "version": 1,
///  "created": "2026-10-13T09:41:07.250Z",
///  "software": {"calib": "...", "sensor": "...", "gfx": "...", "core": "...",
///               "opencv": "5.0.0", "orbbec_sdk": "2.10.6"},
///  "rig": {"sync_file": "config/femto_mega_sync.json",
///          "sync_sha256": "...", "reference": "CL2A141000N"},
///  "board": {"type": "charuco", "squares": [5, 3], "square_m": 0.15,
///            "marker_m": 0.112, "dictionary": "DICT_4X4_100",
///            "first_marker_id": 0, "legacy_pattern": true,
///            "square_measured": false, "square_tolerance_m": null,
///            "planarity_bow_m": null},
///  "origin_tag": {"dictionary": "DICT_APRILTAG_36h11", "id": 0,
///                 "edge_m": 0.15},
///  "streams": {"color": {"width": 3840, "height": 2160, "codec": "mjpeg"},
///              "depth": {"width": 640, "height": 576,
///                        "mode": "nfov_unbinned", "metres_per_unit": 0.001,
///                        "min_m": 0.5, "max_m": 3.86},
///              "fps": 25},
///  "cameras": [{"serial": "CL2A141000N", "model": "Orbbec Femto Mega",
///               "firmware": "...", "ip": "192.168.1.10", "role": "primary",
///               "sync_mode": "primary", "depth_delay_us": 0,
///               "color_lens_model": "brown-conrady",
///               "depth_lens_model": "brown-conrady-k6"}],
///  "factory_calibration": "factory_calibration.json",
///  "captures": [{"index": 0, "file": "captures/000.json",
///                "dropped": false}]}
/// @endcode
///
/// `captures/<NNN>.json`, with `null` for a measure that is empty and for an
/// origin tag the camera did not find; `usable_views` counts the cameras whose
/// verdict @ref is_usable accepts:
///
/// @code{.json}
/// {"format": "volumetric_kit_calib.capture", "version": 1,
///  "index": 0, "time": "2026-10-13T09:41:12.004Z",
///  "trigger_ns": 1760348472003911000, "sequence": 1234, "spread_ms": 1.4,
///  "stillness_px": 0.08, "live_vs_full_px": 0.12,
///  "cameras": {"CL2A141000N": {
///    "color": "images/CL2A141000N/calib_img_CL2A141000N_000.jpg",
///    "depth": "depth/CL2A141000N/calib_depth_CL2A141000N_000.png",
///    "color_bytes": 1620417, "timestamp_ns": 1760348472003911000,
///    "sequence": 1234,
///    "detection": {"method": "full", "image_size": [3840, 2160],
///                  "pixel_convention": "opencv", "ids": [0, 1],
///                  "corners_px": [[812.4, 390.1], [1012.7, 391.3]],
///                  "weights": [1, 1], "marker_ids": [0],
///                  "marker_corners_px": [[[700.2, 300.4], [780.1, 300.9],
///                                         [779.6, 380.2], [699.8, 379.8]]]},
///    "check": {"corners": 8, "markers": 7, "collinear": false,
///              "distance_m": 2.8, "tilt_deg": 12.0, "pnp_rms_px": 0.21,
///              "cell": 4, "verdict": "good"},
///    "origin_tag": {"dictionary": "DICT_APRILTAG_36h11", "id": 0,
///                   "corners_px": [[1904.3, 1201.7], [2010.6, 1203.2],
///                                  [2008.9, 1287.4], [1902.1, 1285.0]]}}},
///  "usable_views": 1}
/// @endcode

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/capture/export.hpp"
#include "volumetric_kit/calib/core/capture_meta.hpp"
#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/core/view_check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace volumetric_kit::calib::capture {

/// @brief What a session was captured with, as `session.json` records it:
///        each a version or commit as the application knows it, or empty.
///
/// @code
/// Software software;
/// software.opencv = CV_VERSION;
/// software.orbbec_sdk = "2.10.6";
/// @endcode
struct Software {
  std::string calib;       ///< calib's version or commit.
  std::string sensor;      ///< volumetric_kit_sensor's.
  std::string gfx;         ///< volumetric_kit_gfx's.
  std::string core;        ///< volumetric_kit_core's.
  std::string opencv;      ///< OpenCV's version.
  std::string orbbec_sdk;  ///< The Orbbec SDK's version.
};

/// @brief The rig's depth mode: what each camera's `SensorInfo` does not say.
///
/// The image sizes and the rate are the cameras' own (their models and
/// `fps`), and colour is the cameras' MJPEG.
///
/// @code
/// DepthMode depth_mode;
/// depth_mode.name = "nfov_unbinned";
/// depth_mode.min_m = 0.5f;
/// depth_mode.max_m = 3.86f;
/// @endcode
struct DepthMode {
  std::string name;  ///< As the camera maker names it, e.g. `"nfov_unbinned"`.
  /// What one depth unit is (metres), as `RgbdFrame::metres_per_unit`.
  float metres_per_unit = 0.001f;
  float min_m = 0.0f;  ///< The nearest depth kept (metres).
  float max_m = 0.0f;  ///< The farthest depth kept (metres).
};

/// @brief One camera of the rig.
///
/// The vendor's facts are strings an application fills from its driver (for
/// an Orbbec camera, sensor's `OrbbecDeviceInfo` and sync configuration), so
/// this tier names no driver.
///
/// @code
/// SessionCamera camera;
/// camera.info = sensor.info();  // serial, model, factory models, role
/// camera.firmware = device.firmware_version;
/// camera.ip_address = device.ip_address;
/// camera.sync_mode = to_string(device.sync_mode);
/// camera.depth_delay_us = sync.depth_delay_us;
/// camera.color_lens_model = device.color_lens_model;
/// camera.depth_lens_model = device.depth_lens_model;
/// @endcode
struct SessionCamera {
  /// What the sensor is: `id` (its serial), `model`, the factory `color` and
  /// `depth` models at the streamed mode, `depth_to_color`, `role` and `fps`.
  sensor::SensorInfo info;
  std::string firmware;             ///< Its firmware version.
  std::string ip_address;           ///< Its IP address; empty for a USB camera.
  std::string sync_mode;            ///< Its sync mode as its driver names it.
  std::int32_t depth_delay_us = 0;  ///< Its depth's delay after the trigger.
  std::string color_lens_model;     ///< The vendor's colour lens model.
  std::string depth_lens_model;     ///< The vendor's depth lens model.
};

/// @brief A session: where it goes, and what it records once.
///
/// @code
/// SessionInfo info;
/// info.directory = out + "/" + session_stamp(now).value();
/// info.sync_path = "config/femto_mega_sync.json";
/// info.reference = array.sensor(array.primary()).info().id;
/// info.board = legacy_femto_board();
/// info.depth_mode = depth_mode;
/// info.cameras = cameras;
/// @endcode
struct SessionInfo {
  /// The session's directory, created at the first capture: nothing may be
  /// there then, and its parent must exist.
  std::string directory;
  Software software;  ///< What the session was captured with.
  /// The rig's sync configuration, whose path and SHA-256 `session.json`
  /// records; read at the first capture.
  std::string sync_path;
  /// The reference camera's serial: the rig's primary, whose colour camera
  /// is the solve's world.
  std::string reference;
  Target board;  ///< The board: a ChArUco board @ref check_target accepts.
  /// Whether `board.square_size_m` was measured, rather than the print's
  /// nominal size.
  bool square_measured = false;
  /// The tag each camera's colour image was searched for
  /// (@ref CameraCapture::origin_tag): one @ref check_origin_tag accepts.
  OriginTag origin_tag = kOriginTag;
  DepthMode depth_mode;  ///< The rig's depth mode.
  /// The cameras, in the order every capture gives them. Every camera
  /// streams the same colour and depth sizes at the same rate.
  std::vector<SessionCamera> cameras;
};

/// @brief One camera's part of a capture.
///
/// The detections and the check are the caller's: this tier cannot link
/// `detect`, so an application runs the full-resolution detections on a
/// thread of its own before it submits the capture.
///
/// @code
/// CameraCapture part;
/// part.jpeg.assign(frame.color_bitstream.data,
///                  frame.color_bitstream.data + frame.color_bitstream.size);
/// part.depth = std::make_shared<const std::vector<std::uint16_t>>(
///     frame.depth, frame.depth + depth_samples);
/// part.timestamp_ns = frame.timestamp_ns;
/// part.sequence = frame.sequence;
/// part.detection = std::move(full_detection);
/// part.check = check;
/// part.origin_tag = tag_search.found;  // detect::detect_tag's
/// @endcode
struct CameraCapture {
  /// The camera's JPEG (`RgbdFrame::color_bitstream`), copied out of the
  /// frame so the driver's buffers go back to it.
  std::vector<std::uint8_t> jpeg;
  /// The depth, a sample for each of the depth model's pixels, row-major, in
  /// the sensor's units, as `RgbdFrame::depth`; not null. Shared, so a caller
  /// may keep the samples it hands over without a copy: the writer only reads
  /// them, and lets them go once the capture is carried out.
  std::shared_ptr<const std::vector<std::uint16_t>> depth;
  std::uint64_t timestamp_ns = 0;  ///< `RgbdFrame::timestamp_ns`.
  std::uint64_t sequence = 0;      ///< `RgbdFrame::sequence`.
  /// The board found in the colour image at full resolution: what the solver
  /// takes. Its `image_size` is the colour image's. The observation's own
  /// camera and frame are not written: the capture's file holds the
  /// detection under the camera's serial.
  Detection detection;
  ViewCheck check;  ///< What the view is worth.
  /// The origin tag (@ref SessionInfo::origin_tag) found in the colour image
  /// at full resolution, in its pixels; empty when the camera found none, or
  /// more than one print of it.
  std::optional<DetectedMarker> origin_tag;
};

/// @brief One capture: a set from every camera, and its set-level facts.
///
/// @code
/// CaptureJob job;
/// job.meta.time_ns = now_ns;  // std::chrono::system_clock
/// job.meta.trigger_ns = set.timestamp_ns;
/// job.meta.sequence = set.sequence;
/// job.meta.spread_ms = set.spread_ns() / 1e6;
/// job.cameras = std::move(parts);  // in SessionInfo::cameras order
/// @endcode
struct CaptureJob {
  /// The set-level facts. `meta.index` is the writer's to set.
  CaptureMeta meta;
  /// One per camera, in @ref SessionInfo::cameras order.
  std::vector<CameraCapture> cameras;
};

/// @brief What became of a request made of a @ref SessionWriter.
///
/// @code
/// for (const WriteResult& done : writer->take_results()) {
///   if (!done.status.ok()) {
///     show(done.status.message());
///   } else if (done.kind == WriteResult::Kind::Capture) {
///     flash("capture " + std::to_string(done.index) + " saved");
///   }
/// }
/// @endcode
struct WriteResult {
  /// @brief What was asked.
  enum class Kind {
    Capture,  ///< A capture (@ref SessionWriter::submit).
    Drop,     ///< A drop (@ref SessionWriter::drop_last).
  };
  Kind kind = Kind::Capture;  ///< What was asked.
  /// The capture written or dropped; -1 when none was.
  int index = -1;
  core::Status status;  ///< OK, or why it failed.
};

/// @brief How a @ref SessionWriter queues.
///
/// @code
/// SessionWriterOptions options;
/// options.queue_capacity = 8;  // at most 8 captures' frames held
/// VKC_ASSIGN(std::unique_ptr<SessionWriter> writer,
///            SessionWriter::create(std::move(info), options));
/// @endcode
struct SessionWriterOptions {
  /// Captures held at once, queued or being written; at least 1.
  std::size_t queue_capacity = 4;
};

/// @brief Writes a session on a thread of its own: a capture or a drop is
///        queued, and its outcome is taken later, so no caller waits on the
///        disk.
///
/// It encodes and writes; it detects nothing (@ref CameraCapture). Requests
/// are carried out in the order they are made. Every member may be called
/// from any thread. Destroying the writer first carries out every request
/// queued.
///
/// @code
/// VKC_ASSIGN(std::unique_ptr<SessionWriter> writer,
///            SessionWriter::create(std::move(info)));
/// VKC_ASSIGN(const bool queued, writer->submit(std::move(job)));
/// if (!queued) show("capture refused: the writer is busy");
/// VKC_TRY(writer->drop_last());  // the clicker's "back"
/// for (const WriteResult& done : writer->take_results()) {
///   if (!done.status.ok()) show(done.status.message());
/// }
/// @endcode
class VC_CAPTURE_API SessionWriter {
 public:
  /// @brief Check @p info and start the writer's thread. Nothing is written
  ///        until the first capture.
  /// @param info     The session. Refused unless: the directory and sync path
  ///                 are non-empty; the board is a ChArUco board
  ///                 @ref check_target accepts, and the origin tag one
  ///                 @ref check_origin_tag accepts; the depth mode has a
  ///                 finite, positive `metres_per_unit` and finite
  ///                 `0 < min_m < max_m`; each serial is a plain file name
  ///                 (letters, digits, `-`, `_`, `.`; not `.` or `..`), and
  ///                 no two differ only in case; each camera has colour and
  ///                 depth models that, with its `depth_to_color`, sensor's
  ///                 calibration file holds (which also needs a camera at
  ///                 least, and no serial twice); every camera streams the
  ///                 same sizes at the same, known `fps`; and the reference
  ///                 is a camera.
  /// @param options  How it queues.
  /// @return The writer; `InvalidArgument` naming what @p info or @p options
  ///         gets wrong; `Unsupported` for a board that is not ChArUco;
  ///         `IoError` if the thread cannot start; or `OutOfMemory`.
  static core::Result<std::unique_ptr<SessionWriter>> create(
      SessionInfo info, SessionWriterOptions options = {});

  /// @brief Carry out every request queued, then stop the thread.
  ~SessionWriter();

  SessionWriter(const SessionWriter&) = delete;
  SessionWriter& operator=(const SessionWriter&) = delete;
  SessionWriter(SessionWriter&&) = delete;
  SessionWriter& operator=(SessionWriter&&) = delete;

  /// @brief Queue a capture, unless the writer holds as many as it may.
  ///
  /// The capture is written in the order of the requests: at the first, the
  /// session's directory and `factory_calibration.json` first; then each
  /// camera's JPEG and depth, `captures/<NNN>.json`, and `session.json`
  /// listing it. Its @ref WriteResult has its index, or why it failed:
  /// `InvalidArgument` when a file it would write is already there (the
  /// session's directory included), `NotFound` when the directory's parent
  /// or the sync file is missing, `IoError` for a write that fails (a full
  /// disk), and `OutOfMemory`.
  /// @param job  The capture. Refused unless it has one part per camera,
  ///             each with a JPEG (starting with its SOI marker), depth with
  ///             a sample per depth pixel, a detection in an image of the
  ///             colour model's size, an origin tag, if any, with the
  ///             session's tag's id, and finite pixels, weights and
  ///             measures; the same for the set's measures; and a
  ///             @ref ViewVerdict for each check.
  /// @return `true` once queued; `false`, with nothing queued, when
  ///         `queue_capacity` captures are already queued or being written;
  ///         `InvalidArgument` naming the camera and what @p job gets wrong;
  ///         or `OutOfMemory`.
  core::Result<bool> submit(CaptureJob job);

  /// @brief Queue a drop of the latest capture requested before it that no
  ///        earlier drop took.
  ///
  /// The capture stays on disk, marked `"dropped": true` in `session.json`,
  /// the one file a drop rewrites; its index is not used again. A drop never
  /// reaches past the capture it takes: when that one failed, nothing is
  /// dropped. Its @ref WriteResult has the capture's index; `NotFound` when
  /// no capture is left to take, or the one it takes was not written; or
  /// why `session.json` could not be written, the capture then left for the
  /// next drop.
  /// @return OK once queued; or `OutOfMemory`.
  core::Status drop_last();

  /// @brief Take the outcome of every request carried out since the last
  ///        call.
  /// @return One per request, in the order they were made.
  std::vector<WriteResult> take_results();

  /// @return The captures queued or being written.
  std::size_t pending() const;

  /// @brief Wait until no request is queued or being carried out.
  void flush();

 private:
  struct State;
  explicit SessionWriter(std::unique_ptr<State> state) noexcept;

  std::unique_ptr<State> state_;
};

/// @brief A session directory's name for @p time: `YYYY-MM-DD_HHMMSS`, in
///        local time.
/// @param time  When the session starts.
/// @return The name; `InvalidArgument` for a time the C library cannot
///         convert to a calendar date; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const std::string name,
///            session_stamp(std::chrono::system_clock::now()));
/// @endcode
VC_CAPTURE_API core::Result<std::string> session_stamp(
    std::chrono::system_clock::time_point time);

}  // namespace volumetric_kit::calib::capture
