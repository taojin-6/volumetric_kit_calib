// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file session_writer.hpp
/// @brief A calibration session on disk: the rig, and each capture's
///        pictures, depth, detections and checks, written on a thread of its
///        own and never overwritten.
///
/// A session is a directory, created at its first capture:
///
/// @code{.txt}
/// session.json                 the rig, the board, the captures
/// factory_calibration.json     sensor's calibration file, unposed
/// images/<id>/calib_img_<id>_<NNN>.jpg
///                              the camera's JPEG, byte for byte; or .pgm,
///                              its 8-bit luma, for a camera with no JPEG
/// depth/<id>/calib_depth_<id>_<NNN>.png
///                              16-bit grey, the sensor's units
/// captures/<NNN>.json          times, detections and checks
/// @endcode
///
/// - `<NNN>` is the capture's index, zero-padded to three digits. The image
///   names are the ones the legacy `camera_calibration` scripts glob for.
/// - A PGM is the luma as read: `P5`, the size, `255`, then the rows.
/// - `factory_calibration.json` is sensor's version-2 calibration file
///   (`sensor/camera/array_calibration.hpp`): the factory colour and depth
///   models at the streamed mode, and `depth_to_color`, with no pose, of
///   each camera that has one; with none, as a stereo pair's, it is not
///   written, and `session.json` names none.
/// - Depth, only from a camera with depth, is as the sensor captured it:
///   raw, not undistorted.
/// - `streams` records the cameras' size and rate, which every camera
///   shares, their coding when known, and the @ref DepthMode when a camera
///   has depth.
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
/// `session.json`, version 2, of a Femto rig: keys a session lacks (a sync
/// file, an Orbbec camera's facts, depth, the SDK's version, a factory
/// model) are left out, and a DeckLink camera has its `input`:
///
/// @code{.json}
/// {"format": "volumetric_kit_calib.session", "version": 2,
///  "created": "2026-10-13T09:41:07.250Z", "mode": "extrinsics",
///  "software": {"calib": "...", "sensor": "...", "gfx": "...", "core": "...",
///               "opencv": "5.0.0", "orbbec_sdk": "2.10.6"},
///  "rig": {"driver": "orbbec", "sync_file": "config/femto_mega_sync.json",
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
///  "cameras": [{"id": "CL2A141000N", "model": "Orbbec Femto Mega",
///               "firmware": "...", "ip": "192.168.1.10", "role": "primary",
///               "sync_mode": "primary", "depth_delay_us": 0,
///               "color_lens_model": "brown-conrady",
///               "depth_lens_model": "brown-conrady-k6",
///               "lens": {"origin": "factory"}}],
///  "factory_calibration": "factory_calibration.json",
///  "captures": [{"index": 0, "file": "captures/000.json",
///                "dropped": false}]}
/// @endcode
///
/// A camera's `lens` is where the lens its views were checked through came
/// from: `factory`, `stand-in`, or `file` with the calibration file's `path`
/// and `sha256`. A checkerboard's `board` has no marker keys
/// (`"type": "checkerboard"`, its squares, square and tolerances); an
/// intrinsic session's `mode` is `"intrinsics"`.
///
/// `captures/<NNN>.json`, with `null` for a measure that is empty, for an
/// origin tag the camera did not find and for the depth of a camera without
/// it; `usable_views` counts the cameras whose verdict @ref is_usable
/// accepts:
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
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"

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
  std::string calib;   ///< calib's version or commit.
  std::string sensor;  ///< volumetric_kit_sensor's.
  std::string gfx;     ///< volumetric_kit_gfx's.
  std::string core;    ///< volumetric_kit_core's.
  std::string opencv;  ///< OpenCV's version.
  /// The Orbbec SDK's version; empty, and not recorded, in a build without
  /// it.
  std::string orbbec_sdk;
};

/// @brief The rig's depth mode: what each camera's depth model does not say.
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

/// @brief An Orbbec camera's own facts, as sensor's `OrbbecDeviceInfo` and
///        the rig's sync file give them.
///
/// @code
/// OrbbecFacts facts;
/// facts.firmware = device.firmware_version;
/// facts.depth_delay_us = sync.depth_delay_us;
/// @endcode
struct OrbbecFacts {
  std::string firmware;             ///< Its firmware version.
  std::string ip_address;           ///< Its IP address; empty for a USB camera.
  std::string sync_mode;            ///< Its sync mode as its driver names it.
  std::int32_t depth_delay_us = 0;  ///< Its depth's delay after the trigger.
  std::string color_lens_model;     ///< The vendor's colour lens model.
  std::string depth_lens_model;     ///< The vendor's depth lens model.
};

/// @brief Where the lens a camera's views are checked through came from.
enum class LensOrigin : std::uint8_t {
  Factory,  ///< The device's own (@ref SessionCamera::color).
  File,     ///< A calibration file (@ref SessionCamera::lens_file).
  StandIn,  ///< A stand-in, no camera's.
};

/// @return @p origin's name in `session.json`: `"factory"`, `"file"` or
///         `"stand-in"`; `"unknown"` for a value that is none of them.
VC_CAPTURE_API const char* to_string(LensOrigin origin) noexcept;

/// @brief One camera of the rig, as its session records it: calib's own
///        record, which names no driver's type.
///
/// @code
/// SessionCamera camera;
/// camera.id = "bm-1";
/// camera.model = "DeckLink 8K Pro";
/// camera.size = {3840, 2160};
/// camera.fps = 30;
/// camera.input = "SDI 2";
/// camera.lens = LensOrigin::File;
/// camera.lens_file = "bm_pair/array_calibration.json";
/// @endcode
struct SessionCamera {
  /// Its calibration key, which names its files: a Femto's serial, `"bm-1"`.
  std::string id;
  std::string model;               ///< What it is, e.g. "Orbbec Femto Mega".
  sensor::camera::ImageSize size;  ///< Its pictures' size.
  std::uint32_t fps = 0;           ///< Its rate, frames a second.
  /// Its role in a synced rig as sensor names it (`"primary"`); empty, and
  /// not recorded, for none.
  std::string role;
  /// The device's own colour lens at @ref size; empty for a camera without
  /// one (a stereo pair's).
  std::optional<sensor::camera::CameraModel> color;
  /// The device's own depth lens: a camera with depth, whose every capture
  /// carries it.
  std::optional<sensor::camera::CameraModel> depth;
  /// Its depth camera's frame to its colour camera's: the factory extrinsic.
  sensor::camera::Mat4d depth_to_color = sensor::camera::Mat4d(1.0);
  std::optional<OrbbecFacts> orbbec;  ///< An Orbbec camera's own facts.
  /// The capture card's input it comes in on (`"SDI 2"`); empty, and not
  /// recorded, for none.
  std::string input;
  /// Where the lens its views are checked through came from.
  LensOrigin lens = LensOrigin::Factory;
  /// With @ref LensOrigin::File, the calibration file's path, whose SHA-256
  /// the session records, reading it at the first capture; empty otherwise.
  std::string lens_file;
};

/// @brief What a session calibrates.
enum class SessionMode : std::uint8_t {
  Extrinsics,  ///< The rig's poses: `"extrinsics"`.
  Intrinsics,  ///< Each camera's lens: `"intrinsics"`.
};

/// @brief A session: where it goes, and what it records once.
///
/// @code
/// SessionInfo info;
/// info.directory = out + "/" + session_stamp(now).value();
/// info.driver = "orbbec";
/// info.sync_path = "config/femto_mega_sync.json";
/// info.reference = cameras[primary].id;
/// info.board = legacy_femto_board();
/// info.depth_mode = depth_mode;
/// info.color_codec = "mjpeg";
/// info.cameras = cameras;
/// @endcode
struct SessionInfo {
  /// The session's directory, created at the first capture: nothing may be
  /// there then, and its parent must exist.
  std::string directory;
  Software software;  ///< What the session was captured with.
  SessionMode mode = SessionMode::Extrinsics;  ///< What it calibrates.
  /// What opened the cameras, as sensor names the driver (`"orbbec"`).
  std::string driver;
  /// The rig's sync configuration, whose path and SHA-256 `session.json`
  /// records, reading it at the first capture; empty for a rig without one.
  std::string sync_path;
  /// The reference camera's id: the rig's primary, whose colour camera is
  /// the solve's world.
  std::string reference;
  /// The board: a ChArUco board or a checkerboard @ref check_target accepts.
  Target board;
  /// Whether `board.square_size_m` was measured, rather than the print's
  /// nominal size.
  bool square_measured = false;
  /// The tag each camera's picture was searched for
  /// (@ref CameraCapture::origin_tag): one @ref check_origin_tag accepts.
  OriginTag origin_tag = kOriginTag;
  /// The depth mode of the cameras with depth; empty when none has it.
  std::optional<DepthMode> depth_mode;
  /// How the cameras code their colour (`"mjpeg"`); empty, and not recorded,
  /// when unknown.
  std::string color_codec;
  /// The cameras, in the order every capture gives them. Every camera
  /// streams the same size at the same rate, and its depth, if any, at the
  /// same size as the others'.
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
/// part.luma = std::move(host_luma);  // written as a PGM: no JPEG
/// part.timestamp_ns = picture.timestamp_ns;
/// part.detection = std::move(full_detection);
/// part.check = check;
/// part.origin_tag = tag_search.found;  // detect::detect_tag's
/// @endcode
struct CameraCapture {
  /// The camera's JPEG (`RgbdFrame::color_bitstream`), copied out of the
  /// frame so the driver's buffers go back to it; empty for a camera that
  /// sent none, whose @ref luma is written.
  std::vector<std::uint8_t> jpeg;
  /// The picture's 8-bit luma, its rows packed: written as a PGM when there
  /// is no @ref jpeg; ignored otherwise.
  std::vector<std::uint8_t> luma;
  /// The depth, a sample for each of the depth model's pixels, row-major, in
  /// the sensor's units, as `RgbdFrame::depth`: from a camera with depth, and
  /// only then. Shared, so a caller may keep the samples it hands over
  /// without a copy: the writer only reads them, and lets them go once the
  /// capture is carried out.
  std::shared_ptr<const std::vector<std::uint16_t>> depth;
  std::uint64_t timestamp_ns = 0;  ///< The picture's capture time.
  std::uint64_t sequence = 0;      ///< Its sequence number.
  /// The board found in the picture at full resolution: what the solver
  /// takes. Its `image_size` is the camera's. The observation's own camera
  /// and frame are not written: the capture's file holds the detection under
  /// the camera's id.
  Detection detection;
  ViewCheck check;  ///< What the view is worth.
  /// The origin tag (@ref SessionInfo::origin_tag) found in the picture at
  /// full resolution, in its pixels; empty when the camera found none, or
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
/// job.meta.spread_ms = set.spread_ns / 1e6;
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
/// It writes; it detects nothing (@ref CameraCapture). Requests are carried
/// out in the order they are made. Every member may be called from any
/// thread. Destroying the writer first carries out every request queued.
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
  /// @param info     The session. Refused unless: the directory, the driver
  ///                 and the reference are non-empty; the board is a ChArUco
  ///                 board or a checkerboard @ref check_target accepts, and
  ///                 the origin tag one
  ///                 @ref check_origin_tag accepts; there is a depth mode,
  ///                 with a finite, positive `metres_per_unit` and finite
  ///                 `0 < min_m < max_m`, exactly when a camera has depth;
  ///                 each id is a plain file name (letters, digits, `-`,
  ///                 `_`, `.`; not `.` or `..`), and no two differ only in
  ///                 case; every camera streams the same non-zero size at
  ///                 the same, known `fps`, and every depth model is one
  ///                 size; each factory colour model is the camera's size;
  ///                 the factory models sensor's calibration file holds,
  ///                 with their `depth_to_color`; a lens file is named
  ///                 exactly when a camera's lens came from one, and a
  ///                 factory lens only by a camera with a factory colour
  ///                 model; and the reference is a camera.
  /// @param options  How it queues.
  /// @return The writer; `InvalidArgument` naming what @p info or @p options
  ///         gets wrong; `Unsupported` for a board of another kind;
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
  /// camera's JPEG or PGM and depth, `captures/<NNN>.json`, and
  /// `session.json` listing it. Its @ref WriteResult has its index, or why
  /// it failed: `InvalidArgument` when a file it would write is already there
  /// (the session's directory included), `NotFound` when the directory's
  /// parent, the sync file or a lens file is missing, `IoError` for a write
  /// that fails (a full disk), and `OutOfMemory`.
  /// @param job  The capture. Refused unless it has one part per camera,
  ///             each with a JPEG (starting with its SOI marker) or the
  ///             camera's luma, depth with a sample per depth pixel from a
  ///             camera with depth and none from another, a detection in an
  ///             image of the camera's size, an origin tag, if any, with the
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
