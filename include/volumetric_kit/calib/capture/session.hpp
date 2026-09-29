// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file session.hpp
/// @brief A capture session on disk: the rig's cameras, and one luma image per
///        camera per capture.
///
/// A session is a directory:
///
/// @code{.txt}
/// <dir>/session.json
/// <dir>/<serial>/calib_img_<serial>_<n>.png   one per camera per capture
/// @endcode
///
/// Each PNG is 8-bit grey: the camera's luma as captured, before
/// undistortion, at the size of its lens. The image names are the ones the
/// legacy `camera_calibration` scripts read, so their bundle adjustment runs
/// on a session as it is. `session.json` holds each camera's serial, role and
/// factory lens, and each capture's index, trigger time and how many frames
/// were averaged into it:
///
/// @code{.json}
/// {"format": "volumetric_kit_calib.session", "version": 1,
///  "cameras": [{"serial": "CL2A141000N", "role": "primary",
///               "lens": {"width": 3840, "height": 2160,
///                        "fx": 2240.0, "fy": 2239.4, "cx": 1913.2,
///                        "cy": 1039.2, "k1": 0.073, "k2": -0.10,
///                        "p1": -0.0005, "p2": -0.00005, "k3": 0.041}}],
///  "captures": [{"index": 0, "trigger_ns": 1234, "frames_averaged": 5}]}
/// @endcode

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/calib/capture/export.hpp"
#include "volumetric_kit/calib/capture/rig_source.hpp"
#include "volumetric_kit/calib/core/result.hpp"

namespace volumetric_kit::calib::capture {

/// @brief One capture of a session: an image from every camera.
struct Capture {
  std::size_t index = 0;              ///< The `<n>` in its image names.
  std::uint64_t trigger_ns = 0;       ///< Its first set's trigger time (ns).
  std::uint32_t frames_averaged = 1;  ///< Sets averaged into each image.
};

/// @brief What `session.json` says: the cameras, each with its lens, and the
///        captures in the order they were taken.
///
/// @code
/// VC_ASSIGN(const Session session, read_session(dir));
/// for (const Capture& capture : session.captures) {
///   VC_ASSIGN(const LumaImage image,
///             read_session_image(dir, session.cameras[0].serial,
///                                capture.index));
/// }
/// @endcode
struct Session {
  std::vector<CameraInfo> cameras;  ///< Every one with its lens.
  std::vector<Capture> captures;    ///< In capture order, indices ascending.
};

/// @return `true` if @p serial can name a camera's directory and images: one
///         or more of letters, digits, `-`, `_` and `.`, and not `.` or `..`.
///         Camera serials are; this keeps a session's paths inside it.
VC_CAPTURE_API bool is_session_serial(const std::string& serial) noexcept;

/// @return Where a session in @p dir keeps camera @p serial's image of
///         capture @p index: `<dir>/<serial>/calib_img_<serial>_<index>.png`.
VC_CAPTURE_API std::string session_image_path(const std::string& dir,
                                              const std::string& serial,
                                              std::size_t index);

/// @brief Read the `session.json` of the session in @p dir.
/// @return The session; @ref Status::Code::NotFound if there is no
///         `session.json`; @ref Status::Code::IoError if it cannot be read;
///         or @ref Status::Code::InvalidArgument naming what is wrong: not
///         JSON, another format or version, no camera, a serial
///         @ref is_session_serial refuses (or a repeated one), an unknown
///         role, a lens
///         `PinholeBrownConrady::create` refuses, or a capture whose index is
///         not above the one before it or that averaged no frames.
VC_CAPTURE_API Result<Session> read_session(const std::string& dir);

/// @brief Read one image of the session in @p dir.
/// @return The image, converted to one channel if it was stored with more;
///         @ref Status::Code::NotFound if the file does not exist; or
///         @ref Status::Code::IoError if it does not decode as a PNG.
VC_CAPTURE_API Result<LumaImage> read_session_image(const std::string& dir,
                                                    const std::string& serial,
                                                    std::size_t index);

/// @brief Writes a session: creates its directory, adds captures, and takes
///        the last one back.
///
/// `session.json` is rewritten after every change, through a temporary file
/// renamed into place, so a session interrupted between captures reads back
/// with every capture completed before the interruption.
///
/// @code
/// VC_ASSIGN(SessionWriter writer,
///           SessionWriter::create("sessions/2026-09-29", source.cameras()));
/// VC_ASSIGN(const std::size_t index,
///           writer.add(set.trigger_ns, /*frames_averaged=*/5, images));
/// VC_TRY(writer.remove_last());  // the clicker's "back"
/// @endcode
class VC_CAPTURE_API SessionWriter {
 public:
  /// @brief Start a session of @p cameras in @p dir, creating the directory
  ///        and writing a `session.json` with no captures.
  /// @param dir      The session's directory: new, or one without a
  ///                 `session.json`.
  /// @param cameras  The rig's cameras, every one with its lens.
  /// @return The writer; @ref Status::Code::InvalidArgument for no cameras,
  ///         one without a lens, a serial that is not a plain file name
  ///         (@ref is_session_serial), a repeated serial, or a @p dir that
  ///         already holds a session; or
  ///         @ref Status::Code::IoError if a directory or `session.json`
  ///         cannot be written.
  static Result<SessionWriter> create(std::string dir,
                                      std::vector<CameraInfo> cameras);

  /// @brief Write one capture: every camera's image, then `session.json`.
  /// @param trigger_ns       The capture's first set's trigger time.
  /// @param frames_averaged  How many sets were averaged into the images;
  ///                         at least one.
  /// @param images           One image per camera, in @ref Session::cameras
  ///                         order, each the size of that camera's lens.
  /// @return The capture's index, one above the last capture's (zero for the
  ///         first); @ref Status::Code::InvalidArgument for a wrong number of
  ///         images, an image of the wrong size or pixel count, or no frames
  ///         averaged -- nothing is written then; or
  ///         @ref Status::Code::IoError if a file cannot be written, in which
  ///         case the images already written are removed and the session is
  ///         as it was.
  Result<std::size_t> add(std::uint64_t trigger_ns,
                          std::uint32_t frames_averaged,
                          const std::vector<LumaImage>& images);

  /// @brief Take back the last capture: delete its images and its entry. The
  ///        next @ref add reuses its index, so indices stay contiguous.
  /// @return OK; @ref Status::Code::NotFound when there is no capture; or
  ///         @ref Status::Code::IoError if `session.json` cannot be written.
  Status remove_last();

  /// @return The session as written so far.
  const Session& session() const noexcept { return session_; }
  /// @return The session's directory.
  const std::string& dir() const noexcept { return dir_; }

 private:
  SessionWriter(std::string dir, Session session)
      : dir_(std::move(dir)), session_(std::move(session)) {}

  std::string dir_;
  Session session_;
};

}  // namespace volumetric_kit::calib::capture
