// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file capture_recorder.hpp
/// @brief Turns one press of the capture key into one capture: the next few
///        complete sets, averaged, written to a session.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "volumetric_kit/calib/capture/export.hpp"
#include "volumetric_kit/calib/capture/rig_source.hpp"
#include "volumetric_kit/calib/capture/session.hpp"
#include "volumetric_kit/calib/core/result.hpp"

namespace volumetric_kit::calib::capture {

/// @brief Options for @ref CaptureRecorder::create.
struct RecorderOptions {
  /// Complete sets averaged, pixel by pixel, into each camera's image. The
  /// board is placed and left still, so averaging costs no sharpness and
  /// divides the sensor and compression noise by about its square root.
  std::uint32_t frames_to_average = 5;
};

/// @brief Collects captures into a session: @ref arm on the capture key, then
///        @ref on_set with every set the source hands out.
///
/// Once armed, the recorder reads every camera's luma from each complete set
/// and adds it to a per-camera sum. A set missing a camera is skipped, not
/// counted, since a capture is only useful to the solve with every camera in
/// it; @ref skipped says how many were. After
/// @ref RecorderOptions::frames_to_average complete sets it writes their
/// average as one capture, and disarms. Sets that arrive while it is not armed
/// cost nothing: no pixel is read.
///
/// @code
/// VC_ASSIGN(CaptureRecorder recorder,
/// CaptureRecorder::create(std::move(writer))); if (key == Key::Capture)
/// recorder.arm(); if (key == Key::Back) VC_TRY(recorder.undo());
/// VC_ASSIGN(const std::optional<SetInfo> set, source.poll_set());
/// if (set) {
///   VC_ASSIGN(const std::optional<std::size_t> written,
///             recorder.on_set(*set, source));
///   if (written) announce_capture(*written);
/// }
/// @endcode
class VC_CAPTURE_API CaptureRecorder {
 public:
  /// @brief Record into @p writer's session.
  /// @return The recorder, not armed; or
  ///         @ref Status::Code::InvalidArgument for no frames to average.
  static Result<CaptureRecorder> create(SessionWriter writer,
                                        RecorderOptions options = {});

  /// @brief Start collecting a capture from the next complete set. Arming an
  ///        armed recorder changes nothing.
  void arm() noexcept;

  /// @brief Drop the capture being collected, if any, and disarm.
  void cancel() noexcept;

  /// @brief Take in one set the source just handed out.
  /// @param set     The set @p source's last @ref IRigSource::poll_set
  ///                returned.
  /// @param source  The source it came from, whose cameras are the
  ///                session's, in its order; read only when armed and the
  ///                set is complete.
  /// @return The index of the capture this set completed and wrote; empty
  ///         otherwise; @ref Status::Code::InvalidArgument if @p set does not
  ///         have one entry per camera of the session; or a read or write
  ///         failure, after which the capture being collected is dropped and
  ///         the recorder disarmed.
  Result<std::optional<std::size_t>> on_set(const SetInfo& set,
                                            IRigSource& source);

  /// @brief Take back the last capture: drop one being collected if armed,
  ///        otherwise remove the last one written.
  /// @return OK; or what @ref SessionWriter::remove_last returns.
  Status undo();

  /// @return `true` while a capture is being collected.
  bool armed() const noexcept { return armed_; }
  /// @return Complete sets collected into the current capture.
  std::uint32_t collected() const noexcept { return collected_; }
  /// @return Incomplete sets skipped while armed, since @ref create.
  std::uint64_t skipped() const noexcept { return skipped_; }
  /// @return The session written so far.
  const SessionWriter& writer() const noexcept { return writer_; }
  /// @return The options it was created with.
  const RecorderOptions& options() const noexcept { return options_; }

 private:
  CaptureRecorder(SessionWriter writer, RecorderOptions options)
      : writer_(std::move(writer)), options_(options) {}

  SessionWriter writer_;
  RecorderOptions options_;
  bool armed_ = false;
  std::uint32_t collected_ = 0;
  std::uint64_t skipped_ = 0;
  std::uint64_t first_trigger_ns_ = 0;
  // Per camera, the running sum of every collected image, pixel by pixel.
  std::vector<std::vector<std::uint32_t>> sums_;
};

}  // namespace volumetric_kit::calib::capture
