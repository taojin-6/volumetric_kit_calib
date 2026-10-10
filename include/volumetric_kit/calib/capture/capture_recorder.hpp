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
#include "volumetric_kit/calib/capture/luma_image.hpp"
#include "volumetric_kit/calib/capture/session.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::capture {

/// @brief Options for @ref CaptureRecorder::create.
struct RecorderOptions {
  /// Complete sets averaged, pixel by pixel, into each camera's image. The
  /// board is placed and left still, so averaging costs no sharpness and
  /// divides the sensor and compression noise by about its square root.
  std::uint32_t frames_to_average = 5;
};

/// @brief Collects captures into a session: @ref arm on the capture key, then
///        @ref on_set with every set the rig hands out.
///
/// Once armed, the recorder adds every camera's luma of each complete set to a
/// per-camera sum. A set missing a camera is skipped, not counted, since a
/// capture is only useful to the solve with every camera in it; @ref skipped
/// says how many were. After @ref RecorderOptions::frames_to_average complete
/// sets it writes their average as one capture, and disarms.
///
/// The caller reads a set's luma back only while @ref armed, and only for a
/// complete set: a live rig's frames (recon's `SensorArray` sets) then stay on
/// the GPU between captures.
///
/// @code
/// VKC_ASSIGN(CaptureRecorder recorder,
///            CaptureRecorder::create(std::move(writer)));
/// if (key == Key::Capture) recorder.arm();
/// if (key == Key::Back) VKC_TRY(recorder.undo());
/// if (set && recorder.armed()) {
///   // One per camera, in the session's order; null where the set has none.
///   std::vector<LumaImage> luma(cameras);
///   std::vector<const LumaImage*> images(cameras, nullptr);
///   if (set->complete()) {
///     for (std::size_t i = 0; i < cameras; ++i) {
///       VKC_ASSIGN(luma[i], read_back_luma(*set, i));  // the app's readback
///       images[i] = &luma[i];
///     }
///   }
///   VKC_ASSIGN(const std::optional<std::size_t> written,
///              recorder.on_set(set->timestamp_ns, images));
///   if (written) announce_capture(*written);
/// }
/// @endcode
class VC_CAPTURE_API CaptureRecorder {
 public:
  /// @brief Record into @p writer's session.
  /// @return The recorder, not armed; or
  ///         @ref core::Status::Code::InvalidArgument for no frames to average.
  static core::Result<CaptureRecorder> create(SessionWriter writer,
                                              RecorderOptions options = {});

  /// @brief Start collecting a capture from the next complete set. Arming an
  ///        armed recorder changes nothing.
  void arm() noexcept;

  /// @brief Drop the capture being collected, if any, and disarm.
  void cancel() noexcept;

  /// @brief Take in one set the rig just handed out.
  /// @param trigger_ns  The set's trigger time (ns).
  /// @param images      Per camera of the session, in its order, the
  ///                    camera's luma at the size of its lens; null where the
  ///                    set has no frame from that camera. Read only when
  ///                    armed and none is null.
  /// @return The index of the capture this set completed and wrote; empty
  ///         otherwise; @ref core::Status::Code::InvalidArgument if @p images
  ///         does not have one entry per camera of the session, or, when
  ///         read, one is not the size of its camera's lens; or a write
  ///         failure. After an image of the wrong size or a write failure,
  ///         the capture being collected is dropped and the recorder
  ///         disarmed.
  core::Result<std::optional<std::size_t>> on_set(
      std::uint64_t trigger_ns, const std::vector<const LumaImage*>& images);

  /// @brief Take back the last capture: drop one being collected if armed,
  ///        otherwise remove the last one written.
  /// @return OK; or what @ref SessionWriter::remove_last returns.
  core::Status undo();

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
