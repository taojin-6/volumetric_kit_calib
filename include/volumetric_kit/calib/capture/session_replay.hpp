// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file session_replay.hpp
/// @brief A recorded session played back as a rig: each capture once, as a
///        complete set, in the order it was taken.

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/calib/capture/export.hpp"
#include "volumetric_kit/calib/capture/rig_source.hpp"
#include "volumetric_kit/calib/capture/session.hpp"
#include "volumetric_kit/calib/core/result.hpp"

namespace volumetric_kit::calib::capture {

/// @brief Plays a session back through @ref IRigSource, so everything
///        downstream of capture runs, and is tested, without cameras.
///
/// Every @ref poll_set after @ref start hands out the next capture as a
/// complete set whose trigger time is the capture's; there is no pacing, so a
/// consumer gets the captures as fast as it polls. @ref read_luma reads the
/// capture's image from disk. After the last capture the source is
/// @ref exhausted; @ref start again replays from the first.
///
/// @code
/// VC_ASSIGN(SessionReplay replay, SessionReplay::open("sessions/2026-09-29"));
/// VC_TRY(replay.start());
/// while (!replay.exhausted()) {
///   VC_ASSIGN(const std::optional<SetInfo> set, replay.poll_set());
///   if (!set) continue;
///   VC_ASSIGN(const LumaImage image, replay.read_luma(0));
/// }
/// @endcode
class VC_CAPTURE_API SessionReplay final : public IRigSource {
 public:
  /// @brief Read the session in @p dir. Opens no image yet.
  /// @return The replay, not started; or what @ref read_session returns.
  static Result<SessionReplay> open(std::string dir);

  SessionReplay(SessionReplay&&) noexcept = default;
  SessionReplay& operator=(SessionReplay&&) noexcept = default;
  ~SessionReplay() override = default;

  /// @return The session's cameras, every one with its lens.
  const std::vector<CameraInfo>& cameras() const noexcept override {
    return session_.cameras;
  }

  /// @brief Rewind to the first capture and start handing captures out.
  /// @return OK.
  Status start() override;

  /// @brief Stop handing captures out. The next @ref start replays from the
  ///        first.
  void stop() noexcept override;

  /// @return The next capture as a complete set; empty when not started or
  ///         when every capture has been handed out.
  Result<std::optional<SetInfo>> poll_set() override;

  /// @return Camera @p camera's image of the capture last handed out, as
  ///         @ref read_session_image reads it, checked against the camera's
  ///         lens size (@ref Status::Code::InvalidArgument if it differs); or
  ///         as @ref IRigSource::read_luma says.
  Result<LumaImage> read_luma(std::size_t camera) override;

  /// @return `true` once every capture has been handed out.
  bool exhausted() const noexcept override;

  /// @return The session being replayed.
  const Session& session() const noexcept { return session_; }

 private:
  SessionReplay(std::string dir, Session session)
      : dir_(std::move(dir)), session_(std::move(session)) {}

  std::string dir_;
  Session session_;
  bool started_ = false;
  std::size_t next_ = 0;                // the next capture to hand out
  std::optional<std::size_t> current_;  // the capture last handed out
};

}  // namespace volumetric_kit::calib::capture
