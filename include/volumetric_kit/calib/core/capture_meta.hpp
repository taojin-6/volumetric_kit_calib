// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file capture_meta.hpp
/// @brief One capture's set-level facts: the record a capture stores beside
///        its images and each camera's @ref view_check.hpp "ViewCheck".
///
/// A plain record in the `core` tier, so the tier that takes a capture and the
/// one that stores it need not depend on each other.

#include <cstdint>
#include <optional>

namespace volumetric_kit::calib {

/// @brief The set-level facts of one capture.
///
/// How many of the capture's views are usable is not stored here: it is
/// counted from its cameras' @ref ViewCheck verdicts when the capture is
/// written.
///
/// @code
/// CaptureMeta meta;
/// meta.index = 12;
/// meta.time_ns = now_since_epoch_ns;  // std::chrono::system_clock
/// meta.trigger_ns = set.timestamp_ns;
/// meta.sequence = set.sequence;
/// meta.spread_ms = 1.4;
/// meta.stillness_px = 0.08;
/// meta.live_vs_full_px = 0.12;
/// @endcode
struct CaptureMeta {
  int index = 0;  ///< The capture's number in its session, as its files are.
  /// When the capture was taken: the host's wall clock
  /// (`std::chrono::system_clock`), in nanoseconds since its epoch.
  std::int64_t time_ns = 0;
  /// The captured set's trigger time (ns), as recon's `FrameSet::timestamp_ns`.
  std::uint64_t trigger_ns = 0;
  /// The captured set's sequence number, as recon's `FrameSet::sequence`.
  std::uint64_t sequence = 0;
  /// The set's latest frame timestamp less its earliest (milliseconds).
  double spread_ms = 0.0;
  /// The median corner motion over the sets that found the board still
  /// (pixels).
  double stillness_px = 0.0;
  /// The median distance between the live detection's corners and the full
  /// detection's (pixels); empty when no camera has both.
  std::optional<double> live_vs_full_px;
};

}  // namespace volumetric_kit::calib
