// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file capture_rule.hpp
/// @brief Which set a capture takes: a press arms the rule, which then takes
///        the first set that is complete and still, or fails after a timeout
///        with the reason it waited.

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/core/view_check.hpp"
#include "volumetric_kit/calib/quality/export.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::quality {

/// @brief The rule's thresholds.
///
/// @code
/// CaptureRuleOptions options;
/// options.timeout = std::chrono::seconds(5);
/// @endcode
struct CaptureRuleOptions {
  /// A set is still when its corners moved less than this since the set
  /// before (median, pixels).
  double still_px = 0.3;
  /// An armed capture that has taken no set by then fails.
  std::chrono::milliseconds timeout{3000};
  /// A set whose frames are spread over more than this (milliseconds) is
  /// warned of; never refused.
  double spread_warn_ms = 5.0;
  /// A set with fewer usable views than this is warned of: it adds nothing
  /// to the extrinsics; never refused.
  int min_views = 2;
};

/// @brief What the live view knows of one set.
///
/// @code
/// LiveSet set;
/// set.present = present;  // a frame from each camera?
/// set.checks = checks;    // check_view(...).check, or {} for no frame
/// set.motion_px = corner_shift(previous, current).value().median_px;
/// set.spread_ms = frame_set.spread_ns() / 1e6;
/// set.saver_full = writer->pending() >= capacity;
/// @endcode
struct LiveSet {
  /// For each camera, in the rule's order: whether the set has its frame.
  std::vector<bool> present;
  /// For each camera: its view's live check; a default one without a frame.
  std::vector<ViewCheck> checks;
  /// How far the board's corners moved since the set before
  /// (@ref CornerShift::median_px); empty when no corner is in both.
  std::optional<double> motion_px;
  double spread_ms = 0.0;  ///< The latest frame's time less the earliest's.
  // TODO: the example's `pending() >= capacity` becomes the writer's own
  // full() once the capture tier gives SessionWriter one.
  bool saver_full = false;  ///< The writer holds as many captures as it may.
};

/// @brief Where the rule stands after a set or a tick.
enum class CaptureState {
  Idle,     ///< Not armed.
  Waiting,  ///< Armed; this set was not taken (@ref CaptureDecision::hold).
  Take,     ///< Take this set; the rule is disarmed.
  Failed,   ///< The capture failed; the rule is disarmed.
};

/// @brief What kept a set from being taken.
enum class Hold {
  None,        ///< Nothing: the set is taken, or the rule is idle.
  NoSet,       ///< No set has come since the press.
  Incomplete,  ///< A camera's frame is missing.
  NoBoard,     ///< No corner of the board is in both this set and the last.
  Moving,      ///< The board moved `still_px` or more since the last set.
  /// The writer is full when the set to take comes: the capture is refused
  /// at once.
  SaverFull,
};

/// @brief The rule's answer to a set or a tick.
///
/// @code
/// VKC_ASSIGN(const CaptureDecision decision, rule.update(set, now));
/// if (decision.state == CaptureState::Failed) {
///   show("capture failed: " + decision.message);
/// } else if (decision.state == CaptureState::Take) {
///   meta.stillness_px = decision.stillness_px;
///   submit(set);  // a caller's own
/// }
/// @endcode
struct CaptureDecision {
  CaptureState state = CaptureState::Idle;  ///< Where the rule stands.
  /// `Waiting`: what held this set. `Failed`: what held the last set before
  /// the timeout (`NoSet` without one), or `SaverFull`.
  Hold hold = Hold::None;
  /// `Waiting` and `Failed`: the hold, named, e.g. `"board moving (0.82
  /// px)"` or `"CL2A14100A4 missing"`. `Take`: its warnings, `"; "` apart,
  /// or empty.
  std::string message;
  double stillness_px = 0.0;  ///< `Take`: the set's motion, for its record.
  int usable_views = 0;       ///< `Take`: the set's usable live views.
  bool spread_high = false;   ///< `Take`: spread above `spread_warn_ms`.
  bool few_views = false;     ///< `Take`: fewer usable views than `min_views`.
};

/// @brief The capture rule, as a state machine the caller drives: @ref arm
///        on a press, @ref update with each set polled, and @ref tick when no
///        set came.
///
/// Armed, it takes the first set, at or after the press and before the
/// timeout, that has every camera's frame and moved less than `still_px`
/// since the set before. The whole set is taken however many cameras see the
/// board; fewer than `min_views` usable views, or frames spread over
/// `spread_warn_ms`, are warned of, never refused. The capture fails when the
/// timeout passes, naming what held the last set, or at once when the set it
/// would take finds the writer full. A value; not thread-safe.
///
/// @code
/// VKC_ASSIGN(CaptureRule rule, CaptureRule::create(serials));
/// if (pressed) rule.arm(std::chrono::steady_clock::now());
/// VKC_ASSIGN(const CaptureDecision decision,
///            rule.update(set, std::chrono::steady_clock::now()));
/// @endcode
class VC_QUALITY_API CaptureRule {
 public:
  /// @brief The clock the rule's times are on.
  using Clock = std::chrono::steady_clock;

  /// @brief A rule for a rig of @p camera_ids.
  /// @param camera_ids  Each camera's id, as messages name it, in the order
  ///                    of every @ref LiveSet; at least one, unique, none
  ///                    empty.
  /// @param options     The thresholds.
  /// @return The rule, idle; `InvalidArgument` naming what @p camera_ids
  ///         (@ref solve::check_camera_ids) or @p options gets wrong; or
  ///         `OutOfMemory`.
  static core::Result<CaptureRule> create(
      const std::vector<std::string>& camera_ids,
      const CaptureRuleOptions& options = {});

  /// @brief Arm the rule: a press.
  /// @param now  When it was pressed.
  /// @return `true`; `false`, leaving the rule as it was, when it is armed
  ///         already.
  bool arm(Clock::time_point now) noexcept;

  /// @return Whether a capture is armed.
  bool armed() const noexcept { return armed_; }

  /// @brief The rule's answer to one set.
  /// @param set  The set.
  /// @param now  When it was polled; not before the press.
  /// @return The decision (`Idle` when not armed); `InvalidArgument` for a
  ///         set whose lists do not have one entry per camera, or whose
  ///         motion or spread is negative or not finite; or `OutOfMemory`.
  core::Result<CaptureDecision> update(const LiveSet& set,
                                       Clock::time_point now);

  /// @brief The rule's answer when no set came: an armed capture fails once
  ///        the timeout passes.
  /// @param now  The time.
  /// @return `Idle`, `Waiting` (with the last set's hold) or `Failed`; or
  ///         `OutOfMemory`.
  core::Result<CaptureDecision> tick(Clock::time_point now);

 private:
  CaptureRule(std::vector<std::string> camera_ids,
              const CaptureRuleOptions& options);

  // A decision in `state` naming the last set's hold.
  CaptureDecision held(CaptureState state) const;

  std::vector<std::string> camera_ids_;
  CaptureRuleOptions options_;
  bool armed_ = false;
  Clock::time_point deadline_{};
  Hold last_hold_ = Hold::NoSet;
  std::string last_message_;
};

}  // namespace volumetric_kit::calib::quality
