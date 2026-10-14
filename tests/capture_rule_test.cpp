// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The capture rule: a press arms it; it takes the first complete, still set,
// warning of a wide spread or too few views but never refusing for them; it
// refuses at once when the writer is full, and fails after its timeout,
// naming what held the last set.

#include "volumetric_kit/calib/quality/capture_rule.hpp"

#include <chrono>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace volumetric_kit::calib::quality {
namespace {

using Code = core::Status::Code;
using std::chrono::milliseconds;
using Time = CaptureRule::Clock::time_point;

const Time kPress = Time{} + std::chrono::hours(1);

CaptureRule rig_rule(const CaptureRuleOptions& options = {}) {
  return CaptureRule::create({"SYN0", "SYN1", "SYN2", "SYN3"}, options).value();
}

// A complete set, every camera's view good, moved `motion_px`.
LiveSet still_set(std::optional<double> motion_px = 0.1) {
  LiveSet set;
  set.present.assign(4, true);
  set.checks.assign(4, ViewCheck{});
  for (ViewCheck& check : set.checks) check.verdict = ViewVerdict::Good;
  set.motion_px = motion_px;
  set.spread_ms = 1.2;
  return set;
}

CaptureDecision update(CaptureRule& rule, const LiveSet& set, Time now) {
  core::Result<CaptureDecision> decision = core::Status::invalid_argument("");
  EXPECT_NO_THROW(decision = rule.update(set, now));
  EXPECT_TRUE(decision.ok()) << decision.status().message();
  return decision.ok() ? *decision : CaptureDecision{};
}

CaptureDecision tick(CaptureRule& rule, Time now) {
  core::Result<CaptureDecision> decision = core::Status::invalid_argument("");
  EXPECT_NO_THROW(decision = rule.tick(now));
  EXPECT_TRUE(decision.ok()) << decision.status().message();
  return decision.ok() ? *decision : CaptureDecision{};
}

TEST(CaptureRule, IsIdleUntilAPress) {
  CaptureRule rule = rig_rule();
  EXPECT_FALSE(rule.armed());
  EXPECT_EQ(update(rule, still_set(), kPress).state, CaptureState::Idle);
  EXPECT_EQ(tick(rule, kPress).state, CaptureState::Idle);
  EXPECT_TRUE(rule.arm(kPress));
  EXPECT_TRUE(rule.armed());
  // A second press neither re-arms nor moves the deadline.
  EXPECT_FALSE(rule.arm(kPress + milliseconds(2000)));
  EXPECT_EQ(tick(rule, kPress + milliseconds(3000)).state,
            CaptureState::Failed);
}

// Armed, it waits out sets that lack a frame, a board seen twice running or
// stillness, and takes the first set with all three; then it is idle.
TEST(CaptureRule, TakesTheFirstCompleteStillSet) {
  CaptureRule rule = rig_rule();
  rule.arm(kPress);
  LiveSet incomplete = still_set();
  incomplete.present[1] = false;
  incomplete.present[3] = false;
  CaptureDecision decision =
      update(rule, incomplete, kPress + milliseconds(40));
  EXPECT_EQ(decision.state, CaptureState::Waiting);
  EXPECT_EQ(decision.hold, Hold::Incomplete);
  EXPECT_EQ(decision.message, "SYN1, SYN3 missing");

  decision = update(rule, still_set(std::nullopt), kPress + milliseconds(80));
  EXPECT_EQ(decision.hold, Hold::NoBoard);
  EXPECT_EQ(decision.message, "board not seen in two sets running");

  // Still means under 0.3 px.
  decision = update(rule, still_set(0.3), kPress + milliseconds(120));
  EXPECT_EQ(decision.state, CaptureState::Waiting);
  EXPECT_EQ(decision.hold, Hold::Moving);
  EXPECT_EQ(decision.message, "board moving (0.30 px)");

  decision = update(rule, still_set(0.29), kPress + milliseconds(160));
  EXPECT_EQ(decision.state, CaptureState::Take);
  EXPECT_EQ(decision.hold, Hold::None);
  EXPECT_EQ(decision.stillness_px, 0.29);
  EXPECT_EQ(decision.usable_views, 4);
  EXPECT_FALSE(decision.spread_high || decision.few_views);
  EXPECT_EQ(decision.message, "");
  EXPECT_FALSE(rule.armed());
  EXPECT_EQ(update(rule, still_set(), kPress + milliseconds(200)).state,
            CaptureState::Idle);
}

TEST(CaptureRule, FailsAfterItsTimeoutNamingTheLastHold) {
  CaptureRule rule = rig_rule();
  rule.arm(kPress);
  LiveSet incomplete = still_set();
  incomplete.present[2] = false;
  EXPECT_EQ(update(rule, still_set(0.82), kPress + milliseconds(1000)).hold,
            Hold::Moving);
  EXPECT_EQ(update(rule, incomplete, kPress + milliseconds(2999)).state,
            CaptureState::Waiting);
  // Between sets, it waits with the last set's hold.
  CaptureDecision decision = tick(rule, kPress + milliseconds(2999));
  EXPECT_EQ(decision.state, CaptureState::Waiting);
  EXPECT_EQ(decision.message, "SYN2 missing");
  // A set at the deadline is too late, even a still one.
  decision = update(rule, still_set(), kPress + milliseconds(3000));
  EXPECT_EQ(decision.state, CaptureState::Failed);
  EXPECT_EQ(decision.hold, Hold::Incomplete);
  EXPECT_EQ(decision.message, "SYN2 missing");
  EXPECT_FALSE(rule.armed());

  // Moving last.
  rule.arm(kPress);
  update(rule, still_set(0.82), kPress + milliseconds(10));
  decision = tick(rule, kPress + milliseconds(3500));
  EXPECT_EQ(decision.state, CaptureState::Failed);
  EXPECT_EQ(decision.hold, Hold::Moving);
  EXPECT_EQ(decision.message, "board moving (0.82 px)");
}

TEST(CaptureRule, FailsWithNoSetSinceThePress) {
  CaptureRuleOptions options;
  options.timeout = milliseconds(500);
  CaptureRule rule = rig_rule(options);
  rule.arm(kPress);
  CaptureDecision decision = tick(rule, kPress + milliseconds(499));
  EXPECT_EQ(decision.state, CaptureState::Waiting);
  EXPECT_EQ(decision.hold, Hold::NoSet);
  EXPECT_EQ(decision.message, "no set since the press");
  decision = tick(rule, kPress + milliseconds(500));
  EXPECT_EQ(decision.state, CaptureState::Failed);
  EXPECT_EQ(decision.hold, Hold::NoSet);
  EXPECT_EQ(decision.message, "no set since the press");
  // Re-armed, its hold starts over.
  rule.arm(kPress + milliseconds(600));
  update(rule, still_set(1.0), kPress + milliseconds(650));
  EXPECT_EQ(tick(rule, kPress + milliseconds(1100)).hold, Hold::Moving);
  rule.arm(kPress + milliseconds(1200));
  EXPECT_EQ(tick(rule, kPress + milliseconds(1250)).hold, Hold::NoSet);
}

// A wide spread and too few views are warned of, never refused.
TEST(CaptureRule, WarnsOfSpreadAndFewViewsButTakesTheSet) {
  CaptureRule rule = rig_rule();
  rule.arm(kPress);
  LiveSet set = still_set();
  set.spread_ms = 5.0;  // not above 5 ms
  set.checks[0].verdict = ViewVerdict::Collinear;
  set.checks[1].verdict = ViewVerdict::None;
  set.checks[2].verdict = ViewVerdict::Oblique;
  CaptureDecision decision = update(rule, set, kPress);
  EXPECT_EQ(decision.state, CaptureState::Take);
  EXPECT_EQ(decision.usable_views, 2);
  EXPECT_FALSE(decision.spread_high || decision.few_views);

  rule.arm(kPress);
  set.spread_ms = 7.25;
  set.checks[2].verdict = ViewVerdict::None;
  decision = update(rule, set, kPress);
  EXPECT_EQ(decision.state, CaptureState::Take);
  EXPECT_EQ(decision.usable_views, 1);
  EXPECT_TRUE(decision.spread_high);
  EXPECT_TRUE(decision.few_views);
  EXPECT_EQ(decision.message,
            "frames spread over 7.25 ms; seen by 1 camera: adds nothing to "
            "the extrinsics");

  rule.arm(kPress);
  set.spread_ms = 0.0;
  set.checks[3].verdict = ViewVerdict::None;
  decision = update(rule, set, kPress);
  EXPECT_EQ(decision.state, CaptureState::Take);
  EXPECT_EQ(decision.message,
            "seen by 0 cameras: adds nothing to the "
            "extrinsics");

  CaptureRuleOptions options;
  options.spread_warn_ms = 8.0;
  options.min_views = 0;
  CaptureRule lenient = rig_rule(options);
  lenient.arm(kPress);
  set.spread_ms = 7.25;
  decision = update(lenient, set, kPress);
  EXPECT_FALSE(decision.spread_high || decision.few_views);
}

// The set it would take finds the writer full: refused at once, not waited
// on. A writer full under a set it would not take holds nothing.
TEST(CaptureRule, RefusesAtOnceWhenTheWriterIsFull) {
  CaptureRule rule = rig_rule();
  rule.arm(kPress);
  LiveSet full = still_set(0.5);
  full.saver_full = true;
  EXPECT_EQ(update(rule, full, kPress).hold, Hold::Moving);
  full.motion_px = 0.1;
  const CaptureDecision decision = update(rule, full, kPress);
  EXPECT_EQ(decision.state, CaptureState::Failed);
  EXPECT_EQ(decision.hold, Hold::SaverFull);
  EXPECT_EQ(decision.message, "the writer's queue is full");
  EXPECT_FALSE(rule.armed());
}

TEST(CaptureRule, RefusesASetThatDoesNotFitAndStaysArmed) {
  CaptureRule rule = rig_rule();
  rule.arm(kPress);
  const auto refused = [&](const LiveSet& set) {
    core::Result<CaptureDecision> decision = CaptureDecision{};
    EXPECT_NO_THROW(decision = rule.update(set, kPress));
    EXPECT_EQ(decision.status().domain(), Code::InvalidArgument);
    EXPECT_TRUE(rule.armed());
    return decision.status().message();
  };
  LiveSet set = still_set();
  set.present.pop_back();
  EXPECT_EQ(refused(set),
            "CaptureRule::update: the set has 3 frames and 4 checks for 4 "
            "cameras");
  set = still_set();
  set.checks.emplace_back();
  EXPECT_EQ(refused(set),
            "CaptureRule::update: the set has 4 frames and 5 checks for 4 "
            "cameras");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const double motion : {-0.1, nan}) {
    EXPECT_EQ(refused(still_set(motion)),
              "CaptureRule::update: the motion must be at least 0 and finite");
  }
  for (const double spread : {-1.0, nan}) {
    set = still_set();
    set.spread_ms = spread;
    EXPECT_EQ(refused(set),
              "CaptureRule::update: the spread must be at least 0 and finite");
  }
}

TEST(CaptureRule, RefusesARigOrOptionsItCannotUse) {
  const auto refused = [](std::vector<std::string> ids,
                          const CaptureRuleOptions& options) {
    core::Result<CaptureRule> rule = core::Status::invalid_argument("");
    EXPECT_NO_THROW(rule = CaptureRule::create(ids, options));
    EXPECT_EQ(rule.status().domain(), Code::InvalidArgument);
    return rule.status().message();
  };
  EXPECT_EQ(refused({}, {}), "CaptureRule: no cameras");
  EXPECT_EQ(refused({"A", ""}, {}), "CaptureRule: camera 1 has no id");
  EXPECT_EQ(refused({"A", "B", "A"}, {}),
            "CaptureRule: camera id \"A\" appears twice");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const double still : {0.0, nan}) {
    CaptureRuleOptions options;
    options.still_px = still;
    EXPECT_EQ(refused({"A"}, options),
              "CaptureRule: options.still_px must be positive and finite");
  }
  CaptureRuleOptions options;
  options.timeout = milliseconds(0);
  EXPECT_EQ(refused({"A"}, options),
            "CaptureRule: options.timeout must be positive");
  for (const double spread : {-1.0, nan}) {
    options = {};
    options.spread_warn_ms = spread;
    EXPECT_EQ(refused({"A"}, options),
              "CaptureRule: options.spread_warn_ms must be at least 0 and "
              "finite");
  }
  options = {};
  options.min_views = -1;
  EXPECT_EQ(refused({"A"}, options),
            "CaptureRule: options.min_views must be at least 0");
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
