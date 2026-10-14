// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/capture_rule.hpp"

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"

namespace volumetric_kit::calib::quality {
namespace {

core::Status refuse_option(const char* name, const char* rule) {
  return core::Status::invalid_argument(std::string("CaptureRule: options.") +
                                        name + " must be " + rule);
}

// A measure as messages show it: two decimals.
std::string format(double value) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(2) << value;
  return out.str();
}

}  // namespace

CaptureRule::CaptureRule(std::vector<std::string> camera_ids,
                         const CaptureRuleOptions& options)
    : camera_ids_(std::move(camera_ids)), options_(options) {}

core::Result<CaptureRule> CaptureRule::create(
    const std::vector<std::string>& camera_ids,
    const CaptureRuleOptions& options) {
  return compute::guarded("CaptureRule", [&]() -> core::Result<CaptureRule> {
    VKC_TRY(solve::check_camera_ids(camera_ids).with_context("CaptureRule"));
    if (!(options.still_px > 0.0) || !std::isfinite(options.still_px)) {
      return refuse_option("still_px", "positive and finite");
    }
    if (options.timeout <= std::chrono::milliseconds::zero()) {
      return refuse_option("timeout", "positive");
    }
    if (!(options.spread_warn_ms >= 0.0) ||
        !std::isfinite(options.spread_warn_ms)) {
      return refuse_option("spread_warn_ms", "at least 0 and finite");
    }
    if (options.min_views < 0) return refuse_option("min_views", "at least 0");
    return CaptureRule(camera_ids, options);
  });
}

bool CaptureRule::arm(Clock::time_point now) noexcept {
  if (armed_) return false;
  armed_ = true;
  deadline_ = now + options_.timeout;
  last_hold_ = Hold::NoSet;
  last_message_.clear();
  return true;
}

CaptureDecision CaptureRule::held(CaptureState state) const {
  CaptureDecision decision;
  decision.state = state;
  decision.hold = last_hold_;
  decision.message =
      last_hold_ == Hold::NoSet ? "no set since the press" : last_message_;
  return decision;
}

core::Result<CaptureDecision> CaptureRule::update(const LiveSet& set,
                                                  Clock::time_point now) {
  return compute::guarded(
      "CaptureRule::update", [&]() -> core::Result<CaptureDecision> {
        const std::size_t k = camera_ids_.size();
        if (set.present.size() != k || set.checks.size() != k) {
          return core::Status::invalid_argument(
              "CaptureRule::update: the set has " +
              std::to_string(set.present.size()) + " frames and " +
              std::to_string(set.checks.size()) + " checks for " +
              std::to_string(k) + " cameras");
        }
        if (set.motion_px &&
            (!(*set.motion_px >= 0.0) || !std::isfinite(*set.motion_px))) {
          return core::Status::invalid_argument(
              "CaptureRule::update: the motion must be at least 0 and finite");
        }
        if (!(set.spread_ms >= 0.0) || !std::isfinite(set.spread_ms)) {
          return core::Status::invalid_argument(
              "CaptureRule::update: the spread must be at least 0 and finite");
        }
        // Every allocation comes before the rule changes, so a failed one
        // leaves it as it was.
        CaptureDecision decision;
        if (!armed_) return decision;
        if (now >= deadline_) {
          decision = held(CaptureState::Failed);
          armed_ = false;
          return decision;
        }

        std::string missing;
        for (std::size_t c = 0; c < k; ++c) {
          if (set.present[c]) continue;
          missing += missing.empty() ? "" : ", ";
          missing += camera_ids_[c];
        }
        if (!missing.empty()) {
          decision.hold = Hold::Incomplete;
          decision.message = missing + " missing";
        } else if (!set.motion_px) {
          decision.hold = Hold::NoBoard;
          decision.message = "board not seen in two sets running";
        } else if (*set.motion_px >= options_.still_px) {
          decision.hold = Hold::Moving;
          decision.message = "board moving (" + format(*set.motion_px) + " px)";
        } else if (set.saver_full) {
          decision.state = CaptureState::Failed;
          decision.hold = Hold::SaverFull;
          decision.message = "the writer's queue is full";
          armed_ = false;
          return decision;
        }
        if (decision.hold != Hold::None) {
          decision.state = CaptureState::Waiting;
          std::string remembered = decision.message;
          last_hold_ = decision.hold;
          last_message_ = std::move(remembered);
          return decision;
        }

        decision.state = CaptureState::Take;
        decision.stillness_px = *set.motion_px;
        for (const ViewCheck& check : set.checks) {
          if (is_usable(check.verdict)) ++decision.usable_views;
        }
        decision.spread_high = set.spread_ms > options_.spread_warn_ms;
        decision.few_views = decision.usable_views < options_.min_views;
        if (decision.spread_high) {
          decision.message =
              "frames spread over " + format(set.spread_ms) + " ms";
        }
        if (decision.few_views) {
          decision.message += decision.message.empty() ? "" : "; ";
          decision.message +=
              "seen by " + std::to_string(decision.usable_views) +
              (decision.usable_views == 1 ? " camera" : " cameras") +
              ": adds nothing to the extrinsics";
        }
        armed_ = false;
        return decision;
      });
}

core::Result<CaptureDecision> CaptureRule::tick(Clock::time_point now) {
  return compute::guarded(
      "CaptureRule::tick", [&]() -> core::Result<CaptureDecision> {
        if (!armed_) return CaptureDecision{};
        if (now < deadline_) return held(CaptureState::Waiting);
        CaptureDecision decision = held(CaptureState::Failed);
        armed_ = false;
        return decision;
      });
}

}  // namespace volumetric_kit::calib::quality
