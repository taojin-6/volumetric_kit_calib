// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file live_pair.hpp
/// @brief Waiting on calib_rig's live worker, and showing it a fake stereo
///        pair's pictures as a person shows a board: the snapshot after a
///        number, the capture rule armed, a still pair taken. Its users link
///        calib_app.

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "fake_stereo.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"

namespace volumetric_kit::calib::test {

/// How long a wait on the live worker lasts before it fails the test.
inline constexpr auto kLivePatience = std::chrono::seconds(30);

/// @brief The live worker's next snapshot after number @p after.
/// @param live    The worker.
/// @param after   The last snapshot's number seen; 0 for none.
/// @param errors  Where what the worker reports on the way is kept; null:
///                each is a failure of the test.
/// @return The snapshot; null, a failure of the test, when none comes within
///         @ref kLivePatience.
inline std::shared_ptr<const app::LiveSnapshot> next_snapshot(
    app::LiveWorker& live, std::uint64_t after,
    std::vector<core::Status>* errors = nullptr) {
  const auto give_up = std::chrono::steady_clock::now() + kLivePatience;
  while (std::chrono::steady_clock::now() < give_up) {
    for (core::Status& error : live.take_errors()) {
      if (errors != nullptr) {
        errors->push_back(std::move(error));
      } else {
        ADD_FAILURE() << error.message();
      }
    }
    std::shared_ptr<const app::LiveSnapshot> s = live.take_snapshot();
    if (s && s->number > after) return s;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ADD_FAILURE() << "no snapshot after " << after;
  return nullptr;
}

/// @brief Wait until the capture rule is armed: it answers a poll with no
///        set with `Waiting`. Fails the test after @ref kLivePatience.
/// @param live  The worker.
inline void wait_armed(const app::LiveWorker& live) {
  const auto give_up = std::chrono::steady_clock::now() + kLivePatience;
  while (live.decision().state != quality::CaptureState::Waiting) {
    ASSERT_LT(std::chrono::steady_clock::now(), give_up);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

/// @brief A fake stereo pair's pictures shown to a live worker as a person
///        shows a board: each pair one frame after the last (30 fps), the
///        right picture 2.4 ms after the left, as the lab's free-running pair
///        drifts.
///
/// @code
/// StillPairs shown(pair, *live, {960, 540});
/// const auto taken = shown.capture(left, right);
/// EXPECT_EQ(taken->decision.state, quality::CaptureState::Take);
/// @endcode
class StillPairs {
 public:
  /// @param pair  The fake sensor the worker's source polls.
  /// @param live  The worker.
  /// @param size  The pictures' size.
  StillPairs(FakeStereo& pair, app::LiveWorker& live,
             sensor::camera::ImageSize size)
      : pair_(pair), live_(live), size_(size) {}

  /// @brief Show one pair.
  /// @return The snapshot after it; null, a failure of the test, when none
  ///         comes.
  std::shared_ptr<const app::LiveSnapshot> show(
      const std::vector<std::uint8_t>& left,
      const std::vector<std::uint8_t>& right) {
    constexpr std::uint64_t kStartNs = 1'760'000'000'000'000'000ull;
    constexpr std::uint64_t kPeriodNs = 33'333'333;
    ++tick_;
    pair_.push(host_pair(size_.width, size_.height, left, right,
                         kStartNs + tick_ * kPeriodNs, 2'400'000, tick_));
    std::shared_ptr<const app::LiveSnapshot> s = next_snapshot(live_, seen_);
    if (s) seen_ = s->number;
    return s;
  }

  /// @brief Show the pair as the board arrives there, arm the capture, and
  ///        show it again, held still.
  /// @return The second snapshot, whose decision is the capture rule's; null,
  ///         a failure of the test, when a snapshot did not come or the rule
  ///         would not arm.
  std::shared_ptr<const app::LiveSnapshot> capture(
      const std::vector<std::uint8_t>& left,
      const std::vector<std::uint8_t>& right) {
    if (show(left, right) == nullptr) return nullptr;
    if (!live_.arm()) {
      ADD_FAILURE() << "the capture rule did not arm";
      return nullptr;
    }
    wait_armed(live_);
    return show(left, right);
  }

 private:
  FakeStereo& pair_;
  app::LiveWorker& live_;
  sensor::camera::ImageSize size_;
  std::uint64_t tick_ = 0;
  std::uint64_t seen_ = 0;
};

}  // namespace volumetric_kit::calib::test
