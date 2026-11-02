// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file fake_stereo.hpp
/// @brief A stand-in stereo sensor for the source and app tiers' tests: pairs
///        of host luma pictures the test pushes, newest kept, so no test opens
///        a camera.

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/stereo/stereo_sensor.hpp"

namespace volumetric_kit::calib::test {

/// @brief What a pushed pair's pictures hold on the host.
struct PairPixels {
  std::vector<std::uint8_t> left;
  std::vector<std::uint8_t> right;
};

/// @brief A stereo pair of `width` x `height` host luma pictures, @p left
///        and @p right (rows packed), the right one @p skew_ns after the left
///        one at @p timestamp_ns; it holds its bytes.
inline sensor::StereoFrame host_pair(std::uint32_t width, std::uint32_t height,
                                     std::vector<std::uint8_t> left,
                                     std::vector<std::uint8_t> right,
                                     std::uint64_t timestamp_ns,
                                     std::int64_t skew_ns,
                                     std::uint64_t sequence) {
  auto pixels = std::make_shared<PairPixels>();
  pixels->left = std::move(left);
  pixels->right = std::move(right);
  sensor::StereoFrame frame;
  for (sensor::YuvImage* side : {&frame.left, &frame.right}) {
    side->layout = sensor::YuvLayout::Luma;
    side->width = width;
    side->height = height;
    side->stride[0] = width;
    side->full_range = true;
  }
  frame.left.host = pixels->left.data();
  frame.right.host = pixels->right.data();
  frame.timestamp_ns = timestamp_ns;
  frame.skew_ns = skew_ns;
  frame.sequence = sequence;
  frame.pixels = std::move(pixels);
  return frame;
}

/// @brief A stereo sensor whose pairs the test pushes; `poll` takes the
///        newest, a pair replaced before a poll counted dropped. Thread-safe:
///        the test pushes while a worker polls.
class FakeStereo final : public sensor::IStereoSensor {
 public:
  /// @brief A sensor reporting @p info.
  explicit FakeStereo(sensor::StereoInfo info) : info_(std::move(info)) {}

  /// @brief Hold @p frame for the next poll, replacing one not taken.
  void push(sensor::StereoFrame frame) {
    const std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.received;
    if (held_) ++stats_.dropped;
    held_ = std::move(frame);
  }
  /// @brief Make every poll from now on fail, as a driver does once its
  ///        card is gone.
  void fail() { failing_.store(true); }
  /// @return Whether it is started.
  bool started() const { return started_.load(); }

  const sensor::StereoInfo& info() const noexcept override { return info_; }
  core::Status start() override {
    started_.store(true);
    const std::lock_guard<std::mutex> lock(mutex_);
    stats_ = {};
    return {};
  }
  void stop() noexcept override {
    started_.store(false);
    const std::lock_guard<std::mutex> lock(mutex_);
    held_.reset();
  }
  core::Result<std::optional<sensor::StereoFrame>> poll() override {
    if (failing_.load()) {
      return core::Status::io_error("the stereo card is gone");
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    std::optional<sensor::StereoFrame> frame = std::move(held_);
    held_.reset();
    if (frame) ++stats_.delivered;
    return frame;
  }
  sensor::SensorStats stats() const noexcept override {
    const std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
  }

 private:
  sensor::StereoInfo info_;
  mutable std::mutex mutex_;
  std::optional<sensor::StereoFrame> held_;
  sensor::SensorStats stats_;
  std::atomic<bool> started_{false};
  std::atomic<bool> failing_{false};
};

/// @brief A pair's info as a DeckLink pair opened without a calibration
///        gives it: both cameras 0 x 0, the host's clock, 30 fps, its skew
///        bound half a frame.
inline sensor::StereoInfo uncalibrated_pair_info() {
  sensor::StereoInfo info;
  info.id = "fake-pair";
  info.model = "fake stereo";
  info.clock = sensor::ClockDomain::Host;
  info.fps = 30;
  info.max_skew_ns = 16'666'666;
  return info;
}

}  // namespace volumetric_kit::calib::test
