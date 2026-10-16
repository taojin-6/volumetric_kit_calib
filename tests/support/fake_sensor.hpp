// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file fake_sensor.hpp
/// @brief A stand-in RGB-D camera for the app tier's tests: frames the test
///        pushes, which a sensor array drains on its own thread, so no test
///        opens a camera.

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"

namespace volumetric_kit::calib::test {

/// @brief A camera whose frames the test pushes; `drain` hands them over.
///        Thread-safe: the test pushes while a worker drains.
class FakeSensor final : public recon::sensor::IRgbdSensor {
 public:
  /// @brief A camera reporting @p info.
  explicit FakeSensor(recon::sensor::SensorInfo info)
      : info_(std::move(info)) {}

  /// @brief Queue @p frame for the next drain.
  void push(recon::sensor::RgbdFrame frame) {
    const std::lock_guard<std::mutex> lock(mutex_);
    frames_.push_back(std::move(frame));
  }
  /// @brief Make the next drain throw, as a driver's bug would.
  void throw_on_drain() { throw_.store(true); }
  /// @return Whether it is started.
  bool started() const { return started_.load(); }
  /// @return How often it was stopped.
  int stops() const { return stops_.load(); }

  const recon::sensor::SensorInfo& info() const noexcept override {
    return info_;
  }
  core::Status set_queue_depth(std::size_t frames) override {
    return frames == 0 ? core::Status::invalid_argument("depth 0")
                       : core::Status{};
  }
  core::Status start() override {
    started_.store(true);
    return {};
  }
  void stop() noexcept override {
    started_.store(false);
    ++stops_;
    const std::lock_guard<std::mutex> lock(mutex_);
    frames_.clear();
  }
  core::Result<std::optional<recon::sensor::RgbdFrame>> poll() override {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (frames_.empty()) return no_frame();
    recon::sensor::RgbdFrame frame = std::move(frames_.back());
    frames_.clear();
    return some_frame(std::move(frame));
  }
  core::Status drain(std::vector<recon::sensor::RgbdFrame>* out) override {
    if (throw_.exchange(false)) throw std::runtime_error("the driver threw");
    const std::lock_guard<std::mutex> lock(mutex_);
    for (recon::sensor::RgbdFrame& frame : frames_) {
      out->push_back(std::move(frame));
    }
    frames_.clear();
    return {};
  }
  recon::sensor::SensorStats stats() const noexcept override { return {}; }

 private:
  recon::sensor::SensorInfo info_;
  std::mutex mutex_;
  std::vector<recon::sensor::RgbdFrame> frames_;
  std::atomic<bool> started_{false};
  std::atomic<bool> throw_{false};
  std::atomic<int> stops_{0};
};

}  // namespace volumetric_kit::calib::test
