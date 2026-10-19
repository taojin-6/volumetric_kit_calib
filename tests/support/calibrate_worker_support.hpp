// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file calibrate_worker_support.hpp
/// @brief Calibrate's tests' helpers: a report without its time, a
///        directory's entries, waiting on the worker, and holding a run as it
///        ends. Its users link calib_app and have nlohmann/json on their
///        include path.

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "volumetric_kit/calib/app/calibrate_worker.hpp"

namespace volumetric_kit::calib::test {

/// @brief A report without the one figure that changes from run to run.
/// @param report  `report.json`'s text.
/// @return The report, its `solver.seconds` left out.
inline nlohmann::json timeless(const std::string& report) {
  nlohmann::json doc = nlohmann::json::parse(report);
  doc["solver"].erase("seconds");
  return doc;
}

/// @brief What a directory holds, by name.
/// @param dir  The directory.
/// @return Its entries' names; none when it cannot be listed.
inline std::set<std::string> entries(const std::filesystem::path& dir) {
  std::set<std::string> out;
  std::error_code ec;
  for (auto it = std::filesystem::directory_iterator(dir, ec);
       !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
    out.insert(it->path().filename().string());
  }
  return out;
}

/// @brief Wait until @p done holds, polling.
/// @param done   What to wait for.
/// @param limit  How long to wait.
/// @return Whether it held before @p limit passed.
template <class Done>
bool wait_for(const Done& done,
              std::chrono::milliseconds limit = std::chrono::minutes(5)) {
  const auto until = std::chrono::steady_clock::now() + limit;
  while (!done()) {
    if (std::chrono::steady_clock::now() > until) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return true;
}

/// @brief Wait until @p worker is not busy.
/// @param worker  The worker.
/// @return Whether it went idle within @ref wait_for's limit.
inline bool idle(const app::CalibrateWorker& worker) {
  return wait_for([&] { return !worker.busy(); });
}

/// @brief @p kept as the capture worker publishes it.
/// @param kept  The snapshot.
/// @return It, shared.
inline std::shared_ptr<const app::KeptCaptures> shared(app::KeptCaptures kept) {
  return std::make_shared<const app::KeptCaptures>(std::move(kept));
}

/// @brief Holds the thread that reaches it until the test lets it go.
class Gate {
 public:
  /// @brief On the thread held: wait to be let go.
  void hold() {
    std::unique_lock<std::mutex> lock(mutex_);
    held_ = true;
    changed_.notify_all();
    changed_.wait(lock, [this] { return open_; });
  }
  /// @return Whether a thread is held, waiting a minute for one.
  bool held() {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, std::chrono::minutes(1),
                             [this] { return held_; });
  }
  /// @brief Let the thread held, and any after it, go.
  void let_go() {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      open_ = true;
    }
    changed_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable changed_;
  bool held_ = false;
  bool open_ = false;
};

/// @brief Lets a gate go as the test ends, before the worker declared ahead
///        of it is stopped, whatever assertion ended the test.
struct LetGoAtTheEnd {
  Gate& gate;  ///< The gate.
  ~LetGoAtTheEnd() { gate.let_go(); }
};

/// @brief @p kept, with its first capture's first depth freed only once
///        @p gate lets go: the worker frees its snapshot as its run ends, so
///        the run is held there, still busy, its outcome decided and not yet
///        posted. The caller keeps none of what it gives the worker.
/// @param kept  The snapshot; its first capture's first depth is replaced
///              by one of the same samples.
/// @param gate  The gate.
/// @return The snapshot, shared.
inline std::shared_ptr<const app::KeptCaptures> held_at_its_end(
    app::KeptCaptures kept, const std::shared_ptr<Gate>& gate) {
  auto first = std::make_shared<app::KeptCapture>(*kept.captures.front());
  first->depth.front() = std::shared_ptr<const std::vector<std::uint16_t>>(
      new std::vector<std::uint16_t>(*first->depth.front()),
      [gate](const std::vector<std::uint16_t>* depth) {
        gate->hold();
        delete depth;
      });
  kept.captures.front() = std::move(first);
  return shared(std::move(kept));
}

}  // namespace volumetric_kit::calib::test
