// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file set_on_exit.hpp
/// @brief A guard that sets a timeline to its value as a test leaves, so a
///        test that fails early leaves nothing waiting for the value.

#include <cstdint>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"

namespace volumetric_kit::calib::test {

/// @brief Sets @ref timeline to @ref value as it goes, unless it is there
///        already: a retire queue, a batch waiting on the device or a thread
///        taking a set is then never left waiting for it. Declared after what
///        waits, so it goes first.
///
/// @code
/// std::future<Taken> taken = std::async(std::launch::async, take);
/// const test::SetOnExit decoded_by_then{*ready, 1};  // before the join
/// @endcode
struct SetOnExit {
  core::TimelineSemaphore& timeline;  ///< The timeline; outlives the guard.
  std::uint64_t value;                ///< The value it reaches at the latest.
  ~SetOnExit() {
    const core::Result<std::uint64_t> now = timeline.value();
    if (now.ok() && now.value() < value) (void)timeline.signal(value);
  }
};

}  // namespace volumetric_kit::calib::test
