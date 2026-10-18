// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Out-of-memory sweeps. A sweep (sweep_allocations) runs a call once as a
// census, recording the call stack of each allocation it makes, then once
// failing the first allocation from each distinct call stack, and once failing
// none. A call stack is its return addresses, hashed, so every allocation site
// fails on each path that reaches it, in the first pass of any loop around it.
// Each allocation failed must come from the call stack the census recorded at
// its index, or the sweep fails rather than test another.
// VC_TEST_OOM_EXHAUSTIVE=1 fails every allocation instead, which takes
// minutes under the sanitizers.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::test {

// Replace allocation only in the calling thread, with no allocations in the
// test framework while armed. The replacement operators live in a separate
// translation unit so GCC does not mistake delete's free for a mismatch.
struct AllocationFailure {
  std::size_t remaining = 0;
  bool armed = false;
  bool injected = false;
  // Neither count nor fail an allocation made under a Vulkan loader's,
  // layer's or driver's frame (SpareVulkan).
  bool spare_vulkan = false;
  // Armed, record each allocation's call stack and fail none (the census).
  bool census = false;
  // The call stack of the allocation failed.
  std::uint64_t failed_stack = 0;
};

extern thread_local AllocationFailure allocation_failure;

class FailAllocation {
 public:
  explicit FailAllocation(std::size_t after) noexcept {
    allocation_failure.remaining = after;
    allocation_failure.armed = true;
    allocation_failure.injected = false;
  }
  ~FailAllocation() { allocation_failure.armed = false; }
  FailAllocation(const FailAllocation&) = delete;
  FailAllocation& operator=(const FailAllocation&) = delete;
};

// While alive, sweeps leave the Vulkan loader, layers and driver alone: they
// report host memory through VkResult, and a bad_alloc thrown through them
// is undefined. Each allocation then walks the stack, so only device tests
// take it.
class SpareVulkan {
 public:
  SpareVulkan() noexcept { allocation_failure.spare_vulkan = true; }
  ~SpareVulkan() { allocation_failure.spare_vulkan = false; }
  SpareVulkan(const SpareVulkan&) = delete;
  SpareVulkan& operator=(const SpareVulkan&) = delete;
};

namespace detail {

// Clears the census's call stacks; an armed FailAllocation records them.
void begin_census() noexcept;
// The allocations a sweep fails, by index, in order, of the `count` the
// census recorded.
std::vector<std::size_t> end_census(std::size_t& count);
// The call stack the census recorded for allocation `index`.
std::uint64_t census_stack(std::size_t index) noexcept;

}  // namespace detail

// Calls `attempt(after)` for each run of a sweep (above): the census, each
// allocation failed, then none. `attempt` arms FailAllocation(after) once,
// around the call alone, and checks what the call returned: as a failure's
// when allocation_failure.injected, else as an unhindered run's.
template <class Attempt>
void sweep_allocations(const Attempt& attempt) {
  std::vector<std::size_t> plan;
  std::size_t count = 0;
  for (std::size_t run = 0; run <= plan.size() + 1; ++run) {
    const bool census = run == 0;
    const std::size_t after = census               ? 0
                              : run <= plan.size() ? plan[run - 1]
                                                   : count;
    if (census) detail::begin_census();
    allocation_failure.census = census;
    attempt(after);  // From one call site, so each run's stacks match.
    allocation_failure.census = false;
    if (census) plan = detail::end_census(count);
    if (::testing::Test::HasFatalFailure()) return;
    if (census) continue;
    if (run > plan.size()) {
      EXPECT_FALSE(allocation_failure.injected)
          << "the call made more than its census's " << count << " allocations";
    } else if (!allocation_failure.injected) {
      ADD_FAILURE() << "allocation " << after << " of the census's " << count
                    << " was not made";
    } else {
      EXPECT_EQ(allocation_failure.failed_stack, detail::census_stack(after))
          << "allocation " << after << " of " << count
          << " came from another call stack than in the census";
    }
  }
}

// Sweep a rejected call, with_context and Result construction included,
// without depending on string allocation counts or small-string buffer sizes.
// The run failing none also checks retry.
template <class Fn>
void expect_oom_when_reporting(const Fn& fn, core::Status::Code expected) {
  const auto original = fn();
  ASSERT_EQ(original.status().domain(), expected);
  std::size_t oom = 0;
  sweep_allocations([&](std::size_t after) {
    SCOPED_TRACE(after);
    std::optional<std::invoke_result_t<const Fn&>> result;
    EXPECT_NO_THROW({
      const FailAllocation failure(after);
      result.emplace(fn());
    });
    ASSERT_TRUE(result.has_value());
    if (!allocation_failure.injected) {
      EXPECT_EQ(result->status().domain(), expected);
      EXPECT_EQ(result->status().message(), original.status().message());
      return;
    }
    EXPECT_EQ(result->status().domain(), core::Status::Code::OutOfMemory);
    ++oom;
  });
  EXPECT_GT(oom, 0u) << "the refusal must exercise allocation failure";
}

// Sweep `fn`: each run that failed an allocation returns OutOfMemory, or what
// `swallowed` accepts as a failure OpenCV swallowed; the run failing none
// returns what an unhindered run does.
template <class Fn, class Swallowed>
void expect_oom_anywhere(const Fn& fn, const Swallowed& swallowed) {
  const auto original = fn();
  std::size_t oom = 0;
  sweep_allocations([&](std::size_t after) {
    std::optional<std::invoke_result_t<const Fn&>> result;
    EXPECT_NO_THROW({
      const FailAllocation failure(after);
      result.emplace(fn());
    });
    ASSERT_TRUE(result.has_value());
    if (!allocation_failure.injected) {
      EXPECT_EQ(result->status().domain(), original.status().domain());
      EXPECT_EQ(result->status().message(), original.status().message());
      return;
    }
    if (result->status().domain() == core::Status::Code::OutOfMemory) {
      ++oom;
    } else {
      ASSERT_TRUE(swallowed(*result))
          << "allocation " << after << ": " << result->status().message();
    }
  });
  EXPECT_GT(oom, 0u);
}

template <class Fn>
void expect_oom_anywhere(const Fn& fn) {
  expect_oom_anywhere(fn, [](const auto&) { return false; });
}

}  // namespace volumetric_kit::calib::test
