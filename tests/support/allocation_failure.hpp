// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

#include <cstddef>
#include <optional>
#include <type_traits>

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

// Sweep every allocation in a rejected call, including with_context and
// Result construction, without depending on string allocation counts or
// small-string buffer sizes. The first uninjected call also checks retry.
template <class Fn>
void expect_oom_when_reporting(const Fn& fn, core::Status::Code expected) {
  const auto original = fn();
  ASSERT_EQ(original.status().domain(), expected);
  for (std::size_t after = 0; after < 32; ++after) {
    SCOPED_TRACE(after);
    std::optional<std::invoke_result_t<const Fn&>> result;
    EXPECT_NO_THROW({
      const FailAllocation failure(after);
      result.emplace(fn());
    });
    ASSERT_TRUE(result.has_value());
    if (!allocation_failure.injected) {
      EXPECT_GT(after, 0u) << "the refusal must exercise allocation failure";
      EXPECT_EQ(result->status().domain(), expected);
      EXPECT_EQ(result->status().message(), original.status().message());
      return;
    }
    EXPECT_EQ(result->status().domain(), core::Status::Code::OutOfMemory);
  }
  FAIL() << "did not reach a refusal without an injected allocation failure";
}

// Fail the n-th allocation of `fn`, for n = 0, 1, ... until a run allocates
// fewer: each run that failed one returns OutOfMemory, or what `swallowed`
// accepts as a failure OpenCV swallowed; the first that did not returns what
// an unhindered run does.
template <class Fn, class Swallowed>
void expect_oom_anywhere(const Fn& fn, const Swallowed& swallowed) {
  const auto original = fn();
  std::size_t oom = 0;
  for (std::size_t after = 0; after < 100000; ++after) {
    std::optional<std::invoke_result_t<const Fn&>> result;
    EXPECT_NO_THROW({
      const FailAllocation failure(after);
      result.emplace(fn());
    });
    ASSERT_TRUE(result.has_value());
    if (!allocation_failure.injected) {
      EXPECT_GT(oom, 0u);
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
  }
  FAIL() << "did not finish without an injected allocation failure";
}

template <class Fn>
void expect_oom_anywhere(const Fn& fn) {
  expect_oom_anywhere(fn, [](const auto&) { return false; });
}

}  // namespace volumetric_kit::calib::test
