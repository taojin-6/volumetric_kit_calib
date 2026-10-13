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
};

extern thread_local AllocationFailure allocation_failure;

class FailAllocation {
 public:
  explicit FailAllocation(std::size_t after) noexcept {
    allocation_failure = {after, true, false};
  }
  ~FailAllocation() { allocation_failure.armed = false; }
  FailAllocation(const FailAllocation&) = delete;
  FailAllocation& operator=(const FailAllocation&) = delete;
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

}  // namespace volumetric_kit::calib::test
