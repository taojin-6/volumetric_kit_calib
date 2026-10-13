// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

#include <cstddef>

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

}  // namespace volumetric_kit::calib::test
