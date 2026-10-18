// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The out-of-memory sweep itself (support/allocation_failure.hpp): what it
// fails, VC_TEST_OOM_EXHAUSTIVE, and its refusal of a call that allocates
// otherwise than in its census.

#include <cstddef>
#include <cstdlib>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include "allocation_failure.hpp"

namespace volumetric_kit::calib::test {
namespace {

// `n` allocations (at most 8) from a loop, then one more; what they make
// outlives the call. Each pass of the loop, not unrolled, allocates from the
// same return address, so from one call stack.
void allocate(std::size_t n) {
  static std::unique_ptr<int> held[9];
#pragma GCC unroll 1
  for (std::size_t i = 0; i < n; ++i) held[i].reset(new int(0));
  held[8].reset(new int(0));
}

// Each run's allocation to fail, and whether it failed one.
struct Runs {
  std::vector<std::size_t> after;
  std::vector<bool> injected;
};

// A sweep of `allocate`, its loop allocating `census` times in the census and
// `later` times in each run after.
Runs sweep(std::size_t census, std::size_t later) {
  Runs runs;
  sweep_allocations([&](std::size_t after) {
    const std::size_t n = runs.after.empty() ? census : later;
    try {
      const FailAllocation failure(after);
      allocate(n);
    } catch (const std::bad_alloc&) {
    }
    runs.after.push_back(after);
    runs.injected.push_back(allocation_failure.injected);
  });
  return runs;
}

// VC_TEST_OOM_EXHAUSTIVE set to 1, or unset, while alive; as it was after.
class Exhaustive {
 public:
  explicit Exhaustive(bool on) {
    if (const char* was = std::getenv(kName)) was_ = was;
    if (on) {
      setenv(kName, "1", 1);
    } else {
      unsetenv(kName);
    }
  }
  ~Exhaustive() {
    if (was_) {
      setenv(kName, was_->c_str(), 1);
    } else {
      unsetenv(kName);
    }
  }
  Exhaustive(const Exhaustive&) = delete;
  Exhaustive& operator=(const Exhaustive&) = delete;

 private:
  static constexpr const char* kName = "VC_TEST_OOM_EXHAUSTIVE";
  std::optional<std::string> was_;
};

// The census, the loop's first allocation, the one after the loop, and none.
TEST(SweepAllocations, FailsTheFirstAllocationFromEachCallStack) {
  const Exhaustive off(false);
  const Runs runs = sweep(4, 4);
  EXPECT_EQ(runs.after, (std::vector<std::size_t>{0, 0, 4, 5}));
  EXPECT_EQ(runs.injected, (std::vector<bool>{false, true, true, false}));
}

TEST(SweepAllocations, FailsEveryAllocationWhenExhaustive) {
  const Exhaustive on(true);
  const Runs runs = sweep(4, 4);
  EXPECT_EQ(runs.after, (std::vector<std::size_t>{0, 0, 1, 2, 3, 4, 5}));
  EXPECT_EQ(runs.injected,
            (std::vector<bool>{false, true, true, true, true, true, false}));
}

// The census sees the loop allocate once; each run after, never. Failing
// allocation 0 then fails the one after the loop, and allocation 1 is not
// made.
TEST(SweepAllocations, RefusesACallThatAllocatesOtherwise) {
  const Exhaustive off(false);
  ::testing::TestPartResultArray failures;
  {
    const ::testing::ScopedFakeTestPartResultReporter reporter(
        ::testing::ScopedFakeTestPartResultReporter::
            INTERCEPT_ONLY_CURRENT_THREAD,
        &failures);
    sweep(1, 0);
  }
  ASSERT_EQ(failures.size(), 2);
  EXPECT_NE(std::string(failures.GetTestPartResult(0).message())
                .find("allocation 0 of 2 came from another call stack"),
            std::string::npos)
      << failures.GetTestPartResult(0).message();
  EXPECT_NE(std::string(failures.GetTestPartResult(1).message())
                .find("allocation 1 of the census's 2 was not made"),
            std::string::npos)
      << failures.GetTestPartResult(1).message();
}

}  // namespace
}  // namespace volumetric_kit::calib::test
