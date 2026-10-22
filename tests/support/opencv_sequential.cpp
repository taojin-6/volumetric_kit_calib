// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Linked into every test under ThreadSanitizer (tests/CMakeLists.txt): OpenCV's
// parallel_for_ runs each loop on the thread that calls it. OpenCV's pool
// (oneTBB in Homebrew's OpenCV) is not instrumented, so ThreadSanitizer would
// take its hand-offs -- a loop body's writes on a pool thread, read by the
// caller after the loop -- for races.

#include <opencv2/core/utility.hpp>

namespace {

// Before main, so before any test starts a thread. One thread, not 0, which
// cv::getNumThreads would report as the pool's size to a test that saves and
// restores it.
[[maybe_unused]] const bool sequential = [] {
  cv::setNumThreads(1);
  return true;
}();

}  // namespace
