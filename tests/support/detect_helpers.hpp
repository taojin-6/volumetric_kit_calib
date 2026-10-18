// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file detect_helpers.hpp
/// @brief What the detectors' tests share: an image as a `LumaView`, the
///        refusal they expect of a bad one, and an OpenCV that throws.

#include <cstdint>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/core/parallel/parallel_backend.hpp>

#include "volumetric_kit/calib/detect/luma.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::test {

/// @brief An 8-bit image as the detectors read it.
/// @param image  A `CV_8UC1` image.
/// @return A view of its pixels, rows `image.step[0]` bytes apart.
inline detect::LumaView luma(const cv::Mat& image) {
  return {static_cast<std::uint32_t>(image.cols),
          static_cast<std::uint32_t>(image.rows), image.step[0], image.data};
}

/// @brief Whether @p status is an `InvalidArgument` with @p message.
/// @param status   The status checked.
/// @param message  Its whole message.
/// @return Success, or a failure saying what the status was instead.
inline ::testing::AssertionResult Refused(const core::Status& status,
                                          const std::string& message) {
  if (status.ok()) return ::testing::AssertionFailure() << "accepted";
  if (status.domain() != core::Status::Code::InvalidArgument) {
    return ::testing::AssertionFailure()
           << "refused as " << core::to_string(status.domain()) << ": "
           << status.message();
  }
  if (status.message() != message) {
    return ::testing::AssertionFailure() << "refused with " << status.message();
  }
  return ::testing::AssertionSuccess();
}

/// @brief OpenCV's parallel_for_ backend replaced, while it lives, by one
///        that throws `code` instead of running the loop; OpenCV's detectors
///        run loops on it.
class ThrowingParallelFor {
 public:
  /// @param code  What the loop throws.
  explicit ThrowingParallelFor(cv::Error::Code code)
      : threads_(cv::getNumThreads()) {
    cv::parallel::setParallelForBackend(std::make_shared<Backend>(code), false);
    // parallel_for_ calls a backend only with more than one thread.
    cv::setNumThreads(4);
  }
  ~ThrowingParallelFor() {
    cv::parallel::setParallelForBackend(
        std::shared_ptr<cv::parallel::ParallelForAPI>(), false);
    cv::setNumThreads(threads_);
  }
  ThrowingParallelFor(const ThrowingParallelFor&) = delete;
  ThrowingParallelFor& operator=(const ThrowingParallelFor&) = delete;

 private:
  class Backend : public cv::parallel::ParallelForAPI {
   public:
    explicit Backend(cv::Error::Code code) : code_(code) {}
    void parallel_for(int, FN_parallel_for_body_cb_t, void*) override {
      throw cv::Exception(code_, "injected", "parallel_for", __FILE__,
                          __LINE__);
    }
    int getThreadNum() const override { return 0; }
    int getNumThreads() const override { return 4; }
    int setNumThreads(int) override { return 4; }
    const char* getName() const override { return "throwing"; }

   private:
    cv::Error::Code code_;
  };

  int threads_;
};

}  // namespace volumetric_kit::calib::test
