// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// guarded(): a body's result passes through, and whatever it throws -- an
// OpenCV error by its code, anything else as the core maps it -- comes back
// as a Status, never as an exception.

#include "volumetric_kit/calib/compute/guarded.hpp"

#include <new>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

namespace volumetric_kit::calib::compute {
namespace {

using Code = core::Status::Code;

// What guarded() returns for a body that throws `exception`.
template <class E>
core::Status thrown(const E& exception) {
  core::Status status;
  EXPECT_NO_THROW(
      status = guarded("thrower", [&]() -> core::Status { throw exception; }));
  return status;
}

cv::Exception opencv_error(cv::Error::Code code) {
  return cv::Exception(code, "the error", "the_function", "file.cpp", 7);
}

TEST(Guarded, PassesTheBodysResultThrough) {
  EXPECT_TRUE(guarded("ok", [] { return core::Status{}; }).ok());
  const core::Status failed = guarded(
      "failed", [] { return core::Status::numerical("did not converge"); });
  EXPECT_EQ(failed.domain(), Code::Numerical);
  EXPECT_EQ(failed.message(), "did not converge");

  const core::Result<int> value =
      guarded("value", []() -> core::Result<int> { return 42; });
  ASSERT_TRUE(value.ok());
  EXPECT_EQ(*value, 42);
  const core::Result<int> error = guarded("error", []() -> core::Result<int> {
    return core::Status::not_found("nothing");
  });
  EXPECT_EQ(error.status().domain(), Code::NotFound);
}

TEST(Guarded, NeverThrows) {
  const auto thrower = []() -> core::Status { throw std::runtime_error("x"); };
  constexpr std::string_view what = "x";
  static_assert(noexcept(guarded(what, thrower)));
  EXPECT_FALSE(guarded(what, thrower).ok());
}

TEST(Guarded, MapsOpenCVsMemoryAndArgumentCodes) {
  EXPECT_EQ(thrown(opencv_error(cv::Error::StsNoMem)).domain(),
            Code::OutOfMemory);
  for (const cv::Error::Code code :
       {cv::Error::StsBadArg, cv::Error::StsBadSize, cv::Error::StsOutOfRange,
        cv::Error::StsUnmatchedSizes, cv::Error::StsBadFlag}) {
    const core::Status status = thrown(opencv_error(code));
    EXPECT_EQ(status.domain(), Code::InvalidArgument) << code;
    EXPECT_FALSE(status.backend()) << code;
  }
}

TEST(Guarded, ReportsOtherOpenCVCodesAsTheBackends) {
  for (const cv::Error::Code code :
       {cv::Error::StsError, cv::Error::StsAssert, cv::Error::StsNotImplemented,
        cv::Error::StsNoConv, cv::Error::GpuNotSupported}) {
    const core::Status status = thrown(opencv_error(code));
    EXPECT_EQ(status.domain(), Code::Backend) << code;
    EXPECT_EQ(status.backend(), core::Status::Backend::Other) << code;
    EXPECT_EQ(status.detail(), code);
  }
}

// A backend failure cannot carry 0, which is success: OpenCV's "no error"
// becomes its unspecified error.
TEST(Guarded, ReportsCodeZeroAsAnUnspecifiedError) {
  const core::Status status = thrown(opencv_error(cv::Error::StsOk));
  EXPECT_EQ(status.domain(), Code::Backend);
  EXPECT_EQ(status.backend(), core::Status::Backend::Other);
  EXPECT_EQ(status.detail(), cv::Error::StsError);
}

TEST(Guarded, NamesWhatWasCalledAndWhatOpenCVSaid) {
  const std::string message =
      thrown(opencv_error(cv::Error::StsBadArg)).message();
  EXPECT_EQ(message.rfind("thrower: ", 0), 0u) << message;
  EXPECT_NE(message.find("the error"), std::string::npos) << message;
  EXPECT_NE(message.find("the_function"), std::string::npos) << message;
  EXPECT_NE(message.back(), '\n');
}

// An OpenCV call's own failure, not one built by hand.
TEST(Guarded, CatchesARealOpenCVFailure) {
  core::Result<cv::Mat> sum = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(sum = guarded("add", []() -> core::Result<cv::Mat> {
                    cv::Mat out;
                    cv::add(cv::Mat::zeros(2, 2, CV_64F),
                            cv::Mat::zeros(3, 3, CV_64F), out);
                    return out;
                  }));
  EXPECT_EQ(sum.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(sum.status().message().rfind("add: ", 0), 0u)
      << sum.status().message();
}

TEST(Guarded, MapsOtherExceptionsAsTheCoreDoes) {
  EXPECT_EQ(thrown(std::bad_alloc()).domain(), Code::OutOfMemory);

  const core::Status runtime = thrown(std::runtime_error("boom"));
  EXPECT_EQ(runtime.domain(), Code::InvalidArgument);
  EXPECT_EQ(runtime.message(), "thrower: boom");

  const core::Status unknown = thrown(42);
  EXPECT_EQ(unknown.domain(), Code::InvalidArgument);
  EXPECT_EQ(unknown.message(), "thrower: unknown exception");
}

TEST(Guarded, CurrentExceptionStatusServesAnyCatch) {
  core::Status status;
  try {
    throw opencv_error(cv::Error::StsOutOfRange);
  } catch (...) {
    status = current_exception_status("worker");
  }
  EXPECT_EQ(status.domain(), Code::InvalidArgument);
  EXPECT_EQ(status.message().rfind("worker: ", 0), 0u) << status.message();
}

}  // namespace
}  // namespace volumetric_kit::calib::compute
