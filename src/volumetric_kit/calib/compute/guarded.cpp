// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/compute/guarded.hpp"

#include <string>

#include <opencv2/core.hpp>

#include "volumetric_kit/core/base/exception_status.hpp"

namespace volumetric_kit::calib::compute {
namespace {

// The status a cv::Exception maps to; it may throw std::bad_alloc while it
// builds the message.
core::Status opencv_status(std::string_view what, const cv::Exception& e) {
  std::string message(what);
  message += ": ";
  message += e.what();
  // OpenCV ends its formatted message with a line break.
  while (!message.empty() && message.back() == '\n') message.pop_back();

  switch (e.code) {
    case cv::Error::StsNoMem:
      return core::Status::out_of_memory(std::move(message));
    case cv::Error::StsBadArg:
    case cv::Error::StsBadSize:
    case cv::Error::StsOutOfRange:
    case cv::Error::StsUnmatchedSizes:
    case cv::Error::StsBadFlag:
      return core::Status::invalid_argument(std::move(message));
    default:
      // A backend failure needs a nonzero code (Status::backend_error).
      return core::Status::backend_error(
          core::Status::Backend::Other,
          e.code != cv::Error::StsOk ? e.code : cv::Error::StsError,
          std::move(message));
  }
}

}  // namespace

core::Status current_exception_status(std::string_view what) noexcept {
  try {
    throw;
  } catch (const cv::Exception& e) {
    try {
      return opencv_status(what, e);
    } catch (...) {
      // The message could not be allocated: the core's mapping reports that
      // without allocating.
      return core::exception_status(what, core::Status::Code::InvalidArgument);
    }
  } catch (...) {
    return core::exception_status(what, core::Status::Code::InvalidArgument);
  }
}

}  // namespace volumetric_kit::calib::compute
