// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file result_checks.hpp
/// @brief A test's hold on the core's `Result` and `Status`: a value it
///        cannot go on without, and a refusal's domain and message.

#include <stdexcept>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::test {

/// @brief @p result's value, for a step the test cannot go on without.
/// @param result  A result.
/// @return Its value; an error is thrown as a `std::runtime_error` with its
///         message, which fails the test.
///
/// @code
/// const std::unique_ptr<DepthChecks> checks =
///     value_of(DepthChecks::create(device, allocator));
/// @endcode
template <class T>
T value_of(core::Result<T> result) {
  if (!result.ok()) throw std::runtime_error(result.status().message());
  return *std::move(result);
}

/// @brief Throws @p status's message as a `std::runtime_error`, failing the
///        test, when it is an error.
/// @param status  A status.
inline void check(const core::Status& status) {
  if (!status.ok()) throw std::runtime_error(status.message());
}

/// @brief Expects @p status to be an `InvalidArgument` whose message starts
///        `<prefix>: ` and says @p says.
/// @param status  The status checked.
/// @param prefix  The call its message must start with.
/// @param says    Text its message must contain.
inline void expect_refused(const core::Status& status,
                           const std::string& prefix, const std::string& says) {
  EXPECT_EQ(status.domain(), core::Status::Code::InvalidArgument) << says;
  EXPECT_EQ(status.message().rfind(prefix + ": ", 0), 0u) << status.message();
  EXPECT_NE(status.message().find(says), std::string::npos) << status.message();
}

}  // namespace volumetric_kit::calib::test
