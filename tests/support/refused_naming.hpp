// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file refused_naming.hpp
/// @brief A GoogleTest predicate for a refusal of a `Target` field.

#include <string>

#include <gtest/gtest.h>

#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::test {

/// @brief Whether @p status refuses as @p code, naming `target.<field>`.
/// @param status  The status checked.
/// @param code    The domain it must have.
/// @param field   The `Target` field its message must start with, followed
///                by a space.
/// @return Success, or a failure saying what the status was instead.
inline ::testing::AssertionResult RefusedNaming(const core::Status& status,
                                                core::Status::Code code,
                                                const std::string& field) {
  if (status.ok()) return ::testing::AssertionFailure() << "accepted";
  if (status.domain() != code) {
    return ::testing::AssertionFailure()
           << "refused as " << core::to_string(status.domain()) << ": "
           << status.message();
  }
  if (status.message().rfind("target." + field + " ", 0) != 0) {
    return ::testing::AssertionFailure()
           << "message is not about " << field << ": " << status.message();
  }
  return ::testing::AssertionSuccess();
}

/// @brief Whether @p status is an `InvalidArgument` naming `target.<field>`.
/// @param status  The status checked.
/// @param field   The `Target` field its message must start with.
/// @return Success, or a failure saying what the status was instead.
inline ::testing::AssertionResult RefusedNaming(const core::Status& status,
                                                const std::string& field) {
  return RefusedNaming(status, core::Status::Code::InvalidArgument, field);
}

}  // namespace volumetric_kit::calib::test
