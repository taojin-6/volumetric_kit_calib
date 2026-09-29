// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file check.hpp
/// @brief Fail-fast contract checks for programmer errors.
///
/// `VC_CHECK` guards preconditions whose violation is a *bug*, not a runtime
/// condition a caller could recover from (e.g. reading the value of an error
/// @ref volumetric_kit::calib::Result). It aborts with a diagnostic rather than
/// throwing, so it works under `-fno-exceptions` on mobile. Recoverable
/// failures use @ref volumetric_kit::calib::Status instead (see result.hpp).

#include <cstdio>
#include <cstdlib>

namespace volumetric_kit::calib {
namespace detail {

/// @brief Print a contract-violation diagnostic and abort. Never returns.
/// @param expr  The stringified failing condition.
/// @param msg   Human-readable context.
/// @param file  Source file (`__FILE__`).
/// @param line  Source line (`__LINE__`).
[[noreturn]] inline void check_fail(const char* expr, const char* msg,
                                    const char* file, int line) {
  std::fprintf(stderr, "%s:%d: VC_CHECK(%s) failed: %s\n", file, line, expr,
               msg);
  std::abort();
}

}  // namespace detail
}  // namespace volumetric_kit::calib

/// @brief Abort with a diagnostic if @p cond is false.
/// @param cond  A precondition expression that must hold.
/// @param msg   A string literal describing the contract.
#define VC_CHECK(cond, msg)                                             \
  do {                                                                  \
    if (!(cond))                                                        \
      ::volumetric_kit::calib::detail::check_fail(#cond, msg, __FILE__, \
                                                  __LINE__);            \
  } while (0)
