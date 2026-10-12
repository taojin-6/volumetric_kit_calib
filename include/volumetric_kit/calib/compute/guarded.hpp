// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file guarded.hpp
/// @brief Running OpenCV, or anything else that throws, behind a `Status`.
///
/// OpenCV reports a failure by throwing `cv::Exception`, and calib's API
/// throws nothing (AGENTS.md, "Error handling"). Each public entry point that
/// calls OpenCV runs its body once through @ref guarded, which returns what
/// the body threw as a `Status`.

#include <string_view>
#include <type_traits>
#include <utility>

#include "volumetric_kit/calib/compute/export.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::compute {

/// @brief The exception being handled, as a `Status`.
///
/// A `cv::Exception` maps by its code:
/// - `StsNoMem` is `OutOfMemory`;
/// - `StsBadArg`, `StsBadSize`, `StsOutOfRange`, `StsUnmatchedSizes` and
///   `StsBadFlag` are `InvalidArgument`;
/// - any other code is a `Backend::Other` failure whose detail is the code,
///   or `StsError` for a code of 0, which a backend failure cannot carry.
///
/// Anything else is the core's `exception_status(what, InvalidArgument)`: a
/// `std::bad_alloc` is `OutOfMemory`.
/// @param what  What was called; it prefixes the message.
/// @pre Called inside a `catch` handler; with no exception being handled the
///      program terminates.
/// @return A non-OK `Status` whose message is `"<what>: "` and the
///         exception's `what()`, or `unknown exception` for one that is not a
///         `std::exception`.
///
/// @code
/// try {
///   run_opencv();
/// } catch (...) {
///   return compute::current_exception_status("detect");
/// }
/// @endcode
VC_COMPUTE_API core::Status current_exception_status(
    std::string_view what) noexcept;

namespace detail {

template <class T>
struct is_status_or_result : std::false_type {};
template <>
struct is_status_or_result<core::Status> : std::true_type {};
template <class T>
struct is_status_or_result<core::Result<T>> : std::true_type {};

}  // namespace detail

/// @brief Run @p fn, returning what it returns, or what it throws as a
///        `Status` (@ref current_exception_status).
/// @tparam Fn  A callable taking no arguments and returning `core::Status` or
///             `core::Result<T>`.
/// @param what  What is being run, e.g. the public function's name; it
///              prefixes the message of a thrown failure.
/// @param fn    The body.
/// @return @p fn's result; or, when it throws, the failure.
///
/// @code
/// core::Result<double> rms(const cv::Mat& residuals) {
///   return compute::guarded("rms", [&]() -> core::Result<double> {
///     return cv::norm(residuals) / std::sqrt(double(residuals.total()));
///   });
/// }
/// @endcode
template <class Fn>
std::invoke_result_t<Fn&&> guarded(std::string_view what, Fn&& fn) noexcept {
  static_assert(detail::is_status_or_result<std::invoke_result_t<Fn&&>>::value,
                "guarded() runs a body returning core::Status or "
                "core::Result<T>");
  try {
    return std::forward<Fn>(fn)();
  } catch (...) {
    return current_exception_status(what);
  }
}

}  // namespace volumetric_kit::calib::compute
