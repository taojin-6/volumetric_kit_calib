// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file impl/result.hpp
/// @brief Out-of-line template definitions for
///        @ref volumetric_kit::calib::Result. Included at the end of result.hpp
///        -- not a standalone header; do not include directly.

#include <utility>

#include "volumetric_kit/calib/core/check.hpp"

namespace volumetric_kit::calib {

template <class T>
Result<T>::Result(T value) : status_(), value_(std::move(value)) {}

template <class T>
Result<T>::Result(Status err) : status_(std::move(err)) {
  VC_CHECK(!status_.ok(), "Result(Status) requires a non-OK status");
}

template <class T>
T& Result<T>::value() & {
  VC_CHECK(ok(), "Result::value() on an error Result");
  return *value_;
}

template <class T>
const T& Result<T>::value() const& {
  VC_CHECK(ok(), "Result::value() on an error Result");
  return *value_;
}

template <class T>
T&& Result<T>::value() && {
  VC_CHECK(ok(), "Result::value() on an error Result");
  return std::move(*value_);
}

template <class T>
T* Result<T>::operator->() {
  VC_CHECK(ok(), "Result::operator-> on an error Result");
  return &*value_;
}

template <class T>
const T* Result<T>::operator->() const {
  VC_CHECK(ok(), "Result::operator-> on an error Result");
  return &*value_;
}

template <class T>
T& Result<T>::operator*() & {
  return value();
}

template <class T>
const T& Result<T>::operator*() const& {
  return value();
}

template <class T>
T&& Result<T>::operator*() && {
  return std::move(*this).value();
}

}  // namespace volumetric_kit::calib
