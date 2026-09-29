// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file result.hpp
/// @brief Exception-free error handling for the public API.
///
/// No exceptions cross the library boundary: mobile (iOS/Android) consumers
/// frequently build with `-fno-exceptions`, where a throwing API is unusable.
/// Fallible calls therefore report failure by value:
///
/// - @ref Status    -- success, or an error domain (@ref Status::Code) with a
///                     context message.
/// - @ref Result    -- a `T` on success, or a `Status` on failure.
///
/// Two macros remove the check-and-propagate boilerplate: @ref VC_TRY (for a
/// `Status` expression) and @ref VC_ASSIGN (for a `Result<T>`). Both
/// early-return on failure, so they appear only inside functions that
/// themselves return `Status` or `Result<T>`.
///
/// Both types are `[[nodiscard]]`: dropping a returned `Status` or `Result` on
/// the floor is a compiler warning (an error under `-Werror`), because with no
/// exceptions an ignored return value is a silently lost failure.
///
/// Misuse -- reading the value of an error `Result` -- is a programmer error,
/// not a runtime one: it fails fast via `VC_CHECK` (see check.hpp) rather than
/// throwing.
///
/// @code
/// Result<PinholeBrownConrady> r = PinholeBrownConrady::create(K, dist);
/// if (!r) return r.status();             // propagate failure to our caller
/// const PinholeBrownConrady& cam = *r;   // safe: guarded by the !r check
/// @endcode

#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "volumetric_kit/calib/core/check.hpp"
#include "volumetric_kit/calib/core/export.hpp"

namespace volumetric_kit::calib {

/// @brief Success, or an error: a domain (@ref Code) and a human-readable
///        message.
///
/// A default-constructed `Status` is success. Build a failure with a domain
/// factory (@ref invalid_argument, @ref not_found, @ref unsupported,
/// @ref out_of_memory, @ref io_error, @ref numerical). Convertible to `bool`
/// (true == success) for terse checks.
///
/// @code
/// Status s = solve_intrinsics(views);
/// if (!s) {
///   log_message(std::string(to_string(s.domain())) + ": " + s.message());
///   return s;
/// }
/// @endcode
class [[nodiscard]] Status {
 public:
  /// @brief The kind of failure a non-OK `Status` reports.
  enum class Code {
    Ok,               ///< Success.
    InvalidArgument,  ///< A malformed or contradictory argument value.
    NotFound,         ///< A named resource or file does not exist.
    Unsupported,      ///< A valid request the build/platform cannot satisfy.
    OutOfMemory,      ///< A host or device allocation failed.
    IoError,          ///< A read/write/decode/encode operation failed.
    Numerical,        ///< A solve failed: singular system, non-convergence, or
                      ///< a degenerate/ill-conditioned configuration.
  };

  /// Construct a success status.
  Status() = default;

  /// @brief Build a non-OK failure status in the named domain.
  /// @param message  Human-readable context.
  /// @return A non-OK `Status` whose @ref domain is the factory's domain.
  static Status invalid_argument(std::string message) {
    return Status{Code::InvalidArgument, std::move(message)};
  }
  /// @copydoc invalid_argument
  static Status not_found(std::string message) {
    return Status{Code::NotFound, std::move(message)};
  }
  /// @copydoc invalid_argument
  static Status unsupported(std::string message) {
    return Status{Code::Unsupported, std::move(message)};
  }
  /// @copydoc invalid_argument
  static Status out_of_memory(std::string message) {
    return Status{Code::OutOfMemory, std::move(message)};
  }
  /// @copydoc invalid_argument
  static Status io_error(std::string message) {
    return Status{Code::IoError, std::move(message)};
  }
  /// @copydoc invalid_argument
  static Status numerical(std::string message) {
    return Status{Code::Numerical, std::move(message)};
  }

  /// @return `true` if this is a success status.
  bool ok() const noexcept { return domain_ == Code::Ok; }
  /// @return `true` on success (same as @ref ok).
  explicit operator bool() const noexcept { return ok(); }

  /// @return The error domain; @ref Code::Ok exactly when @ref ok.
  Code domain() const noexcept { return domain_; }
  /// @return The failure context message; empty when @ref ok.
  const std::string& message() const noexcept { return message_; }

 private:
  Status(Code domain, std::string message)
      : domain_(domain), message_(std::move(message)) {}

  Code domain_ = Code::Ok;
  std::string message_;
};

/// @brief Human-readable name for a @ref Status::Code (e.g. "InvalidArgument").
/// @param code  A domain value.
/// @return A static, never-empty `string_view`.
VC_CORE_API std::string_view to_string(Status::Code code) noexcept;

/// @brief A value of type `T` on success, or a non-OK @ref Status on failure.
/// @tparam T  The success value type (must be movable).
///
/// Constructs implicitly from anything convertible to `T` (success) or from a
/// `Status` (failure), so a function can `return value;` or
/// `return some_error;` directly. Always check @ref ok (or the `bool`
/// conversion) before reading @ref value.
///
/// @code
/// Result<double> mean_reproj_error(const Dataset& d) {
///   if (d.empty()) return Status::invalid_argument("no observations");
///   return compute_rms(d);   // implicit success
/// }
/// @endcode
template <class T>
class [[nodiscard]] Result {
 public:
  /// @brief Construct a success Result from anything convertible to `T`.
  /// @param value  The success value: a `T`, or e.g. a string literal for a
  ///               `Result<std::string>`.
  ///
  /// A template (rather than `Result(T)`) so a conversion to `T` does not use
  /// up the one user-defined conversion an implicit `return` allows. Defined
  /// here because its constraint would otherwise have to be restated
  /// out-of-line.
  template <class U = T,
            std::enable_if_t<std::is_convertible_v<U&&, T> &&
                                 !std::is_same_v<std::decay_t<U>, Result> &&
                                 !std::is_same_v<std::decay_t<U>, Status>,
                             int> = 0>
  Result(U&& value)  // NOLINT(google-explicit-constructor) — ergonomic success
                     // return
      : value_(std::in_place, std::forward<U>(value)) {}
  /// Construct a failure Result; @p err must be non-OK (checked by VC_CHECK).
  Result(Status err);  // NOLINT(google-explicit-constructor) — ergonomic error
                       // return

  /// @return `true` if this holds a value rather than an error.
  bool ok() const noexcept { return status_.ok(); }
  /// @return `true` if this holds a value (same as @ref ok).
  explicit operator bool() const noexcept { return ok(); }
  /// @return The status; non-OK exactly when this is an error Result.
  const Status& status() const noexcept { return status_; }

  /// @brief Access the held value.
  /// @pre @ref ok is true. Calling this on an error Result is a programmer
  ///      error: it aborts via `VC_CHECK` (it never throws), so guard with
  ///      @ref ok first.
  /// @return Reference to the held value.
  T& value() &;
  const T& value() const&;
  T&& value() &&;

  /// @brief Pointer/reference access to the held value.
  /// @pre @ref ok is true; otherwise aborts, as in @ref value.
  T* operator->();
  const T* operator->() const;
  T& operator*() &;
  const T& operator*() const&;
  T&& operator*() &&;

 private:
  Status status_;
  std::optional<T> value_;
};

}  // namespace volumetric_kit::calib

/// @brief Evaluate a `Status` expression and early-return it if not OK.
/// @param expr  An expression yielding a `Status`.
///
/// Usable only inside a function returning `Status` or `Result<T>` -- the early
/// `return` carries the failure outward.
///
/// @code
/// Status run() {
///   VC_TRY(detect(views));   // returns the error if this fails
///   return {};               // success
/// }
/// @endcode
#define VC_TRY(expr)                                     \
  do {                                                   \
    ::volumetric_kit::calib::Status _vc_status = (expr); \
    if (!_vc_status.ok()) return _vc_status;             \
  } while (0)

/// @brief Evaluate a `Result<T>` expression, early-return its `Status` on
///        failure, otherwise move the value into @p decl.
/// @param decl  A variable declaration (e.g. `double rms`) bound to the
///              unwrapped value on success.
/// @param expr  An expression yielding a `Result<T>`.
///
/// The `Result<T>` analogue of @ref VC_TRY. Usable only inside a function
/// returning `Status` or `Result<U>`. Because it declares @p decl in the
/// enclosing scope, it expands to a statement sequence (not a `do { } while`),
/// so it is not a single statement -- never use it as the unbraced body of an
/// `if`/`for`/`while`. The hidden temporary is keyed on `__COUNTER__` (not
/// `__LINE__`), so multiple `VC_ASSIGN`s in one scope never collide. @p decl is
/// a single macro argument, so a type written with a top-level comma needs an
/// alias first (e.g. `using Pair = std::pair<int, int>;`).
///
/// @code
/// Result<Report> analyze(const Dataset& d) {
///   VC_ASSIGN(PinholeBrownConrady cam, PinholeBrownConrady::create(K, dist));
///   return build_report(cam, d);   // `cam` holds the value here
/// }
/// @endcode
#define VC_ASSIGN(decl, expr) VC_ASSIGN_(decl, expr, __COUNTER__)
#define VC_ASSIGN_(decl, expr, id) VC_ASSIGN_IMPL_(decl, expr, id)
#define VC_ASSIGN_IMPL_(decl, expr, id)                       \
  auto _vc_result_##id = (expr);                              \
  if (!_vc_result_##id.ok()) return _vc_result_##id.status(); \
  decl = std::move(_vc_result_##id).value()

#include "volumetric_kit/calib/core/impl/result.hpp"
