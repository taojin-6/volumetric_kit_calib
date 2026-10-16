// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file keys.hpp
/// @brief calib_rig's keys: a capture, and dropping the last one, from the
///        keyboard or a presentation clicker.
///
/// The window's key callback is installed before ImGui's GLFW backend, which
/// chains to it, so the keys are seen before ImGui; they are ignored while
/// ImGui takes text, so typing in a field captures nothing.

#include <cstdint>

#include "volumetric_kit/calib/app/export.hpp"

namespace volumetric_kit::calib::app {

/// @brief What a key asks of the app.
enum class KeyRequest : std::uint8_t {
  None,      ///< Nothing.
  Capture,   ///< Arm a capture.
  DropLast,  ///< Drop the last capture.
};

/// @brief What a GLFW key event asks.
///
/// A press of Space, Right, Page Down or S is a capture; a press of
/// Backspace, Left or Page Up drops the last capture (a clicker's "next" and
/// "back"). A repeat or release asks nothing, so a held key captures once.
/// @param key              The GLFW key (`GLFW_KEY_*`).
/// @param action           The GLFW action (`GLFW_PRESS`, `GLFW_REPEAT` or
///                         `GLFW_RELEASE`).
/// @param want_text_input  Whether ImGui takes text: then every key asks
///                         nothing.
/// @return The request.
///
/// @code
/// glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int, int action,
///                               int) {
///   auto* input = static_cast<Input*>(glfwGetWindowUserPointer(w));
///   input->post(key_request(key, action, input->want_text_input));
/// });
/// @endcode
VC_APP_API KeyRequest key_request(int key, int action,
                                  bool want_text_input) noexcept;

}  // namespace volumetric_kit::calib::app
