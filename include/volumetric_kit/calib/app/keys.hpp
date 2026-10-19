// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file keys.hpp
/// @brief calib_rig's keys: a capture, and dropping the last one, from the
///        keyboard or a presentation clicker; Calibrate, its cancel, and the
///        switch between the camera images and the 3D view.
///
/// The window's key callback is installed before ImGui's GLFW backend, which
/// chains to it, so the keys are seen before ImGui; they are ignored while
/// ImGui takes text, so typing in a field captures nothing. ImGui knows that
/// only once its next frame has begun, so the callback queues each key
/// (@ref KeyQueue) and the frame maps them then. What a request does depends
/// on what the window shows (`rig_action`, rig_mode.hpp).

#include <cstdint>
#include <utility>
#include <vector>

#include "volumetric_kit/calib/app/export.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::app {

/// @brief What a key asks of the app.
enum class KeyRequest : std::uint8_t {
  None,        ///< Nothing.
  Capture,     ///< Arm a capture.
  DropLast,    ///< Drop the last capture.
  Calibrate,   ///< Run Calibrate.
  SwitchView,  ///< Switch between the camera images and the 3D view.
  Cancel,      ///< Cancel a running Calibrate.
};

/// @brief What a GLFW key event asks.
///
/// A press of Space, Right, Page Down or S is a capture; a press of
/// Backspace, Left or Page Up drops the last capture (a clicker's "next" and
/// "back"); C or F5 is Calibrate; V, B or `.` switches the view; Escape
/// cancels. A repeat or release asks nothing, so a held key captures once.
/// Modifiers are not taken, so Shift+F5 is F5.
/// @param key              The GLFW key (`GLFW_KEY_*`).
/// @param action           The GLFW action (`GLFW_PRESS`, `GLFW_REPEAT` or
///                         `GLFW_RELEASE`).
/// @param want_text_input  Whether ImGui takes text: then every key asks
///                         nothing.
/// @return The request.
///
/// @code
/// if (key_request(GLFW_KEY_SPACE, GLFW_PRESS, io.WantTextInput) ==
///     KeyRequest::Capture) {
///   live->arm();
/// }
/// @endcode
VC_APP_API KeyRequest key_request(int key, int action,
                                  bool want_text_input) noexcept;

/// @brief GLFW key events as they come, mapped to requests (@ref key_request)
///        once the window's frame knows whether ImGui takes text.
///
/// ImGui says whether a field takes text only after its frame begins:
/// `ImGuiIO::WantTextInput` then covers a field activated the frame before.
/// Keys polled before the frame are queued, and mapped with that frame's
/// flag, so a key typed just after a click into a field is ignored.
///
/// @code
/// // GLFW's key callback, as glfwPollEvents calls it:
/// (void)queue.push(key, action);
/// // After the frame's ImGui::NewFrame():
/// VKC_ASSIGN(const std::vector<KeyRequest> requests,
///            queue.take(io.WantTextInput));
/// @endcode
class VC_APP_API KeyQueue {
 public:
  /// @brief Queue a key event.
  /// @param key     The GLFW key (`GLFW_KEY_*`).
  /// @param action  The GLFW action.
  /// @return OK; or `OutOfMemory`, the event not queued.
  core::Status push(int key, int action);

  /// @brief Map every key queued since the last call, emptying the queue.
  /// @param want_text_input  Whether ImGui takes text in this frame.
  /// @return Each key's request that is not `None`, in the order pressed; or
  ///         `OutOfMemory`, the queue emptied all the same.
  core::Result<std::vector<KeyRequest>> take(bool want_text_input);

 private:
  std::vector<std::pair<int, int>> events_;  // key, action
};

}  // namespace volumetric_kit::calib::app
