// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/keys.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace volumetric_kit::calib::app {

KeyRequest key_request(int key, int action, bool want_text_input) noexcept {
  if (want_text_input || action != GLFW_PRESS) return KeyRequest::None;
  switch (key) {
    case GLFW_KEY_SPACE:
    case GLFW_KEY_RIGHT:
    case GLFW_KEY_PAGE_DOWN:
    case GLFW_KEY_S:
      return KeyRequest::Capture;
    case GLFW_KEY_BACKSPACE:
    case GLFW_KEY_LEFT:
    case GLFW_KEY_PAGE_UP:
      return KeyRequest::DropLast;
    default:
      return KeyRequest::None;
  }
}

}  // namespace volumetric_kit::calib::app
