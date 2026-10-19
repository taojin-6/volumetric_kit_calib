// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/keys.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "volumetric_kit/calib/compute/guarded.hpp"

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
    case GLFW_KEY_C:
    case GLFW_KEY_F5:
      return KeyRequest::Calibrate;
    case GLFW_KEY_V:
    case GLFW_KEY_B:
    case GLFW_KEY_PERIOD:
      return KeyRequest::SwitchView;
    case GLFW_KEY_ESCAPE:
      return KeyRequest::Cancel;
    default:
      return KeyRequest::None;
  }
}

core::Status KeyQueue::push(int key, int action) {
  return compute::guarded("KeyQueue::push", [&]() -> core::Status {
    events_.emplace_back(key, action);
    return {};
  });
}

core::Result<std::vector<KeyRequest>> KeyQueue::take(bool want_text_input) {
  std::vector<std::pair<int, int>> events;
  events.swap(events_);
  return compute::guarded(
      "KeyQueue::take", [&]() -> core::Result<std::vector<KeyRequest>> {
        std::vector<KeyRequest> requests;
        for (const auto& [key, action] : events) {
          const KeyRequest request = key_request(key, action, want_text_input);
          if (request != KeyRequest::None) requests.push_back(request);
        }
        return requests;
      });
}

}  // namespace volumetric_kit::calib::app
