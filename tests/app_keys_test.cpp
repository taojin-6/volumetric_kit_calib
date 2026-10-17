// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's keys: a press of a capture key or a clicker's "next" captures,
// of a drop key or its "back" drops the last capture; repeats, releases,
// other keys and any key while ImGui takes text ask nothing. Queued keys are
// mapped with the flag of the frame that takes them. Host-only.

#include "volumetric_kit/calib/app/keys.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <gtest/gtest.h>

#include <vector>

namespace volumetric_kit::calib::app {
namespace {

TEST(AppKeys, CaptureKeysCapture) {
  for (const int key :
       {GLFW_KEY_SPACE, GLFW_KEY_RIGHT, GLFW_KEY_PAGE_DOWN, GLFW_KEY_S}) {
    EXPECT_EQ(key_request(key, GLFW_PRESS, false), KeyRequest::Capture) << key;
  }
}

TEST(AppKeys, BackKeysDropTheLastCapture) {
  for (const int key : {GLFW_KEY_BACKSPACE, GLFW_KEY_LEFT, GLFW_KEY_PAGE_UP}) {
    EXPECT_EQ(key_request(key, GLFW_PRESS, false), KeyRequest::DropLast) << key;
  }
}

TEST(AppKeys, OnlyAPressAsks) {
  for (const int key : {GLFW_KEY_SPACE, GLFW_KEY_BACKSPACE}) {
    EXPECT_EQ(key_request(key, GLFW_REPEAT, false), KeyRequest::None) << key;
    EXPECT_EQ(key_request(key, GLFW_RELEASE, false), KeyRequest::None) << key;
  }
}

TEST(AppKeys, NothingWhileImGuiTakesText) {
  for (const int key :
       {GLFW_KEY_SPACE, GLFW_KEY_RIGHT, GLFW_KEY_PAGE_DOWN, GLFW_KEY_S,
        GLFW_KEY_BACKSPACE, GLFW_KEY_LEFT, GLFW_KEY_PAGE_UP}) {
    EXPECT_EQ(key_request(key, GLFW_PRESS, true), KeyRequest::None) << key;
  }
}

TEST(AppKeys, OtherKeysAskNothing) {
  for (const int key :
       {GLFW_KEY_ENTER, GLFW_KEY_ESCAPE, GLFW_KEY_UP, GLFW_KEY_DOWN, GLFW_KEY_C,
        GLFW_KEY_D, GLFW_KEY_TAB, GLFW_KEY_UNKNOWN}) {
    EXPECT_EQ(key_request(key, GLFW_PRESS, false), KeyRequest::None) << key;
  }
}

// A key queued before ImGui's frame is mapped with that frame's flag: typed
// just after a click into a field, it asks nothing.
TEST(AppKeys, AQueuedKeyTakesTheFlagOfTheFrameThatMapsIt) {
  KeyQueue keys;
  ASSERT_TRUE(keys.push(GLFW_KEY_S, GLFW_PRESS).ok());
  core::Result<std::vector<KeyRequest>> taken = keys.take(true);
  ASSERT_TRUE(taken.ok());
  EXPECT_TRUE(taken.value().empty());

  ASSERT_TRUE(keys.push(GLFW_KEY_SPACE, GLFW_PRESS).ok());
  ASSERT_TRUE(keys.push(GLFW_KEY_SPACE, GLFW_RELEASE).ok());
  ASSERT_TRUE(keys.push(GLFW_KEY_C, GLFW_PRESS).ok());
  ASSERT_TRUE(keys.push(GLFW_KEY_LEFT, GLFW_PRESS).ok());
  taken = keys.take(false);
  ASSERT_TRUE(taken.ok());
  EXPECT_EQ(taken.value(), (std::vector<KeyRequest>{KeyRequest::Capture,
                                                    KeyRequest::DropLast}));

  // Taken once: the queue is empty after.
  taken = keys.take(false);
  ASSERT_TRUE(taken.ok());
  EXPECT_TRUE(taken.value().empty());
}

}  // namespace
}  // namespace volumetric_kit::calib::app
