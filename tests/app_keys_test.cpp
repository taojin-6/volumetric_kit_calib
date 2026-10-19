// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's keys: a press of a capture key or a clicker's "next" captures,
// of a drop key or its "back" drops the last capture, C or F5 calibrates, V,
// B or '.' switches the view and Escape cancels; repeats, releases, other
// keys and any key while ImGui takes text ask nothing. Queued keys are
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

TEST(AppKeys, CAndF5Calibrate) {
  for (const int key : {GLFW_KEY_C, GLFW_KEY_F5}) {
    EXPECT_EQ(key_request(key, GLFW_PRESS, false), KeyRequest::Calibrate)
        << key;
  }
}

TEST(AppKeys, VBAndPeriodSwitchTheView) {
  for (const int key : {GLFW_KEY_V, GLFW_KEY_B, GLFW_KEY_PERIOD}) {
    EXPECT_EQ(key_request(key, GLFW_PRESS, false), KeyRequest::SwitchView)
        << key;
  }
}

TEST(AppKeys, EscapeCancels) {
  EXPECT_EQ(key_request(GLFW_KEY_ESCAPE, GLFW_PRESS, false),
            KeyRequest::Cancel);
}

// Every key that asks something, so each mapping below is checked on all.
const int kAsking[] = {GLFW_KEY_SPACE,   GLFW_KEY_RIGHT,     GLFW_KEY_PAGE_DOWN,
                       GLFW_KEY_S,       GLFW_KEY_BACKSPACE, GLFW_KEY_LEFT,
                       GLFW_KEY_PAGE_UP, GLFW_KEY_C,         GLFW_KEY_F5,
                       GLFW_KEY_V,       GLFW_KEY_B,         GLFW_KEY_PERIOD,
                       GLFW_KEY_ESCAPE};

TEST(AppKeys, OnlyAPressAsks) {
  for (const int key : kAsking) {
    EXPECT_EQ(key_request(key, GLFW_REPEAT, false), KeyRequest::None) << key;
    EXPECT_EQ(key_request(key, GLFW_RELEASE, false), KeyRequest::None) << key;
  }
}

TEST(AppKeys, NothingWhileImGuiTakesText) {
  for (const int key : kAsking) {
    EXPECT_EQ(key_request(key, GLFW_PRESS, true), KeyRequest::None) << key;
  }
}

TEST(AppKeys, OtherKeysAskNothing) {
  for (const int key :
       {GLFW_KEY_ENTER, GLFW_KEY_UP, GLFW_KEY_DOWN, GLFW_KEY_D, GLFW_KEY_F4,
        GLFW_KEY_F6, GLFW_KEY_COMMA, GLFW_KEY_TAB, GLFW_KEY_UNKNOWN}) {
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
  ASSERT_TRUE(keys.push(GLFW_KEY_D, GLFW_PRESS).ok());
  ASSERT_TRUE(keys.push(GLFW_KEY_F5, GLFW_PRESS).ok());
  ASSERT_TRUE(keys.push(GLFW_KEY_ESCAPE, GLFW_PRESS).ok());
  ASSERT_TRUE(keys.push(GLFW_KEY_PERIOD, GLFW_PRESS).ok());
  ASSERT_TRUE(keys.push(GLFW_KEY_LEFT, GLFW_PRESS).ok());
  taken = keys.take(false);
  ASSERT_TRUE(taken.ok());
  EXPECT_EQ(taken.value(),
            (std::vector<KeyRequest>{KeyRequest::Capture, KeyRequest::Calibrate,
                                     KeyRequest::Cancel, KeyRequest::SwitchView,
                                     KeyRequest::DropLast}));

  // Taken once: the queue is empty after.
  taken = keys.take(false);
  ASSERT_TRUE(taken.ok());
  EXPECT_TRUE(taken.value().empty());
}

}  // namespace
}  // namespace volumetric_kit::calib::app
