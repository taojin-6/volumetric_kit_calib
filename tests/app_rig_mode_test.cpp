// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's window mode: what every key does in the camera images and the
// 3D view, calibrated or not and calibrating or not, a capture or a drop in
// the 3D view only going back to the images; when Calibrate may run, and why
// not; where the 3D view sits, and so whether its worker prepares; the step
// at a result; and when a result's view is framed afresh. Host-only.

#include "volumetric_kit/calib/app/rig_mode.hpp"

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "volumetric_kit/calib/app/live_view.hpp"

namespace volumetric_kit::calib::app {
namespace {

using viz::ScenePlacement;

std::string name(const RigMode& mode) {
  return std::string(mode.view == RigView::Scene ? "scene" : "images") +
         (mode.calibrated ? ", calibrated" : "") +
         (mode.calibrating ? ", calibrating" : "");
}

// Every mode: both views, calibrated or not, calibrating or not.
std::vector<RigMode> every_mode() {
  std::vector<RigMode> modes;
  for (const RigView view : {RigView::Images, RigView::Scene}) {
    for (const bool calibrated : {false, true}) {
      for (const bool calibrating : {false, true}) {
        modes.push_back({view, calibrated, calibrating});
      }
    }
  }
  return modes;
}

// The table rig_action documents, written out once more by hand.
RigStep expected(const RigMode& mode, KeyRequest request) {
  const bool scene = mode.view == RigView::Scene;
  switch (request) {
    case KeyRequest::Capture:
      return scene ? RigStep{RigAction::ShowImages, "press again to capture"}
                   : RigStep{RigAction::Arm, {}};
    case KeyRequest::DropLast:
      return scene ? RigStep{RigAction::ShowImages,
                             "press again to drop the last capture"}
                   : RigStep{RigAction::DropLast, {}};
    case KeyRequest::Calibrate:
      return {RigAction::Calibrate, {}};
    case KeyRequest::SwitchView:
      if (scene) return {RigAction::ShowImages, {}};
      return mode.calibrated ? RigStep{RigAction::ShowScene, {}}
                             : RigStep{RigAction::Ignore, "calibrate first"};
    case KeyRequest::Cancel:
      return mode.calibrating ? RigStep{RigAction::Cancel, {}}
                              : RigStep{RigAction::Ignore, {}};
    case KeyRequest::None:
      break;
  }
  return {RigAction::Ignore, {}};
}

TEST(AppRigMode, EveryKeyInEveryMode) {
  const KeyRequest requests[] = {KeyRequest::None,       KeyRequest::Capture,
                                 KeyRequest::DropLast,   KeyRequest::Calibrate,
                                 KeyRequest::SwitchView, KeyRequest::Cancel};
  const std::vector<RigMode> modes = every_mode();
  ASSERT_EQ(modes.size(), 8u);
  for (const RigMode& mode : modes) {
    for (const KeyRequest request : requests) {
      SCOPED_TRACE(name(mode) + ", request " +
                   std::to_string(static_cast<int>(request)));
      const RigStep step = rig_action(mode, request);
      const RigStep want = expected(mode, request);
      EXPECT_EQ(step.action, want.action);
      EXPECT_EQ(step.hint, want.hint);
    }
  }
}

// The 3D view never captures or drops blind, calibrating or not; the images
// capture and drop while Calibrate runs, into the next one.
TEST(AppRigMode, TheSceneNeverCapturesOrDropsAndTheImagesAlwaysDo) {
  for (const RigMode& mode : every_mode()) {
    SCOPED_TRACE(name(mode));
    const RigAction capture = rig_action(mode, KeyRequest::Capture).action;
    const RigAction drop = rig_action(mode, KeyRequest::DropLast).action;
    if (mode.view == RigView::Scene) {
      EXPECT_EQ(capture, RigAction::ShowImages);
      EXPECT_EQ(drop, RigAction::ShowImages);
    } else {
      EXPECT_EQ(capture, RigAction::Arm);
      EXPECT_EQ(drop, RigAction::DropLast);
    }
  }
}

TEST(AppRigMode, AnUnknownRequestIsIgnored) {
  const RigStep step = rig_action({}, static_cast<KeyRequest>(99));
  EXPECT_EQ(step.action, RigAction::Ignore);
  EXPECT_TRUE(step.hint.empty());
}

// Calibrate runs once the session is ready, nothing is pending and none is
// running; otherwise the first reason that holds is given, in every view.
TEST(AppRigMode, CalibrateRunsOnceReadyIdleAndNothingPending) {
  for (const RigView view : {RigView::Images, RigView::Scene}) {
    for (const bool calibrated : {false, true}) {
      const RigMode idle{view, calibrated, false};
      const RigMode running{view, calibrated, true};
      SCOPED_TRACE(name(idle));
      CalibrateGate gate = calibrate_gate(idle, true, 0);
      EXPECT_TRUE(gate.enabled);
      EXPECT_TRUE(gate.reason.empty());

      gate = calibrate_gate(running, true, 0);
      EXPECT_FALSE(gate.enabled);
      EXPECT_EQ(gate.reason, "already running");
      gate = calibrate_gate(running, false, 3);
      EXPECT_EQ(gate.reason, "already running");

      gate = calibrate_gate(idle, true, 1);
      EXPECT_FALSE(gate.enabled);
      EXPECT_EQ(gate.reason, "a capture is still being written");
      gate = calibrate_gate(idle, false, 2);
      EXPECT_EQ(gate.reason, "a capture is still being written");

      gate = calibrate_gate(idle, false, 0);
      EXPECT_FALSE(gate.enabled);
      EXPECT_EQ(gate.reason,
                "the session is not ready (see the Session panel)");
    }
  }
}

// The 3D view is hidden with the images, so its worker prepares nothing, and
// fills the window in the scene, where it prepares unless frozen.
TEST(AppRigMode, TheSceneAloneShowsAndFeedsThe3dView) {
  for (const RigMode& mode : every_mode()) {
    SCOPED_TRACE(name(mode));
    const ScenePlacement placement = scene_placement(mode);
    if (mode.view == RigView::Scene) {
      EXPECT_EQ(placement, ScenePlacement::FullWindow);
      EXPECT_TRUE(cloud_live(false, placement, false));
      EXPECT_FALSE(cloud_live(true, placement, false));
    } else {
      EXPECT_EQ(placement, ScenePlacement::Hidden);
      EXPECT_FALSE(cloud_live(false, placement, false));
    }
  }
}

// A result shows the 3D view, calibrated, from either view; whether
// Calibrate runs is the worker's to say.
TEST(AppRigMode, AResultShowsTheSceneCalibrated) {
  for (const RigMode& mode : every_mode()) {
    SCOPED_TRACE(name(mode));
    const RigMode after = on_result(mode);
    EXPECT_EQ(after.view, RigView::Scene);
    EXPECT_TRUE(after.calibrated);
    EXPECT_EQ(after.calibrating, mode.calibrating);
  }
  // Then SwitchView goes back to the images, and to the scene again.
  RigMode mode = on_result({});
  EXPECT_EQ(rig_action(mode, KeyRequest::SwitchView).action,
            RigAction::ShowImages);
  mode.view = RigView::Images;
  EXPECT_EQ(rig_action(mode, KeyRequest::SwitchView).action,
            RigAction::ShowScene);
}

// The view is framed at the first result and when the world's origin moves;
// otherwise the orbit stays.
TEST(AppRigMode, FramesAtTheFirstResultAndANewOrigin) {
  const RigOrigin reference0{0};
  const RigOrigin reference2{2};
  EXPECT_TRUE(frame_on_result(std::nullopt, reference0));
  EXPECT_TRUE(frame_on_result(std::nullopt, reference2));
  EXPECT_FALSE(frame_on_result(reference0, reference0));
  EXPECT_FALSE(frame_on_result(reference2, RigOrigin{2}));
  EXPECT_TRUE(frame_on_result(reference0, reference2));
  EXPECT_TRUE(frame_on_result(reference2, reference0));
  EXPECT_TRUE(reference0 == RigOrigin{});
  EXPECT_TRUE(reference0 != reference2);
  EXPECT_FALSE(reference0 != RigOrigin{});
}

}  // namespace
}  // namespace volumetric_kit::calib::app
