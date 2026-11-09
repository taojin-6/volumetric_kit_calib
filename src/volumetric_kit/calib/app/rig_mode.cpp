// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/rig_mode.hpp"

namespace volumetric_kit::calib::app {

RigStep rig_action(const RigMode& mode, KeyRequest request) noexcept {
  const bool scene = mode.view == RigView::Scene;
  switch (request) {
    case KeyRequest::Capture:
      if (scene) return {RigAction::ShowImages, "press again to capture"};
      return {RigAction::Arm, {}};
    case KeyRequest::DropLast:
      if (scene) {
        return {RigAction::ShowImages, "press again to drop the last capture"};
      }
      return {RigAction::DropLast, {}};
    case KeyRequest::Calibrate:
      if (mode.scan) return {RigAction::Ignore, "Scan mode has no Calibrate"};
      return {RigAction::Calibrate, {}};
    case KeyRequest::SwitchView:
      if (scene) return {RigAction::ShowImages, {}};
      if (mode.scan) return {RigAction::Ignore, "Scan mode has no 3D view"};
      if (mode.intrinsic) {
        return {RigAction::Ignore, "Lenses mode has no 3D view"};
      }
      if (mode.calibrated) return {RigAction::ShowScene, {}};
      return {RigAction::Ignore, "calibrate first"};
    case KeyRequest::Cancel:
      if (mode.calibrating) return {RigAction::Cancel, {}};
      return {RigAction::Ignore, {}};
    case KeyRequest::None:
      break;
  }
  return {RigAction::Ignore, {}};
}

CalibrateGate calibrate_gate(const RigMode& mode, bool ready,
                             std::size_t unresolved) noexcept {
  if (mode.scan) return {false, "Scan mode has no Calibrate"};
  if (mode.calibrating) return {false, "already running"};
  if (unresolved > 0) {
    return {false, "a capture, a drop or a session switch is under way"};
  }
  if (!ready) {
    return {false, mode.intrinsic
                       ? "no camera is ready: press C to see why"
                       : "the session is not ready: press C to see why"};
  }
  return {true, {}};
}

bool search_paused(const RigMode& mode) noexcept { return mode.calibrating; }

viz::ScenePlacement scene_placement(const RigMode& mode) noexcept {
  return mode.view == RigView::Scene ? viz::ScenePlacement::FullWindow
                                     : viz::ScenePlacement::Hidden;
}

RigMode on_result(RigMode mode) noexcept {
  if (!mode.intrinsic) mode.view = RigView::Scene;
  mode.calibrated = true;
  return mode;
}

bool operator==(const RigOrigin& a, const RigOrigin& b) noexcept {
  return a.tag == b.tag && (a.tag || a.reference == b.reference);
}

bool operator!=(const RigOrigin& a, const RigOrigin& b) noexcept {
  return !(a == b);
}

bool frame_on_result(const std::optional<RigOrigin>& framed,
                     const RigOrigin& origin) noexcept {
  return !framed || *framed != origin;
}

std::optional<sensor::camera::Vec3d> world_up(
    const RigOrigin& origin) noexcept {
  if (!origin.tag) return std::nullopt;
  return sensor::camera::Vec3d(0.0, 0.0, 1.0);  // out of the tag's face
}

}  // namespace volumetric_kit::calib::app
