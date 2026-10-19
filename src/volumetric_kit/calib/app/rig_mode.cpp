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
      return {RigAction::Calibrate, {}};
    case KeyRequest::SwitchView:
      if (scene) return {RigAction::ShowImages, {}};
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
                             std::size_t pending) noexcept {
  if (mode.calibrating) return {false, "already running"};
  if (pending > 0) return {false, "a capture is still being written"};
  if (!ready) {
    return {false, "the session is not ready (see the Session panel)"};
  }
  return {true, {}};
}

viz::ScenePlacement scene_placement(const RigMode& mode) noexcept {
  return mode.view == RigView::Scene ? viz::ScenePlacement::FullWindow
                                     : viz::ScenePlacement::Hidden;
}

RigMode on_result(RigMode mode) noexcept {
  mode.view = RigView::Scene;
  mode.calibrated = true;
  return mode;
}

bool operator==(const RigOrigin& a, const RigOrigin& b) noexcept {
  return a.reference == b.reference;
}

bool operator!=(const RigOrigin& a, const RigOrigin& b) noexcept {
  return !(a == b);
}

bool frame_on_result(const std::optional<RigOrigin>& framed,
                     const RigOrigin& origin) noexcept {
  return !framed || *framed != origin;
}

}  // namespace volumetric_kit::calib::app
