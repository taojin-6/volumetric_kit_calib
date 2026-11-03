// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/calib_mode.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = sensor::camera;
using Operation = core::config::Operation;

bool same_size(camera::ImageSize a, camera::ImageSize b) noexcept {
  return a.width == b.width && a.height == b.height;
}

std::string size_text(camera::ImageSize size) {
  return std::to_string(size.width) + "x" + std::to_string(size.height);
}

// @p file's colour lens for @p id, when it is at exactly @p size.
const camera::CameraCalibration* file_lens(const camera::ArrayCalibration* file,
                                           const std::string& id,
                                           camera::ImageSize size) {
  if (file == nullptr) return nullptr;
  const camera::SensorCalibration* entry = camera::find_sensor(*file, id);
  if (entry == nullptr || !entry->color) return nullptr;
  return same_size(entry->color->model.size, size) ? &*entry->color : nullptr;
}

}  // namespace

core::Result<capture::SessionMode> session_mode(Operation operation) {
  return compute::guarded(
      "session_mode", [&]() -> core::Result<capture::SessionMode> {
        if (operation == Operation::CalibrateIntrinsics) {
          return capture::SessionMode::Intrinsics;
        }
        if (operation == Operation::Calibrate) {
          return capture::SessionMode::Extrinsics;
        }
        // By its number: core's load refuses these for calib_rig, so only a
        // scenario made in code gets here, and core spells no operation.
        return core::Status::invalid_argument(
            "operation: " + std::to_string(static_cast<int>(operation)) +
            " is not one calib_rig runs (calibrate, calibrate_intrinsics)");
      });
}

Target board_seed(capture::SessionMode mode) noexcept {
  return mode == capture::SessionMode::Intrinsics ? legacy_checkerboard()
                                                  : legacy_femto_board();
}

viz::BoardRule board_rule(capture::SessionMode mode) noexcept {
  viz::BoardRule rule;
  if (mode == capture::SessionMode::Intrinsics) {
    rule.checkerboard = true;
    rule.min_corners = solve::LensOptions{}.min_corners;
  }
  return rule;
}

quality::CaptureRuleOptions capture_rule(capture::SessionMode mode) noexcept {
  quality::CaptureRuleOptions rule;
  if (mode == capture::SessionMode::Intrinsics) rule.min_views = 1;
  return rule;
}

camera::CameraModel stand_in_lens(camera::ImageSize size) noexcept {
  camera::CameraModel lens;
  lens.size = size;
  const double half_width = 0.5 * size.width;
  lens.intrinsics = {half_width, half_width, 0.5 * size.width - 0.5,
                     0.5 * size.height - 0.5};
  return lens;
}

core::Result<RigLens> rig_lens(
    capture::SessionMode mode, const std::string& id, camera::ImageSize size,
    const std::optional<camera::CameraModel>& factory,
    const camera::ArrayCalibration* file, const std::string& file_path) {
  return compute::guarded("rig_lens", [&]() -> core::Result<RigLens> {
    const bool poses = mode == capture::SessionMode::Extrinsics;
    const camera::CameraCalibration* filed = file_lens(file, id, size);
    if (poses && factory && filed != nullptr &&
        filed->source == camera::IntrinsicsSource::Calibrated) {
      return core::Status::invalid_argument(
          id + " has a factory lens and a calibrated one in " + file_path +
          ": the runtime reads the factory lens, so take the file's out");
    }
    if (filed != nullptr)
      return RigLens{filed->model, capture::LensOrigin::File};
    if (factory) return RigLens{*factory, capture::LensOrigin::Factory};
    if (poses) {
      return core::Status::invalid_argument(
          id + " has no lens at " + size_text(size) +
          (file != nullptr ? " in " + file_path : std::string(" (no file)")) +
          ": run intrinsic mode");
    }
    return RigLens{stand_in_lens(size), capture::LensOrigin::StandIn};
  });
}

std::optional<camera::CameraModel> lens_start(
    const std::string& id, camera::ImageSize size,
    const std::optional<camera::CameraModel>& factory,
    const camera::ArrayCalibration* file) {
  if (const camera::CameraCalibration* own = file_lens(file, id, size)) {
    return own->model;
  }
  if (file != nullptr) {
    for (const camera::SensorCalibration& other : file->sensors) {
      if (other.color && same_size(other.color->model.size, size)) {
        return other.color->model;
      }
    }
  }
  if (factory && same_size(factory->size, size)) return factory;
  return std::nullopt;
}

core::Result<std::vector<quality::LensReadiness>> lens_states(
    const KeptCaptures& kept) {
  return compute::guarded(
      "lens_states",
      [&]() -> core::Result<std::vector<quality::LensReadiness>> {
        const std::size_t n = kept.sensors.size();
        VKC_ASSIGN(const compute::Board board,
                   compute::Board::create(kept.board));
        std::vector<Observation> views;
        std::vector<std::vector<std::int64_t>> frontal(n);
        for (const std::shared_ptr<const KeptCapture>& capture :
             kept.captures) {
          if (capture->views.size() != n || capture->frontal.size() != n) {
            return core::Status::invalid_argument(
                "lens_states: capture " + std::to_string(capture->index) +
                " is not one view per camera");
          }
          for (std::size_t c = 0; c < n; ++c) {
            const Observation& view = capture->views[c].observation;
            if (!view.empty()) views.push_back(view);
            if (capture->frontal[c]) frontal[c].push_back(capture->index);
          }
        }
        std::vector<quality::LensReadiness> states;
        states.reserve(n);
        for (std::size_t c = 0; c < n; ++c) {
          const capture::SessionCamera& camera = kept.sensors[c];
          // A stand-in's tilt is wrong at a fisheye's edge.
          const bool judged = camera.lens != capture::LensOrigin::StandIn;
          VKC_ASSIGN(
              quality::LensReadiness state,
              quality::lens_readiness(camera.id, camera.size, board, views,
                                      judged ? &frontal[c] : nullptr));
          states.push_back(std::move(state));
        }
        return states;
      });
}

bool session_ready(capture::SessionMode mode,
                   const std::optional<quality::SessionSummary>& summary,
                   const std::vector<quality::LensReadiness>& lenses) noexcept {
  if (mode == capture::SessionMode::Intrinsics) {
    return std::any_of(
        lenses.begin(), lenses.end(),
        [](const quality::LensReadiness& lens) { return lens.ready; });
  }
  return summary && summary->ready;
}

core::Result<viz::BoardEditor> apply_mode(const RigScenario& scenario,
                                          CaptureSetup& setup,
                                          LiveOptions& live, RigMode& window) {
  return compute::guarded(
      "apply_mode", [&]() -> core::Result<viz::BoardEditor> {
        VKC_ASSIGN(viz::BoardEditor editor,
                   viz::BoardEditor::create(board_seed(scenario.mode), false,
                                            board_rule(scenario.mode)));
        setup.mode = scenario.mode;
        setup.calibration = scenario.calibration;
        live.rule = capture_rule(scenario.mode);
        window.intrinsic = scenario.mode == capture::SessionMode::Intrinsics;
        return editor;
      });
}

}  // namespace volumetric_kit::calib::app
