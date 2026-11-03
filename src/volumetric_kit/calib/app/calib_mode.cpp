// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/calib_mode.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "size_text.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/quality/results.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = sensor::camera;
using Operation = core::config::Operation;

bool same_size(camera::ImageSize a, camera::ImageSize b) noexcept {
  return a.width == b.width && a.height == b.height;
}

// @p file's colour lens for @p id, at any size.
const camera::CameraCalibration* file_color(
    const camera::ArrayCalibration* file, const std::string& id) {
  if (file == nullptr) return nullptr;
  const camera::SensorCalibration* entry = camera::find_sensor(*file, id);
  return entry != nullptr && entry->color ? &*entry->color : nullptr;
}

// @p file's colour lens for @p id, when it is at exactly @p size.
const camera::CameraCalibration* file_lens(const camera::ArrayCalibration* file,
                                           const std::string& id,
                                           camera::ImageSize size) {
  const camera::CameraCalibration* color = file_color(file, id);
  return color != nullptr && same_size(color->model.size, size) ? color
                                                                : nullptr;
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
    // At any size: the runtime reads the factory lens whatever the file's.
    const camera::CameraCalibration* color = file_color(file, id);
    if (poses && factory && color != nullptr &&
        color->source == camera::IntrinsicsSource::Calibrated) {
      return core::Status::invalid_argument(
          id + " has a factory lens and a calibrated one in " + file_path +
          ": the runtime reads the factory lens, so take the file's out");
    }
    const camera::CameraCalibration* filed = file_lens(file, id, size);
    if (filed != nullptr)
      return RigLens{filed->model, capture::LensOrigin::File};
    if (factory) return RigLens{*factory, capture::LensOrigin::Factory};
    return RigLens{stand_in_lens(size), capture::LensOrigin::StandIn};
  });
}

core::Result<camera::CameraModel> take_lens(
    capture::SessionMode mode, const camera::ArrayCalibration* file,
    const std::string& path, capture::SessionCamera& camera) {
  return compute::guarded(
      "take_lens", [&]() -> core::Result<camera::CameraModel> {
        VKC_ASSIGN(const RigLens lens, rig_lens(mode, camera.id, camera.size,
                                                camera.color, file, path));
        std::string lens_file =
            lens.origin == capture::LensOrigin::File ? path : std::string();
        camera.lens = lens.origin;
        camera.lens_file = std::move(lens_file);
        return lens.model;
      });
}

const char* to_string(LensStatus status) noexcept {
  switch (status) {
    case LensStatus::Calibrated:
      return "calibrated";
    case LensStatus::Factory:
      return "factory";
    case LensStatus::StandIn:
      return "stand-in";
    case LensStatus::Missing:
      return "missing";
  }
  return "unknown";
}

LensStatus lens_status(capture::SessionMode mode,
                       const capture::SessionCamera& camera,
                       const camera::ArrayCalibration* file) {
  if (camera.lens == capture::LensOrigin::StandIn) {
    return mode == capture::SessionMode::Intrinsics ? LensStatus::StandIn
                                                    : LensStatus::Missing;
  }
  return lens_source(camera, file) == camera::IntrinsicsSource::Calibrated
             ? LensStatus::Calibrated
             : LensStatus::Factory;
}

core::Result<std::vector<LensStatus>> lens_statuses(
    capture::SessionMode mode,
    const std::vector<capture::SessionCamera>& cameras,
    const camera::ArrayCalibration* file) {
  return compute::guarded(
      "lens_statuses", [&]() -> core::Result<std::vector<LensStatus>> {
        std::vector<LensStatus> statuses;
        for (const capture::SessionCamera& camera : cameras) {
          statuses.push_back(lens_status(mode, camera, file));
        }
        return statuses;
      });
}

camera::IntrinsicsSource lens_source(const capture::SessionCamera& camera,
                                     const camera::ArrayCalibration* file) {
  const camera::CameraCalibration* filed =
      camera.lens == capture::LensOrigin::File
          ? file_lens(file, camera.id, camera.size)
          : nullptr;
  return filed != nullptr ? filed->source : camera::IntrinsicsSource::Factory;
}

core::Result<std::string> lens_line(capture::SessionMode mode,
                                    const capture::SessionCamera& camera,
                                    const camera::ArrayCalibration* file) {
  return compute::guarded("lens_line", [&]() -> core::Result<std::string> {
    if (mode == capture::SessionMode::Intrinsics) return std::string();
    std::string line = to_string(lens_status(mode, camera, file));
    if (camera.lens == capture::LensOrigin::File) line += " (file)";
    return line;
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
                   const std::vector<quality::LensReadiness>& lenses,
                   const std::vector<LensStatus>& statuses) noexcept {
  if (mode == capture::SessionMode::Intrinsics) {
    return std::any_of(
        lenses.begin(), lenses.end(),
        [](const quality::LensReadiness& lens) { return lens.ready; });
  }
  const bool real_lenses =
      std::none_of(statuses.begin(), statuses.end(), [](LensStatus status) {
        return status == LensStatus::Missing || status == LensStatus::StandIn;
      });
  return real_lenses && summary && summary->ready;
}

core::Result<std::string> capture_refusal(
    capture::SessionMode mode,
    const std::vector<capture::SessionCamera>& cameras,
    const std::vector<LensStatus>& statuses) {
  return compute::guarded(
      "capture_refusal", [&]() -> core::Result<std::string> {
        if (statuses.size() != cameras.size()) {
          return core::Status::invalid_argument(
              "capture_refusal: " + std::to_string(cameras.size()) +
              " cameras, " + std::to_string(statuses.size()) +
              " lens statuses");
        }
        if (mode != capture::SessionMode::Extrinsics) return std::string();
        std::vector<std::string> missing;
        for (std::size_t c = 0; c < cameras.size(); ++c) {
          if (statuses[c] == LensStatus::Missing)
            missing.push_back(cameras[c].id);
        }
        if (missing.empty()) return std::string();
        std::string ids;
        for (std::size_t i = 0; i < missing.size(); ++i) {
          if (i > 0) ids += i + 1 == missing.size() ? " and " : ", ";
          ids += missing[i];
        }
        return ids + (missing.size() == 1 ? " has" : " have") +
               " no lens: load a lens file in Setup first";
      });
}

core::Result<std::vector<NotReady>> calibrate_blockers(
    capture::SessionMode mode,
    const std::vector<capture::SessionCamera>& cameras,
    const std::vector<LensStatus>& statuses, const std::string& path,
    const std::optional<quality::SessionSummary>& summary,
    const std::vector<quality::LensReadiness>& lenses,
    const quality::SessionOptions& options) {
  return compute::guarded(
      "calibrate_blockers", [&]() -> core::Result<std::vector<NotReady>> {
        const std::size_t n = cameras.size();
        if (statuses.size() != n || (!lenses.empty() && lenses.size() != n) ||
            (summary && summary->cameras.size() != n)) {
          return core::Status::invalid_argument(
              "calibrate_blockers: " + std::to_string(n) + " cameras, " +
              std::to_string(statuses.size()) + " lens statuses, " +
              std::to_string(lenses.size()) + " readinesses");
        }
        std::vector<NotReady> out;
        const NotReady none_yet{std::string(), "no captures yet"};
        if (mode == capture::SessionMode::Intrinsics) {
          if (session_ready(mode, summary, lenses, statuses)) return out;
          if (lenses.empty()) return std::vector<NotReady>{none_yet};
          for (std::size_t c = 0; c < n; ++c) {
            out.push_back({cameras[c].id, lenses[c].unmet});
          }
          return out;
        }
        for (std::size_t c = 0; c < n; ++c) {
          if (statuses[c] != LensStatus::Missing &&
              statuses[c] != LensStatus::StandIn) {
            continue;
          }
          out.push_back(
              {cameras[c].id,
               "no lens at " + size_text(cameras[c].size) +
                   (path.empty() ? std::string(" (no lens file)")
                                 : " in " + path) +
                   ": load a lens file in Setup, or fit it in Lenses mode"});
        }
        if (!summary) {
          out.push_back(none_yet);
          return out;
        }
        if (summary->ready) return out;
        if (n < 2) {
          out.push_back({std::string(), "a pose session needs two cameras"});
          return out;
        }
        const int need = options.ready_from;
        for (std::size_t c = 0; c < n; ++c) {
          const quality::CameraSummary& camera = summary->cameras[c];
          if (camera.linked && !camera.link) continue;  // the reference
          if (!camera.link) {
            out.push_back({cameras[c].id,
                           "no captures it shares with another camera: "
                           "capture boards it and another see"});
            continue;
          }
          const std::size_t parent = camera.link->parent;
          if (camera.link->shared >= need || parent >= n) continue;
          out.push_back(
              {cameras[c].id, "shares " + std::to_string(camera.link->shared) +
                                  " captures with " + cameras[parent].id +
                                  ", needs " + std::to_string(need) +
                                  ": capture boards both see"});
        }
        if (out.empty()) {
          out.push_back(
              {std::string(), "the Session panel says what is short"});
        }
        return out;
      });
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
        setup.pairs = scenario.pairs;
        setup.pair_rows = scenario.pair_rows;
        live.rule = capture_rule(scenario.mode);
        window.intrinsic = scenario.mode == capture::SessionMode::Intrinsics;
        return editor;
      });
}

}  // namespace volumetric_kit::calib::app
