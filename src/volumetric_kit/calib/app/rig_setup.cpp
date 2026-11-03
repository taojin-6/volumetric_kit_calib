// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/rig_setup.hpp"

#include <algorithm>
#include <filesystem>
#include <set>
#include <system_error>
#include <tuple>

#include <nlohmann/json.hpp>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/quality/results.hpp"
#include "volumetric_kit/core/base/file_io.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = sensor::camera;
namespace fs = std::filesystem;

std::string size_text(camera::ImageSize size) {
  return std::to_string(size.width) + "x" + std::to_string(size.height);
}

// `items` as "a, b and c"; `last` joins the last two.
std::string listed(const std::vector<std::string>& items, const char* last) {
  std::string text;
  for (std::size_t i = 0; i < items.size(); ++i) {
    if (i > 0) text += i + 1 == items.size() ? last : ", ";
    text += items[i];
  }
  return text;
}

// What a lens run's report says of each camera it fitted and wrote, in its
// order; empty when `report` is not a lens run's.
std::optional<std::vector<std::pair<std::string, bool>>> lens_report(
    const std::string& report) {
  const nlohmann::ordered_json json =
      nlohmann::ordered_json::parse(report, nullptr, false);
  if (json.is_discarded() || !json.is_object() ||
      json.value("format", std::string()) != "volumetric_kit_calib.report") {
    return std::nullopt;
  }
  const auto lenses = json.find("lenses");
  if (lenses == json.end() || !lenses->is_object()) return std::nullopt;
  const auto cameras = lenses->find("cameras");
  if (cameras == lenses->end() || !cameras->is_object()) return std::nullopt;
  std::vector<std::pair<std::string, bool>> out;
  for (auto it = cameras->begin(); it != cameras->end(); ++it) {
    if (!it.value().is_object() || !it.value().value("written", false)) {
      continue;
    }
    out.emplace_back(it.key(),
                     it.value().value("verdict", std::string()) == "pass");
  }
  return out;
}

}  // namespace

core::Result<std::vector<LensResult>> find_lens_results(
    const std::string& root) {
  return compute::guarded(
      "find_lens_results", [&]() -> core::Result<std::vector<LensResult>> {
        std::error_code ec;
        if (!fs::is_directory(root, ec)) {
          return core::Status::not_found("find_lens_results: " + root +
                                         " is not a directory");
        }
        std::vector<LensResult> found;
        for (fs::directory_iterator session(root, ec), end;
             !ec && session != end; session.increment(ec)) {
          const fs::path results = session->path() / kResultsDirectory;
          std::error_code inner;
          if (!fs::is_directory(results, inner)) continue;
          for (fs::directory_iterator run(results, inner); !inner && run != end;
               run.increment(inner)) {
            const fs::path file = run->path() / quality::kArrayCalibrationFile;
            const fs::path report = run->path() / quality::kReportFile;
            std::error_code exists;
            if (!fs::is_regular_file(file, exists) ||
                !fs::is_regular_file(report, exists)) {
              continue;
            }
            const core::Result<std::string> text =
                core::read_file(report.string());
            if (!text.ok()) continue;
            std::optional<std::vector<std::pair<std::string, bool>>> cameras =
                lens_report(*text);
            if (!cameras) continue;
            found.push_back({file.string(), session->path().filename().string(),
                             run->path().filename().string(),
                             std::move(*cameras)});
          }
        }
        // Newest first: the stamp after `RRR_`, then the session, then RRR.
        const auto key = [](const LensResult& r) {
          const std::string stamp =
              r.results.size() > 4 ? r.results.substr(4) : r.results;
          return std::make_tuple(stamp, r.session, r.results);
        };
        std::sort(found.begin(), found.end(),
                  [&](const LensResult& a, const LensResult& b) {
                    return key(a) > key(b);
                  });
        return found;
      });
}

core::Result<std::string> lens_result_label(const LensResult& result) {
  return compute::guarded(
      "lens_result_label", [&]() -> core::Result<std::string> {
        // `RRR_YYYY-MM-DD_HHMMSS` as "YYYY-MM-DD HH:MM:SS".
        const std::string& name = result.results;
        std::string when = name;
        if (name.size() == 21 && name[3] == '_' && name[14] == '_') {
          when = name.substr(4, 10) + " " + name.substr(15, 2) + ":" +
                 name.substr(17, 2) + ":" + name.substr(19, 2);
        }
        std::vector<std::string> verdicts;
        for (const auto& [id, pass] : result.cameras) {
          verdicts.push_back(id + (pass ? " pass" : " fail"));
        }
        return when + "  " +
               (verdicts.empty() ? std::string("no lens written")
                                 : listed(verdicts, ", ")) +
               "  (" + result.session + "/" + name.substr(0, 3) + ")";
      });
}

core::Result<LensFile> load_lens_file(
    const std::string& path,
    const std::vector<capture::SessionCamera>& cameras) {
  return compute::guarded("load_lens_file", [&]() -> core::Result<LensFile> {
    const std::string what = "lens file " + path + ": ";
    core::Result<camera::ArrayCalibration> read =
        camera::read_array_calibration(path);
    if (!read.ok()) return read.status().with_context("lens file " + path);
    std::vector<std::string> wrong;
    std::vector<std::string> without;
    std::vector<std::string> rig;
    for (const capture::SessionCamera& cam : cameras) {
      rig.push_back(cam.id);
      const camera::SensorCalibration* entry =
          camera::find_sensor(*read, cam.id);
      if (entry == nullptr || !entry->color) {
        without.push_back(cam.id);
        continue;
      }
      const camera::ImageSize size = entry->color->model.size;
      if (size.width != cam.size.width || size.height != cam.size.height) {
        wrong.push_back(cam.id + "'s lens is " + size_text(size) +
                        ", its pictures " + size_text(cam.size));
      }
    }
    if (!wrong.empty()) {
      return core::Status::invalid_argument(what + listed(wrong, "; "));
    }
    if (without.size() == cameras.size()) {
      std::vector<std::string> held;
      for (const camera::SensorCalibration& entry : read->sensors) {
        if (entry.color) held.push_back(entry.id);
      }
      return core::Status::invalid_argument(
          what + "no lens for " + listed(rig, " or ") + ": it holds " +
          (held.empty() ? std::string("none") : listed(held, ", ")));
    }
    LensFile out;
    out.path = path;
    for (const std::string& id : without) {
      out.notes.push_back(id + " has no lens in " + path);
    }
    out.calibration =
        std::make_shared<const camera::ArrayCalibration>(*std::move(read));
    return out;
  });
}

core::Result<LensFile> passed_lenses(
    const CalibrateResult& result,
    const std::vector<capture::SessionCamera>& cameras) {
  return compute::guarded("passed_lenses", [&]() -> core::Result<LensFile> {
    if (result.mode != capture::SessionMode::Intrinsics ||
        result.directory.empty()) {
      return core::Status::invalid_argument(
          "use these lenses: the result shown is not a lens run's");
    }
    const std::string path =
        (fs::path(result.directory) / quality::kArrayCalibrationFile).string();
    std::error_code ec;
    if (!fs::is_regular_file(path, ec)) {
      return core::Status::invalid_argument("use these lenses: calibration " +
                                            std::to_string(result.number) +
                                            " wrote no lens file");
    }
    std::set<std::string> passed;
    std::vector<std::string> notes;
    for (const quality::LensVerdict& lens : result.lenses) {
      const std::string& id = lens.fit.camera.id;
      if (std::find(result.report_only.begin(), result.report_only.end(), id) !=
          result.report_only.end()) {
        notes.push_back(id +
                        "'s fit was for the report: its factory lens "
                        "stays");
      } else if (lens.pass) {
        passed.insert(id);
      } else {
        notes.push_back(id + " did not pass (" +
                        (lens.failed.empty() ? std::string("a gate")
                                             : lens.failed.front()) +
                        "): its lens is not taken");
      }
    }
    for (const UnsolvedLens& lens : result.unsolved) {
      notes.push_back(lens.id + " was not fitted (" + lens.reason +
                      "): no lens is taken for it");
    }
    if (passed.empty()) {
      return core::Status::invalid_argument(
          "use these lenses: no lens of calibration " +
          std::to_string(result.number) + " passed: none is taken");
    }
    VKC_ASSIGN(LensFile file, load_lens_file(path, cameras));
    camera::ArrayCalibration kept;
    for (const camera::SensorCalibration& entry : file.calibration->sensors) {
      if (passed.count(entry.id) != 0) kept.sensors.push_back(entry);
    }
    file.calibration =
        std::make_shared<const camera::ArrayCalibration>(std::move(kept));
    file.notes = std::move(notes);
    return file;
  });
}

core::Result<std::vector<viz::PanelLine>> setup_lines(
    capture::SessionMode mode,
    const std::vector<capture::SessionCamera>& cameras,
    const camera::ArrayCalibration* file, const std::string& path) {
  return compute::guarded(
      "setup_lines", [&]() -> core::Result<std::vector<viz::PanelLine>> {
        std::vector<viz::PanelLine> lines;
        lines.push_back({"lens file", path.empty() ? std::string("none") : path,
                         path.empty() ? viz::Tone::Muted : viz::Tone::Normal});
        for (const capture::SessionCamera& cam : cameras) {
          const LensStatus status = lens_status(mode, cam, file);
          std::string value = to_string(status);
          viz::Tone tone = viz::Tone::Normal;
          if (status == LensStatus::Calibrated) tone = viz::Tone::Good;
          if (status == LensStatus::Missing) {
            value += ": load a lens file";
            tone = viz::Tone::Bad;
          }
          if (cam.lens == capture::LensOrigin::File) {
            value += " (" + cam.lens_file + ")";
          }
          lines.push_back({"lens " + cam.id, value, tone});
        }
        return lines;
      });
}

core::Result<viz::BoardEditor> switch_session(
    const RigScenario& next,
    const std::optional<std::pair<Target, bool>>& board, CaptureSetup& setup,
    LiveOptions& live_options, RigMode& window, LiveWorker& live,
    CaptureWorker& captures) {
  return compute::guarded(
      "switch_session", [&]() -> core::Result<viz::BoardEditor> {
        CaptureSetup next_setup = setup;
        LiveOptions next_live = live_options;
        RigMode next_window;  // the images, uncalibrated
        VKC_ASSIGN(viz::BoardEditor editor,
                   apply_mode(next, next_setup, next_live, next_window));
        if (board) VKC_TRY(editor.edit(board->first, board->second));
        for (std::size_t i = 0; i < next_setup.cameras.size(); ++i) {
          VKC_ASSIGN(next_setup.lenses[i],
                     take_lens(next.mode, next.calibration.get(),
                               next.calibration_path, next_setup.cameras[i]));
        }
        // The live worker first: from here it hands on no capture of the
        // session closing, so the capture worker's restart, queued after,
        // comes after all of them.
        VKC_TRY(live.restart(next_setup.lenses, editor.board(),
                             editor.square_measured(), next_live.rule));
        VKC_TRY(captures.restart(next_setup));
        setup = std::move(next_setup);
        live_options = std::move(next_live);
        window = next_window;
        return editor;
      });
}

}  // namespace volumetric_kit::calib::app
