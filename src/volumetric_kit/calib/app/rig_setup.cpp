// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/rig_setup.hpp"

#include <algorithm>
#include <filesystem>
#include <set>
#include <system_error>
#include <tuple>

#include <nlohmann/json.hpp>

#include "size_text.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/quality/results.hpp"
#include "volumetric_kit/core/base/file_io.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = sensor::camera;
namespace fs = std::filesystem;

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
// order; why not, for a report that is not a lens run's.
core::Result<std::vector<std::pair<std::string, bool>>> lens_report(
    const std::string& report) {
  const auto refused = [](const char* why) {
    return core::Status::invalid_argument(std::string("not a lens run's: ") +
                                          why);
  };
  const nlohmann::ordered_json json =
      nlohmann::ordered_json::parse(report, nullptr, false);
  if (json.is_discarded() || !json.is_object()) return refused("no JSON");
  const auto format = json.find("format");
  if (format == json.end() || !format->is_string() ||
      format->get<std::string>() != quality::kReportFormat) {
    return refused("not calib's report");
  }
  const auto lenses = json.find("lenses");
  if (lenses == json.end() || !lenses->is_object()) {
    return refused("no lenses section");
  }
  const auto cameras = lenses->find("cameras");
  if (cameras == lenses->end() || !cameras->is_object()) {
    return refused("no cameras in its lenses section");
  }
  std::vector<std::pair<std::string, bool>> out;
  for (auto it = cameras->begin(); it != cameras->end(); ++it) {
    const nlohmann::ordered_json& camera = it.value();
    if (!camera.is_object()) return refused("a camera that is not an object");
    const auto written = camera.find("written");
    const auto verdict = camera.find("verdict");
    if (written == camera.end() || !written->is_boolean() ||
        verdict == camera.end() || !verdict->is_string()) {
      return refused("a camera without its written or verdict");
    }
    if (written->get<bool>()) {
      out.emplace_back(it.key(), verdict->get<std::string>() == "pass");
    }
  }
  return out;
}

// `path` read as a lens file for the rig's `cameras`, without the lenses of
// `failed` (a lens run's that failed its gates). Its notes name each camera
// it gives no lens, in rig order.
core::Result<LensFile> lenses_for_rig(
    const std::string& path, const std::vector<capture::SessionCamera>& cameras,
    const std::set<std::string>& failed) {
  const std::string what = "lens file " + path + ": ";
  core::Result<camera::ArrayCalibration> read =
      camera::read_array_calibration(path);
  if (!read.ok()) return read.status().with_context("lens file " + path);
  std::vector<std::string> wrong;
  std::vector<std::string> rig;
  std::vector<std::string> notes;
  bool any = false;
  bool any_failed = false;
  for (const capture::SessionCamera& cam : cameras) {
    rig.push_back(cam.id);
    if (failed.count(cam.id) != 0) {
      notes.push_back(cam.id +
                      "'s lens failed its gates in that run: not taken");
      any_failed = true;
      continue;
    }
    const camera::SensorCalibration* entry = camera::find_sensor(*read, cam.id);
    if (entry == nullptr || !entry->color) {
      notes.push_back(cam.id + " has no lens in " + path);
      continue;
    }
    any = true;
    const camera::ImageSize size = entry->color->model.size;
    if (size.width != cam.size.width || size.height != cam.size.height) {
      wrong.push_back(cam.id + "'s lens is " + size_text(size) +
                      ", its pictures " + size_text(cam.size));
    }
  }
  if (!wrong.empty()) {
    return core::Status::invalid_argument(what + listed(wrong, "; "));
  }
  if (!any && any_failed) {
    return core::Status::invalid_argument(
        what + "no lens for " + listed(rig, " or ") +
        " passed in that run, and none is carried");
  }
  if (!any) {
    std::vector<std::string> held;
    for (const camera::SensorCalibration& entry : read->sensors) {
      if (entry.color) held.push_back(entry.id);
    }
    return core::Status::invalid_argument(
        what + "no lens for " + listed(rig, " or ") + ": it holds " +
        (held.empty() ? std::string("none") : listed(held, ", ")));
  }
  std::vector<camera::SensorCalibration>& sensors = read->sensors;
  sensors.erase(std::remove_if(sensors.begin(), sensors.end(),
                               [&](const camera::SensorCalibration& entry) {
                                 return failed.count(entry.id) != 0;
                               }),
                sensors.end());
  LensFile out;
  out.path = path;
  out.notes = std::move(notes);
  out.calibration =
      std::make_shared<const camera::ArrayCalibration>(*std::move(read));
  return out;
}

}  // namespace

core::Result<LensResult> read_lens_run(const std::string& directory) {
  return compute::guarded("read_lens_run", [&]() -> core::Result<LensResult> {
    const fs::path run(directory);
    const fs::path file = run / quality::kArrayCalibrationFile;
    const fs::path report = run / quality::kReportFile;
    if (!results_name(run.filename().string())) {
      return core::Status::invalid_argument("read_lens_run: " + directory +
                                            " is not a results directory");
    }
    std::error_code ec;
    for (const fs::path& needed : {file, report}) {
      if (!fs::is_regular_file(needed, ec)) {
        return core::Status::not_found("read_lens_run: " + directory +
                                       " has no " + needed.filename().string());
      }
    }
    VKC_ASSIGN(const std::string text, core::read_file(report.string()));
    core::Result<std::vector<std::pair<std::string, bool>>> cameras =
        lens_report(text);
    if (!cameras.ok()) {
      return cameras.status().with_context("read_lens_run: " + report.string());
    }
    return LensResult{file.string(),
                      run.parent_path().parent_path().filename().string(),
                      run.filename().string(), *std::move(cameras)};
  });
}

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
            core::Result<LensResult> read = read_lens_run(run->path().string());
            if (read.ok()) found.push_back(*std::move(read));
          }
        }
        // Newest first: the stamp after `RRR_`, then the session, then RRR.
        const auto key = [](const LensResult& r) {
          const ResultsName name = *results_name(r.results);
          return std::make_tuple(name.stamp, r.session, name.number);
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
        const std::optional<ResultsName> name = results_name(result.results);
        if (!name) {
          return core::Status::invalid_argument(
              "lens_result_label: " + result.results +
              " is not a results directory's name");
        }
        // The stamp, `YYYY-MM-DD_HHMMSS` (capture::session_stamp), as
        // "YYYY-MM-DD HH:MM:SS".
        const std::string& stamp = name->stamp;
        std::string when = stamp;
        if (stamp.size() == 17 && stamp[10] == '_') {
          when = stamp.substr(0, 10) + " " + stamp.substr(11, 2) + ":" +
                 stamp.substr(13, 2) + ":" + stamp.substr(15, 2);
        }
        std::string number = std::to_string(name->number);
        number.insert(0, 3 - number.size(), '0');
        std::vector<std::string> verdicts;
        for (const auto& [id, pass] : result.cameras) {
          verdicts.push_back(id + (pass ? " pass" : " fail"));
        }
        return when + "  " +
               (verdicts.empty() ? std::string("no lens written")
                                 : listed(verdicts, ", ")) +
               "  (" + result.session + "/" + number + ")";
      });
}

core::Result<LensFile> load_lens_file(
    const std::string& path,
    const std::vector<capture::SessionCamera>& cameras) {
  return compute::guarded("load_lens_file", [&]() -> core::Result<LensFile> {
    return lenses_for_rig(path, cameras, {});
  });
}

core::Result<LensFile> lens_run_file(
    const LensResult& run, const std::vector<capture::SessionCamera>& cameras) {
  return compute::guarded("lens_run_file", [&]() -> core::Result<LensFile> {
    std::set<std::string> failed;
    for (const auto& [id, pass] : run.cameras) {
      if (!pass) failed.insert(id);
    }
    return lenses_for_rig(run.path, cameras, failed);
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
        // Checked before anything changes; then the live worker first: from
        // here it hands on no capture of the session closing, so the capture
        // worker's restart, queued after and before the window's next press,
        // comes after all of them.
        VKC_TRY(captures.accepts(next_setup));
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
