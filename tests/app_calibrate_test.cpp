// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's Calibrate: the rig solved and measured from the captures kept,
// for 2 to 5 cameras whose ids sort in another order than the rig's, in the
// reference camera's gauge; the same snapshot gives the same files, and a
// recalibration owes nothing to the last; each results directory numbered
// once, its calibration file written last; and the worker's runs, refusals,
// cancels and stops. Synthetic detections, no device.

#include "volumetric_kit/calib/app/calibrate_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "allocation_failure.hpp"
#include "kept_scene.hpp"
#include "scratch_dir.hpp"
#include "synthetic_scene.hpp"
#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace fs = std::filesystem;
namespace camera = recon::camera;
using json = nlohmann::json;
using Code = core::Status::Code;
using SteadyClock = std::chrono::steady_clock;
using test::ScratchDir;

// The time a Calibrate is pressed in the results tests: a fixed one.
const std::chrono::system_clock::time_point kPressed =
    std::chrono::system_clock::from_time_t(1792346400);

// The report, without the one figure that changes from run to run.
json timeless(const std::string& report) {
  json doc = json::parse(report);
  doc["solver"].erase("seconds");
  return doc;
}

// What a directory holds, by name.
std::set<std::string> entries(const fs::path& dir) {
  std::set<std::string> out;
  std::error_code ec;
  for (auto it = fs::directory_iterator(dir, ec);
       !ec && it != fs::directory_iterator(); it.increment(ec)) {
    out.insert(it->path().filename().string());
  }
  return out;
}

std::string read(const fs::path& path) {
  core::Result<std::string> bytes = core::read_file(path.string());
  EXPECT_TRUE(bytes.ok()) << bytes.status().message();
  return bytes.ok() ? *bytes : std::string();
}

// Waits until `done` holds, polling; false once `limit` passes.
template <class Done>
bool wait_for(const Done& done,
              std::chrono::milliseconds limit = std::chrono::minutes(5)) {
  const SteadyClock::time_point until = SteadyClock::now() + limit;
  while (!done()) {
    if (SteadyClock::now() > until) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return true;
}

bool idle(const CalibrateWorker& worker) {
  return wait_for([&] { return !worker.busy(); });
}

// Whether the worker is past its first leave-one-out re-solve.
bool measuring(const CalibrateWorker& worker) {
  return wait_for([&] {
    const std::optional<CalibrateProgress> at = worker.progress();
    return !at || (at->stage == CalibrateStage::Measuring && at->done >= 1);
  });
}

std::unique_ptr<CalibrateWorker> make_worker() {
  core::Result<std::unique_ptr<CalibrateWorker>> made =
      CalibrateWorker::create();
  EXPECT_TRUE(made.ok()) << made.status().message();
  return made.ok() ? std::move(made).value() : nullptr;
}

std::shared_ptr<const KeptCaptures> shared(KeptCaptures kept) {
  return std::make_shared<const KeptCaptures>(std::move(kept));
}

// Every camera within a millimetre and 0.05 degrees of the truth in the
// reference's gauge, the reference exactly at the identity; each pose the
// inverse of its T_cw, in rig order, and the calibration's for its id,
// whatever order the ids sort in; leave-one-out once per capture; the files
// formatted.
TEST(Calibrate, SolvesTwoToFiveCamerasInTheReferencesGauge) {
  struct Case {
    std::size_t cameras;
    std::size_t reference;
  };
  for (const Case& c : {Case{2, 0}, Case{3, 1}, Case{4, 0}, Case{5, 3}}) {
    SCOPED_TRACE(std::to_string(c.cameras) + " cameras, reference " +
                 std::to_string(c.reference));
    const test::Scene scene = test::make_kept_scene(c.cameras, 60);
    const KeptCaptures kept =
        test::kept_captures(scene, c.reference, "unwritten");
    core::Result<CalibrateResult> made = calibrate(kept);
    ASSERT_TRUE(made.ok()) << made.status().message();
    const CalibrateResult& r = *made;

    std::vector<std::string> ids;
    for (const solve::RigCamera& cam : scene.cameras) ids.push_back(cam.id);
    std::vector<std::string> sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    ASSERT_NE(sorted, ids);  // the ids are not in rig order
    EXPECT_EQ(r.ids, ids);
    EXPECT_EQ(r.reference, c.reference);
    EXPECT_EQ(r.captures, kept.captures.size());
    EXPECT_EQ(r.number, -1);
    EXPECT_TRUE(r.directory.empty());
    ASSERT_EQ(r.color_to_world.size(), c.cameras);
    EXPECT_EQ(r.color_to_world[c.reference], camera::Mat4d(1.0));
    const std::vector<camera::Mat4d> truth =
        test::truths_in_world_of(scene, c.reference);
    for (std::size_t k = 0; k < c.cameras; ++k) {
      SCOPED_TRACE(ids[k]);
      const camera::Mat4d world_to_camera =
          camera::rigid_inverse(r.color_to_world[k]);
      EXPECT_LT(1000.0 * glm::length(test::centre_of(world_to_camera) -
                                     test::centre_of(truth[k])),
                1.0);
      EXPECT_LT(test::angle_deg(world_to_camera, truth[k]), 0.05);
      EXPECT_EQ(r.color_to_world[k],
                camera::rigid_inverse(camera::matrix_from_rodrigues(
                    r.solution.world_to_camera[k])));
      const camera::SensorCalibration* sensor =
          camera::find_sensor(r.solution.calibration, ids[k]);
      ASSERT_NE(sensor, nullptr);
      EXPECT_EQ(*sensor->color_to_world, r.color_to_world[k]);
    }
    // The calibration's sensors are in id order, the rig's poses are not.
    EXPECT_NE(r.solution.calibration.sensors.front().id, ids.front());

    ASSERT_EQ(r.boards_to_world.size(), r.solution.boards.size());
    for (std::size_t b = 0; b < r.boards_to_world.size(); ++b) {
      EXPECT_EQ(r.boards_to_world[b], camera::matrix_from_rodrigues(
                                          r.solution.boards[b].board_to_world));
    }
    ASSERT_EQ(r.metrics.leave_one_out.captures.size(), kept.captures.size());
    for (std::size_t i = 0; i < kept.captures.size(); ++i) {
      EXPECT_EQ(r.metrics.leave_one_out.captures[i].capture,
                kept.captures[i]->index);
    }
    EXPECT_EQ(r.metrics.options.initialise.max_pnp_rms_px,
              kept.solve.max_pnp_rms_px);
    EXPECT_GT(r.seconds, 0.0);

    core::Result<camera::ArrayCalibration> file =
        camera::parse_array_calibration(r.files.array_calibration);
    ASSERT_TRUE(file.ok()) << file.status().message();
    EXPECT_EQ(file->world.sensor, ids[c.reference]);
    const json report = json::parse(r.files.report);
    EXPECT_EQ(report["format"], "volumetric_kit_calib.report");
    EXPECT_EQ(report["version"], 1);
    EXPECT_EQ(report["settings"]["reference"], ids[c.reference]);
    EXPECT_EQ(r.files.residuals.rfind(
                  "capture,serial,corner_id,u,v,du,dv,weight,dropped\n", 0),
              0u);
  }
}

// Each press solves from scratch: the same snapshot gives the same files but
// for the time, and the progress comes in order.
TEST(Calibrate, ARerunEqualsTheFirstRun) {
  const test::Scene scene = test::make_kept_scene(3, 20);
  const KeptCaptures kept = test::kept_captures(scene, 0, "unwritten");
  std::vector<CalibrateProgress> steps;
  core::Result<CalibrateResult> first = calibrate(
      kept, [&](const CalibrateProgress& at) { steps.push_back(at); });
  core::Result<CalibrateResult> again = calibrate(kept);
  ASSERT_TRUE(first.ok()) << first.status().message();
  ASSERT_TRUE(again.ok()) << again.status().message();
  EXPECT_EQ(timeless(first->files.report), timeless(again->files.report));
  EXPECT_EQ(first->files.array_calibration, again->files.array_calibration);
  EXPECT_EQ(first->files.residuals, again->files.residuals);
  EXPECT_EQ(first->color_to_world, again->color_to_world);

  // Solving, measuring, then each re-solve counted to the captures.
  const std::size_t k = kept.captures.size();
  ASSERT_EQ(steps.size(), 2 + k);
  EXPECT_EQ(steps[0].stage, CalibrateStage::Solving);
  EXPECT_EQ(steps[1].stage, CalibrateStage::Measuring);
  EXPECT_EQ(steps[1].total, 0u);
  for (std::size_t i = 0; i < k; ++i) {
    EXPECT_EQ(steps[2 + i].stage, CalibrateStage::Measuring);
    EXPECT_EQ(steps[2 + i].done, i + 1);
    EXPECT_EQ(steps[2 + i].total, k);
  }
  for (std::size_t i = 1; i < steps.size(); ++i) {
    EXPECT_GE(steps[i].seconds, steps[i - 1].seconds);
  }
  EXPECT_STREQ(to_string(CalibrateStage::Solving), "solving");
  EXPECT_STREQ(to_string(CalibrateStage::Measuring), "measuring");
  EXPECT_STREQ(to_string(CalibrateStage::Writing), "writing");
  EXPECT_STREQ(to_string(static_cast<CalibrateStage>(9)), "unknown");
}

// A throw from the progress stops the run, which returns it.
TEST(Calibrate, AThrowFromTheProgressStopsTheRun) {
  const test::Scene scene = test::make_kept_scene(2, 10);
  const KeptCaptures kept = test::kept_captures(scene, 0, "unwritten");
  std::size_t calls = 0;
  const core::Result<CalibrateResult> stopped =
      calibrate(kept, [&](const CalibrateProgress& at) {
        ++calls;
        if (at.done == 2) throw std::runtime_error("stop here");
      });
  ASSERT_FALSE(stopped.ok());
  EXPECT_NE(stopped.status().message().find("stop here"), std::string::npos)
      << stopped.status().message();
  EXPECT_EQ(calls, 4u);  // solving, measuring, two re-solves
}

// What the solve cannot take is refused, naming it.
TEST(Calibrate, RefusesWhatTheSolveCannotTake) {
  const test::Scene scene = test::make_kept_scene(4, 20);
  const KeptCaptures base = test::kept_captures(scene, 0, "unwritten");
  const auto refusal = [](const KeptCaptures& kept) {
    const core::Result<CalibrateResult> made = calibrate(kept);
    EXPECT_FALSE(made.ok());
    EXPECT_EQ(made.status().domain(), Code::InvalidArgument);
    return made.status().message();
  };

  KeptCaptures none = base;
  none.captures.clear();
  EXPECT_EQ(refusal(none), "calibrate: no capture is kept");

  KeptCaptures many = base;
  many.captures.assign(151, base.captures.front());
  EXPECT_EQ(refusal(many),
            "calibrate: 151 captures, more than one Calibrate adjusts (150): "
            "drop some");

  KeptCaptures short_of = base;
  auto fewer = std::make_shared<KeptCapture>(*base.captures[2]);
  fewer->views.pop_back();
  short_of.captures[2] = fewer;
  EXPECT_EQ(refusal(short_of),
            "calibrate: capture 5 has 3 views for 4 cameras");

  KeptCaptures renamed = base;
  auto other = std::make_shared<KeptCapture>(*base.captures[0]);
  for (Detection& view : other->views) {
    if (!view.observation.empty()) {
      view.observation.frame_id = 7;
      break;
    }
  }
  renamed.captures[0] = other;
  EXPECT_NE(refusal(renamed).find("calibrate: capture 1's view of camera"),
            std::string::npos);

  // Camera CL0B in no capture: outside the tree, as initialise names it.
  KeptCaptures unlinked = base;
  for (std::shared_ptr<const KeptCapture>& capture : unlinked.captures) {
    auto copy = std::make_shared<KeptCapture>(*capture);
    copy->views[3].observation.points.clear();
    capture = copy;
  }
  const std::string why = refusal(unlinked);
  EXPECT_EQ(why.rfind("calibrate: initialise: ", 0), 0u) << why;
  EXPECT_NE(why.find("CL0B"), std::string::npos) << why;
}

// The name: RRR zero-padded, then the press's stamp.
TEST(WriteResults, NamesEachDirectoryByNumberAndPress) {
  core::Result<std::string> stamp = capture::session_stamp(kPressed);
  ASSERT_TRUE(stamp.ok());
  core::Result<std::string> name = results_directory("/s", 7, kPressed);
  ASSERT_TRUE(name.ok()) << name.status().message();
  EXPECT_EQ(*name, "/s/results/007_" + *stamp);
  EXPECT_TRUE(
      std::regex_match(*stamp, std::regex(R"(\d{4}-\d{2}-\d{2}_\d{6})")));
  EXPECT_EQ(*results_directory("/s", 999, kPressed),
            "/s/results/999_" + *stamp);
  for (const int bad : {-1, 1000}) {
    const core::Result<std::string> refused =
        results_directory("/s", bad, kPressed);
    EXPECT_EQ(refused.status().domain(), Code::InvalidArgument);
    EXPECT_EQ(refused.status().message(), "results_directory: the number " +
                                              std::to_string(bad) +
                                              " is not from 0 to 999");
  }
}

quality::ResultFiles some_files() {
  quality::ResultFiles files;
  files.array_calibration = "{\"calibration\": 1}\n";
  files.report = "{\"report\": 1}\n";
  files.residuals = "capture,serial\n";
  return files;
}

// 000_, then 001_; results/ is made once, and a number an entry already has
// is never taken again, whatever its stamp.
TEST(WriteResults, NumbersEachDirectoryOnce) {
  const ScratchDir session;
  const quality::ResultFiles files = some_files();
  const std::string stamp = *capture::session_stamp(kPressed);
  core::Result<WrittenResults> first =
      write_results(session.path().string(), 0, kPressed, files);
  ASSERT_TRUE(first.ok()) << first.status().message();
  EXPECT_EQ(first->number, 0);
  EXPECT_EQ(first->directory,
            *results_directory(session.path().string(), 0, kPressed));
  core::Result<WrittenResults> second =
      write_results(session.path().string(), 0, kPressed, files);
  ASSERT_TRUE(second.ok()) << second.status().message();
  EXPECT_EQ(second->number, 1);
  const fs::path results = session.path() / kResultsDirectory;
  EXPECT_EQ(entries(results),
            (std::set<std::string>{"000_" + stamp, "001_" + stamp}));
  for (const char* name : {quality::kArrayCalibrationFile, quality::kReportFile,
                           quality::kResidualsFile}) {
    EXPECT_TRUE(fs::is_regular_file(fs::path(second->directory) / name));
  }
  EXPECT_EQ(read(fs::path(second->directory) / quality::kReportFile),
            files.report);

  // An entry 004_ of another press: the next is 005, and a higher first
  // number is taken as it is.
  fs::create_directory(results / "004_2026-01-01_000000");
  core::Result<WrittenResults> skipped =
      write_results(session.path().string(), 2, kPressed, files);
  ASSERT_TRUE(skipped.ok()) << skipped.status().message();
  EXPECT_EQ(skipped->number, 5);
  core::Result<WrittenResults> higher =
      write_results(session.path().string(), 9, kPressed, files);
  ASSERT_TRUE(higher.ok()) << higher.status().message();
  EXPECT_EQ(higher->number, 9);

  // None left below 1000.
  fs::create_directory(results / "999_2026-01-01_000000");
  const core::Result<WrittenResults> full =
      write_results(session.path().string(), 0, kPressed, files);
  EXPECT_EQ(full.status().domain(), Code::InvalidArgument);
  EXPECT_NE(full.status().message().find("every number to 999"),
            std::string::npos)
      << full.status().message();
  const core::Result<WrittenResults> out_of_range =
      write_results(session.path().string(), 1000, kPressed, files);
  EXPECT_EQ(out_of_range.status().domain(), Code::InvalidArgument);
}

// The residuals, then the report, then the calibration file last: a failure
// at the report leaves a directory with no calibration file in it.
TEST(WriteResults, WritesTheCalibrationFileLast) {
  const ScratchDir session;
  const quality::ResultFiles files = some_files();
  std::vector<std::string> order;
  const auto recording = [&](const std::string& path, std::string_view bytes) {
    order.push_back(fs::path(path).filename().string());
    return core::write_file(path, bytes, core::WriteMode::CreateNew);
  };
  ASSERT_TRUE(
      write_results(session.path().string(), 0, kPressed, files, recording)
          .ok());
  EXPECT_EQ(order, (std::vector<std::string>{quality::kResidualsFile,
                                             quality::kReportFile,
                                             quality::kArrayCalibrationFile}));

  order.clear();
  const auto failing = [&](const std::string& path, std::string_view bytes) {
    order.push_back(fs::path(path).filename().string());
    if (order.back() == quality::kReportFile) {
      return core::Status::io_error("the disk is full");
    }
    return core::write_file(path, bytes, core::WriteMode::CreateNew);
  };
  const core::Result<WrittenResults> failed =
      write_results(session.path().string(), 0, kPressed, files, failing);
  ASSERT_FALSE(failed.ok());
  EXPECT_EQ(failed.status().domain(), Code::IoError);
  EXPECT_EQ(failed.status().message(), "write_results: the disk is full");
  EXPECT_EQ(order, (std::vector<std::string>{quality::kResidualsFile,
                                             quality::kReportFile}));
  const std::string stamp = *capture::session_stamp(kPressed);
  EXPECT_EQ(entries(session.path() / kResultsDirectory / ("001_" + stamp)),
            std::set<std::string>{quality::kResidualsFile});
}

// A session that is not there, or a results/ that is not a directory, is
// refused, and nothing is written.
TEST(WriteResults, RefusesAMissingSessionOrAResultsFile) {
  const ScratchDir root;
  const quality::ResultFiles files = some_files();
  const fs::path missing = root.path() / "missing";
  const core::Result<WrittenResults> lost =
      write_results(missing.string(), 0, kPressed, files);
  EXPECT_EQ(lost.status().domain(), Code::NotFound);
  EXPECT_EQ(lost.status().message().rfind("write_results: ", 0), 0u)
      << lost.status().message();
  EXPECT_FALSE(fs::exists(missing));

  ASSERT_TRUE(core::write_file(root / kResultsDirectory, "not a directory",
                               core::WriteMode::CreateNew)
                  .ok());
  const core::Result<WrittenResults> file =
      write_results(root.path().string(), 0, kPressed, files);
  EXPECT_EQ(file.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(entries(root.path()),
            std::set<std::string>{std::string(kResultsDirectory)});
}

// A run's result is published once its three files are on disk, as it
// formatted them; the numbers go up from 0; a request while busy, or a null
// one, starts nothing.
TEST(CalibrateWorker, PublishesEachResultOnceItsFilesAreWritten) {
  const ScratchDir session;
  const test::Scene scene = test::make_kept_scene(3, 20);
  const auto kept =
      shared(test::kept_captures(scene, 1, session.path().string()));
  const std::unique_ptr<CalibrateWorker> worker = make_worker();
  ASSERT_NE(worker, nullptr);
  EXPECT_FALSE(worker->busy());
  EXPECT_FALSE(worker->progress().has_value());
  EXPECT_FALSE(worker->cancel());
  EXPECT_EQ(worker->take_result(), nullptr);
  EXPECT_EQ(worker->request(nullptr).status().domain(), Code::InvalidArgument);

  for (int number = 0; number < 2; ++number) {
    SCOPED_TRACE(number);
    core::Result<bool> started = worker->request(kept);
    ASSERT_TRUE(started.ok()) << started.status().message();
    ASSERT_TRUE(*started);
    EXPECT_TRUE(worker->busy());
    EXPECT_FALSE(*worker->request(kept));  // busy
    ASSERT_TRUE(idle(*worker));
    const std::shared_ptr<const CalibrateResult> result = worker->take_result();
    ASSERT_NE(result, nullptr);
    EXPECT_EQ(worker->take_result(), nullptr);
    EXPECT_TRUE(worker->take_errors().empty());
    EXPECT_EQ(result->number, number);
    const fs::path dir(result->directory);
    EXPECT_EQ(dir.parent_path(), session.path() / kResultsDirectory);
    EXPECT_TRUE(std::regex_match(dir.filename().string(),
                                 std::regex("00" + std::to_string(number) +
                                            R"(_\d{4}-\d{2}-\d{2}_\d{6})")));
    EXPECT_EQ(read(dir / quality::kResidualsFile), result->files.residuals);
    EXPECT_EQ(read(dir / quality::kReportFile), result->files.report);
    EXPECT_EQ(read(dir / quality::kArrayCalibrationFile),
              result->files.array_calibration);
    core::Result<camera::ArrayCalibration> file =
        camera::read_array_calibration(
            (dir / quality::kArrayCalibrationFile).string());
    ASSERT_TRUE(file.ok()) << file.status().message();
    EXPECT_TRUE(camera::validate_array_calibration(*file).ok());
    EXPECT_EQ(file->world.sensor, scene.cameras[1].id);
  }
}

// A recalibration with more captures writes the next directory, and equals a
// fresh run on those captures: nothing of the last run carries over.
TEST(CalibrateWorker, ARecalibrationEqualsAFreshRun) {
  const ScratchDir session;
  const test::Scene scene = test::make_kept_scene(4, 30);
  const std::unique_ptr<CalibrateWorker> worker = make_worker();
  ASSERT_NE(worker, nullptr);
  ASSERT_TRUE(*worker->request(
      shared(test::kept_captures(scene, 0, session.path().string(), 20))));
  ASSERT_TRUE(idle(*worker));
  const std::shared_ptr<const CalibrateResult> first = worker->take_result();
  ASSERT_NE(first, nullptr);
  const KeptCaptures all =
      test::kept_captures(scene, 0, session.path().string());
  ASSERT_TRUE(*worker->request(shared(all)));
  ASSERT_TRUE(idle(*worker));
  const std::shared_ptr<const CalibrateResult> second = worker->take_result();
  ASSERT_NE(second, nullptr);
  EXPECT_EQ(first->number, 0);
  EXPECT_EQ(second->number, 1);
  EXPECT_EQ(first->captures, 20u);
  EXPECT_EQ(second->captures, 30u);

  core::Result<CalibrateResult> fresh = calibrate(all);
  ASSERT_TRUE(fresh.ok()) << fresh.status().message();
  EXPECT_EQ(timeless(second->files.report), timeless(fresh->files.report));
  EXPECT_EQ(second->files.array_calibration, fresh->files.array_calibration);
  EXPECT_EQ(second->color_to_world, fresh->color_to_world);
  EXPECT_NE(first->files.array_calibration, second->files.array_calibration);
}

// A refusal is posted, naming its cause, and nothing is written or
// published; the next run takes number 0.
TEST(CalibrateWorker, PostsARefusalAndWritesNothing) {
  const ScratchDir session;
  const test::Scene scene = test::make_kept_scene(4, 20);
  KeptCaptures unlinked =
      test::kept_captures(scene, 0, session.path().string());
  for (std::shared_ptr<const KeptCapture>& capture : unlinked.captures) {
    auto copy = std::make_shared<KeptCapture>(*capture);
    copy->views[3].observation.points.clear();
    capture = copy;
  }
  const std::unique_ptr<CalibrateWorker> worker = make_worker();
  ASSERT_NE(worker, nullptr);
  ASSERT_TRUE(*worker->request(shared(unlinked)));
  ASSERT_TRUE(idle(*worker));
  EXPECT_EQ(worker->take_result(), nullptr);
  const std::vector<core::Status> errors = worker->take_errors();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].domain(), Code::InvalidArgument);
  EXPECT_EQ(errors[0].message().rfind("Calibrate: calibrate: initialise: ", 0),
            0u)
      << errors[0].message();
  EXPECT_NE(errors[0].message().find("CL0B"), std::string::npos);
  EXPECT_TRUE(entries(session.path()).empty());

  // A results/ that is a file: the write fails, and nothing is published.
  ASSERT_TRUE(core::write_file(session / kResultsDirectory, "a file",
                               core::WriteMode::CreateNew)
                  .ok());
  ASSERT_TRUE(*worker->request(
      shared(test::kept_captures(scene, 0, session.path().string()))));
  ASSERT_TRUE(idle(*worker));
  EXPECT_EQ(worker->take_result(), nullptr);
  const std::vector<core::Status> failed = worker->take_errors();
  ASSERT_EQ(failed.size(), 1u);
  EXPECT_EQ(failed[0].message().rfind("Calibrate: write_results: ", 0), 0u)
      << failed[0].message();
}

// A cancel during leave one out ends the run within a second, and nothing is
// written, published or posted.
TEST(CalibrateWorker, ACancelWritesNothing) {
  const ScratchDir session;
  const test::Scene scene = test::make_kept_scene(4, 60);
  const std::unique_ptr<CalibrateWorker> worker = make_worker();
  ASSERT_NE(worker, nullptr);
  ASSERT_TRUE(*worker->request(
      shared(test::kept_captures(scene, 0, session.path().string()))));
  ASSERT_TRUE(measuring(*worker));
  const std::optional<CalibrateProgress> at = worker->progress();
  ASSERT_TRUE(at.has_value());
  EXPECT_EQ(at->stage, CalibrateStage::Measuring);
  EXPECT_EQ(at->total, 60u);
  EXPECT_LT(at->done, at->total);
  EXPECT_GT(at->seconds, 0.0);
  const SteadyClock::time_point cancelled = SteadyClock::now();
  EXPECT_TRUE(worker->cancel());
  ASSERT_TRUE(idle(*worker));
  EXPECT_LT(SteadyClock::now() - cancelled, std::chrono::seconds(1));
  EXPECT_EQ(worker->take_result(), nullptr);
  EXPECT_TRUE(worker->take_errors().empty());
  EXPECT_TRUE(entries(session.path()).empty());
  EXPECT_FALSE(worker->cancel());  // nothing to cancel
}

// stop() cancels a run and returns once its thread has; the worker then
// takes no request. The destructor stops it too, and a worker given none
// goes at once.
TEST(CalibrateWorker, StopsWithARunUnderWay) {
  const ScratchDir session;
  const test::Scene scene = test::make_kept_scene(4, 60);
  const auto kept =
      shared(test::kept_captures(scene, 0, session.path().string()));
  {
    const std::unique_ptr<CalibrateWorker> worker = make_worker();
    ASSERT_NE(worker, nullptr);
    ASSERT_TRUE(*worker->request(kept));
    ASSERT_TRUE(measuring(*worker));
    const SteadyClock::time_point stopping = SteadyClock::now();
    worker->stop();
    EXPECT_LT(SteadyClock::now() - stopping, std::chrono::seconds(1));
    EXPECT_FALSE(worker->busy());
    EXPECT_FALSE(*worker->request(kept));
    EXPECT_EQ(worker->take_result(), nullptr);
    EXPECT_TRUE(worker->take_errors().empty());
    worker->stop();  // idempotent
  }
  {
    std::unique_ptr<CalibrateWorker> worker = make_worker();
    ASSERT_NE(worker, nullptr);
    ASSERT_TRUE(*worker->request(kept));
    const SteadyClock::time_point destroying = SteadyClock::now();
    worker.reset();  // during its solve, which ends before it does
    EXPECT_LT(SteadyClock::now() - destroying, std::chrono::seconds(30));
  }
  EXPECT_TRUE(entries(session.path()).empty());
  EXPECT_EQ(kept.use_count(), 1);  // no thread holds the snapshot
  make_worker().reset();
}

// Out of memory anywhere in a run fails it whole: a small rig of exact
// detections, so each re-solve takes few iterations.
TEST(Calibrate, OutOfMemoryAnywhereFailsTheRun) {
  const test::Scene scene = test::make_kept_scene(2, 4, 0.0);
  const KeptCaptures kept = test::kept_captures(scene, 0, "unwritten");
  ASSERT_TRUE(calibrate(kept).ok());
  test::expect_oom_anywhere(
      [&] { return calibrate(kept); },
      [](const core::Result<CalibrateResult>& r) {
        return r.status().domain() == Code::Backend &&
               r.status().message().find("solvePnPGeneric") !=
                   std::string::npos;
      });
}

}  // namespace
}  // namespace volumetric_kit::calib::app
