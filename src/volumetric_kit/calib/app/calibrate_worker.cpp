// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/calibrate_worker.hpp"

#include <algorithm>
#include <condition_variable>
#include <exception>
#include <filesystem>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>

#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"

namespace volumetric_kit::calib::app {

namespace {

namespace camera = recon::camera;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using ResultPtr = std::shared_ptr<const CalibrateResult>;

// The highest results number, `RRR`.
constexpr int kMaxNumber = 999;

double seconds_since(Clock::time_point began) {
  return std::chrono::duration<double>(Clock::now() - began).count();
}

// What the worker's progress callback throws once its run is cancelled.
struct Cancelled final : std::exception {
  const char* what() const noexcept override { return "cancelled"; }
};

// Every view with corners in every kept capture, as the solve takes them;
// refuses a capture whose views are not one per camera, each its camera's
// and its capture's.
core::Result<std::vector<Observation>> solve_views(const KeptCaptures& kept) {
  const std::size_t most = solve::BundleAdjustOptions{}.max_captures;
  if (kept.captures.empty()) {
    return core::Status::invalid_argument("no capture is kept");
  }
  if (kept.captures.size() > most) {
    return core::Status::invalid_argument(
        std::to_string(kept.captures.size()) +
        " captures, more than one Calibrate adjusts (" + std::to_string(most) +
        "): drop some");
  }
  std::vector<Observation> views;
  for (const std::shared_ptr<const KeptCapture>& capture : kept.captures) {
    if (capture == nullptr) {
      return core::Status::invalid_argument("a kept capture is null");
    }
    const std::string name = "capture " + std::to_string(capture->index);
    if (capture->views.size() != kept.cameras.size()) {
      return core::Status::invalid_argument(
          name + " has " + std::to_string(capture->views.size()) +
          " views for " + std::to_string(kept.cameras.size()) + " cameras");
    }
    for (std::size_t c = 0; c < kept.cameras.size(); ++c) {
      const Observation& view = capture->views[c].observation;
      if (view.empty()) continue;
      if (view.camera_id != kept.cameras[c].id ||
          view.frame_id != capture->index) {
        return core::Status::invalid_argument(
            name + "'s view of camera " + kept.cameras[c].id +
            " names camera " + view.camera_id + " in capture " +
            std::to_string(view.frame_id));
      }
      views.push_back(view);
    }
  }
  return views;
}

}  // namespace

const char* to_string(CalibrateStage stage) noexcept {
  switch (stage) {
    case CalibrateStage::Solving:
      return "solving";
    case CalibrateStage::Measuring:
      return "measuring";
    case CalibrateStage::Writing:
      return "writing";
  }
  return "unknown";
}

core::Result<CalibrateResult> calibrate(const KeptCaptures& kept,
                                        const CalibrateCallback& progress) {
  constexpr const char* kWhat = "calibrate";
  return compute::guarded(kWhat, [&]() -> core::Result<CalibrateResult> {
    const Clock::time_point began = Clock::now();
    const auto report = [&](CalibrateStage stage, std::size_t done,
                            std::size_t total) {
      if (progress) progress({stage, done, total, seconds_since(began)});
    };
    const auto in = [&](const core::Status& status) {
      return status.with_context(kWhat);
    };
    report(CalibrateStage::Solving, 0, 0);
    core::Result<std::vector<Observation>> views = solve_views(kept);
    if (!views.ok()) return in(views.status());
    core::Result<compute::Board> board = compute::Board::create(kept.board);
    if (!board.ok()) return in(board.status());
    core::Result<solve::InitialRig> start = solve::initialise(
        kept.cameras, *board, *views, kept.reference, kept.solve);
    if (!start.ok()) return in(start.status());
    quality::MetricsOptions options;
    options.initialise = kept.solve;
    core::Result<solve::RigSolution> solution = solve::bundle_adjust(
        kept.cameras, *board, *views, *start, options.bundle_adjust);
    if (!solution.ok()) return in(solution.status());

    report(CalibrateStage::Measuring, 0, 0);
    core::Result<quality::CalibrationMetrics> metrics =
        quality::calibration_metrics(
            kept.cameras, *board, *views, *start, *solution, options,
            [&](std::size_t done, std::size_t total) {
              report(CalibrateStage::Measuring, done, total);
            });
    if (!metrics.ok()) return in(metrics.status());
    core::Result<quality::ResultFiles> files =
        quality::format_results(kept.cameras, *solution, *metrics);
    if (!files.ok()) return in(files.status());

    CalibrateResult out;
    out.reference = kept.reference;
    out.captures = kept.captures.size();
    for (const solve::RigCamera& cam : kept.cameras) {
      const camera::SensorCalibration* sensor =
          camera::find_sensor(solution->calibration, cam.id);
      if (sensor == nullptr || !sensor->color_to_world) {
        return core::Status::numerical(
            "calibrate: the solution has no pose "
            "for camera " +
            cam.id);
      }
      out.ids.push_back(cam.id);
      out.color_to_world.push_back(*sensor->color_to_world);
    }
    for (const solve::SolvedBoard& solved : solution->boards) {
      out.boards_to_world.push_back(
          camera::matrix_from_rodrigues(solved.board_to_world));
    }
    out.solution = *std::move(solution);
    out.metrics = *std::move(metrics);
    out.files = *std::move(files);
    out.seconds = seconds_since(began);
    return out;
  });
}

core::Result<std::string> results_directory(
    const std::string& session, int number,
    std::chrono::system_clock::time_point stamp) {
  constexpr const char* kWhat = "results_directory";
  return compute::guarded(kWhat, [&]() -> core::Result<std::string> {
    if (number < 0 || number > kMaxNumber) {
      return core::Status::invalid_argument(
          std::string(kWhat) + ": the number " + std::to_string(number) +
          " is not from 0 to " + std::to_string(kMaxNumber));
    }
    core::Result<std::string> when = capture::session_stamp(stamp);
    if (!when.ok()) return when.status().with_context(kWhat);
    std::string rrr = std::to_string(number);
    rrr.insert(0, 3 - rrr.size(), '0');
    return (fs::path(session) / kResultsDirectory / (rrr + "_" + *when))
        .string();
  });
}

core::Result<WrittenResults> write_results(
    const std::string& session, int first_number,
    std::chrono::system_clock::time_point stamp,
    const quality::ResultFiles& files, const ResultSink& sink) {
  constexpr const char* kWhat = "write_results";
  return compute::guarded(kWhat, [&]() -> core::Result<WrittenResults> {
    const auto in = [&](const core::Status& status) {
      return status.with_context(kWhat);
    };
    if (session.empty()) {
      // Not the working directory's results/.
      return core::Status::invalid_argument(std::string(kWhat) +
                                            ": no session directory");
    }
    if (first_number < 0 || first_number > kMaxNumber) {
      return core::Status::invalid_argument(
          std::string(kWhat) + ": first_number " +
          std::to_string(first_number) + " is not from 0 to " +
          std::to_string(kMaxNumber));
    }
    // results/, unless it is a directory already; a missing session is
    // NotFound, results/ as a file InvalidArgument (create_new_directory).
    const fs::path results = fs::path(session) / kResultsDirectory;
    std::error_code error;
    if (!fs::is_directory(results, error)) {
      const core::Status made = core::create_new_directory(results.string());
      if (!made.ok() && !fs::is_directory(results, error)) return in(made);
    }
    // Past every number an entry of results/ already has.
    int number = first_number;
    fs::directory_iterator entry(results, error);
    for (; !error && entry != fs::directory_iterator();
         entry.increment(error)) {
      const std::string name = entry->path().filename().string();
      if (name.size() > 4 && name[3] == '_' &&
          std::all_of(name.begin(), name.begin() + 3,
                      [](char c) { return c >= '0' && c <= '9'; })) {
        number = std::max(number, std::stoi(name.substr(0, 3)) + 1);
      }
    }
    if (error) {
      return core::Status::io_error(std::string(kWhat) + ": " +
                                    results.string() +
                                    " cannot be listed: " + error.message());
    }
    // A name taken meanwhile moves to the next number.
    std::string directory;
    for (; number <= kMaxNumber; ++number) {
      VKC_ASSIGN(directory, results_directory(session, number, stamp));
      const core::Status made = core::create_new_directory(directory);
      if (made.ok()) break;
      if (made.domain() != core::Status::Code::InvalidArgument) {
        return in(made);
      }
    }
    if (number > kMaxNumber) {
      return core::Status::invalid_argument(
          std::string(kWhat) + ": every number to " +
          std::to_string(kMaxNumber) + " in " + results.string() + " is taken");
    }
    ResultSink write = sink;
    if (!write) {
      write = [](const std::string& path, std::string_view bytes) {
        return core::write_file(path, bytes, core::WriteMode::CreateNew);
      };
    }
    // The calibration file last, so a directory a failure leaves holds none.
    const std::pair<const char*, const std::string*> order[] = {
        {quality::kResidualsFile, &files.residuals},
        {quality::kReportFile, &files.report},
        {quality::kArrayCalibrationFile, &files.array_calibration}};
    for (const auto& [name, text] : order) {
      const core::Status written =
          write((fs::path(directory) / name).string(), *text);
      if (!written.ok()) return in(written);
    }
    return WrittenResults{number, directory};
  });
}

struct CalibrateWorker::State {
  // The worker's thread's alone.
  int next_number = 0;  // the lowest results number not yet written

  // Shared.
  mutable std::mutex mutex;
  std::condition_variable wake;  // a request, or the stop
  std::shared_ptr<const KeptCaptures> pending;
  std::chrono::system_clock::time_point pressed;
  bool busy = false;  // a run requested and not yet ended
  bool cancelled = false;
  bool stopping = false;
  CalibrateProgress at;  // its seconds counted when read
  Clock::time_point began;
  ResultPtr result;  // newest, not yet taken
  std::vector<core::Status> errors;
  std::thread thread;

  // The run's step: recorded, or the run stopped once cancelled.
  void step(const CalibrateProgress& now) {
    const std::lock_guard<std::mutex> lock(mutex);
    if (cancelled) throw Cancelled();
    at.stage = now.stage;
    at.done = now.done;
    at.total = now.total;
  }

  // One run: its result once written, null when cancelled, or why it has
  // none.
  core::Result<ResultPtr> run_one(const KeptCaptures& kept,
                                  std::chrono::system_clock::time_point when) {
    core::Result<CalibrateResult> made =
        calibrate(kept, [this](const CalibrateProgress& now) { step(now); });
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (cancelled) return ResultPtr();
      if (made.ok()) at = {CalibrateStage::Writing, 0, 0, 0.0};
    }
    if (!made.ok()) return made.status().with_context("Calibrate");
    core::Result<WrittenResults> written =
        write_results(kept.directory, next_number, when, made->files);
    if (!written.ok()) return written.status().with_context("Calibrate");
    // After 999, the next run finds every number taken.
    next_number = std::min(written->number + 1, kMaxNumber);
    made->number = written->number;
    made->directory = std::move(written->directory);
    return ResultPtr(
        std::make_shared<const CalibrateResult>(std::move(made).value()));
  }

  core::Status loop() {
    for (;;) {
      std::shared_ptr<const KeptCaptures> kept;
      std::chrono::system_clock::time_point when;
      {
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait(lock, [this] { return stopping || pending != nullptr; });
        if (stopping) {
          kept = std::move(pending);
          busy = false;
          break;
        }
        kept = std::move(pending);
        when = pressed;
      }
      core::Result<ResultPtr> outcome = compute::guarded(
          "Calibrate",
          [&]() -> core::Result<ResultPtr> { return run_one(*kept, when); });
      kept.reset();  // the snapshot's depth goes before the run is over
      ResultPtr replaced;
      const std::lock_guard<std::mutex> lock(mutex);
      if (!outcome.ok()) {
        errors.push_back(outcome.status());
      } else if (*outcome != nullptr) {
        replaced = std::move(result);
        result = *std::move(outcome);
      }
      busy = false;
    }
    return {};
  }
};

core::Result<std::unique_ptr<CalibrateWorker>> CalibrateWorker::create() {
  return compute::guarded(
      "CalibrateWorker::create",
      [&]() -> core::Result<std::unique_ptr<CalibrateWorker>> {
        std::unique_ptr<CalibrateWorker> worker(
            new CalibrateWorker(std::make_unique<State>()));
        try {
          worker->state_->thread =
              std::thread([w = worker.get()] { w->run(); });
        } catch (const std::system_error& e) {
          return core::Status::io_error(
              std::string("CalibrateWorker: the thread will not start: ") +
              e.what());
        }
        return worker;
      });
}

CalibrateWorker::CalibrateWorker(std::unique_ptr<State> state)
    : state_(std::move(state)) {}

CalibrateWorker::~CalibrateWorker() { stop(); }

void CalibrateWorker::run() noexcept {
  const core::Status ended = compute::guarded(
      "Calibrate", [this]() -> core::Status { return state_->loop(); });
  if (ended.ok()) return;
  const std::lock_guard<std::mutex> lock(state_->mutex);
  state_->errors.push_back(ended);
  state_->busy = false;
}

core::Result<bool> CalibrateWorker::request(
    std::shared_ptr<const KeptCaptures> kept) {
  return compute::guarded(
      "CalibrateWorker::request", [&]() -> core::Result<bool> {
        if (kept == nullptr) {
          return core::Status::invalid_argument(
              "CalibrateWorker::request: no captures kept");
        }
        {
          const std::lock_guard<std::mutex> lock(state_->mutex);
          if (state_->busy || state_->stopping) return false;
          state_->pending = std::move(kept);
          state_->pressed = std::chrono::system_clock::now();
          state_->began = Clock::now();
          state_->at = CalibrateProgress{};
          state_->cancelled = false;
          state_->busy = true;
        }
        state_->wake.notify_all();
        return true;
      });
}

bool CalibrateWorker::busy() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->busy;
}

std::optional<CalibrateProgress> CalibrateWorker::progress() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  if (!state_->busy) return std::nullopt;
  CalibrateProgress at = state_->at;
  at.seconds = seconds_since(state_->began);
  return at;
}

bool CalibrateWorker::cancel() {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  if (!state_->busy || state_->at.stage == CalibrateStage::Writing) {
    return false;
  }
  state_->cancelled = true;
  return true;
}

std::shared_ptr<const CalibrateResult> CalibrateWorker::take_result() {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return std::move(state_->result);
}

std::vector<core::Status> CalibrateWorker::take_errors() {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  std::vector<core::Status> out;
  out.swap(state_->errors);
  return out;
}

void CalibrateWorker::stop() noexcept {
  {
    const std::lock_guard<std::mutex> lock(state_->mutex);
    state_->stopping = true;
    if (state_->busy && state_->at.stage != CalibrateStage::Writing) {
      state_->cancelled = true;
    }
  }
  state_->wake.notify_all();
  if (state_->thread.joinable()) state_->thread.join();
}

}  // namespace volumetric_kit::calib::app
