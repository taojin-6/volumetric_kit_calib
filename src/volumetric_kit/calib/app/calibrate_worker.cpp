// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/calibrate_worker.hpp"

#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>

#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/depth/depth_checks.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"

namespace volumetric_kit::calib::app {

namespace {

namespace camera = recon::camera;
namespace fs = std::filesystem;
namespace sensor = recon::sensor;
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

// What the depth checks share over a Calibrate's captures: the solved rig,
// the views to fit, and what is measured so far.
struct DepthRun {
  const KeptCaptures& kept;
  const compute::Board& board;
  const std::vector<Observation>& views;  // as the solve took them
  std::vector<quality::DepthView> fitted;
  // Each camera's depth in the solve's world, and beside its colour camera.
  std::vector<camera::Mat4d> depth_to_world;
  std::vector<camera::Mat4d> depth_to_color;
  std::unique_ptr<depth::AgreementHistograms> pooled;
  std::vector<quality::BoardDepthCheck> checked;
  std::vector<quality::PairAgreement> pairs;
};

// One kept capture's depth checks: its cameras' raw depth prepared in one
// batch, its views' planes fitted and their corners met with them, and its
// pairs added to the pooled histograms and to its own. Its frames are let go
// as it returns.
core::Status check_capture(std::vector<sensor::GpuFramePrep>& preps,
                           depth::DepthChecks& checks, DepthRun& run,
                           const KeptCapture& capture) {
  const KeptCaptures& kept = run.kept;
  const std::size_t n = kept.cameras.size();
  const quality::DepthCheckOptions options;
  if (capture.depth.size() != n) {
    return core::Status::invalid_argument(std::to_string(capture.depth.size()) +
                                          " depth images for " +
                                          std::to_string(n) + " cameras");
  }
  std::vector<std::optional<sensor::RgbdFrame>> frames(n);
  for (std::size_t c = 0; c < n; ++c) {
    const std::shared_ptr<const std::vector<std::uint16_t>>& raw =
        capture.depth[c];
    const std::optional<camera::CameraModel>& lens = kept.sensors[c].depth;
    if (raw == nullptr || raw->empty() || !lens) continue;
    const std::size_t pixels =
        std::size_t{lens->size.width} * lens->size.height;
    if (raw->size() != pixels) {
      return core::Status::invalid_argument(
          "camera " + kept.cameras[c].id + "'s depth has " +
          std::to_string(raw->size()) + " samples for its " +
          std::to_string(lens->size.width) + " x " +
          std::to_string(lens->size.height) + " depth camera");
    }
    // Read where the capture keeps it; at the colour camera's origin, as
    // the pose comes from the solve (run.depth_to_world).
    sensor::RgbdFrame& frame = frames[c].emplace();
    frame.depth = raw->data();
    frame.pixels = raw;
    frame.metres_per_unit = kept.depth_mode.metres_per_unit;
    frame.depth_camera = *lens;
    frame.min_depth = kept.depth_mode.min_m;
    frame.max_depth = kept.depth_mode.max_m;
    frame.depth_to_color = run.depth_to_color[c];
  }
  VKC_ASSIGN(const std::vector<std::optional<sensor::DeviceFrame>> prepared,
             sensor::GpuFramePrep::prepare_batch(preps, frames));
  std::vector<std::optional<depth::DepthImage>> images(n);
  for (std::size_t c = 0; c < n; ++c) {
    if (!prepared[c]) continue;
    const recon::DepthCameraParams& k = prepared[c]->depth_camera;
    depth::DepthImage& image = images[c].emplace();
    image.depth = prepared[c]->depth.get();
    image.intrinsics = {k.fx, k.fy, k.cx, k.cy};
    image.size = {k.width, k.height};
    image.min_depth_m = k.min_depth;
    image.max_depth_m = k.max_depth;
    image.depth_to_world = run.depth_to_world[c];
    image.depth_to_color = run.depth_to_color[c];
  }

  std::vector<depth::PlaneRegion> regions;
  std::vector<const quality::DepthView*> fitted;
  for (const quality::DepthView& view : run.fitted) {
    if (view.capture != capture.index || !images[view.camera]) continue;
    regions.push_back({*images[view.camera], view.region});
    fitted.push_back(&view);
  }
  VKC_ASSIGN(const std::vector<quality::DepthPlane> planes,
             checks.fit_planes(regions, options));
  for (std::size_t i = 0; i < fitted.size(); ++i) {
    VKC_ASSIGN(quality::BoardDepthCheck check,
               quality::board_depth_check(kept.cameras, run.board,
                                          run.views[fitted[i]->view],
                                          fitted[i]->region, planes[i]));
    run.checked.push_back(std::move(check));
  }
  VKC_TRY(checks.accumulate(*run.pooled, images));
  VKC_ASSIGN(const std::unique_ptr<depth::AgreementHistograms> own,
             checks.histograms(n, options));
  VKC_TRY(checks.accumulate(*own, images));
  VKC_ASSIGN(const std::vector<quality::PairAgreement> pairs,
             checks.summarize(*own, capture.index));
  run.pairs.insert(run.pairs.end(), pairs.begin(), pairs.end());
  return {};
}

// The depth checks of every kept capture against the solved rig
// (CalibrateDepth), each colour camera at `color_to_world`, in rig order.
// `step` is called with the captures checked, before each and after the last.
core::Result<quality::DepthSummary> check_depth(
    std::vector<sensor::GpuFramePrep>& preps, depth::DepthChecks& checks,
    const KeptCaptures& kept, const compute::Board& board,
    const std::vector<Observation>& views, const solve::InitialRig& start,
    const solve::RigSolution& solution,
    const std::vector<camera::Mat4d>& color_to_world,
    const std::function<void(std::size_t)>& step) {
  const std::size_t n = kept.cameras.size();
  const quality::DepthCheckOptions options;
  if (preps.size() != n || kept.sensors.size() != n) {
    return core::Status::invalid_argument(
        "checks for " + std::to_string(preps.size()) + " cameras and " +
        std::to_string(kept.sensors.size()) + " sensors for a rig of " +
        std::to_string(n));
  }
  DepthRun run{kept, board, views, {}, {}, {}, nullptr, {}, {}};
  for (std::size_t c = 0; c < n; ++c) {
    const camera::Mat4d& depth_to_color = kept.sensors[c].depth_to_color;
    run.depth_to_color.push_back(depth_to_color);
    run.depth_to_world.push_back(color_to_world[c] * depth_to_color);
  }
  VKC_ASSIGN(run.fitted, quality::depth_views(board, start, solution,
                                              run.depth_to_color, options));
  VKC_ASSIGN(run.pooled, checks.histograms(n, options));
  const std::size_t captures = kept.captures.size();
  for (std::size_t k = 0; k < captures; ++k) {
    step(k);
    const KeptCapture& capture = *kept.captures[k];
    VKC_TRY(check_capture(preps, checks, run, capture)
                .with_context("capture " + std::to_string(capture.index)));
  }
  step(captures);
  VKC_ASSIGN(const std::vector<quality::PairAgreement> pooled,
             checks.summarize(*run.pooled));
  run.pairs.insert(run.pairs.end(), pooled.begin(), pooled.end());
  return quality::summarize_depth_checks(n, run.checked, run.pairs, options);
}

}  // namespace

const char* to_string(CalibrateStage stage) noexcept {
  switch (stage) {
    case CalibrateStage::Solving:
      return "solving";
    case CalibrateStage::Measuring:
      return "measuring";
    case CalibrateStage::Checking:
      return "checking depth";
    case CalibrateStage::Writing:
      return "writing";
  }
  return "unknown";
}

struct CalibrateDepth::State {
  std::vector<sensor::GpuFramePrep> preps;  // one per camera
  std::unique_ptr<depth::DepthChecks> checks;
};

core::Result<std::unique_ptr<CalibrateDepth>> CalibrateDepth::create(
    core::Device& device, core::Allocator& allocator, std::size_t cameras) {
  constexpr const char* kWhat = "CalibrateDepth::create";
  return compute::guarded(
      kWhat, [&]() -> core::Result<std::unique_ptr<CalibrateDepth>> {
        if (cameras == 0) {
          return core::Status::invalid_argument(std::string(kWhat) +
                                                ": no camera");
        }
        auto state = std::make_unique<State>();
        state->preps.reserve(cameras);
        for (std::size_t c = 0; c < cameras; ++c) {
          core::Result<sensor::GpuFramePrep> prep =
              sensor::GpuFramePrep::create(device, allocator);
          if (!prep.ok()) return prep.status().with_context(kWhat);
          state->preps.push_back(*std::move(prep));
        }
        core::Result<std::unique_ptr<depth::DepthChecks>> checks =
            depth::DepthChecks::create(device, allocator);
        if (!checks.ok()) return checks.status().with_context(kWhat);
        state->checks = *std::move(checks);
        return std::unique_ptr<CalibrateDepth>(
            new CalibrateDepth(std::move(state)));
      });
}

CalibrateDepth::CalibrateDepth(std::unique_ptr<State> state) noexcept
    : state_(std::move(state)) {}

CalibrateDepth::~CalibrateDepth() = default;

std::size_t CalibrateDepth::cameras() const noexcept {
  return state_->preps.size();
}

core::Result<CalibrateResult> calibrate(const KeptCaptures& kept,
                                        const CalibrateCallback& progress,
                                        CalibrateDepth* depth) {
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
    if (depth != nullptr) {
      // A throw from the progress (a stop) stops the run; what the checks
      // throw is their failure.
      std::exception_ptr stopped;
      const auto step = [&](std::size_t done) {
        try {
          report(CalibrateStage::Checking, done, out.captures);
        } catch (...) {
          stopped = std::current_exception();
          throw;
        }
      };
      constexpr const char* kChecks = "depth checks";
      core::Result<quality::DepthSummary> checked = compute::guarded(
          kChecks, [&]() -> core::Result<quality::DepthSummary> {
            return check_depth(depth->state_->preps, *depth->state_->checks,
                               kept, *board, *views, *start, *solution,
                               out.color_to_world, step);
          });
      if (stopped) std::rethrow_exception(stopped);
      if (checked.ok()) {
        out.depth = *std::move(checked);
      } else {
        out.depth_failure = checked.status().with_context(kChecks);
      }
    }
    core::Result<quality::ResultFiles> files = quality::format_results(
        kept.cameras, *solution, *metrics, out.depth ? &*out.depth : nullptr,
        out.depth_failure.ok() ? std::string() : out.depth_failure.message());
    if (!files.ok()) return in(files.status());
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
  // Set before the thread starts.
  ResultSink sink;  // each results file's writer; empty for the default

  // The worker's thread's alone.
  std::unique_ptr<CalibrateDepth> depth;  // null checks no depth
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
    core::Result<CalibrateResult> made = calibrate(
        kept, [this](const CalibrateProgress& now) { step(now); }, depth.get());
    {
      // A success is decided here: once writing, cancel() is refused. Its
      // depth checks' failure is posted, and it is written without them.
      const std::lock_guard<std::mutex> lock(mutex);
      if (cancelled) return ResultPtr();
      if (made.ok()) {
        at = {CalibrateStage::Writing, 0, 0, 0.0};
        if (!made->depth_failure.ok()) {
          errors.push_back(made->depth_failure.with_context("Calibrate"));
        }
      }
    }
    if (!made.ok()) return made.status().with_context("Calibrate");
    core::Result<WrittenResults> written =
        write_results(kept.directory, next_number, when, made->files, sink);
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
        // A failure is decided here, under the lock cancel() takes: one
        // cancelled since run_one looked (the snapshot freed meanwhile) was
        // told it cancelled the run, so the failure is not posted.
        if (!cancelled) errors.push_back(outcome.status());
      } else if (*outcome != nullptr) {
        replaced = std::move(result);
        result = *std::move(outcome);
      }
      busy = false;
    }
    return {};
  }
};

core::Result<std::unique_ptr<CalibrateWorker>> CalibrateWorker::create(
    std::unique_ptr<CalibrateDepth> depth, ResultSink sink) {
  return compute::guarded(
      "CalibrateWorker::create",
      [&]() -> core::Result<std::unique_ptr<CalibrateWorker>> {
        auto state = std::make_unique<State>();
        state->sink = std::move(sink);
        state->depth = std::move(depth);
        std::unique_ptr<CalibrateWorker> worker(
            new CalibrateWorker(std::move(state)));
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
    std::shared_ptr<const KeptCaptures> kept,
    std::chrono::system_clock::time_point pressed) {
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
          state_->pressed = pressed;
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
