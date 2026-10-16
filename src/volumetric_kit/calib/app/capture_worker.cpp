// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/capture_worker.hpp"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>

#include <opencv2/core/utility.hpp>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/detect/charuco.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sensor.hpp"

namespace volumetric_kit::calib::app {

namespace {

namespace sensor = recon::sensor;

// How often the worker looks for the writer's outcomes while nothing is
// queued.
constexpr std::chrono::milliseconds kCollectEvery{20};

// A capture on the writer, awaiting its outcome.
struct Written {
  std::vector<Detection> full;  // each camera's, for the session's checks
  CaptureEvent event;           // Saved, once its index is known
};

// A capture, or a drop of the last one.
struct Job {
  bool drop = false;
  CaptureRequest request;
};

}  // namespace

core::Result<capture::SessionCamera> session_camera(
    const sensor::IRgbdSensor& camera,
    const sensor::OrbbecRigSyncConfig& sync) {
  return compute::guarded(
      "session_camera", [&]() -> core::Result<capture::SessionCamera> {
        capture::SessionCamera out;
        out.info = camera.info();
        const auto* orbbec = dynamic_cast<const sensor::OrbbecSensor*>(&camera);
        if (orbbec != nullptr) {
          const sensor::OrbbecDeviceInfo& device = orbbec->device_info();
          out.firmware = device.firmware_version;
          out.ip_address = device.ip_address;
          out.sync_mode = sensor::to_string(device.sync_mode);
          out.color_lens_model = device.color_lens_model;
          out.depth_lens_model = device.depth_lens_model;
        }
        for (const sensor::OrbbecSyncDevice& entry : sync.devices) {
          if (entry.serial == out.info.id) {
            out.depth_delay_us = entry.sync.depth_delay_us;
          }
        }
        return out;
      });
}

struct CaptureWorker::State {
  CaptureSetup setup;
  std::vector<compute::CvCamera> cameras;  // each camera's colour lens

  // The worker's thread's alone.
  std::unique_ptr<capture::SessionWriter> writer;
  std::optional<compute::Board> board;
  std::optional<quality::SessionChecks> checks;
  std::deque<Written> written;  // on the writer, in the order submitted

  // Shared.
  mutable std::mutex mutex;
  std::condition_variable wake;  // a job, or the stop
  std::condition_variable idle;  // an outcome
  std::deque<Job> jobs;
  bool busy = false;         // a job is being carried out
  std::size_t captures = 0;  // captures queued and not yet resolved
  std::size_t drops = 0;     // drops queued and not yet resolved
  bool stopping = false;
  std::vector<CaptureEvent> events;
  std::optional<quality::SessionSummary> summary;
  std::string directory;
  std::vector<core::Status> errors;
  std::thread thread;

  // Report outcomes, the captures and drops they resolve, and errors.
  void resolve(std::vector<CaptureEvent> outcomes, std::size_t captures_done,
               std::size_t drops_done, std::vector<core::Status> failures,
               std::optional<quality::SessionSummary> latest = {}) {
    const std::lock_guard<std::mutex> lock(mutex);
    for (CaptureEvent& event : outcomes) events.push_back(std::move(event));
    for (core::Status& failure : failures) errors.push_back(std::move(failure));
    captures -= captures_done;
    drops -= drops_done;
    if (latest) summary = std::move(latest);
    idle.notify_all();
  }

  void fail_capture(const CaptureMeta& meta, core::Status status) {
    CaptureEvent event;
    event.kind = CaptureEvent::Kind::Failed;
    event.status = std::move(status);
    event.meta = meta;
    event.meta.index = -1;
    resolve({std::move(event)}, 1, 0, {});
  }

  // The session's writer and checks, at its first capture, for its board.
  core::Status open_session(const CaptureRequest& request) {
    VKC_ASSIGN(compute::Board made, compute::Board::create(request.board));
    std::vector<solve::RigCamera> rig;
    for (const capture::SessionCamera& camera : setup.cameras) {
      rig.push_back({camera.info.id, *camera.info.color});
    }
    VKC_ASSIGN(quality::SessionChecks session,
               quality::SessionChecks::create(rig, made, setup.reference,
                                              setup.session));
    VKC_ASSIGN(const std::string stamp,
               capture::session_stamp(std::chrono::system_clock::now()));
    capture::SessionInfo info;
    info.directory = (std::filesystem::path(setup.root) / stamp).string();
    info.software = setup.software;
    info.sync_path = setup.sync_path;
    info.reference = setup.cameras[setup.reference].info.id;
    info.board = request.board;
    info.square_measured = request.square_measured;
    info.depth_mode = setup.depth_mode;
    info.cameras = setup.cameras;
    const std::string directory_name = info.directory;
    capture::SessionWriterOptions options;
    options.queue_capacity = setup.capacity;
    VKC_ASSIGN(writer,
               capture::SessionWriter::create(std::move(info), options));
    board = std::move(made);
    checks = std::move(session);
    const std::lock_guard<std::mutex> lock(mutex);
    directory = directory_name;
    return {};
  }

  // One capture: detected at full resolution, checked, queued on the writer.
  void capture(CaptureRequest request) {
    if (writer == nullptr) {
      const core::Status opened = open_session(request);
      if (!opened.ok()) {
        fail_capture(request.meta, opened.with_context("session"));
        return;
      }
    }
    const std::size_t n = request.cameras.size();
    std::vector<Detection> full(n);
    std::vector<ViewCheck> views(n);
    std::vector<core::Status> detect_errors(n);
    std::vector<core::Status> check_errors(n);
    cv::parallel_for_(cv::Range(0, static_cast<int>(n)),
                      [&](const cv::Range& range) {
                        for (int c = range.start; c < range.end; ++c) {
                          const auto i = static_cast<std::size_t>(c);
                          detect_one(i, request.cameras[i], full[i], views[i],
                                     detect_errors[i], check_errors[i]);
                        }
                      });
    std::vector<core::Status> failures;
    for (std::size_t i = 0; i < n; ++i) {
      const std::string who = "capture: camera " + setup.cameras[i].info.id;
      if (!detect_errors[i].ok()) {
        failures.push_back(detect_errors[i].with_context(who));
      } else if (!check_errors[i].ok()) {
        failures.push_back(check_errors[i].with_context(who));
      }
    }

    std::vector<Detection> live(n);
    for (std::size_t i = 0; i < n; ++i) live[i] = request.cameras[i].live;
    CaptureEvent event;
    event.kind = CaptureEvent::Kind::Saved;
    event.meta = request.meta;
    const core::Result<quality::CornerShift> shift =
        quality::corner_shift(live, full);
    if (shift.ok()) {
      event.meta.live_vs_full_px = shift.value().median_px;
      event.flagged =
          quality::live_vs_full_flagged(shift.value(), setup.live_vs_full);
    }
    for (const ViewCheck& view : views) {
      if (is_usable(view.verdict)) ++event.usable_views;
    }

    capture::CaptureJob job;
    job.meta = event.meta;
    job.cameras.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      CameraFrameCopy& part = request.cameras[i];
      capture::CameraCapture& out = job.cameras[i];
      out.jpeg = std::move(part.jpeg);
      out.depth = std::move(part.depth);
      out.timestamp_ns = part.timestamp_ns;
      out.sequence = part.sequence;
      out.detection = full[i];
      out.check = views[i];
    }
    if (!failures.empty()) resolve({}, 0, 0, std::move(failures));
    const core::Result<bool> queued = writer->submit(std::move(job));
    if (!queued.ok()) {
      fail_capture(event.meta, queued.status().with_context("capture"));
      return;
    }
    if (!queued.value()) {
      fail_capture(event.meta,
                   core::Status::io_error(
                       "capture refused: the session writer is full"));
      return;
    }
    written.push_back({std::move(full), std::move(event)});
  }

  void detect_one(std::size_t i, const CameraFrameCopy& part, Detection& found,
                  ViewCheck& view, core::Status& detect_error,
                  core::Status& check_error) const {
    const recon::camera::ImageSize size{part.width, part.height};
    detect_error = compute::guarded("full detection", [&]() -> core::Status {
      const detect::LumaView luma{part.width, part.height, part.width,
                                  part.luma.data()};
      VKC_ASSIGN(found,
                 detect::detect_charuco(luma, *board, detect::Mode::Full));
      found.observation.camera_id = setup.cameras[i].info.id;
      return {};
    });
    if (!detect_error.ok()) {
      found = Detection{};
      found.image_size = size;
      return;
    }
    check_error = compute::guarded("full check", [&]() -> core::Status {
      VKC_ASSIGN(
          const quality::CheckedView checked,
          quality::check_view(cameras[i], *board, found, setup.session.view));
      view = checked.check;
      return {};
    });
    if (!check_error.ok()) view = ViewCheck{};
  }

  void drop() {
    if (writer == nullptr) {
      CaptureEvent event;
      event.status = core::Status::not_found("drop: no capture to drop");
      event.meta.index = -1;
      resolve({std::move(event)}, 0, 1, {});
      return;
    }
    const core::Status queued = writer->drop_last();
    if (!queued.ok()) {
      CaptureEvent event;
      event.status = queued.with_context("drop");
      event.meta.index = -1;
      resolve({std::move(event)}, 0, 1, {});
    }
  }

  // The writer's outcomes since the last look, and the session's checks.
  void collect() {
    if (writer == nullptr) return;
    std::vector<capture::WriteResult> results = writer->take_results();
    if (results.empty()) return;
    std::vector<CaptureEvent> outcomes;
    std::vector<core::Status> failures;
    std::size_t captures_done = 0;
    std::size_t drops_done = 0;
    for (capture::WriteResult& result : results) {
      if (result.kind == capture::WriteResult::Kind::Capture) {
        ++captures_done;
        Written done = std::move(written.front());
        written.pop_front();
        if (!result.status.ok()) {
          done.event.kind = CaptureEvent::Kind::Failed;
          done.event.status = std::move(result.status);
          done.event.meta.index = -1;
        } else {
          done.event.meta.index = result.index;
          const core::Status added = checks->add(result.index, done.full);
          if (!added.ok()) failures.push_back(added.with_context("session"));
        }
        outcomes.push_back(std::move(done.event));
        continue;
      }
      ++drops_done;
      CaptureEvent event;
      event.meta.index = result.index;
      if (!result.status.ok()) {
        event.status = std::move(result.status).with_context("drop");
      } else {
        event.kind = CaptureEvent::Kind::Dropped;
        const core::Status dropped = checks->drop(result.index);
        if (!dropped.ok()) failures.push_back(dropped.with_context("session"));
      }
      outcomes.push_back(std::move(event));
    }
    core::Result<quality::SessionSummary> latest = checks->summary();
    std::optional<quality::SessionSummary> summary_now;
    if (latest.ok()) {
      summary_now = std::move(latest).value();
    } else {
      failures.push_back(latest.status().with_context("session"));
    }
    resolve(std::move(outcomes), captures_done, drops_done, std::move(failures),
            std::move(summary_now));
  }

  core::Status loop() {
    for (;;) {
      std::optional<Job> job;
      {
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait_for(lock, kCollectEvery,
                      [&] { return stopping || !jobs.empty(); });
        if (!jobs.empty()) {
          job = std::move(jobs.front());
          jobs.pop_front();
          busy = true;
        } else if (stopping) {
          break;
        }
      }
      if (job) {
        if (job->drop) {
          drop();
        } else {
          capture(std::move(job->request));
        }
        const std::lock_guard<std::mutex> lock(mutex);
        busy = false;
        idle.notify_all();
      }
      collect();
    }
    if (writer != nullptr) {
      writer->flush();
      collect();
    }
    return {};
  }
};

core::Result<std::unique_ptr<CaptureWorker>> CaptureWorker::create(
    CaptureSetup setup) {
  return compute::guarded(
      "CaptureWorker::create",
      [&]() -> core::Result<std::unique_ptr<CaptureWorker>> {
        if (setup.root.empty()) {
          return core::Status::invalid_argument(
              "CaptureWorker: no directory to create sessions in");
        }
        if (setup.cameras.empty()) {
          return core::Status::invalid_argument("CaptureWorker: no cameras");
        }
        if (setup.reference >= setup.cameras.size()) {
          return core::Status::invalid_argument(
              "CaptureWorker: the reference is not a camera");
        }
        if (setup.capacity == 0) {
          return core::Status::invalid_argument(
              "CaptureWorker: a capacity of 0 holds no capture");
        }
        auto state = std::make_unique<State>();
        for (const capture::SessionCamera& camera : setup.cameras) {
          if (!camera.info.color) {
            return core::Status::invalid_argument("CaptureWorker: camera " +
                                                  camera.info.id +
                                                  " has no colour model");
          }
          VKC_ASSIGN(compute::CvCamera cv,
                     compute::cv_camera(*camera.info.color));
          state->cameras.push_back(std::move(cv));
        }
        const std::size_t capacity = setup.capacity;
        state->setup = std::move(setup);
        std::unique_ptr<CaptureWorker> worker(
            new CaptureWorker(std::move(state), capacity));
        try {
          worker->state_->thread =
              std::thread([w = worker.get()] { w->run(); });
        } catch (const std::system_error& e) {
          return core::Status::io_error(
              std::string("CaptureWorker: the thread will not start: ") +
              e.what());
        }
        return worker;
      });
}

CaptureWorker::CaptureWorker(std::unique_ptr<State> state, std::size_t capacity)
    : state_(std::move(state)), capacity_(capacity) {}

CaptureWorker::~CaptureWorker() {
  {
    const std::lock_guard<std::mutex> lock(state_->mutex);
    state_->stopping = true;
  }
  state_->wake.notify_all();
  if (state_->thread.joinable()) state_->thread.join();
}

void CaptureWorker::run() noexcept {
  const core::Status ended = compute::guarded(
      "capture worker", [this]() -> core::Status { return state_->loop(); });
  if (!ended.ok()) state_->resolve({}, 0, 0, {ended});
}

bool CaptureWorker::full() const { return pending() >= capacity_; }

core::Result<bool> CaptureWorker::submit(CaptureRequest request) {
  return compute::guarded("CaptureWorker::submit", [&]() -> core::Result<bool> {
    const std::vector<capture::SessionCamera>& cameras = state_->setup.cameras;
    if (request.cameras.size() != cameras.size()) {
      return core::Status::invalid_argument(
          "capture: " + std::to_string(request.cameras.size()) +
          " cameras' frames for a rig of " + std::to_string(cameras.size()));
    }
    for (std::size_t i = 0; i < cameras.size(); ++i) {
      const CameraFrameCopy& part = request.cameras[i];
      const recon::camera::ImageSize& size = cameras[i].info.color->size;
      if (part.width != size.width || part.height != size.height ||
          part.luma.size() !=
              static_cast<std::size_t>(part.width) * part.height) {
        return core::Status::invalid_argument(
            "capture: camera " + cameras[i].info.id +
            "'s luma is not its colour image's size");
      }
    }
    {
      const std::lock_guard<std::mutex> lock(state_->mutex);
      if (state_->captures >= capacity_) return false;
      state_->jobs.push_back({false, std::move(request)});
      ++state_->captures;
    }
    state_->wake.notify_all();
    return true;
  });
}

core::Status CaptureWorker::drop_last() {
  return compute::guarded("CaptureWorker::drop_last", [&]() -> core::Status {
    {
      const std::lock_guard<std::mutex> lock(state_->mutex);
      state_->jobs.push_back({true, {}});
      ++state_->drops;
    }
    state_->wake.notify_all();
    return {};
  });
}

std::vector<CaptureEvent> CaptureWorker::take_events() {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  std::vector<CaptureEvent> out;
  out.swap(state_->events);
  return out;
}

std::optional<quality::SessionSummary> CaptureWorker::summary() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->summary;
}

std::size_t CaptureWorker::pending() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->captures;
}

std::string CaptureWorker::directory() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->directory;
}

std::vector<core::Status> CaptureWorker::take_errors() {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  std::vector<core::Status> out;
  out.swap(state_->errors);
  return out;
}

void CaptureWorker::flush() {
  std::unique_lock<std::mutex> lock(state_->mutex);
  state_->idle.wait(lock, [&] {
    return state_->jobs.empty() && !state_->busy && state_->captures == 0 &&
           state_->drops == 0;
  });
}

}  // namespace volumetric_kit::calib::app
