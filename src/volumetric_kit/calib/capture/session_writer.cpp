// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/capture/session_writer.hpp"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "session_files.hpp"
#include "sha256.hpp"
#include "volumetric_kit/core/base/exception_status.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/io/image_io.hpp"

namespace volumetric_kit::calib::capture {
namespace {

using recon::camera::ArrayCalibration;
using recon::camera::CameraModel;
using Code = core::Status::Code;

core::Status invalid(const std::string& message) {
  return core::Status::invalid_argument("SessionWriter: " + message);
}

bool finite(const std::optional<double>& value) {
  return !value || std::isfinite(*value);
}

bool finite(const recon::camera::Vec2d& p) {
  return std::isfinite(p.x) && std::isfinite(p.y);
}

std::string size_text(std::uint32_t width, std::uint32_t height) {
  return std::to_string(width) + "x" + std::to_string(height);
}

core::Status check_model(const SessionCamera& camera, const char* which,
                         const std::optional<CameraModel>& model,
                         std::uint32_t width, std::uint32_t height) {
  const std::string& serial = camera.info.id;
  if (!model)
    return invalid("camera " + serial + " has no " + which + " model");
  if (model->size.width != width || model->size.height != height) {
    return invalid("camera " + serial + "'s " + which + " model is " +
                   size_text(model->size.width, model->size.height) +
                   ", the streams' " + size_text(width, height));
  }
  return {};
}

core::Status check_streams(const Streams& s) {
  if (s.color_width == 0 || s.color_height == 0 || s.depth_width == 0 ||
      s.depth_height == 0) {
    return invalid("streams' image sizes must be positive");
  }
  if (s.fps == 0) return invalid("streams.fps must be positive");
  if (!(std::isfinite(s.metres_per_unit) && s.metres_per_unit > 0.0f)) {
    return invalid("streams.metres_per_unit must be finite and positive");
  }
  if (!(std::isfinite(s.min_depth_m) && std::isfinite(s.max_depth_m) &&
        s.min_depth_m > 0.0f && s.min_depth_m < s.max_depth_m)) {
    return invalid("streams need finite 0 < min_depth_m < max_depth_m");
  }
  return {};
}

// The session's own checks; the factory calibration's are recon's.
core::Status check_info(const SessionInfo& info) {
  VKC_TRY(core::check_path("SessionWriter: directory", info.directory));
  VKC_TRY(core::check_path("SessionWriter: sync_path", info.sync_path));
  if (core::Status board = check_target(info.board); !board.ok()) {
    return std::move(board).with_context("SessionWriter: board");
  }
  if (info.board.kind != TargetKind::ChArUco) {
    return core::Status::unsupported(
        std::string("SessionWriter: a session's board is ChArUco, not ") +
        to_string(info.board.kind));
  }
  VKC_TRY(check_streams(info.streams));
  if (info.cameras.empty()) return invalid("a session needs a camera");
  bool reference = false;
  for (std::size_t i = 0; i < info.cameras.size(); ++i) {
    const SessionCamera& camera = info.cameras[i];
    const std::string& serial = camera.info.id;
    if (!detail::is_plain_serial(serial)) {
      return invalid("camera serial \"" + serial +
                     "\" is not a plain file name");
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (info.cameras[j].info.id == serial) {
        return invalid("camera " + serial + " is repeated");
      }
    }
    const Streams& s = info.streams;
    VKC_TRY(check_model(camera, "colour", camera.info.color, s.color_width,
                        s.color_height));
    VKC_TRY(check_model(camera, "depth", camera.info.depth, s.depth_width,
                        s.depth_height));
    reference = reference || serial == info.reference;
  }
  if (!reference) {
    return invalid("reference \"" + info.reference + "\" is not a camera");
  }
  return {};
}

// The factory record: every camera's models and depth_to_color, unposed.
core::Result<ArrayCalibration> factory_calibration(const SessionInfo& info) {
  ArrayCalibration array;
  for (const SessionCamera& camera : info.cameras) {
    recon::camera::SensorCalibration sensor;
    sensor.id = camera.info.id;
    sensor.color = recon::camera::CameraCalibration{
        *camera.info.color, recon::camera::IntrinsicsSource::Factory};
    sensor.depth = recon::camera::CameraCalibration{
        *camera.info.depth, recon::camera::IntrinsicsSource::Factory};
    sensor.depth_to_color = camera.info.depth_to_color;
    array.sensors.push_back(std::move(sensor));
  }
  if (core::Status valid = recon::camera::validate_array_calibration(array);
      !valid.ok()) {
    return std::move(valid).with_context("SessionWriter: factory calibration");
  }
  return array;
}

core::Status check_part(const SessionInfo& info, std::size_t i,
                        const CameraCapture& part) {
  const std::string where = "capture: camera " + info.cameras[i].info.id + ": ";
  const Streams& s = info.streams;
  if (part.jpeg.empty()) return invalid(where + "no JPEG");
  const std::size_t samples =
      static_cast<std::size_t>(s.depth_width) * s.depth_height;
  if (part.depth.size() != samples) {
    return invalid(where + std::to_string(part.depth.size()) +
                   " depth samples for a " +
                   size_text(s.depth_width, s.depth_height) + " depth image");
  }
  const recon::camera::ImageSize& size = part.detection.image_size;
  if (size.width != s.color_width || size.height != s.color_height) {
    return invalid(where + "a detection in a " +
                   size_text(size.width, size.height) +
                   " image, not the colour stream's " +
                   size_text(s.color_width, s.color_height));
  }
  for (const DetectedPoint& point : part.detection.observation.points) {
    if (!finite(point.pixel) || !std::isfinite(point.weight)) {
      return invalid(where + "corner " + std::to_string(point.id) +
                     " is not finite");
    }
  }
  for (const DetectedMarker& marker : part.detection.markers) {
    for (const recon::camera::Vec2d& corner : marker.corners) {
      if (!finite(corner)) {
        return invalid(where + "marker " + std::to_string(marker.id) +
                       " is not finite");
      }
    }
  }
  const ViewCheck& check = part.check;
  if (!finite(check.distance_m) || !finite(check.tilt_deg) ||
      !finite(check.pnp_rms_px)) {
    return invalid(where + "a check's measure is not finite");
  }
  if (std::string_view(to_string(check.verdict)) == "unknown") {
    return invalid(where + "check.verdict is not a ViewVerdict");
  }
  return {};
}

core::Status check_job(const SessionInfo& info, const CaptureJob& job) {
  if (job.cameras.size() != info.cameras.size()) {
    return invalid("capture: " + std::to_string(job.cameras.size()) +
                   " parts for " + std::to_string(info.cameras.size()) +
                   " cameras");
  }
  for (std::size_t i = 0; i < job.cameras.size(); ++i) {
    VKC_TRY(check_part(info, i, job.cameras[i]));
  }
  const CaptureMeta& meta = job.meta;
  if (!std::isfinite(meta.spread_ms) || !std::isfinite(meta.stillness_px) ||
      !finite(meta.live_vs_full_px)) {
    return invalid("capture: a set's measure is not finite");
  }
  return {};
}

std::string_view bytes_of(const std::vector<std::uint8_t>& bytes) {
  return {reinterpret_cast<const char*>(bytes.data()), bytes.size()};
}

std::int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace

struct SessionWriter::State {
  struct Request {
    WriteResult::Kind kind = WriteResult::Kind::Capture;
    CaptureJob job;  // for a capture
  };

  State(SessionInfo session, ArrayCalibration factory_record,
        std::size_t queue_capacity)
      : info(std::move(session)),
        factory(std::move(factory_record)),
        capacity(queue_capacity) {
    const std::string& root = info.directory;
    directories.push_back(root);
    for (const char* tree : {detail::kImagesDir, detail::kDepthDir}) {
      const std::string top = detail::join(root, tree);
      directories.push_back(top);
      for (const SessionCamera& camera : info.cameras) {
        directories.push_back(detail::join(top, camera.info.id));
      }
    }
    directories.push_back(detail::join(root, detail::kCapturesDir));
  }

  // --- The writer's thread alone ----------------------------------------

  // The session's directory tree and factory record, at the first capture;
  // a failure leaves what was made, and the next capture goes on from there.
  core::Status prepare() {
    if (sync_sha256.empty()) {
      VKC_ASSIGN(const std::string sync, core::read_file(info.sync_path));
      sync_sha256 = detail::sha256_hex(sync);
    }
    while (made < directories.size()) {
      if (made == 0) {
        std::string stamp = detail::iso_time(now_ns());
        VKC_TRY(core::create_new_directory(directories[0]));
        created = std::move(stamp);
      } else {
        VKC_TRY(core::create_new_directory(directories[made]));
      }
      ++made;
    }
    if (!factory_written) {
      VKC_TRY(recon::camera::write_array_calibration(
          detail::join(info.directory, detail::kFactoryFile), factory,
          core::WriteMode::CreateNew));
      factory_written = true;
    }
    return {};
  }

  core::Status write_session() {
    return core::write_file(
        detail::join(info.directory, detail::kSessionFile),
        detail::format_session(info, created, sync_sha256, captures),
        core::WriteMode::Replace);
  }

  // Lists `entry` in session.json, or leaves it unlisted on any failure.
  core::Status list(detail::CaptureEntry entry) {
    captures.push_back(entry);
    core::Status written;
    try {
      written = write_session();
    } catch (...) {
      captures.pop_back();
      throw;
    }
    if (!written.ok()) captures.pop_back();
    return written;
  }

  core::Result<int> write_capture(CaptureJob& job) {
    VKC_TRY(prepare());
    // Used from here on, whatever happens: files of it may exist.
    const int index = next_index++;
    job.meta.index = index;
    const Streams& s = info.streams;
    for (std::size_t i = 0; i < info.cameras.size(); ++i) {
      const std::string& serial = info.cameras[i].info.id;
      const CameraCapture& part = job.cameras[i];
      VKC_TRY(core::write_file(
          detail::join(info.directory, detail::color_file(serial, index)),
          bytes_of(part.jpeg), core::WriteMode::CreateNew));
      VKC_ASSIGN(const std::vector<std::uint8_t> png,
                 recon::io::encode_png_u16(part.depth.data(), part.depth.size(),
                                           s.depth_width, s.depth_height));
      VKC_TRY(core::write_file(
          detail::join(info.directory, detail::depth_file(serial, index)),
          bytes_of(png), core::WriteMode::CreateNew));
    }
    VKC_TRY(core::write_file(
        detail::join(info.directory, detail::capture_file(index)),
        detail::format_capture(info, job), core::WriteMode::CreateNew));
    VKC_TRY(list({index, false}));
    return index;
  }

  core::Result<int> drop_last() {
    for (auto it = captures.rbegin(); it != captures.rend(); ++it) {
      if (it->dropped) continue;
      it->dropped = true;
      core::Status written;
      try {
        written = write_session();
      } catch (...) {
        it->dropped = false;
        throw;
      }
      if (!written.ok()) {
        it->dropped = false;
        return written;
      }
      return it->index;
    }
    return core::Status::not_found("SessionWriter: no capture to drop");
  }

  WriteResult carry_out(Request& request) noexcept {
    WriteResult result;
    result.kind = request.kind;
    const bool capture = request.kind == WriteResult::Kind::Capture;
    try {
      core::Result<int> done =
          capture ? write_capture(request.job) : drop_last();
      if (done.ok()) {
        result.index = *done;
      } else {
        result.status = done.status();
      }
    } catch (...) {
      result.status = core::exception_status(
          capture ? "SessionWriter: capture" : "SessionWriter: drop",
          Code::IoError);
    }
    return result;
  }

  void run() noexcept {
    for (;;) {
      std::unique_lock<std::mutex> lock(mutex);
      wake.wait(lock, [this] { return stopping || !queue.empty(); });
      if (queue.empty()) return;  // stopping, with every request carried out
      std::optional<Request> request(std::move(queue.front()));
      queue.pop_front();
      lock.unlock();

      WriteResult result = carry_out(*request);
      // The capture's frames go before its slot is freed, so the capacity
      // bounds the memory held.
      const bool capture = request->kind == WriteResult::Kind::Capture;
      request.reset();

      lock.lock();
      try {
        results.push_back(std::move(result));
      } catch (...) {  // out of memory: this outcome is lost
      }
      if (capture) --held;
      --in_flight;
      idle.notify_all();
    }
  }

  // Set at create, then read-only, so any thread reads them.
  const SessionInfo info;
  const ArrayCalibration factory;
  const std::size_t capacity;
  std::vector<std::string> directories;  // the session's, then its tree

  // The writer's thread's alone.
  std::string sync_sha256;
  std::string created;
  std::size_t made = 0;  // directories made
  bool factory_written = false;
  int next_index = 0;
  std::vector<detail::CaptureEntry> captures;

  std::mutex mutex;
  std::condition_variable wake;  // the thread: a request, or stopping
  std::condition_variable idle;  // flush: a request carried out
  // Guarded by `mutex`.
  std::deque<Request> queue;
  std::size_t held = 0;       // captures queued or being written
  std::size_t in_flight = 0;  // requests queued or being carried out
  bool stopping = false;
  std::vector<WriteResult> results;

  std::thread thread;
};

core::Result<std::unique_ptr<SessionWriter>> SessionWriter::create(
    SessionInfo info, SessionWriterOptions options) {
  try {
    if (options.queue_capacity == 0) {
      return invalid("queue_capacity must be at least 1");
    }
    VKC_TRY(check_info(info));
    VKC_ASSIGN(ArrayCalibration factory, factory_calibration(info));
    // The writer first, so that nothing after the thread starts can throw.
    std::unique_ptr<SessionWriter> writer(
        new SessionWriter(std::make_unique<State>(
            std::move(info), std::move(factory), options.queue_capacity)));
    State* running = writer->state_.get();
    try {
      running->thread = std::thread([running] { running->run(); });
    } catch (const std::system_error& e) {
      return core::Status::io_error(
          std::string("SessionWriter: starting the writer's thread: ") +
          e.what());
    }
    return writer;
  } catch (...) {
    return core::exception_status("SessionWriter::create",
                                  Code::InvalidArgument);
  }
}

SessionWriter::SessionWriter(std::unique_ptr<State> state) noexcept
    : state_(std::move(state)) {}

SessionWriter::~SessionWriter() {
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->stopping = true;
  }
  state_->wake.notify_one();
  if (state_->thread.joinable()) state_->thread.join();
}

core::Result<bool> SessionWriter::submit(CaptureJob job) {
  VKC_TRY(check_job(state_->info, job));
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (state_->held >= state_->capacity) return false;
  try {
    state_->queue.push_back({WriteResult::Kind::Capture, std::move(job)});
  } catch (...) {
    return core::exception_status("SessionWriter::submit", Code::OutOfMemory);
  }
  ++state_->held;
  ++state_->in_flight;
  state_->wake.notify_one();
  return true;
}

core::Status SessionWriter::drop_last() {
  std::lock_guard<std::mutex> lock(state_->mutex);
  try {
    state_->queue.push_back({WriteResult::Kind::Drop, {}});
  } catch (...) {
    return core::exception_status("SessionWriter::drop_last",
                                  Code::OutOfMemory);
  }
  ++state_->in_flight;
  state_->wake.notify_one();
  return {};
}

std::vector<WriteResult> SessionWriter::take_results() {
  std::vector<WriteResult> taken;
  std::lock_guard<std::mutex> lock(state_->mutex);
  taken.swap(state_->results);
  return taken;
}

std::size_t SessionWriter::pending() const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->held;
}

void SessionWriter::flush() {
  std::unique_lock<std::mutex> lock(state_->mutex);
  state_->idle.wait(lock, [this] { return state_->in_flight == 0; });
}

core::Result<std::string> session_stamp(
    std::chrono::system_clock::time_point time) {
  const std::time_t seconds = std::chrono::system_clock::to_time_t(time);
  std::tm local{};
  if (::localtime_r(&seconds, &local) == nullptr) {
    return core::Status::invalid_argument(
        "session_stamp: the time has no local calendar date");
  }
  char text[32];
  if (std::strftime(text, sizeof text, "%Y-%m-%d_%H%M%S", &local) == 0) {
    return core::Status::invalid_argument(
        "session_stamp: the date does not fit the name");
  }
  return std::string(text);
}

}  // namespace volumetric_kit::calib::capture
