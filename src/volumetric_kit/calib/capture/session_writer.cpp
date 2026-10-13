// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/capture/session_writer.hpp"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <deque>
#include <iterator>
#include <list>
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

std::string size_text(const recon::camera::ImageSize& size) {
  return std::to_string(size.width) + "x" + std::to_string(size.height);
}

// Serials name directories, and some file systems ignore case; plain serials
// are ASCII.
bool same_ignoring_case(const std::string& a, const std::string& b) noexcept {
  if (a.size() != b.size()) return false;
  const auto fold = [](char ch) {
    return ch >= 'A' && ch <= 'Z' ? static_cast<char>(ch - 'A' + 'a') : ch;
  };
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (fold(a[i]) != fold(b[i])) return false;
  }
  return true;
}

// Every camera streams the mode the first does.
core::Status check_same_size(const SessionCamera& camera,
                             const SessionCamera& first, const char* which,
                             const CameraModel& model,
                             const CameraModel& first_model) {
  if (model.size.width == first_model.size.width &&
      model.size.height == first_model.size.height) {
    return {};
  }
  return invalid("camera " + camera.info.id + "'s " + which + " model is " +
                 size_text(model.size) + ", camera " + first.info.id + "'s " +
                 size_text(first_model.size));
}

core::Status check_depth_mode(const DepthMode& d) {
  if (!(std::isfinite(d.metres_per_unit) && d.metres_per_unit > 0.0f)) {
    return invalid("depth_mode.metres_per_unit must be finite and positive");
  }
  if (!(std::isfinite(d.min_m) && std::isfinite(d.max_m) && d.min_m > 0.0f &&
        d.min_m < d.max_m)) {
    return invalid("depth_mode needs finite 0 < min_m < max_m");
  }
  return {};
}

// The factory record: every camera's models and depth_to_color, unposed,
// which recon checks: a camera at least, no id twice, models it can hold.
// @pre Every camera has both models.
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

// The session's checks, around recon's of its factory record, which it
// returns.
core::Result<ArrayCalibration> check_info(const SessionInfo& info) {
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
  VKC_TRY(check_depth_mode(info.depth_mode));
  for (const SessionCamera& camera : info.cameras) {
    const std::string& serial = camera.info.id;
    if (!detail::is_plain_serial(serial)) {
      return invalid("camera serial \"" + serial +
                     "\" is not a plain file name");
    }
    if (!camera.info.color) {
      return invalid("camera " + serial + " has no colour model");
    }
    if (!camera.info.depth) {
      return invalid("camera " + serial + " has no depth model");
    }
  }
  VKC_ASSIGN(ArrayCalibration factory, factory_calibration(info));
  // recon found a camera at least, and no serial twice.
  const SessionCamera& first = info.cameras.front();
  bool reference = false;
  for (std::size_t i = 0; i < info.cameras.size(); ++i) {
    const SessionCamera& camera = info.cameras[i];
    const std::string& serial = camera.info.id;
    for (std::size_t j = 0; j < i; ++j) {
      if (same_ignoring_case(info.cameras[j].info.id, serial)) {
        return invalid("cameras " + info.cameras[j].info.id + " and " + serial +
                       " differ only in case");
      }
    }
    VKC_TRY(check_same_size(camera, first, "colour", *camera.info.color,
                            *first.info.color));
    VKC_TRY(check_same_size(camera, first, "depth", *camera.info.depth,
                            *first.info.depth));
    if (camera.info.fps == 0) {
      return invalid("camera " + serial + "'s fps is unknown");
    }
    if (camera.info.fps != first.info.fps) {
      return invalid("camera " + serial + " streams at " +
                     std::to_string(camera.info.fps) + " fps, camera " +
                     first.info.id + " at " + std::to_string(first.info.fps));
    }
    reference = reference || serial == info.reference;
  }
  if (!reference) {
    return invalid("reference \"" + info.reference + "\" is not a camera");
  }
  return factory;
}

// @pre `info` passed check_info.
core::Status check_part(const SessionInfo& info, std::size_t i,
                        const CameraCapture& part) {
  const recon::sensor::SensorInfo& camera = info.cameras[i].info;
  const std::string where = "capture: camera " + camera.id + ": ";
  // A JPEG starts with its SOI marker.
  if (part.jpeg.size() < 2 || part.jpeg[0] != 0xFF || part.jpeg[1] != 0xD8) {
    return invalid(where + "the colour is not a JPEG");
  }
  const recon::camera::ImageSize& depth = camera.depth->size;
  const std::size_t samples =
      static_cast<std::size_t>(depth.width) * depth.height;
  if (part.depth.size() != samples) {
    return invalid(where + std::to_string(part.depth.size()) +
                   " depth samples for a " + size_text(depth) + " depth image");
  }
  const recon::camera::ImageSize& size = part.detection.image_size;
  const recon::camera::ImageSize& color = camera.color->size;
  if (size.width != color.width || size.height != color.height) {
    return invalid(where + "a detection in a " + size_text(size) +
                   " image, not the colour image's " + size_text(color));
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
  // A capture as a drop finds it: its entry in `captures`, or none when it
  // was not written.
  using Outcome = std::optional<std::size_t>;

  struct Request {
    WriteResult::Kind kind = WriteResult::Kind::Capture;
    CaptureJob job;  // a capture's
    // A capture's node for `takeable`, allocated when it is submitted, so
    // the thread records what became of it without allocating.
    std::list<Outcome> outcome;
  };

  State(SessionInfo session, ArrayCalibration factory_record,
        std::size_t queue_capacity) noexcept
      : info(std::move(session)),
        factory(std::move(factory_record)),
        capacity(queue_capacity) {}

  // --- The writer's thread alone ----------------------------------------

  // The session's directory, then its tree.
  std::vector<std::string> tree() const {
    const std::string& root = info.directory;
    std::vector<std::string> paths = {root};
    for (const char* name : {detail::kImagesDir, detail::kDepthDir}) {
      const std::string top = detail::join(root, name);
      paths.push_back(top);
      for (const SessionCamera& camera : info.cameras) {
        paths.push_back(detail::join(top, camera.info.id));
      }
    }
    paths.push_back(detail::join(root, detail::kCapturesDir));
    return paths;
  }

  // The session's directory tree and factory record, at the first capture;
  // a failure leaves what was made, and the next capture goes on from there.
  core::Status prepare() {
    if (sync_sha256.empty()) {
      VKC_ASSIGN(const std::string sync, core::read_file(info.sync_path));
      sync_sha256 = detail::sha256_hex(sync);
    }
    if (directories.empty()) directories = tree();
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
    // TODO: encode and write each camera's files as a task of its own if the
    // rig's write time per capture (hand check H4) asks for it.
    for (std::size_t i = 0; i < info.cameras.size(); ++i) {
      const std::string& serial = info.cameras[i].info.id;
      const recon::camera::ImageSize& depth = info.cameras[i].info.depth->size;
      const CameraCapture& part = job.cameras[i];
      VKC_TRY(core::write_file(
          detail::join(info.directory, detail::color_file(serial, index)),
          bytes_of(part.jpeg), core::WriteMode::CreateNew));
      VKC_ASSIGN(const std::vector<std::uint8_t> png,
                 recon::io::encode_png_u16(part.depth.data(), part.depth.size(),
                                           depth.width, depth.height));
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

  // Takes the latest capture from `takeable`: drops it, or puts it back if
  // session.json cannot say so.
  core::Result<int> drop_last() {
    if (takeable.empty()) {
      return core::Status::not_found("SessionWriter: no capture to drop");
    }
    std::list<Outcome> taken;
    taken.splice(taken.end(), takeable, std::prev(takeable.end()));
    if (!taken.front()) {
      return core::Status::not_found(
          "SessionWriter: the capture to drop was not written");
    }
    detail::CaptureEntry& entry = captures[*taken.front()];
    entry.dropped = true;
    core::Status written;
    try {
      written = write_session();
    } catch (...) {
      entry.dropped = false;
      takeable.splice(takeable.end(), taken);
      throw;
    }
    if (!written.ok()) {
      entry.dropped = false;
      takeable.splice(takeable.end(), taken);
      return written;
    }
    return entry.index;
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
    if (capture && !request.outcome.empty()) {
      // Written or not, it is now the latest capture a drop can take; a
      // written one is the last listed.
      request.outcome.front() =
          result.status.ok() ? Outcome(captures.size() - 1) : std::nullopt;
      takeable.splice(takeable.end(), request.outcome);
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

  // The writer's thread's alone.
  std::vector<std::string> directories;  // the session's, then its tree
  std::string sync_sha256;
  std::string created;
  std::size_t made = 0;  // directories made
  bool factory_written = false;
  int next_index = 0;
  std::vector<detail::CaptureEntry> captures;  // what session.json lists
  // Each capture carried out, written or not, until a drop takes it; the
  // latest last.
  std::list<Outcome> takeable;

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
    VKC_ASSIGN(ArrayCalibration factory, check_info(info));
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
  try {
    VKC_TRY(check_job(state_->info, job));
    State::Request request;
    request.job = std::move(job);
    request.outcome.emplace_back();
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->held >= state_->capacity) return false;
    state_->queue.push_back(std::move(request));
    ++state_->held;
    ++state_->in_flight;
    state_->wake.notify_one();
    return true;
  } catch (...) {
    return core::exception_status("SessionWriter::submit",
                                  Code::InvalidArgument);
  }
}

core::Status SessionWriter::drop_last() {
  try {
    State::Request request;
    request.kind = WriteResult::Kind::Drop;
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->queue.push_back(std::move(request));
    ++state_->in_flight;
    state_->wake.notify_one();
    return {};
  } catch (...) {
    return core::exception_status("SessionWriter::drop_last",
                                  Code::OutOfMemory);
  }
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
  try {
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
  } catch (...) {
    return core::exception_status("session_stamp", Code::InvalidArgument);
  }
}

}  // namespace volumetric_kit::calib::capture
