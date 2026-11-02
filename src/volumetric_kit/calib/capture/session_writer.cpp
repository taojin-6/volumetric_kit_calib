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
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/io/image_io.hpp"

namespace volumetric_kit::calib::capture {
namespace {

using sensor::camera::ArrayCalibration;
using Code = core::Status::Code;

core::Status invalid(const std::string& message) {
  return core::Status::invalid_argument("SessionWriter: " + message);
}

bool finite(const std::optional<double>& value) {
  return !value || std::isfinite(*value);
}

bool finite(const sensor::camera::Vec2d& p) {
  return std::isfinite(p.x) && std::isfinite(p.y);
}

std::string size_text(const sensor::camera::ImageSize& size) {
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

bool same_size(const sensor::camera::ImageSize& a,
               const sensor::camera::ImageSize& b) noexcept {
  return a.width == b.width && a.height == b.height;
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

// The factory record: each camera's factory models and depth_to_color,
// unposed, which sensor checks: no id twice, models it can hold. Empty when no
// camera has a factory model.
core::Result<std::optional<ArrayCalibration>> factory_calibration(
    const SessionInfo& info) {
  ArrayCalibration array;
  for (const SessionCamera& camera : info.cameras) {
    if (!camera.color && !camera.depth) continue;
    sensor::camera::SensorCalibration sensor;
    sensor.id = camera.id;
    if (camera.color) {
      sensor.color = sensor::camera::CameraCalibration{
          *camera.color, sensor::camera::IntrinsicsSource::Factory};
    }
    if (camera.depth) {
      sensor.depth = sensor::camera::CameraCalibration{
          *camera.depth, sensor::camera::IntrinsicsSource::Factory};
    }
    sensor.depth_to_color = camera.depth_to_color;
    array.sensors.push_back(std::move(sensor));
  }
  if (array.sensors.empty()) return std::optional<ArrayCalibration>();
  if (core::Status valid = sensor::camera::validate_array_calibration(array);
      !valid.ok()) {
    return std::move(valid).with_context("SessionWriter: factory calibration");
  }
  return std::optional<ArrayCalibration>(std::move(array));
}

core::Status check_camera(const SessionCamera& camera,
                          const SessionCamera& first) {
  const std::string& id = camera.id;
  if (!detail::is_plain_serial(id)) {
    return invalid("camera id \"" + id + "\" is not a plain file name");
  }
  if (camera.size.width == 0 || camera.size.height == 0) {
    return invalid("camera " + id + " has no size");
  }
  if (!same_size(camera.size, first.size)) {
    return invalid("camera " + id + " is " + size_text(camera.size) +
                   ", camera " + first.id + " " + size_text(first.size));
  }
  if (camera.color && !same_size(camera.color->size, camera.size)) {
    return invalid("camera " + id + "'s colour model is " +
                   size_text(camera.color->size) + ", not its " +
                   size_text(camera.size));
  }
  if (camera.fps == 0) return invalid("camera " + id + "'s fps is unknown");
  if (camera.fps != first.fps) {
    return invalid("camera " + id + " streams at " +
                   std::to_string(camera.fps) + " fps, camera " + first.id +
                   " at " + std::to_string(first.fps));
  }
  if ((camera.lens == LensOrigin::File) == camera.lens_file.empty()) {
    return invalid("camera " + id +
                   (camera.lens == LensOrigin::File
                        ? "'s lens came from a file it does not name"
                        : " names a lens file its lens did not come from"));
  }
  if (camera.lens != LensOrigin::Factory && camera.lens != LensOrigin::File &&
      camera.lens != LensOrigin::StandIn) {
    return invalid("camera " + id + "'s lens origin is not a LensOrigin");
  }
  if (camera.lens == LensOrigin::Factory && !camera.color) {
    return invalid("camera " + id +
                   "'s lens came from a factory model it does not have");
  }
  if (!camera.lens_file.empty()) {
    VKC_TRY(core::check_path("SessionWriter: camera " + id + "'s lens file",
                             camera.lens_file));
  }
  return {};
}

// The session's checks, around sensor's of its factory record, which it
// returns.
core::Result<std::optional<ArrayCalibration>> check_info(
    const SessionInfo& info) {
  VKC_TRY(core::check_path("SessionWriter: directory", info.directory));
  if (!info.sync_path.empty()) {
    VKC_TRY(core::check_path("SessionWriter: sync_path", info.sync_path));
  }
  if (info.driver.empty()) return invalid("no driver named");
  if (core::Status board = check_target(info.board); !board.ok()) {
    return std::move(board).with_context("SessionWriter: board");
  }
  if (info.board.kind != TargetKind::ChArUco) {
    return core::Status::unsupported(
        std::string("SessionWriter: a session's board is ChArUco, not ") +
        to_string(info.board.kind));
  }
  if (core::Status tag = check_origin_tag(info.origin_tag); !tag.ok()) {
    return std::move(tag).with_context("SessionWriter");
  }
  if (info.mode != SessionMode::Extrinsics &&
      info.mode != SessionMode::Intrinsics) {
    return invalid("mode is not a SessionMode");
  }
  if (info.cameras.empty()) return invalid("no cameras");
  const SessionCamera& first = info.cameras.front();
  const SessionCamera* first_depth = nullptr;
  bool reference = false;
  for (std::size_t i = 0; i < info.cameras.size(); ++i) {
    const SessionCamera& camera = info.cameras[i];
    VKC_TRY(check_camera(camera, first));
    for (std::size_t j = 0; j < i; ++j) {
      if (same_ignoring_case(info.cameras[j].id, camera.id)) {
        return invalid(info.cameras[j].id == camera.id
                           ? "camera " + camera.id + " is named twice"
                           : "cameras " + info.cameras[j].id + " and " +
                                 camera.id + " differ only in case");
      }
    }
    if (camera.depth) {
      if (first_depth == nullptr) first_depth = &camera;
      if (!same_size(camera.depth->size, first_depth->depth->size)) {
        return invalid("camera " + camera.id + "'s depth model is " +
                       size_text(camera.depth->size) + ", camera " +
                       first_depth->id + "'s " +
                       size_text(first_depth->depth->size));
      }
    }
    reference = reference || camera.id == info.reference;
  }
  if (first_depth != nullptr && !info.depth_mode) {
    return invalid("camera " + first_depth->id +
                   " has depth, and the session no depth mode");
  }
  if (first_depth == nullptr && info.depth_mode) {
    return invalid("a depth mode, and no camera with depth");
  }
  if (info.depth_mode) VKC_TRY(check_depth_mode(*info.depth_mode));
  if (!reference) {
    return invalid("reference \"" + info.reference + "\" is not a camera");
  }
  return factory_calibration(info);
}

// @pre `info` passed check_info.
core::Status check_part(const SessionInfo& info, std::size_t i,
                        const CameraCapture& part) {
  const SessionCamera& camera = info.cameras[i];
  const std::string where = "capture: camera " + camera.id + ": ";
  if (!part.jpeg.empty()) {
    // A JPEG starts with its SOI marker.
    if (part.jpeg.size() < 2 || part.jpeg[0] != 0xFF || part.jpeg[1] != 0xD8) {
      return invalid(where + "the colour is not a JPEG");
    }
  } else if (part.luma.size() !=
             static_cast<std::size_t>(camera.size.width) * camera.size.height) {
    return invalid(where + "neither a JPEG nor its " + size_text(camera.size) +
                   " luma");
  }
  if (!camera.depth) {
    if (part.depth != nullptr) {
      return invalid(where + "depth from a camera without depth");
    }
  } else {
    if (part.depth == nullptr) return invalid(where + "no depth");
    const sensor::camera::ImageSize& depth = camera.depth->size;
    const std::size_t samples =
        static_cast<std::size_t>(depth.width) * depth.height;
    if (part.depth->size() != samples) {
      return invalid(where + std::to_string(part.depth->size()) +
                     " depth samples for a " + size_text(depth) +
                     " depth image");
    }
  }
  const sensor::camera::ImageSize& size = part.detection.image_size;
  if (!same_size(size, camera.size)) {
    return invalid(where + "a detection in a " + size_text(size) +
                   " image, not the camera's " + size_text(camera.size));
  }
  for (const DetectedPoint& point : part.detection.observation.points) {
    if (!finite(point.pixel) || !std::isfinite(point.weight)) {
      return invalid(where + "corner " + std::to_string(point.id) +
                     " is not finite");
    }
  }
  for (const DetectedMarker& marker : part.detection.markers) {
    for (const sensor::camera::Vec2d& corner : marker.corners) {
      if (!finite(corner)) {
        return invalid(where + "marker " + std::to_string(marker.id) +
                       " is not finite");
      }
    }
  }
  if (part.origin_tag) {
    if (part.origin_tag->id != info.origin_tag.id) {
      return invalid(
          where + "origin tag " + std::to_string(part.origin_tag->id) +
          " is not the session's " + std::to_string(info.origin_tag.id));
    }
    for (const sensor::camera::Vec2d& corner : part.origin_tag->corners) {
      if (!finite(corner)) {
        return invalid(where + "the origin tag is not finite");
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

  // May throw: a deque may allocate as it is made (libstdc++'s does).
  State(SessionInfo session, std::optional<ArrayCalibration> factory_record,
        std::size_t queue_capacity)
      : info(std::move(session)),
        factory(std::move(factory_record)),
        capacity(queue_capacity) {}

  // --- The writer's thread alone ----------------------------------------

  // The session's directory, then its tree: each camera's images, and each
  // depth camera's depth.
  std::vector<std::string> tree() const {
    const std::string& root = info.directory;
    std::vector<std::string> paths = {root};
    const std::string images = detail::join(root, detail::kImagesDir);
    paths.push_back(images);
    for (const SessionCamera& camera : info.cameras) {
      paths.push_back(detail::join(images, camera.id));
    }
    if (info.depth_mode) {
      const std::string depth = detail::join(root, detail::kDepthDir);
      paths.push_back(depth);
      for (const SessionCamera& camera : info.cameras) {
        if (camera.depth) paths.push_back(detail::join(depth, camera.id));
      }
    }
    paths.push_back(detail::join(root, detail::kCapturesDir));
    return paths;
  }

  // The SHA-256 of the files the session names, the sync file and the lens
  // files, each read once.
  core::Status read_named_files() {
    if (!info.sync_path.empty() && facts.sync_sha256.empty()) {
      VKC_ASSIGN(const std::string sync, core::read_file(info.sync_path));
      facts.sync_sha256 = detail::sha256_hex(sync);
    }
    facts.lens_sha256.resize(info.cameras.size());
    for (std::size_t i = 0; i < info.cameras.size(); ++i) {
      const std::string& path = info.cameras[i].lens_file;
      if (path.empty() || !facts.lens_sha256[i].empty()) continue;
      for (std::size_t j = 0; j < i; ++j) {
        if (info.cameras[j].lens_file == path) {
          facts.lens_sha256[i] = facts.lens_sha256[j];
        }
      }
      if (!facts.lens_sha256[i].empty()) continue;
      VKC_ASSIGN(const std::string lens, core::read_file(path));
      facts.lens_sha256[i] = detail::sha256_hex(lens);
    }
    return {};
  }

  // The session's directory tree and factory record, at the first capture;
  // a failure leaves what was made, and the next capture goes on from there.
  core::Status prepare() {
    VKC_TRY(read_named_files());
    if (directories.empty()) directories = tree();
    while (made < directories.size()) {
      if (made == 0) {
        std::string stamp = detail::iso_time(now_ns());
        VKC_TRY(core::create_new_directory(directories[0]));
        facts.created = std::move(stamp);
      } else {
        VKC_TRY(core::create_new_directory(directories[made]));
      }
      ++made;
    }
    if (factory && !facts.factory) {
      VKC_TRY(sensor::camera::write_array_calibration(
          detail::join(info.directory, detail::kFactoryFile), *factory,
          core::WriteMode::CreateNew));
      facts.factory = true;
    }
    return {};
  }

  core::Status write_session() {
    return core::write_file(detail::join(info.directory, detail::kSessionFile),
                            detail::format_session(info, facts, captures),
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
      const SessionCamera& camera = info.cameras[i];
      const CameraCapture& part = job.cameras[i];
      const bool jpeg = !part.jpeg.empty();
      const std::string color = detail::join(
          info.directory, detail::color_file(camera.id, index, jpeg));
      if (jpeg) {
        VKC_TRY(core::write_file(color, bytes_of(part.jpeg),
                                 core::WriteMode::CreateNew));
      } else {
        VKC_TRY(core::write_file(color, detail::pgm(part.luma, camera.size),
                                 core::WriteMode::CreateNew));
      }
      if (!camera.depth) continue;
      const sensor::camera::ImageSize& depth = camera.depth->size;
      const std::vector<std::uint16_t>& samples = *part.depth;
      VKC_ASSIGN(const std::vector<std::uint8_t> png,
                 sensor::io::encode_png_u16(samples.data(), samples.size(),
                                            depth.width, depth.height));
      VKC_TRY(core::write_file(
          detail::join(info.directory, detail::depth_file(camera.id, index)),
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
  const std::optional<ArrayCalibration> factory;  // none without a factory lens
  const std::size_t capacity;

  // The writer's thread's alone.
  std::vector<std::string> directories;  // the session's, then its tree
  detail::SessionFacts facts;            // what session.json adds to `info`
  std::size_t made = 0;                  // directories made
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
    VKC_ASSIGN(std::optional<ArrayCalibration> factory, check_info(info));
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

const char* to_string(LensOrigin origin) noexcept {
  switch (origin) {
    case LensOrigin::Factory:
      return "factory";
    case LensOrigin::File:
      return "file";
    case LensOrigin::StandIn:
      return "stand-in";
  }
  return "unknown";
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
