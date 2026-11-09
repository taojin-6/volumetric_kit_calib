// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/capture_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>

#include <opencv2/core/utility.hpp>

#include "volumetric_kit/calib/app/calib_mode.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/detect/board.hpp"
#include "volumetric_kit/calib/detect/tag.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/sensor/io/image_io.hpp"
#include "volumetric_kit/sensor/v210_rgb.hpp"

namespace volumetric_kit::calib::app {

namespace {

// How often the worker looks for the writer's outcomes while nothing is
// queued.
constexpr std::chrono::milliseconds kCollectEvery{20};

using DepthSamples = std::shared_ptr<const std::vector<std::uint16_t>>;
using Tag = std::optional<DetectedMarker>;

// A capture on the writer, awaiting its outcome.
struct Written {
  std::vector<Detection> full;      // each camera's, for the session's checks
  std::vector<DepthSamples> depth;  // each camera's, shared with the writer
  std::vector<Tag> tags;            // each camera's origin tag
  std::vector<bool> frontal;        // each camera's view frontal
  CaptureEvent event;               // Saved, once its index is known
};

// The origin tag's search in one camera's luma.
struct TagSearch {
  Tag found;  // when the luma holds exactly one print of it
  int matches = 0;
  core::Status error;
};

TagSearch search_tag(const CameraFrameCopy& part) {
  TagSearch out;
  out.error = compute::guarded("origin tag search", [&]() -> core::Status {
    const detect::LumaView luma{part.width, part.height, part.width,
                                part.luma.data()};
    VKC_ASSIGN(const detect::MarkerSearch search,
               detect::detect_tag(luma, kOriginTag));
    out.found = search.found;
    out.matches = search.matches;
    return {};
  });
  return out;
}

// A V210 picture's words as an 8-bit RGB PNG file's bytes: converted as
// GpuFramePrep's colour pass converts it (v210_to_rgb8), then encoded at the
// encoder's default, with no setting shared, so cameras encode at once.
core::Result<std::vector<std::uint8_t>> colour_png(const ColourCopy& copy) {
  return compute::guarded(
      "still", [&]() -> core::Result<std::vector<std::uint8_t>> {
        sensor::YuvImage picture = copy.picture;
        picture.host = copy.words.data();
        std::vector<std::uint8_t> rgb(std::size_t{picture.width} *
                                      picture.height * 3);
        VKC_TRY(sensor::v210_to_rgb8(picture, rgb.data(), rgb.size()));
        return sensor::io::encode_png_rgb8(rgb.data(), rgb.size(),
                                           picture.width, picture.height);
      });
}

// A capture, a drop of the last one, or the next session.
struct Job {
  enum class Kind { Capture, Drop, Restart };
  Kind kind = Kind::Capture;
  CaptureRequest request;
  std::optional<CaptureSetup> setup;       // the next session's
  std::vector<compute::CvCamera> cameras;  // its lenses, as setup's
};

// Work refused once the thread has ended.
core::Status stopped() {
  return core::Status::io_error("the capture worker has stopped");
}

// Each camera's lens of `setup`, as compute's bridge takes it, once `setup`
// is checked: create's rules.
core::Result<std::vector<compute::CvCamera>> checked_lenses(
    const CaptureSetup& setup) {
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
  if (setup.max_kept == 0) {
    return core::Status::invalid_argument(
        "CaptureWorker: a max_kept of 0 keeps no capture");
  }
  if (setup.lenses.size() != setup.cameras.size()) {
    return core::Status::invalid_argument(
        "CaptureWorker: " + std::to_string(setup.lenses.size()) +
        " lenses for " + std::to_string(setup.cameras.size()) + " cameras");
  }
  std::vector<compute::CvCamera> cameras;
  for (std::size_t i = 0; i < setup.cameras.size(); ++i) {
    const capture::SessionCamera& camera = setup.cameras[i];
    const sensor::camera::ImageSize& lens = setup.lenses[i].size;
    if (lens.width != camera.size.width || lens.height != camera.size.height) {
      return core::Status::invalid_argument(
          "CaptureWorker: camera " + camera.id +
          "'s lens is not its pictures' size");
    }
    VKC_ASSIGN(compute::CvCamera cv, compute::cv_camera(setup.lenses[i]));
    cameras.push_back(std::move(cv));
  }
  return cameras;
}

}  // namespace

struct CaptureWorker::State {
  // Written by the worker's thread alone, under the lock (a restart), which
  // submit() takes to read it.
  CaptureSetup setup;

  // The worker's thread's alone.
  std::vector<compute::CvCamera> cameras;  // each camera's lens
  std::unique_ptr<capture::SessionWriter> writer;
  std::optional<compute::Board> board;
  std::optional<quality::SessionChecks> checks;
  // The session's depth unit, its first capture's; 0 without depth.
  float metres_per_unit = 0.0f;
  std::deque<Written> written;  // on the writer, in the order submitted
  KeptCaptures kept_base;       // what each snapshot holds but its captures
  std::vector<std::shared_ptr<const KeptCapture>> kept_captures;

  // Shared.
  mutable std::mutex mutex;
  std::condition_variable wake;  // a job, or the stop
  std::condition_variable idle;  // an outcome
  std::deque<Job> jobs;
  bool busy = false;         // a job is being carried out
  std::size_t captures = 0;  // captures queued and not yet resolved
  std::size_t drops = 0;     // drops queued and not yet resolved
  std::size_t restarts = 0;  // restarts queued and not yet carried out
  bool releasing = false;    // a snapshot replaced is being let go
  bool stopping = false;
  bool ended = false;  // the thread has returned: no job is carried out
  std::vector<CaptureEvent> events;
  std::optional<quality::SessionSummary> summary;
  std::vector<quality::LensReadiness> lenses;  // published with `summary`
  std::shared_ptr<const KeptCaptures> kept;    // published with `summary`
  std::string directory;
  std::vector<core::Status> errors;
  std::thread thread;

  // Report outcomes, the captures and drops they resolve, and errors, with
  // the session's summary and kept captures when they changed.
  void resolve(
      std::vector<CaptureEvent> outcomes, std::size_t captures_done,
      std::size_t drops_done, std::vector<core::Status> failures,
      std::optional<quality::SessionSummary> latest = {},
      std::shared_ptr<const KeptCaptures> kept_latest = {},
      std::optional<std::vector<quality::LensReadiness>> lenses_latest = {}) {
    const bool replaced = kept_latest != nullptr;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      for (CaptureEvent& event : outcomes) events.push_back(std::move(event));
      for (core::Status& failure : failures) {
        errors.push_back(std::move(failure));
      }
      captures -= captures_done;
      drops -= drops_done;
      if (latest) summary = std::move(latest);
      if (lenses_latest) lenses = std::move(*lenses_latest);
      if (replaced) {
        kept.swap(kept_latest);
        releasing = true;
      }
      idle.notify_all();
    }
    if (!replaced) return;
    // The snapshot replaced may hold a dropped capture's depth: it is let go
    // outside the lock, and before flush() returns.
    kept_latest.reset();
    const std::lock_guard<std::mutex> lock(mutex);
    releasing = false;
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

  // The depth unit of a request's cameras with depth (submit checked they
  // share one); 0 when none has depth.
  float unit_of(const CaptureRequest& request) const {
    for (std::size_t i = 0; i < request.cameras.size(); ++i) {
      if (setup.cameras[i].depth) return request.cameras[i].metres_per_unit;
    }
    return 0.0f;
  }

  // The session's writer and checks, at its first capture, for its board;
  // a scan's writer alone, as it searches for no board.
  core::Status open_session(const CaptureRequest& request) {
    const bool scan = setup.mode == capture::SessionMode::Scan;
    std::optional<compute::Board> made;
    if (!scan) {
      VKC_ASSIGN(made, compute::Board::create(request.board));
    }
    std::vector<solve::RigCamera> rig;
    for (std::size_t i = 0; i < setup.cameras.size(); ++i) {
      rig.push_back({setup.cameras[i].id, setup.lenses[i],
                     lens_source(setup.cameras[i], setup.calibration.get())});
    }
    std::optional<quality::SessionChecks> session;
    if (!scan) {
      VKC_ASSIGN(session, quality::SessionChecks::create(
                              rig, *made, setup.reference, setup.session));
    }
    KeptCaptures base;
    base.mode = setup.mode;
    base.calibration = setup.calibration;
    base.pairs = setup.pairs;
    base.pair_rows = setup.pair_rows;
    base.board = request.board;
    base.cameras = std::move(rig);
    base.sensors = setup.cameras;
    base.reference = setup.reference;
    base.solve = setup.session.solve;
    VKC_ASSIGN(const std::string stamp,
               capture::session_stamp(std::chrono::system_clock::now()));
    // A session of the same second (the one a switch just closed) keeps its
    // name: this one takes the next free `_<n>`.
    namespace fs = std::filesystem;
    fs::path where = fs::path(setup.root) / stamp;
    std::error_code ec;
    for (int n = 2; fs::exists(where, ec); ++n) {
      where = fs::path(setup.root) / (stamp + "_" + std::to_string(n));
    }
    const float unit = unit_of(request);
    capture::SessionInfo info;
    info.directory = where.string();
    info.software = setup.software;
    info.mode = setup.mode;
    info.driver = setup.driver;
    info.sync_path = setup.sync_path;
    info.reference = setup.cameras[setup.reference].id;
    info.board = request.board;
    info.square_measured = request.square_measured;
    info.depth_mode = setup.depth_mode;
    if (info.depth_mode) {
      info.depth_mode->metres_per_unit = unit;
      base.depth_mode = *info.depth_mode;
    }
    info.color_codec = setup.color_codec;
    info.cameras = setup.cameras;
    const std::string directory_name = info.directory;
    base.directory = info.directory;
    capture::SessionWriterOptions options;
    options.queue_capacity = setup.capacity;
    VKC_ASSIGN(writer,
               capture::SessionWriter::create(std::move(info), options));
    board = std::move(made);
    checks = std::move(session);
    kept_base = std::move(base);
    metres_per_unit = unit;
    const std::lock_guard<std::mutex> lock(mutex);
    directory = directory_name;
    return {};
  }

  // Camera `who`'s origin tag search's failure, or its tag in more than one
  // print, posted into `failures`: neither fails the capture.
  static void tag_failure(const std::string& who, const TagSearch& search,
                          std::vector<core::Status>& failures) {
    if (!search.error.ok()) {
      failures.push_back(search.error.with_context(who));
    } else if (search.matches > 1) {
      failures.push_back(core::Status::invalid_argument(
          who + ": " + std::to_string(search.matches) +
          " prints of the origin tag in view: none is kept"));
    }
  }

  // `job` queued on the writer, awaiting its outcome as `waiting`; or the
  // capture failed, saying why.
  void submit_job(capture::CaptureJob job, Written waiting) {
    const core::Result<bool> queued = writer->submit(std::move(job));
    if (!queued.ok()) {
      fail_capture(waiting.event.meta, queued.status().with_context("capture"));
      return;
    }
    if (!queued.value()) {
      fail_capture(waiting.event.meta,
                   core::Status::io_error(
                       "capture refused: the session writer is full"));
      return;
    }
    written.push_back(std::move(waiting));
  }

  // One capture: its board detected at full resolution and checked, and the
  // origin tag searched for, in each camera's luma; or a scan's stills.
  // Queued on the writer.
  void capture(CaptureRequest request) {
    if (writer == nullptr) {
      const core::Status opened = open_session(request);
      if (!opened.ok()) {
        fail_capture(request.meta, opened.with_context("session"));
        return;
      }
    } else if (unit_of(request) != metres_per_unit) {
      fail_capture(request.meta,
                   core::Status::invalid_argument(
                       "capture: its depth unit is not the session's"));
      return;
    }
    if (setup.mode == capture::SessionMode::Scan) {
      capture_scan(std::move(request));
      return;
    }
    const std::size_t n = request.cameras.size();
    std::vector<Detection> full(n);
    std::vector<ViewCheck> views(n);
    // Not a vector<bool>: each thread writes its own camera's.
    std::vector<char> frontal(n, 0);
    std::vector<core::Status> detect_errors(n);
    std::vector<core::Status> check_errors(n);
    std::vector<TagSearch> searches(n);
    // The origin tag places the rig's poses; a lens session has none.
    const bool tag = setup.mode == capture::SessionMode::Extrinsics;
    cv::parallel_for_(
        cv::Range(0, static_cast<int>(n)), [&](const cv::Range& range) {
          for (int c = range.start; c < range.end; ++c) {
            const auto i = static_cast<std::size_t>(c);
            detect_one(i, request.cameras[i], full[i], views[i], frontal[i],
                       detect_errors[i], check_errors[i]);
            if (tag) searches[i] = search_tag(request.cameras[i]);
          }
        });
    std::vector<core::Status> failures;
    for (std::size_t i = 0; i < n; ++i) {
      const std::string who = "capture: camera " + setup.cameras[i].id;
      if (!detect_errors[i].ok()) {
        failures.push_back(detect_errors[i].with_context(who));
      } else if (!check_errors[i].ok()) {
        failures.push_back(check_errors[i].with_context(who));
      }
      tag_failure(who, searches[i], failures);
    }

    std::vector<Detection> live(n);
    for (std::size_t i = 0; i < n; ++i) live[i] = request.cameras[i].live;
    CaptureEvent event;
    event.kind = CaptureEvent::Kind::Saved;
    event.meta = request.meta;
    const core::Result<quality::CornerShift> shift =
        quality::corner_shift(live, full, quality::corner_match(*board));
    if (shift.ok()) {
      event.meta.live_vs_full_px = shift.value().median_px;
      event.flagged =
          quality::live_vs_full_flagged(shift.value(), setup.live_vs_full);
    }
    for (const ViewCheck& view : views) {
      if (is_usable(view.verdict)) ++event.usable_views;
    }
    std::vector<Tag> tags(n);
    for (std::size_t i = 0; i < n; ++i) {
      tags[i] = searches[i].found;
      if (tags[i]) ++event.tag_cameras;
    }

    capture::CaptureJob job;
    job.meta = event.meta;
    job.cameras.resize(n);
    std::vector<DepthSamples> depth(n);
    for (std::size_t i = 0; i < n; ++i) {
      CameraFrameCopy& part = request.cameras[i];
      capture::CameraCapture& out = job.cameras[i];
      out.jpeg = std::move(part.jpeg);
      // The luma the session saves of a camera with no JPEG.
      if (out.jpeg.empty()) out.luma = std::move(part.luma);
      // The samples move into the one buffer the writer and the kept capture
      // share.
      if (setup.cameras[i].depth) {
        depth[i] = std::make_shared<const std::vector<std::uint16_t>>(
            std::move(part.depth));
      }
      out.depth = depth[i];
      out.timestamp_ns = part.timestamp_ns;
      out.sequence = part.sequence;
      out.detection = full[i];
      out.check = views[i];
      out.origin_tag = tags[i];
    }
    if (!failures.empty()) resolve({}, 0, 0, std::move(failures));
    submit_job(
        std::move(job),
        {std::move(full), std::move(depth), std::move(tags),
         std::vector<bool>(frontal.begin(), frontal.end()), std::move(event)});
  }

  // A scan's capture: no board searched or judged; each camera's origin tag
  // searched for in its luma, and each V210 picture made a PNG still, every
  // search and still at once (2n tasks of one loop, the stills the longest:
  // a 2160p pair about 0.77 s on an M5 Max, DECISIONS.md). A camera with a JPEG
  // keeps it; one with neither, its luma. A still that cannot be made fails
  // the capture, so none is saved grey by mistake.
  void capture_scan(CaptureRequest request) {
    const std::size_t n = request.cameras.size();
    std::vector<TagSearch> searches(n);
    std::vector<std::vector<std::uint8_t>> stills(n);
    std::vector<core::Status> still_errors(n);
    cv::parallel_for_(
        cv::Range(0, static_cast<int>(2 * n)), [&](const cv::Range& range) {
          for (int t = range.start; t < range.end; ++t) {
            const auto task = static_cast<std::size_t>(t);
            if (task < n) {
              searches[task] = search_tag(request.cameras[task]);
              continue;
            }
            const std::size_t i = task - n;
            std::optional<ColourCopy>& colour = request.cameras[i].colour;
            if (!colour || !request.cameras[i].jpeg.empty()) continue;
            core::Result<std::vector<std::uint8_t>> png = colour_png(*colour);
            colour.reset();  // its 22 MB of words go at once
            if (png.ok()) {
              stills[i] = std::move(png).value();
            } else {
              still_errors[i] = png.status().with_context("still");
            }
          }
        });
    for (std::size_t i = 0; i < n; ++i) {
      if (still_errors[i].ok()) continue;
      fail_capture(request.meta, still_errors[i].with_context(
                                     "capture: camera " + setup.cameras[i].id));
      return;
    }
    CaptureEvent event;
    event.kind = CaptureEvent::Kind::Saved;
    event.meta = request.meta;
    capture::CaptureJob job;
    job.meta = request.meta;
    job.cameras.resize(n);
    std::vector<Tag> tags(n);
    std::vector<Detection> none(n);
    for (std::size_t i = 0; i < n; ++i) {
      CameraFrameCopy& part = request.cameras[i];
      capture::CameraCapture& out = job.cameras[i];
      out.jpeg = std::move(part.jpeg);
      if (out.jpeg.empty()) out.png = std::move(stills[i]);
      if (out.jpeg.empty() && out.png.empty()) out.luma = std::move(part.luma);
      if (setup.cameras[i].depth) {
        out.depth = std::make_shared<const std::vector<std::uint16_t>>(
            std::move(part.depth));
      }
      out.timestamp_ns = part.timestamp_ns;
      out.sequence = part.sequence;
      none[i].image_size = setup.cameras[i].size;
      out.detection = none[i];
      tags[i] = searches[i].found;
      out.origin_tag = tags[i];
      if (tags[i]) ++event.tag_cameras;
    }
    std::vector<core::Status> failures;
    for (std::size_t i = 0; i < n; ++i) {
      tag_failure("capture: camera " + setup.cameras[i].id, searches[i],
                  failures);
    }
    if (!failures.empty()) resolve({}, 0, 0, std::move(failures));
    // Nothing is kept for Calibrate, which takes no scan: no depth.
    submit_job(std::move(job),
               {std::move(none), std::vector<DepthSamples>(n), std::move(tags),
                std::vector<bool>(n, false), std::move(event)});
  }

  void detect_one(std::size_t i, const CameraFrameCopy& part, Detection& found,
                  ViewCheck& view, char& frontal, core::Status& detect_error,
                  core::Status& check_error) const {
    const sensor::camera::ImageSize size{part.width, part.height};
    detect_error = compute::guarded("full detection", [&]() -> core::Status {
      const detect::LumaView luma{part.width, part.height, part.width,
                                  part.luma.data()};
      VKC_ASSIGN(found, detect::detect_board(luma, *board));
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
      frontal = checked.frontal ? 1 : 0;
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

  // This session closed, its captures and drops carried out and its writer
  // flushed, and the next one's setup taken: nothing is written until its
  // first capture.
  void restart_session(CaptureSetup next,
                       std::vector<compute::CvCamera> next_cameras) {
    if (writer != nullptr) {
      writer->flush();
      collect();
      writer.reset();
    }
    board.reset();
    checks.reset();
    metres_per_unit = 0.0f;
    written.clear();
    kept_base = KeptCaptures{};
    kept_captures.clear();
    cameras = std::move(next_cameras);
    std::shared_ptr<const KeptCaptures> released;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      setup = std::move(next);
      summary.reset();
      lenses.clear();
      released = std::move(kept);
      kept.reset();
      directory.clear();
      releasing = released != nullptr;
    }
    // The last snapshot may hold the session's depth: let go outside the
    // lock, and before flush() returns.
    released.reset();
    const std::lock_guard<std::mutex> lock(mutex);
    releasing = false;
    --restarts;
    idle.notify_all();
  }

  // The writer's outcomes since the last look, the session's checks and the
  // captures kept.
  void collect() {
    if (writer == nullptr) return;
    std::vector<capture::WriteResult> results = writer->take_results();
    if (results.empty()) return;
    std::vector<CaptureEvent> outcomes;
    std::vector<core::Status> failures;
    std::size_t captures_done = 0;
    std::size_t drops_done = 0;
    std::vector<std::shared_ptr<const KeptCapture>> now = kept_captures;
    bool kept_changed = false;
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
          done.event.pictures = std::move(result.pictures);
          // Every view names its camera, one whose detection failed too.
          for (std::size_t c = 0; c < done.full.size(); ++c) {
            Observation& view = done.full[c].observation;
            view.camera_id = setup.cameras[c].id;
            view.frame_id = result.index;
          }
          // A scan has no checks.
          if (checks) {
            const core::Status added = checks->add(result.index, done.full);
            if (!added.ok()) failures.push_back(added.with_context("session"));
          }
          auto kept_capture = std::make_shared<KeptCapture>();
          kept_capture->index = result.index;
          kept_capture->views = std::move(done.full);
          kept_capture->depth = std::move(done.depth);
          kept_capture->tags = std::move(done.tags);
          kept_capture->frontal = std::move(done.frontal);
          now.push_back(std::move(kept_capture));
          kept_changed = true;
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
        if (checks) {
          const core::Status dropped = checks->drop(result.index);
          if (!dropped.ok()) {
            failures.push_back(dropped.with_context("session"));
          }
        }
        const auto taken = std::find_if(
            now.begin(), now.end(),
            [&](const std::shared_ptr<const KeptCapture>& capture) {
              return capture->index == result.index;
            });
        if (taken != now.end()) now.erase(taken);
        kept_changed = true;
      }
      outcomes.push_back(std::move(event));
    }
    std::optional<quality::SessionSummary> summary_now;
    if (checks) {
      core::Result<quality::SessionSummary> latest = checks->summary();
      if (latest.ok()) {
        summary_now = std::move(latest).value();
      } else {
        failures.push_back(latest.status().with_context("session"));
      }
    }
    std::shared_ptr<const KeptCaptures> kept_now;
    std::optional<std::vector<quality::LensReadiness>> lenses_now;
    if (kept_changed) {
      auto snapshot = std::make_shared<KeptCaptures>(kept_base);
      snapshot->captures = now;
      kept_now = std::move(snapshot);
      kept_captures = std::move(now);
      // A lens session's readiness, for its panel and Calibrate's gate.
      if (setup.mode == capture::SessionMode::Intrinsics) {
        core::Result<std::vector<quality::LensReadiness>> states =
            lens_states(*kept_now);
        if (states.ok()) {
          lenses_now = std::move(states).value();
        } else {
          failures.push_back(states.status().with_context("session"));
        }
      }
    }
    resolve(std::move(outcomes), captures_done, drops_done, std::move(failures),
            std::move(summary_now), std::move(kept_now), std::move(lenses_now));
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
        switch (job->kind) {
          case Job::Kind::Drop:
            drop();
            break;
          case Job::Kind::Restart:
            restart_session(std::move(*job->setup), std::move(job->cameras));
            break;
          case Job::Kind::Capture:
            capture(std::move(job->request));
            break;
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
        auto state = std::make_unique<State>();
        VKC_ASSIGN(state->cameras, checked_lenses(setup));
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
  const std::lock_guard<std::mutex> lock(state_->mutex);
  if (!ended.ok()) state_->errors.push_back(ended);
  state_->ended = true;
  state_->idle.notify_all();
}

bool CaptureWorker::full() const { return pending() >= capacity_; }

core::Result<bool> CaptureWorker::submit(CaptureRequest request) {
  return compute::guarded("CaptureWorker::submit", [&]() -> core::Result<bool> {
    // The setup is the worker's to replace (a restart), under this lock.
    std::unique_lock<std::mutex> lock(state_->mutex);
    const std::vector<capture::SessionCamera>& cameras = state_->setup.cameras;
    if (request.cameras.size() != cameras.size()) {
      return core::Status::invalid_argument(
          "capture: " + std::to_string(request.cameras.size()) +
          " cameras' frames for a rig of " + std::to_string(cameras.size()));
    }
    const CameraFrameCopy* first_depth = nullptr;
    std::size_t first = 0;
    for (std::size_t i = 0; i < cameras.size(); ++i) {
      const CameraFrameCopy& part = request.cameras[i];
      const sensor::camera::ImageSize& size = cameras[i].size;
      if (part.width != size.width || part.height != size.height ||
          part.luma.size() !=
              static_cast<std::size_t>(part.width) * part.height) {
        return core::Status::invalid_argument(
            "capture: camera " + cameras[i].id +
            "'s luma is not its pictures' size");
      }
      if (part.colour &&
          (part.colour->picture.layout != sensor::YuvLayout::V210 ||
           part.colour->picture.width != size.width ||
           part.colour->picture.height != size.height)) {
        return core::Status::invalid_argument(
            "capture: camera " + cameras[i].id +
            "'s colour is not V210 at its pictures' size");
      }
      if (!cameras[i].depth) {
        if (!part.depth.empty()) {
          return core::Status::invalid_argument(
              "capture: camera " + cameras[i].id + " has no depth camera");
        }
        continue;
      }
      const float unit = part.metres_per_unit;
      if (!(std::isfinite(unit) && unit > 0.0f)) {
        return core::Status::invalid_argument(
            "capture: camera " + cameras[i].id + " has no depth unit");
      }
      if (first_depth == nullptr) {
        first_depth = &part;
        first = i;
      } else if (unit != first_depth->metres_per_unit) {
        return core::Status::invalid_argument(
            "capture: camera " + cameras[i].id +
            "'s depth unit is not camera " + cameras[first].id + "'s");
      }
    }
    const std::size_t max_kept = state_->setup.max_kept;
    // A scan keeps no depth and is never calibrated: no bound.
    const bool scan = state_->setup.mode == capture::SessionMode::Scan;
    if (state_->ended) return stopped();
    const std::size_t kept =
        state_->kept != nullptr ? state_->kept->captures.size() : 0;
    if (!scan && kept + state_->captures >= max_kept) {
      return core::Status::unsupported(
          "capture: drop a capture first: one Calibrate adjusts at most " +
          std::to_string(max_kept));
    }
    if (state_->captures >= capacity_) return false;
    Job job;
    job.request = std::move(request);
    state_->jobs.push_back(std::move(job));
    ++state_->captures;
    lock.unlock();
    state_->wake.notify_all();
    return true;
  });
}

core::Status CaptureWorker::drop_last() {
  return compute::guarded("CaptureWorker::drop_last", [&]() -> core::Status {
    {
      const std::lock_guard<std::mutex> lock(state_->mutex);
      if (state_->ended) return stopped();
      Job job;
      job.kind = Job::Kind::Drop;
      state_->jobs.push_back(std::move(job));
      ++state_->drops;
    }
    state_->wake.notify_all();
    return {};
  });
}

namespace {

// `next` against the session it follows, `now`: the same cameras (ids,
// sizes, depth) and capacity.
core::Status same_rig(const CaptureSetup& now, std::size_t capacity,
                      const CaptureSetup& next) {
  bool same = now.cameras.size() == next.cameras.size();
  for (std::size_t i = 0; same && i < now.cameras.size(); ++i) {
    const capture::SessionCamera& a = now.cameras[i];
    const capture::SessionCamera& b = next.cameras[i];
    same = a.id == b.id && a.size.width == b.size.width &&
           a.size.height == b.size.height &&
           a.depth.has_value() == b.depth.has_value();
  }
  if (!same) {
    return core::Status::invalid_argument(
        "CaptureWorker::restart: the next session's cameras are not this "
        "one's");
  }
  if (next.capacity != capacity) {
    return core::Status::invalid_argument(
        "CaptureWorker::restart: the capacity is " + std::to_string(capacity) +
        ", not " + std::to_string(next.capacity));
  }
  return {};
}

}  // namespace

core::Status CaptureWorker::accepts(const CaptureSetup& setup) const {
  return compute::guarded("CaptureWorker::accepts", [&]() -> core::Status {
    VKC_TRY(checked_lenses(setup).status());
    const std::lock_guard<std::mutex> lock(state_->mutex);
    return same_rig(state_->setup, capacity_, setup);
  });
}

core::Status CaptureWorker::restart(CaptureSetup setup) {
  return compute::guarded("CaptureWorker::restart", [&]() -> core::Status {
    VKC_ASSIGN(std::vector<compute::CvCamera> cameras, checked_lenses(setup));
    {
      const std::lock_guard<std::mutex> lock(state_->mutex);
      if (state_->ended) return stopped();
      VKC_TRY(same_rig(state_->setup, capacity_, setup));
      Job job;
      job.kind = Job::Kind::Restart;
      job.setup = std::move(setup);
      job.cameras = std::move(cameras);
      state_->jobs.push_back(std::move(job));
      ++state_->restarts;
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

std::vector<quality::LensReadiness> CaptureWorker::lenses() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->lenses;
}

std::shared_ptr<const KeptCaptures> CaptureWorker::kept() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->kept;
}

std::size_t CaptureWorker::pending() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->captures;
}

std::size_t CaptureWorker::unresolved() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->captures + state_->drops + state_->restarts;
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
    return state_->ended ||
           (state_->jobs.empty() && !state_->busy && state_->captures == 0 &&
            state_->drops == 0 && state_->restarts == 0 && !state_->releasing);
  });
}

}  // namespace volumetric_kit::calib::app
