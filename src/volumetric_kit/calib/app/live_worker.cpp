// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/live_worker.hpp"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>

#include <opencv2/core/utility.hpp>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/detect/charuco.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sensor.hpp"
#include "volumetric_kit/recon/sensor/utils/picture_readback.hpp"

namespace volumetric_kit::calib::app {

namespace {

namespace sensor = recon::sensor;
using Clock = std::chrono::steady_clock;

// How quickly the published rate follows a change: the weight of each new
// set's rate, about a second's worth at 25 sets a second.
constexpr double kRateWeight = 0.04;

core::Status with_camera(const std::string& id, const core::Status& status) {
  return status.with_context("camera " + id);
}

}  // namespace

struct LiveWorker::State {
  explicit State(sensor::SensorArray&& rig) : array(std::move(rig)) {}

  // The worker's thread's alone, once it runs.
  sensor::SensorArray array;
  std::unique_ptr<sensor::PictureReadback> readback;
  std::vector<compute::CvCamera> cameras;
  std::optional<compute::Board> detector_board;
  std::optional<quality::CaptureRule> rule;
  std::vector<Detection> previous;  // the last set's live detections
  bool have_previous = false;
  LiveOptions options;
  CaptureSink* sink = nullptr;
  std::uint64_t published = 0;
  std::optional<Clock::time_point> last_set;
  double rate = 0.0;

  // Shared with the window's thread.
  mutable std::mutex mutex;
  std::shared_ptr<const LiveSnapshot> snapshot;  // newest, not yet taken
  std::optional<Clock::time_point> press;        // a press not yet armed
  std::optional<std::pair<Target, bool>> next_board;
  Target board;
  bool square_measured = false;
  quality::CaptureDecision decision;  // the latest that was not Idle
  std::vector<core::Status> errors;
  std::atomic<bool> stopping{false};
  std::atomic<bool> running{false};
  std::atomic<std::uint64_t> captures{0};
  std::thread thread;

  void post(core::Status status) {
    const std::lock_guard<std::mutex> lock(mutex);
    errors.push_back(std::move(status));
  }

  // The window's requests since the last set: a press, a new board.
  core::Status apply_requests() {
    std::optional<Clock::time_point> pressed;
    std::optional<std::pair<Target, bool>> board_change;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      pressed = press;
      press.reset();
      board_change = std::move(next_board);
      next_board.reset();
    }
    if (board_change && captures.load() == 0) {
      VKC_ASSIGN(compute::Board made,
                 compute::Board::create(board_change->first));
      detector_board = std::move(made);
      const std::lock_guard<std::mutex> lock(mutex);
      board = board_change->first;
      square_measured = board_change->second;
    }
    if (pressed && sink != nullptr) rule->arm(*pressed);
    return {};
  }

  void publish_decision(const quality::CaptureDecision& latest) {
    if (latest.state == quality::CaptureState::Idle) return;
    const std::lock_guard<std::mutex> lock(mutex);
    decision = latest;
  }

  // Each camera's live detection and check, every camera at once.
  void detect(const std::vector<std::optional<sensor::PictureView>>& views,
              LiveSnapshot& out) {
    const std::size_t n = views.size();
    out.detections.assign(n, Detection{});
    out.checks.assign(n, ViewCheck{});
    out.errors.assign(n, core::Status{});
    const compute::Board& board_used = *detector_board;
    cv::parallel_for_(
        cv::Range(0, static_cast<int>(n)), [&](const cv::Range& range) {
          for (int c = range.start; c < range.end; ++c) {
            const auto i = static_cast<std::size_t>(c);
            out.errors[i] = detect_one(i, views[i], board_used,
                                       out.detections[i], out.checks[i]);
          }
        });
  }

  core::Status detect_one(std::size_t i,
                          const std::optional<sensor::PictureView>& view,
                          const compute::Board& board_used, Detection& found,
                          ViewCheck& check) const {
    if (!view) return {};
    return compute::guarded("live detection", [&]() -> core::Status {
      const detect::LumaView luma{view->width, view->height, view->stride,
                                  view->luma};
      VKC_ASSIGN(found,
                 detect::detect_charuco(luma, board_used, detect::Mode::Hybrid));
      // A view the check refuses still shows what was found.
      VKC_ASSIGN(const quality::CheckedView checked,
                 quality::check_view(cameras[i], board_used, found,
                                     options.view));
      check = checked.check;
      return {};
    });
  }

  // The set's frames copied out for the capture worker.
  core::Result<CaptureRequest> capture_request(
      const sensor::FrameSet& set,
      const std::vector<std::optional<sensor::PictureView>>& views,
      const LiveSnapshot& live, const quality::CaptureDecision& taken,
      const std::vector<std::string>& ids) const {
    CaptureRequest request;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      request.board = board;
      request.square_measured = square_measured;
    }
    request.meta.time_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    request.meta.trigger_ns = set.timestamp_ns;
    request.meta.sequence = set.sequence;
    request.meta.spread_ms = static_cast<double>(set.spread_ns()) / 1e6;
    request.meta.stillness_px = taken.stillness_px;
    request.cameras.resize(set.frames.size());
    for (std::size_t i = 0; i < set.frames.size(); ++i) {
      const std::optional<sensor::RgbdFrame>& frame = set.frames[i];
      if (!frame || !views[i]) {
        return core::Status::invalid_argument("camera " + ids[i] +
                                              " has no picture in the set");
      }
      if (frame->color_bitstream.codec != sensor::ColorBitstream::Codec::Jpeg ||
          frame->color_bitstream.data == nullptr) {
        return core::Status::invalid_argument("camera " + ids[i] +
                                              " sent no JPEG with its frame");
      }
      if (frame->depth == nullptr) {
        return core::Status::invalid_argument("camera " + ids[i] +
                                              " sent no depth with its frame");
      }
      CameraFrameCopy& part = request.cameras[i];
      const sensor::PictureView& view = *views[i];
      part.width = view.width;
      part.height = view.height;
      part.luma.resize(static_cast<std::size_t>(view.width) * view.height);
      for (std::uint32_t row = 0; row < view.height; ++row) {
        std::memcpy(&part.luma[static_cast<std::size_t>(row) * view.width],
                    view.luma + static_cast<std::size_t>(row) * view.stride,
                    view.width);
      }
      part.jpeg.assign(
          frame->color_bitstream.data,
          frame->color_bitstream.data + frame->color_bitstream.size);
      const std::size_t samples =
          static_cast<std::size_t>(frame->depth_camera.size.width) *
          frame->depth_camera.size.height;
      part.depth.assign(frame->depth, frame->depth + samples);
      part.timestamp_ns = frame->timestamp_ns;
      part.sequence = frame->sequence;
      part.live = live.detections[i];
    }
    return request;
  }

  // One set: read back, detected, checked, ruled on, published.
  core::Status process(sensor::FrameSet set, core::StageMetrics stages,
                       const std::vector<std::string>& ids) {
    auto out = std::make_shared<LiveSnapshot>();
    out->stages = std::move(stages);
    core::Result<std::vector<std::optional<sensor::PictureView>>> read =
        readback->read(set.frames, sensor::PictureReadback::Planes::Luma,
                       &out->stages);
    if (!read.ok()) {
      post(read.status().with_context("live view: set " +
                                      std::to_string(set.sequence)));
      return {};
    }
    const std::vector<std::optional<sensor::PictureView>>& views = read.value();
    {
      core::StageScope scope(out->stages, "detect");
      detect(views, *out);
    }
    for (std::size_t i = 0; i < out->errors.size(); ++i) {
      if (!out->errors[i].ok()) {
        out->errors[i] = with_camera(ids[i], out->errors[i]);
      }
    }

    // Stillness: each camera's corners against the set before.
    out->stillness_px.assign(ids.size(), std::nullopt);
    std::optional<double> motion;
    if (have_previous) {
      const core::Result<quality::CornerShift> shift =
          quality::corner_shift(previous, out->detections);
      if (shift.ok()) {
        out->stillness_px = shift.value().cameras;
        motion = shift.value().median_px;
      }
    }
    previous = out->detections;
    have_previous = true;

    quality::LiveSet live_set;
    live_set.present.resize(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
      live_set.present[i] = set.frames[i].has_value();
    }
    live_set.checks = out->checks;
    live_set.motion_px = motion;
    live_set.spread_ms = static_cast<double>(set.spread_ns()) / 1e6;
    live_set.saver_full = sink != nullptr && sink->full();
    VKC_ASSIGN(out->decision, rule->update(live_set, Clock::now()));
    if (out->decision.state == quality::CaptureState::Take) {
      take(set, views, *out, ids);
    }
    publish_decision(out->decision);

    out->lost.assign(ids.size(), std::nullopt);
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const auto* orbbec =
          dynamic_cast<const sensor::OrbbecSensor*>(&array.sensor(i));
      if (orbbec != nullptr) out->lost[i] = orbbec->orbbec_stats().lost;
    }
    out->stats = array.stats();
    const Clock::time_point now = Clock::now();
    if (last_set) {
      const double seconds =
          std::chrono::duration<double>(now - *last_set).count();
      if (seconds > 0.0) {
        const double instant = 1.0 / seconds;
        rate = rate == 0.0 ? instant : rate + kRateWeight * (instant - rate);
      }
    }
    last_set = now;
    out->sets_per_second = rate;
    out->number = ++published;
    out->set = std::make_shared<const sensor::FrameSet>(std::move(set));
    const std::lock_guard<std::mutex> lock(mutex);
    snapshot = std::move(out);
    return {};
  }

  // The rule took this set: copy it out and hand it to the sink.
  void take(const sensor::FrameSet& set,
            const std::vector<std::optional<sensor::PictureView>>& views,
            LiveSnapshot& live, const std::vector<std::string>& ids) {
    core::Result<CaptureRequest> request =
        capture_request(set, views, live, live.decision, ids);
    core::Result<bool> queued = request.ok()
                                    ? sink->submit(std::move(request).value())
                                    : core::Result<bool>(request.status());
    if (queued.ok() && queued.value()) {
      captures.fetch_add(1);
      return;
    }
    live.decision.state = quality::CaptureState::Failed;
    if (!queued.ok()) {
      live.decision.hold = quality::Hold::None;
      live.decision.message = queued.status().message();
      post(queued.status().with_context("capture"));
    } else {
      live.decision.hold = quality::Hold::SaverFull;
      live.decision.message = "the writer is full";
    }
  }

  core::Status loop(const std::vector<std::string>& ids) {
    VKC_TRY(array.start());
    while (!stopping.load()) {
      VKC_TRY(apply_requests());
      core::StageMetrics stages;
      std::optional<sensor::FrameSet> set;
      {
        core::StageScope scope(stages, "poll");
        VKC_ASSIGN(set, array.poll_set());
      }
      if (set) {
        VKC_TRY(process(std::move(*set), std::move(stages), ids));
        continue;
      }
      if (rule->armed()) {
        VKC_ASSIGN(const quality::CaptureDecision ticked,
                   rule->tick(Clock::now()));
        publish_decision(ticked);
      }
      if (array.exhausted()) {
        return core::Status::io_error("the rig stopped: a camera is gone");
      }
      std::this_thread::sleep_for(options.idle);
    }
    return {};
  }
};

core::Result<std::unique_ptr<LiveWorker>> LiveWorker::create(
    sensor::SensorArray&& array, const core::Device& device,
    core::Allocator& allocator, const Target& board, bool square_measured,
    CaptureSink* sink, const LiveOptions& options) {
  auto state = std::make_unique<State>(std::move(array));
  if (options.idle.count() < 0) {
    return core::Status::invalid_argument(
        "LiveWorker: the idle wait is negative");
  }
  return compute::guarded(
      "LiveWorker::create", [&]() -> core::Result<std::unique_ptr<LiveWorker>> {
        std::vector<std::string> ids;
        for (std::size_t i = 0; i < state->array.size(); ++i) {
          const sensor::SensorInfo& info = state->array.sensor(i).info();
          if (!info.color) {
            return core::Status::invalid_argument(
                "LiveWorker: camera " + info.id + " has no colour model");
          }
          VKC_ASSIGN(compute::CvCamera cv, compute::cv_camera(*info.color));
          state->cameras.push_back(std::move(cv));
          ids.push_back(info.id);
        }
        VKC_ASSIGN(compute::Board made, compute::Board::create(board));
        state->detector_board = std::move(made);
        VKC_ASSIGN(quality::CaptureRule rule,
                   quality::CaptureRule::create(ids, options.rule));
        state->rule = std::move(rule);
        VKC_ASSIGN(state->readback,
                   sensor::PictureReadback::create(device, allocator));
        state->board = board;
        state->square_measured = square_measured;
        state->options = options;
        state->sink = sink;
        std::unique_ptr<LiveWorker> worker(
            new LiveWorker(std::move(state), std::move(ids)));
        worker->state_->running.store(true);
        try {
          worker->state_->thread =
              std::thread([w = worker.get()] { w->run(); });
        } catch (const std::system_error& e) {
          worker->state_->running.store(false);
          return core::Status::io_error(
              std::string("LiveWorker: the thread will not start: ") +
              e.what());
        }
        return worker;
      });
}

LiveWorker::LiveWorker(std::unique_ptr<State> state,
                       std::vector<std::string> ids)
    : state_(std::move(state)), ids_(std::move(ids)) {}

LiveWorker::~LiveWorker() { stop(); }

void LiveWorker::run() noexcept {
  const core::Status ended = compute::guarded(
      "live view", [this]() -> core::Status { return state_->loop(ids_); });
  state_->array.stop();
  if (!ended.ok()) state_->post(ended);
  state_->running.store(false);
}

void LiveWorker::stop() noexcept {
  state_->stopping.store(true);
  if (state_->thread.joinable()) state_->thread.join();
}

bool LiveWorker::running() const noexcept { return state_->running.load(); }

bool LiveWorker::arm() {
  if (state_->sink == nullptr) return false;
  const std::lock_guard<std::mutex> lock(state_->mutex);
  if (!state_->press) state_->press = Clock::now();
  return true;
}

core::Status LiveWorker::set_board(const Target& board, bool square_measured) {
  return compute::guarded("LiveWorker::set_board", [&]() -> core::Status {
    if (state_->captures.load() > 0) {
      return core::Status::invalid_argument(
          "the board is locked: a capture was taken");
    }
    VKC_TRY(compute::Board::create(board).status());
    const std::lock_guard<std::mutex> lock(state_->mutex);
    state_->next_board = std::make_pair(board, square_measured);
    return {};
  });
}

Target LiveWorker::board() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->board;
}

bool LiveWorker::square_measured() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->square_measured;
}

std::uint64_t LiveWorker::captures() const noexcept {
  return state_->captures.load();
}

std::shared_ptr<const LiveSnapshot> LiveWorker::take_snapshot() {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return std::move(state_->snapshot);
}

quality::CaptureDecision LiveWorker::decision() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->decision;
}

std::vector<core::Status> LiveWorker::take_errors() {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  std::vector<core::Status> out;
  out.swap(state_->errors);
  return out;
}

}  // namespace volumetric_kit::calib::app
