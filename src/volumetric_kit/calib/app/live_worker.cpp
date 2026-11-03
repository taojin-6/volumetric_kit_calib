// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/live_worker.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>

#include <opencv2/core/utility.hpp>

#include "size_text.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/detect/board.hpp"

namespace volumetric_kit::calib::app {

namespace {

using Clock = std::chrono::steady_clock;

// How quickly the published rate follows a change: the weight of each new
// set's rate, about a second's worth at 25 sets a second.
constexpr double kRateWeight = 0.04;

core::Status with_camera(const std::string& id, const core::Status& status) {
  return status.with_context("camera " + id);
}

// What restart() hands the thread: the next session's lenses, board and
// capture rule, made and checked on the caller's thread.
struct Restart {
  std::vector<compute::CvCamera> cameras;
  compute::Board board;
  quality::CaptureRule rule;
};

// Each camera's lens, in the source's order, as compute's bridge takes it;
// refuses a count or a size that is not the source's.
core::Result<std::vector<compute::CvCamera>> live_lenses(
    const std::vector<source::SourceCamera>& cameras,
    const std::vector<sensor::camera::CameraModel>& lenses) {
  if (lenses.size() != cameras.size()) {
    return core::Status::invalid_argument(
        "LiveWorker: " + std::to_string(lenses.size()) + " lenses for " +
        std::to_string(cameras.size()) + " cameras");
  }
  std::vector<compute::CvCamera> out;
  for (std::size_t i = 0; i < cameras.size(); ++i) {
    const sensor::camera::ImageSize& size = cameras[i].size;
    if (lenses[i].size.width != size.width ||
        lenses[i].size.height != size.height) {
      return core::Status::invalid_argument(
          "LiveWorker: camera " + cameras[i].id + "'s lens is " +
          size_text(lenses[i].size) + ", its pictures " + size_text(size));
    }
    VKC_ASSIGN(compute::CvCamera cv, compute::cv_camera(lenses[i]));
    out.push_back(std::move(cv));
  }
  return out;
}

// The rule's thresholds as the source sets them: a pair's skew within its
// sensor's bound (`RigSource::max_skew_ns`) is no warning.
quality::CaptureRuleOptions source_rule(
    const std::optional<std::uint64_t>& skew_bound_ns,
    quality::CaptureRuleOptions rule) {
  if (skew_bound_ns) {
    rule.spread_warn_ms = std::max(rule.spread_warn_ms,
                                   static_cast<double>(*skew_bound_ns) / 1e6);
  }
  return rule;
}

}  // namespace

struct LiveWorker::State {
  explicit State(std::unique_ptr<source::RigSource> rig)
      : source(std::move(rig)) {}

  // The worker's thread's alone, once it runs.
  std::unique_ptr<source::RigSource> source;
  std::vector<compute::CvCamera> cameras;
  std::optional<compute::Board> detector_board;
  std::optional<quality::CaptureRule> rule;
  std::vector<Detection> previous;  // the last set's live detections
  bool have_previous = false;
  LiveOptions options;  // the rule's spread warning is the source's
  // The source's sync bound, read once at create: restart() raises a rule's
  // spread warning to it on the window's thread.
  std::optional<std::uint64_t> skew_bound_ns;
  CaptureSink* sink = nullptr;
  SetSink* sets = nullptr;
  std::uint64_t published = 0;
  std::optional<Clock::time_point> last_set;
  double rate = 0.0;

  // Shared with the window's thread.
  mutable std::mutex mutex;
  std::shared_ptr<const LiveSnapshot> snapshot;  // newest, not yet taken
  std::optional<Clock::time_point> press;        // a press not yet armed
  std::optional<std::pair<Target, bool>> next_board;
  std::optional<Restart> restart;  // the next session's, not yet taken
  quality::CaptureRuleOptions rule_options;  // the rule's, for the panels
  Target board;
  bool square_measured = false;
  quality::CaptureDecision decision;  // the latest that was not Idle
  std::vector<core::Status> errors;
  bool paused = false;         // the search paused (pause_search)
  bool searching = false;      // a set's search under way (begin_search)
  std::uint64_t searched = 0;  // the searches ended so far
  std::condition_variable search_ended;
  std::atomic<bool> detecting{true};
  std::atomic<bool> stopping{false};
  std::atomic<bool> running{false};
  std::atomic<std::uint64_t> captures{0};
  std::thread thread;

  void post(core::Status status) {
    const std::lock_guard<std::mutex> lock(mutex);
    errors.push_back(std::move(status));
  }

  // The window's requests since the last set: the next session, a press, a
  // new board.
  core::Status apply_requests() {
    std::optional<Clock::time_point> pressed;
    std::optional<std::pair<Target, bool>> board_change;
    std::optional<Restart> next;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      pressed = press;
      press.reset();
      board_change = std::move(next_board);
      next_board.reset();
      next = std::move(restart);
      restart.reset();
    }
    if (next) {
      cameras = std::move(next->cameras);
      detector_board = std::move(next->board);
      rule = std::move(next->rule);
      have_previous = false;
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

  // Whether to search a set's pictures: while the window searches and the
  // search is not paused, or for an armed capture. Decided under the lock
  // pause_search() takes, so once a pause returns no search starts but an
  // armed capture's; one started is under way until end_search().
  bool begin_search() {
    const std::lock_guard<std::mutex> lock(mutex);
    searching = (detecting.load() && !paused) || rule->armed();
    return searching;
  }

  // The search under way, if any, ended: a pause waiting for it returns.
  void end_search() {
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (!searching) return;
      searching = false;
      ++searched;
    }
    search_ended.notify_all();
  }

  void publish_decision(const quality::CaptureDecision& latest) {
    if (latest.state == quality::CaptureState::Idle) return;
    const std::lock_guard<std::mutex> lock(mutex);
    decision = latest;
  }

  // Each camera's live detection and check, every camera at once.
  void detect(const source::PictureSet& set,
              const std::vector<source::HostLuma>& host, LiveSnapshot& out) {
    const std::size_t n = set.cameras.size();
    out.detections.assign(n, Detection{});
    out.checks.assign(n, ViewCheck{});
    out.errors.assign(n, core::Status{});
    const compute::Board& board_used = *detector_board;
    cv::parallel_for_(
        cv::Range(0, static_cast<int>(n)), [&](const cv::Range& range) {
          for (int c = range.start; c < range.end; ++c) {
            const auto i = static_cast<std::size_t>(c);
            if (!set.cameras[i].present) continue;
            out.errors[i] = detect_one(i, host[i], board_used,
                                       out.detections[i], out.checks[i]);
          }
        });
  }

  core::Status detect_one(std::size_t i, const source::HostLuma& picture,
                          const compute::Board& board_used, Detection& found,
                          ViewCheck& check) const {
    return compute::guarded("live detection", [&]() -> core::Status {
      // Searched in the half the GPU made, each corner refined in the luma.
      VKC_ASSIGN(found,
                 detect::detect_board(picture.luma, board_used,
                                      detect::DetectOptions{&picture.half}));
      // A view the check refuses still shows what was found.
      VKC_ASSIGN(
          const quality::CheckedView checked,
          quality::check_view(cameras[i], board_used, found, options.view));
      check = checked.check;
      return {};
    });
  }

  // The set's pictures and frames copied out for the capture worker: each
  // camera's host luma, and its JPEG and depth where its frame has them.
  core::Result<CaptureRequest> capture_request(
      const source::PictureSet& set, const std::vector<source::HostLuma>& host,
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
    request.meta.spread_ms = static_cast<double>(set.spread_ns) / 1e6;
    request.meta.stillness_px = taken.stillness_px;
    const std::vector<source::SourceCamera>& sourced = source->cameras();
    request.cameras.resize(set.cameras.size());
    for (std::size_t i = 0; i < set.cameras.size(); ++i) {
      const detect::LumaView& luma = host[i].luma;
      if (!set.cameras[i].present || luma.data == nullptr) {
        return core::Status::invalid_argument("camera " + ids[i] +
                                              " has no picture in the set");
      }
      CameraFrameCopy& part = request.cameras[i];
      part.width = luma.width;
      part.height = luma.height;
      part.luma.resize(static_cast<std::size_t>(luma.width) * luma.height);
      for (std::uint32_t row = 0; row < luma.height; ++row) {
        std::memcpy(&part.luma[static_cast<std::size_t>(row) * luma.width],
                    luma.data + static_cast<std::size_t>(row) * luma.stride,
                    luma.width);
      }
      part.timestamp_ns = set.cameras[i].timestamp_ns;
      part.sequence = set.sequence;
      part.live = live.detections[i];
      const sensor::RgbdFrame* frame = set.rgbd != nullptr &&
                                               i < set.rgbd->frames.size() &&
                                               set.rgbd->frames[i]
                                           ? &*set.rgbd->frames[i]
                                           : nullptr;
      if (frame == nullptr) continue;
      part.sequence = frame->sequence;
      if (frame->color_bitstream.codec == sensor::ColorBitstream::Codec::Jpeg &&
          frame->color_bitstream.data != nullptr) {
        part.jpeg.assign(
            frame->color_bitstream.data,
            frame->color_bitstream.data + frame->color_bitstream.size);
      }
      if (!sourced[i].depth) continue;
      if (frame->depth == nullptr) {
        return core::Status::invalid_argument("camera " + ids[i] +
                                              " sent no depth with its frame");
      }
      const std::size_t samples =
          static_cast<std::size_t>(frame->depth_camera.size.width) *
          frame->depth_camera.size.height;
      part.depth.assign(frame->depth, frame->depth + samples);
      part.metres_per_unit = frame->metres_per_unit;
    }
    return request;
  }

  // One set: offered to the set sink, detected, checked, ruled on,
  // published. A set taken without luma -- the worker not detecting and no
  // capture armed -- or whose search is paused is offered and published
  // alone.
  core::Status process(source::PictureSet taken, core::StageMetrics stages,
                       bool pictures, const std::vector<std::string>& ids) {
    auto out = std::make_shared<LiveSnapshot>();
    out->stages = std::move(stages);
    out->set = std::make_shared<const source::PictureSet>(std::move(taken));
    const source::PictureSet& set = *out->set;
    // Decided before the offer, so a pause waits the offer out too.
    const bool searched = pictures && begin_search();
    if (sets != nullptr && set.rgbd != nullptr) sets->offer(set.rgbd);
    if (!searched) {
      out->detections.assign(ids.size(), Detection{});
      out->checks.assign(ids.size(), ViewCheck{});
      out->errors.assign(ids.size(), core::Status{});
      out->stillness_px.assign(ids.size(), std::nullopt);
      have_previous = false;  // no stillness across sets not searched
      publish(std::move(out));
      return {};
    }
    const std::vector<source::HostLuma>& host = source->host();
    {
      core::StageScope scope(out->stages, "detect");
      detect(set, host, *out);
    }
    end_search();
    for (std::size_t i = 0; i < out->errors.size(); ++i) {
      if (!out->errors[i].ok()) {
        out->errors[i] = with_camera(ids[i], out->errors[i]);
      }
    }

    // Stillness: each camera's corners against the set before.
    out->stillness_px.assign(ids.size(), std::nullopt);
    std::optional<double> motion;
    if (have_previous) {
      const core::Result<quality::CornerShift> shift = quality::corner_shift(
          previous, out->detections, quality::corner_match(*detector_board));
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
      live_set.present[i] = set.cameras[i].present;
    }
    live_set.checks = out->checks;
    live_set.motion_px = motion;
    live_set.spread_ms = static_cast<double>(set.spread_ns) / 1e6;
    live_set.saver_full = sink != nullptr && sink->full();
    VKC_ASSIGN(out->decision, rule->update(live_set, Clock::now()));
    if (out->decision.state == quality::CaptureState::Take) {
      take(set, host, *out, ids);
    }
    publish_decision(out->decision);
    publish(std::move(out));
    return {};
  }

  // A set's snapshot, with the source's counters and rate, published: newest
  // wins.
  void publish(std::shared_ptr<LiveSnapshot> out) {
    out->stats = source->stats();
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
    // A snapshot the window did not take is let go after the lock: it may
    // hold the last of its set's frames.
    std::shared_ptr<const LiveSnapshot> replaced;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      replaced = std::exchange(snapshot, std::move(out));
    }
  }

  // The rule took this set: copy it out and hand it to the sink.
  void take(const source::PictureSet& set,
            const std::vector<source::HostLuma>& host, LiveSnapshot& live,
            const std::vector<std::string>& ids) {
    core::Result<CaptureRequest> request =
        capture_request(set, host, live, live.decision, ids);
    core::Result<bool> queued = false;
    if (!request.ok()) {
      queued = request.status();
    } else {
      // Under the lock restart() takes: a capture of this session is handed
      // on before the next session starts, or not at all.
      const std::lock_guard<std::mutex> lock(mutex);
      queued = restart ? core::Result<bool>(core::Status::unsupported(
                             "a new session is starting: capture again"))
                       : sink->submit(std::move(request).value());
      if (queued.ok() && queued.value()) {
        captures.fetch_add(1);
        return;
      }
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
    VKC_TRY(source->start());
    while (!stopping.load()) {
      VKC_TRY(apply_requests());
      core::StageMetrics stages;
      // Pictures while they are shown, or a capture is armed; whether the
      // board is searched for in them is decided as each set comes
      // (begin_search).
      const bool pictures = detecting.load() || rule->armed();
      core::Result<std::optional<source::PictureSet>> taken =
          source->take(&stages, pictures);
      if (!taken.ok()) {
        // The sensors' failure ends the thread; a set's luma, that set.
        if (source->failed()) return taken.status();
        post(taken.status().with_context("live view"));
        continue;
      }
      if (taken.value()) {
        VKC_TRY(process(std::move(*taken.value()), std::move(stages), pictures,
                        ids));
        continue;
      }
      if (rule->armed()) {
        VKC_ASSIGN(const quality::CaptureDecision ticked,
                   rule->tick(Clock::now()));
        publish_decision(ticked);
      }
      std::this_thread::sleep_for(options.idle);
    }
    return {};
  }
};

core::Result<std::unique_ptr<LiveWorker>> LiveWorker::create(
    std::unique_ptr<source::RigSource> source,
    const std::vector<sensor::camera::CameraModel>& lenses, const Target& board,
    bool square_measured, CaptureSink* sink, SetSink* sets,
    const LiveOptions& options) {
  auto state = std::make_unique<State>(std::move(source));
  if (state->source == nullptr) {
    return core::Status::invalid_argument("LiveWorker: no source");
  }
  if (options.idle.count() < 0) {
    return core::Status::invalid_argument(
        "LiveWorker: the idle wait is negative");
  }
  return compute::guarded(
      "LiveWorker::create", [&]() -> core::Result<std::unique_ptr<LiveWorker>> {
        const std::vector<source::SourceCamera>& cameras =
            state->source->cameras();
        VKC_ASSIGN(state->cameras, live_lenses(cameras, lenses));
        std::vector<std::string> ids;
        for (const source::SourceCamera& camera : cameras) {
          ids.push_back(camera.id);
        }
        VKC_ASSIGN(compute::Board made, compute::Board::create(board));
        state->detector_board = std::move(made);
        state->options = options;
        state->skew_bound_ns = state->source->max_skew_ns();
        state->options.rule = source_rule(state->skew_bound_ns, options.rule);
        state->rule_options = state->options.rule;
        VKC_ASSIGN(quality::CaptureRule rule,
                   quality::CaptureRule::create(ids, state->options.rule));
        state->rule = std::move(rule);
        state->board = board;
        state->square_measured = square_measured;
        state->sink = sink;
        state->sets = sets;
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
  state_->end_search();  // a failure's search ends with the thread
  state_->source->stop();
  if (!ended.ok()) state_->post(ended);
  state_->running.store(false);
}

void LiveWorker::stop() noexcept {
  state_->stopping.store(true);
  if (state_->thread.joinable()) state_->thread.join();
}

bool LiveWorker::running() const noexcept { return state_->running.load(); }

quality::CaptureRuleOptions LiveWorker::rule_options() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->rule_options;
}

void LiveWorker::set_detecting(bool on) noexcept {
  state_->detecting.store(on);
}

void LiveWorker::pause_search(bool paused) {
  std::unique_lock<std::mutex> lock(state_->mutex);
  const bool pausing = paused && !state_->paused;
  state_->paused = paused;
  if (!pausing || !state_->searching) return;
  // The search under way ends first; one an armed capture starts after it
  // is not waited for.
  const std::uint64_t under_way = state_->searched;
  state_->search_ended.wait(lock,
                            [&] { return state_->searched != under_way; });
}

core::Status LiveWorker::restart(
    const std::vector<sensor::camera::CameraModel>& lenses, const Target& board,
    bool square_measured, const quality::CaptureRuleOptions& rule) {
  return compute::guarded("LiveWorker::restart", [&]() -> core::Status {
    // The source's camera list is fixed once open, so any thread may read it.
    VKC_ASSIGN(std::vector<compute::CvCamera> cameras,
               live_lenses(state_->source->cameras(), lenses));
    VKC_ASSIGN(compute::Board made, compute::Board::create(board));
    const quality::CaptureRuleOptions options =
        source_rule(state_->skew_bound_ns, rule);
    VKC_ASSIGN(quality::CaptureRule fresh,
               quality::CaptureRule::create(ids_, options));
    const std::lock_guard<std::mutex> lock(state_->mutex);
    state_->restart =
        Restart{std::move(cameras), std::move(made), std::move(fresh)};
    state_->rule_options = options;
    state_->board = board;
    state_->square_measured = square_measured;
    state_->press.reset();
    state_->next_board.reset();
    state_->captures.store(0);
    return {};
  });
}

bool LiveWorker::arm() {
  if (state_->sink == nullptr || !running()) return false;
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
