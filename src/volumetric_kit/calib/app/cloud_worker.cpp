// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/cloud_worker.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::app {

namespace {

namespace sensor = recon::sensor;
using SetPtr = std::shared_ptr<const sensor::FrameSet>;
using SnapshotPtr = std::shared_ptr<const CloudSnapshot>;

}  // namespace

struct CloudWorker::State {
  explicit State(core::Device& recon_device) : device(recon_device) {}

  // The worker's thread's alone, once it runs.
  core::Device& device;
  std::vector<sensor::GpuFramePrep> preps;  // one per camera
  core::TimelineSemaphore ready;            // each snapshot's writes done
  std::uint64_t signalled = 0;              // the last value set on it
  std::uint64_t published = 0;
  std::uint32_t live_every = 1;

  // Shared with the live worker's and the window's threads.
  mutable std::mutex mutex;
  std::condition_variable wake;
  SetPtr pending;        // offered, not yet taken
  SnapshotPtr snapshot;  // newest, not yet taken
  std::vector<core::Status> errors;
  std::uint64_t offers = 0;  // counted for live_every while live
  bool live = true;
  bool color = true;
  bool stopping = false;
  std::atomic<std::uint64_t> prepared{0};
  std::thread thread;

  void post(core::Status status) {
    const std::lock_guard<std::mutex> lock(mutex);
    errors.push_back(std::move(status));
  }

  // One set prepared: each frame at the world's origin, its colour left out
  // without `with_color`, in one batch; the set let go; the writes' end set
  // on the timeline.
  core::Result<SnapshotPtr> prepare(SetPtr set, bool with_color) {
    if (set->frames.size() != preps.size()) {
      return core::Status::invalid_argument(
          "a set of " + std::to_string(set->frames.size()) + " frames for " +
          std::to_string(preps.size()) + " cameras");
    }
    auto out = std::make_shared<CloudSnapshot>();
    out->timestamp_ns = set->timestamp_ns;
    out->sequence = set->sequence;
    out->color = with_color;
    {
      std::vector<std::optional<sensor::RgbdFrame>> frames(set->frames.size());
      for (std::size_t i = 0; i < frames.size(); ++i) {
        if (!set->frames[i]) continue;
        sensor::RgbdFrame& frame = frames[i].emplace(*set->frames[i]);
        // The window poses each camera when it draws.
        frame.color_to_world = recon::camera::Mat4d(1.0);
        if (!with_color) {
          frame.color = {};
          frame.color_packed = nullptr;
        }
      }
      VKC_ASSIGN(out->frames, sensor::GpuFramePrep::prepare_batch(
                                  preps, frames, &out->stages));
    }
    set.reset();  // nothing of the set is held past its preparation
    // The preparation waited for its fence, which makes no device write
    // visible to another queue: a signal after it on the same queue gives
    // the renderer's frame a semaphore to wait on that does.
    const std::uint64_t value = signalled + 1;
    VKC_ASSIGN(
        core::Device::PendingSubmit done,
        device.submit_pending([](VkCommandBuffer) {}, {}, {{&ready, value}}));
    signalled = value;
    VKC_TRY(done.wait());
    out->ready = {&ready, value};
    out->number = ++published;
    return SnapshotPtr(std::move(out));
  }

  core::Status loop() {
    for (;;) {
      SetPtr set;
      bool with_color = true;
      {
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait(lock, [this] { return stopping || pending != nullptr; });
        if (stopping) return {};
        set = std::move(pending);
        with_color = color;
      }
      core::Result<SnapshotPtr> made =
          compute::guarded("3D view", [&]() -> core::Result<SnapshotPtr> {
            return prepare(std::move(set), with_color);
          });
      if (!made.ok()) {
        post(made.status().with_context("3D view"));
        continue;
      }
      SnapshotPtr replaced;
      {
        const std::lock_guard<std::mutex> lock(mutex);
        // A snapshot finished after the view froze is not shown.
        if (live) {
          replaced = std::move(snapshot);
          snapshot = std::move(made).value();
        }
      }
      prepared.fetch_add(1);  // counted once it is there to take
    }
  }
};

core::Result<std::unique_ptr<CloudWorker>> CloudWorker::create(
    core::Device& device, core::Allocator& allocator, std::size_t cameras,
    const std::vector<std::uint32_t>& families, const CloudOptions& options) {
  return compute::guarded(
      "CloudWorker::create",
      [&]() -> core::Result<std::unique_ptr<CloudWorker>> {
        if (cameras == 0) {
          return core::Status::invalid_argument("CloudWorker: no camera");
        }
        if (options.live_every == 0) {
          return core::Status::invalid_argument("CloudWorker: live_every is 0");
        }
        if (families.size() > core::BufferDesc::kMaxQueueFamilies) {
          return core::Status::invalid_argument(
              "CloudWorker: " + std::to_string(families.size()) +
              " queue families, more than " +
              std::to_string(core::BufferDesc::kMaxQueueFamilies));
        }
        // Colour and depth both read in place by the renderer.
        sensor::GpuFramePrepConfig config;
        for (std::size_t i = 0; i < families.size(); ++i) {
          config.color_queue_families[i] = families[i];
          config.depth_queue_families[i] = families[i];
        }
        config.color_queue_family_count =
            static_cast<std::uint32_t>(families.size());
        config.depth_queue_family_count =
            static_cast<std::uint32_t>(families.size());
        auto state = std::make_unique<State>(device);
        state->preps.reserve(cameras);
        for (std::size_t c = 0; c < cameras; ++c) {
          VKC_ASSIGN(sensor::GpuFramePrep prep,
                     sensor::GpuFramePrep::create(device, allocator, config));
          state->preps.push_back(std::move(prep));
        }
        VKC_ASSIGN(state->ready, core::TimelineSemaphore::create(device, 0));
        state->live_every = options.live_every;
        std::unique_ptr<CloudWorker> worker(new CloudWorker(std::move(state)));
        try {
          worker->state_->thread =
              std::thread([w = worker.get()] { w->run(); });
        } catch (const std::system_error& e) {
          return core::Status::io_error(
              std::string("CloudWorker: the thread will not start: ") +
              e.what());
        }
        return worker;
      });
}

CloudWorker::CloudWorker(std::unique_ptr<State> state)
    : state_(std::move(state)) {}

CloudWorker::~CloudWorker() { stop(); }

void CloudWorker::run() noexcept {
  const core::Status ended = compute::guarded(
      "3D view", [this]() -> core::Status { return state_->loop(); });
  if (!ended.ok()) state_->post(ended);
}

void CloudWorker::offer(SetPtr set) {
  SetPtr replaced;  // let go after the lock, as `set` is when skipped
  {
    const std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->stopping || !state_->live) return;
    if (state_->offers++ % state_->live_every != 0) return;
    replaced = std::move(state_->pending);
    state_->pending = std::move(set);
  }
  state_->wake.notify_one();
}

void CloudWorker::set_live(bool live) {
  SetPtr dropped;
  SnapshotPtr unseen;
  const std::lock_guard<std::mutex> lock(state_->mutex);
  if (state_->live == live) return;
  state_->live = live;
  state_->offers = 0;  // going live prepares the next set offered
  if (!live) {
    dropped = std::move(state_->pending);
    unseen = std::move(state_->snapshot);
  }
}

bool CloudWorker::live() const {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->live;
}

void CloudWorker::set_color(bool color) {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  state_->color = color;
}

std::shared_ptr<const CloudSnapshot> CloudWorker::take_snapshot() {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return std::move(state_->snapshot);
}

std::uint64_t CloudWorker::prepared() const noexcept {
  return state_->prepared.load();
}

std::vector<core::Status> CloudWorker::take_errors() {
  const std::lock_guard<std::mutex> lock(state_->mutex);
  std::vector<core::Status> out;
  out.swap(state_->errors);
  return out;
}

void CloudWorker::stop() noexcept {
  SetPtr dropped;
  {
    const std::lock_guard<std::mutex> lock(state_->mutex);
    state_->stopping = true;
    dropped = std::move(state_->pending);
  }
  state_->wake.notify_all();
  if (state_->thread.joinable()) state_->thread.join();
}

}  // namespace volumetric_kit::calib::app
