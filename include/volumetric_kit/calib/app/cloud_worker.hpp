// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file cloud_worker.hpp
/// @brief calib_rig's 3D-view worker (its T2 thread): each camera's frame of
///        a live set prepared on recon's queue family -- undistorted depth,
///        and colour unless the view is solid -- and handed to the window,
///        which draws its points in place.
///
/// The live worker offers each set it reads back (@ref SetSink); this worker
/// takes the newest, every `live_every`-th, prepares it with recon's
/// `GpuFramePrep` (one per camera) in one batch, lets the set go, and
/// publishes a @ref CloudSnapshot: newest wins. A snapshot carries no poses:
/// it is prepared with the colour camera at the world's origin, and the
/// window places each camera at the rig's poses when it draws
/// (`viz::cloud_draws`), so a frozen snapshot follows a new solve with no
/// second preparation. Its buffers are shared with the renderer's queue
/// family, and the window's frame waits on @ref CloudSnapshot::ready before
/// its vertices read them.
// TODO: CA12b gives this worker the depth checks (calib_depth's DepthChecks)
// and the captured snapshot's JPEG decode (recon's JpegDecoder), as the sole
// owner of both, with one live set between check captures.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "volumetric_kit/calib/app/export.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"

namespace volumetric_kit::core {
class Allocator;
class Device;
}  // namespace volumetric_kit::core

namespace volumetric_kit::calib::app {

/// @brief One live set's frames, prepared on the device for the 3D view.
///
/// It holds the prepared buffers, so the window holds it until the GPU is done
/// with every frame that drew it (gfx's `RetireQueue`); the set itself is
/// gone.
///
/// @code
/// if (std::shared_ptr<const CloudSnapshot> s = cloud->take_snapshot()) {
///   cloud_latest = std::move(s);
/// }
/// // Each frame that draws it:
/// frame.waits.push_back({cloud_latest->ready,
///                        VK_PIPELINE_STAGE_VERTEX_SHADER_BIT});
/// @endcode
struct CloudSnapshot {
  std::uint64_t number = 0;        ///< Snapshots published, from 1.
  std::uint64_t timestamp_ns = 0;  ///< The set's trigger time.
  std::uint64_t sequence = 0;      ///< The set's sequence number.
  /// Whether colour was prepared: false for a set prepared in the view's
  /// solid mode, whose frames hold depth only.
  bool color = false;
  /// Each camera's prepared frame, in the rig's order: its depth (metres) and
  /// colour shared with the renderer's queue family, posed with the colour
  /// camera at the origin. Empty where the set had no frame from it.
  std::vector<std::optional<recon::sensor::DeviceFrame>> frames;
  /// Reached once the preparation's writes are complete, on the worker's
  /// timeline: a frame that draws the snapshot waits for it at
  /// `VERTEX_SHADER`, which makes the writes visible to its vertices. Already
  /// reached when the snapshot is published.
  core::TimelinePoint ready;
  /// The preparation's times: recon's `"frame prep"` row, host and device.
  core::StageMetrics stages;
};

/// @brief How the 3D-view worker samples the live sets.
///
/// @code
/// CloudOptions options;
/// options.live_every = 3;  // a third of the sets
/// @endcode
// TODO: take live_every from the core's calib_rig view settings once the
// scenario has them (CO5).
struct CloudOptions {
  /// Prepare the first set offered and every `live_every`-th after it; at
  /// least 1.
  std::uint32_t live_every = 2;
};

/// @brief The 3D-view worker: a thread that alone owns each camera's
///        `GpuFramePrep`, and publishes a @ref CloudSnapshot per set it
///        prepares.
///
/// @ref offer is for the live worker's thread; every other member is for the
/// window's. A set the preparation refuses is reported (@ref take_errors) and
/// skipped, and the worker goes on; so is anything its thread throws for one
/// set.
///
/// @warning The device and allocator must outlive the worker and every
///          snapshot, and every snapshot must be released before the worker
///          is destroyed: its @ref CloudSnapshot::ready is the worker's. As
///          a live worker's set sink, it is bound by @ref LiveWorker's
///          warning too.
///
/// @code
/// VKC_ASSIGN(std::unique_ptr<CloudWorker> cloud,
///            CloudWorker::create(recon_device, recon_allocator, cameras,
///                                {compute_family, graphics_family}));
/// // The live worker offers it each set (LiveWorker::create's `sets`).
/// // Each frame of the window:
/// cloud->set_live(cloud_live(frozen, placement, minimized));
/// cloud->set_color(mode != gfx::pipelines::CloudColor::Solid);
/// if (auto s = cloud->take_snapshot()) cloud_latest = std::move(s);
/// @endcode
class VC_APP_API CloudWorker final : public SetSink {
 public:
  /// @brief Build each camera's preparation and start the thread.
  /// @param device     recon's device, on the compute library's queue family,
  ///                   where the cameras' pictures are; it enabled recon's
  ///                   requirements.
  /// @param allocator  Its allocator, for the prepared buffers.
  /// @param cameras    The rig's cameras; at least one.
  /// @param families   The queue families that read the prepared frames:
  ///                   recon's and the renderer's, at most
  ///                   `core::BufferDesc::kMaxQueueFamilies`; a duplicate
  ///                   collapses, so one family leaves them EXCLUSIVE.
  /// @param options    How the live sets are sampled.
  /// @return The worker, its thread waiting for a set; `InvalidArgument` for
  ///         no camera, a `live_every` of 0, or what `GpuFramePrep::create`
  ///         refuses (too many families, or one the device lacks); a
  ///         preparation's or the timeline's failure; `IoError` if the thread
  ///         cannot start; or `OutOfMemory`.
  static core::Result<std::unique_ptr<CloudWorker>> create(
      core::Device& device, core::Allocator& allocator, std::size_t cameras,
      const std::vector<std::uint32_t>& families,
      const CloudOptions& options = {});

  /// @brief As @ref stop.
  ~CloudWorker() override;
  CloudWorker(const CloudWorker&) = delete;
  CloudWorker& operator=(const CloudWorker&) = delete;
  CloudWorker(CloudWorker&&) = delete;
  CloudWorker& operator=(CloudWorker&&) = delete;

  /// @brief Take a set to prepare: kept until the thread takes it, replacing
  ///        one it has not taken; let go at once while the worker is not live
  ///        or skips it (`live_every`).
  /// @param set  The set, one frame per camera, in the rig's order.
  void offer(std::shared_ptr<const recon::sensor::FrameSet> set) override;

  /// @brief Whether sets offered are prepared: false while the view is
  ///        frozen, hidden or minimized (@ref cloud_live), when offers are
  ///        let go and the window keeps its last snapshot. Going live again
  ///        prepares the next set offered.
  /// @param live  Whether to prepare.
  void set_live(bool live);
  /// @return Whether sets offered are prepared.
  bool live() const;

  /// @brief Whether sets are prepared with colour: false in the view's solid
  ///        mode, when only depth is prepared, from the next set taken.
  /// @param color  Whether to prepare colour.
  void set_color(bool color);

  /// @brief Take the newest snapshot not yet taken.
  /// @return It; null when the worker published none since the last call.
  std::shared_ptr<const CloudSnapshot> take_snapshot();

  /// @return The sets prepared so far; each one's snapshot is published, or
  ///         let go when the worker was no longer live, before it is counted.
  std::uint64_t prepared() const noexcept;

  /// @brief Take what went wrong since the last call: sets the preparation
  ///        refused.
  /// @return The failures, oldest first.
  std::vector<core::Status> take_errors();

  /// @brief Stop the thread and wait for it, letting go of a set not yet
  ///        taken. Idempotent.
  void stop() noexcept;

 private:
  struct State;
  explicit CloudWorker(std::unique_ptr<State> state);
  void run() noexcept;

  std::unique_ptr<State> state_;
};

}  // namespace volumetric_kit::calib::app
