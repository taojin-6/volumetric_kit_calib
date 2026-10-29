// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file live_worker.hpp
/// @brief calib_rig's acquisition and live detection (its T1 thread): the
///        rig's sets, each camera's luma read back, the board found and each
///        view checked, the capture rule driven, and the newest result handed
///        to the window.
///
/// The worker is the only user of the rig's `SensorArray`, which is not
/// thread-safe. Each set it polls is read back to the host in one submit
/// (sensor's `PictureReadback`, luma alone), the board is found in every
/// camera's luma at once (hybrid detection, then `quality::check_view`), and
/// the result is published as a @ref LiveSnapshot: newest wins, so a slow
/// window skips sets rather than queueing them, and the worker holds at most
/// the set it is on and the one published. A capture the rule takes is copied
/// out of its set and handed to a @ref CaptureSink, so the set's frames go
/// back to the driver at once. Each set is also offered, before it is
/// searched (@ref LiveWorker::set_detecting), to a @ref SetSink: the 3D
/// view's worker, which prepares its depth.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/core/capture_meta.hpp"
#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/core/view_check.hpp"
#include "volumetric_kit/calib/quality/capture_rule.hpp"
#include "volumetric_kit/calib/quality/live_checks.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"

namespace volumetric_kit::core {
class Allocator;
class Device;
}  // namespace volumetric_kit::core

namespace volumetric_kit::calib::app {

/// @brief One camera's part of a capture, copied out of its set.
///
/// @code
/// const CameraFrameCopy& part = request.cameras[i];
/// const detect::LumaView luma{part.width, part.height, part.width,
///                             part.luma.data()};
/// @endcode
struct CameraFrameCopy {
  std::uint32_t width = 0;   ///< The luma's width (pixels).
  std::uint32_t height = 0;  ///< The luma's height (rows).
  /// The luma as the live view read it back, rows packed: what the capture's
  /// full-resolution detection searches.
  std::vector<std::uint8_t> luma;
  /// The camera's JPEG, byte for byte (`RgbdFrame::color_bitstream`).
  std::vector<std::uint8_t> jpeg;
  /// The depth, a sample per depth pixel, row-major, in the sensor's units
  /// (`RgbdFrame::depth`).
  std::vector<std::uint16_t> depth;
  /// What one depth unit is (metres): `RgbdFrame::metres_per_unit`.
  float metres_per_unit = 0.0f;
  std::uint64_t timestamp_ns = 0;  ///< `RgbdFrame::timestamp_ns`.
  std::uint64_t sequence = 0;      ///< `RgbdFrame::sequence`.
  Detection live;  ///< The live (hybrid) detection, for the live-vs-full flag.
};

/// @brief A capture the rule took: every camera's frame of one set, copied
///        out of it, and the set's facts.
///
/// @code
/// for (const CameraFrameCopy& part : request.cameras) {
///   VKC_ASSIGN(Detection full, detect::detect_charuco(
///                                  {part.width, part.height, part.width,
///                                   part.luma.data()},
///                                  board, detect::Mode::Full));
/// }
/// @endcode
struct CaptureRequest {
  /// The board the live view searched for; the same for every capture of a
  /// session, as it locks at the first.
  Target board;
  /// Whether its square size was measured.
  bool square_measured = false;
  /// The set's facts: its wall-clock time, trigger, sequence, spread and
  /// stillness. The writer numbers it, and `live_vs_full_px` is the full
  /// detection's to fill.
  CaptureMeta meta;
  std::vector<CameraFrameCopy> cameras;  ///< One per camera, in rig order.
};

/// @brief Where a @ref LiveWorker hands the captures its rule takes: the
///        capture worker (@ref CaptureWorker), or a test's stand-in.
///
/// Both members are called from the live worker's thread while the window's
/// thread may call the sink's own members: an implementation is thread-safe.
///
/// @code
/// class Collect final : public CaptureSink {
///  public:
///   bool full() const override { return false; }
///   core::Result<bool> submit(CaptureRequest request) override {
///     requests.push_back(std::move(request));  // under a mutex
///     return true;
///   }
///   std::vector<CaptureRequest> requests;
/// };
/// @endcode
class CaptureSink {
 public:
  virtual ~CaptureSink() = default;

  /// @return Whether it holds as many captures as it may: the capture rule's
  ///         `quality::LiveSet::saver_full`.
  virtual bool full() const = 0;

  /// @brief Queue a capture.
  /// @param request  The capture.
  /// @return `true` once queued; `false`, with nothing queued, when full; or
  ///         why it was refused.
  virtual core::Result<bool> submit(CaptureRequest request) = 0;

 protected:
  CaptureSink() = default;
  CaptureSink(const CaptureSink&) = default;
  CaptureSink& operator=(const CaptureSink&) = default;
};

/// @brief Where a @ref LiveWorker offers each set, before it searches it
///        (@ref LiveWorker::set_detecting): the 3D view's worker
///        (@ref CloudWorker), or a test's stand-in.
///
/// Called from the live worker's thread while the window's thread may call
/// the sink's own members: an implementation is thread-safe, and returns at
/// once, keeping the set no longer than it needs it, since every set held
/// keeps its frames from the driver.
///
/// @code
/// class Keep final : public SetSink {
///  public:
///   void offer(std::shared_ptr<const sensor::FrameSet> set) override {
///     latest = std::move(set);  // under a mutex
///   }
///   std::shared_ptr<const sensor::FrameSet> latest;
/// };
/// @endcode
class SetSink {
 public:
  virtual ~SetSink() = default;

  /// @brief Take a set the live worker polled.
  /// @param set  The set, shared with the live worker's snapshot of it.
  virtual void offer(std::shared_ptr<const sensor::FrameSet> set) = 0;

 protected:
  SetSink() = default;
  SetSink(const SetSink&) = default;
  SetSink& operator=(const SetSink&) = default;
};

/// @brief What the live view knows of one set: the frames, where the board is
///        in each, what each view is worth, and how the rig keeps up.
///
/// It holds the set, so its frames -- the decoded pictures the window draws
/// -- stay alive while it does; the window holds it until the GPU is done
/// with the frame that copied them.
///
/// @code
/// if (std::shared_ptr<const LiveSnapshot> s = live->take_snapshot()) {
///   for (std::size_t i = 0; i < s->detections.size(); ++i) {
///     draw_corners(s->detections[i]);  // a caller's own
///   }
/// }
/// @endcode
struct LiveSnapshot {
  std::uint64_t number = 0;  ///< The sets the worker has published, from 1.
  /// The set, its frames as the driver handed them out.
  std::shared_ptr<const sensor::FrameSet> set;
  /// Each camera's live (hybrid) detection, in rig order: empty where the set
  /// has no frame, its frame no picture, or the board was not found, and
  /// every camera's while the set is not searched
  /// (@ref LiveWorker::set_detecting).
  std::vector<Detection> detections;
  /// Each camera's view, checked (`quality::check_view`); a default one where
  /// the detection or the check failed.
  std::vector<ViewCheck> checks;
  /// Why each camera's detection or check failed; OK where it did not.
  std::vector<core::Status> errors;
  /// How far each camera's corners moved since the set before (median,
  /// pixels); empty where no corner is in both.
  std::vector<std::optional<double>> stillness_px;
  /// The frames each camera's driver lost (an Orbbec camera's
  /// `OrbbecStreamStats::lost`); empty where the driver does not count them.
  std::vector<std::optional<std::uint64_t>> lost;
  sensor::SensorArrayStats stats;     ///< The array's counters.
  quality::CaptureDecision decision;  ///< The capture rule's answer.
  /// The stages' times: `"poll"`, then, for a set searched, `"readback"`
  /// (with its device time) and `"detect"`.
  core::StageMetrics stages;
  /// Sets a second, smoothed over the last second or so.
  double sets_per_second = 0.0;
};

/// @brief How the live worker detects, checks and captures.
///
/// @code
/// LiveOptions options;
/// options.rule.timeout = std::chrono::seconds(5);
/// @endcode
struct LiveOptions {
  quality::ViewCheckOptions view;    ///< Each view's check.
  quality::CaptureRuleOptions rule;  ///< The capture rule's thresholds.
  /// How long the worker waits before it polls again when no set is ready.
  std::chrono::milliseconds idle{2};
};

/// @brief The live worker: a thread that owns the rig's `SensorArray` and its
///        picture readback, and publishes a @ref LiveSnapshot per set.
///
/// Every other member is for the window's thread. The worker starts the
/// array on its own thread; a failure there, a sensor's failure while
/// polling -- a camera disconnected among them, as sensor's Orbbec driver
/// fails every poll after it -- or anything its thread throws ends it,
/// posting why (@ref take_errors). A set it cannot read back, or a camera it
/// cannot detect in, is reported and skipped, and the live view goes on.
///
/// @warning The device and allocator the readback uses, the capture sink and
///          the set sink must outlive the worker. Release every snapshot, and
///          have the set sink let go of every set it was offered
///          (@ref CloudWorker::stop), before destroying it, so the frames go
///          back to their sensors before the array closes them.
///
/// @code
/// VKC_ASSIGN(std::unique_ptr<LiveWorker> live,
///            LiveWorker::create(std::move(array), device, allocator, board,
///                               false, capture_worker.get()));
/// // Each frame of the window:
/// if (pressed) live->arm();
/// if (auto snapshot = live->take_snapshot()) latest = std::move(snapshot);
/// for (const core::Status& error : live->take_errors()) show(error);
/// // At the end:
/// live->stop();
/// latest.reset();
/// @endcode
class LiveWorker {
 public:
  /// @brief Take the rig and start its thread.
  /// @param array            The rig, opened and not started; taken whatever
  ///                         the outcome. Every sensor needs a colour model.
  /// @param device           The device its pictures are on (the compute
  ///                         library's), for the readback.
  /// @param allocator        Its allocator, for the readback's host memory.
  /// @param board            The board to find, which `compute::Board`
  ///                         takes.
  /// @param square_measured  Whether its square size was measured.
  /// @param sink             Where captures go; null for a live view that
  ///                         captures nothing (@ref arm refuses).
  /// @param sets             Where each set is offered (@ref SetSink); null
  ///                         for none.
  /// @param options          How it detects, checks and captures.
  /// @return The worker, its thread running; `InvalidArgument` for a sensor
  ///         without a colour model (naming it), a board or options refused,
  ///         or an idle wait that is negative; the readback's failure;
  ///         `IoError` if the thread cannot start; or `OutOfMemory`.
  static core::Result<std::unique_ptr<LiveWorker>> create(
      sensor::SensorArray&& array, const core::Device& device,
      core::Allocator& allocator, const Target& board, bool square_measured,
      CaptureSink* sink, SetSink* sets = nullptr,
      const LiveOptions& options = {});

  /// @brief As @ref stop, then release the array, closing its sensors.
  ~LiveWorker();
  LiveWorker(const LiveWorker&) = delete;
  LiveWorker& operator=(const LiveWorker&) = delete;
  LiveWorker(LiveWorker&&) = delete;
  LiveWorker& operator=(LiveWorker&&) = delete;

  /// @brief Stop the thread and wait for it; the array is stopped, and the
  ///        set it was on released. Idempotent.
  void stop() noexcept;

  /// @return Whether the thread still polls the rig.
  bool running() const noexcept;

  /// @return Each camera's id, in rig order.
  const std::vector<std::string>& ids() const noexcept { return ids_; }

  /// @brief Whether to search each set for the board: on, the default,
  ///        while the window shows the camera images; off while it shows the
  ///        3D view alone.
  ///
  /// Off, each set is still polled, offered to the set sink and published
  /// with the rig's counters, but nothing is read back, detected or ruled on
  /// -- unless a capture is armed, which goes on being searched for.
  /// @param on  Whether to search.
  void set_detecting(bool on) noexcept;

  /// @brief Arm a capture: a press, at this moment.
  /// @return `true` once the worker will arm its rule; `false` with no
  ///         capture sink, or once the thread has ended (@ref running).
  bool arm();

  /// @brief Search for another board from the next set on, until the first
  ///        capture locks it.
  /// @param board            The board, which `compute::Board` takes.
  /// @param square_measured  Whether its square size was measured.
  /// @return OK; `InvalidArgument` once a capture was taken, or for a board
  ///         `compute::Board::create` refuses; or `OutOfMemory`.
  core::Status set_board(const Target& board, bool square_measured);

  /// @return The board searched for; once a capture was taken, the one every
  ///         capture used.
  Target board() const;
  /// @return Whether its square size was measured.
  bool square_measured() const;

  /// @return The captures handed to the sink so far.
  std::uint64_t captures() const noexcept;

  /// @brief Take the newest snapshot not yet taken.
  /// @return It; null when the worker published none since the last call.
  std::shared_ptr<const LiveSnapshot> take_snapshot();

  /// @return The capture rule's latest answer that was not `Idle`: a set's,
  ///         or a timeout's while no set came.
  quality::CaptureDecision decision() const;

  /// @brief Take what went wrong since the last call: sets that could not be
  ///        read back, captures the sink refused, and why the thread ended.
  /// @return The failures, oldest first.
  std::vector<core::Status> take_errors();

 private:
  struct State;
  LiveWorker(std::unique_ptr<State> state, std::vector<std::string> ids);
  void run() noexcept;

  std::unique_ptr<State> state_;
  std::vector<std::string> ids_;
};

}  // namespace volumetric_kit::calib::app
