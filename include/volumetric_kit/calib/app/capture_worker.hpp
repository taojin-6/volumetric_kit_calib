// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file capture_worker.hpp
/// @brief calib_rig's capture worker (its T3 thread): each capture the live
///        view takes is detected again at full resolution, checked and handed
///        to the session writer, off the window's thread.
///
/// The live view finds the board by the faster hybrid; the solver takes full-
/// resolution corners, so each capture's luma is searched again here (Full
/// mode) and its views checked, the two detections compared
/// (`quality::live_vs_full_flagged`), and the capture queued on the capture
/// tier's `SessionWriter`, which writes it on a thread of its own. The
/// session's directory is created at the first capture, under the scenario's
/// `output.path`. Captures and drops are carried out in the order they come,
/// and each one's outcome is reported (@ref take_events), with the session's
/// checks once it is written (@ref summary) and the captures kept for
/// Calibrate (@ref kept).

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/app/export.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/calib/core/capture_meta.hpp"
#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/quality/live_checks.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"

namespace volumetric_kit::calib::app {

/// @brief What calib_rig records a session was captured with: calib's,
///        the core's, recon's and gfx's commits as this build fetched them
///        (empty where a checkout is not a git one), and OpenCV's and the
///        Orbbec SDK's versions.
/// @return The versions.
///
/// @code
/// CaptureSetup setup;
/// setup.software = app_software();
/// @endcode
VC_APP_API capture::Software app_software();

/// @brief One camera of the rig, as its session records it.
///
/// @code
/// for (std::size_t i = 0; i < array.size(); ++i) {
///   VKC_ASSIGN(capture::SessionCamera camera,
///              session_camera(array.sensor(i), sync));
///   setup.cameras.push_back(std::move(camera));
/// }
/// @endcode
/// @param camera  The sensor. An Orbbec camera's firmware, IP address, sync
///                mode and lens models come from its `device_info`; another
///                sensor's are left empty.
/// @param sync    The rig's sync configuration: the camera's depth delay, by
///                serial (0 for a camera it does not list).
/// @return The camera; or `OutOfMemory`.
VC_APP_API core::Result<capture::SessionCamera> session_camera(
    const recon::sensor::IRgbdSensor& camera,
    const recon::sensor::OrbbecRigSyncConfig& sync);

/// @brief What the capture worker writes, and how it judges each capture.
///
/// @code
/// CaptureSetup setup;
/// setup.root = config.output.path;
/// setup.software = app_software();
/// setup.sync_path = config.source.sync_config;
/// setup.reference = array.primary();
/// setup.depth_mode = preset.depth_mode;
/// setup.cameras = cameras;  // session_camera's, in rig order
/// @endcode
struct CaptureSetup {
  /// The existing directory each session's own directory
  /// (`YYYY-MM-DD_HHMMSS`, `capture::session_stamp`) is created in.
  std::string root;
  capture::Software software;  ///< What the session was captured with.
  std::string sync_path;       ///< The rig's sync configuration file.
  std::size_t reference = 0;   ///< The reference camera, the rig's primary.
  /// The rig's depth mode: its name and range. Its unit is the first
  /// capture's frames' (`CameraFrameCopy::metres_per_unit`), and a later
  /// capture in another unit fails.
  capture::DepthMode depth_mode;
  /// The cameras, in rig order, as every capture gives them: each with a
  /// colour model, which the full detection's check takes.
  std::vector<capture::SessionCamera> cameras;
  /// Each full view's check (`view`), and the session's thresholds.
  quality::SessionOptions session;
  quality::LiveVsFullOptions live_vs_full;  ///< When a capture is flagged.
  /// Captures held at once: queued here, being detected, or queued on the
  /// writer; at least 1.
  std::size_t capacity = 4;
  /// Captures kept and pending at once, at most: what one Calibrate adjusts,
  /// which also bounds the raw depth held (@ref KeptCapture::depth); at
  /// least 1.
  // TODO: more than 150 captures, should a session ever need them.
  std::size_t max_kept = solve::BundleAdjustOptions{}.max_captures;
};

/// @brief A capture kept for Calibrate: each camera's full detection and raw
///        depth, as the session wrote them.
///
/// @code
/// for (std::size_t c = 0; c < capture.views.size(); ++c) {
///   const Observation& view = capture.views[c].observation;
///   if (!view.empty()) observations.push_back(view);  // the solve's input
///   const std::vector<std::uint16_t>& depth = *capture.depth[c];
/// }
/// @endcode
struct KeptCapture {
  int index = -1;  ///< As the session numbers it (`captures/<NNN>.json`).
  /// Each camera's board found at full resolution, in rig order: the
  /// detection its file holds, with `observation.camera_id` the camera's
  /// serial and `observation.frame_id` the capture's index. A camera that
  /// found no board, or whose detection failed, has no corners.
  std::vector<Detection> views;
  /// Each camera's raw depth, in rig order, not null: the very samples the
  /// session wrote as its PNG, shared with the writer, never copied.
  std::vector<std::shared_ptr<const std::vector<std::uint16_t>>> depth;
};

/// @brief What Calibrate works on: the session's kept captures and what they
///        were captured with.
///
/// Immutable once published (@ref CaptureWorker::kept), so any thread may
/// read it, and it outlives the worker. Its captures are shared with later
/// snapshots, so each publish copies pointers, not captures; a dropped
/// capture's depth is freed once no snapshot holds it.
///
/// @code
/// std::shared_ptr<const KeptCaptures> kept = captures->kept();
/// VKC_ASSIGN(const compute::Board board, compute::Board::create(kept->board));
/// const solve::RigCamera& reference = kept->cameras[kept->reference];
/// @endcode
struct KeptCaptures {
  Target board;  ///< The session's board, locked at its first capture.
  /// The rig's colour cameras, in rig order, as the session's checks take
  /// them: each one's serial and colour model.
  std::vector<solve::RigCamera> cameras;
  /// Each camera's sensor, in rig order (`CaptureSetup::cameras`): its depth
  /// model and `depth_to_color` among them.
  std::vector<recon::sensor::SensorInfo> sensors;
  std::size_t reference = 0;  ///< The reference camera, by index.
  /// The rig's depth mode, in the session's depth unit.
  capture::DepthMode depth_mode;
  /// The solve's thresholds, the session's (`CaptureSetup::session.solve`).
  solve::InitialiseOptions solve;
  std::string directory;  ///< The session's directory.
  /// Every capture written and not dropped, by index ascending.
  std::vector<std::shared_ptr<const KeptCapture>> captures;
};

/// @brief What became of a capture or a drop.
///
/// @code
/// for (const CaptureEvent& event : worker->take_events()) {
///   if (event.kind == CaptureEvent::Kind::Saved) flash(event.meta.index);
///   if (event.kind == CaptureEvent::Kind::Failed) show(event.status);
/// }
/// @endcode
struct CaptureEvent {
  /// @brief What happened.
  enum class Kind {
    Saved,    ///< A capture is on disk.
    Dropped,  ///< A capture was dropped.
    Failed,   ///< A capture or a drop failed: @ref status says why.
  };
  Kind kind = Kind::Failed;  ///< What happened.
  core::Status status;       ///< OK, or why it failed.
  /// The capture's facts: its index (-1 when none was written or dropped),
  /// times, spread, stillness and live-vs-full shift.
  CaptureMeta meta;
  int usable_views = 0;  ///< Saved: its usable views at full resolution.
  /// Saved: its live corners disagree with its full ones.
  bool flagged = false;
};

/// @brief The capture worker: a thread that detects each capture at full
///        resolution and queues it on the session writer.
///
/// Every member may be called from any thread. Destroying the worker first
/// carries out every capture and drop queued, then flushes the writer.
///
/// @code
/// VKC_ASSIGN(std::unique_ptr<CaptureWorker> captures,
///            CaptureWorker::create(std::move(setup)));
/// // The live worker submits through its CaptureSink; the window:
/// VKC_TRY(captures->drop_last());  // the clicker's "back"
/// for (const CaptureEvent& event : captures->take_events()) show(event);
/// if (auto summary = captures->summary()) draw_session(*summary);
/// if (auto kept = captures->kept()) calibrate(*kept);  // on another thread
/// @endcode
class VC_APP_API CaptureWorker final : public CaptureSink {
 public:
  /// @brief Check @p setup and start the thread; nothing is written until the
  ///        first capture.
  /// @param setup  What to write. Refused unless the root is non-empty, there
  ///               is a camera, each with a colour model, the reference is
  ///               one, and the capacity and `max_kept` are at least 1.
  /// @return The worker; `InvalidArgument` naming what @p setup gets wrong;
  ///         `IoError` if the thread cannot start; or `OutOfMemory`.
  static core::Result<std::unique_ptr<CaptureWorker>> create(
      CaptureSetup setup);

  /// @brief Carry out everything queued, flush the writer, stop the thread.
  ~CaptureWorker() override;
  CaptureWorker(const CaptureWorker&) = delete;
  CaptureWorker& operator=(const CaptureWorker&) = delete;
  CaptureWorker(CaptureWorker&&) = delete;
  CaptureWorker& operator=(CaptureWorker&&) = delete;

  /// @return Whether @ref pending has reached the capacity.
  bool full() const override;

  /// @brief Queue a capture.
  /// @param request  The capture, one part per camera, each with its luma
  ///                 (`width * height` bytes) and the one depth unit.
  /// @return `true` once queued; `false`, with nothing queued, when
  ///         @ref full; `Unsupported`, with nothing queued, while the
  ///         captures kept and pending add up to `max_kept` (a drop makes
  ///         room); `InvalidArgument` for a request with a part per camera
  ///         missing or extra, a part's luma not its size, or a part without
  ///         a depth unit or in another unit than the first part; or
  ///         `OutOfMemory`.
  core::Result<bool> submit(CaptureRequest request) override;

  /// @brief Queue a drop of the latest capture requested before it that no
  ///        drop took (`SessionWriter::drop_last`): the capture stays on
  ///        disk, marked dropped. Its outcome is an event: `NotFound` before
  ///        any capture was written.
  /// @return OK once queued; or `OutOfMemory`.
  core::Status drop_last();

  /// @brief Take the outcome of every capture and drop since the last call.
  /// @return One event per capture or drop, in the order they were made.
  std::vector<CaptureEvent> take_events();

  /// @return What the captures kept add up to; empty before the first is
  ///         written.
  std::optional<quality::SessionSummary> summary() const;

  /// @brief The captures kept, for Calibrate.
  ///
  /// A capture joins once written (its `Saved` event) and leaves when
  /// dropped; one whose write failed never joins. Each change publishes a new
  /// snapshot, with @ref summary in one step, so once @ref flush returns the
  /// two count the same captures.
  /// @return The latest snapshot (@ref KeptCaptures); null before the first
  ///         capture is written.
  std::shared_ptr<const KeptCaptures> kept() const;

  /// @return The captures queued, being detected, or queued on the writer.
  std::size_t pending() const;
  /// @return The most captures it holds at once.
  std::size_t capacity() const noexcept { return capacity_; }

  /// @return The session's directory; empty before the first capture.
  std::string directory() const;

  /// @brief Take what went wrong beyond a capture's own outcome: a session
  ///        check that failed, and why the thread ended.
  /// @return The failures, oldest first.
  std::vector<core::Status> take_errors();

  /// @brief Wait until nothing is queued, being detected, or being written.
  void flush();

 private:
  struct State;
  CaptureWorker(std::unique_ptr<State> state, std::size_t capacity);
  void run() noexcept;

  std::unique_ptr<State> state_;
  std::size_t capacity_ = 0;
};

}  // namespace volumetric_kit::calib::app
