// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file capture_worker.hpp
/// @brief calib_rig's capture worker (its T3 thread): each capture the live
///        view takes is detected again at full resolution, checked and handed
///        to the session writer, off the window's thread.
///
/// The live view searches each picture's half; the solver takes corners found
/// in the full picture, so each capture's luma is searched again here and
/// its views checked, the two detections compared
/// (`quality::live_vs_full_flagged`), the origin tag searched for in the same
/// luma (`detect::detect_tag`, `kOriginTag`), and the capture queued on the
/// capture tier's `SessionWriter`, which writes it on a thread of its own. The
/// session's directory is created at the first capture, under the scenario's
/// `output.path`, named for its time (`capture::session_stamp`), with `_2`,
/// `_3`, ... after it when a session of that second is there. Captures, drops
/// and restarts (the next session, @ref CaptureWorker::restart) are carried
/// out in the order they come, and each capture's and drop's outcome is
/// reported (@ref take_events), with the session's checks once it is written
/// (@ref summary) and the captures kept for Calibrate (@ref kept).
///
/// A scan session (`capture::SessionMode::Scan`) searches for no board and
/// checks nothing: each camera's origin tag is searched for in its luma, and
/// its V210 picture (@ref CameraFrameCopy::colour) is converted to 8-bit
/// R'G'B' (`sensor::v210_to_rgb8`) and encoded as a PNG
/// (`sensor::io::encode_png_rgb8`), every camera's search and still at once,
/// on this thread's OpenCV loop; a camera with a JPEG keeps it, and one with
/// neither its luma. The writer then writes the files.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/calib/core/capture_meta.hpp"
#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/quality/lens_readiness.hpp"
#include "volumetric_kit/calib/quality/live_checks.hpp"
#include "volumetric_kit/calib/quality/pair_row_check.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"

namespace volumetric_kit::calib::app {

/// @brief What calib_rig records a session was captured with: calib's,
///        the core's, sensor's and gfx's commits as this build fetched them
///        (empty where a checkout is not a git one), and OpenCV's and the
///        Orbbec SDK's versions.
/// @return The versions.
///
/// @code
/// CaptureSetup setup;
/// setup.software = app_software();
/// @endcode
capture::Software app_software();

/// @brief What the capture worker writes, and how it judges each capture.
///
/// @code
/// CaptureSetup setup;
/// setup.root = config.output.path;
/// setup.software = app_software();
/// setup.driver = opened.driver;
/// setup.sync_path = opened.sync_path;
/// setup.reference = opened.reference;
/// setup.depth_mode = opened.depth_mode;
/// setup.cameras = opened.cameras;  // the opener's, in the source's order
/// setup.lenses = opened.lenses;
/// @endcode
struct CaptureSetup {
  /// The existing directory each session's own directory
  /// (`YYYY-MM-DD_HHMMSS`, `capture::session_stamp`, with `_2`, `_3`, ...
  /// after it when a session of that second is there) is created in.
  std::string root;
  capture::Software software;  ///< What the session was captured with.
  /// What the session calibrates: each camera's lens, or the rig's poses;
  /// or nothing, a scan's stills.
  capture::SessionMode mode = capture::SessionMode::Extrinsics;
  /// The scenario's calibration file (`source.calibration`), read; null for
  /// none. A lens run's file carries its other cameras' lenses
  /// (`quality::format_lens_results`), and its lenses are fits' starts.
  std::shared_ptr<const sensor::camera::ArrayCalibration> calibration;
  /// The stereo pairs whose rectified rows a pose session's Calibrate checks
  /// (`RigScenario::pairs`); empty for none.
  std::vector<core::config::StereoPairConfig> pairs;
  /// How their rows are checked (`RigScenario::pair_rows`).
  quality::PairRowCheckOptions pair_rows;
  std::string driver;  ///< What opened the cameras (`"orbbec"`).
  /// The rig's sync configuration file; empty for a rig without one.
  std::string sync_path;
  std::size_t reference = 0;  ///< The reference camera, the rig's primary.
  /// The depth mode of the cameras with depth: its name and range; empty
  /// when none has depth. Its unit is the first capture's frames'
  /// (`CameraFrameCopy::metres_per_unit`), and a later capture in another
  /// unit fails.
  std::optional<capture::DepthMode> depth_mode;
  /// How the cameras code their colour (`"mjpeg"`); empty when unknown.
  std::string color_codec;
  /// The cameras, in rig order, as every capture gives them.
  std::vector<capture::SessionCamera> cameras;
  /// Each camera's lens, in rig order, at its pictures' size: what its full
  /// view is checked through, and the session's solve takes.
  std::vector<sensor::camera::CameraModel> lenses;
  /// Each full view's check (`view`), and the session's thresholds.
  quality::SessionOptions session;
  quality::LiveVsFullOptions live_vs_full;  ///< When a capture is flagged.
  /// Captures held at once: queued here, being detected, or queued on the
  /// writer; at least 1.
  std::size_t capacity = 4;
  /// Captures kept and pending at once, at most: what one Calibrate adjusts,
  /// which also bounds the raw depth held (@ref KeptCapture::depth), 2 bytes
  /// per depth pixel per camera and capture (0.74 MB at nfov_unbinned,
  /// 2.1 MB at wfov_unbinned); at least 1. A scan session, which keeps no
  /// depth and is never calibrated, has no bound.
  // TODO: more than 150 captures, should a session ever need them.
  std::size_t max_kept = solve::BundleAdjustOptions{}.max_captures;
};

/// @brief A capture kept for Calibrate: each camera's full detection, raw
///        depth and origin tag, as the session wrote them.
///
/// @code
/// for (std::size_t c = 0; c < capture.views.size(); ++c) {
///   const Observation& view = capture.views[c].observation;
///   if (!view.empty()) observations.push_back(view);  // the solve's input
///   if (capture.depth[c]) check_depth(*capture.depth[c]);
///   if (capture.tags[c]) tag_views.push_back(*capture.tags[c]);
/// }
/// @endcode
struct KeptCapture {
  int index = -1;  ///< As the session numbers it (`captures/<NNN>.json`).
  /// Each camera's board found at full resolution, in rig order: the
  /// detection its file holds, with `observation.camera_id` the camera's id
  /// and `observation.frame_id` the capture's index. A camera that found no
  /// board, or whose detection failed, has no corners, as has every camera
  /// in a scan session, which searches for none.
  std::vector<Detection> views;
  /// Each camera's raw depth, in rig order: the very samples the session
  /// wrote as its PNG, shared with the writer, never copied; null for a
  /// camera without depth, and in a scan session, which keeps none.
  std::vector<std::shared_ptr<const std::vector<std::uint16_t>>> depth;
  /// Each camera's origin tag (`kOriginTag`), in rig order, in its colour
  /// image's pixels: the corners its file holds; empty where the camera
  /// found none, more than one print of it, or its search failed.
  std::vector<std::optional<DetectedMarker>> tags;
  /// Whether each camera's view, in rig order, is frontal through its lens
  /// (`quality::CheckedView::frontal`).
  std::vector<bool> frontal;
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
  /// What the session calibrates (`CaptureSetup::mode`).
  capture::SessionMode mode = capture::SessionMode::Extrinsics;
  /// The scenario's calibration file (`CaptureSetup::calibration`); null for
  /// none.
  std::shared_ptr<const sensor::camera::ArrayCalibration> calibration;
  /// The stereo pairs whose rows Calibrate checks (`CaptureSetup::pairs`).
  std::vector<core::config::StereoPairConfig> pairs;
  /// How their rows are checked (`CaptureSetup::pair_rows`).
  quality::PairRowCheckOptions pair_rows;
  Target board;  ///< The session's board, locked at its first capture.
  /// The rig's cameras, in rig order, as the session's checks take them:
  /// each one's id and lens (`CaptureSetup::lenses`), and how a calibration
  /// file marks the lens (`lens_source`).
  std::vector<solve::RigCamera> cameras;
  /// Each camera's record, in rig order (`CaptureSetup::cameras`): its
  /// depth model and `depth_to_color` among them.
  std::vector<capture::SessionCamera> sensors;
  std::size_t reference = 0;  ///< The reference camera, by index.
  /// The depth mode of the cameras with depth, in the session's depth unit;
  /// as default-made when none has depth.
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
  /// Saved: its cameras that found the origin tag (@ref KeptCapture::tags).
  int tag_cameras = 0;
  /// Saved: each camera's picture file and its size, in rig order
  /// (`capture::WriteResult::pictures`).
  std::vector<capture::PictureFile> pictures;
};

/// @brief The capture worker: a thread that detects each capture at full
///        resolution and queues it on the session writer.
///
/// Every member may be called from any thread. Destroying the worker first
/// carries out every capture and drop queued, then flushes the writer. A
/// failure that ends the thread is posted (@ref take_errors); from then on
/// new work is refused, and what was queued is never carried out.
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
class CaptureWorker final : public CaptureSink {
 public:
  /// @brief Check @p setup and start the thread; nothing is written until the
  ///        first capture.
  /// @param setup  What to write. Refused unless the root is non-empty, there
  ///               is a camera, each with a lens at its size, the reference
  ///               is one, and the capacity and `max_kept` are at least 1.
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
  ///                 (`width * height` bytes); a camera with depth's part
  ///                 with its depth in the one depth unit, another's with
  ///                 none. A part without a JPEG saves its luma, or in a
  ///                 scan session its colour, when it has a copy of it.
  /// @return `true` once queued; `false`, with nothing queued, when
  ///         @ref full; `Unsupported`, with nothing queued, while the
  ///         captures kept and pending add up to `max_kept` (a drop makes
  ///         room), but in a scan session; `InvalidArgument` for a request
  ///         with a part per camera missing or extra, a part's luma not its
  ///         size, a colour copy that is not V210 at its size or whose words
  ///         do not hold its rows (@ref ColourCopy::words), a camera
  ///         with depth's part without a depth unit or in another unit than
  ///         the first such part, or depth from a camera without it;
  ///         `IoError` once the thread has ended; or `OutOfMemory`.
  core::Result<bool> submit(CaptureRequest request) override;

  /// @brief Queue a drop of the latest capture requested before it that no
  ///        drop took (`SessionWriter::drop_last`): the capture stays on
  ///        disk, marked dropped. Its outcome is an event: `NotFound` before
  ///        any capture was written.
  /// @return OK once queued; `IoError` once the thread has ended; or
  ///         `OutOfMemory`.
  core::Status drop_last();

  /// @brief Close this session and start the next with @p setup, the cameras
  ///        as they are: carried out in turn, after every capture and drop
  ///        queued before it, which go to this session.
  ///
  /// The writer is flushed and closed, its captures staying on disk; the
  /// summary, the lens readiness and the captures kept go empty; and the next
  /// capture opens a new session directory under @p setup's root, in its
  /// mode, with its lenses. Before the first capture there is nothing to
  /// close.
  /// @param setup  The next session's, as @ref create checks it, with the
  ///               same cameras (ids, sizes, depth) and capacity.
  /// @return OK once queued; `InvalidArgument` naming what @p setup gets
  ///         wrong (@ref accepts); `IoError` once the thread has ended; or
  ///         `OutOfMemory`.
  core::Status restart(CaptureSetup setup);

  /// @brief Whether @ref restart would take @p setup, checked as it checks
  ///        it, so a caller can check before it changes anything else.
  /// @param setup  The next session's.
  /// @return OK; `InvalidArgument` naming what @p setup gets wrong; or
  ///         `OutOfMemory`.
  core::Status accepts(const CaptureSetup& setup) const;

  /// @brief Take the outcome of every capture and drop since the last call.
  /// @return One event per capture or drop, in the order they were made.
  std::vector<CaptureEvent> take_events();

  /// @return What the captures kept add up to; empty before the first is
  ///         written.
  std::optional<quality::SessionSummary> summary() const;

  /// @return In an intrinsic session, how far along each camera's views are
  ///         toward its lens fit (`lens_states`), in rig order, published
  ///         with @ref summary; empty before the first capture is written
  ///         and in an extrinsic session.
  std::vector<quality::LensReadiness> lenses() const;

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
  /// @brief The captures, drops and restarts not yet resolved: @ref pending,
  ///        each drop queued until its event, and each @ref restart until it
  ///        is carried out.
  ///
  /// Read before @ref kept: once it is 0, a snapshot read after reflects
  /// every capture and drop made before.
  /// @return Their number.
  std::size_t unresolved() const;
  /// @return The most captures it holds at once.
  std::size_t capacity() const noexcept { return capacity_; }

  /// @return The session's directory; empty before the first capture.
  std::string directory() const;

  /// @brief Take what went wrong beyond a capture's own outcome: a camera's
  ///        full detection, check or origin-tag search that failed, or its
  ///        image holding more than one print of the origin tag
  ///        (`InvalidArgument`, none of them kept), none of which fails the
  ///        capture; a session check that failed; and why the thread ended.
  ///        (A scan's still that cannot be made fails its capture.)
  /// @return The failures, oldest first.
  std::vector<core::Status> take_errors();

  /// @brief Wait until nothing is queued, being detected, or being written,
  ///        and the worker holds no snapshot but its latest (@ref kept); or
  ///        until the thread has ended.
  void flush();

 private:
  struct State;
  CaptureWorker(std::unique_ptr<State> state, std::size_t capacity);
  void run() noexcept;

  std::unique_ptr<State> state_;
  std::size_t capacity_ = 0;
};

}  // namespace volumetric_kit::calib::app
