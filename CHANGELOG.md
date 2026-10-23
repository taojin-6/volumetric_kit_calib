# Changelog

All notable changes to `volumetric_kit_calib` are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed

- calib_rig says a camera's mosaic tile refusal once, until its reason
  changes (`app::refusals_to_say`), instead of at every set: on Linux, where
  every tile is refused, 100 lines a second pushed every other message out of
  the Messages window.
- `LiveWorker` documents what recon's Orbbec driver does: a camera that
  disconnects fails every poll, so the live view ends, saying why once. The
  "the sets go on without it" message, which that driver never let run, is
  gone.
- `CaptureWorker` and `CalibrateWorker` refuse new work with `IoError` once
  their thread has ended, and `CaptureWorker::flush` returns, rather than
  queueing work no thread carries out. `CalibrateWorker::request` after
  `stop` is now that `IoError`, no longer `false`.
- `LiveWorker` lets go of a snapshot the window never took after releasing
  its lock, so freeing that snapshot's frames no longer delays the window.
- calib_rig's Stages panel no longer flickers: its `tiles` row, recorded
  only as a live set arrives, came and went with the total as the window
  redrew between sets. `viz::hold_stages` holds the last recording's time on
  every frame the mosaic is shown, and none while it is hidden.
- `solve::view_pose` poses a board tilted exactly about its own x or y axis,
  which OpenCV 5.0.0's IPPE leaves with no pose on GCC's builds, in the
  board's frame and the quarter-turned one alike: it then solves the board
  turned an eighth turn.
- `RigMosaic::record` keeps a page's pictures and ages when gfx refuses a
  replacement in a different coding, including cameras with no new picture.
- Tile marks keep the last framebuffer column at fractional display scales
  through gfx's scissor-rounding repair.
- `draw_tile_marks` keeps the title's text and background inside all four
  tile edges when the image pans or zooms, including on high-DPI displays.
- `solve::bundle_adjust` counts a shared capture only when each camera keeps
  enough positive-weight corners to fix a pose, and returns `Numerical` if
  the intrinsics sensitivity diagnostics overflow.
- `check_initialise_options` takes its diagnostic prefix as a
  `std::string_view`, so constructing a default or literal prefix cannot
  allocate before the error guard.
- `CaptureRule::update` returns `OutOfMemory` if formatting a motion or spread
  message runs out of memory, preserving its armed state and previous hold.

### Added

- `solve::BundleAdjustOptions::uncertainty`: off, `bundle_adjust` skips the
  covariance and the intrinsics sensitivity; its guards still run.
- Calibrate's world at the origin tag (DECISIONS.md, 2026-10-19):
  `app::calibrate` locates the tag from the kept captures' tags after the
  metrics (`CalibrateStage::Locating`) and anchors the outputs last, so
  `CalibrateResult::color_to_world` and `boards_to_world` are in the tag's
  world, with `CalibrateResult::world` (`quality::ResultWorld`: the
  `WorldOrigin` and the boards on its floor); without a tag world the
  reference camera stays the origin, as before. `array_calibration.json`
  names a tag world as recon's `world.apriltag`; a reference world's file
  is unchanged. `report.json` gains `world`: the origin, the tag sought and
  its corners' refinement (`detect::kTagCornerRefinement`), why there is no
  tag world, the tag's fit, edges and corners, each camera's height, the
  boards on the floor and `uncertainty_frame`. In `solve`:
  `kMaxTagWorldRmsPx` (3 px), above which `origin_tag_frame` refuses a tag
  world as `TagMiss::PoorFit`, and `WorldOrigin::sought`; in `core`:
  `apriltag_family`; in `quality`: `floor_boards`, each solved board within
  5 degrees and 20 mm of the tag's floor, with its tilt and height. In
  `viz`: `display_rotation` and `SceneView::frame` take the world's up, a
  tag world's +z, so its floor lies level; `tag_lines` and
  `SceneOptions::tag_edge_m`, the tag's outline at the origin;
  `world_panel`, the Calibration window's world lines (the origin, the
  edge's warning, why there is no tag world, each camera's height and the
  floor boards); and `calibration_notices`, a result's notices, each amber
  world line among them. `RigOrigin::tag` frames the 3D view again when the
  world moves between the tag and a reference camera, and `world_up` gives
  its up.
- The depth checks at Calibrate (DECISIONS.md, 2026-10-16, "Depth checks:
  the host's geometry over the device's results", amended 2026-10-19):
  `app::CalibrateDepth`, a depth-only `GpuFramePrep` per camera and calib's
  `DepthChecks` on recon's device, built as calib_rig starts and owned by
  `CalibrateWorker` (T4), with which `app::calibrate` checks each kept
  capture's raw depth after the metrics, one capture at a time: each solved
  view's board plane, and every ordered pair's agreement, pooled and in each
  capture, at the solved poses; `CalibrateStage::Checking` counts the
  captures, and a cancel stops at the next. `CalibrateResult::depth` holds
  the summary; a failure of the checks is posted, and the results are
  written without them (`CalibrateResult::depth_failure`, `report.json`'s
  new `depth_unchecked`). In `quality`: `depth_views`, the views a solved
  rig's checks fit (`DepthView`); `CameraDepth::verdict`, from
  `camera_verdict` (fail if the scale or the plane fails, pass if both
  pass, else insufficient), written as `depth.cameras[id].verdict`. In
  `viz`: `depth_panel`, the Calibration window's depth lines: each camera's
  verdict with its ratio, offset and angle, the pairs that pass and fail,
  each pooled pair that fails with its medians, and each capture with a
  pair failing on its own.
- Calibrate in calib_rig (DECISIONS.md, 2026-10-18, "Calibrate (T4)"): C,
  F5 or a Calibrate button runs `CalibrateWorker` on the captures kept, with
  a progress line and Cancel (Escape) while it runs; each result fills the
  window with the 3D view, every camera's live points at the solved poses
  with its frusta and the solved boards, and V, B or `.` goes back to the
  camera images to capture more. `app::RigMode` (`app/rig_mode.hpp`) is the
  window's mode: `rig_action`, what each key does in the images and in the
  3D view; `calibrate_gate`, when Calibrate may run and why not, held back
  by `CaptureWorker::unresolved`, the captures and drops not yet resolved;
  `scene_placement`, `on_result` and `frame_on_result` (`RigOrigin`).
  `KeyRequest` gains `Calibrate`, `SwitchView` and `Cancel`.
  `app::placed_cameras` places the 3D view's cameras at a result's poses.
  In `viz`: `calibration_panel`, the Calibration window's lines (the RMS,
  the transfer, the leave-one-out moves, the principal-point verdict, each
  camera's centre, the results directory), and `calibrate_progress_line`.
- Calibrate's worker, `app::CalibrateWorker` (T4; DECISIONS.md, 2026-10-18):
  each request solves a snapshot of the kept captures from scratch with the
  thread-free `app::calibrate` (`initialise`, `bundle_adjust`,
  `calibration_metrics`, `format_results`, in the reference camera's gauge),
  then `app::write_results` writes `residuals.csv`, `report.json` and
  `array_calibration.json`, in that order, to the session's new
  `results/<RRR>_<YYYY-MM-DD_HHMMSS>/`, numbered once and stamped at the
  press; the result is published once the files are on disk. It reports its
  stage and leave-one-out progress, cancels between re-solves writing
  nothing, posts each refusal or failure, and stops on request.
- The origin tag (DECISIONS.md, 2026-10-18): `OriginTag`, the rig's
  `kOriginTag` (AprilTag 36h11 id 0, 0.15 m) and `check_origin_tag` in
  `core`; `detect::detect_tag`, which finds exactly one print of it in a
  `LumaView` with OpenCV's `ArucoDetector` and sub-pixel corners, as a
  `MarkerSearch`. `compute::opencv_dictionary`, each `MarkerDictionary` as
  OpenCV's, is public, and `LumaView` has a header of its own,
  `detect/luma.hpp`, which `charuco.hpp` includes.
- The world at the origin tag (`solve/world_tag.hpp`; DECISIONS.md,
  2026-10-18): `solve::origin_tag_frame` locates the tag from a solved rig,
  its poses fixed, from every camera's `TagView` of it (an N-view DLT, then
  least squares on `cv::LevMarq`, `bundle_adjust`'s outlier rule on whole
  views while it leaves `min_cameras` cameras, and a size-free square
  fitted in OpenCV's marker frame), as a `WorldOrigin`: a `TagWorld` (the
  frame, corners, edges against the print's, residuals per view, each
  camera's height) or a `TagMiss` with its reason (`TagFrameOptions`).
  `solve::anchor_rig` moves a solution's outputs, never the solution, into
  that world as an `AnchoredRig`. `compute::finite`, whether a transform's
  numbers are finite, which the solve and the checks share.
- `CaptureWorker::kept`, the captures kept for Calibrate (DECISIONS.md,
  2026-10-13): an immutable `KeptCaptures` snapshot of every capture written
  and not dropped, each a `KeptCapture` of every camera's full detection,
  numbered for the solve, and its raw depth, shared with the session writer;
  with the session's board, cameras, sensors, depth mode and solve
  thresholds. A capture that would make the captures kept and pending exceed
  `CaptureSetup::max_kept`, by default `bundle_adjust`'s 150, is refused.
- The origin tag at capture (DECISIONS.md, 2026-10-18): the capture worker
  searches each camera's full luma for `kOriginTag` beside the board, and
  keeps the corners in `KeptCapture::tags`; a camera with two prints of it
  keeps neither, and a failed search or two prints is posted, never failing
  the capture. `CaptureEvent::tag_cameras` counts the cameras that found it,
  which the capture panel's `last` line shows. The capture tier writes the
  corners as each camera's `origin_tag` in `captures/<NNN>.json` (`null` for
  none) and the tag searched for, `SessionInfo::origin_tag`, in
  `session.json`; both stay at version 1. The session panel's `origin tag`
  line (`viz::origin_tag_line`, from `app::tag_coverage`) counts the cameras
  and views of it in the captures kept, green at 2 cameras, amber at 1.
- calib_rig's live 3D view (DECISIONS.md, 2026-10-17): hidden with the
  camera images and the whole window at a Calibrate result, frozen or live,
  solid, tinted-luma or true colour, each camera shown or not in its tint,
  at first gfx's categorical colour of its index (`scene_camera`).
  `CloudWorker`, its T2 thread, takes each set `LiveWorker` reads back
  through its new `SetSink`, prepares every `CloudOptions::live_every`-th
  on recon's queue family and publishes a `CloudSnapshot`; `record_cloud`
  draws a snapshot's points at the rig's poses and holds it until its frame
  retires; `cloud_live` says when the worker prepares (not frozen, hidden or
  minimized); `rig_requirements` merges recon's, calib_depth's and gfx's
  needs with `largePoints`. In `viz`: `cloud_draw`, a prepared frame as gfx's
  `DepthCloudDraw` (`CloudStyle`), and `cloud_draws`, each `SceneCamera`'s;
  `frustum_lines`, `axes_lines`, `board_lines` and `scene_lines`;
  `display_rotation`, `window_split` (`ScenePlacement`) and `SceneView`.
- The `app` tier, `volumetric_kit::calib_app`, and `calib_rig`, live view and
  capture (DECISIONS.md, 2026-10-16), built with `viz` given `VC_BUILD_APPS`
  (top-level only by default), GLFW and recon's Orbbec driver, build-tree
  only. `calib_rig --config` loads a CalibRig scenario (`check_scenario`, its
  preset and sync file as a `RigScenario`; `run_calib_rig`): every camera of
  the rig's sync file opened at the scenario's modes (`rig_preset`,
  `depth_mode`, `orbbec_options`) and held to them (`check_preset`);
  `LiveWorker`, the one owner of the `SensorArray`, publishes a `LiveSnapshot`
  per set (luma read back, hybrid detection, view checks, the capture rule)
  and copies a taken capture out of its set as a `CaptureRequest`;
  `CaptureWorker` detects it at full resolution and writes the session
  (`session_camera`, `app_software`, the checkouts' commits read at every
  build); `record_snapshot`, `tile_overlays` and `camera_lives` draw a
  snapshot through the viz tier; `key_request` and `KeyQueue` map the keys and
  a clicker.
- `operator==` and `operator!=` for `Target`, every field compared.
- The `depth` tier, `volumetric_kit::calib_depth`, built with `VC_WITH_RIG`
  (DECISIONS.md, 2026-10-16, "Depth checks are calib's kernels"):
  `DepthChecks::fit_planes` fits each view's board plane on the GPU to its
  camera's depth, read in place (`DepthImage`, `PlaneRegion`), in two gated
  rounds in one submit, for any number of cameras;
  `DepthChecks::device_requirements` is the core's compute baseline.
  `DepthCheckOptions` gains the fit's `seed_mm`, `min_gate_mm`, `sigma_gate`
  and `min_inliers`, which `report.json` records; `check_depth_options` and
  `check_board_region` are the checks each depth step runs.
- The depth agreement on the GPU (DECISIONS.md, 2026-10-16, "Depth checks are
  calib's kernels"): `DepthChecks::histograms` makes `AgreementHistograms` of
  every ordered pair of any number of cameras; `accumulate` bins one
  capture's depth differences into them, edge-gated, skipping empty slots;
  `summarize` gives each pair's `PairAgreement`. `DepthCheckOptions` gains
  `window_mm`, `bin_mm`, `edge_ratio` and `agreement_stride`, which
  `check_depth_options` checks and `report.json` records.
- The depth checks' host geometry and verdicts (DECISIONS.md, 2026-10-16,
  "Depth checks"): `quality::board_region`, the depth pixels a view's plane
  fit takes, and `in_board_region`, its rule; `board_depth_check`, a view's
  corner rays met with its fitted `DepthPlane`, giving each adjacent pair's
  distance over the square, and the plane's offset and angle from the solved
  board; `summarize_depth_checks`, each camera's and the board's ratios
  pooled over the captures, every ordered pair's `PairAgreement`, pooled and
  per capture, and their `DepthVerdict`s. `format_results` writes the
  summary as `report.json`'s `depth`.
- `quality::calibration_metrics`, what a solved rig is worth (DECISIONS.md,
  2026-10-16): the reprojection error at the first rig and at the solution
  (overall, without oblique views, per camera, capture and view, the legacy
  RMSE and a histogram), the rejections by round and reason,
  `cross_camera_transfer` (the legacy `verify_rig.py`'s check),
  `leave_one_out` and `principal_point_ab` (warm re-solves), and the
  cameras' centres and baselines. `format_results` gives the results
  directory's files as text: recon's calibration file, `report.json` and
  `residuals.csv`.
- `solve::check_bundle_adjust_options`, `bundle_adjust`'s check of its
  options; `solve::adjusts`, its rule for the views it takes; and
  `to_string` of `solve::Dropped`, `solve::ViewFate` and `compute::ViewUse`,
  the names the report gives them.
- `compute::quantile`, numpy's linear percentile.
- The `viz` tier, `volumetric_kit::calib_viz`, built with `VC_WITH_RIG`,
  build-tree only (DECISIONS.md, 2026-10-16): `RigMosaic`
  copies each camera's decoded NV12 picture on the device into its tile of
  one mosaic, for any number of cameras of mixed sizes, paged over several
  textures when one cannot hold it (`mosaic_layout`), each page converted
  once per set, all shown through one `ImageView2D` (`tile_write` and
  `tile_conversion` map recon's `YuvImage` to gfx's write and conversion;
  `tile_region`, `tile_at`, `sets_since_picture`, `refused`, `discard`);
  `tile_marks` and `draw_tile_marks` mark the corners, markers, ids, verdict
  and coverage over a tile's region through gfx's `ImageOverlay`, the title
  as its caption; `camera_panel`, `set_panel`, `session_panel`,
  `capture_panel` and `stage_frame_metrics` build the live panels, drawn by
  `draw_panel_lines` and `draw_session_panel`, in `Tone`s that follow the
  quality tier's options;
  `BoardEditor` and `draw_board_editor` (its fields through `board_fields`
  and `apply_fields`) edit the board, checked as the detector's `Board`
  takes it, until the first capture locks it. `VC_WITH_RIG` fetches
  `volumetric_kit_gfx`, pinned at `76d0655` (main after #136, #137, with the
  overlay scissor repair, #138, `DepthCloudPipeline`, #141, a minimized
  window's framebuffer size of 0 x 0, and #142, `categorical_rgba8`).
- `marker_count`, the markers a board carries, in `calib/core/target.hpp`.
- `solve::bundle_adjust`, the rig's bundle adjustment on OpenCV's
  `cv::LevMarq` from `initialise`'s first rig: a Huber loss, outlier rounds
  and guards, and a `RigSolution` holding the poses, recon's
  `ArrayCalibration` of them, the boards, every corner's residual, what was
  left out, each camera's covariance and its sensitivity to the intrinsics
  (DECISIONS.md, 2026-10-14). `solve::view_residuals` gives one view's
  residuals and their derivatives.
- The `quality` tier, `volumetric_kit::calib_quality`: the live checks and the
  capture rule (DECISIONS.md, 2026-10-14). `check_view` fills a view's
  `ViewCheck` (verdict, distance, tilt, fit, 3 x 3 cell) from its own pose and
  says whether it is frontal; `corner_shift` measures how far corners moved
  between two sets of detections, and `live_vs_full_flagged` judges a
  capture's live corners by it; `CaptureRule` takes the first complete, still
  set after a press, or fails naming why; `SessionChecks` sums up the captures
  kept: views and 8 x 6 coverage per camera, the captures shared through views
  the solve uses, a graded tree preview, when Calibrate may run, and the
  duplicate, floor-only and mostly-frontal warnings.
- `compute::median`, `angle_deg`, `rotation_angle_deg`, `Board::centre`,
  `centre_in`, `face_normal` and `project`, which the solve and the checks
  share.
- `solve::check_rig`, `check_camera_ids`, `check_initialise_options`,
  `view_fate` and `reached_from`: `initialise`'s checks of its inputs, its
  rule for the views it uses, and the cameras the reference reaches, which
  the checks share.
- The `capture` tier, `volumetric_kit::calib_capture`, built with
  `VC_WITH_RIG` (OFF by default; fetches all of recon and needs the Vulkan
  SDK): `SessionWriter` writes a calibration session -- each camera's JPEG,
  raw 16-bit depth, the factory models in recon's calibration file, and each
  capture's detections and checks -- on a thread of its own behind a bounded
  queue, never overwriting a file (DECISIONS.md, 2026-10-13).
  `session_stamp` names a session's directory.
- `Detection` and `DetectedMarker`, a board's corners and markers in one
  image, in `calib/core/detection.hpp`; `to_string` for `TargetKind`,
  `MarkerDictionary` and `ViewVerdict`, the names calib's files write; and
  `is_usable(ViewVerdict)`, the verdicts that count as usable views.

### Changed

- build: **calib pins core 0.19.0** (ff44a98, core #49's merge), gfx at
  #152's merge (f147e00) and recon at #271's merge (9fd8790), and every
  build requires core 0.18.0 or newer (`VC_CORE_MIN_VERSION`, from 0.6.1),
  the pinned gfx's floor; `calib_app` requires its config tier at that
  floor. calib_rig now presents FIFO, paced to the display (gfx #150's
  default), where it presented MAILBOX, and its panels show ImGui's colours
  as authored on the `_SRGB` swapchain (gfx #144), where they drew washed
  out. It builds its overlay's config with `gfx::ui::overlay_config`. Tests:
  `core_requirement.*`.
- `quality::leave_one_out` and `principal_point_ab` re-solve only the
  corners the solution kept, in one solve each, with no outlier rounds or
  uncertainty step (DECISIONS.md, 2026-10-16, amended 2026-10-22): when the
  solve dropped outliers, Calibrate's metrics take about 2.5 times less CPU
  at 60 and 150 captures.
- `depth::DepthChecks::fit_planes` walks only the pixels round each region's
  projected rectangle (a pixel wider, clamped to the image), not the whole
  depth image, in both rounds; the counts are unchanged and the planes the
  same to float rounding.
- `depth::DepthChecks::accumulate` takes a capture's own histograms to count
  in the same pass (`alone`), emptying them first: Calibrate's depth checks
  run the agreement kernel once a capture rather than twice, and make no
  histograms per capture.
- calib_rig's live worker reads no luma back and searches no set for the
  board while the 3D view fills the window (`LiveWorker::set_detecting`),
  unless a capture is armed: each set's readback (8.3 MB a 4K camera) and
  detections are saved there.
- `calib_viz` and `calib_app` are static libraries with no export macros:
  `VC_VIZ_API`, `VC_APP_API` and their `export.hpp` headers are gone.
- A Debug build with `VC_SANITIZE` compiles at `-O1`, AddressSanitizer's
  recommended level, so a sanitized run is several times faster; assertions
  stay on.
- ctest runs each test binary as one test, named for its file
  (`detect_charuco`), so a suite's fixture is built once and OpenCV loads
  once; the out-of-memory binaries still run each case in its own process.
  `GTEST_FILTER` picks cases.
- The default tests draw a few seeds of each statistical claim; the full
  sweeps, DECISIONS.md's evidence, carry the ctest label `slow`.
- `app::CalibrateWorker::create` takes the depth checks before the results
  sink, `create(depth, sink)`, and `app::calibrate` takes them after the
  progress; `quality::format_results` takes why the depth was not checked
  after the depth checks, and the rig's world (`ResultWorld`) after that.
- `viz::SessionPanel::calibrate` is gone: it copied
  `quality::SessionSummary::ready`, one of the conditions `app::calibrate_gate`
  checks before Calibrate runs.
- `report.json` writes `solver.seconds` to the microsecond, with six
  decimals, so a report's length does not vary with its solve's time.
- `quality::SessionOptions::ready_from` defaults to
  `solve::BundleAdjustOptions{}.min_shared`, the adjustment's own guard (3,
  as before).
- `capture::CameraCapture::depth` is a
  `std::shared_ptr<const std::vector<std::uint16_t>>`, so a caller keeps the
  samples it hands the writer without a copy; `SessionWriter::submit` refuses
  a null one.
- The principal-point A/B verdict is the RMS alone, by a margin of
  min(2 % of the larger RMS, 0.05 px) (DECISIONS.md, 2026-10-16, amended
  2026-10-18): `MetricsOptions::principal_margin_fraction` and
  `principal_margin_px`, which `principal_point_verdict` and
  `principal_point_ab` take too and `report.json`'s `settings.metrics`
  records. The transfer is information. `PrincipalPointVerdict::Mixed` is
  now `Inconclusive`, named `inconclusive` in the report (still version 1).
  `principal_point_gap` gives the sides' gap relative to the larger RMS.
- OpenCV 5 is required, and `VC_WITH_OPENCV` is gone (DECISIONS.md,
  2026-10-17): `compute`, `detect`, `solve` and `quality` always build, the
  installed package always re-finds OpenCV, and `VC_WITH_RIG` alone builds
  `capture`, `depth` and `viz`. On Linux, `tools/install-opencv.sh` builds
  OpenCV 5 from its source; CI's Ubuntu legs and sanitizer job now build and
  test every OpenCV tier on it.
- `initialise` places the board of a capture whose views are all ambiguous,
  from those views, so the adjustment takes them.
- recon is pinned at `ca4b94c` (recon main, with its #228; its #245, whose
  exported `pinhole.glsl` the depth tier's kernels include; its #246, whose
  `GpuFramePrepConfig::depth_queue_families` shares a prepared frame's depth
  with the renderer's queue family; and its #254, whose
  `WorldFrame::apriltag` names a tag world in the calibration file), and
  CI's Orbbec SDK action with it. `cv_pose` now converts through recon's
  `rodrigues_from_matrix`, accurate where `cv::Rodrigues` returns zero (below
  about 1e-5 rad).
- calib requires `volumetric_kit_core` 0.6.1 or newer (was 0.1.0), with its
  vulkan tier under `VC_WITH_RIG`, and 0.11.0, with its config tier, for
  `calib_app`, and pins core main at 0.11.0. The installed package finds the
  core directly, at the version calib was built with, and refuses another
  core, from recon or found by the application first; an application may now
  find the core itself too (DECISIONS.md, 2026-10-13).
- calib's camera model is recon's (DECISIONS.md, 2026-10-12). Migrating:
  `PinholeBrownConrady`, `ImageSize` and `calib/core/camera_model.hpp` are
  removed; use `volumetric_kit::recon::camera::CameraModel` and
  `check_camera_model` from `volumetric_kit/recon/camera/camera_model.hpp`.
  `DetectedPoint::pixel` is recon's `camera::Vec2d` (`glm::dvec2`), not an
  Eigen vector. calib fetches recon's camera tier alone (`VR_CAMERA_ONLY`) and
  installs it beside itself, and its package finds recon.
  An application that uses recon beyond its camera tier makes recon available
  before calib. The build needs GLM instead of Eigen, and still no Vulkan SDK.
- `Target` names its markers: `dictionary` (`MarkerDictionary`, OpenCV's
  predefined ArUco and AprilTag dictionaries), `first_marker_id` and
  `legacy_pattern` (OpenCV's pre-4.6 ChArUco layout).
- The projection demo projects through recon's lens and no longer unprojects.
- The projection demo now reads a versioned JSON scenario via core_config,
  which owns parsing, defaults, and static validation. Run it with
  `--config config/project_point.json`; the library API is unchanged.

### Changed

- calib builds with C++ exceptions enabled, as recon and the core do: CI no
  longer builds it with `-fno-exceptions`, and a consumer builds it with
  exceptions on. No exception crosses its API; fallible calls still return
  `Status` or `Result`.
- calib writes volumetric_kit_core's names as the core does, and its
  re-exports of them are gone. Migrating: `vc::Status`, `vc::Result` and
  `vc::to_string` → `vkc::` (`volumetric_kit::core::`); include the core's
  `volumetric_kit/core/base/result.hpp` instead of
  `volumetric_kit/calib/core/result.hpp`, which is removed. Code inside
  `volumetric_kit::calib` writes `core::Status`.
- calib builds on `volumetric_kit_core`, fetched pinned by commit, and its
  installed package re-finds it. `Status` and `Result` are the core's types
  (using-declarations in `volumetric_kit::calib`), so they pass to `recon` and
  `gfx` unchanged. Migrating code on `main`: `VC_TRY` / `VC_ASSIGN` /
  `VC_CHECK` → `VKC_TRY` / `VKC_ASSIGN` / `VKC_CHECK`;
  `volumetric_kit/calib/core/check.hpp` is gone (include the core's
  `volumetric_kit/core/base/check.hpp`); a failed check now reports through the
  core's log sink, which prints to stderr by default; `std::move(r).value()`
  returns the value rather than a reference into `r`.
- `volumetric_kit_core` is pinned at 0.1.0 (main after its PR #21; was PR #13),
  and calib requires 0.1.0 or newer (`vkc_require_core`). calib's code is
  unchanged. What reaches its consumers:
  - An application that declares the core before calib pins 0.1.0 or newer;
    on an older one, calib's configure fails at `vkc_require_core`.
  - The installed package config makes the same check after it re-finds the
    core: beside an older core, `find_package(volumetric_kit_calib)` reports
    the package not found, naming the core, as a failed `find_dependency`
    does.
  - The installed package re-finds the core at 0.1, whose soname is `0.1`:
    rebuild against it.
  - The core supports only GCC and Clang -- it dropped Windows and MSVC -- and
    calib's public headers include its `result.hpp`, so calib does too.
  - Replacing the log handler from inside a handler call no longer deadlocks
    when another thread finishes a call on the old handler, and a handler that
    throws out of a failed `VKC_CHECK` no longer makes the thread's next failed
    check bypass it.
  - `Status::backend_error` takes the backend whose call failed; calib makes no
    such call.
  - The base tier adds `StageMetrics` and `StageScope`, the family's named
    host and device spans; calib does not report in them yet.
  - The core now has a vulkan tier, which calib's build leaves off
    (`VKC_WITH_VULKAN` defaults OFF in a subproject, and calib does not set
    it): calib still builds with no Vulkan SDK, and its installed core neither
    ships the tier's headers nor re-finds Vulkan. An application that also
    fetches recon or gfx declares the core first, with `VKC_WITH_VULKAN` ON,
    so the tier is built whichever library it adds first.
- The reserved `VC_WITH_VULKAN` / `VC_WITH_CUDA` options are removed: GPU work
  runs on the core's vulkan tier (DECISIONS.md).
- clang-format is pinned to 22.1.8, as in the sibling repositories.

### Added

- The `detect` tier, `volumetric_kit::calib_detect` (DECISIONS.md,
  2026-10-12): `detect_charuco` finds a ChArUco `Board` in an 8-bit luma image
  (`LumaView`, onto which recon's `PictureView` maps), at full resolution
  (`Mode::Full`) or by the live view's hybrid (`Mode::Hybrid`). The installed
  package re-finds OpenCV's imgproc too.
- The `compute` tier, `volumetric_kit::calib_compute` (needs an installed
  OpenCV 5, 5.0 or a later 5.x): OpenCV behind recon's and calib's types
  (DECISIONS.md, 2026-10-12). `cv_camera` gives a `CameraModel` as OpenCV's
  camera matrix and eight coefficients, and `undistort_points` its pixels'
  rays, undistorted to convergence; `cv_matrix`, `matrix_from_cv`, `cv_pose`
  and `matrix_from_cv_pose` convert rigid transforms; `Board` builds a ChArUco
  `Target` as OpenCV's board with its corners by id; `usable_for_pose` says
  whether a view of it fixes a pose (`ViewUse`, phrased by `describe`), and
  refuses one whose corners, or all but one, lie on a line; `guarded` and
  `current_exception_status` return what OpenCV throws as a `Status`. The
  installed package re-finds OpenCV.
- The `solve` tier, `volumetric_kit::calib_solve`: the first rig. `view_pose`
  gives a view's IPPE poses, the better first, with their RMS and
  `ambiguity()`, and `tilt_deg` the board's turn from the camera;
  `covisibility` and `spanning_tree` give the captures each pair of cameras
  shares and the tree of the links that share the most, its ties broken by
  camera id; `initialise` places every camera (`T_cw`, the reference's colour
  camera the world) and every capture's board (`T_wb`), recording each view's
  fate (`ViewRecord`) and each link's inliers and weakness (`PlacedLink`).
- Build-system foundation: top-level CMake (C++17), tiered component layout, the
  `volumetric_kit::calib_core` library target + `volumetric_kit::calib` umbrella
  alias, and first-class install/export so the library is consumable via both
  FetchContent and `find_package(volumetric_kit_calib)`.
- Dependency strategy: googletest vendored via pinned FetchContent for the test
  suite only.
- `core` tier:
  - Exception-free error handling from `volumetric_kit_core`'s base tier:
    `Status` (with a `Numerical` domain for solver failures) and `Result<T>`,
    both `[[nodiscard]]`, named in calib's namespace, plus the core's `VKC_TRY`
    / `VKC_ASSIGN` propagation macros and `VKC_CHECK` fail-fast contract check.
  - Physically-parameterized data model: `SensorProfile`, `LensProfile`,
    `Target`, `Observation` — the inputs the quality tier needs to be physical
    rather than heuristic. Profile fields are `std::optional`, so "unknown" is
    never confused with a real zero (e.g. a global shutter's line time).
  - `check_target`, which refuses a board a detector cannot use, naming the
    field, and `legacy_femto_board()`, the rig's 5 x 3 ChArUco board.
  - Plain records for the tiers to come: `ViewCheck` (one camera's view of the
    board: corners, markers, pose measures, `ViewVerdict`) and `CaptureMeta`
    (one capture's set-level facts).
- GoogleTest `tests/` (version smoke test, the board and its check, data
  model), configure tests of the core version calib requires
  (`tests/core_requirement/`) and of a brain with no Vulkan
  (`tests/no_vulkan/`, through an installed package), and an
  `examples/01_project_point` CLI.
- Lint/CI hygiene: `.clang-format`, `.cmake-format.yaml`, and a pre-commit
  config (clang-format, cmake-format, trailing-whitespace, end-of-file-fixer).

### Removed

- The install path (DECISIONS.md, 2026-10-22): calib is fetched by commit
  and installs nothing, so `VC_INSTALL`, its install rules and package config
  (`find_package(volumetric_kit_calib)`) are gone.
- Windows and MSVC support, which no calib target or CI leg used
  (DECISIONS.md, "Platforms"): calib builds with GCC or Clang for Linux,
  macOS, iOS and Android, as the core does. The MSVC warning flags
  (`/W4`, `/WX`) and the sanitizer option's MSVC refusal are gone.
