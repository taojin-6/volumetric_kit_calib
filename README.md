# volumetric_kit_calib

A calibration **companion** for the `volumetric_kit` capture rig — not another
solver, but the **intelligence layer around the solve**: live acquisition
guidance, physical diagnostics ("is this error good, given the sensor's noise
floor?"), and a requirements solver (spec → binding hardware constraint →
prescription). It covers single-camera intrinsics, stereo, multi-camera bundle
adjustment, fisheye, cross-camera color, and the optics that gate quality
(depth-of-field, diffraction, focus). The full architecture and domain model
are in `DESIGN.md`, a design doc kept local: it is gitignored, as in
`volumetric_kit_gfx`, so it is not in a clone.

> **Status:** Early scaffolding. The `core` tier — the data model (the board
> and its check, observations, detections, view and capture records, lens and
> sensor profiles) over recon's camera model and the core's exception-free
> `Status`/`Result` — builds and is tested (GoogleTest); so do the `compute`
> tier's OpenCV bridge, the `detect` tier's board and tag detection, the `solve`
> tier's rig solve and the `quality` tier's live checks and solved-rig
> metrics; with `VC_WITH_RIG`, the `capture` tier's session writer, the `viz`
> tier's live and 3D views and the `depth` tier's board-plane fit and
> pairwise depth agreement; and, with GLFW and recon's Orbbec driver too, the
> `app` tier's `calib_rig`, live view, 3D view, capture and Calibrate's worker.
> Calibrate in calib_rig's window and the color tier are not yet
> implemented; the legacy `camera_calibration` Python
> scripts (a separate checkout beside the `volumetric_kit/` folder) remain
> the reference oracle.

[AGENTS.md](AGENTS.md) is the shared working guide for Codex and Claude Code;
`CLAUDE.md` imports it. [DECISIONS.md](DECISIONS.md) records the locked
constraints and open GPU-backend choice without publishing the local design.

## Architecture (tiered)

`core` → `compute` → {`detect`, `solve`, `color`} → `quality` →
{`depth`, `viz`} → `app` (+ `capture` off `core`). A tier may depend only on
tiers to its left. The headless "brain"
(`core`+`compute`+`detect`+`solve`+`quality`) builds with **no**
`volumetric_kit_gfx` dependency; only the `viz`/`app` tiers pull in the
renderer.

- **Shared core:** every tier builds on
  [`volumetric_kit_core`](https://github.com/taojin-6/volumetric_kit_core), the
  family's shared foundation. Its base tier supplies calib's `Status`/`Result`;
  its vulkan tier, the rig tiers' GPU work, so the brain needs no Vulkan SDK.
- **Cameras and sensors:** cameras are `volumetric_kit_recon`'s
  `CameraModel`, from its camera tier, which calib fetches alone so the brain
  still needs no Vulkan SDK ([DECISIONS.md](DECISIONS.md#locked-decisions)).
- **Capture:** the `capture` tier writes a calibration session to disk on a
  thread of its own and never overwrites: each camera's JPEG as the camera
  coded it, raw 16-bit depth, the factory models in recon's calibration file,
  and the detections and checks
  ([DECISIONS.md](DECISIONS.md#2026-10-13--sessions-on-disk)). It takes
  recon's sensor and io tiers, so `VC_WITH_RIG` fetches all of recon.
- **Compute:** a CPU backend is the correctness oracle: OpenCV, behind
  recon's and calib's types. The `compute` tier holds the bridge -- the camera
  model and rigid transforms in OpenCV's types, the ChArUco board and whether a
  view of it fixes a pose, the measures the solve and the checks share, and
  `guarded()`, which returns what OpenCV throws as a `Status`. GPU work runs
  on the core's vulkan tier, on a `VkDevice` shared with `recon` and `gfx`, so
  frames pass between them without copies (MoltenVK on Apple; native Metal
  deferred).
- **Detection:** the `detect` tier finds the ChArUco board in an 8-bit luma
  image with OpenCV's detector, as corners by id and markers, at full
  resolution or by the live view's faster hybrid; and the origin tag on the
  floor, one AprilTag by id, its corners refined to sub-pixel.
- **Solve:** the rig's extrinsics from views of the board. `initialise` places
  every camera and board from the views alone -- each view's own pose (IPPE),
  the spanning tree of the camera links that share the most captures, each
  link's robust relative pose -- in the world of the reference camera's colour
  frame: the first rig. `bundle_adjust` refines it on OpenCV's `cv::LevMarq`,
  with a Huber loss and outlier rounds, and reports each camera's covariance
  and its sensitivity to the intrinsics, and the poses as recon's
  calibration.
- **Quality:** before any solve, what each view and set is worth and which
  set a capture takes. Each view's verdict, distance, tilt, fit and cell come
  from its own pose; a capture takes the first complete set whose board is
  still, warning of a wide frame spread or too few views; and the captures
  kept add up to views and image coverage per camera, the captures each pair
  of cameras shares through views the solve uses, the tree it would place
  them by, when Calibrate may run, and warnings of repeated, floor-only and
  mostly frontal views. All geometric
  ([DECISIONS.md](DECISIONS.md#2026-10-14--live-checks-and-the-capture-rule)).
  After the solve, what the rig is worth: the reprojection error before and
  after, what was left out, the cross-camera transfer (the legacy
  `verify_rig.py`'s check), each camera's move with one capture left out, a
  principal-point A/B test decided by the RMS with a margin, and the
  cameras' baselines; and the text of the results directory's files --
  recon's calibration file, `report.json` and `residuals.csv` -- which the
  application writes
  ([DECISIONS.md](DECISIONS.md#2026-10-16--the-solved-rigs-metrics-and-its-results-files)).
  Over the device's depth results, for any number of cameras: the board's
  scale and plane from each view's depth, each ordered pair's depth
  agreement, and their verdicts
  ([DECISIONS.md](DECISIONS.md#2026-10-16--depth-checks-the-hosts-geometry-over-the-devices-results)).
- **Depth:** the depth checks' device side, on the core's vulkan tier, over
  each camera's depth where recon's frame preparation left it, for any number
  of cameras: each view's board plane fitted on the GPU in two gated rounds in
  one submit, with 48 bytes per view read back; and each ordered pair's depth
  differences binned on the GPU over every capture, with 32 bytes per pair
  read back
  ([DECISIONS.md](DECISIONS.md#2026-10-16--depth-checks-are-calibs-kernels)).
- **Viz:** the live view, on `volumetric_kit_gfx`. Each camera's decoded
  picture is copied on the device into its tile of one mosaic, for any number
  of cameras, panned and zoomed as one view, with the corners, markers and
  coverage marked over each tile in the detector's pixels;
  panels show each camera's view, the latest set, the session and the
  capture; the board's editor locks at the first capture
  ([DECISIONS.md](DECISIONS.md#2026-10-16--the-live-view-on-gfx)). The 3D
  view draws each camera's prepared depth as points where recon left it, at
  the rig's poses, with the cameras' frusta, the world's axes and the solved
  boards, turned upright
  ([DECISIONS.md](DECISIONS.md#2026-10-17--the-live-3d-view-drawn-from-prepared-frames-in-place)).
- **App:** `calib_rig` opens every camera of the rig's sync file, at the
  scenario's modes, and shows them live with the board found in each; Space
  (or a clicker's "next") captures a still set, Backspace (or "back") drops
  the last. A worker thread owns the rig and finds the board in each set's
  luma, another prepares sets on the GPU for the 3D view (hidden until the
  rig is calibrated, then beside the mosaic or in the whole window), another
  detects each capture at full resolution, writes it and keeps it in memory
  for Calibrate, and the window draws and never waits
  ([DECISIONS.md](DECISIONS.md#2026-10-16--calib_rigs-live-view-and-capture)).
  Calibrate's worker solves and measures the kept captures and writes a
  results directory
  ([DECISIONS.md](DECISIONS.md#2026-10-18--calibrate-t4)).
- **Mobile (Android + iOS):** a first-class, fully on-device target — the
  board-constrained solve is small enough to run on an ARM CPU.

## Prerequisites

calib needs a C++17 compiler -- GCC or Clang; Windows and MSVC are not
supported ([DECISIONS.md](DECISIONS.md#platforms)) -- CMake ≥ 3.21, GLM, and
OpenCV 5 (5.0 or a later 5.x: its core, imgproc, objdetect and geometry
modules, and, for the tests, imgcodecs and calib). The first configure
fetches `volumetric_kit_core` and `volumetric_kit_recon` (pinned by commit)
and, for tests, googletest, so it needs network access. Ubuntu's packages are
OpenCV 4, so `tools/install-opencv.sh` builds OpenCV 5 from its source, the
newest 5.x release or a given tag, with only those modules, as CI does:

```bash
brew install cmake glm opencv                              # macOS
sudo apt-get install cmake build-essential git libglm-dev  # Ubuntu, then:
tools/install-opencv.sh latest /usr/local
```

For another prefix, pass `-DOpenCV_DIR=<prefix>/lib/cmake/opencv5` to calib's
configure. `VC_WITH_RIG` (OFF by default) builds `capture`, `depth` and
`viz`, over all of recon, which needs the Vulkan SDK and a GLSL compiler: the
core's vulkan tier
[prerequisites](https://github.com/taojin-6/volumetric_kit_core#prerequisites).
`viz` fetches `volumetric_kit_gfx` (pinned by commit). Given GLFW
(`brew install glfw`) and recon's Orbbec driver
(`-DVR_WITH_ORBBEC=ON -DVR_WITH_FFMPEG=ON`, with the
[Orbbec SDK](https://github.com/taojin-6/volumetric_kit_recon#optional-orbbec-sdk)
and [FFmpeg](https://github.com/taojin-6/volumetric_kit_recon#optional-ffmpeg)),
`VC_WITH_RIG` builds the `app` tier and `calib_rig` too. To build against a
local core, recon or gfx checkout instead of the pinned one, pass
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<path>`,
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=<path>` or
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_GFX=<path>`.

## Build and test

```bash
calib_root="$(git rev-parse --show-toplevel)"
cmake -S "$calib_root" -B "$calib_root/build" -DCMAKE_BUILD_TYPE=Release \
  -DVC_WITH_RIG=ON  # leave out without the Vulkan SDK
cmake --build "$calib_root/build" --parallel
ctest --test-dir "$calib_root/build" --output-on-failure
```

Run the demo:

```bash
./build/examples/example_01_project_point --config config/project_point.json
```

The demo reads the family's versioned JSON scenario format through
`volumetric_kit::core_config`. Core owns defaults and static validation, including
unknown fields and application compatibility. Edit `config/project_point.json`
to change the camera and point. The demo accepts only `--config PATH` and `--help`;
the calibration library itself keeps its core_base/recon_camera dependency.

Run calib_rig on a rig (the session root must exist):

```bash
./build/apps/calib_rig --config my_rig.json
```

```json
{"version": 1, "operation": "calibrate",
 "source": {"kind": "live", "sync_config": "femto_mega_sync.json"},
 "output": {"path": "sessions"}}
```

The core's
[CONFIG.md](https://github.com/taojin-6/volumetric_kit_core/blob/main/CONFIG.md)
has every field, the modes and the window among them.

## Use it in your project

```cmake
include(FetchContent)
FetchContent_Declare(
  volumetric_kit_calib
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_calib.git
  GIT_TAG <tag-or-sha>)
FetchContent_MakeAvailable(volumetric_kit_calib)

target_link_libraries(your_app PRIVATE volumetric_kit::calib_core)
```

`VC_BUILD_TESTS`, `VC_BUILD_EXAMPLES`, `VC_BUILD_APPS`, `VC_INSTALL`, and
`VC_WARNINGS_AS_ERRORS` default ON only when `volumetric_kit_calib` is the
top-level project, so a FetchContent build of it never fails on a warning your
compiler adds. `volumetric_kit::calib_compute`, `volumetric_kit::calib_detect`,
`volumetric_kit::calib_solve` and `volumetric_kit::calib_quality` are always
built. Set `VC_WITH_RIG` ON before calib is made available to link
`volumetric_kit::calib_capture`, `volumetric_kit::calib_depth`,
`volumetric_kit::calib_viz` and, with `VC_BUILD_APPS`, GLFW and recon's Orbbec
driver, `volumetric_kit::calib_app`; `calib_viz` and `calib_app` are
build-tree only: an installed calib lacks them, as an installed gfx lacks its
ui tier.
`VC_SANITIZE` is a semicolon list, empty (off) by default —
e.g. `-DVC_SANITIZE="address;undefined"`. An installed calib carries the core
and recon (its camera tier, or all of it with `VC_WITH_RIG`) beside it, and
`find_package(volumetric_kit_calib)` finds all three, and the installed
OpenCV. An application may find the core itself too, before calib or after
it; if it finds another core first, calib is not found, and the message names
both.

calib needs `volumetric_kit_core` 0.6.1 or newer, with its vulkan tier for
`VC_WITH_RIG`, and 0.11.0, with its config tier, for `calib_app`. An
application that declares the core before calib, to share it with `recon` or
`gfx`, pins at least that: on an older core, calib's configure fails at
`vkc_require_core`, and an installed calib is not found. Without
`VC_WITH_RIG`, calib makes recon available camera-only, so an application that
uses recon beyond its camera tier makes recon available, with its options,
before calib; with it, an application that makes recon available first makes
all of it.

## Development

```bash
pre-commit install   # clang-format + cmake-format + hygiene hooks
```

## License

Released under the [MIT License](LICENSE). Each source file carries an
`SPDX-License-Identifier: MIT` header.
