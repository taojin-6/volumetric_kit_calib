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
> sensor profiles) over sensor's camera model and the core's exception-free
> `Status`/`Result` — builds and is tested (GoogleTest); so do the `compute`
> tier's OpenCV bridge, the `detect` tier's board and tag detection, the `solve`
> tier's rig and lens solves and the `quality` tier's live checks, solved-rig
> metrics, lens gates and pair row check; with `VC_WITH_RIG`, the `source`
> tier's grey pictures from any sensor source, the `capture` tier's session
> writer, the `viz` tier's live and 3D views and the `depth` tier's
> board-plane fit and pairwise depth agreement; and, with GLFW and a sensor
> driver too, Orbbec's or DeckLink's, the `app` tier's `calib_rig`, live view,
> 3D view, capture and Calibrate, of each camera's lens or the rig's poses.
> The color tier is not yet implemented; the legacy `camera_calibration` Python
> scripts (a separate checkout beside the `volumetric_kit/` folder) remain
> the reference oracle.

[AGENTS.md](AGENTS.md) is the shared working guide for Codex and Claude Code;
`CLAUDE.md` imports it. [DECISIONS.md](DECISIONS.md) records the locked
constraints and open GPU-backend choice without publishing the local design.

## Architecture (tiered)

`core` → `compute` → {`detect`, `solve`, `color`} → `quality` →
{`depth`, `viz`} → `app` (+ `capture` off `core`, `source` off `detect`). A
tier may depend only on tiers to its left. The headless "brain"
(`core`+`compute`+`detect`+`solve`+`quality`) builds with **no**
`volumetric_kit_gfx` dependency; only the `viz`/`app` tiers pull in the
renderer.

- **Shared core:** every tier builds on
  [`volumetric_kit_core`](https://github.com/taojin-6/volumetric_kit_core), the
  family's shared foundation. Its base tier supplies calib's `Status`/`Result`;
  its vulkan tier, the rig tiers' GPU work, so the brain needs no Vulkan SDK.
- **Cameras and sensors:** cameras are
  [`volumetric_kit_sensor`](https://github.com/taojin-6/volumetric_kit_sensor)'s
  `CameraModel`, from its camera tier, which calib fetches alone so the brain
  still needs no Vulkan SDK ([DECISIONS.md](DECISIONS.md#locked-decisions)).
- **Source:** the `source` tier reads every camera as one source of synced
  8-bit grey pictures with camera ids, whatever drives it: a hardware-synced
  array (the Femto rig) or one stereo sensor (a DeckLink pair). Each set's
  luma and its half are made on the GPU, from any layout a driver hands out,
  and read back in one submit; the set itself stays on the device
  ([DECISIONS.md](DECISIONS.md#2026-11-02--every-camera-through-one-source)).
  It is public, but outside the stable API.
- **Capture:** the `capture` tier writes a calibration session to disk on a
  thread of its own and never overwrites: each camera's JPEG as the camera
  coded it, or its luma as a PGM, raw 16-bit depth from cameras with depth,
  the factory models in sensor's calibration file, and the detections of the
  board and the origin tag, and the checks
  ([DECISIONS.md](DECISIONS.md#2026-10-13--sessions-on-disk)). It takes
  sensor's image I/O tier, so `VC_WITH_RIG` builds sensor beyond its camera
  tier.
- **Compute:** a CPU backend is the correctness oracle: OpenCV, behind
  sensor's and calib's types. The `compute` tier holds the bridge -- the camera
  model and rigid transforms in OpenCV's types, the ChArUco board and whether a
  view of it fixes a pose, the measures the solve and the checks share, and
  `guarded()`, which returns what OpenCV throws as a `Status`. GPU work runs
  on the core's vulkan tier, on a `VkDevice` shared with `sensor` and `gfx`, so
  frames pass between them without copies (MoltenVK on Apple; native Metal
  deferred).
- **Detection:** the `detect` tier finds the board in an 8-bit luma image,
  through one detector for both kinds: a ChArUco board with OpenCV's
  detector, as corners by id and markers; a plain checkerboard with
  `findChessboardCornersSB`, cut by the image's edge too, its ids holding in
  one view only. It searches the image, or the image halved for the live
  view, refining each corner in the image. It also finds the origin tag on
  the floor, one AprilTag by id, its corners refined to sub-pixel.
- **Solve:** the rig's extrinsics from views of the board. `initialise` places
  every camera and board from the views alone -- each view's own pose (IPPE),
  the spanning tree of the camera links that share the most captures, each
  link's robust relative pose -- in the world of the reference camera's colour
  frame: the first rig. `bundle_adjust` refines it on OpenCV's `cv::LevMarq`,
  with a Huber loss and outlier rounds, and reports each camera's covariance
  and its sensitivity to the intrinsics, and the poses as sensor's
  calibration. `origin_tag_frame` then triangulates the origin tag on the
  floor from the solved rig, and `anchor_rig` moves the outputs into its
  world, z up from the floor
  ([DECISIONS.md](DECISIONS.md#2026-10-19--the-world-at-the-origin-tag)).
  `solve_lens` fits one camera's lens from its own views of a board, either
  kind: sensor's rational lens by `cv::calibrateCamera`, from two starts, with
  views held out in every region of the image, and the adjustment's outlier
  bounds
  ([DECISIONS.md](DECISIONS.md#2026-11-01--a-cameras-lens-from-its-views)).
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
  principal-point A/B test decided by the RMS with a margin, the cameras'
  baselines and the boards on the origin tag's floor; and the text of the
  results directory's files -- sensor's calibration file in the rig's world,
  `report.json` and `residuals.csv` -- which the application writes
  ([DECISIONS.md](DECISIONS.md#2026-10-16--the-solved-rigs-metrics-and-its-results-files)).
  For a lens: whether a camera's views are enough yet, its gates (the
  held-out RMS and its ratio to the fit's, few held-out corners past the fit's
  bound, no pattern left in the residuals over the image, no fold inside the
  frame, corners reaching the image's corners and sides), and a lens-only
  results directory
  ([DECISIONS.md](DECISIONS.md#2026-11-01--a-cameras-lens-from-its-views)).
  For a stereo pair: how far apart each board corner's rows land in its two
  rectified views, through sensor's own rectification at the runtime's size,
  judged cell by cell over the view, and whether the pair is named the right
  way round
  ([DECISIONS.md](DECISIONS.md#2026-11-02--a-stereo-pairs-rectified-rows)).
  Over the device's depth results, for any number of cameras: the board's
  scale and plane from each view's depth, each ordered pair's depth
  agreement, and their verdicts
  ([DECISIONS.md](DECISIONS.md#2026-10-16--depth-checks-the-hosts-geometry-over-the-devices-results)).
- **Depth:** the depth checks' device side, on the core's vulkan tier, over
  each camera's depth where sensor's frame preparation left it, for any number
  of cameras: each view's board plane fitted on the GPU in two gated rounds in
  one submit, with 48 bytes per view read back; and each ordered pair's depth
  differences binned on the GPU over every capture, with 32 bytes per pair
  read back
  ([DECISIONS.md](DECISIONS.md#2026-10-16--depth-checks-are-calibs-kernels)).
- **Viz:** the live view, on `volumetric_kit_gfx`. Each camera's picture --
  a decoder's NV12 images in colour, any other camera's luma in grey -- is
  copied on the device into its tile of one mosaic, for any number
  of cameras, panned and zoomed as one view, with the corners, markers and
  coverage marked over each tile in the detector's pixels;
  panels show each camera's view, the latest set, the session and the
  capture; the board's editor locks at the first capture
  ([DECISIONS.md](DECISIONS.md#2026-10-16--the-live-view-on-gfx)). The 3D
  view draws each camera's prepared depth as points where sensor left it, at
  the rig's poses, with the cameras' frusta, the world's axes, the solved
  boards and the origin tag, turned upright, the tag's floor level
  ([DECISIONS.md](DECISIONS.md#2026-10-17--the-live-3d-view-drawn-from-prepared-frames-in-place)).
- **App:** `calib_rig` opens the scenario's cameras -- every camera of a
  Femto rig's sync file at its modes, or one DeckLink pair with no
  calibration -- and shows them live with the board found in each; Space
  (or a clicker's "next") captures a still set, Backspace (or "back") drops
  the last. A worker thread owns the rig's source and finds the board in
  each set's half, another prepares sets on the GPU for the 3D view (hidden
  until the rig is calibrated), another finds the board and the origin tag
  in each capture at full resolution, writes it and keeps it in memory for
  Calibrate, and the window draws and never waits
  ([DECISIONS.md](DECISIONS.md#2026-10-16--calib_rigs-live-view-and-capture)).
  Calibrate's worker solves and measures the kept captures, locates the
  origin tag, checks their raw depth against the solved rig on the GPU,
  pass or fail per camera and per pair, and writes a results directory in
  the tag's world, or the reference camera's when it cannot be located;
  each result fills the window with the live points at the solved poses,
  and a key goes back to the images
  ([DECISIONS.md](DECISIONS.md#2026-10-18--calibrate-t4)). In intrinsic
  mode it instead fits each ready camera's lens and writes a lens-only
  results directory ([its modes](#intrinsic-and-extrinsic-mode);
  [DECISIONS.md](DECISIONS.md#2026-11-02--intrinsic-mode)).
- **Mobile (Android + iOS):** a first-class, fully on-device target — the
  board-constrained solve is small enough to run on an ARM CPU.

## Stable API

A program built on calib uses the brain through these headers, under
`include/volumetric_kit/calib/`; each builds without Vulkan or gfx. The API is
these headers and calib's types they use, wherever declared; what stays
stable, and how a break is flagged, is
[DECISIONS.md](DECISIONS.md#2026-11-01--the-stable-api)'s rule. Every other
header is internal, but for `source/rig_source.hpp`: public, for `calib_rig`
and the tests, and not promised until a second program reads a rig through
it.

| Layer | Headers |
| --- | --- |
| Board | `core/target.hpp`, `compute/board.hpp` |
| Detect | `detect/luma.hpp`, `detect/board.hpp`, `detect/tag.hpp` |
| Views | `core/observation.hpp`, `core/detection.hpp`: the views are a `std::vector<Observation>`, by camera id and capture index |
| Solve | `solve/initialise.hpp`, `solve/bundle_adjust.hpp`, `solve/world_tag.hpp`, `solve/lens.hpp` |
| Quality | `quality/calibration_metrics.hpp`, `quality/results.hpp`, `quality/lens_gate.hpp`, `quality/pair_row_check.hpp` |
| File | sensor's `camera/array_calibration.hpp`: calib formats it, the caller writes it |

## Prerequisites

calib needs a C++17 compiler -- GCC or Clang; Windows and MSVC are not
supported ([DECISIONS.md](DECISIONS.md#platforms)) -- CMake ≥ 3.21, GLM, and
OpenCV 5 (5.0 or a later 5.x: its core, imgproc, objdetect, geometry and calib
modules, and, for the tests and examples, imgcodecs). The first configure
fetches `volumetric_kit_core` and `volumetric_kit_sensor` (pinned by commit)
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
`viz`, over sensor beyond its camera tier, which needs the Vulkan SDK and a
GLSL compiler: the core's vulkan tier
[prerequisites](https://github.com/taojin-6/volumetric_kit_core#prerequisites).
`viz` fetches `volumetric_kit_gfx` (pinned by commit). Given GLFW
(`brew install glfw`) and a sensor driver -- Orbbec's (`-DVS_WITH_ORBBEC=ON`,
with the
[Orbbec SDK](https://github.com/taojin-6/volumetric_kit_sensor#orbbec-driver);
off Apple also `-DVS_WITH_FFMPEG=ON -DVS_WITH_CUDA=ON`, with FFmpeg and the
CUDA toolkit for sensor's
[video decoders](https://github.com/taojin-6/volumetric_kit_sensor#video-decoders)),
or DeckLink's (`-DVS_WITH_DECKLINK=ON`, Linux x86-64, with the DeckLink SDK's
headers at `DeckLinkSDK_ROOT`) -- `VC_WITH_RIG` builds the `app` tier and
`calib_rig` too, with an opener for each driver built. To build against a
local core, sensor or gfx checkout instead of the pinned one, pass
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<path>`,
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_SENSOR=<path>` or
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_GFX=<path>`.

## Build and test

```bash
calib_root="$(git rev-parse --show-toplevel)"
cmake -S "$calib_root" -B "$calib_root/build" -DCMAKE_BUILD_TYPE=Release \
  -DVC_WITH_RIG=ON  # leave out without the Vulkan SDK
cmake --build "$calib_root/build" --parallel
```

Run the tests as [AGENTS.md](AGENTS.md#build-and-validation) does. Then run
the demo:

```bash
./build/examples/example_01_project_point --config config/project_point.json
```

The demo reads the family's versioned JSON scenario format through
`volumetric_kit::core_config`. Core owns defaults and static validation, including
unknown fields and application compatibility. Edit `config/project_point.json`
to change the camera and point. The demo accepts only `--config PATH` and `--help`;
the calibration library itself keeps its core_base/sensor_camera dependency.

Fit one camera's lens from saved pictures of a checkerboard, offline, with the
stable API alone: here 14 x 9 squares of 20 mm. It prints every gate missed,
and writes the lens file, the report and the residuals to `out/` (which must
exist).

```bash
./build/examples/example_02_lens_from_images bm-1 14 9 20 out images/bm-1/*.pgm
```

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

Its keys, on a press (none while a text field has the keyboard):

| Key | Camera images | 3D view |
| --- | --- | --- |
| Space, →, Page Down, S | capture | to the images, "press again" |
| Backspace, ←, Page Up | drop the last capture | to the images |
| C, F5 | Calibrate | Calibrate |
| V, B, `.` | to the 3D view, once calibrated (extrinsic mode) | to the images |
| Escape | cancel Calibrate | cancel Calibrate |

### Intrinsic and extrinsic mode

The scenario's `operation` is calib_rig's mode:

| | Intrinsic (`calibrate_intrinsics`) | Extrinsic (`calibrate`) |
| --- | --- | --- |
| Board | the checkerboard, its squares and size edited | the rig's ChArUco board |
| Each camera's live lens | `source.calibration`'s at the stream's size, else the driver's, else a stand-in | the file's or the driver's; none stops the run |
| A capture | a still board, one view enough | a still board |
| Calibrate | once a camera is ready (the Lenses panel) | once the session's links are |
| Its result | each ready camera's lens, in a lens-only `array_calibration.json`; a factory lens is reported, never written | the rig's poses, and the 3D view |

The checkerboard is the intrinsic board: the rig's 5 x 3 ChArUco board has 8
inner corners, fewer than a lens view takes (12), and is refused there.
calib_rig fits a DeckLink pair's lenses; it does not solve the pair's poses
yet:

```json
{"version": 1, "operation": "calibrate_intrinsics",
 "source": {"kind": "live", "driver": "decklink",
   "cameras": [{"id": "bm-1", "input": "SDI 2"}, {"id": "bm-2", "input": "SDI 1"}],
   "pairs": [{"left": "bm-1", "right": "bm-2"}],
   "calibration": "bm_pair/array_calibration.json"},
 "output": {"path": "sessions"}}
```

A pair's lenses, in the lab:

1. **Board.** Measure the checkerboard's square with calipers over 12
   squares, check the board is flat (note any bow) with about a square of
   white margin around it, and enter it with "square measured" ticked.
2. **Cameras.** Tape focus and iris; fix exposure and white balance over
   PTP.
3. **Session.** Run the scenario above, `source.calibration` the pair's
   current file. Capture about 40 still boards, tilted 20-45 degrees, near
   and far, pushing the board into each corner and along both sides until
   each camera's line in the Lenses panel turns green; then press C. The
   lens file is `<session>/results/<RRR>_<stamp>/array_calibration.json`,
   its report beside it.

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

`VC_BUILD_TESTS`, `VC_BUILD_EXAMPLES`, `VC_BUILD_APPS` and
`VC_WARNINGS_AS_ERRORS` default ON only when `volumetric_kit_calib` is the
top-level project, so a FetchContent build of it never fails on a warning your
compiler adds. `volumetric_kit::calib_compute`, `volumetric_kit::calib_detect`,
`volumetric_kit::calib_solve` and `volumetric_kit::calib_quality` are always
built. Set `VC_WITH_RIG` ON before calib is made available to link
`volumetric_kit::calib_source`, `volumetric_kit::calib_capture`,
`volumetric_kit::calib_depth`, `volumetric_kit::calib_viz` and, with
`VC_BUILD_APPS`, GLFW and a sensor driver, `volumetric_kit::calib_app`.
`VC_SANITIZE` is a semicolon list, empty (off) by default —
e.g. `-DVC_SANITIZE="address;undefined"`. calib is consumed by commit, as
above, and installs nothing: it has no install rules or package config
([DECISIONS.md](DECISIONS.md#2026-10-22--consumed-by-commit-never-installed)).

calib needs `volumetric_kit_core` 0.29.0 or newer, with its vulkan tier for
`VC_WITH_RIG`, and 0.40.0 or newer with its config tier for `calib_app`; and
`volumetric_kit_sensor` 0.21.0 or newer. An application that declares either
before calib, to share it with another sibling, pins at least that: on an
older one, calib's configure fails at `vkc_require_core` or
`vkc_require_sibling`. Without `VC_WITH_RIG`,
calib makes sensor available camera-only (`VS_CAMERA_ONLY`), so an application
that uses sensor beyond its camera tier makes sensor available, with its
options, before calib; with it, an application that makes sensor available
first builds it beyond its camera tier.

## Development

```bash
pre-commit install   # clang-format + cmake-format + hygiene hooks
```

## License

Released under the [MIT License](LICENSE). Each source file carries an
`SPDX-License-Identifier: MIT` header.
