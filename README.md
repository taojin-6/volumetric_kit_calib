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
> `Status`/`Result` — builds and is tested (GoogleTest); so, with
> `VC_WITH_OPENCV`, do the `compute` tier's OpenCV bridge, the `detect` tier's
> board detection, the `solve` tier's rig solve and the `quality` tier's live
> checks, and, with `VC_WITH_RIG`, the `capture` tier's session writer. The
> metrics after a solve and the color, viz, and app tiers are not yet
> implemented; the legacy `camera_calibration` Python
> scripts (a separate checkout beside the `volumetric_kit/` folder) remain the
> reference oracle.

[AGENTS.md](AGENTS.md) is the shared working guide for Codex and Claude Code;
`CLAUDE.md` imports it. [DECISIONS.md](DECISIONS.md) records the locked
constraints and open GPU-backend choice without publishing the local design.

## Architecture (tiered)

`core` → `compute` → {`detect`, `solve`, `color`} → `quality` → `viz` → `app`
(+ `capture` off `core`). A tier may depend only on tiers to its left. The
headless "brain" (`core`+`compute`+`detect`+`solve`+`quality`) builds with **no**
`volumetric_kit_gfx` dependency; only the `viz`/`app` tiers pull in the renderer.

- **Shared core:** every tier builds on
  [`volumetric_kit_core`](https://github.com/taojin-6/volumetric_kit_core), the
  family's shared foundation. Its base tier supplies calib's `Status`/`Result`
  today. Its vulkan tier will supply GPU compute -- calib's build leaves it off
  until calib writes GPU code, so it needs no Vulkan SDK.
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
  resolution or by the live view's faster hybrid.
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
- **Mobile (Android + iOS):** a first-class, fully on-device target — the
  board-constrained solve is small enough to run on an ARM CPU.

## Prerequisites

The `core` tier needs a C++17 compiler -- GCC or Clang; Windows and MSVC are
not supported ([DECISIONS.md](DECISIONS.md#platforms)) -- CMake ≥ 3.21, and
GLM. The first configure fetches `volumetric_kit_core` and
`volumetric_kit_recon` (pinned by commit) and, for tests, googletest, so it
needs network access:

```bash
brew install cmake glm          # macOS
# sudo apt-get install cmake libglm-dev   # Ubuntu
```

`VC_WITH_OPENCV` (OFF by default) builds `compute` and the tiers on it, which
need an installed OpenCV 5 (5.0 or a later 5.x; the tests also use its
imgcodecs and calib modules): `brew install opencv` on macOS; on Ubuntu,
whose packages are OpenCV 4, one you build yourself. `VC_WITH_RIG` (OFF by
default) builds `capture`, over all of recon, which needs the Vulkan SDK and
a GLSL compiler: the core's vulkan tier
[prerequisites](https://github.com/taojin-6/volumetric_kit_core#prerequisites).
(Later tiers add `volumetric_kit_gfx` for `viz`/`app`.) To build against a
local core or recon checkout instead of the pinned one, pass
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<path>` or
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=<path>`.

## Build and test

```bash
calib_root="$(git rev-parse --show-toplevel)"
cmake -S "$calib_root" -B "$calib_root/build" -DCMAKE_BUILD_TYPE=Release \
  -DVC_WITH_OPENCV=ON -DVC_WITH_RIG=ON  # leave out what you lack
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

`VC_BUILD_TESTS`, `VC_BUILD_EXAMPLES`, `VC_INSTALL`, and `VC_WARNINGS_AS_ERRORS`
default ON only when `volumetric_kit_calib` is the top-level project, so a
FetchContent build of it never fails on a warning your compiler adds. Set
`VC_WITH_OPENCV` ON before calib is made available to link
`volumetric_kit::calib_compute`, `volumetric_kit::calib_detect`,
`volumetric_kit::calib_solve` and `volumetric_kit::calib_quality`, and
`VC_WITH_RIG` ON to link `volumetric_kit::calib_capture`.
`VC_SANITIZE` is a semicolon list, empty (off) by default —
e.g. `-DVC_SANITIZE="address;undefined"`. An installed calib carries the core
and recon (its camera tier, or all of it with `VC_WITH_RIG`) beside it, and
`find_package(volumetric_kit_calib)` finds all three, and the installed OpenCV
when calib was built with it. An application may find the core itself too,
before calib or after it; if it finds another core first, calib is not found,
and the message names both.

calib needs `volumetric_kit_core` 0.6.1 or newer, with its vulkan tier for
`VC_WITH_RIG`. An application that declares the core before calib, to share it
with `recon` or `gfx`, pins at least that: on an older core, calib's configure
fails at `vkc_require_core`, and an installed calib is not found. Without
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
