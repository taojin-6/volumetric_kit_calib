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
> and its check, observations, view and capture records, lens and sensor
> profiles) over recon's camera model and the core's exception-free
> `Status`/`Result` — builds and is tested (GoogleTest). The compute backends,
> detection, solve, quality, color, capture, viz, and app tiers are not yet
> implemented; the legacy `camera_calibration` Python scripts (a separate
> checkout beside the `volumetric_kit/` folder) remain the reference oracle.

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
  still needs no Vulkan SDK; the `capture` tier will take frames from recon's
  sensor tier and its drivers ([DECISIONS.md](DECISIONS.md#locked-decisions)).
- **Compute:** a CPU backend is the correctness oracle. GPU work runs on the
  core's vulkan tier, on a `VkDevice` shared with `recon` and `gfx`, so frames
  pass between them without copies (MoltenVK on Apple; native Metal deferred).
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

(Later tiers add OpenCV for `detect`, Ceres for `solve`, and
`volumetric_kit_gfx` for `viz`/`app`.) To build against a local core or recon
checkout instead of the pinned one, pass
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<path>` or
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=<path>`.

## Build and test

```bash
calib_root="$(git rev-parse --show-toplevel)"
cmake -S "$calib_root" -B "$calib_root/build" -DCMAKE_BUILD_TYPE=Release
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
FetchContent build of it never fails on a warning your compiler adds.
`VC_SANITIZE` is a semicolon list, empty (off) by default —
e.g. `-DVC_SANITIZE="address;undefined"`. An installed calib carries the core
and recon's camera tier beside it, and `find_package(volumetric_kit_calib)`
finds all three.

calib needs `volumetric_kit_core` 0.1.0 or newer. An application that declares
the core before calib, to share it with `recon` or `gfx`, pins at least that:
on an older core, calib's configure fails at `vkc_require_core`, and an
installed calib is not found. calib makes recon available camera-only, so an
application that uses recon beyond its camera tier makes recon available, with
its options, before calib.

Until [volumetric_kit_core#31](https://github.com/taojin-6/volumetric_kit_core/issues/31)
is fixed, a second read of the core's package in one scope recurses in
`vkc_require_core`. So a consumer of an installed calib finds the core only
through it, and finds calib and recon both `REQUIRED`.

## Development

```bash
pre-commit install   # clang-format + cmake-format + hygiene hooks
```

## License

Released under the [MIT License](LICENSE). Each source file carries an
`SPDX-License-Identifier: MIT` header.
