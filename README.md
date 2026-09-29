# volumetric_kit_calib

A calibration **companion** for the `volumetric_kit` capture rig — not another
solver, but the **intelligence layer around the solve**: live acquisition
guidance, physical diagnostics ("is this error good, given the sensor's noise
floor?"), and a requirements solver (spec → binding hardware constraint →
prescription). It covers single-camera intrinsics, stereo, multi-camera bundle
adjustment, fisheye, cross-camera color, and the optics that gate quality
(depth-of-field, diffraction, focus). See [`DESIGN.md`](./DESIGN.md) for the
full architecture and the domain model.

> **Status:** Early scaffolding. The `core` tier — exception-free
> `Status`/`Result`, the physically-parameterized data model (camera/lens/sensor
> models, board target, observation), and the `PinholeBrownConrady` camera model
> — builds and is tested (GoogleTest). The compute backends, detection, solve,
> quality, color, capture, viz, and app tiers are not yet implemented; the
> existing `../camera_calibration/` Python scripts remain the reference oracle.

## Architecture (tiered)

`core` → `compute` → {`detect`, `solve`, `color`} → `quality` → `viz` → `app`
(+ `capture` off `core`). A tier may depend only on tiers to its left. The
headless "brain" (`core`+`compute`+`detect`+`solve`+`quality`) builds with **no**
`volumetric_kit_gfx` dependency; only the `viz`/`app` tiers pull in the renderer.

- **Compute backends:** CPU (oracle) + Vulkan (portable; Apple via MoltenVK) +
  CUDA (opt-in desktop-NVIDIA). Native Metal is deferred. See `DESIGN.md`.
- **Mobile (Android + iOS):** a first-class, fully on-device target — the
  board-constrained solve is small enough to run on an ARM CPU.

## Prerequisites

The `core` tier needs a C++17 compiler, CMake ≥ 3.21, and Eigen:

```bash
brew install cmake eigen          # macOS
# sudo apt-get install cmake libeigen3-dev   # Ubuntu
```

(Later tiers add OpenCV for `detect`, Ceres for `solve`, and
`volumetric_kit_gfx` for `viz`/`app`.)

## Build and test

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

Run the demo:

```bash
./build/examples/example_01_project_point
```

## Use it in your project

```cmake
# FetchContent (pin GIT_TAG to a release tag or commit SHA):
include(FetchContent)
FetchContent_Declare(
  volumetric_kit_calib
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_calib.git
  GIT_TAG main)
FetchContent_MakeAvailable(volumetric_kit_calib)

target_link_libraries(your_app PRIVATE volumetric_kit::calib_core)
```

`VC_BUILD_TESTS`, `VC_BUILD_EXAMPLES`, and `VC_INSTALL` default ON only when
`volumetric_kit_calib` is the top-level project. `VC_WITH_VULKAN` / `VC_WITH_CUDA`
are reserved for the compute-backend tiers (not yet wired). `VC_SANITIZE` is a
semicolon list, empty (off) by default — e.g. `-DVC_SANITIZE="address;undefined"`.

## Development

```bash
pre-commit install   # clang-format + cmake-format + hygiene hooks
```

## License

Released under the [MIT License](LICENSE). Each source file carries an
`SPDX-License-Identifier: MIT` header.
