# Changelog

All notable changes to `volumetric_kit_calib` are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- calib builds on `volumetric_kit_core`, fetched pinned by commit, and its
  installed package re-finds it. `Status` and `Result` are the core's types
  (using-declarations in `volumetric_kit::calib`), so they pass to `recon` and
  `gfx` unchanged. Migrating code on `main`: `VC_TRY` / `VC_ASSIGN` /
  `VC_CHECK` → `VKC_TRY` / `VKC_ASSIGN` / `VKC_CHECK`;
  `volumetric_kit/calib/core/check.hpp` is gone (include the core's
  `volumetric_kit/core/base/check.hpp`); a failed check now reports through the
  core's log sink, which prints to stderr by default; `std::move(r).value()`
  returns the value rather than a reference into `r`.
- The reserved `VC_WITH_VULKAN` / `VC_WITH_CUDA` options are removed: GPU work
  runs on the core's vulkan tier (DECISIONS.md).
- clang-format is pinned to 22.1.8, as in the sibling repositories.

### Added

- Build-system foundation: top-level CMake (C++17), tiered component layout, the
  `volumetric_kit::calib_core` library target + `volumetric_kit::calib` umbrella
  alias, and first-class install/export so the library is consumable via both
  FetchContent and `find_package(volumetric_kit_calib)`.
- Dependency strategy: Eigen from the system (the linear-algebra spine, a PUBLIC
  dep); googletest vendored via pinned FetchContent for the test suite only.
- `core` tier:
  - Exception-free error handling from `volumetric_kit_core`'s base tier:
    `Status` (with a `Numerical` domain for solver failures) and `Result<T>`,
    both `[[nodiscard]]`, named in calib's namespace, plus the core's `VKC_TRY`
    / `VKC_ASSIGN` propagation macros and `VKC_CHECK` fail-fast contract check.
  - Physically-parameterized data model: `SensorProfile`, `LensProfile`,
    `Target`, `Observation` — the inputs the quality tier needs to be physical
    rather than heuristic. Profile fields are `std::optional`, so "unknown" is
    never confused with a real zero (e.g. a global shutter's line time).
  - `PinholeBrownConrady` camera model with validated construction (image size,
    intrinsics and distortion), closed-form `project`, and a Newton `unproject`
    that returns an error rather than a wrong ray past the distortion's fold.
    Both return `Result`, so a point behind the camera is an error, not an
    abort.
- GoogleTest `tests/` (version smoke test, `Status`/`Result` incl. death tests,
  camera model against OpenCV `projectPoints` values, data model) and an
  `examples/01_project_point` CLI.
- Lint/CI hygiene: `.clang-format`, `.cmake-format.yaml`, and a pre-commit
  config (clang-format, cmake-format, trailing-whitespace, end-of-file-fixer).
