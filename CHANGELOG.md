# Changelog

All notable changes to `volumetric_kit_calib` are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

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
- `volumetric_kit_core` is pinned at main after its PR #13 (was PR #1). The
  base tier calib uses kept its API, so calib's code is unchanged and its
  consumers have nothing to migrate. What reaches them:
  - The core supports only GCC and Clang -- it dropped Windows and MSVC -- and
    calib's public headers include its `result.hpp`, so calib does too.
  - Replacing the log handler from inside a handler call no longer deadlocks
    when another thread finishes a call on the old handler.
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

### Removed

- Windows and MSVC support, which no calib target or CI leg used
  (DECISIONS.md, "Platforms"): calib builds with GCC or Clang for Linux,
  macOS, iOS and Android, as the core does. The MSVC warning flags
  (`/W4`, `/WX`) and the sanitizer option's MSVC refusal are gone.
