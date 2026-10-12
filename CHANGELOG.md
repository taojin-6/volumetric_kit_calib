# Changelog

All notable changes to `volumetric_kit_calib` are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- calib's camera model is recon's (DECISIONS.md, 2026-10-12). Migrating:
  `PinholeBrownConrady`, `ImageSize` and `calib/core/camera_model.hpp` are
  removed; use `volumetric_kit::recon::camera::CameraModel` and
  `check_camera_model` from `volumetric_kit/recon/camera/camera_model.hpp`.
  `DetectedPoint::pixel` is recon's `camera::Vec2d` (`glm::dvec2`), not an
  Eigen vector. calib fetches recon's camera tier alone (`VR_CAMERA_ONLY`) and
  installs it beside itself; its package finds recon, and the core through it.
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

- Windows and MSVC support, which no calib target or CI leg used
  (DECISIONS.md, "Platforms"): calib builds with GCC or Clang for Linux,
  macOS, iOS and Android, as the core does. The MSVC warning flags
  (`/W4`, `/WX`) and the sanitizer option's MSVC refusal are gone.
