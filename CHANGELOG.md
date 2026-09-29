# Changelog

All notable changes to `volumetric_kit_calib` are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Build-system foundation: top-level CMake (C++17), tiered component layout, the
  `volumetric_kit::calib_core` library target + `volumetric_kit::calib` umbrella
  alias, and first-class install/export so the library is consumable via both
  FetchContent and `find_package(volumetric_kit_calib)`.
- Dependency strategy: Eigen from the system (the linear-algebra spine, a PUBLIC
  dep); googletest vendored via pinned FetchContent for the test suite only.
- `core` tier:
  - Exception-free error handling — `Status` (with a `Numerical` domain for
    solver failures) and `Result<T>`, plus the `VC_TRY` / `VC_ASSIGN`
    propagation macros and the `VC_CHECK` fail-fast contract check.
  - Physically-parameterized data model: `SensorProfile`, `LensProfile`,
    `Target`, `Observation` — the inputs the quality tier needs to be physical
    rather than heuristic.
  - `PinholeBrownConrady` camera model with validated construction, closed-form
    `project`, and iterative `unproject` (distortion removal).
- GoogleTest `tests/` (version smoke test, `Status`/`Result`, camera-model
  round-trip) and an `examples/01_project_point` CLI.
- Lint/CI hygiene: `.clang-format`, `.cmake-format.yaml`, and a pre-commit
  config (clang-format, cmake-format, trailing-whitespace, end-of-file-fixer).
