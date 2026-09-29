# volumetric_kit_calib

A calibration **companion** for `volumetric_kit`: the intelligence layer around
the calibration solve — live acquisition guidance, physical diagnostics, and a
requirements solver. Full design + domain model in `DESIGN.md`, which is
local-only (gitignored, as in gfx); from a worktree, read it at
`../../DESIGN.md`.

## Naming conventions (use these consistently)

- Package/repo: `volumetric_kit_calib`
- Namespace: `volumetric_kit::calib`. In docs, `vc::` abbreviates
  `volumetric_kit::calib::`; `vg::` abbreviates `volumetric_kit::gfx::`.
- Headers: `include/volumetric_kit/calib/<tier>/…` → e.g.
  `#include "volumetric_kit/calib/core/result.hpp"`
- CMake: `find_package(volumetric_kit_calib)`; component targets
  `volumetric_kit::calib_core`, `…_compute`, `…_detect`, `…_solve`, `…_quality`,
  `…_color`, `…_capture`, `…_viz`, `…_app`; umbrella alias `volumetric_kit::calib`.
- Macros/export: `VC_*` (e.g. `VC_TRY`, `VC_CHECK`, `VC_CORE_API`).

## Architecture (tiered)

`core` → `compute` → {`detect`, `solve`, `color`} → `quality` → `viz` → `app`
(+ `capture` off `core`). The dependency rule is strict: a tier may depend only
on tiers to its left. The headless **brain**
(`core`+`compute`+`detect`+`solve`+`quality`) builds with **no**
`volumetric_kit_gfx` dependency; only `viz`/`app` pull in the renderer.

## Locked decisions (see DESIGN.md for the full rationale)

- **Consume `volumetric_kit_gfx` for all rendering + UI; do not re-implement.**
  The 3D copilot + ImGui dashboards ride `vg::` pipelines.
- **Mobile (Android + iOS) is first-class and fully on-device, solve included.**
  Target-based BA is small (a known board fixes structure → hundreds of sparse
  params → Ceres on an ARM CPU in tens of ms), so the solve runs on CPU
  everywhere; the GPU (Vulkan) is for dense per-pixel work.
- **The physically-parameterized sensor/lens model is the spine** — the quality
  tier is physical, not heuristic, so `SensorProfile`/`LensProfile` are required
  inputs, not optional metadata.
- **The legacy `camera_calibration` Python scripts are the reference oracle**
  (parity target), then subsumed. They are a separate checkout beside the
  `volumetric_kit/` folder: `../../camera_calibration/` from the primary
  checkout.

## Open decisions

- **The GPU compute path is under review.** A CPU backend is the correctness
  oracle either way. DESIGN.md plans calib's own Vulkan (portable; Apple via
  MoltenVK) + opt-in desktop-NVIDIA CUDA backends; the alternative being weighed
  is running calib's GPU work on recon's core over a shared `VkDevice`. Native
  Metal is deferred in both. Don't build compute-backend code until this is
  settled.

## Error handling (no exceptions cross the API)

Mobile consumers build with `-fno-exceptions`. Fallible calls return `Status`
(success or an error domain + message) or `Result<T>` (a value or a `Status`).
Propagate with `VC_TRY` / `VC_ASSIGN`. Programmer errors (reading an error
`Result`'s value) fail fast via `VC_CHECK`, never throw. Solver failures use the
`Status::Code::Numerical` domain.

## Where to start

The repo is currently `core`-tier scaffolding: build system, `Status`/`Result`,
the data model, and the `PinholeBrownConrady` camera model, all tested. The next
milestones (DESIGN.md roadmap): C2 `detect` + CPU `compute` diffed against the
legacy Python oracle; C3 `solve/intrinsics` + covariance; C4 the headline brain
(coverage / next-best-view / image-gate / optics / requirements solver).

## Conventions

- **Doxygen on every public class/function**, matching
  `include/volumetric_kit/calib/core/result.hpp`: `@file`/`@brief` on the header,
  `@brief` + a `@code … @endcode` example per class, and `@brief`/`@param`/
  `@return` (`@pre` where relevant) on methods; accessors may be a single
  `/// @return`.
- **Mark deferred work inline with a greppable `// TODO:`** rather than tracking
  it only in prose or commits.
- Prefer plain, behavior-level tests over friend-class backdoors into private
  state; the `core` tier is pure CPU so its tests run on every machine.

## Working preferences

- Implement every change in a dedicated **git worktree** under `.worktrees/`
  (`git worktree add .worktrees/<branch> -b <branch>`) — never a sibling in the
  parent folder — and remove it with `git worktree remove` once the PR merges.
  A worktree has no `DESIGN.md` (it is gitignored); read the primary
  checkout's.
