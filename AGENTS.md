# Shared repository instructions

This is the canonical working guide for Codex, Claude Code, and other agents.
Keep shared instructions here. `CLAUDE.md` imports this file with `@AGENTS.md`;
do not duplicate rules there or create a circular reference.

## Project and boundaries

`volumetric_kit_calib` is the calibration companion: acquisition guidance,
physical diagnostics, and a requirements solver around the calibration solve.
The implemented core and planned tiers are distinguished in
[README.md](README.md#architecture-tiered).

- Use `volumetric_kit_gfx` for rendering and UI; keep the headless brain free
  of that dependency.
- Mobile is first-class and fully on-device; the board-constrained solve runs
  on CPU. `SensorProfile` and `LensProfile` are optional until intrinsics work
  uses them; the extrinsics checks are geometric
  ([DECISIONS.md](DECISIONS.md#2026-10-14--live-checks-and-the-capture-rule)).
- calib builds on [`volumetric_kit_core`](https://github.com/taojin-6/volumetric_kit_core)
  (fetched, pinned by commit). Code two or more siblings need belongs in the
  core, as a task in that repository; cameras and sensors do not
  ([DECISIONS.md](DECISIONS.md#locked-decisions)). From the primary checkout
  it is `../volumetric_kit_core/`.
- Cameras are `volumetric_kit_recon`'s camera tier (`CameraModel`, rigid
  transforms), fetched alone (`VR_CAMERA_ONLY`) so the brain needs no Vulkan;
  calib keeps no camera model of its own. `VC_WITH_RIG` fetches all of recon
  for the `capture` tier, and gfx for the `viz` tier.
- GPU work runs on the core's vulkan tier, on a `VkDevice` shared with `recon`
  and `gfx` ([DECISIONS.md](DECISIONS.md#locked-decisions)); calib keeps no
  GPU backend of its own. A CPU backend is the correctness oracle.
- The legacy `camera_calibration` Python checkout is the parity oracle;
  it sits beside the family directory (`../../camera_calibration/` from the
  primary checkout). Keep this task's changes in this repository.

## Read what the task needs

| Task | Read |
| --- | --- |
| Architecture, implemented scope, prerequisites | [README.md](README.md) and the affected public headers |
| Locked constraints or GPU backend choice | [DECISIONS.md](DECISIONS.md) |
| `Status`/`Result`, the shared tiers | The core's README.md, DECISIONS.md and `include/volumetric_kit/core/` |
| Local detailed design and domain model | The primary checkout's gitignored `DESIGN.md`, when the task needs it |
| Formatting and CI setup | [CONTRIBUTING.md](CONTRIBUTING.md), `.pre-commit-config.yaml`, relevant workflows |

`DESIGN.md` is intentionally local-only and absent from clones/worktrees. From
`.worktrees/<branch>/`, the primary copy is `../../DESIGN.md`. Do not force-add
or publish it. The committed decision record remains available without it.
Read only the sections needed for the current task.

## Naming conventions

- Package/repo: `volumetric_kit_calib`
- Namespace: `volumetric_kit::calib`. In docs, `vc::` abbreviates
  `volumetric_kit::calib::`; `vg::` abbreviates `volumetric_kit::gfx::`.
- Headers: `include/volumetric_kit/calib/<tier>/…` → e.g.
  `#include "volumetric_kit/calib/core/target.hpp"`
- volumetric_kit_core's types, functions and macros keep the core's names:
  `core::Status` inside calib's namespaces, `vkc::Status` in tests and
  examples, from the core's own headers. calib re-exports none of them.
- CMake: component targets `volumetric_kit::calib_core`, `…_compute`,
  `…_detect`, `…_solve`, `…_quality`, `…_color`, `…_capture`, `…_depth`,
  `…_viz`, `…_app`; umbrella alias `volumetric_kit::calib`. Applications fetch
  calib by commit; it installs nothing.
- Macros/export: `VC_*` for calib's own (e.g. `VC_CORE_API`, `VC_BUILD_TESTS`);
  the error-handling macros are the core's (`VKC_TRY`, `VKC_ASSIGN`,
  `VKC_CHECK`).

## Architecture

`core` → `compute` → {`detect`, `solve`, `color`} → `quality` →
{`depth`, `viz`} → `app` (+ `capture` off `core`). The dependency rule is
strict: a tier may depend only on tiers to its left. The headless **brain**
(`core`+`compute`+`detect`+`solve`+`quality`) builds with **no**
`volumetric_kit_gfx` dependency; only `viz`/`app` pull in the renderer.
`depth` runs the depth checks' kernels on the core's vulkan tier, with neither
gfx nor recon's sensor tiers.

## Error handling

calib builds with C++ exceptions enabled, as its siblings do
([DECISIONS.md](DECISIONS.md#2026-10-10--calib-builds-with-exceptions-enabled)),
and no exception crosses its API. Fallible calls return `Status`
(success or an error domain + message) or `Result<T>` (a value or a `Status`)
-- the core's `core::Status` and `core::Result<T>`, from
`volumetric_kit/core/base/result.hpp`. Propagate with `VKC_TRY` /
`VKC_ASSIGN`. Programmer errors (reading an error `Result`'s value) fail fast
via `VKC_CHECK`, never throw. Solver failures use the
`core::Status::Code::Numerical` domain. A public function that calls OpenCV
runs its body once through `compute::guarded`, which returns what OpenCV (or
anything else) throws as a `Status`.

## Conventions

C++17, with no compiler extensions, built with GCC or Clang for Linux, macOS,
iOS and Android. Windows and MSVC are unsupported
([DECISIONS.md](DECISIONS.md#platforms)): add no `_WIN32`/`_MSC_VER` paths or
MSVC flags.

- **Doxygen on every public class/function**, matching
  `include/volumetric_kit/calib/core/target.hpp`: `@file`/`@brief` on the header,
  `@brief` + a `@code … @endcode` example per class, and `@brief`/`@param`/
  `@return` (`@pre` where relevant) on methods; accessors may be a single
  `/// @return`.
- **Mark deferred work inline with a greppable `// TODO:`** rather than tracking
  it only in prose or commits.
- Prefer plain, behavior-level tests over friend-class backdoors into private
  state; the `core` tier is pure CPU so its tests run on every machine.

## Working with Git

- Implement each task in a dedicated branch and git worktree under
  `.worktrees/`; never create a sibling worktree in the parent folder.
  Concurrent Claude/Codex tasks use separate worktrees.
- Preserve unrelated local changes and other worktrees. Remove your worktree
  with `git worktree remove` after its PR merges.
- Use absolute paths for `git -C`, `cmake -S/-B`, and file operations so work
  cannot spill into a sibling checkout.
- Use Conventional Commits, e.g. `feat(core): …`, `fix(core): …`, `docs: …`.
- Assign PRs to the authenticated user (`gh pr create --assignee @me`).
- Commit shared decisions and handoff context with the project so either agent
  can continue the task.

## Build and validation

Run from the task's worktree. The build requires CMake ≥ 3.21, a C++17
compiler, GLM, an installed OpenCV 5, and network access on the first
configure (the core, recon, gfx with `viz`, and googletest are fetched);
`VC_WITH_RIG` needs the Vulkan SDK. See README and CI for platform
dependencies.

```sh
calib_root="$(git rev-parse --show-toplevel)"
cmake -S "$calib_root" -B "$calib_root/build" -DCMAKE_BUILD_TYPE=Release \
  -DVC_BUILD_TESTS=ON -DVC_WITH_RIG=ON
cmake --build "$calib_root/build" --parallel
ctest --test-dir "$calib_root/build" --output-on-failure \
  -j "$(getconf _NPROCESSORS_ONLN)" -LE "slow|package"
pre-commit run --all-files --show-diff-on-failure
git -C "$calib_root" diff --check
```

That `ctest` is the default suite. It leaves out the labels `slow`, the full
statistical sweeps, and `package`, the nested configures of calib's build;
`-L slow` and `-L package` run them, as CI does. Each test binary but the
out-of-memory ones is one ctest test, named for its file (`detect_charuco`);
run one case with the binary's `--gtest_filter`, or `GTEST_FILTER` under
ctest.

- Use the pinned formatting tools in `.pre-commit-config.yaml`; CI runs the
  same hooks. Set the build type explicitly and report it with measurements.
- For code changes, build affected targets and run relevant tests, with
  regression coverage for changed behavior. Broaden validation when shared
  contracts or the change's scope warrant it.
- Documentation-only changes need formatting, link, and consistency checks;
  no build is needed. Hooks can be scoped with `pre-commit run --files`.
- Core tests are CPU-only. The `depth` and `viz` tiers' GPU tests use the
  core's Vulkan fixtures, which skip without a device; run them under the
  validation layer with `VKC_TEST_VALIDATION=1` (on macOS, with
  `DYLD_FALLBACK_LIBRARY_PATH=/opt/homebrew/lib` so the layer loads).
- The `app` tier and `calib_rig` also need an installed GLFW and
  `-DVR_WITH_ORBBEC=ON`, as CI's rig job builds them, and off Apple
  `-DVR_WITH_FFMPEG=ON -DVR_WITH_CUDA=ON`. No test opens a camera: the app's
  tests drive fake sensors.

## Keeping guidance current

- Keep this guide concise (roughly 100–200 lines). Put detailed contracts and
  rationale in DECISIONS.md or the relevant committed integration document.
- Update a changed decision and its rationale in the same commit as the code,
  then update any essential rule here and affected README/contributor guidance.
  Preserve the distinction between locked choices, open questions, and plans.
- Keep local-only DESIGN.md local. `CLAUDE.md` stays an import; add only
  Claude-specific instructions below it if a concrete need arises.
