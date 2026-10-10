# Calibration decisions

This record preserves the constraints previously committed in `CLAUDE.md`,
and the choices made since. [AGENTS.md](AGENTS.md) is the shared working guide;
[README.md](README.md) describes the implemented scope. The more extensive
domain model and rationale remain in the intentionally gitignored local
`DESIGN.md`; this change does not publish that document.

## Locked decisions

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
- **Capture sessions keep the legacy scripts' image names**
  (`<serial>/calib_img_<serial>_<n>.png`), so the legacy bundle adjustment
  reads a session unchanged.
- **The solve's world frame is the rig's sync primary** (2026-09-28), which a
  session records as its reference camera. The legacy solve gauged on
  another camera (CL2A141000G), so its extrinsics differ by that rigid
  transform.

- **Build on `volumetric_kit_core`.** calib uses the family's shared
  foundation ([volumetric_kit_core](https://github.com/taojin-6/volumetric_kit_core))
  instead of a bottom layer of its own: `Status`, `Result`, `VKC_TRY`,
  `VKC_ASSIGN`, `VKC_CHECK` and the log sink from its base tier now, and the
  camera models (its camera tier) and capture drivers (its sensor tier) as
  those land. calib names them as the core does -- `core::Status` in calib's
  namespaces, `vkc::Status` outside -- with no `vc::` re-exports (amended
  2026-10-05; see the dated entry), so a calib error passes to `recon` or
  `gfx` unchanged. The core is fetched pinned by commit (`third_party/`), and the
  installed package re-finds it at the minor version it was built with. Why:
  `recon`'s and `gfx`'s copies of the same bottom layer had drifted (the core's
  DECISIONS.md has the evidence); one copy means a fix lands once.
- **GPU work runs on the core's vulkan tier, on a `VkDevice` shared with
  `recon` and `gfx`** (MoltenVK on Apple). calib carries no GPU backend of its
  own, so the reserved `VC_WITH_VULKAN` / `VC_WITH_CUDA` options are gone. Why:
  frames `recon`'s decoders leave in Vulkan buffers can be read in place only
  on the same device -- a calib device of its own would copy every frame
  through the host -- and the family keeps one Vulkan core, not three. CUDA is
  added per kernel only where profiling shows a need, in the core; native Metal
  stays deferred. A CPU backend remains the correctness oracle. The core's
  vulkan tier has landed, and calib's pin carries it; calib's build leaves it
  off (`VKC_WITH_VULKAN`), and so needs no Vulkan SDK, until calib writes GPU
  code.
- **Linux, macOS, iOS and Android, with GCC or Clang; never Windows or MSVC**
  (2026-10-04). See "Platforms" below.
- **The rational lens model comes first; fisheye is planned, not built.** The
  Orbbec Femto Mega reports its factory lens in the rational model (k1–k6,
  p1, p2), which `recon`'s rig calibration file also stores, so that model is
  next. Fisheye (Kannala-Brandt) waits, but unprojection returns unit 3D rays
  and stored lenses carry a model name, so it can be added without breaking
  either.

### Platforms

calib builds for Linux, macOS, iOS and Android, with GCC or Clang (Apple's
included), as the family's core does (its DECISIONS.md, "Platforms"). Windows
and MSVC are not supported: the core, whose headers calib's public headers
include, builds nowhere else; no calib target or CI leg runs on Windows; and a
Windows branch could only rot untested. The code and build assume POSIX and those compilers
outright, with no `_WIN32` or `_MSC_VER` paths and no MSVC flags. Supporting
Windows would be a new decision, landing with its CI leg -- and the core's.

## Open decisions

None at present. (The GPU compute path, open until 2026-10-03, is now the
locked decision above.)

## 2026-10-05 — calib writes the core's names

calib named the core's `Status`, `Result` and `to_string` in its own
namespace through using-declarations in `calib/core/result.hpp`, as recon and
gfx did in theirs. recon (its #167) and gfx (its #106) have since dropped
their re-exports: two spellings of one type cost every reader, and each
re-export header is one more file to keep in step with the core's. calib
follows, so the family writes the core's names one way.

- **The rule.** calib's namespaces write `core::Status`, `core::Result` and
  `core::to_string`; tests, examples and consumers write
  `volumetric_kit::core::` (a `vkc` alias). calib already used the core's
  `VKC_*` macros, so no macro moves. calib declares no `core` namespace of its
  own, so `core::Status` in its namespaces finds `volumetric_kit::core`.
- **What goes.** `calib/core/result.hpp`, whose includers now include the
  core's `volumetric_kit/core/base/result.hpp`, and `result_test`, which only
  checked that calib's names were the core's types.
- **What it costs.** A consumer writing `vc::Status` or `vc::Result` writes
  `vkc::` on its next pin bump; calib's own names are unchanged.

## 2026-10-03 — The shared core and the GPU path

A review of the repository against the goals -- generic hardware support,
compute on the right processor, and few memory transfers on both discrete
and unified-memory GPUs -- found the family's Vulkan cores already drifting.
The family created `volumetric_kit_core` and settled the GPU path on its
vulkan tier over a shared `VkDevice`. calib adopts the core's base tier
first; its camera model moves to the core's camera tier when that lands.

## 2026-10-02 — Shared agent guidance

`AGENTS.md` now owns the shared working rules and task-based reading map.
`CLAUDE.md` imports it with `@AGENTS.md`. This decision record keeps the
previously committed constraints available to both agents without copying the
local-only design document. Update shared rules here and in AGENTS.md as
appropriate when a decision changes; keep tool-specific configuration separate.
