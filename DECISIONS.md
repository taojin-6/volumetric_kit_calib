# Calibration decisions

This record preserves the constraints and unresolved choice previously
committed in `CLAUDE.md`. [AGENTS.md](AGENTS.md) is the shared working guide;
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

## Open decisions

- **The GPU compute path is under review.** A CPU backend is the correctness
  oracle either way. DESIGN.md plans calib's own Vulkan (portable; Apple via
  MoltenVK) + opt-in desktop-NVIDIA CUDA backends; the alternative being weighed
  is running calib's GPU work on recon's core over a shared `VkDevice`. Native
  Metal is deferred in both. Don't build compute-backend code until this is
  settled.

## 2026-10-02 — Shared agent guidance

`AGENTS.md` now owns the shared working rules and task-based reading map.
`CLAUDE.md` imports it with `@AGENTS.md`. This decision record keeps the
previously committed constraints available to both agents without copying the
local-only design document. Update shared rules here and in AGENTS.md as
appropriate when a decision changes; keep tool-specific configuration separate.
