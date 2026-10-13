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

- **Build on `volumetric_kit_core`.** calib uses the family's shared
  foundation ([volumetric_kit_core](https://github.com/taojin-6/volumetric_kit_core))
  instead of a bottom layer of its own: `Status`, `Result`, `VKC_TRY`,
  `VKC_ASSIGN`, `VKC_CHECK` and the log sink from its base tier. calib names
  them as the core does -- `core::Status` in calib's namespaces, `vkc::Status`
  outside -- with no `vc::` re-exports (amended 2026-10-05; see the dated
  entry), so a calib error passes to `recon` or `gfx` unchanged. The core is
  fetched pinned by commit (`third_party/`), and the installed package
  re-finds it, through recon's, holds it to the minor version calib was built
  with, and requires calib's floor of it (`vkc_require_core`). Why:
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
- **The camera model is recon's; capture drivers come from
  `volumetric_kit_recon`'s sensor tier** (2026-10-08, amended 2026-10-12; see
  the dated entry). calib's cameras are recon's `camera::CameraModel`, from its
  camera tier, and its `capture` tier will take frames from recon's
  `IRgbdSensor` and `SensorArray` rather than drive a device itself. Why: the
  core has no camera or sensor tier (core PR #14 proposed one and was
  closed), and recon's DECISIONS.md (2026-10-06, "The family's camera
  vocabulary is recon's `camera` tier") makes recon what calib builds on for
  cameras and sensors.
- **Linux, macOS, iOS and Android, with GCC or Clang; never Windows or MSVC**
  (2026-10-04). See "Platforms" below.

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

## 2026-10-12 — OpenCV, behind calib's types

calib's CPU maths -- the lens model, poses and the solve's first rig now,
detection as it lands -- is OpenCV 5's, called from the `compute` tier on.
Callers hold recon's `CameraModel` and `Mat4d` and calib's `Target`; `compute`
converts them (`cv_camera`, `cv_pose`, `Board`), so no second camera or board
record exists.

- **A prerequisite, and optional.** OpenCV is installed and found
  (`find_package(OpenCV 5.0 CONFIG)`), never fetched, like recon's SDKs.
  `VC_WITH_OPENCV` is OFF by default, so the core tier still builds anywhere.
  5.0 is the floor: calib calls its geometry module. CI builds the OpenCV
  tiers on macOS (Homebrew) only, until its Ubuntu legs have a cached OpenCV 5
  build ([README](README.md#prerequisites)).
- **No exception crosses.** A public function that calls OpenCV runs its body
  through `compute::guarded`, which maps a `cv::Exception`'s code to a
  `Status` domain (`guarded.hpp`) and anything else through the core's
  `exception_status`. calib therefore needs core 0.2.0, which brought it.

## 2026-10-12 — The camera model is recon's

calib's 2026-10-08 entry kept a camera model of its own, because recon's camera
tier has no projection or unprojection for a solver. calib's solver will take
its lens maths from OpenCV instead, so that reason is gone, and calib takes
recon's `camera::CameraModel` (image size, pinhole intrinsics, OpenCV's
eight-coefficient rational lens) and its rigid transforms.

- **Why.** The factory models arrive as recon's `SensorInfo::color` and
  `::depth`, so a calib model would be a second record of the same fact; and
  calib's five-coefficient model could not hold the Femto Mega's depth lens,
  whose k4–k6 are not zero.
- **What goes.** `PinholeBrownConrady`, `ImageSize` and their Newton
  projection and unprojection; Eigen, whose last use was
  `DetectedPoint::pixel`, now recon's `Vec2d`. The projection example maps the
  scenario's camera into `CameraModel` and projects through recon's
  `distort_rational`. The locked "rational lens model first, fisheye later"
  goes too: lens models are recon's camera tier's to choose, and calib plans no
  unprojection of its own.
- **How recon arrives.** Fetched camera-only (`VR_CAMERA_ONLY`): recon_camera
  over the core's base tier and GLM, so the brain still builds with no Vulkan
  SDK (the `no_vulkan.brain` test). calib installs recon beside itself, and its
  package config finds the core through recon's, as a second read of the
  core's package recurses
  ([volumetric_kit_core#31](https://github.com/taojin-6/volumetric_kit_core/issues/31)).
- **The board names its dictionary.** `Target` adds `dictionary`,
  `first_marker_id` and `legacy_pattern`, so a board is fully described without
  OpenCV; `check_target` validates one and `legacy_femto_board()` is the rig's
  5 x 3 ChArUco board.

## 2026-10-10 — calib builds with exceptions enabled

The family stopped building with `-fno-exceptions`: recon's CI builds with
exceptions (its DECISIONS, 2026-10-10) and keeps the handlers that turn a
backend's or an allocator's throw into a `Status`, and the core has no
`-fno-exceptions` leg. calib follows, and drops its own.

- **What stays.** Fallible calls return `core::Status` / `core::Result`, and
  no exception crosses calib's API. Code may catch what a standard-library or
  third-party call throws (`std::bad_alloc`, `std::system_error` from
  `std::thread`) and return it as a `Status`.
- **What goes.** CI's `-fno-exceptions` job, and the rule that calib's code
  compile without exceptions. A consumer builds calib with exceptions
  enabled.

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
first. (Amended 2026-10-12: the camera model is recon's; see the locked
decisions.)

## 2026-10-02 — Shared agent guidance

`AGENTS.md` now owns the shared working rules and task-based reading map.
`CLAUDE.md` imports it with `@AGENTS.md`. This decision record keeps the
previously committed constraints available to both agents without copying the
local-only design document. Update shared rules here and in AGENTS.md as
appropriate when a decision changes; keep tool-specific configuration separate.

## 2026-10-11 — Scenario configuration belongs to core

Applications use the shared versioned JSON format from core_config. Core owns
parsing, typed settings, defaults, and static validation, including unsupported
fields and cross-field constraints. Each application maps validated settings to
its runtime APIs; existing camera/model invariants and runtime resource checks
stay with their owners. The implemented projection demo uses this contract now.
Future calibration applications will extend it when their behavior exists.

The optional examples link core_config privately. calib_core still needs only
core_base and recon_camera, and public engine headers expose no JSON types.
