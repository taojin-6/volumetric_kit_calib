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
  Target-based BA is small (a known board fixes structure → hundreds of
  params → OpenCV's `cv::LevMarq`, which ships for iOS and Android, on an ARM
  CPU in tens of ms; see the 2026-10-14 entry), so the solve runs on CPU
  everywhere; the GPU (Vulkan) is for dense per-pixel work.
- **The physically-parameterized sensor/lens model is the spine** (amended
  2026-10-14; see the dated entry) — `SensorProfile`/`LensProfile` are calib's
  physical description of a camera, optional inputs until intrinsics work
  gives them a use; the extrinsics checks and metrics are geometric.
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
  re-finds it at the version calib was built with, requires calib's floor of
  it (`vkc_require_core`), and refuses a recon that brings another. Why:
  `recon`'s and `gfx`'s copies of the same bottom layer had drifted (the core's
  DECISIONS.md has the evidence); one copy means a fix lands once.
- **GPU work runs on the core's vulkan tier, on a `VkDevice` shared with
  `recon` and `gfx`** (MoltenVK on Apple). calib carries no GPU backend of its
  own, so the reserved `VC_WITH_VULKAN` / `VC_WITH_CUDA` options are gone. Why:
  frames `recon`'s decoders leave in Vulkan buffers can be read in place only
  on the same device -- a calib device of its own would copy every frame
  through the host -- and the family keeps one Vulkan core, not three. CUDA is
  added per kernel only where profiling shows a need, in the core; native Metal
  stays deferred. A CPU backend remains the correctness oracle. calib turns
  the core's vulkan tier on (`VKC_WITH_VULKAN`) only for the rig tiers
  (`VC_WITH_RIG`), so the brain needs no Vulkan SDK.
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

## 2026-10-16 — The solved rig's metrics and its results files

The `quality` tier measures a solved rig and formats its results directory's
files (`quality/calibration_metrics.hpp` and `results.hpp` have the
definitions and the schemas).

- **Formatted, not written.** The brain does no file I/O: the application
  creates each results directory and writes `array_calibration.json` (recon's
  own formatter, posed by each camera's T_cw), `report.json` and
  `residuals.csv` with the core's `write_file(CreateNew)`.
- **Errors per corner, unweighted, over the corners kept.** The first rig's
  and the solution's errors are over the same corners, so the legacy RMSE
  (the RMS over the root of 2, the legacy solve's figure) and its reduction
  compare like with like.
- **Transfer is the legacy check.** `verify_rig.py`'s definition, independent
  of the adjustment: each view's own pose carried into every other camera
  that poses the capture. A view counts when it fixes a pose, not when it has
  4 corners.
- **Re-solves are warm.** Leave one out and the principal-point A/B test run
  `bundle_adjust` from the solution. A re-solve `bundle_adjust` refuses
  (`InvalidArgument` or `Numerical`, e.g. a link short of `min_shared`) is
  recorded as refused, not an error, so it costs no other metric. The A/B
  sides' RMS is over the corners both keep, and a verdict needs both the RMS
  and the transfer's median to agree.
- **Everything left out is listed:** the views `initialise` excluded, with
  what it measured, beside the adjustment's rejections, so each can be
  recaptured.
- **calib's JSON files are written by hand**, as recon writes its
  calibration file, with the one private writer,
  `src/volumetric_kit/calib/core/json_text.hpp`:
  nlohmann/json makes each number (the shortest form that reads back, `null`
  when not finite) and string, but none of its containers, whose destructors
  allocate and so terminate when memory runs out. The capture tier's session
  files are still to be moved onto it (a `TODO:` in `session_files.cpp`).

## 2026-10-16 — The live view on gfx

The `viz` tier draws the live view (`viz/mosaic.hpp`, `overlay.hpp`,
`panels.hpp` and `board_editor.hpp`).

- **Pictures stay on the device.** A tile takes recon's decoded NV12 images
  as they are; gfx's image pipeline copies and converts them on the GPU, so
  no picture reaches the host to be shown. Only control data -- corners,
  checks, counters -- goes from the host to the screen. Planes in a buffer
  and I420, as Linux's decoders leave them, are refused until the tile takes
  over their queue family's ownership (and gfx maps I420).
- **One mosaic, any number of cameras.** Each camera's picture is a tile of
  one NV12 picture: gfx copies each new picture into its tile, the others
  keep theirs, and each page with a new picture is converted once per set,
  so the rig takes one conversion, one mip chain and one draw per page,
  whatever its camera count, and one view pans and zooms it all. calib picks
  the tiles: cells of the largest picture's size in a grid as square as it
  can be, at least 16 texels apart, with origins and extent on multiples of
  16, so the mosaic's mips to level 4 keep the tiles apart (gfx's
  `ImageWrite` gives the rule). A mosaic too large for the device's textures
  or a page's texel budget is paged over several textures in blocks of whole
  cells, each drawn at its place in the one view over a black backdrop, so
  the tiles, their marks and the black gutters are where one page would put
  them. A page's pictures share one coding, as gfx converts a page whole.
- **One camera's fault leaves the others live.** A picture the mosaic cannot
  take, or a page whose pictures mix codings or whose write gfx refuses, is
  left out and named while the rest are recorded, so a rig of many cameras
  does not freeze for one. A coding change forgets the page's old pictures
  only after gfx accepts every write. Since gfx validates inside
  `record_write`, that first write is followed by a discard and a second
  write on a coding change alone, clearing uncovered tiles before converting
  the page. A refusal keeps the previous pictures and advances their ages
  as a set with no new picture does.
- **Marks in the detector's pixels.** Corners keep OpenCV's integer pixel
  centres, and gfx's `ImageOverlay` is the one home of the half-pixel and
  framebuffer-scale mapping onto a tile's region of the mosaic, of the clip
  to it, and of the title's place in its visible corner.
- **Models, then ImGui.** The marks and the panels are plain models built from
  host data, so they are tested without a device; drawing them is a thin pass
  through gfx. Their tones follow the quality tier's options, so the panels
  judge as the checks and the capture rule do.
- **The board locks at the first capture.** Every capture's detections, and
  the solve, rest on it.
- **How it builds.** gfx is fetched, pinned by commit, when `VC_WITH_RIG` and
  `VC_WITH_OPENCV` are both on, since `viz` draws the quality tier's checks.
  gfx's libraries must build against calib's core pin, the one every sibling
  compiles against, as gfx's own core pin yields to it. `viz` also links
  recon's sensor array tier, whose frame sets, counters and decoded pictures
  its panels and mosaic take. `calib_viz` is build-tree only, as gfx's ui
  tier is, until gfx installs the ImGui headers.

## 2026-10-14 — The bundle adjustment on `cv::LevMarq`

The solve refines the first rig with OpenCV's `cv::LevMarq`, not Ceres
(`solve/bundle_adjust.hpp` has the method).

- **Why not Ceres.** The problem is small: 6 parameters per camera and per
  capture, about 380 at 60 captures and at most about 900 at the 150-capture
  cap, so a dense normal-equations solve (Cholesky) suffices. A 60-capture
  solve takes about 25 ms on the synthetic rig (Release, Apple M5 Max). Ceres
  would be a second solver prerequisite, beside OpenCV, on every platform.
- **No matrix-to-vector Rodrigues inside it.** The derivatives chain
  `cv::projectPoints`' with `cv::Rodrigues`' vector-to-matrix one, never
  `composeRT` or a matrix-to-vector conversion, whose derivative is singular
  near a half turn; the 2026-09-29 session has boards at 167° and 174°.
- **OpenCV's matrices where recon has no type.** The solve's results hold a
  `cv::Matx` for a shape recon lacks (a camera's 6 x 6 covariance, a corner's
  2 x 6 derivatives), amending the 2026-10-12 entry's "behind calib's
  types"; cameras and poses stay recon's.
- **Ambiguous views are adjusted.** A capture whose views are all ambiguous
  takes its board from them in `initialise`, so the adjustment can use them
  rather than drop the capture.

## 2026-10-14 — Live checks and the capture rule

The `quality` tier's first part says, set by set, what each camera's view is
worth and which set a capture takes, and sums up the captures kept
(`quality/live_checks.hpp`, `capture_rule.hpp` and `session_checks.hpp` hold
the thresholds).

- **Why the profiles became optional.** Every check reads the detection, the
  view's own pose (IPPE), coverage and shared captures, so the locked "spine"
  entry is amended rather than given a use invented for v1; intrinsics work
  is what will need them (noise-weighted corners, blur and rolling-shutter
  gates).
- **Stillness gates; spread and views only warn.** A press arms the rule,
  which takes the first set with every camera's frame whose board is still
  since the set before, or fails after a timeout naming what held the last
  set. The whole set is saved however many cameras see the board. The
  frames' spread mostly reflects the SDK's clock skew under the hardware
  trigger, so it is recorded and warned of, never a gate; a set with too few
  usable views adds nothing to the extrinsics and is warned of too. The set
  the rule would take is refused at once when the writer is full.
- **Live corners are flagged, not refused.** The live view's hybrid corners
  are for display; the solver takes the capture's full-resolution ones. A
  capture whose two disagree is flagged.
- **Tilted views are favoured.** The rig's 5 x 3 board fixes its own rotation
  far worse facing the camera: at 0.3 px of corner noise the worst axis's
  1-sigma is 0.86 degrees face-on at 2 m (2.64 at 3.5 m), against 0.22 at a
  20-degree tilt and 0.11 at 45. A view tilted under 20 degrees is frontal,
  and a camera whose views are mostly frontal is warned, beside the duplicate
  and floor-only warnings. Coverage counts every usable view.
- **The session counts what the solve uses.** Shared captures, the tree
  preview and when Calibrate may run count only the views `initialise` places
  cameras by (`solve::view_fate`), so Calibrate is never offered on links the
  solve then lacks. An ambiguous view, mostly a board facing the camera,
  still counts toward coverage.

## 2026-10-13 — Sessions on disk

The `capture` tier writes the rig application's calibration sessions
(`capture/session_writer.hpp` has the layout and both schemas).

- **What is kept.** Each camera's JPEG byte for byte, as the camera coded it
  (recon's `RgbdFrame::color_bitstream`); raw 16-bit depth, through recon's
  `encode_png_u16`; the factory models at the streamed mode, in recon's
  calibration file, unposed; and the full-resolution detections, the solver's
  input, in numbers that read back exactly.
- **Never overwritten.** The session's directory is created at the first
  capture, refusing one already there, and every file in it once, through the
  core's `write_file(CreateNew)`, except `session.json`: it lists the captures
  and is replaced atomically after each capture and each drop. A drop marks a
  capture and keeps its files; no index is used twice.
- **Written, never read.** calib reads no session back: no resume and no
  replay. A crash loses no capture already listed; a new run starts a new
  session.
- **Off the caller's thread.** One writer thread behind a bounded queue, which
  refuses a capture when full; the caller polls the outcomes. The writer
  encodes and writes; the full-resolution detection is the caller's, done on
  a worker of the application's before it submits, since `capture` cannot
  link `detect`.
- **Records in `core`.** The detector fills `Detection` and the writer stores
  it, so it is a plain `core` record, as `ViewCheck` is.
- **How it builds.** `VC_WITH_RIG`, OFF by default, fetches all of recon and
  turns on the core's vulkan tier, so it needs the Vulkan SDK. gfx comes with
  the `viz` tier, its first user.
- **Core 0.6.1.** Its package can be read twice in one configure
  ([volumetric_kit_core#31](https://github.com/taojin-6/volumetric_kit_core/issues/31)),
  so calib's package config finds the core directly, and an application may
  find it too, but not another. calib requires it, since its installed
  package reads the core again through recon's.

## 2026-10-12 — OpenCV, behind calib's types

calib's CPU maths -- the lens model, poses, detection and the solve -- is
OpenCV 5's, called from the `compute` tier on. Callers hold recon's
`CameraModel` and `Mat4d` and calib's `Target`; `compute` converts them
(`cv_camera`, `cv_pose`, `Board`), so no second camera or board record exists.

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
- **Detection.** OpenCV's `CharucoDetector`, default parameters, no camera
  model. The live view finds the markers in the image halved and refines each
  corner at full resolution, in under a third of the time (2.9 against
  10.1 ms for a 4K image on an M5 Max), but less surely: in the scratch
  OpenCV study behind these settings, on rendered views, its corners erred by
  up to 0.54-0.72 px, against 0.24 px at full resolution. Its detections are
  for display and live checks only; a capture's, which the solver takes, are
  found at full resolution.

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
  SDK (the `no_vulkan.brain` test). calib installs recon beside itself.
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
