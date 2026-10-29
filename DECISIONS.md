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
  entry), so a calib error passes to `sensor` or `gfx` unchanged. The core is
  fetched pinned by commit (`third_party/`), and calib's configure refuses
  one older than calib builds with (`vkc_require_core`). Why:
  `recon`'s and `gfx`'s copies of the same bottom layer had drifted (the core's
  DECISIONS.md has the evidence); one copy means a fix lands once.
- **GPU work runs on the core's vulkan tier, on a `VkDevice` shared with
  `sensor` and `gfx`** (MoltenVK on Apple). calib carries no GPU backend of its
  own, so the reserved `VC_WITH_VULKAN` / `VC_WITH_CUDA` options are gone. Why:
  frames `sensor`'s decoders leave in Vulkan buffers can be read in place only
  on the same device -- a calib device of its own would copy every frame
  through the host -- and the family keeps one Vulkan core, not three. CUDA is
  added per kernel only where profiling shows a need, in the core; native Metal
  stays deferred. A CPU backend remains the correctness oracle. calib turns
  the core's vulkan tier on (`VKC_WITH_VULKAN`) only for the rig tiers
  (`VC_WITH_RIG`), so the brain needs no Vulkan SDK.
- **The camera model and capture drivers are `volumetric_kit_sensor`'s**
  (2026-10-08, amended 2026-10-12 and 2026-10-29; see the dated entries).
  calib's cameras are sensor's `camera::CameraModel`, from its camera tier,
  and the rig tiers take frames from its `IRgbdSensor` and `SensorArray`
  rather than drive a device themselves. Why: the core has no camera or sensor
  tier (core PR #14 proposed one and was closed), and the family's camera
  vocabulary is the `camera` tier, now sensor's (its DECISIONS.md,
  2026-10-06).
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

## 2026-10-29 — Built on sensor, not recon

recon's camera and sensor tiers are now
[volumetric_kit_sensor](https://github.com/taojin-6/volumetric_kit_sensor)
(recon #311 and #312), a repository of its own on the core alone: the camera
model and calibration file, the sensor interface, frame preparation, picture
readback, the sensor array, image files, the Orbbec driver and its decoders,
and the frame contract (`DeviceFrame`, `pinhole.glsl`). Everything calib took
from recon is there; none of what went to volumetric_kit_grid or stayed in
recon (fusion, texturing), so calib fetches core, sensor and gfx, and neither
recon nor grid.

- **The same code under sensor's names, with no aliases.** sensor 0.1.0's
  tiers are recon's at their move, and its CHANGELOG.md maps every old name
  to its new one: headers `volumetric_kit/sensor/…`, namespaces
  `volumetric_kit::sensor` and `volumetric_kit::sensor::camera`, targets
  `volumetric_kit::sensor_<tier>`, options `VS_*`, GLSL `vs_*`. Why: a re-pin
  is then a rename, and a forwarding name would be one more to retire.
- **sensor is declared after the core and before gfx, and checked with
  `vkc_require_sibling(volumetric_kit::sensor 0.1.0 TARGETS …)`**: camera-only
  (`VS_CAMERA_ONLY`) without `VC_WITH_RIG`, with the camera, frame,
  interface, prep, array and image I/O tiers with it. Why: the core comes
  first so every sibling shares it, and the check (core 0.27.0's) refuses, at
  configure time, an older sensor or an override without a tier calib links.
  The core floor is sensor's, 0.29.0 (the 2026-10-13 entry).
- **`calib_depth`'s kernels include sensor's `pinhole.glsl` through
  `INCLUDE_TARGETS volumetric_kit::sensor_frame`**, the core's export, in
  place of recon's `VR_SHADER_INCLUDE_DIR`; its body is unchanged.
- **The rig's device asks for sensor's requirements**, which are what recon's
  were: `scalarBlockLayout`, and the external-memory and Metal-object
  extensions where offered.
- **A session's `software` names sensor's commit** where it named recon's
  (`capture::Software::sensor`). Why: a session names the code that captured
  it; nothing reads a session back (the 2026-10-13 entry), so its format
  version stays 1.
- Earlier entries say recon for what is now sensor's; their decisions stand.

## 2026-10-27 — Each camera's depth offset

A depth that reads too far or too near is a correction, not only a
verdict: Calibrate's depth checks give each camera's offset along its depth
camera's axis (`quality/depth_checks.hpp` has the definitions).

- **Axial, as sensor applies it.** sensor subtracts a camera's offset from
  every return, along the depth camera's axis. A board's plane offset is
  measured along the board's normal, which an axial error of b moves by
  about `b (n . d) / d_z`, so each view's offset is made axial,
  `offset d_z / (n . d)`, with `d` the ray from the depth camera to the
  chessboard's centre and `n` the board's normal: a board seen square on
  the axis gives its offset, one seen at 60 degrees twice it.
- **The median over the views**, per camera, as the other verdicts take
  medians.
- **Judged after the offset.** The plane verdict no longer fails a camera
  for having an offset. It fails views that disagree once the offset is
  taken off: the median of `|a - offset|` over their axial offsets `a`,
  against `max_scatter_mm`, 5 mm, a pair's bound on its signed median,
  since what is left is depth error another camera would see. One view has
  no scatter, so its camera's plane is insufficient and no offset is written
  for it. The verdict also fails a median angle past `max_angle_deg`, and
  an offset sensor's calibration file cannot carry, which sensor's validator
  decides, so its bound is not copied here. The median offset along the
  normals is still reported.
- **Measured on raw depth.** The checks prepare the kept raw depth with no
  calibration, so an offset is the depth's own, never a correction of an
  earlier one.
- **Written when the plane passes.** `format_results` puts each camera's
  offset whose plane verdict passed into the results' calibration file
  (sensor's `depth_offset_m`, which makes it version 3), and none for any
  other camera: an offset belongs with the poses solved beside it, so a
  session's always replace an earlier file's, and a camera whose plane did
  not pass keeps its raw depth.
- **Oblique views weigh most.** A view's axial offset multiplies its plane
  fit's error by `d_z / (n . d)`, about 3 at 70 degrees. The median keeps
  one such view from moving the offset; weighing or leaving out oblique
  views waits until a session shows the need (the `TODO:` in
  `quality/depth_checks.cpp`).

## 2026-10-22 — Consumed by commit, never installed

The family's packages are not installable (the owner's decision): an
application fetches calib by commit, as the player and the iOS app fetch
their siblings, and none finds an installed calib. So calib has no install
rules, package config or `VC_INSTALL`, and neither have the core, recon and
gfx it pins. The brain's freedom from Vulkan is checked in a build of its
own, configured and built in tree
(`no_vulkan.brain`), not through an install. `calib_viz` and `calib_app`,
which serve calib_rig and the tests alone, are static libraries with no
export macros.

## 2026-10-19 — The world at the origin tag

Calibrate's results and its 3D view are in the origin tag's world when
`solve::origin_tag_frame` gives one, and in the reference camera's
otherwise (anchored last, as the "Calibrate (T4)" entry has it).

- **The file names the tag** (the user's call): a tag world's
  `array_calibration.json` is recon's `world.apriltag`, with the tag's
  printed edge, not the edge measured, so a reader can find the print
  again. A reference world's file is as it was, to the byte. recon before
  its #254 refuses a tag-world file.
- **The report names the origin.** `report.json`'s `world` says which
  origin the poses are in and, without a tag world, why, so the report
  alone tells where the calibration file's poses are.
- **At most 3 px RMS** (the user's call; `solve::kMaxTagWorldRmsPx` says
  why): a world that fits worse is refused, the reference camera staying
  the origin, and the report, the Calibration window and a notice say why.
- **The floor boards reported, not used** (the user's call). z is the
  tag's alone; the solved boards lying on its floor are listed with their
  tilt and height (`quality::floor_boards`), with no verdict, to decide on
  levelling after H5. On 2026-09-29 the tag's z was 1.1-1.4 degrees off
  them, about 20 mm at 1 m.

## 2026-10-18 — Calibrate (T4)

Calibrate runs on a worker of its own (`app/calibrate_worker.hpp` has the
steps, the numbering and the write order).

- **A snapshot, solved from scratch.** Capturing goes on while Calibrate
  runs, and what is captured meanwhile goes into the next press. A result
  depends on the captures kept alone: a dropped capture leaves no trace, and
  a recalibration owes nothing to the last.
- **The reference camera's gauge throughout.** The world at the floor tag is
  applied last, to the outputs only (`solve::anchor_rig`), so the metrics'
  re-solves, the leave-one-out moves, the covariance and the depth checks
  keep the gauge they were measured in.
- **Depth at the solved poses, in double.** Each depth camera is placed at
  its solved colour pose times recon's `depth_to_color`, composed in double;
  T4 prepares the raw depth with the colour camera at the origin, as T2
  does, and never reads the float pose recon bakes into a prepared frame.
  The raw depth's unit and range are the session's depth mode's.
- **On disk before it is shown.** The window never shows a result whose files
  are missing, and nothing is written until the solve, the metrics and the
  depth checks are done, so a refusal or a cancel leaves nothing behind. The
  directory's stamp is the press's, in the session's own format (the user's
  call).
- **Colour lenses, poses and depth offsets** in the results' calibration
  file (the user's call; the offsets amended 2026-10-27, "Each camera's
  depth offset"): `SensorArray` reads the poses and the offsets, and the
  session's factory file already has the depth models.
- **Cancelled between steps.** `bundle_adjust` is not interruptible, so a
  cancel takes effect at the next progress call (`app::CalibrateCallback`
  lists them); the writing, a few files, is not cancelled.
- **The images, then the cloud** (the user's calls). calib_rig
  shows the camera images until a result, then the live points at the solved
  poses in the whole window, and a key goes back to the images to capture
  more and recalibrate (`app/rig_mode.hpp`). Calibrate is C or F5 and Escape
  cancels; the keys the clicker's extra buttons send are not known, so the
  capture and drop keys stay as they were.

## 2026-10-18 — The origin tag

Calibrate puts the world at an AprilTag on the floor (CA12b), `kOriginTag`,
which `detect::detect_tag` finds in each camera's image.

- **The rig's print:** the one on the floor that the legacy
  `set_world_from_tag.py` anchored its world to.
- **Found at capture.** Nothing reads a session back (the 2026-10-13 entry),
  so the tag is searched for while the capture's full luma is at hand, in the
  luma the board's detection already reads: no readback.
- **OpenCV's detector, not libapriltag.** OpenCV is already every build's
  prerequisite and decodes AprilTag dictionaries; libapriltag would be a
  second detector to carry. The legacy script used OpenCV's too.
- **OpenCV's corner order, not AprilTag's.** The legacy script built its
  world on OpenCV's order, so a world built from the first corner to the
  second matches its world; it is turned 180 degrees about z from one built
  along the top edge of a print of AprilTag's own image. libapriltag's z,
  which implicit_surface_compression's per-camera tag pose used, points into
  the tag.
- **Triangulated, never one view's pose.** On the 2026-09-29 JPEGs each
  camera's own IPPE_SQUARE pose put the tag's centre 23-77 mm from the
  others' and its normal 0.4-1.6 degrees off. So the corners are
  triangulated, as the legacy script did (a DLT, then a least-squares
  refinement), from 2 or more cameras; with fewer the reference camera stays
  the origin (the user's call).
- **Its size measured, not assumed.** In the same study a rigid 0.15 m square
  fitted the corners worse than free ones (1.2-1.7 against 0.65-1.0 px RMS)
  and gave a frame only 0.05-0.07 mm and 0.07-0.19 degrees from theirs; the
  free corners' mean edge was 147.7-148.6 mm there.
- **Each camera above it.** The legacy script refused a world only when the
  cameras were below the tag on average. A camera that saw the printed face
  is above it, so any one below means a tag face down or corners mirrored.
- **Sub-pixel refinement (SUBPIX).** On the 2026-09-29 session's JPEGs, in
  the scratch study behind this choice, a still tag's corners varied by
  0.23-0.30 px (largest sd) across captures with it, against 0.47-0.50 px
  unrefined and 0.59-1.07 px with APRILTAG's, at the unrefined 43 ms per 4K
  image; APRILTAG's took 475 ms and tilted the tag's z by 0.75 degrees
  against the other two.
- **Its pull.** cornerSubPix pulls a black square's outer corner in along the
  corner's bisector: 0.1-0.3 px on synthetic tags 100-160 px across, at a
  camera's blur (`detect_tag_test`), while across it each corner holds to
  0.15 px. The tag's centre stays put, so the frame does not move, but its
  edges read short, by up to 0.6 % at those sizes.

## 2026-10-17 — The live 3D view, drawn from prepared frames in place

calib_rig shows the rig in 3D over its mosaic
(`viz/scene_view.hpp`, `viz/point_cloud.hpp`, `app/cloud_worker.hpp`): each
camera's depth as points, its frusta, the world's axes and the solved boards.

- **Hidden until the rig is calibrated** (amended 2026-10-18, the user's
  calls). Before Calibrate only the reference is placed, and the camera
  images, where the board is aimed and checked, matter more: the mosaic has
  the window and T2 prepares nothing. A result gives the view the whole
  window (the 2026-10-18 entry, "Calibrate (T4)").

- **No point cloud is built.** recon prepares each camera's frame
  (`GpuFramePrep`: undistorted depth, and colour), gfx draws a point per
  depth pixel straight from those buffers (`DepthCloudPipeline`), and calib
  composes the two with its poses. No pixel crosses the host; the lines are
  a few KB made on the host from the cameras' models, uploaded when they
  change.
- **A worker of its own (T2)** alone owns its preparations, one per camera,
  on recon's queue family. The live worker offers it each set it polls;
  it prepares the newest, every `live_every`-th, lets the set go at once,
  and publishes a snapshot, newest wins. Frozen, hidden or minimized, it
  prepares nothing and lets offers go, and the window keeps its snapshot.
  In the solid mode it prepares depth only, so a cloud frozen in it has no
  colour to show and the colour modes wait until the view is live again.
- **Poses at draw time.** A frame is prepared with its colour camera at the
  origin, its baked pose ignored, and `cloud_draw` places it at the rig's
  poses when it is drawn, so a frozen cloud follows a new solve with no
  second preparation. Before Calibrate only the reference camera, the
  world's origin, is placed: CalibRig takes no calibration file.
- **Read in place across families.** The prepared depth and colour are
  CONCURRENT across recon's and the renderer's families (recon's
  `depth_queue_families`, `color_queue_families`). `cloud_draw` refuses an
  EXCLUSIVE buffer while the families differ, which the validation layer
  would not report, and a style that leaves either family unset. The
  preparation waits for its fence, which makes no device write visible to
  another queue, so the worker then sets a timeline value on recon's queue,
  and each frame that draws the snapshot waits for it at the vertex stage,
  as gfx's contract asks.
- **Upright** (amended 2026-10-19). gfx's `OrbitCamera` keeps +Y up, so
  the view turns the world's up to +Y: the origin tag's +z, so its floor
  lies level, or, in the reference camera's world, whose +y is down, the
  placed cameras' mean up (-y). Their mean forward, levelled, goes to -Z;
  an outside-in ring's forwards cancel, so the first camera's is taken.
- **Wide points where the GPU has them.** calib_rig's device asks for
  `largePoints`, merged with recon's and calib_depth's needs
  (`rig_requirements`), and is made without it, with one-pixel points, on a
  GPU that lacks it.
- **Still to come:** the 3D view's settings in the scenario (the core's
  CO5) and the cameras' names over the view, which want a world-anchored
  label in gfx; each is a `TODO:` where it lands.

## 2026-10-17 — OpenCV 5 is required

calib has one dependency set on every platform: OpenCV 5 is a prerequisite of
every build, as GLM is, and `VC_WITH_OPENCV` is gone. `compute`, `detect`,
`solve` and `quality` always build. `VC_WITH_RIG` stays optional, for the
Vulkan SDK, and alone builds `capture`, `depth` and `viz`.

- **Why.** With the OpenCV tiers optional, the Ubuntu legs, whose packages are
  OpenCV 4, built the core tier alone: calib's real code was built and tested
  on macOS only, never with GCC and never under the sanitizers. Now every leg
  builds and tests every OpenCV tier, and the sanitizer job runs them.
- **How Linux gets it.** Installed and found, never fetched by calib's build,
  as before: `tools/install-opencv.sh` builds a release from OpenCV's source
  into a prefix, with only the modules calib and its tests use and its bundled
  JPEG and PNG. `latest` takes the newest 5.x tag, so Linux follows OpenCV's
  releases as Homebrew does on macOS. CI's `.github/actions/opencv` runs the
  script and caches the install, keyed on the release, the OS release and
  architecture, the compiler and the script, so a new release or a changed
  script builds it once and every other run restores it. OpenCV itself is not
  sanitized; calib's code is.
- **What went:** the option, the configure's own message for a missing
  OpenCV and its test (`opencv_requirement`), the OpenCV-free variants of the
  no-Vulkan package test and its consumer, and CI's `opencv` input.

## 2026-10-16 — CI on the family's layout: macOS on the Mac, the rest hosted

calib's CI takes its siblings' layout: `ci.yml` calls `_build.yml`,
`lint.yml` and `sanitizers.yml`, and its `required` job passes only when all
of them did.

- **The macOS legs run on the family's Mac** (label `mac`), as recon's and
  gfx's do, on runners registered by the core's
  [runner tooling](https://github.com/taojin-6/volumetric_kit_core/blob/main/tools/runners/README.md);
  **everything else on GitHub-hosted runners**, free for a public repository
  (amended 2026-10-17, when calib became public). Why the Mac: a hosted macOS
  job spends 2-3 minutes installing Homebrew's OpenCV, Vulkan SDK and FFmpeg,
  and tests on 3 cores; on the Mac they are installed and the tests run on 5.
  No calib job needs the Linux GPU hosts: none draws on an NVIDIA GPU. The
  hosted `required` gate always reports; while the Mac is offline its legs
  queue, and `runner: ''` moves a leg back to a hosted runner.
- **Two macOS legs:** the tests of calib's threads -- the session writer's
  and the app's workers' -- under ThreadSanitizer in RelWithDebInfo, and
  Release with the rig tiers, whose GPU tests run on the Mac's GPU under the
  validation layer (amended 2026-10-22). Ubuntu 24.04 and 26.04 build Release,
  26.04 with shared libraries; Linux Debug is the sanitizer job's.
  Every leg builds the OpenCV tiers (amended 2026-10-17): the Mac's Homebrew
  has OpenCV 5, and the Linux jobs restore their cached OpenCV 5.
- **The Mac's Homebrew is shared**, so a job there installs nothing and fails
  naming any formula missing; the owner installs it by hand while CI is idle.
- **A brew upgrade needs no cache clearing:** each macOS leg's dependency
  cache keys on the installed versions of the formulae its build finds, by the
  core's `homebrew-key` action, so the next run after an upgrade rebuilds the
  tree once. Why: the core's
  [DECISIONS.md](https://github.com/taojin-6/volumetric_kit_core/blob/main/DECISIONS.md#a-brew-upgrade-needs-no-cache-clearing).
- **Protecting `main`** with the `required` check is the owner's choice.

## 2026-10-16 — calib_rig's live view and capture

The `app` tier runs the rig (`app/calib_rig.hpp`, `live_worker.hpp`,
`capture_worker.hpp`); `apps/calib_rig/main.cpp` only loads the scenario.

- **Three threads, one owner each** (a fourth, the 3D view's, since
  2026-10-17, and a fifth, Calibrate's, since 2026-10-18; see the dated
  entries). The live worker (T1) alone touches the
  rig's `SensorArray`, which is not thread-safe: it polls each set, reads every
  camera's luma back in one submit, finds the board (hybrid) and checks each
  view, drives the capture rule, and publishes a snapshot, newest wins.
  While the 3D view fills the window and no capture is armed, it reads
  nothing back and searches nothing (amended 2026-10-22). The
  capture worker (T3) searches each capture's luma again at full resolution,
  the solver's input, and queues it on the session writer. The window (T0)
  turns keys and buttons into requests and never waits on a camera, a
  detector or the disk.
- **Frames go back quickly.** A capture is copied out of its set (luma, JPEG,
  depth) so the set returns to the driver at once; the window holds a
  snapshot's set in gfx's `RetireQueue` until the frame that copied its
  pictures into the mosaic retires.
- **One device, two families.** The cameras decode onto the compute side of
  the one `VkDevice` gfx shares (`gfx_glfw`'s `make_shared_device` with recon's
  requirements, merged with calib_depth's and `largePoints` since 2026-10-17;
  see the dated entry), their pictures readable from both families, so T1
  reads luma back on compute while T0 copies the same images on graphics.
- **Settings are the core's.** The scenario is `config::Application::CalibRig`
  (core 0.11.0): the sync file, the modes, the session root and the window.
  Detection and capture thresholds stay calib's defaults. The modes requested
  are the ones every camera is held to when the rig opens, for any number of
  cameras. The depth mode's name and range come from the Femto Mega's own
  table for its size, not from fusion settings the scenario does not use;
  calib keeps that table (`app::depth_mode`) until recon's Orbbec driver names
  its modes. The depth unit is the frames' own.
- **Nothing is written outside a session.** The session root must exist, and
  the sync file must read, before the window opens.
- **Keys before ImGui.** calib_rig's key callback is installed before ImGui's
  GLFW backend, which chains to it. It queues each key, and the frame maps
  them once ImGui has begun it, so keys are ignored while a field takes text,
  even one clicked into the frame before; a held key captures once.
- **How it builds.** calib_rig is an application's choice: `VC_BUILD_APPS`,
  on by default only at the top level, as the examples are. The tier is then
  built with `viz` when gfx's `gfx_glfw` (an installed GLFW) and recon's
  Orbbec driver (`VR_WITH_ORBBEC`) exist, and turns the core's config tier
  on. The driver's colour decoders take no FFmpeg on Apple, where recon
  refuses `VR_WITH_FFMPEG`, and `VR_WITH_FFMPEG` with `VR_WITH_CUDA`
  elsewhere (amended 2026-10-27). It is a static library, as `viz` is. A
  session records calib's and the siblings' commits as git gives them at each
  build, `-dirty` with uncommitted changes.

## 2026-10-16 — Depth checks are calib's kernels

The depth checks' per-pixel work -- each view's board plane, and each
ordered pair of cameras' depth agreement -- runs in calib's own `depth` tier
(`depth/depth_checks.hpp`), the device half of the entry below.

- **A tier of calib's own.** `depth` sits beside `viz` (`quality` →
  {`depth`, `viz`}), built with `VC_WITH_RIG`, and links
  `calib_quality` and `core_vulkan` alone: `DepthImage` is what it reads of
  recon's `DeviceFrame`, which the application maps, so it names no sensor
  type and nothing of gfx. Its device requirements are the core's compute
  baseline (std430 blocks, no float64, no subgroups), so it adds nothing to a
  rig's device.
- **In place.** The kernels bind recon's prepared depth buffers where they
  are, on recon's queue family; each region's or pair's block goes up, and
  48 bytes per region and 32 per pair come back.
- **One pinhole, one region rule.** The kernels include recon's exported
  `pinhole.glsl` (recon pinned at or after its #245), and `board_region.glsl`
  is `in_board_region` on the device, which a test kernel holds to the host's
  pixel by pixel.
- **A deterministic fit, not RANSAC**, seeded by the solved board: the points
  within `seed_mm` of it, then within `max(min_gate_mm, sigma_gate rms)` of
  that plane, each round a least-squares plane by a cyclic Jacobi solve, both
  in one submit. The moments are float, so they are taken in the board's
  frame about a point near the inliers (the rectangle's centre on the solved
  board, then round 0's centroid), and summed in a fixed order -- a
  shared-memory tree into each workgroup's slot, then a fixed walk of the
  slots, with no float atomics -- so the same depth gives the same plane to
  the bit.
- **Too little is a result, not an error.** Too few inliers, or inliers on
  one line, give a plane that is not valid, with its counts, for the verdicts
  to call insufficient. The options and regions are refused by the public
  checks `board_region` and `board_depth_check` run.
- **Agreement in integer histograms.** A pair's samples move from one
  camera's depth frame into the other's by a transform composed in double on
  the host, and meet the nearest pixel by recon's pinhole. The edge gate
  checks the pixel in both cameras, as flying pixels appear in either. The
  differences are counted in shared memory, at most 4096 `u32` bins: 16 KiB,
  the least Vulkan allows, so the requirements stay the baseline. They are
  then added with integer `atomicAdd`, exact in any order, so captures
  accumulate on the device, pooled or one capture's, to the bit. The window
  is an even number of bins, so 0 is a bin edge and the magnitudes are the
  histogram folded. A serial walk of each pair's bins gives its quantiles,
  interpolated within the bin.

## 2026-10-16 — Depth checks: the host's geometry over the device's results

calib measures the depth; recon only prepares it, and gfx draws it
(`quality/depth_checks.hpp` has the definitions).

- **Device and host.** Per-pixel work on depth runs on the device, in calib's
  kernels, and returns a few numbers per view and per pair. What derives from
  detection and the solve stays on the host with the solver: the board
  region, one ray per corner, the ray-plane meetings and the verdicts.
- **A region by rays**, not by where a depth point lies, so a pixel with no
  depth is classified as one with depth is, and out-of-range returns are
  counted among the members. The device follows `in_board_region`.
- **Two kinds of check.** The board's scale does not depend on the solved
  board, so it checks the depth against the printed board. The plane's offset
  and angle do depend on it, so they check `depth_to_color` and the depth's
  units; the offset is each camera's depth offset, judged once it is taken
  off (amended 2026-10-27, "Each camera's depth offset").
- **At Calibrate, on its worker (T4)** (amended 2026-10-19, the user's
  calls). T4 owns a depth-only preparation per camera and the checks
  (`app::CalibrateDepth`), built as calib_rig starts, and checks each kept
  capture's raw depth in turn after the metrics; T2 is unchanged, so no
  captured cloud is prepared and no JPEG decoded for the checks. A camera's
  verdict joins its scale's and its plane's (`quality::camera_verdict`).
- **Verdicts** rest on means and medians, pooled over every capture checked
  (a capture's own pairs on that capture's, so the panel names the capture
  where a pair fails), never on the p90, which resampling inflates. The
  bounds are first values, to be revisited from a real session (H5), and
  calib_rig takes `DepthCheckOptions{}` as they are, with no setting. The
  scale's 0.2 % catches a board printed 0.3 % large, as the 2026-09-29 board
  was. The panel shows each figure beside its verdict, since the plane's
  bound can fail a good camera (the `TODO:` in `depth/depth_checks.hpp`).
- **One `DepthCheckOptions`** goes to every step, the region's inset
  included, so the settings the report records are the ones used.
- **`report.json`** gains `depth`, which is `null` when the depth was not
  checked. Its version stays 1, since nothing reads the report yet.

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
- **Re-solves are warm** (amended 2026-10-22). Leave one out and the
  principal-point A/B test run `bundle_adjust` from the solution: its poses
  and the corners it kept, as one solve with no outlier rounds and no
  covariance or sensitivity. Outliers are not decided again per trial, which
  cost a second solve whenever the solve dropped any, so a move is its
  capture's alone and the A/B sides are paired by construction; on the
  synthetic rig with outliers both agree with full re-solves far below the
  digits shown. A re-solve `bundle_adjust` refuses
  (`InvalidArgument` or `Numerical`, e.g. a link short of `min_shared`) is
  recorded as refused, not an error, so it costs no other metric.
- **The A/B verdict is the RMS alone, by a margin** (amended 2026-10-18, the
  user's call). The sides' RMS is over the corners both keep, so their gap is
  paired and far steadier than the transfer, which stays as information. On
  the synthetic rig (`make_scene`'s 60 placements at 0.3 px of noise, seeds
  1 to 16 for plan and noise), the truth at either side gave a gap
  (`principal_point_gap`) of at least 2.7 % and the truth midway at most
  1.2 %: the margin's 2 % names the right side in all 32 draws and a side in
  none of the 16 midway, where requiring the transfer's median to agree too
  failed 5 of the 32. These are libc++'s draws: libstdc++'s
  `std::normal_distribution` draws other noise from the same seeds, where the
  tests pass too but the gaps were not measured. It is no guarantee: at
  0.1 px the truth midway reached 3.0 % (2 of 16 named a side), and with 20
  placements at 0.3 px a side's own gap fell to 1.7 %.
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
  checks, counters -- goes from the host to the screen. What Linux's
  decoders leave -- NVDEC's images still owned outside Vulkan, and nvJPEG's
  I420 planes in a buffer -- is refused until the tile takes over their
  ownership (and gfx maps I420); a readback that has taken NVDEC's images
  over leaves them to the tile (amended 2026-10-27).
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
- **How it builds.** gfx is fetched, pinned by commit, with `VC_WITH_RIG`.
  gfx's libraries must build against calib's core pin, the one every sibling
  compiles against, as gfx's own core pin yields to it. `viz` also links
  recon's sensor array tier, whose frame sets, counters and decoded pictures
  its panels and mosaic take, and its sensor utils tier, whose prepared
  frames the 3D view draws. `calib_viz` is a static library (2026-10-22).

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
  session. Calibrate takes the kept captures' detections, raw depth and
  origin tags from memory (`CaptureWorker::kept`), not from the files.
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
- **Core 0.29.0** is the oldest calib builds with (`VC_CORE_MIN_VERSION`),
  the floor its configure checks: the highest of the pinned siblings' floors,
  sensor's, which every build fetches, so one floor covers every build
  (amended 2026-10-27 and 2026-10-29).

## 2026-10-12 — OpenCV, behind calib's types

calib's CPU maths -- the lens model, poses, detection and the solve -- is
OpenCV 5's, called from the `compute` tier on. Callers hold recon's
`CameraModel` and `Mat4d` and calib's `Target`; `compute` converts them
(`cv_camera`, `cv_pose`, `Board`), so no second camera or board record exists.

- **A prerequisite** (required since 2026-10-17; see the dated entry). OpenCV
  is installed and found (`find_package(OpenCV 5.0 CONFIG)`), never fetched,
  like recon's SDKs. 5.0 is the floor: calib calls its geometry module.
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
  SDK (the `no_vulkan.brain` test).
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
