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

## 2026-11-01 — The stable API

The brain is calib's API for programs built on it: the headers in
README.md's [table](README.md#stable-api) (board, detect, views, solve,
quality, and sensor's calibration file, which calib formats and the caller
writes). From 0.1.0:

1. The API is the headers in that table, calib's own types they use, wherever
   declared (`compute::CvCamera`, `compute::ViewUse`, `quality::FloorBoard`,
   ...), and the OpenCV 5 and sensor camera types they use (`cv::Point3d`,
   `cv::aruco::CharucoBoard`, `CameraModel`, `ArrayCalibration`, ...): a break
   there is a calib break.
2. Additive changes are free: new headers, functions or overloads; trailing
   parameters and struct fields with defaults; trailing enum values.
3. Anything else is breaking: a `!` commit, a CHANGELOG "Breaking" line with
   its one-line migration, and a line here saying why.
4. Every feature PR bumps the minor version, in merge order, as sensor and
   the core do.
5. `tests/api_test.cpp` builds and runs the sample programs on the table's
   headers alone, on synthetic data. A break in a call they make fails the
   build; a break elsewhere in the table is caught in review against it.
6. calib stays consumed by commit, never installed (the 2026-10-22 entry):
   no ABI promise. Output files keep their own versions (sensor's file,
   `report.json`, `session.json`).
7. The house rules stay: `core::Status` and `Result`, no exception across the
   API, a `@code` example per class.

Every other header is internal and, those types aside, changes freely: `app/`,
`viz/`, `capture/`, `depth/`, and the rest of `compute/` and `quality/`.
`source/rig_source.hpp` is public but not promised
(the 2026-11-02 entry on the source). Why: other programs (an offline lens
solve, the second camera pair) assemble the brain, and a rule that shows in
review costs little. Freezing everything would stall the app; an ABI promise
means nothing for a library nobody installs.

## 2026-11-03 — Fast tests in CI

CI's tests prove each rule on the smallest case that shows it; full-size
work and statistical sweeps are for hand runs.

- **A test binary should take about 5 s in Release and 30 s under a
  sanitizer, both on an M5 Max**, with the fewest pictures, captures and
  fits that still prove each of its rules, and no fixed sleeps. A slow
  binary is cut by sharing what its cases make (renders, searches, fits) and
  by holding each rule on the inputs that show it, found by probing:
  detect_board keeps the searches that fail with its corner filter off and
  the half that differs with OpenCV's random numbers unseeded. A case that
  proved one rule on several seeds, rigs or poses keeps those it needs, and
  the rest go; no rule goes. detect_board's cases keep every scene, seed and
  state in `detect_board.sweeps`, labelled `slow`.
- **No CI leg runs the `slow` label**; `ctest -L slow` runs the sweeps and
  `detect_board.timing` by hand. The `package` tests, calib's nested
  configures, run on the Release rig leg.
- **With CI's Linux OpenCV, built without LAPACK or Eigen
  (`tools/install-opencv.sh`), a lens fit costs its dense solve over every
  view's pose**, whatever its iterations: a one-iteration fit of a 38-view
  session takes about 650 ms under the sanitizers. So the lens tests share
  each seed's fit across cases and run the rules on a fit's mechanics
  (starts, passes, outlier rounds) on half a session, every other view;
  rules about the session itself (reach, wrong minima, the curve) keep all
  of it.
- **The deps caches hash the pins (`third_party/CMakeLists.txt`), not the
  root CMakeLists.txt**, whose version every PR bumps: so main's sanitizer
  job missed on nearly every PR, building for 280-470 s against 160 s on a
  hit (its 12 runs to 41711671795). Each restores by prefix, and the prefix
  carries what make does not track in a restored tree: the workspace path,
  which FetchContent bakes in, the Homebrew key on macOS and, in the
  sanitizer job, the compiler, the OS release and the versions of the header
  and shader-tool packages (a package keeps its files' old mtimes). After a
  restore from other pins or flags, FetchContent checks the moved dependency
  out again and make rebuilds only what changed. An exact hit saves nothing,
  so what moves the dependencies' objects outside the hashed files (the root
  CMakeLists.txt's flags, a runner image's libraries) is rebuilt on each run
  until the pins next change.
- **The sanitizer job compiles through ccache**, each run saving its cache
  and restoring the newest whose key starts with the deps key's parts (the
  workspace, compiler, OS and packages); ccache hashes each object's flags
  and headers itself. With the deps cached, the job's
  build is calib's own objects under the sanitizers: 219-297 s (run
  41734682596, both attempts).

## 2026-11-03 — A tracked camera's world at the origin tag

`solve::tracked_world_from_tag` places a tracked camera's world in the
tag's from the camera's stills of the tag, posed as its session records
them (sensor's `camera/tracked_stills.hpp`); `register_tracked` makes
sensor's registration file of it (`solve/tracked_world.hpp` has the method).

- **Gravity gives the tilt; the fit solves yaw and position** (the
  owner's call). A hold's rays are nearly parallel: triangulating its
  corners, as `origin_tag_frame` does from the rig, fixes no size, and in
  the planning simulation no hold landed within the tracker's basin (1° of
  yaw, 5 cm). With the tilt given, 94-99 % did, and one still holds yaw and
  position, face-on included.
- **The edge is the rig's measured one, an argument** (`TagWorld::edge_m`):
  the printed size puts a camera 1.9 % of its range off, about 40 mm at
  2.2 m.
- **The seed in closed form, not IPPE.** With the tilt given the tag's
  plane is level, and one still's yaw, height and shift are a similarity
  of where its corners' rays meet it. IPPE's square poses of a marker
  face-on are a half turn, where OpenCV 5.0.0's IPPE loses the angle or
  gives no pose (`view_pose.cpp`).
- **A poor fit is above 12 px, not the rig's 3** (`kMaxTrackedRmsPx` says
  why): each still carries the tracker's own pose error, which at 0.1° and
  2 mm a still fits a hold at up to about 9 px RMS from 1.2 m, where one
  still 10 cm off fits at 48 px or more.
- **And a poor fit moves a camera over 3 cm from the stills' median**
  (`kMaxTrackedSeedGapM`): from 2.2 m a still whose pose is a few cm off
  pulls every camera 6–19 cm along its line of sight at under 12 px, since
  range shows only in the tag's apparent size. On the bars' noise with
  one still 3 cm off, the RMS gate alone let 28–41 % of holds from 2.2 m
  out of the basin; with this gate 0–1 %, or 3 % with the tag 0.65° off
  level, an error neither gate sees. Clean holds lose up to 1 % from
  2.2 m, 3 % from 3 m.
- **Misses are `TagMiss` values,** as the rig's tag world's are, with no
  `TagWorld`: the fit locates no corners.

## 2026-11-03 — One calib_rig, the mode chosen in it

calib_rig is one app: its Setup panel picks the mode, Lenses or Poses, and
the lens file, and the scenario's `operation` and `source.calibration` are
only where it starts (the owner's: "if we already have intrinsics, we just
need to load it and do extrinsics").

- **A switch is a new session, the cameras left open** (`switch_session`).
  After the first capture a switch of mode or lens file closes the session,
  its captures on disk, and the next capture opens a new one; before it,
  nothing is written. The live worker restarts first (`LiveWorker::restart`:
  its lenses, the board unlocked, the mode's capture rule): from that call
  it hands no capture on, under the lock it submits by, until it takes them,
  so none of the closing session reaches the next; the capture worker's
  restart, queued after it and before the window's next press, carries out
  what came before it first. The next setup is checked first
  (`CaptureWorker::accepts`), so a refusal changes nothing. A session
  started within the second of the last takes `_2`, and each session's
  results are numbered from 0. Refused while Calibrate runs, which reads its
  own session. Each mode keeps the board it last had. In the Setup panel, a
  switch that would close a session with captures asks first: they stay on
  disk, but Calibrate can no longer take them.
- **A missing lens no longer stops a pose run.** The camera takes the
  stand-in for the live view (`missing` in Setup), and Calibrate waits until
  every camera has a lens the runtime reads: the file's, calibrated, or the
  driver's factory one (a Femto's), naming each camera without and the fix.
  A capture is refused meanwhile, naming the camera: a lens file loaded
  later starts a new session, so its captures could never be calibrated.
  Why: the lens file is now loaded in the app, after it opens.
- **The lens file.** The list holds every lens run under `output.path`, a
  results directory with `array_calibration.json` and a report with a
  `lenses` section, newest first by its stamp, with each written camera's
  pass or fail; or a typed path, with no native dialog (no new dependency).
  A file is refused when a rig camera's lens there is at another size than
  its pictures, or no rig camera has one, saying which; a camera it lacks is
  noted. A lens run, picked in the list or by "Use these lenses", gives the
  next session its file without the lenses that failed in it
  (`lens_run_file`, the verdicts its `report.json`'s): each camera its lens
  that passed, or the one the run carried from its session's file when it
  did not fit the camera; a camera whose lens failed takes none, and is
  named. A failed lens is still written, for trials, and a typed path loads
  a file whole. Why: the list took a failed lens as calibrated, and the
  button dropped a carried one (the L3 review).
- **Calibrate is never silent.** Its button stays pressable; pressed while
  held back, the Calibration window opens and says why per camera
  (`calibrate_blockers`): a lens session's shortfall names each direction
  short with its reach ("reach 0.83 toward the top-left corner, needs 0.9:
  capture there"), a pose session's each lens missing and each link short.
  While it runs, the window shows it running; a run that ends with no
  result says there why (its failure, or a cancel), the last result cleared
  as it starts. Why: on 2026-11-02 the owner pressed Calibrate with neither
  camera at the 0.9 reach, and only a message in another window said so.
- **Each lens checked on the pose solve** (`check_rig_lens`): in the Lenses
  panel and `report.json`'s `lens_checks`. A warning: the pair rows stay the
  gate for depth; a check refused for its inputs reads insufficient, `not
  checked: ...`. Suspect reads "the lenses disagree": a lens session says
  which is off.
- **Calibrate has the CPU and OpenCV's loops.** The live view's search
  pauses while it runs (`LiveWorker::pause_search`; the pictures keep
  coming, their luma still made), and a press pauses it first, waiting out
  the set being searched. A lens run then fits the ready cameras at once in
  one OpenCV loop. OpenCV runs a loop started while another runs, in any
  thread, in turn on its caller's thread; so with no other loop running as
  the fits start, each fit's own loops run on its thread, and the result is
  fitting them in turn on one thread's, bit for bit, whatever the cores. A
  capture armed meanwhile keeps its search, and the fits may then run in
  turn. Without the wait, the set in flight usually held OpenCV's loop as
  the fits started: in 17 of 20 random presses they ran in turn, in 181 to
  218 ms, their last bits depending on the timing (the L3 review). On the
  M5 Max (18 threads, two cameras' 40 views at 4K, Release), a press with
  calib_rig's live search running waited 0 to 84 ms (median 37) for its
  set, then fitted in 122 to 129 ms, the one-thread files in 20 of 20
  presses (210 ms on one thread). OpenCV's threads stay uncapped (the TODO
  from 2026-10-16): the fits took 162 to 178 ms in turn and 117 to 119 ms
  at once at a cap of 2, 4, 8 or none; the live search took 87 ms a set at
  any cap but one thread, which made it 165 ms and the fits 200 ms. A cap
  gains nothing.

## 2026-11-02 — A lens checked on views of any board

`quality/lens_check.hpp` says whether a lens holds on one camera's residuals,
their RMS and share dropped held to the lens gates' limits and each cell's
mean to Student's t: `check_lens` poses each of its views of either board
through the lens held fixed; `check_lens_residuals` takes any solve's
residuals, and `check_rig_lens` a rig solve's. A lens session's views go to
`check_lens`; a pose session's solve to `check_rig_lens`, whose cameras share
each capture's board pose, so its residuals keep what a view's own pose takes
up. The numbers are the lab lens's, synthetic, 0.15 px of corner noise, and
for the rig the lab-like pair's (amended 2026-11-03). `lens_check.sweeps`
gives the noise-alone numbers at the defaults; the rest come from a study
outside the suite (random sessions as the sweep draws them, 50 to 200 each).

- **Posed as a held-out view.** `solve::pose_lens_views`
  (`solve/lens_poses.hpp`, outside the stable table) poses each view through
  the functions `solve_lens` poses a held-out one by; its corner bound is the
  adjustment's rule over the views' residuals at their first poses, as
  `solve_lens`'s is over the fitted views'. The fit itself is untouched.
- **The lens gates' limits, by reference** (`LensCheckOptions`, from
  `LensGateOptions`'s defaults): the RMS at most 0.5 px at 3840 wide, the
  held-out views'; at most 1% of corners dropped (past `check_lens`'s bound,
  in a view left with no pose, or by the solve), named by the parts of the
  image they lie in. The cells' maths and the width's scaling are one
  function each, `judge_lens`'s too; its results are unchanged.
- **Each 8 x 6 cell of 6 corners within 0.1 px, or t standard errors:** t is
  Student's for the cell's corners at the two-sided 0.27% of a normal's 3,
  the pair row check's, one function. The mean's error is taken as a vector,
  so noise alone fails a cell of independent corners about 1 time in 17,600
  at 6 corners and 9,400 at 30; 3 standard errors fail it 1 in 170 at 6 and
  1 in 2,500 at 30, the bias gate's design, which `judge_lens` keeps at its
  30. On a rig solve the shared poses take up part of each view's noise: its
  cells' means pass 2 standard errors less than half as often as
  independent corners' do. Noise alone, 0.1 to 0.3 px, failed no cell at any
  minimum tried: 3 to 30 on gridded sessions (`test::view_plan`, 30 captures
  over one camera's view, and 15 and 20 of them; 200 seeds each) and 4 to 10
  on random ones (boards anywhere in the pair's shared view, 1.4 to 3 m away,
  tilted up to 30 degrees; 15 to 40 captures, 100 to 200 sessions each); at
  6, none of some 135,000 cells. So the limit sets the minimum: 6 is the
  fewest whose limit, t over the root of the corners times the solve's
  0.19 px, stays within the RMS limit (0.43 px; 5 gives 0.56, 4 gives 0.88).
  Its cost is coverage: the true pair over random sessions reads good 0.75,
  4, 31 and 67.5% of the time at 15, 20, 30 and 40 captures
  (`lens_check.sweeps`, 400 checks each; at 8 corners, by the study, about
  0, 0, 6 and 24%),
  else insufficient, naming where to capture. `check_lens` takes the same
  rule: its 300 noise checks stay good.
- **Views of 6 corners or more:** a pose takes 6 of a view's numbers, so 6
  corners leave 6 to show the lens, and a 5 x 3 board's view keeps its pose
  with 2 corners lost. A corner 8 px off costs such a view that corner; one
  10 px or more pulls the pose so far that its neighbours pass the bound, and
  the view goes: 4 of the 377 views below would read suspect.
- **The verdict:** `Suspect` for any limit passed, naming each, and each cell
  by the part of the image it lies in; else `Insufficient` when a part of the
  image's 3 x 3 split (the cells by their centres: 3, 2 and 3 columns, 2
  rows each) has no judged cell, naming those parts and their fullest cell's
  corners, or the corners past the dropped share lie in one view (below);
  else `Good`. Suspect comes first, as in the pair check: no capture
  elsewhere mends it. `LensCheck::Verdict` is nested as `PairRowCheck`'s is,
  and `check_lens` takes a camera id and `Observation`s, as `solve_lens`
  does: a `LensView` holds no corners. Every entry returns a `LensCheck`; a
  solve's has no bound (`bound_px` not a number).
- **What a view's pose hides:** any lens error close to affine over the board.
  On the lab checkerboard, 110 views over the image: k1 1% high reads suspect
  by its cells, 3% by the RMS too, and a step of 3 px over 100 px three
  quarters across reads suspect in the right third alone at 0.37 px RMS; f 1%
  high and k3 raised to 10 px at 70 degrees read good, cx 30 px off suspect
  by one corner cell of few corners. On the rig's 5 x 3 ChArUco board, 377
  views: only a lens off by pixels reads suspect (k3 raised to 5 px at 56
  degrees, 23 at 65, by its RMS and its corners past the bound); k1 3% high
  and the step of 6 px read good. Noise alone (0.1, 0.2 and 0.3 px, both
  boards, 50 seeds: `lens_check.sweeps`) read suspect in none of 300 checks,
  the worst cell at 0.61 of its limit.
- **The rig hides far less.** 60 captures over the pair's view (main 5): k1
  1% high on bm-1 moves the solve's RMS from 0.19 to 0.66 px and reads
  suspect by the RMS and the cells, where `check_lens` on the same views
  moves from 0.17 to 0.18 px and reads good; the step of 3 px reads suspect,
  its worst cell in the right third, where `check_lens` reads good. Random
  sessions of 30 captures read bm-1 suspect for k1 0.3% high 92% of the time,
  1% high, the x step of 3 px, a y step of 3 px and k3 raised by 1e-3 98 to
  100%, a y step of 1 px 42%. On a gridded session of 30 captures f 0.3%
  high and cx 5 px off read good: the solve's poses take them up. Noise alone
  on main 5's session (`lens_check.sweeps`, 300 checks) reads good, the worst
  cell at 0.78.
- **A lens off shows in every camera sharing its boards,** and not always
  most in its own. With bm-1's lens off and bm-2's true, each camera's own
  residuals read bm-2 alone suspect in 66 of the 74 gridded sessions of 20
  captures where either did (f 0.3% high), and bm-1 good beside a suspect
  bm-2 in 24 of 200 of 30 captures (k1 0.3% high). So `check_rig_lens` reads
  a camera suspect too when it shares boards with one whose own residuals
  read suspect, naming it: the wrong lens never reads good beside it. Two
  cameras share a board when both kept corners of its capture. The check
  says the lenses disagree, not which is off; `check_lens` on a lens session
  says which.
- **Boards one camera kept alone are left out** (`LensCheck::unshared`
  counts their corners): captures whose kept corners are all that
  camera's. Such a board's pose is that camera's own, which takes up what
  the check looks for, and the capture rule warns of such sets without
  refusing them: bm-1's 30 captures over its view seen by it alone, with 10
  seen by both, read k1 1% high good on all 40 (RMS 0.32 px); the same 30
  seen by both read suspect (0.65 px). A capture no camera kept still counts
  its drops: leaving it out as well would cost detection (k3 raised by
  3e-4: bm-1's own residuals suspect in 137 rather than 164 of 192 random
  sessions of 30 captures, good in 9 rather than 4) for fewer false alarms
  at 0.3 px (2, 2, 0 and 1% of the sweep's checks rather than those below);
  none at 0.15 px either way.
- **The solve's dropped corners count, from 2 views on.** A lens off by
  pixels has its corners dropped and the rest fit (k3 raised by 3e-4: a
  median 7% of them, by 1e-3: 14%), so the share is held to 1%, as
  `check_lens`'s. A pose session's view has 8 corners, so one board dropped
  is past 1% under 100 captures, and the solve drops a whole view for one
  corner 2 to 3 px off: with 1% of corners 1.5 px off, bm-1's own residuals
  read suspect in 8.5, 7.5 and 10% of random sessions of 15, 20 and 30
  captures, and 2, 1 and 1.5% when the share counts only from 2 views on
  (`min_dropped_views`); beside bm-2, whose own read so more often, 3, 5.5
  and 4%, still above the design's 2%. Drops in one view read insufficient
  instead, naming them: capture the board there again. The cost is
  detection at 15 to 20 captures (k3 raised by 3e-4: suspect in 96 rather
  than 157 of 200 sessions of 15), those sessions insufficient, not good.
  Noise alone (`lens_check.sweeps`, 400 checks each) reads no camera suspect
  at 0.15 px from 15 to 40 captures; at 0.3 px 5.5, 4, 1 and 1% at 15, 20,
  30 and 40, all but one at 15 and one at 20 by drops from 2 views or more
  (the solve's first round leaves boards 1 to 2 px off where `initialise`
  marked views ambiguous); none on gridded sessions. Those drops are open,
  the solve's to mend; the rule is the owner's call.
- **Good holds as far as the corners reach** (`LensCheck::reach`, by
  `solve::pixel_reach`). Inner cells meet the 9 parts' rule: on
  `test::view_plan`'s 30 captures over bm-1's view, its true lens reads good
  with its corners 0.62 to 0.68 of the way to each image corner and side (a
  lens fit's gates ask 0.9), and k3 raised by 3e-4, 13 to 60 px in the outer
  columns, reads good too, in 200 of 200 sessions each. A floor on good is
  open, the owner's call: 0.7 would keep 23 of 72 good verdicts at 30 random
  captures and none on the gridded 30, and remove every good verdict for k3
  raised by 3e-4.
- **The lab's first pose session** (2026-11-02, 10 captures along the bottom
  of the pair's rectified view) reads insufficient for both cameras, the
  image's top and middle unjudged; RMS 0.163 and 0.156 px, the worst judged
  cell at 0.52 of its limit.

## 2026-11-02 — A DeckLink pair's poses

calib_rig solves a DeckLink pair's poses (`operation` `calibrate`) as it
solves the Femto rig's, the same solve and tag world, with what a stereo
source adds:

- **Lenses by `rig_lens`'s rule for poses** (the entry below), so a pair's
  come from the scenario's file alone. A factory camera's calibrated lens
  there is refused at any size, not only the stream's: the runtime reads the
  factory lens whatever the file holds. Each lens keeps its file's mark
  (`lens_source`, carried as `RigCamera::source`), so the pair's file still
  says `calibrated`.
- **Each of `source.pairs` checked** (`check_pair_rows`) on the anchored
  file, named as the scenario names it, at `source.depth_width` x
  `depth_height`, which core resolves to FFS's 960 x 544 for a DeckLink
  pair: the size has one home. The verdict goes to `report.json`'s `pairs`
  and the Calibration window; the file is written whatever it is, as a lens
  that misses a gate is. `swapped` judges no rows. It says to name the left
  camera first in `source.pairs`, here and in FuseDecklink's scenario, and
  run the session again, when the cables are as in the lens session: lenses
  and poses are keyed by id, so swapping the inputs would hand each id the
  other camera's lens. If the cables moved since, each id's pictures already
  come from the camera whose lens it does not hold, so the cables go back.
  `insufficient` names where to capture.
- **Depth is checked only for cameras with depth;** a pair has none, so its
  report has no depth section.

## 2026-11-02 — Intrinsic mode

calib_rig's mode is its scenario's `operation` (the one it starts in, since
2026-11-03): `calibrate_intrinsics` fits
each camera's lens, `calibrate` solves the rig's poses as before. One table
holds what differs (`app/calib_mode.hpp`); the workers read the mode, never
the driver. `calib_app` needs core 0.40.0 (`VC_APP_CORE_MIN_VERSION`), the
rest of calib still 0.29.0.

- **Only what calib runs.** `check_scenario` takes those two operations, and
  the drivers `orbbec` and `decklink` (its poses from the entry above), each
  where the build has its arm; anything else is refused, naming the field
  and its value. Why: the core's config carries both from 0.40.0, and no
  scenario may run another solve than it asks for.
- **Each camera's live lens** (`rig_lens`): the scenario's file's
  (`source.calibration`) at exactly the stream's size, else the driver's
  factory lens, else, for lenses alone, a stand-in (fx = fy = width / 2,
  centred). Through it distance and tilt are rough and frontal views are not
  judged; corners, cells and reach need no lens. For poses a camera with no
  lens takes the stand-in too, and Calibrate waits for a real one (amended
  2026-11-03; it stopped the run), and a factory camera with a calibrated
  lens in the file is refused: the runtime reads the factory lens.
- **A fit's start** (`lens_start`): the camera's file lens, another camera's
  at its size, its factory lens; else OpenCV's own alone. Never the
  stand-in: from a poor start the fit lands in wrong minima.
- **The board** is entered in the Board window, seeded per mode, its kind
  the seed's: the checkerboard for lenses, the rig's ChArUco board for
  poses. A lens session takes a board of at least
  `LensOptions::min_corners` (12) inner corners, so a whole board gives a
  lens view: the rig's 5 x 3 board has 8 and is refused, saying why; the
  floor stays 12. No kind switch: the lab has no ChArUco board that large.
- **Capture and Calibrate.** One view is a lens capture (`min_views` 1). The
  Lenses panel shows each camera's readiness (`quality::lens_readiness`),
  and C waits for one camera to be ready. Calibrate fits each ready camera
  (`solve_lens`, `judge_lens`, `format_lens_results` over the scenario's
  file), lists the others with why, and fits a factory camera for its report
  alone; with no lens to write it leaves no calibration file. No origin tag
  search, no 3D view, no depth work.
- **The opener's arms are internal,** each built where sensor builds its
  driver: no public header of the app names a driver's type, `calib_app`
  links `sensor_orbbec` privately and builds with either driver. The
  DeckLink arm opens the scenario's pair with no calibration (sensor reports
  its cameras 0 x 0), its ports, mode and rate from the scenario; CI's
  ubuntu-24.04 leg builds it on the core's copy of the SDK's headers and
  runs the host tests on its path, with no card.
- **One detector.** `detect_charuco` and its header are gone: `detect_board`
  finds both kinds, its ChArUco half the one home of that search.

## 2026-11-02 — Every camera through one source

calib reads its cameras through one class, `source::RigSource`
(`source/rig_source.hpp`, tier `calib_source`, after `detect`): a
hardware-synced array (the Femto rig), or one stereo sensor and its two
cameras' ids (a DeckLink pair).

- **Grey pictures, made on the GPU.** Each `take()` polls -- the array's next
  set, or the pair's newest -- makes every picture's 8-bit luma and its half
  in one pass of sensor's `LumaPass`, whatever its layout (NV12 images,
  nvJPEG's planes, V210, host YUYV or luma), and reads both back in one list
  read. The live view searches the half and refines each corner in the luma.
  The CPU halving (`cv::resize`) is gone: the GPU's half is byte-equal to it,
  and the board's corners are where it put them (0 px on 48 corners at 4K).
- **A set may be kept; host luma may not.** A `PictureSet` lives on the
  device and may be shared: each camera's presence, time and grey tile, the
  RGB-D set, an owner that holds the tiles, and, since 2026-11-03, its luma
  pass's ready value (the entry on the live view). `host()` is valid until
  the next take, as it sits in the readback's reused staging buffer, so the
  live snapshot keeps the set and the detections, never the host views.
- **One code names a driver.** The app's opener (`app/open_source.hpp`)
  opens the cameras and hands the rest a source and calib's own camera
  records. Its arms are Orbbec's and DeckLink's (the entry on intrinsic
  mode). The workers hold no driver type and cast nothing.
- **Loss as sensor counts it.** A camera's `failed` frames are a loss; its
  `dropped` ones, a newer frame winning, and the sets an array skips are not.
- **The spread warning per source.** An array keeps 5 ms. A stereo sensor
  never hands out a pair past its own bound (`StereoInfo::max_skew_ns`, half
  a frame free-running: 16.7 ms at 30p), and a capture waits for a still
  board, so the warning rises to that bound: a pair's skew is no warning, and
  a moving board is not captured.
- **Any camera saves.** A capture copies each camera's host luma, plus its
  JPEG and depth where its frame has them. A camera with no JPEG is saved as
  a PGM of its luma (8.3 MB at 4K); a camera with depth still needs it.
  `session.json` is version 2: calib's own camera record (id, model, size,
  optional factory lenses and Orbbec facts, an input, its lens's origin and a
  lens file's SHA-256), and the session's mode and driver; a sync file, a
  depth mode, the SDK's version and the factory record only where the rig
  has them. A Femto session's file is version 1's but for the version, `id`
  for `serial`, and those keys.
- **Grey tiles.** The mosaic writes a luma picture in a device buffer as a
  grey tile, its chroma one neutral buffer made once; a decoder's NV12 images
  stay a colour tile. So every camera's tile shows, nvJPEG's on Linux too.
- **The cost.** On the M5 Max (Release, 300 sets), a Femto-like set of four
  4K NV12 pictures takes 11.6 ms median, 10.8 before (+0.8 ms): copying each
  luma image and making its half (0.8 ms on the GPU) outweigh the CPU halving
  saved (0.3 ms). A 4K V210 pair takes 11.1 ms, 9.0 of it the search. The
  GPU-only rule decides it; sensor's pass reading NV12 images in place, or
  one submit for the pass and the readback, would win most of it back.
- **Public, not promised.** `source/rig_source.hpp` carries sensor's pre-1.0
  driver-side types and has one user, `calib_rig`, so it stays outside
  README's stable table until a second consumer. `capture` takes plain
  records, no source type.
- **calib needs sensor 0.21.0**, for `LumaPass` and the list read on its prep
  tier, and its stereo tier with `VC_WITH_RIG`.

## 2026-11-02 — A stereo pair's rectified rows

`quality::check_pair_rows` says whether a solved pair's rows agree where its
stereo matcher searches (`quality/pair_row_check.hpp`). Δy is a board corner's
row in the left rectified view less its row in the right one. The limits are
estimates until a real session is judged.

- **Through the runtime's own rectification:** sensor's `stereo_calibration`
  and `rectify_stereo`, the functions its `StereoRectifier` uses, at the size
  and alpha the caller gives, the runtime's from core's config: so the check
  measures the runtime as it is, and calib keeps no copy of Bouguet's method.
  The size has no default; its one home is the config.
- **Each corner undistorted** by `compute::undistort_points`, iterated toward
  1e-10 px (within 1.2e-8 px out to the lab lens's image corners), then
  turned by its camera's `R1` or `R2` and projected by the rectified camera.
  On the lab's lens OpenCV's default five iterations miss by 1.7 px at 45
  degrees off axis, 16 px at 57, where the runtime's view ends, and 152 px at
  the image's corners. A corner outside either view is not counted.
- **Judged cell by cell:** on a 3 x 3 grid over the left view, each cell of 8
  corners or more holds its mean Δy within 0.15 px, or t standard errors of
  that mean if larger: t is Student's for the cell's corners at the two-sided
  0.27% of a normal's 3 (4.53 at 8 corners, 4.09 at 10, 3.64 at 15, 3.28 at
  30). A standard error from a few corners is often small by chance, and 3 of
  them failed right pairs on noise alone: on the lab-like pair, about 4% at
  0.4 px of corner noise and 8-10 corners a cell, up to 10% at 0.6 px. With
  t, noise fails a cell of a right pair at most 0.27% of the time, and a pair
  of nine judged cells at most 2.4%: on that pair, under 1% at 0.4 px and
  under 2% at 0.6 px. Where 0.15 px is the larger, as at the usual noise,
  nothing changes. Why not a percentile: noise sets it (0.1 px of corner
  noise gives a 95th percentile of 0.085 px, as each corner's slope predicts:
  0.22 rectified rows a source pixel at the view's centre, 0.49 at its
  corners), while what harms the matcher is local: a lens 3 px off at one
  edge reads 0.29 px in that cell and leaves the 95th percentile at 0.11 px.
  The median and 95th percentile are reported. At f 371 the gate is a
  relative pitch of 0.4 mrad.
- **Enough cells:** 7 judged, one in each outer column among them; else
  `Insufficient`, naming each part of the view short of corners and its count:
  where to capture. A cell past its limit fails the pair first: no capture
  elsewhere mends it.
- **Left and right are the config's.** When `rectify_stereo` refuses the pair
  as named, the other order is tried: `Swapped` if it rectifies, naming the
  left camera, else `Refused` with the first refusal: the pair's as named (a
  degenerate pair named the other way round reads "swap them"). No copy of the
  rectifier's rule and no parsing of its words. `stereo_calibration`'s
  refusals (an id not in the file, unposed or without a lens) are the
  caller's errors.
- **Named for its rows:** `quality::PairCheck` is already an ordered pair's
  depth agreement, in the stable API through `DepthSummary`, which
  `format_results` takes; renaming it would break the API for nothing.
- **The report's `pairs` section** joins version 2: each pair's verdict,
  cells, statistics, settings and its cameras' lens sources, `[]` with no
  pair checked. The lens file's path and digest are the application's to add
  when it checks pairs.

## 2026-11-01 — A camera's lens from its views

`solve::solve_lens` fits one camera's lens from its views of a board;
`quality::judge_lens` says what it is worth, and `format_lens_results` gives
its results files (`solve/lens.hpp`, `quality/lens_gate.hpp`). The numbers
below are estimates until a real session is judged; `examples/02_lens_from_images`
judges one again offline, from its saved pictures.

- **The lens stays sensor's eight coefficient rational one, R8.** Why: sensor's
  file and runtime read nothing else, it follows a fisheye truth within 0.3 px
  to 68 degrees once corners reach that far, and the runtime looks to about
  56. A fisheye model costs about four PRs; it comes only if the fold or bias
  gate fails on real data with full reach, or a rectification wider than alpha
  0 is wanted.
- **Two starts, the lower RMS kept.** From a poor start R8 lands in wrong
  minima, and OpenCV's own start at times throws or diverges, so
  `solve_lens` tries both, on the same corners in every round, so their RMS
  compares like with like. A stand-in lens is never a start. Two poor starts
  can still lose to a wrong minimum: the gates are the guard.
- **The hold-out goes by region and rhythm.** The middle view of every 5 in
  capture order within each cell of a 3 x 3 grid, where views centre: so a
  person's rhythm cannot put every held-out view in one place, and a region of
  3 gives one, where the 5th of each would give few from 40 views over 9
  regions. It never takes the last view reaching 0.9 toward a corner or side,
  so the fit keeps its reach.
- **Held-out corners answer to the final fit's corner bound,** as a fitted
  view's do, so the ratio compares like with like. Those past it are marked
  dropped and the view is posed again on the rest. The bound is at least 3 px,
  six times the 0.5 px gate, so an overfit of the size the gates judge cannot
  hide under it, and a detector's glitch does not end a session, which no
  later capture could mend: a held-out view stays held out. More than 1% of
  held-out corners past it fails, so a wrong lens cannot hide there either.
- **The adjustment's outlier bounds, corners first.** One rule, in
  `solve/common`. A lens fit has no robust loss, so a bad corner pulls its
  view's pose: its corners go first, a few good neighbours with them at
  worst, and the view is judged in the next round. A whole view shifted is not
  dropped: its pose takes up nearly all of it, 40 px and more on this lens, and
  it pulls the lens a little, to 1.7 px at 56 degrees for one such view. No
  physical cause of one is known.
- **A second pass when a round dropped anything.** OpenCV's own start can land
  at 2 px RMS in the first round, and what that fit condemns was never bad: one
  session lost a view that alone reached a corner. So the rounds run once more
  with everything back, from the lens they reached. That doubles the time, and
  only then.
- **The gates:** the held-out RMS at most 0.5 px at 3840 wide, scaled by width;
  it over the fit's at most 1.3, judged only above 0.1 px at that width, as a
  right lens down at the detector's noise (rendered pictures give 0.03 px) can
  miss a ratio of two small numbers; at most 1% of held-out corners past the
  bound; in each 8 x 6 cell of 30 fitted corners or more, the mean residual at
  most 0.1 px, or 3 standard errors of that mean if larger; no image corner past
  the lens's fold; and reach, below. Why the cells: each view's pose absorbs
  most of a wrong lens, and one 6 px off at 56 degrees passed every RMS gate,
  where right fits leave 0.03-0.07 px in the worst cell and wrong ones 0.12 px
  and up. The floor keeps noisier real corners from failing a right lens: the
  mean's standard error is taken as a vector, so a cell's false alarm is about
  1 in 2500 at 30 corners and 1 in 8000 in a large cell.
- **The fold is compared in normalized radius**, through sensor's
  `fold_radius2`: a lens whose fold lies past the frame is a good lens. A pole
  of its denominator bounds no radius, but a fit can hold one inside the frame
  with its numerator all but cancelling it, its curve running on past: so each
  image corner must also read nearer the axis than the fold through the fit.
  Boards stopping at 0.7 gave a pole at 55.8 degrees, the corners reading 79,
  and the runtime cuts every ray at that number. The 2024 bm-2 fit folds at 74.8
  degrees, its image corners about 79 degrees off axis; the lab's current lens
  never folds.
- **Sigma is reported, not gated.** R8's focal length trades against its
  coefficients: units' sets differ 5 times over while their curves agree.
- **Reach is lens-free:** how far corners get toward an image corner or side,
  along the line to it, as a fraction of the way. A camera is ready at 30 views
  to fit, 75% of the 8 x 6 cells, 0.9 toward every corner and side, and at most
  half frontal, that last with a real lens only: a stand-in's tilt is wrong at
  a fisheye's edge. Corner cells alone are met at about 56 degrees, where R8
  fits still fold inside the frame; at 0.9, about 70, no right fit does.
- **Reach is a gate too,** on the fitted corners kept, beside readiness, which
  counts the plan before any fit and has only the app to ask it: a verdict's
  `pass` must hold by itself, and outlier rounds can take reach away. Boards
  stopping at 0.7 passed every other gate with 40 px off at 68 degrees, with
  no corner there to show it.
- **A lens-only file:** sensor's version 2, colour lenses alone: those solved,
  and the base file's other cameras' as they are. Its poses, solved with old
  lenses, its world and everything else are dropped. A lens that misses a gate
  is written with its verdict, so a trial pose can use it; one fitted for the
  report alone, a factory lens, is not written.
- **`report.json` is version 2**, with an optional `lenses` section; a lens run's
  report has that alone. `pairs` joins it with the pair check.
- **`RigCamera::source`** carries a lens's provenance into the file, where
  the adjustment wrote `Factory` for every camera.
- **calib needs sensor 0.20.0** (0.21.0 since 2026-11-02), for `fold_radius2`
  on its camera tier, and `opencv_calib` in the brain, privately, in the
  solve tier.

## 2026-11-01 — Plain checkerboards, one detector

A lens is fitted from a plain checkerboard, the lab's (13 x 8 inner corners
of 0.02 m, `legacy_checkerboard()`), which must reach the fisheye's image
corners, past where a whole board fits.

- **`compute::Board` builds a checkerboard**, with no OpenCV board: its
  corners sit on the ChArUco board's grid, row by row. `charuco()` is a
  pointer, null for a checkerboard (a break). Why: a checkerboard has no
  OpenCV board, and a pointer says so where it is read.
- **One geometric collinearity test** on the board's corners for both kinds,
  in place of OpenCV's ChArUco-only one: three corners not on one line span
  at least half a square. Its verdict on every ChArUco test view is
  OpenCV's.
- **`detect::detect_board` for both kinds.** A checkerboard is found by
  `findChessboardCornersSB` asked for 5 x 4 inner corners with
  `CALIB_CB_LARGER`, so any larger grid counts, a board cut by the picture's
  edge included. Why 5 x 4: asked for the whole board, SB finds no cut board;
  asked for 4 x 3, a still board cut by the edge comes back as a different
  grid from frame to frame; 5 x 4 and 6 x 5 are stable. So `check_target`
  needs at least 5 x 4, either way round (a break: it took 1 x 1, though no
  `Board` could be built from a checkerboard). In the picture (captures) SB
  runs with `NORMALIZE_IMAGE` and `EXHAUSTIVE`, not `ACCURACY` (four times the
  time for 0.01-0.02 px); in the half (`DetectOptions::half`, the live view)
  with `NORMALIZE_IMAGE`.
- **Each corner is refined in the picture**, on both paths, by the ChArUco
  half's refinement, which both kinds share. Why: SB's own corners err
  0.14-0.19 px (median) on a board square to the pixels, 0.02-0.09 px turned
  from them, on both renderers; refined, 0.02-0.05 px either way. These
  corners fit lenses.
- **Each corner SB returns is checked.** It is kept when it is in step with
  its neighbours along its row and column, and its four squares, seen at
  their centres, are dark and light by turns, with at least half the grid's
  median contrast and half the span from black to white around them; a grid
  with half its corners failing is dropped whole. Beside the picture's edge
  one square's centre may be off it. Why: grown past the size asked for, SB
  at times returns corners that are not the board's. On the design review's
  renderer 5 of 24 views in the fisheye's corners had corners 20-80 px off
  the board; on calib's, a whole board turned 45 degrees gained a row of the
  T-junctions where it meets its white margin (1 noise draw in 12), and
  searches returned grids skipping every other row or column. A skipping
  grid's corners are true, and its own median contrast is low, so only the
  span around each corner tells. Unchecked, such corners would be fitted,
  with wrong ids.
- **Each search seeds OpenCV's random numbers.** SB builds a k-d tree from
  `cv::theRNG()`, on the calling thread, and its grid depends on that tree:
  from different states, on one thread or several, one picture gave a 4 x 8
  grid, a 5 x 4 grid or none. `detect_board` starts them from one seed and
  puts them back after, so a picture always gives one detection and a
  caller's numbers stay its own. This makes SB repeatable, not right: noise
  moves its grid as the seed did, one more reason stillness pairs corners by
  position.
- **The grid found is numbered row-major from its own first corner**, turned
  a quarter when it fits the board only that way round; a grid larger than
  the board either way round is refused, naming both sizes. Its first corner
  can change between frames of a still board, which a lens does not mind:
  each view has its own pose. So an id names a corner in its own view only.
- **Stillness and live-vs-full pair a checkerboard's corners by position**
  (`quality::CornerMatch::Nearest`, within a third of the corner's spacing),
  a ChArUco board's by id. Why: by id, a still board whose numbering moved
  reads a square or more of motion, and the capture would time out in the
  corners the lens needs. With corners in both and none paired, the shift is
  a third of the spacing, so the capture rule says the board is moving, not
  gone. A move of almost exactly a whole number of squares between two live
  sets reads still; position cannot tell, and the odds are low.
- **The rig's solve refuses a checkerboard** (`initialise`,
  `bundle_adjust`): extrinsics pair corners across cameras by id, which only
  a board with markers gives.
- The session writer takes a checkerboard, and the board editor one in
  intrinsic mode (the 2026-11-02 entry).

## 2026-11-01 — Depth as sensor prepares it, outliers dropped

Calibrate's depth checks and the 3D view prepare depth with sensor's
defaults, so they drop depth outliers, as sensor does for every depth sensor
since 0.13.0: a return with fewer than 20 returns, itself included, within 8
pixel footprints is no return. calib requires sensor 0.13.0
(`VC_SENSOR_MIN_VERSION`) for it.

- **Kept, not turned off.** Why: it is the owner's default for every depth
  sensor, and calib checks and shows the depth a capture fuses. On
  Calibrate's synthetic depth (2 to 5 cameras) the filter moves no board's
  plane; a pair's pooled agreement changes by at most 190 of its 2 to 3
  million samples and its quantiles by under 0.5 µm, one capture's by under
  0.03 mm.

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
  `vkc_require_sibling(volumetric_kit::sensor 0.1.0 TARGETS …)`** (0.13.0
  since 2026-11-01, the entry above): camera-only
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
- **Colour lenses and poses only** in the results' calibration file (the
  user's call): `SensorArray` reads only the poses, and the session's
  factory file already has the depth models.
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
  rig's source (`source::RigSource` since 2026-11-02; see the dated entry),
  which is not thread-safe: it takes each set with every camera's luma and
  half, finds the board and checks each view, drives the capture rule, and
  publishes a snapshot, newest wins.
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
  units.
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
  over leaves them to the tile (amended 2026-10-27). Since 2026-11-02 such a
  camera shows its luma as a grey tile (the dated entry on the source).
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
- **A decoder's ready value, waited for on the device** (2026-11-03). A
  picture whose writer did not wait for its work (`YuvImage::ready`, as
  NVIDIA's decoders hand theirs out) is copied only in a frame whose
  submission waits for that value at the transfer stage. In the Images view
  that is NVDEC's NV12 images: before, only the luma batch's fence stood
  between CUDA's write and the window's copy on another queue. gfx's frame
  loop takes only values already reached, and no host waits on one (a host
  wait on a value CUDA sets wakes about 10 ms late), so the window reads the
  value, never waits on it, and records a set with a value not reached yet
  at a later frame, every tile keeping its last picture meanwhile. The live
  worker's luma pass and the 3D view's preparation wait for it in their own
  batches (sensor's `LumaPass` and `GpuFramePrep`); a capture copies host
  data alone.
- **The luma pass's value, too** (2026-11-03, amended the same day, the
  owner's call on #72's follow-ups). The pass writes the grey tiles and takes
  the decoder's images over on sensor's queue, and its fence makes no device
  write visible to the renderer's. So the source sets a timeline value on
  its queue after the pass and its readback, waited for on the host, so it
  is reached when the set comes (`source::PictureSet::ready`), as the 3D
  view's worker does after its preparation. Each frame that copies the set
  waits for it at the transfer stage, as it waits for a decoder's. The
  validation layer, synchronization validation included, reported nothing
  without it.
- **A set taken without luma is not shown** (2026-11-03, the same call). A
  set taken while the 3D view showed holds pictures no pass took over, and
  on NVIDIA the mosaic refuses them (NVDEC's images still owned outside
  Vulkan, nvJPEG's planes in a buffer): the first frame back said a refusal
  per camera. The window skips such a set, and each tile keeps its last
  picture until a set taken with luma comes, a set or two later.
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
- **Live corners are flagged, not refused.** The live view's corners, found
  in the half, are for display; the solver takes the capture's
  full-resolution ones. A
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
  (recon's `RgbdFrame::color_bitstream`), or, without one, its luma as a PGM
  (2026-11-02); raw 16-bit depth from a camera with depth, through recon's
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
