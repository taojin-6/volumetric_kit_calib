// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's Setup panel: one app, the mode chosen in it. End to end, on a
// device, with one live worker and one capture worker throughout, the cameras
// never reopened: a fake pair like the lab's at 960 x 540, opened with no
// calibration, its lens session of checkerboard pictures through the lab's
// fisheye; Calibrate fits both lenses, which pass; "Use these lenses" takes
// them for a new lens session, and the switch to Poses closes it and opens a
// pose session, whose ChArUco pictures solve the pair with its rows passing
// and each lens checked good, in report.json, the lens session left on disk.
// Then, from host data: the lens runs listed under a root newest first with
// each camera's verdict, a report not a lens run's left out alone; a lens
// file checked against the rig's ids and sizes; a lens run giving the next
// session, picked or used alike, its lenses that passed and those it carried,
// never one that failed, saying so; a pose session refusing captures and
// Calibrate while a lens is missing, naming the camera; and the Setup panel's
// lens lines. The device part runs under the validation layer when the
// environment asks (VKC_TEST_VALIDATION) and skips without a device.

#include "volumetric_kit/calib/app/rig_setup.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "calibrate_worker_support.hpp"
#include "fake_stereo.hpp"
#include "lab_sessions.hpp"
#include "live_pair.hpp"
#include "result_checks.hpp"
#include "scratch_dir.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/quality/lens_gate.hpp"
#include "volumetric_kit/calib/source/rig_source.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = sensor::camera;
namespace fs = std::filesystem;
using json = nlohmann::json;
using Code = core::Status::Code;
using Verdict = quality::PairRowCheck::Verdict;
using test::ScratchDir;

constexpr camera::ImageSize kSize = test::kLabQuarter;

// A pair's camera record as the DeckLink arm gives it.
capture::SessionCamera pair_camera(const char* id, const char* input) {
  capture::SessionCamera camera;
  camera.id = id;
  camera.model = "fake stereo";
  camera.size = kSize;
  camera.fps = 30;
  camera.input = input;
  return camera;
}

class AppSetup : public core::test::VulkanDeviceTest {
 protected:
  core::DeviceRequirements requirements() const override {
    return sensor::device_requirements();
  }
};

// A results run's outcome, as the Calibrate worker publishes it.
std::shared_ptr<const CalibrateResult> run(
    CalibrateWorker& worker, std::shared_ptr<const KeptCaptures> kept) {
  const core::Result<bool> started = worker.request(std::move(kept));
  EXPECT_TRUE(started.ok() && started.value());
  EXPECT_TRUE(test::idle(worker));
  for (const core::Status& error : worker.take_errors()) {
    ADD_FAILURE() << error.message();
  }
  return worker.take_result();
}

// The owner's ask: one app, the mode chosen in it, lenses then poses with
// the cameras open throughout.
TEST_F(AppSetup, LensesThenPosesInOneApp) {
  const test::FisheyeTruth truth = test::quarter_fisheye();
  const std::vector<test::SyntheticCamera> pair = test::lab_pair();
  const Target lens_board = test::lab_lens_board();
  const Target pose_board = test::lab_pose_board();
  const std::vector<std::vector<std::uint8_t>> lens_pictures =
      test::lens_session_pictures(truth,
                                  compute::Board::create(lens_board).value());
  const std::vector<test::ShownPair> pose_pictures =
      test::pair_session_pictures(truth, pair,
                                  compute::Board::create(pose_board).value());
  ASSERT_GE(lens_pictures.size(), 38u);
  ASSERT_GE(pose_pictures.size(), 30u);

  // The lens scenario: no lens file, the pair and its rows.
  const ScratchDir root;
  RigScenario scenario;
  scenario.mode = capture::SessionMode::Intrinsics;
  scenario.driver = "decklink";
  scenario.pairs = {{"bm-1", "bm-2"}};
  scenario.pair_rows.rectified = {960, 544};
  OpenedSource opened;
  ASSERT_TRUE(
      add_stereo_camera(pair_camera("bm-1", "SDI 2"), scenario, "", opened)
          .ok());
  ASSERT_TRUE(
      add_stereo_camera(pair_camera("bm-2", "SDI 1"), scenario, "", opened)
          .ok());
  CaptureSetup setup;
  setup.root = root.path().string();
  setup.software = app_software();
  setup.driver = "decklink";
  setup.cameras = opened.cameras;
  setup.lenses = opened.lenses;
  LiveOptions live_options;
  RigMode window;
  ASSERT_TRUE(apply_mode(scenario, setup, live_options, window).ok());
  EXPECT_EQ(
      lens_statuses(setup.mode, setup.cameras, nullptr).value(),
      (std::vector<LensStatus>{LensStatus::StandIn, LensStatus::StandIn}));

  core::Result<std::unique_ptr<CaptureWorker>> made =
      CaptureWorker::create(setup);
  ASSERT_TRUE(made.ok()) << made.status().message();
  const std::unique_ptr<CaptureWorker> captures = std::move(made).value();
  auto stereo =
      std::make_unique<test::FakeStereo>(test::uncalibrated_pair_info());
  test::FakeStereo& fake = *stereo;
  source::StereoMember member{std::move(stereo), {"bm-1", "bm-2"}};
  member.sizes = {kSize, kSize};
  core::Result<std::unique_ptr<source::RigSource>> rig =
      source::RigSource::open(std::move(member), device(), allocator());
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  core::Result<std::unique_ptr<LiveWorker>> started =
      LiveWorker::create(std::move(rig).value(), setup.lenses, lens_board, true,
                         captures.get(), nullptr, live_options);
  ASSERT_TRUE(started.ok()) << started.status().message();
  const std::unique_ptr<LiveWorker> live = std::move(started).value();
  core::Result<std::unique_ptr<CalibrateWorker>> worker =
      CalibrateWorker::create();
  ASSERT_TRUE(worker.ok()) << worker.status().message();
  const std::unique_ptr<CalibrateWorker> calibrate = std::move(worker).value();

  // The lens session: both cameras see every board.
  test::StillPairs feed(fake, *live, kSize);
  for (std::size_t k = 0; k < lens_pictures.size(); ++k) {
    const std::shared_ptr<const LiveSnapshot> taken =
        feed.capture(lens_pictures[k], lens_pictures[k]);
    ASSERT_NE(taken, nullptr);
    ASSERT_EQ(taken->decision.state, quality::CaptureState::Take)
        << "lens placement " << k << ": " << taken->decision.message;
    captures->flush();
  }
  const std::shared_ptr<const KeptCaptures> lens_kept = captures->kept();
  ASSERT_NE(lens_kept, nullptr);
  ASSERT_EQ(lens_kept->captures.size(), lens_pictures.size());
  const fs::path lens_session = lens_kept->directory;
  const std::vector<quality::LensReadiness> ready = captures->lenses();
  ASSERT_EQ(ready.size(), 2u);
  EXPECT_TRUE(ready[0].ready) << ready[0].unmet;
  EXPECT_TRUE(ready[1].ready) << ready[1].unmet;
  const std::shared_ptr<const CalibrateResult> lenses =
      run(*calibrate, lens_kept);
  ASSERT_NE(lenses, nullptr);
  ASSERT_EQ(lenses->lenses.size(), 2u);
  for (const quality::LensVerdict& lens : lenses->lenses) {
    EXPECT_TRUE(lens.pass) << lens.fit.camera.id << ": "
                           << (lens.failed.empty() ? "" : lens.failed.front());
  }

  // "Use these lenses": a new lens session with the lenses that passed.
  const core::Result<LensResult> written = read_lens_run(lenses->directory);
  ASSERT_TRUE(written.ok()) << written.status().message();
  const core::Result<LensFile> used = lens_run_file(*written, setup.cameras);
  ASSERT_TRUE(used.ok()) << used.status().message();
  EXPECT_TRUE(used->notes.empty());
  EXPECT_EQ(used->path,
            (fs::path(lenses->directory) / "array_calibration.json").string());
  RigScenario next = scenario;
  next.calibration = used->calibration;
  next.calibration_path = used->path;
  core::Result<viz::BoardEditor> editor = switch_session(
      next, std::nullopt, setup, live_options, window, *live, *captures);
  ASSERT_TRUE(editor.ok()) << editor.status().message();
  EXPECT_EQ(
      lens_statuses(setup.mode, setup.cameras, setup.calibration.get()).value(),
      (std::vector<LensStatus>{LensStatus::Calibrated,
                               LensStatus::Calibrated}));
  EXPECT_EQ(setup.cameras[1].lens_file, used->path);

  // The switch to Poses: the board the pose session takes, a pose session's
  // capture rule, and the lenses through which each view is checked.
  next.mode = capture::SessionMode::Extrinsics;
  editor = switch_session(next, std::make_pair(pose_board, true), setup,
                          live_options, window, *live, *captures);
  ASSERT_TRUE(editor.ok()) << editor.status().message();
  EXPECT_EQ(editor->board(), pose_board);
  EXPECT_FALSE(editor->locked());
  EXPECT_FALSE(window.intrinsic);
  EXPECT_FALSE(window.calibrated);
  EXPECT_EQ(live->board(), pose_board);
  EXPECT_EQ(live->captures(), 0u);
  EXPECT_EQ(live->rule_options().min_views, capture_rule(next.mode).min_views);
  EXPECT_EQ(setup.lenses[0].intrinsics.fx,
            lenses->lenses[0].fit.camera.model.intrinsics.fx);
  const std::vector<LensStatus> statuses =
      lens_statuses(setup.mode, setup.cameras, setup.calibration.get()).value();
  EXPECT_EQ(statuses, (std::vector<LensStatus>{LensStatus::Calibrated,
                                               LensStatus::Calibrated}));
  captures->flush();
  EXPECT_EQ(captures->kept(), nullptr);

  for (std::size_t k = 0; k < pose_pictures.size(); ++k) {
    const std::shared_ptr<const LiveSnapshot> taken =
        feed.capture(pose_pictures[k].left, pose_pictures[k].right);
    ASSERT_NE(taken, nullptr);
    ASSERT_EQ(taken->decision.state, quality::CaptureState::Take)
        << "pose placement " << k << ": " << taken->decision.message;
    EXPECT_FALSE(taken->decision.few_views) << k;
    captures->flush();
  }
  live->stop();
  for (const core::Status& error : captures->take_errors()) {
    ADD_FAILURE() << error.message();
  }
  const std::shared_ptr<const KeptCaptures> pose_kept = captures->kept();
  ASSERT_NE(pose_kept, nullptr);
  ASSERT_EQ(pose_kept->captures.size(), pose_pictures.size());
  EXPECT_EQ(pose_kept->mode, capture::SessionMode::Extrinsics);
  EXPECT_EQ(pose_kept->board, pose_board);
  EXPECT_NE(fs::path(pose_kept->directory), lens_session);
  EXPECT_TRUE(session_ready(setup.mode, captures->summary(), {}, statuses));
  for (const solve::RigCamera& cam : pose_kept->cameras) {
    EXPECT_EQ(cam.source, camera::IntrinsicsSource::Calibrated) << cam.id;
  }

  const std::shared_ptr<const CalibrateResult> poses =
      run(*calibrate, pose_kept);
  ASSERT_NE(poses, nullptr);
  ASSERT_EQ(poses->pairs.size(), 1u);
  EXPECT_EQ(poses->pairs[0].verdict, Verdict::Pass) << poses->pairs[0].reason;
  ASSERT_EQ(poses->lens_checks.size(), 2u);
  const json report =
      json::parse(test::read(fs::path(poses->directory) / "report.json"));
  for (const quality::LensCheck& check : poses->lens_checks) {
    std::printf("lens check %s: %s, RMS %.3f px, %zu corners: %s\n",
                check.camera.c_str(), quality::to_string(check.verdict),
                check.rms_px, check.corners, check.reason.c_str());
    EXPECT_EQ(check.verdict, quality::LensCheck::Verdict::Good)
        << check.camera << ": " << check.reason;
    EXPECT_EQ(report["lens_checks"][check.camera]["verdict"], "good");
  }

  // The lens session is closed on disk, its captures listed, in its mode.
  const json closed = json::parse(test::read(lens_session / "session.json"));
  EXPECT_EQ(closed["mode"], "intrinsics");
  EXPECT_EQ(closed["captures"].size(), lens_pictures.size());
  const json opened_pose =
      json::parse(test::read(fs::path(pose_kept->directory) / "session.json"));
  EXPECT_EQ(opened_pose["mode"], "extrinsics");
  EXPECT_EQ(opened_pose["cameras"][0]["lens"]["origin"], "file");
  EXPECT_EQ(opened_pose["cameras"][0]["lens"]["path"], used->path);
}

// --- Host data.

camera::CameraModel lens_at(camera::ImageSize size, double f) {
  camera::CameraModel m;
  m.size = size;
  m.intrinsics = {f, f, 0.5 * size.width - 0.5, 0.5 * size.height - 0.5};
  return m;
}

// A lens run's verdict as judge_lens gives it, made by hand.
quality::LensVerdict verdict(const std::string& id, double f, bool pass) {
  quality::LensVerdict v;
  v.fit.camera = {id, lens_at(kSize, f), camera::IntrinsicsSource::Calibrated};
  v.pass = pass;
  if (!pass) {
    v.failed = {
        "the fitted corners reach 0.83 of the way toward the top-left "
        "corner, short of 0.9"};
  }
  return v;
}

// A lens run's results directory under `root`, as Calibrate writes it, over
// the file its session started from, `base`.
fs::path lens_run(const fs::path& root, const std::string& session,
                  const std::string& results,
                  const std::vector<quality::LensVerdict>& lenses,
                  const std::vector<std::string>& unsolved = {},
                  const camera::ArrayCalibration* base = nullptr) {
  const fs::path dir = root / session / "results" / results;
  fs::create_directories(dir);
  const quality::ResultFiles files =
      quality::format_lens_results(lenses, unsolved, base).value();
  EXPECT_TRUE(core::write_file((dir / "report.json").string(), files.report,
                               core::WriteMode::CreateNew)
                  .ok());
  if (!files.array_calibration.empty()) {
    EXPECT_TRUE(core::write_file((dir / "array_calibration.json").string(),
                                 files.array_calibration,
                                 core::WriteMode::CreateNew)
                    .ok());
  }
  return dir;
}

std::vector<capture::SessionCamera> lab_pair_records() {
  return {pair_camera("bm-5", "SDI 2"), pair_camera("bm-6", "SDI 1")};
}

// The picker's list: each lens run under the root, newest first by when
// Calibrate was pressed, with each camera's verdict; a pose run, a report
// that does not parse or holds a field of another type, and a directory
// without a lens file are left out, each alone.
TEST(AppSetupHost, ListsTheLensRunsNewestFirstWithEachVerdict) {
  const ScratchDir root;
  lens_run(root.path(), "2026-11-01_090000", "000_2026-11-01_100000",
           {verdict("bm-5", 900.0, true), verdict("bm-6", 901.0, true)});
  lens_run(root.path(), "2026-11-02_215407", "000_2026-11-02_235712",
           {verdict("bm-5", 902.0, true), verdict("bm-6", 903.0, false)});
  lens_run(root.path(), "2026-11-02_215407", "001_2026-11-02_220000",
           {verdict("bm-5", 904.0, false)}, {"bm-6"});
  // A pose run's directory has a lens file and a report without `lenses`.
  const fs::path poses =
      root.path() / "2026-11-03_080000" / "results" / "000_2026-11-03_090000";
  fs::create_directories(poses);
  ASSERT_TRUE(core::write_file((poses / "array_calibration.json").string(),
                               "{}", core::WriteMode::CreateNew)
                  .ok());
  ASSERT_TRUE(core::write_file(
                  (poses / "report.json").string(),
                  R"({"format": "volumetric_kit_calib.report", "version": 2})",
                  core::WriteMode::CreateNew)
                  .ok());
  const fs::path broken =
      lens_run(root.path(), "2026-11-03_080000", "001_2026-11-03_091500",
               {verdict("bm-5", 905.0, true)});
  fs::remove(broken / "report.json");
  ASSERT_TRUE(core::write_file((broken / "report.json").string(), "{",
                               core::WriteMode::CreateNew)
                  .ok());
  lens_run(root.path(), "2026-11-03_080000", "002_2026-11-03_092000", {},
           {"bm-5"});  // no lens written: no file
  // A report whose fields are of other types, each beside a lens file.
  const char* odd[] = {
      R"({"format": 5})",
      R"({"format": "volumetric_kit_calib.report", "lenses": {"cameras":
          {"bm-5": {"written": "yes", "verdict": "pass"}}}})",
      R"({"format": "volumetric_kit_calib.report", "lenses": {"cameras":
          {"bm-5": {"written": true, "verdict": 1}}}})"};
  for (int i = 0; i < 3; ++i) {
    const fs::path dir = lens_run(
        root.path(), "2026-11-04_080000",
        "00" + std::to_string(i) + "_2026-11-04_0900" + std::to_string(10 + i),
        {verdict("bm-5", 906.0, true)});
    fs::remove(dir / "report.json");
    ASSERT_TRUE(core::write_file((dir / "report.json").string(), odd[i],
                                 core::WriteMode::CreateNew)
                    .ok());
    const core::Status refused = read_lens_run(dir.string()).status();
    EXPECT_EQ(refused.domain(), Code::InvalidArgument) << odd[i];
    EXPECT_NE(refused.message().find((dir / "report.json").string() +
                                     ": not a lens run's"),
              std::string::npos)
        << refused.message();
  }

  const std::vector<LensResult> found =
      find_lens_results(root.path().string()).value();
  ASSERT_EQ(found.size(), 3u);
  EXPECT_EQ(found[0].results, "000_2026-11-02_235712");
  EXPECT_EQ(found[1].results, "001_2026-11-02_220000");
  EXPECT_EQ(found[2].results, "000_2026-11-01_100000");
  EXPECT_EQ(found[0].session, "2026-11-02_215407");
  EXPECT_EQ(found[0].path, (root.path() / "2026-11-02_215407" / "results" /
                            "000_2026-11-02_235712" / "array_calibration.json")
                               .string());
  using Cameras = std::vector<std::pair<std::string, bool>>;
  EXPECT_EQ(found[0].cameras, (Cameras{{"bm-5", true}, {"bm-6", false}}));
  EXPECT_EQ(found[1].cameras, (Cameras{{"bm-5", false}}));
  EXPECT_EQ(lens_result_label(found[0]).value(),
            "2026-11-02 23:57:12  bm-5 pass, bm-6 fail  "
            "(2026-11-02_215407/000)");
  EXPECT_EQ(lens_result_label(found[1]).value(),
            "2026-11-02 22:00:00  bm-5 fail  (2026-11-02_215407/001)");
  test::expect_refused(
      find_lens_results((root.path() / "none").string()).status(),
      Code::NotFound, "is not a directory");
  // One run read alone, as "Use these lenses" reads the run it shows.
  const LensResult one = read_lens_run((root.path() / "2026-11-02_215407" /
                                        "results" / "000_2026-11-02_235712")
                                           .string())
                             .value();
  EXPECT_EQ(one.path, found[0].path);
  EXPECT_EQ(one.cameras, found[0].cameras);
  test::expect_refused(read_lens_run((root.path() / "2026-11-03_080000" /
                                      "results" / "002_2026-11-03_092000")
                                         .string())
                           .status(),
                       Code::NotFound, "has no array_calibration.json");
}

// A lens file is checked against the rig: each camera's lens at its
// pictures' size, at least one of the rig's cameras there; a camera it lacks
// is noted.
TEST(AppSetupHost, ALensFileIsCheckedAgainstTheRig) {
  const ScratchDir dir;
  const auto write =
      [&](const char* name,
          const std::vector<std::pair<std::string, camera::ImageSize>>&
              lenses) {
        camera::ArrayCalibration file;
        for (const auto& [id, size] : lenses) {
          camera::SensorCalibration& entry = file.sensors.emplace_back();
          entry.id = id;
          entry.color = camera::CameraCalibration{
              lens_at(size, 900.0), camera::IntrinsicsSource::Calibrated};
        }
        const std::string path = (dir.path() / name).string();
        EXPECT_TRUE(core::write_file(
                        path, camera::format_array_calibration(file).value(),
                        core::WriteMode::CreateNew)
                        .ok());
        return path;
      };
  const std::vector<capture::SessionCamera> rig = lab_pair_records();
  const std::string both =
      write("both.json", {{"bm-5", kSize}, {"bm-6", kSize}});
  const LensFile read = load_lens_file(both, rig).value();
  EXPECT_EQ(read.path, both);
  EXPECT_TRUE(read.notes.empty());
  EXPECT_EQ(read.calibration->sensors.size(), 2u);

  const std::string one = write("one.json", {{"bm-5", kSize}, {"bm-1", kSize}});
  const LensFile partly = load_lens_file(one, rig).value();
  EXPECT_EQ(partly.notes,
            (std::vector<std::string>{"bm-6 has no lens in " + one}));

  const std::string sized =
      write("sized.json", {{"bm-5", kSize}, {"bm-6", {1920, 1080}}});
  test::expect_refused(load_lens_file(sized, rig).status(),
                       Code::InvalidArgument,
                       "lens file " + sized +
                           ": bm-6's lens is 1920x1080, its pictures 960x540");
  const std::string others =
      write("others.json", {{"bm-1", kSize}, {"bm-2", kSize}});
  test::expect_refused(load_lens_file(others, rig).status(),
                       Code::InvalidArgument,
                       "lens file " + others +
                           ": no lens for bm-5 or bm-6: it holds bm-1, bm-2");
  const std::string missing = (dir.path() / "missing.json").string();
  const core::Status gone = load_lens_file(missing, rig).status();
  EXPECT_FALSE(gone.ok());
  EXPECT_NE(gone.message().find("lens file " + missing), std::string::npos)
      << gone.message();
}

// A lens run gives the next session, picked in the list or used alike, its
// lens file without the lenses that failed in it, saying so; a pose session
// then refuses captures and holds Calibrate back, naming the camera whose
// lens failed, and the fix.
TEST(AppSetupHost, ALensRunGivesItsPassedLensesNeverAFailedOne) {
  const ScratchDir root;
  const std::vector<capture::SessionCamera> rig = lab_pair_records();
  const fs::path dir =
      lens_run(root.path(), "s", "000_2026-11-02_235712",
               {verdict("bm-5", 902.0, true), verdict("bm-6", 903.0, false)});
  const LensResult run = read_lens_run(dir.string()).value();
  const LensFile used = lens_run_file(run, rig).value();
  EXPECT_EQ(used.path, (dir / "array_calibration.json").string());
  ASSERT_EQ(used.calibration->sensors.size(), 1u);
  EXPECT_EQ(used.calibration->sensors[0].id, "bm-5");
  EXPECT_EQ(used.notes,
            (std::vector<std::string>{
                "bm-6's lens failed its gates in that run: not taken"}));
  // The list's run is the same run: the same lenses.
  const std::vector<LensResult> listed =
      find_lens_results(root.path().string()).value();
  ASSERT_EQ(listed.size(), 1u);
  const LensFile picked = lens_run_file(listed[0], rig).value();
  EXPECT_EQ(picked.notes, used.notes);
  ASSERT_EQ(picked.calibration->sensors.size(), 1u);
  EXPECT_EQ(picked.calibration->sensors[0].id, "bm-5");

  // Poses with them: bm-6's lens is missing, so a capture is refused and
  // Calibrate says why.
  std::vector<capture::SessionCamera> cameras = rig;
  for (capture::SessionCamera& camera : cameras) {
    ASSERT_TRUE(take_lens(capture::SessionMode::Extrinsics,
                          used.calibration.get(), used.path, camera)
                    .ok());
  }
  const std::vector<LensStatus> statuses =
      lens_statuses(capture::SessionMode::Extrinsics, cameras,
                    used.calibration.get())
          .value();
  EXPECT_EQ(statuses, (std::vector<LensStatus>{LensStatus::Calibrated,
                                               LensStatus::Missing}));
  EXPECT_EQ(capture_refusal(capture::SessionMode::Extrinsics, cameras, statuses)
                .value(),
            "bm-6 has no lens: load a lens file in Setup first");
  quality::SessionSummary linked;
  linked.ready = true;
  linked.cameras.resize(2);
  EXPECT_FALSE(
      session_ready(capture::SessionMode::Extrinsics, linked, {}, statuses));
  const std::vector<NotReady> why =
      calibrate_blockers(capture::SessionMode::Extrinsics, cameras, statuses,
                         used.path, linked, {}, {})
          .value();
  ASSERT_EQ(why.size(), 1u);
  EXPECT_EQ(why[0].camera, "bm-6");
  EXPECT_EQ(why[0].why, "no lens at 960x540 in " + used.path +
                            ": load a lens file in Setup, or fit it in Lenses "
                            "mode");

  // None passed and none carried: nothing to take.
  const fs::path failed = lens_run(root.path(), "s", "001_2026-11-02_235800",
                                   {verdict("bm-5", 902.0, false)});
  test::expect_refused(
      lens_run_file(read_lens_run(failed.string()).value(), rig).status(),
      Code::InvalidArgument,
      "no lens for bm-5 or bm-6 passed in that run, and none is carried");
}

// A run that fitted some cameras alone gives the others the lenses it
// carried from the file its session started from, so the next session keeps
// a camera's good lens; a camera whose refit failed takes neither.
TEST(AppSetupHost, ALensRunKeepsTheLensesItCarried) {
  const ScratchDir root;
  const std::vector<capture::SessionCamera> rig = lab_pair_records();
  camera::ArrayCalibration base;
  for (const char* id : {"bm-5", "bm-6"}) {
    camera::SensorCalibration& entry = base.sensors.emplace_back();
    entry.id = id;
    entry.color = camera::CameraCalibration{
        lens_at(kSize, 880.0), camera::IntrinsicsSource::Calibrated};
  }
  // bm-6 was not ready: the run fitted bm-5 alone.
  const fs::path dir =
      lens_run(root.path(), "s", "000_2026-11-03_100000",
               {verdict("bm-5", 902.0, true)}, {"bm-6"}, &base);
  const LensFile used =
      lens_run_file(read_lens_run(dir.string()).value(), rig).value();
  EXPECT_TRUE(used.notes.empty());
  ASSERT_EQ(used.calibration->sensors.size(), 2u);
  const camera::SensorCalibration* fresh =
      camera::find_sensor(*used.calibration, "bm-5");
  const camera::SensorCalibration* carried =
      camera::find_sensor(*used.calibration, "bm-6");
  ASSERT_NE(fresh, nullptr);
  ASSERT_NE(carried, nullptr);
  EXPECT_EQ(fresh->color->model.intrinsics.fx, 902.0);
  EXPECT_EQ(carried->color->model.intrinsics.fx, 880.0);
  std::vector<capture::SessionCamera> cameras = rig;
  for (capture::SessionCamera& camera : cameras) {
    ASSERT_TRUE(take_lens(capture::SessionMode::Extrinsics,
                          used.calibration.get(), used.path, camera)
                    .ok());
  }
  const std::vector<LensStatus> statuses =
      lens_statuses(capture::SessionMode::Extrinsics, cameras,
                    used.calibration.get())
          .value();
  EXPECT_EQ(statuses, (std::vector<LensStatus>{LensStatus::Calibrated,
                                               LensStatus::Calibrated}));
  EXPECT_EQ(capture_refusal(capture::SessionMode::Extrinsics, cameras, statuses)
                .value(),
            "");

  // bm-6 refitted and failed: its fit replaced the carried lens in the
  // run's file, so it takes none.
  const fs::path refit = lens_run(
      root.path(), "s", "001_2026-11-03_100500",
      {verdict("bm-5", 902.0, true), verdict("bm-6", 903.0, false)}, {}, &base);
  const LensFile partly =
      lens_run_file(read_lens_run(refit.string()).value(), rig).value();
  EXPECT_EQ(partly.calibration->sensors.size(), 1u);
  EXPECT_EQ(camera::find_sensor(*partly.calibration, "bm-6"), nullptr);

  // Both missing: one line names both; a lens session refuses nothing.
  const std::vector<LensStatus> none = {LensStatus::Missing,
                                        LensStatus::Missing};
  EXPECT_EQ(
      capture_refusal(capture::SessionMode::Extrinsics, rig, none).value(),
      "bm-5 and bm-6 have no lens: load a lens file in Setup first");
  EXPECT_EQ(capture_refusal(capture::SessionMode::Intrinsics, rig,
                            {LensStatus::StandIn, LensStatus::StandIn})
                .value(),
            "");
  test::expect_refused(
      capture_refusal(capture::SessionMode::Extrinsics, rig, {}).status(),
      Code::InvalidArgument, "2 cameras, 0 lens statuses");
}

// The Setup panel's lens lines: the file, then each camera's lens, a file's
// with its path, a missing one red with the fix.
TEST(AppSetupHost, TheSetupPanelSaysWhereEachLensComesFrom) {
  camera::ArrayCalibration file;
  camera::SensorCalibration& entry = file.sensors.emplace_back();
  entry.id = "bm-5";
  entry.color = camera::CameraCalibration{lens_at(kSize, 900.0),
                                          camera::IntrinsicsSource::Calibrated};
  std::vector<capture::SessionCamera> cameras = lab_pair_records();
  for (capture::SessionCamera& camera : cameras) {
    ASSERT_TRUE(
        take_lens(capture::SessionMode::Extrinsics, &file, "l.json", camera)
            .ok());
  }
  const std::vector<viz::PanelLine> poses =
      setup_lines(capture::SessionMode::Extrinsics, cameras, &file, "l.json")
          .value();
  ASSERT_EQ(poses.size(), 3u);
  EXPECT_EQ(poses[0].label, "lens file");
  EXPECT_EQ(poses[0].value, "l.json");
  EXPECT_EQ(poses[1].label, "lens bm-5");
  EXPECT_EQ(poses[1].value, "calibrated (l.json)");
  EXPECT_EQ(poses[1].tone, viz::Tone::Good);
  EXPECT_EQ(poses[2].value, "missing: load a lens file");
  EXPECT_EQ(poses[2].tone, viz::Tone::Bad);
  const std::vector<viz::PanelLine> lenses =
      setup_lines(capture::SessionMode::Intrinsics, cameras, &file, "l.json")
          .value();
  EXPECT_EQ(lenses[2].value, "stand-in");
  EXPECT_EQ(lenses[2].tone, viz::Tone::Normal);
  const std::vector<viz::PanelLine> none =
      setup_lines(capture::SessionMode::Intrinsics, lab_pair_records(), nullptr,
                  "")
          .value();
  EXPECT_EQ(none[0].value, "none");
  EXPECT_EQ(none[0].tone, viz::Tone::Muted);
}

}  // namespace
}  // namespace volumetric_kit::calib::app
