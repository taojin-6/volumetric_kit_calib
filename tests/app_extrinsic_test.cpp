// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's extrinsic mode for a stereo pair. End to end, on a device: a
// lens file as intrinsic mode writes it, each camera's lens fitted to a lens
// session through the lab's fisheye at 960 x 540; then a pose session of a
// fake stereo pair like the lab's (0.40 m apart, turned by its magnitudes),
// opened with no calibration, its lenses the file's, of 960 x 540 pictures of
// the ChArUco board rendered through the fisheye: the live view takes each
// still board both cameras see, the session is written, and Calibrate solves
// the pair, checks its rectified rows at FFS's 960 x 544 and writes the pair's
// file, whose poses lie within 1 mm and 0.05 degrees of the truth, its lenses
// still calibrated, and whose rows pass in every part of the view. Named the
// other way round, the pair is Swapped, the panel saying how to name it, and
// the file the same; from the centre's captures alone it is Insufficient, the
// panel saying where to capture. And what the pose session refuses of the
// file: a camera with no lens at the pictures' size, naming the file, never a
// stand-in. The device part runs under the validation layer when the
// environment asks (VKC_TEST_VALIDATION) and skips without a device.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include "detect_helpers.hpp"
#include "fake_stereo.hpp"
#include "live_pair.hpp"
#include "scratch_dir.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/app/calib_mode.hpp"
#include "volumetric_kit/calib/app/calibrate_worker.hpp"
#include "volumetric_kit/calib/app/capture_worker.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/app/open_source.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/detect/board.hpp"
#include "volumetric_kit/calib/quality/lens_gate.hpp"
#include "volumetric_kit/calib/quality/pair_row_check.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"
#include "volumetric_kit/calib/source/rig_source.hpp"
#include "volumetric_kit/calib/viz/panels.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = sensor::camera;
namespace fs = std::filesystem;
using json = nlohmann::json;
using Code = core::Status::Code;
using Verdict = quality::PairRowCheck::Verdict;
using test::ScratchDir;

constexpr camera::ImageSize kSize{960, 540};       // test::quarter_fisheye()'s
constexpr camera::ImageSize kRectified{960, 544};  // FFS's, as core resolves it
constexpr double kDeg = 3.14159265358979323846 / 180.0;

void refused(const core::Status& status, Code code, const std::string& text) {
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.domain(), code) << status.message();
  EXPECT_NE(status.message().find(text), std::string::npos)
      << status.message() << " lacks " << text;
}

// A pair's lens file as intrinsic mode writes it (format_lens_results): each
// camera's lens fitted (solve_lens, judge_lens) to a lens session's views
// through `truth`, written as `<dir>/array_calibration.json`.
std::string lens_file(const test::FisheyeTruth& truth, const fs::path& dir) {
  const compute::Board board = test::lab_checkerboard();
  std::vector<quality::LensVerdict> lenses;
  std::uint32_t seed = 1;
  for (const char* id : {"bm-1", "bm-2"}) {
    test::FisheyeCapture capture;
    capture.seed = seed++;
    capture.camera = id;
    const std::vector<Observation> views =
        test::fisheye_views(board, truth, capture);
    solve::LensFit fit =
        solve::solve_lens(id, truth.size, board, views).value();
    lenses.push_back(quality::judge_lens(std::move(fit)).value());
  }
  const quality::ResultFiles files =
      quality::format_lens_results(lenses, {}, nullptr).value();
  const std::string path = (dir / "array_calibration.json").string();
  EXPECT_TRUE(core::write_file(path, files.array_calibration,
                               core::WriteMode::CreateNew)
                  .ok());
  return path;
}

// The rig's ChArUco board at twice its size: at a quarter of 4K, its markers
// as many pixels as the lab's board's at 4K from twice as far.
Target pair_board() {
  Target board = legacy_femto_board();
  board.square_size_m *= 2.0;
  board.marker_size_m *= 2.0;
  return board;
}

// One placement of the board as both cameras of the pair picture it.
struct Shown {
  std::vector<std::uint8_t> left;
  std::vector<std::uint8_t> right;
};

// A pose session's pictures, as a person covers the pair's shared view by
// the runbook: the board's centre toward the middle of each of the 9 parts
// of the left camera's view the rows are judged in -- the centre first, then
// 35 degrees across and 22 up and down -- at 2.0 and 2.6 m, tilted 15 degrees
// one way and the other about its x axis; kept where the live view finds the
// board in both cameras' halves, and the capture in both full pictures.
std::vector<Shown> pair_session_pictures(
    const test::FisheyeTruth& truth,
    const std::vector<test::SyntheticCamera>& pair,
    const compute::Board& board) {
  test::RenderOptions options;
  options.ray = truth.pixel_to_ray();
  const camera::Mat4d left_to_world =
      camera::rigid_inverse(pair[0].world_to_camera);
  std::vector<Shown> shown;
  for (const double down : {0.0, -22.0, 22.0}) {
    for (const double across : {0.0, -35.0, 35.0}) {
      for (const double distance : {2.0, 2.6}) {
        for (const double tilt : {-15.0, 15.0}) {
          const double tx = std::tan(across * kDeg);
          const double ty = std::tan(down * kDeg);
          const camera::Mat4d board_to_world =
              left_to_world *
              test::facing(board, std::atan(std::hypot(tx, ty)) / kDeg,
                           std::atan2(ty, tx) / kDeg, distance, 0.0, tilt);
          Shown placed;
          bool seen = true;
          for (std::size_t c = 0; c < 2 && seen; ++c) {
            const cv::Mat picture = test::render_board(
                truth.frame(), pair[c].world_to_camera * board_to_world, board,
                options);
            const cv::Mat half = test::halved(picture);
            const detect::LumaView half_view = test::luma(half);
            const core::Result<Detection> live =
                detect::detect_board(test::luma(picture), board, {&half_view});
            const core::Result<Detection> full =
                detect::detect_board(test::luma(picture), board);
            seen = live.ok() && full.ok() && live->observation.size() >= 6 &&
                   full->observation.size() >= 6;
            (c == 0 ? placed.left : placed.right)
                .assign(picture.data, picture.data + picture.total());
          }
          if (seen) shown.push_back(std::move(placed));
        }
      }
    }
  }
  return shown;
}

// How far a solved pose is from the truth: the translation (mm) and the
// rotation (degrees) of the one to the other.
struct PoseError {
  double mm = 0.0;
  double deg = 0.0;
};
PoseError pose_error(const camera::Mat4d& solved, const camera::Mat4d& truth) {
  const camera::Mat4d off = camera::rigid_inverse(truth) * solved;
  const camera::Mat3d r(off);
  const double cosine =
      std::clamp(0.5 * (r[0][0] + r[1][1] + r[2][2] - 1.0), -1.0, 1.0);
  return {1000.0 * glm::length(camera::Vec3d(off[3])),
          std::acos(cosine) / kDeg};
}

class AppExtrinsic : public core::test::VulkanDeviceTest {
 protected:
  core::DeviceRequirements requirements() const override {
    return sensor::device_requirements();
  }
};

// The design's end to end: a lens file, then a pose session through a stereo
// source, gives the pair's file, its poses as true as the views allow and
// its rows passing.
TEST_F(AppExtrinsic, APairsSessionGivesItsPosesAndRows) {
  const test::FisheyeTruth truth = test::quarter_fisheye();
  ASSERT_EQ(truth.size.width, kSize.width);
  ASSERT_EQ(truth.size.height, kSize.height);
  const std::vector<test::SyntheticCamera> pair = test::lab_pair();
  const Target target = pair_board();
  const compute::Board board = compute::Board::create(target).value();
  const std::vector<Shown> shown = pair_session_pictures(truth, pair, board);
  // 36 placements; about 2 in every part of the view at least.
  ASSERT_GE(shown.size(), 30u);

  // The scenario as check_scenario reads README's pose scenario: calibrate,
  // the pair, the lens file intrinsic mode wrote.
  const ScratchDir root;
  const std::string lenses = lens_file(truth, root.path());
  RigScenario scenario;
  scenario.mode = capture::SessionMode::Extrinsics;
  scenario.driver = "decklink";
  scenario.calibration = std::make_shared<const camera::ArrayCalibration>(
      camera::read_array_calibration(lenses).value());
  scenario.pairs = {{"bm-1", "bm-2"}};
  scenario.pair_rows.rectified = kRectified;

  // The pair's records and lenses as the DeckLink arm makes them: the file's.
  OpenedSource opened;
  const struct {
    const char* id;
    const char* input;
  } ports[] = {{"bm-1", "SDI 2"}, {"bm-2", "SDI 1"}};
  for (const auto& port : ports) {
    capture::SessionCamera camera;
    camera.id = port.id;
    camera.model = "fake stereo";
    camera.size = kSize;
    camera.fps = 30;
    camera.input = port.input;
    const core::Status added =
        add_stereo_camera(std::move(camera), scenario, lenses, opened);
    ASSERT_TRUE(added.ok()) << added.message();
  }
  for (const capture::SessionCamera& camera : opened.cameras) {
    EXPECT_EQ(camera.lens, capture::LensOrigin::File);
    EXPECT_EQ(camera.lens_file, lenses);
  }

  CaptureSetup setup;
  setup.root = root.path().string();
  setup.software = app_software();
  setup.driver = "decklink";
  setup.cameras = opened.cameras;
  setup.lenses = opened.lenses;
  LiveOptions options;
  RigMode window;
  const viz::BoardEditor editor =
      apply_mode(scenario, setup, options, window).value();
  EXPECT_EQ(editor.board(), legacy_femto_board());  // this test's is twice it
  EXPECT_FALSE(window.intrinsic);
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
      LiveWorker::create(std::move(rig).value(), setup.lenses, target, true,
                         captures.get(), nullptr, options);
  ASSERT_TRUE(started.ok()) << started.status().message();
  const std::unique_ptr<LiveWorker> live = std::move(started).value();

  test::StillPairs feed(fake, *live, kSize);
  for (std::size_t k = 0; k < shown.size(); ++k) {
    const std::shared_ptr<const LiveSnapshot> taken =
        feed.capture(shown[k].left, shown[k].right);
    ASSERT_NE(taken, nullptr);
    ASSERT_EQ(taken->decision.state, quality::CaptureState::Take)
        << "placement " << k << ": " << taken->decision.message;
    EXPECT_FALSE(taken->decision.few_views) << k;
    captures->flush();
  }
  live->stop();
  for (const core::Status& error : captures->take_errors()) {
    ADD_FAILURE() << error.message();
  }
  const std::shared_ptr<const KeptCaptures> kept = captures->kept();
  ASSERT_NE(kept, nullptr);
  ASSERT_EQ(kept->captures.size(), shown.size());
  EXPECT_EQ(kept->mode, capture::SessionMode::Extrinsics);
  ASSERT_EQ(kept->pairs.size(), 1u);
  EXPECT_EQ(kept->pairs[0].left, "bm-1");
  EXPECT_EQ(kept->pairs[0].right, "bm-2");
  EXPECT_EQ(kept->pair_rows.rectified.width, kRectified.width);
  EXPECT_EQ(kept->pair_rows.rectified.height, kRectified.height);
  // The lenses from the file keep its mark, which the solve carries.
  for (const solve::RigCamera& camera : kept->cameras) {
    EXPECT_EQ(camera.source, camera::IntrinsicsSource::Calibrated) << camera.id;
  }

  const core::Result<CalibrateResult> result = calibrate(*kept);
  ASSERT_TRUE(result.ok()) << result.status().message();
  EXPECT_EQ(result->mode, capture::SessionMode::Extrinsics);
  EXPECT_FALSE(result->depth.has_value());  // a pair has no depth
  ASSERT_EQ(result->pairs.size(), 1u);
  const quality::PairRowCheck& rows = result->pairs[0];
  EXPECT_EQ(rows.verdict, Verdict::Pass) << rows.reason;
  EXPECT_EQ(rows.left, "bm-1");
  EXPECT_EQ(rows.right, "bm-2");
  EXPECT_EQ(rows.options.rectified.width, kRectified.width);
  EXPECT_EQ(rows.options.rectified.height, kRectified.height);
  EXPECT_EQ(rows.captures, shown.size());
  std::printf(
      "rows: %zu captures, %zu corners, median %.3f px, p95 %.3f px, cells",
      rows.captures, rows.corners, rows.dy_median_px, rows.dy_p95_px);
  for (std::size_t c = 0; c < rows.cell_mean_px.size(); ++c) {
    std::printf(" %.3f (%zu)", rows.cell_mean_px[c], rows.cell_corners[c]);
  }
  std::printf("\n");

  // Written, the pair's file reads back through sensor: each camera posed in
  // the left camera's world (no tag), its lens the file's, still calibrated.
  const core::Result<WrittenResults> written = write_results(
      kept->directory, 0, std::chrono::system_clock::now(), result->files);
  ASSERT_TRUE(written.ok()) << written.status().message();
  const core::Result<camera::ArrayCalibration> file =
      camera::read_array_calibration(written->directory +
                                     "/array_calibration.json");
  ASSERT_TRUE(file.ok()) << file.status().message();
  const camera::Mat4d truth_left = pair[0].world_to_camera;
  for (std::size_t c = 0; c < 2; ++c) {
    const camera::SensorCalibration* entry =
        camera::find_sensor(*file, pair[c].id);
    ASSERT_NE(entry, nullptr) << pair[c].id;
    ASSERT_TRUE(entry->color.has_value());
    EXPECT_EQ(entry->color->source, camera::IntrinsicsSource::Calibrated);
    EXPECT_EQ(entry->color->model.intrinsics.fx,
              camera::find_sensor(*scenario.calibration, pair[c].id)
                  ->color->model.intrinsics.fx);
    ASSERT_TRUE(entry->color_to_world.has_value());
    const camera::Mat4d truth_pose =
        truth_left * camera::rigid_inverse(pair[c].world_to_camera);
    const PoseError error = pose_error(*entry->color_to_world, truth_pose);
    std::printf("%s: %.3f mm, %.4f deg\n", pair[c].id.c_str(), error.mm,
                error.deg);
    EXPECT_LT(error.mm, 1.0) << pair[c].id;
    EXPECT_LT(error.deg, 0.05) << pair[c].id;
  }
  const json report =
      json::parse(test::read(fs::path(written->directory) / "report.json"));
  ASSERT_EQ(report["pairs"].size(), 1u);
  EXPECT_EQ(report["pairs"][0]["verdict"], "pass");
  EXPECT_TRUE(report["depth"].is_null());
  const json session =
      json::parse(test::read(fs::path(kept->directory) / "session.json"));
  EXPECT_EQ(session["mode"], "extrinsics");
  EXPECT_EQ(session["cameras"][0]["lens"]["origin"], "file");
  EXPECT_EQ(session["cameras"][0]["lens"]["path"], lenses);
  const std::vector<viz::PanelLine> passed =
      viz::pair_rows_panel(result->pairs).value();
  ASSERT_EQ(passed.size(), 1u);
  EXPECT_EQ(passed[0].label, "rows bm-1 | bm-2");
  EXPECT_EQ(passed[0].value.rfind("pass: cells 9/9, worst ", 0), 0u)
      << passed[0].value;
  EXPECT_EQ(passed[0].tone, viz::Tone::Good);

  // Named the other way round, the pair is Swapped, and the panel says how
  // to name it; the poses do not depend on its naming, so the file is the
  // same.
  KeptCaptures reversed = *kept;
  reversed.pairs = {{"bm-2", "bm-1"}};
  const core::Result<CalibrateResult> swapped = calibrate(reversed);
  ASSERT_TRUE(swapped.ok()) << swapped.status().message();
  ASSERT_EQ(swapped->pairs.size(), 1u);
  EXPECT_EQ(swapped->pairs[0].verdict, Verdict::Swapped)
      << swapped->pairs[0].reason;
  EXPECT_EQ(swapped->files.array_calibration, result->files.array_calibration);
  const std::vector<viz::PanelLine> named =
      viz::pair_rows_panel(swapped->pairs).value();
  ASSERT_EQ(named.size(), 1u);
  EXPECT_EQ(named[0].label, "rows bm-2 | bm-1");
  EXPECT_EQ(named[0].value,
            "swapped: bm-1 is the left camera: put it first in source.pairs, "
            "here and in the runtime's scenario; the file needs no change");
  EXPECT_EQ(named[0].tone, viz::Tone::Warn);

  // From the centre's captures alone, too few parts of the view hold
  // corners: Insufficient, and the panel says where to capture.
  KeptCaptures centre = *kept;
  centre.captures.resize(4);
  const core::Result<CalibrateResult> few = calibrate(centre);
  ASSERT_TRUE(few.ok()) << few.status().message();
  ASSERT_EQ(few->pairs.size(), 1u);
  EXPECT_EQ(few->pairs[0].verdict, Verdict::Insufficient)
      << few->pairs[0].reason;
  const std::vector<viz::PanelLine> where =
      viz::pair_rows_panel(few->pairs).value();
  ASSERT_EQ(where.size(), 1u);
  EXPECT_NE(where[0].value.find("top-left (0)"), std::string::npos)
      << where[0].value;
  EXPECT_NE(where[0].value.find("capture the board there, then press C again"),
            std::string::npos)
      << where[0].value;
  EXPECT_EQ(where[0].tone, viz::Tone::Warn);
}

// A pose session's lenses come from the scenario's file alone: at the
// pictures' size, each keeps the file's mark; a camera the file lacks, or
// has at another size only, stops the run, naming the file, for there is no
// stand-in for poses.
TEST(AppExtrinsicLenses, ComeFromTheScenariosFileAlone) {
  const ScratchDir dir;
  camera::ArrayCalibration lenses;
  camera::CameraModel at_size = test::quarter_fisheye().frame();
  for (const char* id : {"bm-1", "bm-3"}) {
    camera::SensorCalibration& entry = lenses.sensors.emplace_back();
    entry.id = id;
    entry.color = camera::CameraCalibration{
        at_size, camera::IntrinsicsSource::Calibrated};
  }
  lenses.sensors[1].color->model = test::lab_lens();  // 4K only
  const std::string path = (dir.path() / "array_calibration.json").string();
  ASSERT_TRUE(core::write_file(path,
                               camera::format_array_calibration(lenses).value(),
                               core::WriteMode::CreateNew)
                  .ok());
  RigScenario scenario;
  scenario.mode = capture::SessionMode::Extrinsics;
  scenario.driver = "decklink";
  scenario.calibration = std::make_shared<const camera::ArrayCalibration>(
      camera::read_array_calibration(path).value());
  const auto camera_of = [](const char* id) {
    capture::SessionCamera camera;
    camera.id = id;
    camera.size = kSize;
    return camera;
  };
  OpenedSource opened;
  ASSERT_TRUE(
      add_stereo_camera(camera_of("bm-1"), scenario, path, opened).ok());
  ASSERT_EQ(opened.cameras.size(), 1u);
  EXPECT_EQ(opened.cameras[0].lens, capture::LensOrigin::File);
  EXPECT_EQ(lens_source(opened.cameras[0], scenario.calibration.get()),
            camera::IntrinsicsSource::Calibrated);
  for (const char* id : {"bm-2", "bm-3"}) {
    SCOPED_TRACE(id);
    refused(add_stereo_camera(camera_of(id), scenario, path, opened),
            Code::InvalidArgument,
            std::string(id) + " has no lens at 960x540 in " + path +
                ": run intrinsic mode");
  }
  EXPECT_EQ(opened.cameras.size(), 1u);
}

}  // namespace
}  // namespace volumetric_kit::calib::app
