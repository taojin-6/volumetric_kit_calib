// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's intrinsic mode. End to end, on a device: a fake stereo pair,
// opened with no calibration, of 960 x 540 pictures of a checkerboard
// rendered through the lab's fisheye scaled from 4K; the live view finds it in
// each picture's half through a stand-in lens, a still board is captured at
// each placement with one view enough, the session is written, and
// Calibrate's lens run gives a lens file whose lens passes every gate, the
// right camera, which saw a few boards, listed as not ready. Then Calibrate's
// lens run on views alone: a factory camera fitted for its report and never
// written, a run with nothing to write leaving no calibration file, and the
// mode deciding the solve. The device part runs under the validation layer
// when the environment asks (VKC_TEST_VALIDATION) and skips without a device.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include "calibrate_worker_support.hpp"
#include "detect_helpers.hpp"
#include "fake_stereo.hpp"
#include "scratch_dir.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/app/calib_mode.hpp"
#include "volumetric_kit/calib/app/calibrate_worker.hpp"
#include "volumetric_kit/calib/app/capture_worker.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/detect/board.hpp"
#include "volumetric_kit/calib/source/rig_source.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = sensor::camera;
namespace fs = std::filesystem;
using json = nlohmann::json;
using Code = core::Status::Code;
using test::ScratchDir;

constexpr camera::ImageSize kSize{960, 540};
constexpr std::uint64_t kStartNs = 1'760'000'000'000'000'000ull;
constexpr std::uint64_t kPeriodNs = 33'333'333;  // 30 fps
constexpr auto kPatience = std::chrono::seconds(30);

// The lab's fisheye (test::FisheyeTruth's 4K defaults) at a quarter of its
// size, pixel centres at integers.
test::FisheyeTruth scaled_truth() {
  test::FisheyeTruth truth;
  truth.size = kSize;
  truth.f /= 4.0;
  truth.cx = (truth.cx + 0.5) / 4.0 - 0.5;
  truth.cy = (truth.cy + 0.5) / 4.0 - 0.5;
  return truth;
}

// The lab's checkerboard with 40 mm squares: at a quarter of 4K, half a 4K
// picture's pixels a square, as near as the 20 mm board at half the distance.
Target board_target() {
  Target board = legacy_checkerboard();
  board.square_size_m = 0.04;
  return board;
}

// The board at `pose`, through `truth`, when the live view finds it in the
// picture's half and the capture gives a view to fit: the picture, and the
// capture's corners.
struct Placed {
  std::vector<std::uint8_t> picture;
  Observation full;
};
std::optional<Placed> place(const test::FisheyeTruth& truth,
                            const compute::Board& board,
                            const camera::Mat4d& pose) {
  test::RenderOptions options;
  options.ray = truth.pixel_to_ray();
  const cv::Mat picture =
      test::render_board(truth.frame(), pose, board, options);
  const cv::Mat half = test::halved(picture);
  const detect::LumaView half_view = test::luma(half);
  const core::Result<Detection> live =
      detect::detect_board(test::luma(picture), board, {&half_view});
  const core::Result<Detection> full =
      detect::detect_board(test::luma(picture), board);
  if (!live.ok() || !full.ok() || live->observation.empty() ||
      full->observation.size() <
          static_cast<std::size_t>(solve::LensOptions{}.min_corners)) {
    return std::nullopt;
  }
  return Placed{{picture.data, picture.data + picture.total()},
                full->observation};
}

// A lens session's pictures, as a person makes them by the runbook: the
// board tilted 20-35 degrees near the axis and further out, its tilt's axis
// turning; then pushed toward each image corner and side, turned along the
// line to it, and held where the live view finds it and its corners reach
// 0.9 of the way there, as the Lenses panel asks, four times each.
std::vector<std::vector<std::uint8_t>> lens_session_pictures(
    const test::FisheyeTruth& truth, const compute::Board& board) {
  std::vector<std::vector<std::uint8_t>> pictures;
  const auto keep = [&](const camera::Mat4d& pose) {
    std::optional<Placed> placed = place(truth, board, pose);
    if (placed) pictures.push_back(std::move(placed->picture));
  };
  for (int i = 0; i < 18; ++i) {
    keep(test::facing(board, 8.0 + 6.0 * (i % 3), 20.0 * i,
                      0.65 + 0.05 * (i % 2), 40.0 * i, 25.0 + 5.0 * (i % 3)));
  }
  for (int i = 0; i < 18; ++i) {
    keep(test::facing(board, 38.0 + 4.0 * (i % 2), 20.0 * i + 10.0, 0.55,
                      40.0 * i + 15.0, 20.0 + 5.0 * (i % 3)));
  }
  // Toward each image corner and side, in solve::LensReach's order.
  const struct {
    double azimuth_deg;
    double roll_deg;
  } goals[] = {{-150.6, -17.0}, {-29.4, 17.0}, {150.6, 17.0},
               {29.4, -17.0},   {180.0, -5.0}, {0.0, 5.0}};
  constexpr int kEach = 4;
  for (std::size_t g = 0; g < std::size(goals); ++g) {
    int kept = 0;
    for (double off = 58.0; off <= 74.0 && kept < kEach; off += 2.0) {
      for (const double tilt : {0.0, 10.0, 20.0}) {
        for (const double turn : {0.0, -10.0, 10.0}) {
          if (kept == kEach) break;
          std::optional<Placed> placed =
              place(truth, board,
                    test::facing(board, off, goals[g].azimuth_deg, 0.5,
                                 goals[g].roll_deg + turn, tilt));
          if (!placed) continue;
          double reach = 0.0;
          for (const DetectedPoint& point : placed->full.points) {
            reach = std::max(reach, solve::pixel_reach(kSize, point.pixel)[g]);
          }
          if (reach < solve::kLensReach) continue;
          pictures.push_back(std::move(placed->picture));
          ++kept;
        }
      }
    }
  }
  return pictures;
}

// A stereo pair's camera record, as the DeckLink arm makes it: no factory
// lens, the stand-in its live lens.
capture::SessionCamera pair_camera(const std::string& id,
                                   const std::string& input) {
  capture::SessionCamera camera;
  camera.id = id;
  camera.model = "fake stereo";
  camera.size = kSize;
  camera.fps = 30;
  camera.input = input;
  camera.lens = capture::LensOrigin::StandIn;
  return camera;
}

class AppIntrinsic : public core::test::VulkanDeviceTest {
 protected:
  core::DeviceRequirements requirements() const override {
    return sensor::device_requirements();
  }

  // The next snapshot after number `after`; what the worker reports on the
  // way fails the test.
  static std::shared_ptr<const LiveSnapshot> next(LiveWorker& live,
                                                  std::uint64_t after) {
    const auto give_up = std::chrono::steady_clock::now() + kPatience;
    while (std::chrono::steady_clock::now() < give_up) {
      for (const core::Status& error : live.take_errors()) {
        ADD_FAILURE() << error.message();
      }
      std::shared_ptr<const LiveSnapshot> s = live.take_snapshot();
      if (s && s->number > after) return s;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ADD_FAILURE() << "no snapshot after " << after;
    return nullptr;
  }

  // Waits until the rule is armed (it answers a set-less poll with Waiting).
  static void wait_armed(const LiveWorker& live) {
    const auto give_up = std::chrono::steady_clock::now() + kPatience;
    while (live.decision().state != quality::CaptureState::Waiting) {
      ASSERT_LT(std::chrono::steady_clock::now(), give_up);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
};

// The design's end to end: an intrinsic session through a stereo source gives
// a lens file whose lens passes the gates; the camera short of views is
// listed, not written.
TEST_F(AppIntrinsic, ASessionGivesALensFileThatPassesItsGates) {
  const test::FisheyeTruth truth = scaled_truth();
  const Target target = board_target();
  const compute::Board board = compute::Board::create(target).value();
  const std::vector<std::vector<std::uint8_t>> pictures =
      lens_session_pictures(truth, board);
  // A lens is fitted from 30 views, and one in five or so is held out.
  ASSERT_GE(pictures.size(), 38u);
  const std::vector<std::uint8_t> blank(
      static_cast<std::size_t>(kSize.width) * kSize.height, 110);

  const ScratchDir root;
  CaptureSetup setup;
  setup.root = root.path().string();
  setup.software = app_software();
  setup.mode = capture::SessionMode::Intrinsics;
  setup.driver = "decklink";
  setup.cameras = {pair_camera("bm-1", "SDI 2"), pair_camera("bm-2", "SDI 1")};
  setup.lenses = {stand_in_lens(kSize), stand_in_lens(kSize)};
  core::Result<std::unique_ptr<CaptureWorker>> made =
      CaptureWorker::create(setup);
  ASSERT_TRUE(made.ok()) << made.status().message();
  const std::unique_ptr<CaptureWorker> captures = std::move(made).value();

  auto stereo =
      std::make_unique<test::FakeStereo>(test::uncalibrated_pair_info());
  test::FakeStereo& pair = *stereo;
  source::StereoMember member{std::move(stereo), {"bm-1", "bm-2"}};
  member.sizes = {kSize, kSize};
  core::Result<std::unique_ptr<source::RigSource>> rig =
      source::RigSource::open(std::move(member), device(), allocator());
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  LiveOptions options;
  options.rule = capture_rule(capture::SessionMode::Intrinsics);
  core::Result<std::unique_ptr<LiveWorker>> started =
      LiveWorker::create(std::move(rig).value(), setup.lenses, target, true,
                         captures.get(), nullptr, options);
  ASSERT_TRUE(started.ok()) << started.status().message();
  const std::unique_ptr<LiveWorker> live = std::move(started).value();

  // The right camera sees the board in the first few captures alone.
  constexpr std::size_t kRightSees = 6;
  std::uint64_t tick = 0;
  std::uint64_t seen = 0;
  const auto show = [&](std::size_t k) {
    ++tick;
    pair.push(test::host_pair(kSize.width, kSize.height, pictures[k],
                              k < kRightSees ? pictures[k] : blank,
                              kStartNs + tick * kPeriodNs, 2'400'000, tick));
    std::shared_ptr<const LiveSnapshot> s = next(*live, seen);
    if (s) seen = s->number;
    return s;
  };
  for (std::size_t k = 0; k < pictures.size(); ++k) {
    ASSERT_NE(show(k), nullptr);  // the board moved to its next place
    ASSERT_TRUE(live->arm());
    wait_armed(*live);
    const std::shared_ptr<const LiveSnapshot> taken = show(k);  // held still
    ASSERT_NE(taken, nullptr);
    ASSERT_EQ(taken->decision.state, quality::CaptureState::Take)
        << "placement " << k << ": " << taken->decision.message;
    // One view is a lens capture.
    EXPECT_FALSE(taken->decision.few_views) << k;
    captures->flush();
  }
  live->stop();
  for (const core::Status& error : captures->take_errors()) {
    ADD_FAILURE() << error.message();
  }
  const std::shared_ptr<const KeptCaptures> kept = captures->kept();
  ASSERT_NE(kept, nullptr);
  ASSERT_EQ(kept->captures.size(), pictures.size());
  EXPECT_EQ(kept->mode, capture::SessionMode::Intrinsics);

  // The Lenses panel's readiness: the left camera's, not the right's; no
  // frontal share through a stand-in.
  const std::vector<quality::LensReadiness> lenses = captures->lenses();
  ASSERT_EQ(lenses.size(), 2u);
  const solve::LensReach& reach = lenses[0].reach;
  EXPECT_TRUE(lenses[0].ready)
      << lenses[0].views << " views, " << lenses[0].cells << " of the cells, "
      << "reach " << reach[0] << " " << reach[1] << " " << reach[2] << " "
      << reach[3] << " " << reach[4] << " " << reach[5];
  EXPECT_FALSE(lenses[0].frontal.has_value());
  EXPECT_FALSE(lenses[1].ready);
  EXPECT_LE(lenses[1].views, static_cast<int>(kRightSees));

  const core::Result<CalibrateResult> result = calibrate(*kept);
  ASSERT_TRUE(result.ok()) << result.status().message();
  EXPECT_EQ(result->mode, capture::SessionMode::Intrinsics);
  EXPECT_EQ(result->ids, (std::vector<std::string>{"bm-1", "bm-2"}));
  ASSERT_EQ(result->lenses.size(), 1u);
  const quality::LensVerdict& lens = result->lenses[0];
  EXPECT_EQ(lens.fit.camera.id, "bm-1");
  EXPECT_TRUE(lens.pass) << (lens.failed.empty() ? "" : lens.failed.front());
  EXPECT_FALSE(lens.frame_folds);
  // Its start was OpenCV's own: the stand-in is never one.
  EXPECT_FALSE(lens.fit.from_start);
  EXPECT_TRUE(result->report_only.empty());
  ASSERT_EQ(result->unsolved.size(), 1u);
  EXPECT_EQ(result->unsolved[0].id, "bm-2");
  EXPECT_EQ(result->unsolved[0].reason.rfind("not ready: ", 0), 0u)
      << result->unsolved[0].reason;
  EXPECT_NE(result->unsolved[0].reason.find(" views of 30"), std::string::npos)
      << result->unsolved[0].reason;

  // Written, the lens file reads back through sensor: bm-1's lens, at the
  // pictures' size, calibrated and unposed; no bm-2.
  const core::Result<WrittenResults> written = write_results(
      kept->directory, 0, std::chrono::system_clock::now(), result->files);
  ASSERT_TRUE(written.ok()) << written.status().message();
  const core::Result<camera::ArrayCalibration> file =
      camera::read_array_calibration(written->directory +
                                     "/array_calibration.json");
  ASSERT_TRUE(file.ok()) << file.status().message();
  const camera::SensorCalibration* bm1 = camera::find_sensor(*file, "bm-1");
  ASSERT_NE(bm1, nullptr);
  ASSERT_TRUE(bm1->color.has_value());
  EXPECT_EQ(bm1->color->source, camera::IntrinsicsSource::Calibrated);
  EXPECT_EQ(bm1->color->model.size.width, kSize.width);
  EXPECT_EQ(bm1->color->model.size.height, kSize.height);
  EXPECT_FALSE(bm1->color_to_world.has_value());
  EXPECT_EQ(camera::find_sensor(*file, "bm-2"), nullptr);
  const json report =
      json::parse(test::read(fs::path(written->directory) / "report.json"));
  EXPECT_EQ(report["lenses"]["unsolved"], json::array({"bm-2"}));
  EXPECT_EQ(report["lenses"]["cameras"]["bm-1"]["verdict"], "pass");
  EXPECT_EQ(report["lenses"]["cameras"]["bm-1"]["written"], true);

  // The session says what it calibrated, on which board.
  const json session =
      json::parse(test::read(fs::path(kept->directory) / "session.json"));
  EXPECT_EQ(session["mode"], "intrinsics");
  EXPECT_EQ(session["board"]["type"], "checkerboard");
  EXPECT_EQ(session["board"]["squares"], json::array({14, 9}));
}

// A lens session's kept captures from each camera's views: capture i holds
// each camera's view of frame i, or none.
KeptCaptures lens_session(const std::vector<capture::SessionCamera>& cameras,
                          const std::vector<std::vector<Observation>>& views,
                          const std::string& directory) {
  KeptCaptures kept;
  kept.mode = capture::SessionMode::Intrinsics;
  kept.board = legacy_checkerboard();
  kept.sensors = cameras;
  kept.directory = directory;
  std::int64_t captures = 0;
  std::vector<std::map<std::int64_t, Observation>> by_frame(cameras.size());
  for (std::size_t c = 0; c < cameras.size(); ++c) {
    kept.cameras.push_back(
        {cameras[c].id,
         cameras[c].color ? *cameras[c].color : stand_in_lens(cameras[c].size),
         camera::IntrinsicsSource::Factory});
    for (const Observation& view : views[c]) {
      by_frame[c][view.frame_id] = view;
      captures = std::max(captures, view.frame_id + 1);
    }
  }
  for (std::int64_t i = 0; i < captures; ++i) {
    auto capture = std::make_shared<KeptCapture>();
    capture->index = static_cast<int>(i);
    for (std::size_t c = 0; c < cameras.size(); ++c) {
      Detection detection;
      detection.image_size = cameras[c].size;
      const auto found = by_frame[c].find(i);
      if (found != by_frame[c].end()) detection.observation = found->second;
      capture->views.push_back(std::move(detection));
    }
    capture->depth.resize(cameras.size());
    capture->tags.resize(cameras.size());
    capture->frontal.assign(cameras.size(), false);
    kept.captures.push_back(std::move(capture));
  }
  return kept;
}

// A camera of a lens session at 4K, through a stand-in, or with a factory
// lens.
capture::SessionCamera lens_camera(const std::string& id, bool factory) {
  capture::SessionCamera camera;
  camera.id = id;
  camera.model = factory ? "Orbbec Femto Mega" : "fake stereo";
  camera.size = {3840, 2160};
  camera.fps = 30;
  if (factory) {
    camera.color = test::color_like_lens();
    camera.lens = capture::LensOrigin::Factory;
  } else {
    camera.lens = capture::LensOrigin::StandIn;
  }
  return camera;
}

// A camera's views through the lab's fisheye at 4K, `count` of them at most.
std::vector<Observation> fisheye_views(const std::string& id,
                                       std::uint32_t seed,
                                       std::size_t count = 40) {
  test::FisheyeCapture capture;
  capture.seed = seed;
  capture.camera = id;
  std::vector<Observation> views = test::fisheye_views(
      test::lab_checkerboard(), test::FisheyeTruth{}, capture);
  if (views.size() > count) views.resize(count);
  return views;
}

// A factory camera's lens is fitted and reported, never written: the runtime
// reads the driver's. A camera short of views is listed, with why.
TEST(AppIntrinsicRun, AFactoryCameraIsFittedForItsReportAlone) {
  ASSERT_EQ(test::color_like_lens().size.width, 3840u);
  const KeptCaptures kept = lens_session(
      {lens_camera("bm-1", false), lens_camera("CL2A141000N", true),
       lens_camera("bm-3", false)},
      {fisheye_views("bm-1", 1), fisheye_views("CL2A141000N", 2),
       fisheye_views("bm-3", 3, 5)},
      "");
  const core::Result<CalibrateResult> result = calibrate(kept);
  ASSERT_TRUE(result.ok()) << result.status().message();
  ASSERT_EQ(result->lenses.size(), 2u);
  EXPECT_EQ(result->lenses[0].fit.camera.id, "bm-1");
  EXPECT_EQ(result->lenses[1].fit.camera.id, "CL2A141000N");
  EXPECT_EQ(result->report_only, (std::vector<std::string>{"CL2A141000N"}));
  ASSERT_EQ(result->unsolved.size(), 1u);
  EXPECT_EQ(result->unsolved[0].id, "bm-3");
  // Its 5 views, one held out.
  EXPECT_EQ(result->unsolved[0].reason, "not ready: 4 views of 30");

  const core::Result<camera::ArrayCalibration> file =
      camera::parse_array_calibration(result->files.array_calibration);
  ASSERT_TRUE(file.ok()) << file.status().message();
  ASSERT_EQ(file->sensors.size(), 1u);
  EXPECT_EQ(file->sensors[0].id, "bm-1");
  const json report = json::parse(result->files.report);
  const json& cameras = report["lenses"]["cameras"];
  EXPECT_EQ(cameras["bm-1"]["written"], true);
  EXPECT_EQ(cameras["CL2A141000N"]["written"], false);
  EXPECT_EQ(report["lenses"]["unsolved"], json::array({"bm-3"}));
}

// With no lens to write -- every camera a factory one -- the results have no
// calibration file.
TEST(AppIntrinsicRun, ARunWithNoLensToWriteHasNoCalibrationFile) {
  const ScratchDir session;
  const KeptCaptures kept =
      lens_session({lens_camera("CL2A141000N", true)},
                   {fisheye_views("CL2A141000N", 2)}, session.path().string());
  const core::Result<CalibrateResult> result = calibrate(kept);
  ASSERT_TRUE(result.ok()) << result.status().message();
  EXPECT_TRUE(result->files.array_calibration.empty());
  const core::Result<WrittenResults> written =
      write_results(session.path().string(), 0,
                    std::chrono::system_clock::now(), result->files);
  ASSERT_TRUE(written.ok()) << written.status().message();
  EXPECT_EQ(test::entries(written->directory),
            (std::set<std::string>{"report.json", "residuals.csv"}));
}

// The session's mode decides the solve: the same views, as an extrinsic
// session, go to the rig's solve, which takes no checkerboard.
TEST(AppIntrinsicRun, TheModeDecidesTheSolve) {
  KeptCaptures kept = lens_session(
      {lens_camera("bm-1", false), lens_camera("bm-2", false)},
      {fisheye_views("bm-1", 1, 3), fisheye_views("bm-2", 2, 3)}, "");
  const core::Result<CalibrateResult> lenses = calibrate(kept);
  ASSERT_TRUE(lenses.ok()) << lenses.status().message();
  EXPECT_EQ(lenses->mode, capture::SessionMode::Intrinsics);
  EXPECT_EQ(lenses->unsolved.size(), 2u);
  EXPECT_TRUE(lenses->files.array_calibration.empty());
  EXPECT_FALSE(lenses->files.report.empty());
  kept.mode = capture::SessionMode::Extrinsics;
  const core::Result<CalibrateResult> poses = calibrate(kept);
  ASSERT_FALSE(poses.ok());
  EXPECT_EQ(poses.status().domain(), Code::InvalidArgument);
  EXPECT_NE(poses.status().message().find("checkerboard"), std::string::npos)
      << poses.status().message();
}

}  // namespace
}  // namespace volumetric_kit::calib::app
