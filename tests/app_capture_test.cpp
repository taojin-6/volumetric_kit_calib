// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's capture worker: each capture the live view takes is detected
// again at full resolution in its luma, checked, compared with its live
// corners and written as a session under the scenario's root; drops, a full
// queue, requests that do not fit the rig, a missing root and the drain at
// shutdown. Synthetic board renders, a few cameras, no device. Then what a
// session records of each camera and of the software.

#include "volumetric_kit/calib/app/capture_worker.hpp"

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <regex>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include "fake_sensor.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/app/preset.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/detect/charuco.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/recon/io/image_io.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace fs = std::filesystem;
namespace camera = recon::camera;
namespace sensor = recon::sensor;
using json = nlohmann::json;
using Code = core::Status::Code;

constexpr std::uint32_t kWidth = 960;
constexpr std::uint32_t kHeight = 540;

// The rig's 4K colour lens at a quarter of its size.
camera::CameraModel lens() {
  camera::CameraModel m = test::color_like_lens();
  m.size = {kWidth, kHeight};
  m.intrinsics.fx /= 4.0;
  m.intrinsics.fy /= 4.0;
  m.intrinsics.cx /= 4.0;
  m.intrinsics.cy /= 4.0;
  return m;
}

camera::Mat4d board_pose(const compute::Board& board, std::size_t i) {
  const double k = static_cast<double>(i);
  return test::board_facing_camera(board, {0.02 * k, -0.02, 1.1 + 0.1 * k},
                                   15.0 + 5.0 * k, 6.0 * k - 5.0);
}

// A fresh directory, removed with what it holds when the test ends.
class ScratchDir {
 public:
  ScratchDir() {
    static std::atomic<int> counter{0};
    const auto* test = ::testing::UnitTest::GetInstance()->current_test_info();
    std::error_code ec;
    path_ = fs::temp_directory_path(ec) /
            ("vc_app_capture_" + std::to_string(::getpid()) + "_" +
             test->name() + "_" + std::to_string(counter++));
    fs::remove_all(path_, ec);
    fs::create_directories(path_, ec);
  }
  ~ScratchDir() {
    std::error_code ec;
    fs::remove_all(path_, ec);
  }
  ScratchDir(const ScratchDir&) = delete;
  ScratchDir& operator=(const ScratchDir&) = delete;
  const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

capture::SessionCamera rig_camera(std::size_t i) {
  capture::SessionCamera camera;
  camera.info.id = "SYN" + std::to_string(i);
  camera.info.model = "fake";
  camera.info.color = lens();
  camera.info.depth = test::depth_like_lens();
  camera.info.depth_to_color[3] = glm::dvec4(-0.032, 0.0, 0.002, 1.0);
  camera.info.role =
      i == 0 ? sensor::SyncRole::Primary : sensor::SyncRole::Secondary;
  camera.info.fps = 25;
  return camera;
}

CaptureSetup setup(const fs::path& root, std::size_t cameras,
                   std::size_t capacity = 4) {
  CaptureSetup out;
  out.root = root.string();
  out.software = app_software();
  out.sync_path = (root / "rig.json").string();
  out.reference = 0;
  out.depth_mode = depth_mode({640, 576}).value();
  for (std::size_t i = 0; i < cameras; ++i) {
    out.cameras.push_back(rig_camera(i));
  }
  out.capacity = capacity;
  // The writer records the sync file's SHA-256.
  EXPECT_TRUE(core::write_file(out.sync_path, "{\"devices\":[]}",
                               core::WriteMode::Replace)
                  .ok());
  return out;
}

// Camera `i`'s view of the board, rendered, and where its corners project.
struct View {
  std::vector<std::uint8_t> luma;
  std::vector<cv::Point2d> truth;
};

const View& board_view(std::size_t i) {
  static std::vector<View> views;
  if (views.size() <= i) views.resize(i + 1);
  if (views[i].luma.empty()) {
    const compute::Board board = test::femto_board();
    const cv::Mat render =
        test::render_board(lens(), board_pose(board, i), board);
    views[i].luma.assign(render.data, render.data + render.total());
    views[i].truth =
        test::project(lens(), board_pose(board, i), board.object_points());
  }
  return views[i];
}

std::vector<std::uint8_t> jpeg_of(std::size_t i, int tag) {
  return {0xFF,
          0xD8,
          0xFF,
          0xE0,
          static_cast<std::uint8_t>(i),
          static_cast<std::uint8_t>(tag),
          0xFF,
          0xD9};
}

std::vector<std::uint16_t> depth_of(std::size_t i) {
  std::vector<std::uint16_t> depth(640 * 576);
  for (std::size_t p = 0; p < depth.size(); ++p) {
    depth[p] = static_cast<std::uint16_t>(900 + i * 11 + p % 17);
  }
  return depth;
}

// A capture of `cameras` cameras, each live detection its corners shifted
// `live_shift_px` right of where they project; `blank` sees no board.
CaptureRequest request(std::size_t cameras, double live_shift_px, int tag = 0,
                       std::optional<std::size_t> blank = {}) {
  CaptureRequest out;
  out.board = legacy_femto_board();
  out.meta.time_ns = 1'760'000'000'000'000'000 + tag;
  out.meta.trigger_ns = 1'760'000'000'000'000'100ull + tag;
  out.meta.sequence = 40 + static_cast<std::uint64_t>(tag);
  out.meta.spread_ms = 1.25;
  out.meta.stillness_px = 0.0625;
  for (std::size_t i = 0; i < cameras; ++i) {
    CameraFrameCopy part;
    part.width = kWidth;
    part.height = kHeight;
    if (blank && *blank == i) {
      part.luma.assign(static_cast<std::size_t>(kWidth) * kHeight, 110);
    } else {
      const View& view = board_view(i);
      part.luma = view.luma;
      part.live.image_size = {kWidth, kHeight};
      for (std::size_t id = 0; id < view.truth.size(); ++id) {
        part.live.observation.points.push_back(
            {static_cast<int>(id),
             {view.truth[id].x + live_shift_px, view.truth[id].y},
             1.0});
      }
    }
    part.jpeg = jpeg_of(i, tag);
    part.depth = depth_of(i);
    part.timestamp_ns = out.meta.trigger_ns + i;
    part.sequence = out.meta.sequence;
    out.cameras.push_back(std::move(part));
  }
  return out;
}

std::unique_ptr<CaptureWorker> make(CaptureSetup s) {
  core::Result<std::unique_ptr<CaptureWorker>> made =
      CaptureWorker::create(std::move(s));
  EXPECT_TRUE(made.ok()) << made.status().message();
  return made.ok() ? std::move(made).value() : nullptr;
}

bool submit(CaptureWorker& worker, CaptureRequest r) {
  const core::Result<bool> queued = worker.submit(std::move(r));
  EXPECT_TRUE(queued.ok()) << queued.status().message();
  return queued.ok() && queued.value();
}

std::string read(const fs::path& path) {
  const core::Result<std::string> bytes = core::read_file(path.string());
  EXPECT_TRUE(bytes.ok()) << bytes.status().message();
  return bytes.ok() ? bytes.value() : std::string();
}

TEST(AppCapture, WritesEachCaptureDetectedAtFullResolution) {
  ScratchDir root;
  std::unique_ptr<CaptureWorker> worker = make(setup(root.path(), 3));
  ASSERT_NE(worker, nullptr);
  EXPECT_TRUE(worker->directory().empty());
  EXPECT_FALSE(worker->summary().has_value());
  ASSERT_TRUE(submit(*worker, request(3, 0.5)));
  worker->flush();
  EXPECT_EQ(worker->pending(), 0u);
  EXPECT_TRUE(worker->take_errors().empty());

  const std::vector<CaptureEvent> events = worker->take_events();
  ASSERT_EQ(events.size(), 1u);
  const CaptureEvent& saved = events[0];
  ASSERT_EQ(saved.kind, CaptureEvent::Kind::Saved) << saved.status.message();
  EXPECT_EQ(saved.meta.index, 0);
  EXPECT_EQ(saved.usable_views, 3);
  // The live corners sit 0.5 px from where they project, the full ones
  // within a few hundredths: flagged.
  ASSERT_TRUE(saved.meta.live_vs_full_px.has_value());
  EXPECT_NEAR(*saved.meta.live_vs_full_px, 0.5, 0.1);
  EXPECT_TRUE(saved.flagged);
  EXPECT_EQ(saved.meta.sequence, 40u);

  // The session, under the root, named for when it started.
  const fs::path dir = worker->directory();
  EXPECT_EQ(dir.parent_path(), root.path());
  EXPECT_TRUE(std::regex_match(dir.filename().string(),
                               std::regex(R"(\d{4}-\d{2}-\d{2}_\d{6})")));
  const json session = json::parse(read(dir / "session.json"));
  EXPECT_EQ(session["rig"]["reference"], "SYN0");
  EXPECT_EQ(session["streams"]["depth"]["mode"], "nfov_unbinned");
  EXPECT_EQ(session["streams"]["fps"], 25);
  EXPECT_EQ(session["software"]["opencv"], CV_VERSION);
  ASSERT_EQ(session["captures"].size(), 1u);
  EXPECT_TRUE(fs::exists(dir / "factory_calibration.json"));

  const json capture = json::parse(read(dir / "captures" / "000.json"));
  EXPECT_NEAR(capture["live_vs_full_px"].get<double>(), 0.5, 0.1);
  for (std::size_t i = 0; i < 3; ++i) {
    SCOPED_TRACE("camera " + std::to_string(i));
    const std::string serial = "SYN" + std::to_string(i);
    const std::vector<std::uint8_t> jpeg = jpeg_of(i, 0);
    EXPECT_EQ(
        read(dir / "images" / serial / ("calib_img_" + serial + "_000.jpg")),
        std::string(jpeg.begin(), jpeg.end()));
    const core::Result<std::vector<std::uint16_t>> depth =
        recon::io::load_depth_u16(
            (dir / "depth" / serial / ("calib_depth_" + serial + "_000.png"))
                .string(),
            640, 576);
    ASSERT_TRUE(depth.ok()) << depth.status().message();
    EXPECT_TRUE(depth.value() == depth_of(i));
    const json& cam = capture["cameras"][serial];
    EXPECT_EQ(cam["detection"]["method"], "full");
    EXPECT_EQ(cam["check"]["corners"], 8);
    const std::vector<cv::Point2d>& truth = board_view(i).truth;
    const json& ids = cam["detection"]["ids"];
    const json& corners = cam["detection"]["corners_px"];
    ASSERT_EQ(ids.size(), 8u);
    for (std::size_t k = 0; k < ids.size(); ++k) {
      const cv::Point2d& t = truth[ids[k].get<std::size_t>()];
      // Full mode's own bound (detect_charuco's tests).
      EXPECT_LT(std::hypot(corners[k][0].get<double>() - t.x,
                           corners[k][1].get<double>() - t.y),
                0.3);
    }
    EXPECT_EQ(cam["timestamp_ns"], 1'760'000'000'000'000'100ull + i);
  }
  EXPECT_EQ(capture["usable_views"], 3);

  // The session's checks count it.
  const std::optional<quality::SessionSummary> summary = worker->summary();
  ASSERT_TRUE(summary.has_value());
  EXPECT_EQ(summary->kept, 1);
  ASSERT_EQ(summary->cameras.size(), 3u);
  for (const quality::CameraSummary& cam : summary->cameras) {
    EXPECT_EQ(cam.usable_views, 1);
  }
}

// Live corners where the full ones are: not flagged. A camera that sees no
// board is saved all the same, with no view.
TEST(AppCapture, SavesEveryCameraWhateverItSees) {
  ScratchDir root;
  std::unique_ptr<CaptureWorker> worker = make(setup(root.path(), 3));
  ASSERT_NE(worker, nullptr);
  ASSERT_TRUE(submit(*worker, request(3, 0.0, 0, std::size_t{2})));
  worker->flush();
  const std::vector<CaptureEvent> events = worker->take_events();
  ASSERT_EQ(events.size(), 1u);
  ASSERT_EQ(events[0].kind, CaptureEvent::Kind::Saved);
  EXPECT_EQ(events[0].usable_views, 2);
  EXPECT_FALSE(events[0].flagged);
  ASSERT_TRUE(events[0].meta.live_vs_full_px.has_value());
  EXPECT_LT(*events[0].meta.live_vs_full_px, 0.1);
  const json capture =
      json::parse(read(fs::path(worker->directory()) / "captures/000.json"));
  EXPECT_EQ(capture["cameras"]["SYN2"]["check"]["verdict"], "none");
  EXPECT_TRUE(capture["cameras"]["SYN2"]["detection"]["ids"].empty());
  EXPECT_EQ(capture["usable_views"], 2);
}

TEST(AppCapture, DropsTheLastCaptureAndNamesADropBeforeAny) {
  ScratchDir root;
  std::unique_ptr<CaptureWorker> worker = make(setup(root.path(), 2));
  ASSERT_NE(worker, nullptr);
  ASSERT_TRUE(worker->drop_last().ok());
  worker->flush();
  std::vector<CaptureEvent> events = worker->take_events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].kind, CaptureEvent::Kind::Failed);
  EXPECT_EQ(events[0].status.domain(), Code::NotFound);
  EXPECT_EQ(events[0].meta.index, -1);

  ASSERT_TRUE(submit(*worker, request(2, 0.0, 1)));
  ASSERT_TRUE(submit(*worker, request(2, 0.0, 2)));
  ASSERT_TRUE(worker->drop_last().ok());  // after both, in order
  worker->flush();
  events = worker->take_events();
  ASSERT_EQ(events.size(), 3u);
  EXPECT_EQ(events[0].kind, CaptureEvent::Kind::Saved);
  EXPECT_EQ(events[0].meta.index, 0);
  EXPECT_EQ(events[1].kind, CaptureEvent::Kind::Saved);
  EXPECT_EQ(events[1].meta.index, 1);
  EXPECT_EQ(events[2].kind, CaptureEvent::Kind::Dropped);
  EXPECT_EQ(events[2].meta.index, 1);
  const std::optional<quality::SessionSummary> summary = worker->summary();
  ASSERT_TRUE(summary.has_value());
  EXPECT_EQ(summary->kept, 1);
  EXPECT_EQ(summary->dropped, 1);
  const json session =
      json::parse(read(fs::path(worker->directory()) / "session.json"));
  ASSERT_EQ(session["captures"].size(), 2u);
  EXPECT_FALSE(session["captures"][0]["dropped"].get<bool>());
  EXPECT_TRUE(session["captures"][1]["dropped"].get<bool>());
}

// A capture is held until written: with room for one, a second is refused
// until the first is on disk.
TEST(AppCapture, RefusesACaptureWhenFull) {
  ScratchDir root;
  std::unique_ptr<CaptureWorker> worker = make(setup(root.path(), 1, 1));
  ASSERT_NE(worker, nullptr);
  EXPECT_EQ(worker->capacity(), 1u);
  EXPECT_FALSE(worker->full());
  ASSERT_TRUE(submit(*worker, request(1, 0.0, 1)));
  EXPECT_TRUE(worker->full());
  EXPECT_EQ(worker->pending(), 1u);
  EXPECT_FALSE(submit(*worker, request(1, 0.0, 2)));
  worker->flush();
  EXPECT_FALSE(worker->full());
  EXPECT_TRUE(submit(*worker, request(1, 0.0, 3)));
  worker->flush();
  const std::vector<CaptureEvent> events = worker->take_events();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].meta.index, 0);
  EXPECT_EQ(events[1].meta.index, 1);
  EXPECT_EQ(events[1].meta.sequence, 43u);
}

TEST(AppCapture, RefusesARequestThatDoesNotFitTheRig) {
  ScratchDir root;
  std::unique_ptr<CaptureWorker> worker = make(setup(root.path(), 2));
  ASSERT_NE(worker, nullptr);
  core::Result<bool> queued = worker->submit(request(3, 0.0));
  ASSERT_FALSE(queued.ok());
  EXPECT_EQ(queued.status().domain(), Code::InvalidArgument);
  EXPECT_NE(queued.status().message().find("3 cameras' frames for a rig of 2"),
            std::string::npos);
  CaptureRequest small = request(2, 0.0);
  small.cameras[1].luma.pop_back();
  queued = worker->submit(std::move(small));
  ASSERT_FALSE(queued.ok());
  EXPECT_NE(queued.status().message().find(
                "SYN1's luma is not its colour image's size"),
            std::string::npos);
  CaptureRequest other = request(2, 0.0);
  other.cameras[0].width = kWidth / 2;
  EXPECT_FALSE(worker->submit(std::move(other)).ok());
  EXPECT_EQ(worker->pending(), 0u);
  worker->flush();
  EXPECT_TRUE(worker->take_events().empty());
  EXPECT_TRUE(worker->directory().empty());
}

TEST(AppCapture, RefusesASetupItCannotWrite) {
  ScratchDir root;
  const auto refused = [](CaptureSetup s, const std::string& text) {
    const core::Result<std::unique_ptr<CaptureWorker>> made =
        CaptureWorker::create(std::move(s));
    ASSERT_FALSE(made.ok()) << text;
    EXPECT_EQ(made.status().domain(), Code::InvalidArgument);
    EXPECT_NE(made.status().message().find(text), std::string::npos)
        << made.status().message();
  };
  CaptureSetup s = setup(root.path(), 2);
  s.root.clear();
  refused(s, "no directory");
  s = setup(root.path(), 2);
  s.cameras.clear();
  refused(s, "no cameras");
  s = setup(root.path(), 2);
  s.reference = 2;
  refused(s, "the reference is not a camera");
  s = setup(root.path(), 2);
  s.capacity = 0;
  refused(s, "a capacity of 0");
  s = setup(root.path(), 2);
  s.cameras[1].info.color.reset();
  refused(s, "SYN1 has no colour model");
}

// The writer finds no root: the capture fails, named, and the next is tried
// afresh.
TEST(AppCapture, AMissingRootFailsTheCapture) {
  ScratchDir root;
  CaptureSetup s = setup(root.path(), 1);
  s.root = (root.path() / "missing").string();
  std::unique_ptr<CaptureWorker> worker = make(std::move(s));
  ASSERT_NE(worker, nullptr);
  ASSERT_TRUE(submit(*worker, request(1, 0.0)));
  worker->flush();
  const std::vector<CaptureEvent> events = worker->take_events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].kind, CaptureEvent::Kind::Failed);
  EXPECT_EQ(events[0].status.domain(), Code::NotFound)
      << events[0].status.message();
  EXPECT_EQ(events[0].meta.index, -1);
  EXPECT_FALSE(worker->full());
  EXPECT_FALSE(fs::exists(root.path() / "missing"));
}

// Destroying the worker writes what it holds first.
TEST(AppCapture, WritesWhatItHoldsWhenDestroyed) {
  ScratchDir root;
  std::unique_ptr<CaptureWorker> worker = make(setup(root.path(), 2));
  ASSERT_NE(worker, nullptr);
  ASSERT_TRUE(submit(*worker, request(2, 0.0, 1)));
  ASSERT_TRUE(submit(*worker, request(2, 0.0, 2)));
  ASSERT_TRUE(worker->drop_last().ok());
  worker.reset();
  std::vector<fs::path> sessions;
  for (const auto& entry : fs::directory_iterator(root.path())) {
    if (entry.is_directory()) sessions.push_back(entry.path());
  }
  ASSERT_EQ(sessions.size(), 1u);
  EXPECT_TRUE(fs::exists(sessions[0] / "captures" / "000.json"));
  EXPECT_TRUE(fs::exists(sessions[0] / "captures" / "001.json"));
  const json session = json::parse(read(sessions[0] / "session.json"));
  ASSERT_EQ(session["captures"].size(), 2u);
  EXPECT_TRUE(session["captures"][1]["dropped"].get<bool>());
}

TEST(AppCapture, RecordsEachCameraAndItsDepthDelay) {
  sensor::SensorInfo info = rig_camera(1).info;
  const test::FakeSensor fake(info);
  sensor::OrbbecRigSyncConfig sync;
  sync.devices.push_back({"SYN0", {}});
  sync.devices.push_back({"SYN1", {}});
  sync.devices[1].sync.depth_delay_us = 160;
  core::Result<capture::SessionCamera> camera = session_camera(fake, sync);
  ASSERT_TRUE(camera.ok()) << camera.status().message();
  EXPECT_EQ(camera->info.id, "SYN1");
  EXPECT_EQ(camera->info.color->size.width, kWidth);
  EXPECT_EQ(camera->depth_delay_us, 160);
  // Not an Orbbec camera: no vendor facts.
  EXPECT_TRUE(camera->firmware.empty());
  EXPECT_TRUE(camera->ip_address.empty());
  EXPECT_TRUE(camera->sync_mode.empty());
  EXPECT_TRUE(camera->color_lens_model.empty());
  // A camera the sync file does not list has no delay.
  sync.devices.pop_back();
  camera = session_camera(fake, sync);
  ASSERT_TRUE(camera.ok());
  EXPECT_EQ(camera->depth_delay_us, 0);
}

TEST(AppCapture, RecordsTheSoftwareItWasBuiltFrom) {
  const capture::Software software = app_software();
  EXPECT_EQ(software.opencv, CV_VERSION);
  const std::regex commit("([0-9a-f]{40})?");
  for (const std::string* sha :
       {&software.calib, &software.core, &software.recon, &software.gfx}) {
    EXPECT_TRUE(std::regex_match(*sha, commit)) << *sha;
  }
  EXPECT_TRUE(
      std::regex_match(software.orbbec_sdk, std::regex(R"(\d+\.\d+\.\d+)")))
      << software.orbbec_sdk;
}

}  // namespace
}  // namespace volumetric_kit::calib::app
