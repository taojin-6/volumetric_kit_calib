// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's capture worker: each capture the live view takes is detected
// again at full resolution in its luma, checked, compared with its live
// corners and written as a session under the scenario's root; drops, a full
// queue, requests that do not fit the rig, a missing root and the drain at
// shutdown; the session's depth unit, the frames'; the captures kept for
// Calibrate, each view named for its camera even when its detection failed,
// their depth shared with the writer, and their cap. Synthetic board renders,
// a few cameras, no device. Then what a session records of each camera and of
// the software.

#include "volumetric_kit/calib/app/capture_worker.hpp"

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/parallel/parallel_backend.hpp>

#include "allocation_failure.hpp"
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
    part.metres_per_unit = 0.001f;
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

// OpenCV's parallel_for_ run on the calling thread, a stripe at a time, with
// the first allocation of stripe `fail` of its first loop failing: the capture
// worker's loop over the cameras gives each camera a stripe.
class FailAStripe final : public cv::parallel::ParallelForAPI {
 public:
  explicit FailAStripe(int fail) : fail_(fail) {}
  void parallel_for(int tasks, FN_parallel_for_body_cb_t body,
                    void* data) override {
    const bool first = !used_.exchange(true);
    for (int t = 0; t < tasks; ++t) {
      if (first && t == fail_) {
        const test::FailAllocation failure(0);
        body(t, t + 1, data);
        injected_ = test::allocation_failure.injected;
      } else {
        body(t, t + 1, data);
      }
    }
  }
  int getThreadNum() const override { return 0; }
  int getNumThreads() const override { return threads_; }
  int setNumThreads(int threads) override {
    const int old = threads_;
    threads_ = threads;
    return old;
  }
  const char* getName() const override { return "fail_a_stripe"; }
  bool injected() const { return injected_; }

 private:
  int fail_;
  int threads_ = 0;
  std::atomic<bool> used_{false};
  std::atomic<bool> injected_{false};
};

// OpenCV's parallel_for_ on `api` while alive, then on OpenCV's own again.
// Made and destroyed while no other thread runs OpenCV.
class ParallelBackend {
 public:
  explicit ParallelBackend(std::shared_ptr<cv::parallel::ParallelForAPI> api) {
    cv::parallel::setParallelForBackend(api);
  }
  ~ParallelBackend() {
    cv::parallel::setParallelForBackend(
        std::shared_ptr<cv::parallel::ParallelForAPI>());
  }
  ParallelBackend(const ParallelBackend&) = delete;
  ParallelBackend& operator=(const ParallelBackend&) = delete;
};

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
  s = setup(root.path(), 2);
  s.max_kept = 0;
  refused(s, "a max_kept of 0");
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
  EXPECT_EQ(worker->kept(), nullptr);
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

// The session's depth unit is its first capture's frames'; a capture in
// another is refused, and so is one whose cameras disagree.
TEST(AppCapture, TakesTheDepthUnitFromTheFrames) {
  ScratchDir root;
  std::unique_ptr<CaptureWorker> worker = make(setup(root.path(), 2));
  ASSERT_NE(worker, nullptr);
  CaptureRequest first = request(2, 0.0, 1);
  for (CameraFrameCopy& part : first.cameras) part.metres_per_unit = 0.0005f;
  ASSERT_TRUE(submit(*worker, std::move(first)));
  ASSERT_TRUE(submit(*worker, request(2, 0.0, 2)));  // 0.001
  worker->flush();
  // The refusal is said at once, the save once written: either may be first.
  const std::vector<CaptureEvent> events = worker->take_events();
  ASSERT_EQ(events.size(), 2u);
  const auto of_kind = [&](CaptureEvent::Kind kind) {
    return std::find_if(events.begin(), events.end(),
                        [&](const CaptureEvent& e) { return e.kind == kind; });
  };
  ASSERT_NE(of_kind(CaptureEvent::Kind::Saved), events.end());
  const auto failed = of_kind(CaptureEvent::Kind::Failed);
  ASSERT_NE(failed, events.end());
  EXPECT_EQ(failed->status.message(),
            "capture: its depth unit is not the session's");
  const json session =
      json::parse(read(fs::path(worker->directory()) / "session.json"));
  EXPECT_EQ(session["streams"]["depth"]["metres_per_unit"].get<double>(),
            0.0005);
  EXPECT_EQ(session["captures"].size(), 1u);

  CaptureRequest mixed = request(2, 0.0, 3);
  mixed.cameras[1].metres_per_unit = 0.0005f;
  core::Result<bool> queued = worker->submit(std::move(mixed));
  ASSERT_FALSE(queued.ok());
  EXPECT_EQ(queued.status().message(),
            "capture: camera SYN1's depth unit is not camera SYN0's");
  CaptureRequest none = request(2, 0.0, 4);
  none.cameras[0].metres_per_unit = 0.0f;
  queued = worker->submit(std::move(none));
  ASSERT_FALSE(queued.ok());
  EXPECT_EQ(queued.status().message(),
            "capture: camera SYN0 has no depth unit");
}

// Each written capture is kept for Calibrate: every camera's full detection,
// numbered for the solve, and the very depth samples submitted, which the
// writer wrote; with the board, cameras, depth mode and thresholds the
// session was captured with.
TEST(AppCapture, KeepsEachWrittenCaptureForCalibrate) {
  ScratchDir root;
  CaptureSetup s = setup(root.path(), 3);
  s.session.solve.weak_below = 7;
  std::unique_ptr<CaptureWorker> worker = make(s);
  ASSERT_NE(worker, nullptr);
  EXPECT_EQ(worker->kept(), nullptr);
  std::vector<std::vector<const std::uint16_t*>> buffers;  // as submitted
  for (int tag = 0; tag < 3; ++tag) {
    // Capture 1's camera 2 sees no board.
    CaptureRequest r = request(3, 0.0, tag,
                               tag == 1 ? std::optional<std::size_t>(2)
                                        : std::optional<std::size_t>());
    buffers.emplace_back();
    for (const CameraFrameCopy& part : r.cameras) {
      buffers.back().push_back(part.depth.data());
    }
    ASSERT_TRUE(submit(*worker, std::move(r)));
  }
  worker->flush();
  EXPECT_TRUE(worker->take_errors().empty());
  const std::shared_ptr<const KeptCaptures> kept = worker->kept();
  ASSERT_NE(kept, nullptr);

  EXPECT_EQ(kept->board, legacy_femto_board());
  EXPECT_EQ(kept->directory, worker->directory());
  EXPECT_EQ(kept->reference, 0u);
  EXPECT_EQ(kept->solve.weak_below, 7);
  EXPECT_EQ(kept->depth_mode.name, s.depth_mode.name);
  EXPECT_EQ(kept->depth_mode.metres_per_unit, 0.001f);
  EXPECT_EQ(kept->depth_mode.min_m, s.depth_mode.min_m);
  EXPECT_EQ(kept->depth_mode.max_m, s.depth_mode.max_m);
  ASSERT_EQ(kept->cameras.size(), 3u);
  ASSERT_EQ(kept->sensors.size(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    const sensor::SensorInfo& given = s.cameras[i].info;
    EXPECT_EQ(kept->cameras[i].id, given.id);
    EXPECT_EQ(kept->cameras[i].model.intrinsics.fx, given.color->intrinsics.fx);
    EXPECT_EQ(kept->sensors[i].id, given.id);
    ASSERT_TRUE(kept->sensors[i].depth.has_value());
    EXPECT_EQ(kept->sensors[i].depth->size.width, given.depth->size.width);
    EXPECT_EQ(kept->sensors[i].depth_to_color, given.depth_to_color);
  }

  ASSERT_EQ(kept->captures.size(), 3u);
  const fs::path dir = worker->directory();
  for (std::size_t k = 0; k < 3; ++k) {
    SCOPED_TRACE("capture " + std::to_string(k));
    const KeptCapture& capture = *kept->captures[k];
    EXPECT_EQ(capture.index, static_cast<int>(k));
    ASSERT_EQ(capture.views.size(), 3u);
    ASSERT_EQ(capture.depth.size(), 3u);
    const json file = json::parse(
        read(dir / "captures" / ("00" + std::to_string(k) + ".json")));
    for (std::size_t i = 0; i < 3; ++i) {
      SCOPED_TRACE("camera " + std::to_string(i));
      const std::string serial = "SYN" + std::to_string(i);
      const Observation& view = capture.views[i].observation;
      EXPECT_EQ(view.camera_id, serial);
      EXPECT_EQ(view.frame_id, static_cast<std::int64_t>(k));
      // The detection the capture's file holds.
      const json& written = file["cameras"][serial]["detection"];
      ASSERT_EQ(view.points.size(), written["ids"].size());
      for (std::size_t p = 0; p < view.points.size(); ++p) {
        EXPECT_EQ(view.points[p].id, written["ids"][p].get<int>());
        EXPECT_EQ(view.points[p].pixel.x,
                  written["corners_px"][p][0].get<double>());
        EXPECT_EQ(view.points[p].pixel.y,
                  written["corners_px"][p][1].get<double>());
      }
      EXPECT_EQ(capture.views[i].markers.size(), written["marker_ids"].size());
      // The buffer submitted, which the writer wrote: never copied.
      ASSERT_NE(capture.depth[i], nullptr);
      EXPECT_EQ(capture.depth[i]->data(), buffers[k][i]);
      EXPECT_TRUE(*capture.depth[i] == depth_of(i));
      const core::Result<std::vector<std::uint16_t>> png =
          recon::io::load_depth_u16(
              (dir / "depth" / serial /
               ("calib_depth_" + serial + "_00" + std::to_string(k) + ".png"))
                  .string(),
              640, 576);
      ASSERT_TRUE(png.ok()) << png.status().message();
      EXPECT_TRUE(png.value() == *capture.depth[i]);
    }
  }
  EXPECT_TRUE(kept->captures[1]->views[2].observation.empty());
  EXPECT_FALSE(kept->captures[0]->views[2].observation.empty());
  const std::optional<quality::SessionSummary> summary = worker->summary();
  ASSERT_TRUE(summary.has_value());
  EXPECT_EQ(static_cast<std::size_t>(summary->kept), kept->captures.size());
}

// A camera whose full detection fails (out of memory here) is saved with no
// view, named, and its kept view still names its camera, as the solve takes
// every view.
TEST(AppCapture, AKeptViewNamesItsCameraWhenItsDetectionFailed) {
  ScratchDir root;
  const auto stripes = std::make_shared<FailAStripe>(1);
  const ParallelBackend backend(stripes);  // outlives the worker
  std::unique_ptr<CaptureWorker> worker = make(setup(root.path(), 3));
  ASSERT_NE(worker, nullptr);
  ASSERT_TRUE(submit(*worker, request(3, 0.0)));
  worker->flush();
  ASSERT_TRUE(stripes->injected());
  const std::vector<core::Status> errors = worker->take_errors();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].domain(), Code::OutOfMemory);
  EXPECT_EQ(errors[0].message().rfind("capture: camera SYN1", 0), 0u)
      << errors[0].message();
  const std::vector<CaptureEvent> events = worker->take_events();
  ASSERT_EQ(events.size(), 1u);
  ASSERT_EQ(events[0].kind, CaptureEvent::Kind::Saved)
      << events[0].status.message();
  EXPECT_EQ(events[0].usable_views, 2);

  const std::shared_ptr<const KeptCaptures> kept = worker->kept();
  ASSERT_NE(kept, nullptr);
  ASSERT_EQ(kept->captures.size(), 1u);
  const std::vector<Detection>& views = kept->captures[0]->views;
  ASSERT_EQ(views.size(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    SCOPED_TRACE("camera " + std::to_string(i));
    EXPECT_EQ(views[i].observation.camera_id, "SYN" + std::to_string(i));
    EXPECT_EQ(views[i].observation.frame_id, 0);
    EXPECT_EQ(views[i].observation.empty(), i == 1);
  }
}

// A drop publishes a new snapshot without the capture; one taken before is
// unchanged, and the dropped capture's depth goes with it: once flush()
// returns, the worker holds the snapshot it replaced no more.
TEST(AppCapture, ADropLeavesEarlierSnapshotsAndFreesTheDepth) {
  ScratchDir root;
  std::unique_ptr<CaptureWorker> worker = make(setup(root.path(), 2));
  ASSERT_NE(worker, nullptr);
  for (int tag = 0; tag < 3; ++tag) {
    ASSERT_TRUE(submit(*worker, request(2, 0.0, tag)));
  }
  worker->flush();
  std::shared_ptr<const KeptCaptures> before = worker->kept();
  ASSERT_NE(before, nullptr);
  ASSERT_EQ(before->captures.size(), 3u);
  const std::shared_ptr<const KeptCapture> first = before->captures[0];
  const std::weak_ptr<const std::vector<std::uint16_t>> dropped =
      before->captures[2]->depth[1];

  ASSERT_TRUE(worker->drop_last().ok());
  worker->flush();
  const std::shared_ptr<const KeptCaptures> after = worker->kept();
  ASSERT_NE(after, nullptr);
  EXPECT_NE(after, before);
  ASSERT_EQ(after->captures.size(), 2u);
  EXPECT_EQ(after->captures[0], first);  // shared, not copied
  EXPECT_EQ(after->captures[1]->index, 1);
  ASSERT_EQ(before->captures.size(), 3u);
  EXPECT_EQ(before->captures[2]->index, 2);
  const std::optional<quality::SessionSummary> summary = worker->summary();
  ASSERT_TRUE(summary.has_value());
  EXPECT_EQ(static_cast<std::size_t>(summary->kept), after->captures.size());
  EXPECT_EQ(summary->dropped, 1);

  EXPECT_FALSE(dropped.expired());  // the earlier snapshot holds it
  before.reset();
  EXPECT_TRUE(dropped.expired());

  ASSERT_TRUE(submit(*worker, request(2, 0.0, 3)));
  worker->flush();
  ASSERT_EQ(worker->kept()->captures.size(), 3u);
  EXPECT_EQ(worker->kept()->captures.back()->index, 3);
}

// A capture the writer fails to write is never kept; the next one is.
TEST(AppCapture, KeepsNoCaptureWhoseWriteFailed) {
  if (::geteuid() == 0) GTEST_SKIP() << "root writes in read-only directories";
  ScratchDir root;
  std::unique_ptr<CaptureWorker> worker = make(setup(root.path(), 2));
  ASSERT_NE(worker, nullptr);
  ASSERT_TRUE(submit(*worker, request(2, 0.0, 0)));
  worker->flush();
  const fs::path images = fs::path(worker->directory()) / "images" / "SYN1";
  fs::permissions(images, fs::perms::owner_read | fs::perms::owner_exec);
  ASSERT_TRUE(submit(*worker, request(2, 0.0, 1)));
  worker->flush();
  fs::permissions(images, fs::perms::owner_all);
  ASSERT_TRUE(submit(*worker, request(2, 0.0, 2)));
  worker->flush();

  const std::vector<CaptureEvent> events = worker->take_events();
  ASSERT_EQ(events.size(), 3u);
  EXPECT_EQ(events[0].kind, CaptureEvent::Kind::Saved);
  EXPECT_EQ(events[1].kind, CaptureEvent::Kind::Failed);
  EXPECT_EQ(events[2].kind, CaptureEvent::Kind::Saved);
  EXPECT_EQ(events[2].meta.index, 2);  // the failed one's index is spent
  const std::shared_ptr<const KeptCaptures> kept = worker->kept();
  ASSERT_NE(kept, nullptr);
  ASSERT_EQ(kept->captures.size(), 2u);
  EXPECT_EQ(kept->captures[0]->index, 0);
  EXPECT_EQ(kept->captures[1]->index, 2);
  EXPECT_EQ(worker->summary()->kept, 2);
}

// Captures kept and pending add up to at most what one Calibrate adjusts: a
// capture beyond is refused, named, with nothing queued, until a drop.
TEST(AppCapture, RefusesACaptureBeyondWhatOneCalibrateAdjusts) {
  EXPECT_EQ(CaptureSetup{}.max_kept, solve::BundleAdjustOptions{}.max_captures);
  ScratchDir root;
  CaptureSetup s = setup(root.path(), 1);
  s.max_kept = 2;
  std::unique_ptr<CaptureWorker> worker = make(std::move(s));
  ASSERT_NE(worker, nullptr);
  const auto refused = [&worker](int tag) {
    const core::Result<bool> queued = worker->submit(request(1, 0.0, tag));
    ASSERT_FALSE(queued.ok());
    EXPECT_EQ(queued.status().domain(), Code::Unsupported);
    EXPECT_EQ(queued.status().message(),
              "capture: drop a capture first: one Calibrate adjusts at most 2");
  };
  ASSERT_TRUE(submit(*worker, request(1, 0.0, 0)));
  ASSERT_TRUE(submit(*worker, request(1, 0.0, 1)));
  refused(2);  // kept and pending, however far the two have got
  worker->flush();
  refused(3);  // both kept
  EXPECT_EQ(worker->pending(), 0u);
  std::vector<CaptureEvent> events = worker->take_events();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[1].kind, CaptureEvent::Kind::Saved);
  EXPECT_EQ(worker->kept()->captures.size(), 2u);
  const json session =
      json::parse(read(fs::path(worker->directory()) / "session.json"));
  EXPECT_EQ(session["captures"].size(), 2u);

  ASSERT_TRUE(worker->drop_last().ok());
  worker->flush();
  ASSERT_TRUE(submit(*worker, request(1, 0.0, 4)));
  worker->flush();
  events = worker->take_events();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[1].kind, CaptureEvent::Kind::Saved);
  ASSERT_EQ(worker->kept()->captures.size(), 2u);
  EXPECT_EQ(worker->kept()->captures[1]->index, 2);
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

// Each checkout's commit as the build read it, "-dirty" with uncommitted
// changes; calib's is the one app_capture.software_commit expects
// (VC_EXPECT_CALIB_COMMIT), read from git when the tests run.
TEST(AppCapture, RecordsTheSoftwareItWasBuiltFrom) {
  const capture::Software software = app_software();
  EXPECT_EQ(software.opencv, CV_VERSION);
  const std::regex commit("([0-9a-f]{40}(-dirty)?)?");
  for (const std::string* sha :
       {&software.calib, &software.core, &software.recon, &software.gfx}) {
    EXPECT_TRUE(std::regex_match(*sha, commit)) << *sha;
  }
  if (const char* expected = std::getenv("VC_EXPECT_CALIB_COMMIT")) {
    EXPECT_EQ(software.calib, expected);
  }
  EXPECT_TRUE(
      std::regex_match(software.orbbec_sdk, std::regex(R"(\d+\.\d+\.\d+)")))
      << software.orbbec_sdk;
}

}  // namespace
}  // namespace volumetric_kit::calib::app
