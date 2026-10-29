// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's live worker on a device: fake cameras push synthetic NV12 board
// pictures in device images, with depth and a JPEG, through sensor's
// array; the worker reads each set's luma back, finds the board and checks
// each view, and publishes a snapshot whose corners sit within 0.1 px of where
// the board projects, for 1, 3 and 4 cameras. Then a capture: the request the
// rule takes holds every camera's luma, JPEG, depth, times and live corners;
// a full writer refuses it; the board locks. What the rule is told of a set:
// a camera missing, the frames' spread; a taken set the worker cannot copy
// out. The window takes the newest snapshot, once, and the set sink every set
// read back; while the window shows no image, every set, with nothing read
// back or searched unless a capture is armed. Then the thread's end: a sensor
// that throws, or a camera disconnected, ends it, saying why, and stopping
// releases every frame. Runs under the validation layer when the environment
// asks (VKC_TEST_VALIDATION); skips without a Vulkan device.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "device_picture.hpp"
#include "fake_sensor.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/app/preset.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = sensor::camera;
using Code = core::Status::Code;

constexpr std::uint64_t kPeriodNs = 40'000'000;  // 25 fps
constexpr std::uint64_t kStartNs = 1'760'000'000'000'000'000ull;
constexpr std::uint32_t kWidth = 1280;
constexpr std::uint32_t kHeight = 720;
constexpr auto kPatience = std::chrono::seconds(30);

// The rig's 4K colour lens at a third of its size, as the test's pictures.
camera::CameraModel live_lens() {
  camera::CameraModel lens = test::color_like_lens();
  lens.size = {kWidth, kHeight};
  lens.intrinsics.fx /= 3.0;
  lens.intrinsics.fy /= 3.0;
  lens.intrinsics.cx /= 3.0;
  lens.intrinsics.cy /= 3.0;
  return lens;
}

// Where camera `i` sees the board: each its own distance, tilt and roll.
camera::Mat4d board_pose(const compute::Board& board, std::size_t i) {
  const double k = static_cast<double>(i);
  return test::board_facing_camera(board,
                                   {0.03 * k - 0.04, 0.02, 1.3 + 0.1 * k},
                                   12.0 + 6.0 * k, 7.0 * k - 8.0);
}

// Camera `i`'s board, rendered once for every test.
const cv::Mat& board_render(std::size_t i) {
  static std::mutex mutex;
  static std::vector<cv::Mat> renders;
  const std::lock_guard<std::mutex> lock(mutex);
  if (renders.size() <= i) renders.resize(i + 1);
  if (renders[i].empty()) {
    const compute::Board board = test::femto_board();
    renders[i] = test::render_board(live_lens(), board_pose(board, i), board);
  }
  return renders[i];
}

// What a frame holds on the host: its depth and its JPEG.
struct HostPixels {
  std::vector<std::uint16_t> depth;
  std::vector<std::uint8_t> jpeg;
};

// How a push departs from the usual frames.
struct Push {
  std::vector<std::size_t> missing;         // cameras that send nothing
  std::optional<std::size_t> without_jpeg;  // a camera that sends no JPEG
  std::vector<std::int64_t> skew_ns;        // added to each camera's time
  std::shared_ptr<const void> held;         // held by camera 0's frame too
};

// One fake camera, and what it shows.
struct Camera {
  test::FakeSensor* fake = nullptr;
  sensor::YuvImage picture;
  std::vector<std::uint8_t> luma;
  std::vector<cv::Point2d> truth;    // each corner's pixel, by id; none blank
  std::weak_ptr<const void> pixels;  // the last frame's host data
};

// Collects the captures the worker hands it.
class CollectSink final : public CaptureSink {
 public:
  bool full() const override { return report_full; }
  core::Result<bool> submit(CaptureRequest request) override {
    const std::lock_guard<std::mutex> lock(mutex);
    if (!accept) return false;
    requests.push_back(std::move(request));
    return true;
  }
  std::vector<CaptureRequest> taken() {
    const std::lock_guard<std::mutex> lock(mutex);
    return requests;
  }

  bool report_full = false;
  bool accept = true;

 private:
  std::mutex mutex;
  std::vector<CaptureRequest> requests;
};

class AppLive : public core::test::VulkanDeviceTest {
 protected:
  // A rig of `n` cameras, camera `i` seeing the board at board_pose(i), or
  // a blank wall where `blank` says.
  void make_rig(std::size_t n, const std::vector<std::size_t>& blank = {}) {
    const compute::Board board = test::femto_board();
    cameras_.assign(n, Camera{});
    sensors_.clear();
    for (std::size_t i = 0; i < n; ++i) {
      Camera& cam = cameras_[i];
      const bool empty =
          std::find(blank.begin(), blank.end(), i) != blank.end();
      if (empty) {
        cam.luma.assign(static_cast<std::size_t>(kWidth) * kHeight, 110);
      } else {
        const cv::Mat& render = board_render(i);
        cam.luma.assign(render.data, render.data + render.total());
        cam.truth = test::project(live_lens(), board_pose(board, i),
                                  board.object_points());
      }
      cam.picture =
          test::device_nv12(device(), allocator(), kWidth, kHeight, cam.luma);
      ASSERT_NE(cam.picture.image[0], nullptr);
      ASSERT_NE(cam.picture.image[1], nullptr);
      sensor::SensorInfo info;
      info.id = "SYN" + std::to_string(i);
      info.model = "fake";
      info.color = live_lens();
      info.depth = test::depth_like_lens();
      info.role =
          i == 0 ? sensor::SyncRole::Primary : sensor::SyncRole::Secondary;
      info.clock = sensor::ClockDomain::Host;
      info.fps = 25;
      auto fake = std::make_unique<test::FakeSensor>(info);
      cam.fake = fake.get();
      sensors_.push_back(std::move(fake));
    }
  }

  // The worker over the rig.
  std::unique_ptr<LiveWorker> start(CaptureSink* sink = nullptr,
                                    SetSink* sets = nullptr) {
    core::Result<sensor::SensorArray> array =
        sensor::SensorArray::open(std::move(sensors_), {});
    EXPECT_TRUE(array.ok()) << array.status().message();
    if (!array.ok()) return nullptr;
    core::Result<std::unique_ptr<LiveWorker>> worker =
        LiveWorker::create(std::move(array).value(), device(), allocator(),
                           legacy_femto_board(), false, sink, sets);
    EXPECT_TRUE(worker.ok()) << worker.status().message();
    return worker.ok() ? std::move(worker).value() : nullptr;
  }

  // One frame from every camera, of trigger `tick`, as `how` says: every
  // frame made first, then all pushed at once, since the array hands a
  // trigger out without a camera silent for 50 ms of the host's clock.
  void push(std::uint64_t tick, const Push& how = {}) {
    std::vector<std::pair<std::size_t, sensor::RgbdFrame>> frames;
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
      if (std::find(how.missing.begin(), how.missing.end(), i) !=
          how.missing.end()) {
        continue;
      }
      Camera& cam = cameras_[i];
      auto host = std::make_shared<HostPixels>();
      host->depth.resize(640 * 576);
      for (std::size_t p = 0; p < host->depth.size(); ++p) {
        host->depth[p] = static_cast<std::uint16_t>(1000 + i * 7 + p % 13);
      }
      host->jpeg = {0xFF,
                    0xD8,
                    0xFF,
                    0xE0,
                    static_cast<std::uint8_t>(i),
                    static_cast<std::uint8_t>(tick),
                    0xFF,
                    0xD9};
      sensor::RgbdFrame frame;
      frame.depth = host->depth.data();
      frame.metres_per_unit = 0.001f;
      frame.depth_camera = test::depth_like_lens();
      frame.min_depth = 0.5f;
      frame.max_depth = 3.86f;
      frame.color = cam.picture;
      frame.color_camera = live_lens();
      if (how.without_jpeg != i) {
        frame.color_bitstream = {sensor::ColorBitstream::Codec::Jpeg,
                                 host->jpeg.data(), host->jpeg.size()};
      }
      const std::int64_t skew = i < how.skew_ns.size() ? how.skew_ns[i] : 0;
      frame.timestamp_ns = static_cast<std::uint64_t>(
          static_cast<std::int64_t>(kStartNs + tick * kPeriodNs) + skew);
      frame.sequence = 100 + tick;
      cam.pixels = host;
      if (i == 0 && how.held) {
        frame.pixels = std::make_shared<std::pair<std::shared_ptr<HostPixels>,
                                                  std::shared_ptr<const void>>>(
            std::move(host), how.held);
      } else {
        frame.pixels = std::move(host);
      }
      frames.emplace_back(i, std::move(frame));
    }
    for (auto& [i, frame] : frames) cameras_[i].fake->push(std::move(frame));
  }

  // Waits until the array has drained every frame pushed: the worker polled
  // them, and a complete set went out at that poll.
  void wait_drained() {
    const auto give_up = std::chrono::steady_clock::now() + kPatience;
    for (const Camera& cam : cameras_) {
      while (cam.fake->queued() != 0) {
        ASSERT_LT(std::chrono::steady_clock::now(), give_up);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  // The next snapshot after number `after`; what the worker reported on the
  // way is a failure, or kept in `errors` when it is given.
  static std::shared_ptr<const LiveSnapshot> next(
      LiveWorker& live, std::uint64_t after,
      std::vector<core::Status>* errors = nullptr) {
    const auto give_up = std::chrono::steady_clock::now() + kPatience;
    while (std::chrono::steady_clock::now() < give_up) {
      for (core::Status& error : live.take_errors()) {
        if (errors != nullptr) {
          errors->push_back(std::move(error));
        } else {
          ADD_FAILURE() << error.message();
        }
      }
      std::shared_ptr<const LiveSnapshot> s = live.take_snapshot();
      if (s && s->number > after) return s;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
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

  std::vector<Camera> cameras_;
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors_;
};

// Each camera's live corners lie within 0.1 px of where the board projects,
// in the camera's own tile of the snapshot, for 1, 3 and 4 cameras.
TEST_F(AppLive, FindsEachCamerasBoardWhereItProjects) {
  for (const std::size_t n : {1u, 3u, 4u}) {
    SCOPED_TRACE(std::to_string(n) + " cameras");
    make_rig(n);
    std::unique_ptr<LiveWorker> live = start();
    ASSERT_NE(live, nullptr);
    ASSERT_EQ(live->ids().size(), n);
    push(1);
    const std::shared_ptr<const LiveSnapshot> s = next(*live, 0);
    ASSERT_NE(s, nullptr);
    ASSERT_NE(s->set, nullptr);
    ASSERT_EQ(s->set->frames.size(), n);
    ASSERT_EQ(s->detections.size(), n);
    EXPECT_EQ(s->set->timestamp_ns, kStartNs + kPeriodNs);
    for (std::size_t i = 0; i < n; ++i) {
      SCOPED_TRACE("camera " + std::to_string(i));
      EXPECT_TRUE(s->errors[i].ok()) << s->errors[i].message();
      const Detection& found = s->detections[i];
      EXPECT_EQ(found.image_size.width, kWidth);
      EXPECT_EQ(found.image_size.height, kHeight);
      ASSERT_EQ(found.observation.points.size(), 8u);
      for (const DetectedPoint& point : found.observation.points) {
        const cv::Point2d& truth =
            cameras_[i].truth[static_cast<std::size_t>(point.id)];
        EXPECT_LT(std::hypot(point.pixel.x - truth.x, point.pixel.y - truth.y),
                  0.1)
            << "corner " << point.id;
      }
      EXPECT_TRUE(is_usable(s->checks[i].verdict));
      EXPECT_EQ(s->checks[i].corners, 8);
      EXPECT_FALSE(s->lost[i].has_value());  // not an Orbbec camera
    }
    EXPECT_EQ(s->decision.state, quality::CaptureState::Idle);
    EXPECT_EQ(s->stats.sets, 1u);
    std::vector<std::string> stages;
    for (const core::StageRow& row : s->stages.rows()) {
      stages.emplace_back(row.name);
    }
    EXPECT_EQ(stages, (std::vector<std::string>{"poll", "readback", "detect"}));

    // The next set: every corner where it was, so each is still.
    push(2);
    const std::shared_ptr<const LiveSnapshot> again = next(*live, s->number);
    ASSERT_NE(again, nullptr);
    for (std::size_t i = 0; i < n; ++i) {
      ASSERT_TRUE(again->stillness_px[i].has_value());
      EXPECT_EQ(*again->stillness_px[i], 0.0);
    }
    EXPECT_EQ(again->number, s->number + 1);
  }
}

// A camera that sees no board has an empty detection and no verdict, and no
// error; the others are found.
TEST_F(AppLive, ACameraWithoutTheBoardLeavesTheOthersFound) {
  make_rig(3, {1});
  std::unique_ptr<LiveWorker> live = start();
  ASSERT_NE(live, nullptr);
  push(1);
  const std::shared_ptr<const LiveSnapshot> s = next(*live, 0);
  ASSERT_NE(s, nullptr);
  EXPECT_TRUE(s->detections[1].observation.points.empty());
  EXPECT_EQ(s->checks[1].verdict, ViewVerdict::None);
  EXPECT_TRUE(s->errors[1].ok());
  EXPECT_EQ(s->detections[0].observation.points.size(), 8u);
  EXPECT_EQ(s->detections[2].observation.points.size(), 8u);
}

// A press arms the rule; the next still set is taken whole: every camera's
// luma as read back, JPEG, depth, times and live corners, and the set's
// facts. The board then locks.
TEST_F(AppLive, ACaptureCopiesEveryCamerasFrameOutOfItsSet) {
  make_rig(3);
  CollectSink sink;
  std::unique_ptr<LiveWorker> live = start(&sink);
  ASSERT_NE(live, nullptr);
  push(1);
  const std::shared_ptr<const LiveSnapshot> first = next(*live, 0);
  ASSERT_NE(first, nullptr);
  EXPECT_TRUE(sink.taken().empty());
  EXPECT_TRUE(live->set_board(legacy_femto_board(), true).ok());

  ASSERT_TRUE(live->arm());
  wait_armed(*live);
  push(2);
  const std::shared_ptr<const LiveSnapshot> s = next(*live, first->number);
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->decision.state, quality::CaptureState::Take);
  const std::vector<CaptureRequest> taken = sink.taken();
  ASSERT_EQ(taken.size(), 1u);
  EXPECT_EQ(live->captures(), 1u);
  const CaptureRequest& request = taken[0];

  EXPECT_EQ(request.meta.trigger_ns, kStartNs + 2 * kPeriodNs);
  EXPECT_EQ(request.meta.sequence, 102u);
  EXPECT_EQ(request.meta.spread_ms, 0.0);
  EXPECT_EQ(request.meta.stillness_px, 0.0);
  EXPECT_FALSE(request.meta.live_vs_full_px.has_value());
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
  EXPECT_LE(request.meta.time_ns, now_ns);
  EXPECT_GT(request.meta.time_ns, now_ns - std::int64_t{60'000'000'000});
  EXPECT_EQ(request.board.rows, legacy_femto_board().rows);
  EXPECT_TRUE(request.square_measured);  // the board set before the capture

  ASSERT_EQ(request.cameras.size(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    SCOPED_TRACE("camera " + std::to_string(i));
    const CameraFrameCopy& part = request.cameras[i];
    EXPECT_EQ(part.width, kWidth);
    EXPECT_EQ(part.height, kHeight);
    EXPECT_TRUE(part.luma == cameras_[i].luma);
    EXPECT_EQ(part.jpeg, (std::vector<std::uint8_t>{
                             0xFF, 0xD8, 0xFF, 0xE0,
                             static_cast<std::uint8_t>(i), 2, 0xFF, 0xD9}));
    ASSERT_EQ(part.depth.size(), 640u * 576u);
    EXPECT_EQ(part.metres_per_unit, 0.001f);
    EXPECT_EQ(part.depth[0], 1000 + i * 7);
    EXPECT_EQ(part.depth[640 * 576 - 1], 1000 + i * 7 + (640 * 576 - 1) % 13);
    EXPECT_EQ(part.timestamp_ns, kStartNs + 2 * kPeriodNs);
    EXPECT_EQ(part.sequence, 102u);
    ASSERT_EQ(part.live.observation.points.size(),
              s->detections[i].observation.points.size());
    for (std::size_t p = 0; p < part.live.observation.points.size(); ++p) {
      EXPECT_EQ(part.live.observation.points[p].pixel.x,
                s->detections[i].observation.points[p].pixel.x);
    }
  }
  EXPECT_EQ(live->decision().state, quality::CaptureState::Take);

  // The board is locked now.
  const core::Status locked = live->set_board(legacy_femto_board(), false);
  EXPECT_EQ(locked.domain(), Code::InvalidArgument);
  EXPECT_NE(locked.message().find("locked"), std::string::npos);
  EXPECT_TRUE(live->square_measured());
}

// A full writer: the rule refuses the set at once, and so does the worker
// when the sink refuses what the rule took.
TEST_F(AppLive, AFullWriterRefusesTheCapture) {
  for (const bool report_full : {true, false}) {
    SCOPED_TRACE(report_full ? "full before" : "refused at submit");
    make_rig(1);
    CollectSink sink;
    sink.report_full = report_full;
    // Full before: the sink would take the capture if asked, so only the rule
    // keeps it out. Refused at submit: the sink says it has room, then
    // refuses.
    sink.accept = report_full;
    std::unique_ptr<LiveWorker> live = start(&sink);
    ASSERT_NE(live, nullptr);
    push(1);
    const std::shared_ptr<const LiveSnapshot> first = next(*live, 0);
    ASSERT_NE(first, nullptr);
    ASSERT_TRUE(live->arm());
    wait_armed(*live);
    push(2);
    const std::shared_ptr<const LiveSnapshot> s = next(*live, first->number);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->decision.state, quality::CaptureState::Failed);
    EXPECT_EQ(s->decision.hold, quality::Hold::SaverFull);
    EXPECT_TRUE(sink.taken().empty());
    EXPECT_EQ(live->captures(), 0u);
    EXPECT_TRUE(live->set_board(legacy_femto_board(), false).ok());
  }
}

TEST_F(AppLive, CapturesNothingWithoutASinkAndRefusesABadBoard) {
  make_rig(1);
  std::unique_ptr<LiveWorker> live = start();
  ASSERT_NE(live, nullptr);
  EXPECT_FALSE(live->arm());
  Target bad = legacy_femto_board();
  bad.rows = 0;
  EXPECT_EQ(live->set_board(bad, false).domain(), Code::InvalidArgument);
  EXPECT_EQ(live->board().rows, legacy_femto_board().rows);
}

// A sensor without a colour model is refused before the thread starts.
TEST_F(AppLive, RefusesACameraWithoutColour) {
  sensor::SensorInfo info;
  info.id = "NOCOLOR";
  info.fps = 25;
  sensors_.clear();
  sensors_.push_back(std::make_unique<test::FakeSensor>(info));
  core::Result<sensor::SensorArray> array =
      sensor::SensorArray::open(std::move(sensors_), {});
  ASSERT_TRUE(array.ok()) << array.status().message();
  const core::Result<std::unique_ptr<LiveWorker>> live =
      LiveWorker::create(std::move(array).value(), device(), allocator(),
                         legacy_femto_board(), false, nullptr);
  ASSERT_FALSE(live.ok());
  EXPECT_EQ(live.status().domain(), Code::InvalidArgument);
  EXPECT_NE(live.status().message().find("NOCOLOR has no colour model"),
            std::string::npos);
}

// What the thread throws ends it, posting why, with the rig stopped.
TEST_F(AppLive, AThrowingSensorEndsTheThreadWithAStatus) {
  make_rig(3);
  test::FakeSensor* thrower = cameras_[2].fake;
  CollectSink sink;
  std::unique_ptr<LiveWorker> live = start(&sink);
  ASSERT_NE(live, nullptr);
  thrower->throw_on_drain();
  const auto give_up = std::chrono::steady_clock::now() + kPatience;
  while (live->running()) {
    ASSERT_LT(std::chrono::steady_clock::now(), give_up);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const std::vector<core::Status> errors = live->take_errors();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].message(), "live view: the driver threw");
  EXPECT_FALSE(thrower->started());
  EXPECT_GE(thrower->stops(), 1);
  EXPECT_FALSE(live->arm());  // nothing left to take a capture
}

// A camera disconnected fails every poll after it, as sensor's Orbbec driver
// does: the thread ends, saying why once, with the rig stopped.
TEST_F(AppLive, ACameraDisconnectedEndsTheThread) {
  make_rig(3);
  test::FakeSensor* gone = cameras_[2].fake;
  CollectSink sink;
  std::unique_ptr<LiveWorker> live = start(&sink);
  ASSERT_NE(live, nullptr);
  push(1);
  ASSERT_NE(next(*live, 0), nullptr);
  gone->disconnect();
  const auto give_up = std::chrono::steady_clock::now() + kPatience;
  while (live->running()) {
    ASSERT_LT(std::chrono::steady_clock::now(), give_up);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const std::vector<core::Status> errors = live->take_errors();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].domain(), Code::IoError);
  EXPECT_EQ(errors[0].message(),
            "camera SYN2 disconnected; reopen it to use it again");
  EXPECT_FALSE(cameras_[0].fake->started());
  EXPECT_FALSE(live->arm());  // nothing left to take a capture
}

// The window takes the newest snapshot, once: those published while it was
// not looking are let go, frames and all.
TEST_F(AppLive, TheNewestSnapshotWinsAndIsTakenOnce) {
  make_rig(2);
  std::unique_ptr<LiveWorker> live = start();
  ASSERT_NE(live, nullptr);
  std::vector<std::weak_ptr<const void>> frames;  // camera 0's, by set
  for (std::uint64_t tick = 1; tick <= 3; ++tick) {
    push(tick);
    frames.push_back(cameras_[0].pixels);
    wait_drained();  // its set went out; the next waits for the next poll
  }
  // Set 3's snapshot replaces set 2's, never taken.
  const auto give_up = std::chrono::steady_clock::now() + kPatience;
  while (!frames[1].expired()) {
    ASSERT_LT(std::chrono::steady_clock::now(), give_up);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_TRUE(frames[0].expired());
  const std::shared_ptr<const LiveSnapshot> s = live->take_snapshot();
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->number, 3u);
  EXPECT_EQ(s->set->timestamp_ns, kStartNs + 3 * kPeriodNs);
  EXPECT_FALSE(frames[2].expired());
  EXPECT_EQ(live->take_snapshot(), nullptr);  // taken once
  EXPECT_TRUE(live->take_errors().empty());
}

// A snapshot replaced before the window took it is let go after the worker's
// lock: letting its frames go holds up none of the window's reads.
TEST_F(AppLive, AReplacedSnapshotIsLetGoOutsideTheLock) {
  // Before the worker, to outlive it: set 1's frame, let go on the worker's
  // thread, says so and waits until the test has read the worker.
  std::promise<void> releasing;
  std::promise<void> read;
  Push probed;
  probed.held = std::shared_ptr<const void>(
      &probed, [&releasing, done = read.get_future().share()](const void*) {
        releasing.set_value();
        (void)done.wait_for(kPatience);
      });
  make_rig(2);
  std::unique_ptr<LiveWorker> live = start();
  ASSERT_NE(live, nullptr);
  push(1, probed);
  probed.held.reset();  // the frame holds the probe alone
  wait_drained();
  push(2);  // its snapshot replaces set 1's, never taken
  ASSERT_EQ(releasing.get_future().wait_for(kPatience),
            std::future_status::ready);
  std::future<void> reader =
      std::async(std::launch::async, [&] { (void)live->decision(); });
  const bool unlocked =
      reader.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
  read.set_value();  // the release goes on
  reader.wait();
  EXPECT_TRUE(unlocked) << "the worker let set 1 go under its lock";
  const std::shared_ptr<const LiveSnapshot> s = live->take_snapshot();
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->number, 2u);  // set 1's was published, then replaced
}

// Keeps every set the worker offers it.
class KeepSets final : public SetSink {
 public:
  void offer(std::shared_ptr<const sensor::FrameSet> set) override {
    const std::lock_guard<std::mutex> lock(mutex);
    sets.push_back(std::move(set));
  }
  std::vector<std::shared_ptr<const sensor::FrameSet>> taken() {
    const std::lock_guard<std::mutex> lock(mutex);
    return sets;
  }
  // Let every set go, as the worker asks of its set sink before it closes
  // the sensors.
  void release() {
    const std::lock_guard<std::mutex> lock(mutex);
    sets.clear();
  }

 private:
  std::mutex mutex;
  std::vector<std::shared_ptr<const sensor::FrameSet>> sets;
};

// Each set the worker reads back is offered to its set sink, the very set its
// snapshot holds, for the 3D view's worker to prepare.
TEST_F(AppLive, OffersEachSetItReadsBackToItsSetSink) {
  make_rig(3);
  KeepSets sets;
  std::unique_ptr<LiveWorker> live = start(nullptr, &sets);
  ASSERT_NE(live, nullptr);
  push(1);
  const std::shared_ptr<const LiveSnapshot> first = next(*live, 0);
  ASSERT_NE(first, nullptr);
  push(2);
  const std::shared_ptr<const LiveSnapshot> second = next(*live, first->number);
  ASSERT_NE(second, nullptr);
  const std::vector<std::shared_ptr<const sensor::FrameSet>> taken =
      sets.taken();
  ASSERT_EQ(taken.size(), 2u);
  EXPECT_EQ(taken[0], first->set);
  EXPECT_EQ(taken[1], second->set);
  EXPECT_EQ(taken[1]->frames.size(), 3u);
  live->stop();
  sets.release();
}

// Not detecting, as while the window shows the 3D view alone, each set is
// offered and published with the rig's counters, but nothing is read back,
// detected or ruled on. Detecting again, the next set is searched, with no
// stillness against the set searched before the pause; and a capture armed
// is searched for, and taken, with detection off.
TEST_F(AppLive, SearchesNoSetWhileNotDetectingUnlessACaptureIsArmed) {
  const auto stages_of = [](const LiveSnapshot& s) {
    std::vector<std::string> names;
    for (const core::StageRow& row : s.stages.rows()) {
      names.emplace_back(row.name);
    }
    return names;
  };
  const std::vector<std::string> searched = {"poll", "readback", "detect"};
  make_rig(2);
  CollectSink sink;
  KeepSets sets;
  std::unique_ptr<LiveWorker> live = start(&sink, &sets);
  ASSERT_NE(live, nullptr);
  push(1);
  const std::shared_ptr<const LiveSnapshot> first = next(*live, 0);
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(stages_of(*first), searched);

  live->set_detecting(false);
  push(2);
  const std::shared_ptr<const LiveSnapshot> off = next(*live, first->number);
  ASSERT_NE(off, nullptr);
  EXPECT_EQ(stages_of(*off), std::vector<std::string>{"poll"});
  ASSERT_EQ(off->detections.size(), 2u);
  ASSERT_EQ(off->checks.size(), 2u);
  ASSERT_EQ(off->errors.size(), 2u);
  ASSERT_EQ(off->stillness_px.size(), 2u);
  for (std::size_t i = 0; i < 2; ++i) {
    EXPECT_TRUE(off->detections[i].observation.points.empty());
    EXPECT_EQ(off->checks[i].verdict, ViewVerdict::None);
    EXPECT_TRUE(off->errors[i].ok());
    EXPECT_FALSE(off->stillness_px[i].has_value());
  }
  EXPECT_EQ(off->decision.state, quality::CaptureState::Idle);
  EXPECT_EQ(off->stats.sets, 2u);
  ASSERT_EQ(sets.taken().size(), 2u);
  EXPECT_EQ(sets.taken()[1], off->set);

  live->set_detecting(true);
  push(3);
  const std::shared_ptr<const LiveSnapshot> on = next(*live, off->number);
  ASSERT_NE(on, nullptr);
  EXPECT_EQ(stages_of(*on), searched);
  for (std::size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(on->detections[i].observation.points.size(), 8u);
    EXPECT_FALSE(on->stillness_px[i].has_value());
  }

  ASSERT_TRUE(live->arm());
  wait_armed(*live);
  live->set_detecting(false);
  push(4);
  const std::shared_ptr<const LiveSnapshot> armed = next(*live, on->number);
  ASSERT_NE(armed, nullptr);
  EXPECT_EQ(stages_of(*armed), searched);
  EXPECT_EQ(armed->decision.state, quality::CaptureState::Take);
  EXPECT_EQ(sink.taken().size(), 1u);
  // The take disarmed the rule: the next set is not searched.
  push(5);
  const std::shared_ptr<const LiveSnapshot> after = next(*live, armed->number);
  ASSERT_NE(after, nullptr);
  EXPECT_EQ(stages_of(*after), std::vector<std::string>{"poll"});
  EXPECT_EQ(sets.taken().size(), 5u);
  live->stop();
  sets.release();
}

// Armed, a set missing a camera is not taken: the rule waits, naming the
// camera, and nothing reaches the sink.
TEST_F(AppLive, ASetMissingACameraIsNotTaken) {
  make_rig(3);
  CollectSink sink;
  std::unique_ptr<LiveWorker> live = start(&sink);
  ASSERT_NE(live, nullptr);
  push(1);
  const std::shared_ptr<const LiveSnapshot> first = next(*live, 0);
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(live->arm());
  wait_armed(*live);
  Push without;
  without.missing = {2};
  push(2, without);
  const std::shared_ptr<const LiveSnapshot> s = next(*live, first->number);
  ASSERT_NE(s, nullptr);
  EXPECT_FALSE(s->set->frames[2].has_value());
  EXPECT_EQ(s->decision.state, quality::CaptureState::Waiting);
  EXPECT_EQ(s->decision.hold, quality::Hold::Incomplete);
  EXPECT_NE(s->decision.message.find("SYN2 missing"), std::string::npos)
      << s->decision.message;
  EXPECT_TRUE(sink.taken().empty());
  EXPECT_EQ(live->captures(), 0u);
}

// A set the rule takes that the worker cannot copy out -- a camera sent no
// JPEG -- fails the capture, saying why, and nothing reaches the sink.
TEST_F(AppLive, ATakenSetWithoutAJpegFailsTheCapture) {
  make_rig(2);
  CollectSink sink;
  std::unique_ptr<LiveWorker> live = start(&sink);
  ASSERT_NE(live, nullptr);
  push(1);
  const std::shared_ptr<const LiveSnapshot> first = next(*live, 0);
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(live->arm());
  wait_armed(*live);
  Push no_jpeg;
  no_jpeg.without_jpeg = 1;
  push(2, no_jpeg);
  std::vector<core::Status> errors;
  const std::shared_ptr<const LiveSnapshot> s =
      next(*live, first->number, &errors);
  ASSERT_NE(s, nullptr);
  for (core::Status& error : live->take_errors()) {
    errors.push_back(std::move(error));
  }
  EXPECT_EQ(s->decision.state, quality::CaptureState::Failed);
  EXPECT_EQ(s->decision.hold, quality::Hold::None);
  EXPECT_EQ(s->decision.message, "camera SYN1 sent no JPEG with its frame");
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].message(),
            "capture: camera SYN1 sent no JPEG with its frame");
  EXPECT_TRUE(sink.taken().empty());
  EXPECT_EQ(live->captures(), 0u);
  EXPECT_EQ(live->decision().state, quality::CaptureState::Failed);
}

// The rule hears the set's spread: frames 9 ms apart are taken, warned of,
// and the capture records the spread.
TEST_F(AppLive, ACaptureRecordsItsSpreadAndWarnsOfIt) {
  make_rig(3);
  CollectSink sink;
  std::unique_ptr<LiveWorker> live = start(&sink);
  ASSERT_NE(live, nullptr);
  push(1);
  const std::shared_ptr<const LiveSnapshot> first = next(*live, 0);
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(live->arm());
  wait_armed(*live);
  Push skewed;
  skewed.skew_ns = {0, 8'000'000, -1'000'000};  // 8 ms late, 1 ms early
  push(2, skewed);
  const std::shared_ptr<const LiveSnapshot> s = next(*live, first->number);
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->decision.state, quality::CaptureState::Take);
  EXPECT_TRUE(s->decision.spread_high);  // past spread_warn_ms, 5 ms
  const std::vector<CaptureRequest> taken = sink.taken();
  ASSERT_EQ(taken.size(), 1u);
  EXPECT_DOUBLE_EQ(taken[0].meta.spread_ms, 9.0);
  EXPECT_EQ(taken[0].cameras[1].timestamp_ns,
            kStartNs + 2 * kPeriodNs + 8'000'000);
}

// Stopping joins the thread and stops the rig; once the snapshots are gone
// and the worker destroyed, no frame is held.
TEST_F(AppLive, StopsCleanlyAndReleasesEveryFrame) {
  make_rig(4);
  std::unique_ptr<LiveWorker> live = start();
  ASSERT_NE(live, nullptr);
  std::vector<test::FakeSensor*> fakes;
  for (const Camera& cam : cameras_) fakes.push_back(cam.fake);
  push(1);
  std::shared_ptr<const LiveSnapshot> held = next(*live, 0);
  ASSERT_NE(held, nullptr);
  const std::weak_ptr<const void> first = held->set->frames[0]->pixels;
  push(2);  // published by the worker, never taken
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  live->stop();
  EXPECT_FALSE(live->running());
  for (test::FakeSensor* fake : fakes) {
    EXPECT_FALSE(fake->started());
    EXPECT_GE(fake->stops(), 1);
  }
  live->stop();  // idempotent
  EXPECT_TRUE(live->take_errors().empty());
  EXPECT_FALSE(first.expired());  // the window's snapshot holds it
  held.reset();
  EXPECT_TRUE(first.expired());
  live.reset();  // and with it the snapshot it published
  for (const Camera& cam : cameras_) EXPECT_TRUE(cam.pixels.expired());
}

TEST_F(AppLive, OpensOrbbecCamerasAtThePresetReadableByEachFamily) {
  RigPreset preset;
  preset.color = {1920, 1080};
  preset.depth = {320, 288};
  preset.fps = 15;
  preset.color_codec = core::config::ColorCodec::Mjpeg;
  preset.depth_mode = depth_mode(preset.depth).value();
  const sensor::OrbbecSensor::Options options =
      orbbec_options(preset, device(), allocator(), {3, 7});
  EXPECT_EQ(options.color_width, 1920u);
  EXPECT_EQ(options.color_height, 1080u);
  EXPECT_EQ(options.depth_width, 320u);
  EXPECT_EQ(options.depth_height, 288u);
  EXPECT_EQ(options.fps, 15u);
  EXPECT_EQ(options.color_codec, sensor::OrbbecColorCodec::Mjpeg);
  preset.color_codec = core::config::ColorCodec::Hevc;  // as the scenario says
  EXPECT_EQ(orbbec_options(preset, device(), allocator(), {3}).color_codec,
            sensor::OrbbecColorCodec::Hevc);
  preset.color_codec = core::config::ColorCodec::Mjpeg;
  EXPECT_EQ(options.min_depth, 0.5f);
  EXPECT_EQ(options.max_depth, 5.46f);
  EXPECT_EQ(options.device, &device());
  EXPECT_EQ(options.allocator, &allocator());
  EXPECT_EQ(options.color_queue_family_count, 2u);
  EXPECT_EQ(options.color_queue_families[0], 3u);
  EXPECT_EQ(options.color_queue_families[1], 7u);
  EXPECT_TRUE(options.serial.empty());
  EXPECT_FALSE(options.sync.has_value());
  EXPECT_FALSE(options.apply_sync);
  // No colour controls: each camera opens at auto exposure, which the
  // driver writes where the camera differs.
  EXPECT_FALSE(options.color_controls.has_value());
  // More families than the options hold: kept to the array, the count left
  // for the camera's open to refuse.
  const sensor::OrbbecSensor::Options many =
      orbbec_options(preset, device(), allocator(), {1, 2, 3, 4, 5});
  EXPECT_EQ(many.color_queue_family_count, 5u);
  EXPECT_EQ(many.color_queue_families[3], 4u);
}

}  // namespace
}  // namespace volumetric_kit::calib::app
