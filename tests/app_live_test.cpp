// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's live worker on a device: fake cameras push synthetic NV12 board
// pictures in device images, with depth and a JPEG, through recon's sensor
// array; the worker reads each set's luma back, finds the board and checks
// each view, and publishes a snapshot whose corners sit within 0.1 px of where
// the board projects, for 1, 3 and 4 cameras. Then a capture: the request the
// rule takes holds every camera's luma, JPEG, depth, times and live corners;
// a full writer refuses it; the board locks. Then the thread's end: a
// sensor that throws posts why, and stopping releases every frame. Runs under
// the validation layer when the environment asks (VKC_TEST_VALIDATION); skips
// without a Vulkan device.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
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

namespace camera = recon::camera;
namespace sensor = recon::sensor;
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
  std::unique_ptr<LiveWorker> start(CaptureSink* sink = nullptr) {
    core::Result<sensor::SensorArray> array =
        sensor::SensorArray::open(std::move(sensors_), {});
    EXPECT_TRUE(array.ok()) << array.status().message();
    if (!array.ok()) return nullptr;
    core::Result<std::unique_ptr<LiveWorker>> worker =
        LiveWorker::create(std::move(array).value(), device(), allocator(),
                           legacy_femto_board(), false, sink);
    EXPECT_TRUE(worker.ok()) << worker.status().message();
    return worker.ok() ? std::move(worker).value() : nullptr;
  }

  // One frame from every camera, of trigger `tick`: every frame made first,
  // then all pushed at once, since the array hands a trigger out without a
  // camera silent for 50 ms of the host's clock.
  void push(std::uint64_t tick) {
    std::vector<sensor::RgbdFrame> frames;
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
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
      frame.color_bitstream = {sensor::ColorBitstream::Codec::Jpeg,
                               host->jpeg.data(), host->jpeg.size()};
      frame.timestamp_ns = kStartNs + tick * kPeriodNs;
      frame.sequence = 100 + tick;
      cam.pixels = host;
      frame.pixels = std::move(host);
      frames.push_back(std::move(frame));
    }
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
      cameras_[i].fake->push(std::move(frames[i]));
    }
  }

  // The next snapshot after number `after`.
  static std::shared_ptr<const LiveSnapshot> next(LiveWorker& live,
                                                  std::uint64_t after) {
    const auto give_up = std::chrono::steady_clock::now() + kPatience;
    while (std::chrono::steady_clock::now() < give_up) {
      for (const core::Status& error : live.take_errors()) {
        ADD_FAILURE() << error.message();
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
  std::unique_ptr<LiveWorker> live = start();
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
  preset.depth_mode = depth_mode(preset.depth).value();
  const sensor::OrbbecSensor::Options options =
      orbbec_options(preset, device(), allocator(), {3, 7});
  EXPECT_EQ(options.color_width, 1920u);
  EXPECT_EQ(options.color_height, 1080u);
  EXPECT_EQ(options.depth_width, 320u);
  EXPECT_EQ(options.depth_height, 288u);
  EXPECT_EQ(options.fps, 15u);
  EXPECT_EQ(options.color_codec, sensor::OrbbecColorCodec::Mjpeg);
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
  // More families than the options hold: kept to the array, the count left
  // for the camera's open to refuse.
  const sensor::OrbbecSensor::Options many =
      orbbec_options(preset, device(), allocator(), {1, 2, 3, 4, 5});
  EXPECT_EQ(many.color_queue_family_count, 5u);
  EXPECT_EQ(many.color_queue_families[3], 4u);
}

}  // namespace
}  // namespace volumetric_kit::calib::app
