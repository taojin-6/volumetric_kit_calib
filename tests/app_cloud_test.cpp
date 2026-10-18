// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's 3D view. The worker (T2) on a device: synthetic RGB-D frames,
// host depth and colour, prepared for 1, 3 and 5 cameras with the colour
// camera at the origin whatever the frame's baked pose; the newest snapshot
// wins, sampled every live_every-th set; freezing holds the window's and
// lets offers go; no set is held past its preparation; solid prepares depth
// only; a refused set arrives as a Status and the worker goes on. Then across
// two queue families, under synchronization validation: a known point,
// prepared on the compute family and drawn on the graphics family through
// gfx's depth-cloud pipeline, the frame waiting for the preparation's
// semaphore, lands on its pixel with its colour by recon's rule, the snapshot
// held until its frame retires; a frozen snapshot redraws at a new pose with
// no second preparation; a frame EXCLUSIVE to the compute family is refused,
// and with no camera placed nothing is drawn, neither holding anything. And
// the device calib_rig asks for. The GPU cases skip without a Vulkan device,
// the cross-family ones also with one family.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

#include "volumetric_kit/calib/app/calib_rig.hpp"
#include "volumetric_kit/calib/app/cloud_worker.hpp"
#include "volumetric_kit/calib/app/live_view.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/core/vulkan/command_buffer.hpp"
#include "volumetric_kit/core/vulkan/command_pool.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/core/vulkan/vk_result.hpp"
#include "volumetric_kit/gfx/core/device_requirements.hpp"
#include "volumetric_kit/gfx/core/offscreen_target.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = recon::camera;
namespace pipelines = gfx::pipelines;
namespace sensor = recon::sensor;
using camera::Mat4d;
using camera::Vec3d;
using Code = core::Status::Code;

constexpr std::uint32_t kDepthW = 16;
constexpr std::uint32_t kDepthH = 12;
constexpr std::uint32_t kColorW = 20;
constexpr std::uint32_t kColorH = 15;
constexpr std::uint32_t kU = 5;  // the depth pixel with a return
constexpr std::uint32_t kV = 4;
constexpr std::uint16_t kRaw = 1500;  // its depth: 1.5 m at 1 mm a unit
constexpr double kZ = 1.5;
constexpr auto kPatience = std::chrono::seconds(30);

Mat4d pose(double degrees, Vec3d axis, Vec3d translation) {
  return glm::translate(Mat4d(1.0), translation) *
         glm::rotate(Mat4d(1.0), glm::radians(degrees), axis);
}

// Pinhole cameras, every intrinsic distinct; no lens, so the preparation
// keeps each pixel where it was.
camera::CameraModel pinhole(std::uint32_t w, std::uint32_t h, double fx,
                            double fy, double cx, double cy) {
  camera::CameraModel model;
  model.size = {w, h};
  model.intrinsics = {fx, fy, cx, cy};
  return model;
}
camera::CameraModel depth_model() {
  return pinhole(kDepthW, kDepthH, 14.0, 13.0, 7.3, 5.6);
}
camera::CameraModel color_model() {
  return pinhole(kColorW, kColorH, 17.0, 16.0, 9.6, 7.2);
}
Mat4d depth_to_color() {
  return pose(3.0, {0.2, 1.0, 0.1}, {0.02, -0.01, 0.005});
}

// Each colour pixel's word says where it is.
std::uint32_t coded(std::uint32_t x, std::uint32_t y) {
  return pipelines::pack_rgba8(static_cast<std::uint8_t>(10 * x + 3),
                               static_cast<std::uint8_t>(10 * y + 7), 77);
}

struct HostPixels {
  std::vector<std::uint16_t> depth;
  std::vector<std::uint32_t> color;
};

// A set of `n` frames: each with no return but kRaw at (kU, kV), every
// colour pixel coded, and a baked pose the 3D view must not use. Each frame's
// pixels are watched through `watched` when it is given.
std::shared_ptr<const sensor::FrameSet> make_set(
    std::size_t n, std::uint64_t sequence,
    std::vector<std::weak_ptr<const void>>* watched = nullptr,
    float min_depth = 0.5f) {
  auto set = std::make_shared<sensor::FrameSet>();
  set->timestamp_ns = 1'000'000 * sequence;
  set->sequence = sequence;
  for (std::size_t i = 0; i < n; ++i) {
    auto host = std::make_shared<HostPixels>();
    host->depth.assign(std::size_t{kDepthW} * kDepthH, 0);
    host->depth[std::size_t{kV} * kDepthW + kU] = kRaw;
    host->color.resize(std::size_t{kColorW} * kColorH);
    for (std::uint32_t y = 0; y < kColorH; ++y) {
      for (std::uint32_t x = 0; x < kColorW; ++x) {
        host->color[std::size_t{y} * kColorW + x] = coded(x, y);
      }
    }
    sensor::RgbdFrame frame;
    frame.depth = host->depth.data();
    frame.metres_per_unit = 0.001f;
    frame.depth_camera = depth_model();
    frame.min_depth = min_depth;
    frame.max_depth = 3.86f;
    frame.color_packed = host->color.data();
    frame.color_camera = color_model();
    frame.depth_to_color = depth_to_color();
    frame.color_to_world = pose(40.0, {1.0, 0.0, 0.0}, {3.0, 2.0, 1.0});
    frame.timestamp_ns = set->timestamp_ns;
    frame.sequence = sequence;
    if (watched != nullptr) watched->push_back(host);
    frame.pixels = std::move(host);
    set->frames.emplace_back(std::move(frame));
  }
  return set;
}

bool all_gone(const std::vector<std::weak_ptr<const void>>& watched) {
  for (const std::weak_ptr<const void>& w : watched) {
    if (!w.expired()) return false;
  }
  return true;
}

// The next snapshot; what the worker reported on the way is a failure.
std::shared_ptr<const CloudSnapshot> next_snapshot(CloudWorker& cloud) {
  const auto give_up = std::chrono::steady_clock::now() + kPatience;
  while (std::chrono::steady_clock::now() < give_up) {
    for (const core::Status& error : cloud.take_errors()) {
      ADD_FAILURE() << error.message();
    }
    if (std::shared_ptr<const CloudSnapshot> s = cloud.take_snapshot()) {
      return s;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ADD_FAILURE() << "no snapshot";
  return nullptr;
}

void wait_prepared(const CloudWorker& cloud, std::uint64_t count) {
  const auto give_up = std::chrono::steady_clock::now() + kPatience;
  while (cloud.prepared() < count) {
    ASSERT_LT(std::chrono::steady_clock::now(), give_up)
        << "prepared " << count;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

std::vector<core::Status> wait_errors(CloudWorker& cloud) {
  const auto give_up = std::chrono::steady_clock::now() + kPatience;
  while (std::chrono::steady_clock::now() < give_up) {
    std::vector<core::Status> errors = cloud.take_errors();
    if (!errors.empty()) return errors;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ADD_FAILURE() << "no error";
  return {};
}

// --- The worker, on one device ----------------------------------------------

CloudOptions every_set() {
  CloudOptions options;
  options.live_every = 1;
  return options;
}

class AppCloud : public core::test::VulkanDeviceTest {
 protected:
  core::DeviceRequirements requirements() const override {
    return recon::device_requirements();
  }

  // A worker preparing every set offered, unless `options` say otherwise.
  std::unique_ptr<CloudWorker> start(std::size_t cameras,
                                     CloudOptions options = every_set()) {
    core::Result<std::unique_ptr<CloudWorker>> made = CloudWorker::create(
        device(), allocator(), cameras,
        {device().queue_family(), device().queue_family()}, options);
    EXPECT_TRUE(made.ok()) << made.status().message();
    return made.ok() ? std::move(made).value() : nullptr;
  }
};

// Each camera's depth and colour, prepared with the colour camera at the
// world's origin, the depth camera at its extrinsic: the frame's baked pose
// is not the snapshot's. For 1, 3 and 5 cameras.
TEST_F(AppCloud, PreparesEveryCameraAtTheOriginForAnyNumberOfCameras) {
  for (const std::size_t n : {1u, 3u, 5u}) {
    SCOPED_TRACE(std::to_string(n) + " cameras");
    std::unique_ptr<CloudWorker> cloud = start(n);
    ASSERT_NE(cloud, nullptr);
    cloud->offer(make_set(n, 7));
    const std::shared_ptr<const CloudSnapshot> s = next_snapshot(*cloud);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->number, 1u);
    EXPECT_EQ(s->sequence, 7u);
    EXPECT_EQ(s->timestamp_ns, 7'000'000u);
    EXPECT_TRUE(s->color);
    ASSERT_EQ(s->frames.size(), n);
    for (std::size_t i = 0; i < n; ++i) {
      ASSERT_TRUE(s->frames[i].has_value());
      const sensor::DeviceFrame& f = *s->frames[i];
      ASSERT_NE(f.depth, nullptr);
      ASSERT_NE(f.color, nullptr);
      EXPECT_NE(f.depth->usage() & VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, 0u);
      EXPECT_EQ(f.depth_camera.fx, 14.0f);
      EXPECT_EQ(f.depth_camera.width, kDepthW);
      EXPECT_EQ(f.color_camera.cy, 7.2f);
      EXPECT_EQ(f.color_camera.cam_to_world, recon::Mat4f(1.0f));
      EXPECT_EQ(f.depth_camera.cam_to_world, recon::Mat4f(depth_to_color()));
    }
    ASSERT_NE(s->ready.semaphore, nullptr);
    EXPECT_GT(s->ready.value, 0u);  // signalled, not the semaphore's start
    const core::Result<std::uint64_t> reached = s->ready.semaphore->value();
    ASSERT_TRUE(reached.ok());
    EXPECT_GE(reached.value(), s->ready.value);
    bool timed = false;
    for (const core::StageRow& row : s->stages.rows()) {
      timed = timed || std::string(row.name) == "frame prep";
    }
    EXPECT_TRUE(timed);
  }
}

// The window takes the newest snapshot, once. Offered faster than they are
// prepared, the sets are not queued: each one waiting is let go when a newer
// one is offered, so at most the set being prepared and the newest are held;
// the newest is the last prepared, none comes out of order, and each
// snapshot waits on a value of its own.
TEST_F(AppCloud, TheNewestWins) {
  std::unique_ptr<CloudWorker> cloud = start(2);
  ASSERT_NE(cloud, nullptr);
  cloud->offer(make_set(2, 1));
  wait_prepared(*cloud, 1);
  cloud->offer(make_set(2, 2));
  wait_prepared(*cloud, 2);
  std::shared_ptr<const CloudSnapshot> s = cloud->take_snapshot();
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->sequence, 2u);
  EXPECT_EQ(s->number, 2u);
  EXPECT_EQ(cloud->take_snapshot(), nullptr);

  // Each set's pixels, watched apart: four offers take microseconds, one
  // preparation milliseconds.
  std::vector<std::vector<std::weak_ptr<const void>>> burst(4);
  for (std::uint64_t seq = 3; seq <= 6; ++seq) {
    cloud->offer(make_set(2, seq, &burst[seq - 3]));
  }
  std::size_t held = 0;
  for (const std::vector<std::weak_ptr<const void>>& set : burst) {
    if (!all_gone(set)) ++held;
  }
  EXPECT_LE(held, 2u);
  std::uint64_t last = 2;
  std::uint64_t ready = s->ready.value;
  const auto give_up = std::chrono::steady_clock::now() + kPatience;
  while (last < 6 && std::chrono::steady_clock::now() < give_up) {
    s = cloud->take_snapshot();
    if (s != nullptr) {
      EXPECT_GT(s->sequence, last);
      EXPECT_GT(s->ready.value, ready);
      last = s->sequence;
      ready = s->ready.value;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(last, 6u);
  for (const std::vector<std::weak_ptr<const void>>& set : burst) {
    EXPECT_TRUE(all_gone(set));
  }
  EXPECT_TRUE(cloud->take_errors().empty());
}

// With live_every 3 the first set and every third after it are prepared;
// the others are let go as they are offered.
TEST_F(AppCloud, PreparesEveryLiveEveryThSet) {
  CloudOptions options;
  options.live_every = 3;
  std::unique_ptr<CloudWorker> cloud = start(1, options);
  ASSERT_NE(cloud, nullptr);
  std::vector<std::uint64_t> seen;
  for (std::uint64_t seq = 1; seq <= 7; ++seq) {
    std::vector<std::weak_ptr<const void>> watched;
    cloud->offer(make_set(1, seq, &watched));
    if (seq % 3 == 1) {
      const std::shared_ptr<const CloudSnapshot> s = next_snapshot(*cloud);
      ASSERT_NE(s, nullptr);
      seen.push_back(s->sequence);
    } else {
      EXPECT_TRUE(all_gone(watched)) << "set " << seq;
    }
  }
  EXPECT_EQ(seen, (std::vector<std::uint64_t>{1, 4, 7}));
  EXPECT_EQ(cloud->prepared(), 3u);
}

// Frozen, the worker prepares nothing, lets every offer go at once, and
// drops a snapshot the window has not taken; the window keeps the one it
// holds. Live again, the next set is prepared.
TEST_F(AppCloud, FreezeHolds) {
  std::unique_ptr<CloudWorker> cloud = start(1);
  ASSERT_NE(cloud, nullptr);
  cloud->offer(make_set(1, 1));
  const std::shared_ptr<const CloudSnapshot> held = next_snapshot(*cloud);
  ASSERT_NE(held, nullptr);

  cloud->set_live(false);
  EXPECT_FALSE(cloud->live());
  std::vector<std::weak_ptr<const void>> watched;
  cloud->offer(make_set(1, 2, &watched));
  EXPECT_TRUE(all_gone(watched));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(cloud->take_snapshot(), nullptr);
  EXPECT_EQ(cloud->prepared(), 1u);
  EXPECT_EQ(held->sequence, 1u);

  cloud->set_live(true);
  cloud->offer(make_set(1, 3));
  wait_prepared(*cloud, 2);
  cloud->set_live(false);  // before the window took it
  EXPECT_EQ(cloud->take_snapshot(), nullptr);
  cloud->set_live(true);
  cloud->offer(make_set(1, 4));
  const std::shared_ptr<const CloudSnapshot> live = next_snapshot(*cloud);
  ASSERT_NE(live, nullptr);
  EXPECT_EQ(live->sequence, 4u);
}

// Once prepared, nothing of the set is held: its frames' pixels go back.
TEST_F(AppCloud, HoldsNoSetPastItsPreparation) {
  std::unique_ptr<CloudWorker> cloud = start(3);
  ASSERT_NE(cloud, nullptr);
  std::vector<std::weak_ptr<const void>> watched;
  std::shared_ptr<const sensor::FrameSet> set = make_set(3, 1, &watched);
  const std::weak_ptr<const sensor::FrameSet> watched_set = set;
  cloud->offer(std::move(set));
  const std::shared_ptr<const CloudSnapshot> s = next_snapshot(*cloud);
  ASSERT_NE(s, nullptr);
  EXPECT_TRUE(watched_set.expired());
  EXPECT_TRUE(all_gone(watched));
  EXPECT_TRUE(s->frames[2].has_value());
}

// The solid mode prepares depth only; colour comes back with the next set
// once a colour mode asks for it.
TEST_F(AppCloud, SolidPreparesDepthOnly) {
  std::unique_ptr<CloudWorker> cloud = start(2);
  ASSERT_NE(cloud, nullptr);
  cloud->set_color(false);
  cloud->offer(make_set(2, 1));
  std::shared_ptr<const CloudSnapshot> s = next_snapshot(*cloud);
  ASSERT_NE(s, nullptr);
  EXPECT_FALSE(s->color);
  for (const std::optional<sensor::DeviceFrame>& f : s->frames) {
    ASSERT_TRUE(f.has_value());
    EXPECT_NE(f->depth, nullptr);
    EXPECT_EQ(f->color, nullptr);
  }
  cloud->set_color(true);
  cloud->offer(make_set(2, 2));
  s = next_snapshot(*cloud);
  ASSERT_NE(s, nullptr);
  EXPECT_TRUE(s->color);
  EXPECT_NE(s->frames[0]->color, nullptr);
}

// A set the preparation refuses, or one of the wrong size, arrives as a
// Status; the worker goes on with the next.
TEST_F(AppCloud, ARefusedSetArrivesAsAStatus) {
  std::unique_ptr<CloudWorker> cloud = start(2);
  ASSERT_NE(cloud, nullptr);
  cloud->offer(make_set(2, 1, nullptr, 0.0f));  // no near bound
  std::vector<core::Status> errors = wait_errors(*cloud);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].domain(), Code::InvalidArgument);
  EXPECT_NE(errors[0].message().find("3D view"), std::string::npos);
  cloud->offer(make_set(3, 2));
  errors = wait_errors(*cloud);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors[0].message().find("a set of 3 frames for 2 cameras"),
            std::string::npos)
      << errors[0].message();
  EXPECT_EQ(cloud->prepared(), 0u);
  cloud->offer(make_set(2, 3));
  const std::shared_ptr<const CloudSnapshot> s = next_snapshot(*cloud);
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->sequence, 3u);
}

TEST_F(AppCloud, RefusesWhatItCannotPrepare) {
  const std::uint32_t family = device().queue_family();
  const auto refused = [&](std::size_t cameras,
                           const std::vector<std::uint32_t>& families,
                           std::uint32_t live_every) {
    CloudOptions options;
    options.live_every = live_every;
    return CloudWorker::create(device(), allocator(), cameras, families,
                               options)
        .status();
  };
  EXPECT_EQ(refused(0, {family}, 1).domain(), Code::InvalidArgument);
  EXPECT_EQ(refused(1, {family}, 0).domain(), Code::InvalidArgument);
  EXPECT_EQ(refused(1, std::vector<std::uint32_t>(5, family), 1).domain(),
            Code::InvalidArgument);
  EXPECT_EQ(refused(1, {family, 9999}, 1).domain(), Code::InvalidArgument);
}

// --- Across two queue families ----------------------------------------------

// World to Vulkan clip space through pinhole camera `k` posed `view_to_world`,
// so that a point it images at pixel (x, y), centres on integers, lands at
// (x + 0.5, y + 0.5) of a k-sized viewport.
glm::mat4 pinhole_view_proj(const camera::CameraModel& k,
                            const Mat4d& view_to_world) {
  const double near_z = 0.05;
  const double far_z = 20.0;
  const double w = k.size.width;
  const double h = k.size.height;
  glm::dmat4 p(0.0);
  p[0][0] = 2.0 * k.intrinsics.fx / w;
  p[2][0] = 2.0 * (k.intrinsics.cx + 0.5) / w - 1.0;
  p[1][1] = 2.0 * k.intrinsics.fy / h;
  p[2][1] = 2.0 * (k.intrinsics.cy + 0.5) / h - 1.0;
  p[2][2] = far_z / (far_z - near_z);
  p[3][2] = -far_z * near_z / (far_z - near_z);
  p[2][3] = 1.0;
  return glm::mat4(p * camera::rigid_inverse(view_to_world));
}

// Where camera `k` images a point in its frame: pixel coordinates, centres on
// integers.
glm::dvec2 image(const camera::CameraModel& k, const Vec3d& p) {
  return {k.intrinsics.fx * p.x / p.z + k.intrinsics.cx,
          k.intrinsics.fy * p.y / p.z + k.intrinsics.cy};
}

// The pixel whose centre is nearest `x`, which must not sit on a boundary.
std::uint32_t nearest(double x) {
  EXPECT_GT(std::abs(x - std::floor(x) - 0.5), 0.05) << x;
  return static_cast<std::uint32_t>(std::floor(x + 0.5));
}

// The point the frames' depth has, in their depth camera.
Vec3d the_point() {
  const camera::CameraModel k = depth_model();
  return {(kU - k.intrinsics.cx) * kZ / k.intrinsics.fx,
          (kV - k.intrinsics.cy) * kZ / k.intrinsics.fy, kZ};
}

class AppCloudAcrossFamilies : public core::test::SharedDeviceTest {
 protected:
  core::DeviceRequirements compute_requirements() const override {
    return recon::device_requirements();
  }
  core::DeviceRequirements graphics_requirements() const override {
    return gfx::device_requirements();
  }
  // The draws read the buffers through descriptors, which synchronization
  // validation tracks only at this level.
  core::test::Validation validation() const override {
    return core::test::Validation::ShaderAccesses;
  }

  void SetUp() override {
    core::test::SharedDeviceTest::SetUp();
    if (base_setup_incomplete()) return;
    core::test::TwoFamilyDevice& two = two_family();
    core::Result<core::TimelineSemaphore> timeline =
        core::TimelineSemaphore::create(two.graphics(), 0);
    ASSERT_TRUE(timeline.ok()) << timeline.status().message();
    timeline_.emplace(std::move(timeline).value());
    gfx::RenderTargetLayout layout;
    layout.color_formats[0] = VK_FORMAT_R8G8B8A8_UNORM;
    layout.color_count = 1;
    layout.depth_format = VK_FORMAT_D32_SFLOAT;
    core::Result<std::unique_ptr<pipelines::DepthCloudPipeline>> pipeline =
        pipelines::DepthCloudPipeline::create(
            two.graphics(), two.graphics_allocator(), layout, *timeline_);
    ASSERT_TRUE(pipeline.ok()) << pipeline.status().message();
    pipeline_ = std::move(pipeline).value();
    retire_.emplace(*timeline_);
  }

  void TearDown() override {
    retire_.reset();
    pipeline_.reset();
    timeline_.reset();
    core::test::SharedDeviceTest::TearDown();
  }

  std::unique_ptr<CloudWorker> start(
      const std::vector<std::uint32_t>& families) {
    core::test::TwoFamilyDevice& two = two_family();
    core::Result<std::unique_ptr<CloudWorker>> made = CloudWorker::create(
        two.compute(), two.compute_allocator(), 1, families, every_set());
    EXPECT_TRUE(made.ok()) << made.status().message();
    return made.ok() ? std::move(made).value() : nullptr;
  }

  // Both families, as calib_rig names them.
  std::vector<std::uint32_t> both() {
    return {two_family().families()[0], two_family().families()[1]};
  }

  viz::CloudStyle style(const glm::mat4& view_proj) {
    viz::CloudStyle s;
    s.mode = pipelines::CloudColor::True;
    s.stride = 1;
    s.point_size = 1.0f;
    s.viewport = {{0, 0}, {kDepthW, kDepthH}};
    s.view_proj = view_proj;
    s.producer_family = two_family().families()[0];
    s.renderer_family = two_family().families()[1];
    return s;
  }

  struct Rendered {
    core::Status recorded;
    std::optional<core::TimelinePoint> wait;
    std::vector<std::uint8_t> rgba;  // kDepthW x kDepthH, rows packed
  };

  // record_cloud as the window runs it, as frame `next_`, into a
  // kDepthW x kDepthH target cleared to black: the frame's submission waits
  // for what it returns at VERTEX_SHADER, with no host wait before it; then,
  // once the frame has executed, `between` runs, which may release
  // `snapshot`; then the frame's number is set and the retire queue polled.
  Rendered render(const std::shared_ptr<const CloudSnapshot>& snapshot,
                  const std::vector<viz::SceneCamera>& cameras,
                  const viz::CloudStyle& s,
                  const std::function<void()>& between = {}) {
    Rendered out;
    core::test::TwoFamilyDevice& two = two_family();
    core::Device& graphics = two.graphics();
    const std::uint64_t number = next_++;
    gfx::OffscreenTargetDesc desc;
    desc.extent = {kDepthW, kDepthH};
    desc.color_format = VK_FORMAT_R8G8B8A8_UNORM;
    desc.depth_format = VK_FORMAT_D32_SFLOAT;
    core::Result<gfx::OffscreenTarget> target =
        gfx::OffscreenTarget::create(two.graphics_allocator(), desc);
    core::Result<core::CommandPool> pool =
        core::CommandPool::create(graphics.handle(), graphics.queue_family());
    EXPECT_TRUE(target.ok() && pool.ok());
    if (!target.ok() || !pool.ok()) return out;
    core::Result<core::CommandBuffer> cmd = pool.value().allocate_primary();
    EXPECT_TRUE(cmd.ok());
    if (!cmd.ok()) return out;
    EXPECT_TRUE(
        cmd.value().begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT).ok());
    const VkCommandBuffer raw = cmd.value().handle();
    target.value().prepare(raw);
    gfx::RenderTargetBeginInfo begin;
    begin.clear_color.float32[3] = 1.0f;
    const gfx::RenderTarget rt = target.value().target();
    rt.begin(raw, begin);
    core::Result<std::optional<core::TimelinePoint>> wait = record_cloud(
        *pipeline_, *retire_, raw, desc.extent, number, snapshot, cameras, s);
    rt.end(raw);
    target.value().record_readback(raw);
    EXPECT_TRUE(cmd.value().end().ok());
    out.recorded = wait.status();
    if (wait.ok()) out.wait = wait.value();
    const core::Status ran = [&]() -> core::Status {
      VKC_ASSIGN(core::Fence done, core::Fence::create(graphics.handle()));
      VkSubmitInfo submit{};
      submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submit.commandBufferCount = 1;
      submit.pCommandBuffers = &raw;
      VkTimelineSemaphoreSubmitInfo values{};
      VkSemaphore semaphore = VK_NULL_HANDLE;
      const VkPipelineStageFlags stage = VK_PIPELINE_STAGE_VERTEX_SHADER_BIT;
      if (out.wait) {
        semaphore = out.wait->semaphore->handle();
        values.sType = VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO;
        values.waitSemaphoreValueCount = 1;
        values.pWaitSemaphoreValues = &out.wait->value;
        submit.pNext = &values;
        submit.waitSemaphoreCount = 1;
        submit.pWaitSemaphores = &semaphore;
        submit.pWaitDstStageMask = &stage;
      }
      VKC_VK_TRY(graphics.queue_submit(1, &submit, done.handle()));
      return done.wait();
    }();
    EXPECT_TRUE(ran.ok()) << ran.message();
    if (between) between();
    EXPECT_TRUE(timeline_->signal(number).ok());
    retire_->poll();
    const auto* px = static_cast<const std::uint8_t*>(target.value().pixels());
    if (ran.ok() && px != nullptr) {
      out.rgba.assign(px, px + std::size_t{kDepthW} * kDepthH * 4);
    }
    return out;
  }

  // The pixels drawn: not the black the target was cleared to.
  static std::vector<std::pair<std::uint32_t, std::uint32_t>> lit(
      const std::vector<std::uint8_t>& rgba) {
    std::vector<std::pair<std::uint32_t, std::uint32_t>> out;
    for (std::uint32_t y = 0; y < kDepthH; ++y) {
      for (std::uint32_t x = 0; x < kDepthW; ++x) {
        const std::size_t at = (std::size_t{y} * kDepthW + x) * 4;
        if (rgba[at] != 0 || rgba[at + 1] != 0 || rgba[at + 2] != 0) {
          out.emplace_back(x, y);
        }
      }
    }
    return out;
  }

  static std::uint32_t rgb_at(const std::vector<std::uint8_t>& rgba,
                              std::uint32_t x, std::uint32_t y) {
    const std::size_t at = (std::size_t{y} * kDepthW + x) * 4;
    return pipelines::pack_rgba8(rgba[at], rgba[at + 1], rgba[at + 2], 0);
  }

  // The camera, placed at `color_to_world`.
  static std::vector<viz::SceneCamera> placed(const Mat4d& color_to_world) {
    viz::SceneCamera camera;
    camera.color = color_model();
    camera.depth = depth_model();
    camera.depth_to_color = depth_to_color();
    camera.color_to_world = color_to_world;
    return {camera};
  }

  std::optional<core::TimelineSemaphore> timeline_;
  std::unique_ptr<pipelines::DepthCloudPipeline> pipeline_;
  std::optional<gfx::RetireQueue> retire_;
  std::uint64_t next_ = 1;
};

// A synthetic frame prepared on the compute family and drawn on the graphics
// family in place: its one point lands on its depth pixel through a view of
// its own depth camera, in the colour of the colour pixel recon's rule gives
// it -- the nearest centre, through the factory extrinsic. The frame waits at
// the vertex stage for the value the preparation signalled, which
// record_cloud returns, as gfx's contract asks of a producer on another
// queue, and validation, synchronization included, stays clean. The snapshot
// is held until its frame retires: executed but not yet retired, with the
// test's own reference gone, the retire queue alone keeps it.
TEST_F(AppCloudAcrossFamilies, AKnownPointLandsOnItsPixelWithItsColour) {
  std::unique_ptr<CloudWorker> cloud = start(both());
  ASSERT_NE(cloud, nullptr);
  cloud->offer(make_set(1, 1));
  std::shared_ptr<const CloudSnapshot> snapshot = next_snapshot(*cloud);
  ASSERT_NE(snapshot, nullptr);
  EXPECT_EQ(snapshot->frames[0]->depth->sharing_mode(),
            VK_SHARING_MODE_CONCURRENT);
  EXPECT_EQ(snapshot->frames[0]->color->sharing_mode(),
            VK_SHARING_MODE_CONCURRENT);
  const core::TimelinePoint ready = snapshot->ready;
  const std::weak_ptr<const CloudSnapshot> watched = snapshot;

  const Mat4d color_to_world = pose(17.0, {0.3, 1.0, 0.2}, {0.4, -0.2, 1.5});
  const Mat4d depth_to_world = color_to_world * depth_to_color();
  const Rendered drawn =
      render(snapshot, placed(color_to_world),
             style(pinhole_view_proj(depth_model(), depth_to_world)), [&] {
               snapshot.reset();
               EXPECT_EQ(retire_->pending(), 1u);
               retire_->poll();  // the frame's number is not reached yet
               EXPECT_FALSE(watched.expired());
             });
  EXPECT_TRUE(watched.expired());
  EXPECT_EQ(retire_->pending(), 0u);
  ASSERT_TRUE(drawn.recorded.ok()) << drawn.recorded.message();
  ASSERT_TRUE(drawn.wait.has_value());
  EXPECT_EQ(drawn.wait->semaphore, ready.semaphore);
  EXPECT_EQ(drawn.wait->value, ready.value);
  EXPECT_GT(drawn.wait->value, 0u);
  ASSERT_FALSE(drawn.rgba.empty());

  const glm::dvec2 c = image(
      color_model(), Vec3d(depth_to_color() * glm::dvec4(the_point(), 1.0)));
  const std::uint32_t expected = coded(nearest(c.x), nearest(c.y));
  const auto points = lit(drawn.rgba);
  ASSERT_EQ(points.size(), 1u);
  EXPECT_EQ(points[0], std::make_pair(kU, kV));
  EXPECT_EQ(rgb_at(drawn.rgba, kU, kV), expected & 0x00FFFFFFu);
}

// Frozen, the snapshot redraws where a new pose puts its camera, with no
// second preparation: the point moves as the pose moves it through the same
// view, and keeps its colour, which the camera's own extrinsic decides.
TEST_F(AppCloudAcrossFamilies, AFrozenSnapshotRedrawsAtNewPosesWithoutARePrep) {
  std::unique_ptr<CloudWorker> cloud = start(both());
  ASSERT_NE(cloud, nullptr);
  cloud->offer(make_set(1, 1));
  const std::shared_ptr<const CloudSnapshot> snapshot = next_snapshot(*cloud);
  ASSERT_NE(snapshot, nullptr);
  cloud->set_live(false);
  cloud->offer(make_set(1, 2));

  const Mat4d before = pose(17.0, {0.3, 1.0, 0.2}, {0.4, -0.2, 1.5});
  // Calibrate moved the camera 20 cm along its own x.
  const Mat4d after = before * pose(0.0, {0.0, 0.0, 1.0}, {0.2, 0.0, 0.0});
  const Mat4d view_pose = before * depth_to_color();
  const viz::CloudStyle s = style(pinhole_view_proj(depth_model(), view_pose));
  const Rendered first = render(snapshot, placed(before), s);
  const Rendered second = render(snapshot, placed(after), s);
  ASSERT_TRUE(first.recorded.ok()) << first.recorded.message();
  ASSERT_TRUE(second.recorded.ok()) << second.recorded.message();
  ASSERT_FALSE(second.rgba.empty());

  const Vec3d world =
      Vec3d(after * depth_to_color() * glm::dvec4(the_point(), 1.0));
  const glm::dvec2 at =
      image(depth_model(),
            Vec3d(camera::rigid_inverse(view_pose) * glm::dvec4(world, 1.0)));
  const std::pair<std::uint32_t, std::uint32_t> moved{nearest(at.x),
                                                      nearest(at.y)};
  EXPECT_NE(moved, std::make_pair(kU, kV));
  ASSERT_EQ(lit(first.rgba).size(), 1u);
  const auto points = lit(second.rgba);
  ASSERT_EQ(points.size(), 1u);
  EXPECT_EQ(points[0], moved);
  EXPECT_EQ(rgb_at(second.rgba, moved.first, moved.second),
            rgb_at(first.rgba, kU, kV));
  EXPECT_EQ(cloud->prepared(), 1u);
  EXPECT_EQ(cloud->take_snapshot(), nullptr);
}

// A worker that names only its own family leaves the frames EXCLUSIVE to it,
// so the renderer's family may not read them: record_cloud refuses, with
// nothing recorded and nothing held.
TEST_F(AppCloudAcrossFamilies, RefusesAFrameExclusiveToTheComputeFamily) {
  std::unique_ptr<CloudWorker> cloud = start({two_family().families()[0]});
  ASSERT_NE(cloud, nullptr);
  cloud->offer(make_set(1, 1));
  const std::shared_ptr<const CloudSnapshot> snapshot = next_snapshot(*cloud);
  ASSERT_NE(snapshot, nullptr);
  EXPECT_EQ(snapshot->frames[0]->depth->sharing_mode(),
            VK_SHARING_MODE_EXCLUSIVE);
  const Rendered drawn =
      render(snapshot, placed(Mat4d(1.0)),
             style(pinhole_view_proj(depth_model(), depth_to_color())),
             [&] { EXPECT_EQ(retire_->pending(), 0u); });
  EXPECT_EQ(drawn.recorded.domain(), Code::InvalidArgument);
  EXPECT_NE(drawn.recorded.message().find("EXCLUSIVE"), std::string::npos)
      << drawn.recorded.message();
  EXPECT_FALSE(drawn.wait.has_value());
  EXPECT_TRUE(lit(drawn.rgba).empty());
}

// With no camera placed there is nothing to draw: record_cloud returns no
// wait, and holds nothing.
TEST_F(AppCloudAcrossFamilies, DrawsAndHoldsNothingWithNoCameraPlaced) {
  std::unique_ptr<CloudWorker> cloud = start(both());
  ASSERT_NE(cloud, nullptr);
  cloud->offer(make_set(1, 1));
  const std::shared_ptr<const CloudSnapshot> snapshot = next_snapshot(*cloud);
  ASSERT_NE(snapshot, nullptr);
  std::vector<viz::SceneCamera> unplaced = placed(Mat4d(1.0));
  unplaced[0].color_to_world.reset();
  const Rendered drawn =
      render(snapshot, unplaced,
             style(pinhole_view_proj(depth_model(), depth_to_color())),
             [&] { EXPECT_EQ(retire_->pending(), 0u); });
  ASSERT_TRUE(drawn.recorded.ok()) << drawn.recorded.message();
  EXPECT_FALSE(drawn.wait.has_value());
  EXPECT_TRUE(lit(drawn.rgba).empty());
}

// --- The device
// ---------------------------------------------------------------

// calib_rig's renderer asks for wide points, which every point it draws may
// use, and its compute side for recon's and the depth checks' needs.
TEST(AppCloudDevice, TheRigsDeviceAsksForLargePoints) {
  const core::Result<RigRequirements> wide = rig_requirements();
  ASSERT_TRUE(wide.ok()) << wide.status().message();
  EXPECT_EQ(wide.value().graphics.features.largePoints, VK_TRUE);
  EXPECT_TRUE(wide.value().graphics.dynamic_rendering);
  EXPECT_TRUE(wide.value().compute.scalar_block_layout);
  EXPECT_NE(wide.value().compute.queue_flags & VK_QUEUE_COMPUTE_BIT, 0u);
  EXPECT_TRUE(core::merge(wide.value().compute, wide.value().graphics).ok());
  const core::Result<RigRequirements> narrow = rig_requirements(false);
  ASSERT_TRUE(narrow.ok());
  EXPECT_EQ(narrow.value().graphics.features.largePoints, VK_FALSE);
}

}  // namespace
}  // namespace volumetric_kit::calib::app
