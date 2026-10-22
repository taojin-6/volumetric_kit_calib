// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// What the window makes of a live snapshot: each camera's picture drawn in its
// own tile of the rig mosaic, a camera missing from the next set keeping its
// last picture, and the set held until the frame that copied it retires on
// the window's timeline; a snapshot that does not fit refused with nothing
// held. Then the app's device split: the live worker reads a camera's picture
// back on the compute library's queue family while the window copies the same
// images into the mosaic on the renderer's. Then each camera's tile refusal,
// said once, the marks and panels a snapshot gives each camera, the origin
// tag's coverage of the captures kept, and the 3D view's cameras before and
// at Calibrate's result. The GPU
// cases run under the validation layer
// when the environment asks (VKC_TEST_VALIDATION) and skip without a Vulkan
// device, the cross-family case also with one family.

#include "volumetric_kit/calib/app/live_view.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "device_picture.hpp"
#include "fake_sensor.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/core/testing/two_family_device.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/core/vulkan/command_buffer.hpp"
#include "volumetric_kit/core/vulkan/command_pool.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/gfx/core/device_requirements.hpp"
#include "volumetric_kit/gfx/core/offscreen_target.hpp"
#include "volumetric_kit/gfx/pipelines/color.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace sensor = recon::sensor;
using Code = core::Status::Code;

constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;
constexpr VkExtent2D kExtent{128, 96};
constexpr std::uint32_t kTileW = 32;
constexpr std::uint32_t kTileH = 24;

gfx::RenderTargetLayout color_layout() {
  gfx::RenderTargetLayout layout;
  layout.color_formats[0] = kFormat;
  layout.color_count = 1;
  return layout;
}

// A set of one frame per luma, a picture of that grey; none where empty.
struct Set {
  std::shared_ptr<const sensor::FrameSet> frames;
  std::vector<std::weak_ptr<const void>> pixels;  // each frame's
};

class AppLiveView : public core::test::VulkanDeviceTest {
 protected:
  core::DeviceRequirements requirements() const override {
    return gfx::device_requirements();
  }

  Set make_set(const std::vector<std::optional<std::uint8_t>>& grey) {
    auto set = std::make_shared<sensor::FrameSet>();
    Set out;
    for (const std::optional<std::uint8_t>& level : grey) {
      if (!level) {
        set->frames.emplace_back();
        out.pixels.emplace_back();
        continue;
      }
      sensor::RgbdFrame frame;
      frame.color = test::device_nv12(
          device(), allocator(), kTileW, kTileH,
          std::vector<std::uint8_t>(std::size_t{kTileW} * kTileH, *level));
      EXPECT_NE(frame.color.image[0], nullptr);
      auto held = std::make_shared<int>(*level);
      out.pixels.emplace_back(held);
      frame.pixels = std::move(held);
      set->frames.emplace_back(std::move(frame));
    }
    out.frames = std::move(set);
    return out;
  }

  static std::shared_ptr<const LiveSnapshot> snapshot_of(const Set& set) {
    auto snapshot = std::make_shared<LiveSnapshot>();
    snapshot->set = set.frames;
    const std::size_t n = set.frames->frames.size();
    snapshot->detections.resize(n);
    snapshot->checks.resize(n);
    snapshot->errors.resize(n);
    return snapshot;
  }

  // Records the snapshot into the mosaic as frame `number`, draws the mosaic,
  // and returns the target's pixels.
  std::vector<std::uint8_t> render(viz::RigMosaic& mosaic,
                                   gfx::RetireQueue& retire,
                                   const std::shared_ptr<const LiveSnapshot>& s,
                                   std::uint64_t number) {
    gfx::OffscreenTargetDesc desc;
    desc.extent = kExtent;
    desc.color_format = kFormat;
    core::Result<gfx::OffscreenTarget> target =
        gfx::OffscreenTarget::create(allocator(), desc);
    core::Result<core::CommandPool> pool =
        core::CommandPool::create(device().handle(), device().queue_family());
    EXPECT_TRUE(target.ok() && pool.ok());
    if (!target.ok() || !pool.ok()) return {};
    core::Result<core::CommandBuffer> cmd = pool.value().allocate_primary();
    EXPECT_TRUE(cmd.ok());
    if (!cmd.ok()) return {};
    EXPECT_TRUE(
        cmd.value().begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT).ok());
    const VkCommandBuffer raw = cmd.value().handle();
    const core::Status recorded =
        record_snapshot(mosaic, retire, raw, s, number);
    EXPECT_TRUE(recorded.ok()) << recorded.message();
    target.value().prepare(raw);
    const gfx::RenderTarget rt = target.value().target();
    rt.begin(raw, gfx::RenderTargetBeginInfo{});
    mosaic.draw(raw, kExtent);
    rt.end(raw);
    target.value().record_readback(raw);
    EXPECT_TRUE(cmd.value().end().ok());
    const core::Status submitted = device().submit_and_wait(raw);
    EXPECT_TRUE(submitted.ok()) << submitted.message();
    if (!submitted.ok()) return {};
    const auto* px = static_cast<const std::uint8_t*>(target.value().pixels());
    return {px, px + std::size_t{kExtent.width} * kExtent.height * 4};
  }

  // The grey at the middle of tile `i`, as drawn.
  static int tile_grey(const viz::RigMosaic& mosaic,
                       const std::vector<std::uint8_t>& pixels, std::size_t i) {
    const gfx::ui::OverlayRegion region = mosaic.tile_region(i);
    const glm::vec2 middle =
        mosaic.view().image_to_target(region.origin + 0.5f * region.size);
    const auto x = static_cast<std::size_t>(middle.x);
    const auto y = static_cast<std::size_t>(middle.y);
    return pixels[(y * kExtent.width + x) * 4 + 1];
  }

  std::unique_ptr<viz::RigMosaic> make_mosaic(std::size_t n) {
    core::Result<std::unique_ptr<viz::RigMosaic>> mosaic =
        viz::RigMosaic::create(
            device(), allocator(), color_layout(),
            std::vector<recon::camera::ImageSize>(n, {kTileW, kTileH}));
    EXPECT_TRUE(mosaic.ok()) << mosaic.status().message();
    if (!mosaic.ok()) return nullptr;
    mosaic.value()->view().set_viewport({0.0f, 0.0f},
                                        {static_cast<float>(kExtent.width),
                                         static_cast<float>(kExtent.height)});
    mosaic.value()->view().fit();
    return std::move(mosaic).value();
  }
};

TEST_F(AppLiveView, DrawsEachCamerasPictureInItsTileUntilItsFrameRetires) {
  std::unique_ptr<viz::RigMosaic> mosaic = make_mosaic(3);
  ASSERT_NE(mosaic, nullptr);
  core::Result<core::TimelineSemaphore> timeline =
      core::TimelineSemaphore::create(device());
  ASSERT_TRUE(timeline.ok()) << timeline.status().message();
  gfx::RetireQueue retire(timeline.value());

  Set first = make_set({40, 120, 200});
  std::vector<std::uint8_t> pixels =
      render(*mosaic, retire, snapshot_of(first), 1);
  ASSERT_FALSE(pixels.empty());
  const int expected[] = {40, 120, 200};
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(tile_grey(*mosaic, pixels, i), expected[i], 3) << i;
  }

  // The window has let the snapshot go; the queue holds its set until frame
  // 1 retires.
  const std::vector<std::weak_ptr<const void>> held = first.pixels;
  first = Set{};
  for (const auto& frame : held) EXPECT_FALSE(frame.expired());
  retire.poll();
  for (const auto& frame : held) EXPECT_FALSE(frame.expired());
  ASSERT_TRUE(timeline.value().signal(1).ok());
  retire.poll();
  for (const auto& frame : held) EXPECT_TRUE(frame.expired());

  // The next set lacks camera 1: its tile keeps its last picture.
  Set second = make_set({80, std::nullopt, 160});
  pixels = render(*mosaic, retire, snapshot_of(second), 2);
  ASSERT_FALSE(pixels.empty());
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 0), 80, 3);
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 1), 120, 3);
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 2), 160, 3);
  EXPECT_TRUE(mosaic->refused().empty());
  ASSERT_TRUE(timeline.value().signal(2).ok());
  retire.poll();
  EXPECT_EQ(retire.pending(), 0u);
}

TEST_F(AppLiveView, RefusesASnapshotThatDoesNotFitTheMosaic) {
  std::unique_ptr<viz::RigMosaic> mosaic = make_mosaic(2);
  ASSERT_NE(mosaic, nullptr);
  core::Result<core::TimelineSemaphore> timeline =
      core::TimelineSemaphore::create(device());
  ASSERT_TRUE(timeline.ok());
  gfx::RetireQueue retire(timeline.value());
  core::Status refused =
      record_snapshot(*mosaic, retire, VK_NULL_HANDLE, nullptr, 1);
  EXPECT_EQ(refused.domain(), Code::InvalidArgument);
  const Set three = make_set({10, 20, 30});
  refused =
      record_snapshot(*mosaic, retire, VK_NULL_HANDLE, snapshot_of(three), 1);
  EXPECT_EQ(refused.domain(), Code::InvalidArgument);
  EXPECT_NE(refused.message().find("a set of 3 frames for a mosaic of 2"),
            std::string::npos);
  EXPECT_EQ(retire.pending(), 0u);
}

class AppLiveCrossFamily : public core::test::SharedDeviceTest {
 protected:
  core::DeviceRequirements graphics_requirements() const override {
    return gfx::device_requirements();
  }
};

// As calib_rig runs: the camera's decoded picture CONCURRENT across both
// families (recon's color_queue_families), the live worker finding the board
// in it on the compute family, and the window drawing it, one texel a pixel,
// on the graphics family.
TEST_F(AppLiveCrossFamily, TheWindowDrawsWhatTheLiveWorkerReadOnAnotherFamily) {
  core::test::TwoFamilyDevice& two = two_family();
  const std::vector<std::uint32_t> families(two.families().begin(),
                                            two.families().end());
  // A 960 x 544 picture, the mosaic's own size, so the view fits it 1:1.
  constexpr std::uint32_t kW = 960;
  constexpr std::uint32_t kH = 544;
  recon::camera::CameraModel lens = test::color_like_lens();
  lens.size = {kW, kH};
  lens.intrinsics.fx /= 4.0;
  lens.intrinsics.fy /= 4.0;
  lens.intrinsics.cx /= 4.0;
  lens.intrinsics.cy /= 4.0;
  const compute::Board board = test::femto_board();
  const cv::Mat render = test::render_board(
      lens, test::board_facing_camera(board, {0.0, 0.0, 1.2}, 20.0, 5.0),
      board);
  const std::vector<std::uint8_t> luma(render.data,
                                       render.data + render.total());
  const sensor::YuvImage picture = test::device_nv12(
      two.compute(), two.compute_allocator(), kW, kH, luma, families);
  ASSERT_NE(picture.image[0], nullptr);
  ASSERT_NE(picture.image[1], nullptr);

  sensor::SensorInfo info;
  info.id = "SYN0";
  info.color = lens;
  info.fps = 25;
  auto fake = std::make_unique<test::FakeSensor>(info);
  test::FakeSensor& camera = *fake;
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors;
  sensors.push_back(std::move(fake));
  core::Result<sensor::SensorArray> array =
      sensor::SensorArray::open(std::move(sensors), {});
  ASSERT_TRUE(array.ok()) << array.status().message();
  core::Result<std::unique_ptr<LiveWorker>> live = LiveWorker::create(
      std::move(array).value(), two.compute(), two.compute_allocator(),
      legacy_femto_board(), false, nullptr);
  ASSERT_TRUE(live.ok()) << live.status().message();
  sensor::RgbdFrame frame;
  frame.color = picture;
  frame.color_camera = lens;
  frame.timestamp_ns = 1'760'000'000'000'000'000ull;
  camera.push(std::move(frame));
  std::shared_ptr<const LiveSnapshot> snapshot;
  const auto give_up =
      std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (!snapshot && std::chrono::steady_clock::now() < give_up) {
    snapshot = live.value()->take_snapshot();
  }
  ASSERT_NE(snapshot, nullptr);
  EXPECT_TRUE(live.value()->take_errors().empty());
  EXPECT_EQ(snapshot->detections[0].observation.points.size(), 8u);
  EXPECT_TRUE(is_usable(snapshot->checks[0].verdict));

  core::Result<std::unique_ptr<viz::RigMosaic>> mosaic = viz::RigMosaic::create(
      two.graphics(), two.graphics_allocator(), color_layout(), {{kW, kH}});
  ASSERT_TRUE(mosaic.ok()) << mosaic.status().message();
  viz::RigMosaic& tiles = *mosaic.value();
  tiles.view().set_viewport({0.0f, 0.0f},
                            {static_cast<float>(kW), static_cast<float>(kH)});
  tiles.view().fit();
  ASSERT_EQ(tiles.view().scale(), 1.0f);
  core::Result<core::TimelineSemaphore> timeline =
      core::TimelineSemaphore::create(two.graphics());
  ASSERT_TRUE(timeline.ok()) << timeline.status().message();
  gfx::RetireQueue retire(timeline.value());

  gfx::OffscreenTargetDesc desc;
  desc.extent = {kW, kH};
  desc.color_format = kFormat;
  core::Result<gfx::OffscreenTarget> target =
      gfx::OffscreenTarget::create(two.graphics_allocator(), desc);
  core::Result<core::CommandPool> pool = core::CommandPool::create(
      two.graphics().handle(), two.graphics().queue_family());
  ASSERT_TRUE(target.ok() && pool.ok());
  core::Result<core::CommandBuffer> cmd = pool.value().allocate_primary();
  ASSERT_TRUE(cmd.ok());
  ASSERT_TRUE(
      cmd.value().begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT).ok());
  const VkCommandBuffer raw = cmd.value().handle();
  const core::Status recorded =
      record_snapshot(tiles, retire, raw, snapshot, 1);
  ASSERT_TRUE(recorded.ok()) << recorded.message();
  target.value().prepare(raw);
  const gfx::RenderTarget rt = target.value().target();
  rt.begin(raw, gfx::RenderTargetBeginInfo{});
  tiles.draw(raw, {kW, kH});
  rt.end(raw);
  target.value().record_readback(raw);
  ASSERT_TRUE(cmd.value().end().ok());
  const core::Status drawn = two.graphics().submit_and_wait(raw);
  ASSERT_TRUE(drawn.ok()) << drawn.message();

  // Every texel as the camera coded it: grey, through the full-range matrix.
  const auto* px = static_cast<const std::uint8_t*>(target.value().pixels());
  int worst = 0;
  for (std::uint32_t y = 0; y < kH; y += 3) {
    for (std::uint32_t x = 0; x < kW; x += 3) {
      const std::size_t at = std::size_t{y} * kW + x;
      worst = std::max(worst, std::abs(static_cast<int>(px[at * 4 + 1]) -
                                       static_cast<int>(luma[at])));
    }
  }
  EXPECT_LE(worst, 3);

  live.value()->stop();
  ASSERT_TRUE(timeline.value().signal(1).ok());
  retire.poll();
  EXPECT_EQ(retire.pending(), 0u);
}

// The mosaic refuses a camera's picture alike at every set: the window says
// each camera's refusal once, until its message changes.
TEST(AppLiveViewModels, SaysEachCamerasTileRefusalOnceUntilItChanges) {
  const auto refusal = [](std::size_t camera, const char* why) {
    return viz::TileRefusal{camera, core::Status::unsupported(why)};
  };
  std::vector<std::string> said;
  core::Result<std::vector<viz::TileRefusal>> unsaid = refusals_to_say(
      {refusal(1, "an I420 picture"), refusal(3, "an I420 picture")}, said);
  ASSERT_TRUE(unsaid.ok()) << unsaid.status().message();
  ASSERT_EQ(unsaid->size(), 2u);
  EXPECT_EQ((*unsaid)[0].camera, 1u);
  EXPECT_EQ((*unsaid)[1].camera, 3u);
  EXPECT_EQ((*unsaid)[1].status.message(), "an I420 picture");

  // The next sets: nothing new, then camera 1's reason changes.
  unsaid = refusals_to_say(
      {refusal(1, "an I420 picture"), refusal(3, "an I420 picture")}, said);
  ASSERT_TRUE(unsaid.ok());
  EXPECT_TRUE(unsaid->empty());
  unsaid = refusals_to_say(
      {refusal(1, "buffer planes"), refusal(3, "an I420 picture")}, said);
  ASSERT_TRUE(unsaid.ok());
  ASSERT_EQ(unsaid->size(), 1u);
  EXPECT_EQ((*unsaid)[0].camera, 1u);
  EXPECT_EQ((*unsaid)[0].status.message(), "buffer planes");
}

// Each tile's overlay names its camera and takes its check, its detection
// when it found the board, and its part of the session.
TEST(AppLiveViewModels, GivesEachTileItsCamerasMarks) {
  LiveSnapshot snapshot;
  snapshot.detections.resize(3);
  snapshot.detections[1].observation.points.push_back({0, {10.0, 12.0}, 1.0});
  snapshot.checks.resize(3);
  snapshot.checks[1].verdict = ViewVerdict::Partial;
  snapshot.checks[2].verdict = ViewVerdict::Collinear;
  const std::vector<std::string> ids = {"A", "B", "C"};
  quality::SessionSummary session;
  session.cameras.resize(3);

  core::Result<std::vector<viz::TileOverlay>> tiles =
      tile_overlays(snapshot, ids, &session);
  ASSERT_TRUE(tiles.ok()) << tiles.status().message();
  ASSERT_EQ(tiles->size(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(tiles.value()[i].id, ids[i]);
    EXPECT_EQ(tiles.value()[i].check.verdict, snapshot.checks[i].verdict);
    EXPECT_EQ(tiles.value()[i].session, &session.cameras[i]);
  }
  EXPECT_EQ(tiles.value()[0].detection, nullptr);  // nothing found
  EXPECT_EQ(tiles.value()[1].detection, &snapshot.detections[1]);

  tiles = tile_overlays(snapshot, ids, nullptr);
  ASSERT_TRUE(tiles.ok());
  EXPECT_EQ(tiles.value()[2].session, nullptr);

  EXPECT_EQ(tile_overlays(snapshot, {"A", "B"}, nullptr).status().domain(),
            Code::InvalidArgument);
  session.cameras.resize(2);
  EXPECT_EQ(tile_overlays(snapshot, ids, &session).status().domain(),
            Code::InvalidArgument);
}

// Each camera's panel takes its frame's presence, check, stillness, counters
// and lost frames.
TEST(AppLiveViewModels, GivesEachCameraItsLiveState) {
  LiveSnapshot snapshot;
  auto set = std::make_shared<sensor::FrameSet>();
  set->frames.resize(2);
  set->frames[0].emplace();
  snapshot.set = set;
  snapshot.checks.resize(2);
  snapshot.checks[0].verdict = ViewVerdict::Good;
  snapshot.stillness_px = {0.25, std::nullopt};
  snapshot.lost = {std::nullopt, 7u};
  snapshot.stats.sensors.resize(2);
  snapshot.stats.sensors[1].received = 42;
  const core::Result<std::vector<viz::CameraLive>> cameras =
      camera_lives(snapshot);
  ASSERT_TRUE(cameras.ok());
  ASSERT_EQ(cameras->size(), 2u);
  EXPECT_TRUE(cameras.value()[0].present);
  EXPECT_FALSE(cameras.value()[1].present);
  EXPECT_EQ(cameras.value()[0].check.verdict, ViewVerdict::Good);
  EXPECT_EQ(cameras.value()[0].stillness_px, 0.25);
  EXPECT_FALSE(cameras.value()[1].stillness_px.has_value());
  EXPECT_FALSE(cameras.value()[0].lost.has_value());
  EXPECT_EQ(cameras.value()[1].lost, 7u);
  EXPECT_EQ(cameras.value()[1].frames.received, 42u);
}

// The session panel's origin tag: each camera that found it in a capture
// kept, and every such view; a null capture, or one with no tags, adds none.
TEST(AppLiveViewModels, CountsTheCamerasAndViewsOfTheOriginTag) {
  const DetectedMarker tag{0, {}};
  const auto capture = [](std::vector<std::optional<DetectedMarker>> tags) {
    auto kept = std::make_shared<KeptCapture>();
    kept->tags = std::move(tags);
    return std::shared_ptr<const KeptCapture>(std::move(kept));
  };
  KeptCaptures kept;
  kept.cameras.resize(3);
  viz::TagCoverage coverage = tag_coverage(kept);
  EXPECT_EQ(coverage.cameras, 3u);
  EXPECT_EQ(coverage.seen_by, 0u);
  EXPECT_EQ(coverage.views, 0u);

  kept.captures = {capture({tag, std::nullopt, std::nullopt}),
                   capture({tag, std::nullopt, tag}), nullptr, capture({}),
                   capture({std::nullopt, std::nullopt, tag})};
  coverage = tag_coverage(kept);
  EXPECT_EQ(coverage.cameras, 3u);
  EXPECT_EQ(coverage.seen_by, 2u);  // camera 1 never saw it
  EXPECT_EQ(coverage.views, 4u);
}

// Before Calibrate only the reference is placed, at the world's origin, and
// each camera, past the palette's own colours too, starts in gfx's
// categorical colour of its index, so no two cameras share a tint.
TEST(AppLiveViewModels, PlacesTheReferenceAndTintsEachCameraByItsIndex) {
  sensor::SensorInfo info;
  info.color = test::color_like_lens();
  info.depth = test::depth_like_lens();
  info.depth_to_color[3][0] = -0.032;  // the depth camera 32 mm to one side
  constexpr std::size_t kCameras = gfx::pipelines::kCategoricalPaletteSize + 3;
  constexpr std::size_t kReference = 2;
  std::vector<std::uint32_t> tints;
  for (std::size_t i = 0; i < kCameras; ++i) {
    const viz::SceneCamera camera = scene_camera(info, i, i == kReference);
    EXPECT_EQ(camera.tint, gfx::pipelines::categorical_rgba8(i)) << i;
    EXPECT_EQ(camera.color_to_world.has_value(), i == kReference) << i;
    EXPECT_TRUE(camera.shown) << i;
    EXPECT_EQ(camera.color.size.width, info.color->size.width) << i;
    EXPECT_EQ(camera.color.intrinsics.fx, info.color->intrinsics.fx) << i;
    ASSERT_TRUE(camera.depth.has_value()) << i;
    EXPECT_EQ(camera.depth->intrinsics.fx, info.depth->intrinsics.fx) << i;
    EXPECT_EQ(camera.depth_to_color, info.depth_to_color) << i;
    tints.push_back(camera.tint);
  }
  EXPECT_EQ(scene_camera(info, 0, true).color_to_world,
            recon::camera::Mat4d(1.0));
  std::sort(tints.begin(), tints.end());
  EXPECT_EQ(std::adjacent_find(tints.begin(), tints.end()), tints.end());
}

// At a result every camera is placed at its own pose, in rig order, whatever
// its id, the reference among them; each keeps its models, tint and toggle.
// Poses for another number of cameras move none.
TEST(AppLiveViewModels, PlacesEveryCameraAtItsPoseInRigOrder) {
  sensor::SensorInfo info;
  info.color = test::color_like_lens();
  info.depth = test::depth_like_lens();
  for (const std::size_t n : {1u, 3u, 5u}) {
    SCOPED_TRACE(n);
    const std::size_t reference = n / 2;
    std::vector<viz::SceneCamera> cameras;
    std::vector<recon::camera::Mat4d> poses;
    for (std::size_t i = 0; i < n; ++i) {
      cameras.push_back(scene_camera(info, i, i == reference));
      cameras.back().shown = i % 2 == 0;
      cameras.back().depth_to_color[3][0] = -0.01 * static_cast<double>(i);
      // A turn about y and a step along x, each camera's own.
      const double turn = 0.3 * static_cast<double>(i);
      recon::camera::Mat4d pose(1.0);
      pose[0][0] = std::cos(turn);
      pose[0][2] = -std::sin(turn);
      pose[2][0] = std::sin(turn);
      pose[2][2] = std::cos(turn);
      pose[3][0] = 0.5 * static_cast<double>(i);
      poses.push_back(i == reference ? recon::camera::Mat4d(1.0) : pose);
    }
    const std::vector<viz::SceneCamera> before = cameras;

    std::vector<recon::camera::Mat4d> extra = poses;
    extra.push_back(recon::camera::Mat4d(1.0));
    const std::vector<recon::camera::Mat4d> fewer(poses.begin(),
                                                  poses.end() - 1);
    for (const auto& wrong : {extra, fewer}) {
      const core::Status refused = placed_cameras(cameras, wrong);
      EXPECT_EQ(refused.domain(), Code::InvalidArgument);
      for (std::size_t i = 0; i < n; ++i) {
        EXPECT_EQ(cameras[i].color_to_world, before[i].color_to_world) << i;
      }
    }

    const core::Status placed = placed_cameras(cameras, poses);
    ASSERT_TRUE(placed.ok()) << placed.message();
    for (std::size_t i = 0; i < n; ++i) {
      SCOPED_TRACE(i);
      ASSERT_TRUE(cameras[i].color_to_world.has_value());
      EXPECT_EQ(*cameras[i].color_to_world, poses[i]);
      EXPECT_EQ(cameras[i].tint, before[i].tint);
      EXPECT_EQ(cameras[i].shown, before[i].shown);
      EXPECT_EQ(cameras[i].depth_to_color, before[i].depth_to_color);
      EXPECT_EQ(cameras[i].color.intrinsics.fx, info.color->intrinsics.fx);
      EXPECT_TRUE(cameras[i].depth.has_value());
    }
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::app
