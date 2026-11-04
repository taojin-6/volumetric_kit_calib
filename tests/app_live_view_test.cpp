// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// What the window makes of a live snapshot: each camera's picture drawn in its
// own tile of the rig mosaic, a camera missing from the next set keeping its
// last picture, and the set held until the frame that copied it retires on
// the window's timeline; a snapshot that does not fit refused with nothing
// held; every copy in a frame that waits at the copies' stage for its set's
// luma pass, and a decoder's picture with a ready value copied only once it
// is reached, in a frame that waits for that too, the window's step keeping
// the snapshot fresh and its tiles untimed until then; a set taken without
// luma not shown at all, each tile keeping its last picture. Then the app's
// device split: the live worker reads a camera's picture back on the compute
// library's queue family while the window copies the same images into the
// mosaic on the renderer's; and, on the compute and graphics queues, a grey
// tile the luma pass wrote copied after its pass's value, and a set taken
// while the 3D view showed, its picture's ready value set later, prepared by
// the 3D view's worker only after it and not shown by the window, which
// shows the next set taken with luma.
// Then each camera's tile refusal, said once, the marks and panels a
// snapshot gives each camera, the origin tag's coverage of the captures
// kept, and the 3D view's cameras before and at Calibrate's result. The GPU
// cases run under the validation layer when the environment asks
// (VKC_TEST_VALIDATION) and skip without a Vulkan device, the cross-family
// case also with one family, and the 3D view's with one queue.

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
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "fake_sensor.hpp"
#include "set_on_exit.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/core/testing/two_family_device.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/core/testing/write_image.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_buffer.hpp"
#include "volumetric_kit/core/vulkan/command_pool.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/image.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/core/vulkan/vk_result.hpp"
#include "volumetric_kit/gfx/core/device_requirements.hpp"
#include "volumetric_kit/gfx/core/offscreen_target.hpp"
#include "volumetric_kit/gfx/pipelines/color.hpp"
#include "volumetric_kit/gfx/windowing/frame_loop.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"
#include "volumetric_kit/sensor/testing/nv12_picture.hpp"

namespace volumetric_kit::calib::app {
namespace {

using Code = core::Status::Code;

// `luma` (`width * height` bytes, rows packed) with neutral chroma.
sensor::testing::HostNv12 grey_nv12(std::uint32_t width, std::uint32_t height,
                                    std::vector<std::uint8_t> luma) {
  sensor::testing::HostNv12 picture;
  picture.width = width;
  picture.height = height;
  picture.luma = std::move(luma);
  picture.chroma.assign(std::size_t{(width + 1) / 2} * ((height + 1) / 2) * 2,
                        128);
  return picture;
}

// @p host's NV12 planes in device images written on @p device's queue and
// left in GENERAL, each with a view and SAMPLED usage, as sensor's
// testing::device_picture makes them, but CONCURRENT across @p families, as
// calib_rig's decoded pictures are (sensor's color_queue_families), or
// exclusive to the device's family with none. sensor's are always exclusive,
// and no public helper makes them concurrent. @p luma, when given, is the
// luma image, which the test rewrites as a decoder writes its next picture.
core::Result<sensor::YuvImage> concurrent_picture(
    const core::Device& device, core::Allocator& allocator,
    const sensor::testing::HostNv12& host,
    const std::vector<std::uint32_t>& families,
    std::shared_ptr<core::Image>* luma = nullptr) {
  struct Plane {
    const std::vector<std::uint8_t>* bytes;
    std::uint32_t width, height;
    VkFormat format;
  };
  const Plane planes[2] = {
      {&host.luma, host.width, host.height, VK_FORMAT_R8_UNORM},
      {&host.chroma, (host.width + 1) / 2, (host.height + 1) / 2,
       VK_FORMAT_R8G8_UNORM}};
  sensor::YuvImage picture;
  picture.layout = sensor::YuvLayout::Nv12;
  picture.width = host.width;
  picture.height = host.height;
  for (std::size_t k = 0; k < 2; ++k) {
    core::ImageDesc desc;
    desc.extent = {planes[k].width, planes[k].height};
    desc.format = planes[k].format;
    desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                 VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    desc.queue_families = families.empty() ? nullptr : families.data();
    desc.queue_family_count = static_cast<std::uint32_t>(families.size());
    VKC_ASSIGN(core::Image image, allocator.create_image(desc));
    VKC_TRY(core::test::write_image(
        device, allocator, image, planes[k].bytes->data(),
        planes[k].bytes->size(), VK_IMAGE_LAYOUT_GENERAL));
    auto shared = std::make_shared<core::Image>(std::move(image));
    if (k == 0 && luma != nullptr) *luma = shared;
    picture.image[k] = std::move(shared);
  }
  return picture;
}

// What the window's mosaic step (record_fresh) did in a frame, which starts
// with its snapshot fresh.
struct Step {
  bool recorded = false;
  bool fresh = true;
  std::vector<gfx::windowing::FrameWait> waits;  // the frame's
  core::StageMetrics ran;                        // the tiles' run
};

// Whether @p ran has a row named @p name.
bool has_row(const core::StageMetrics& ran, const char* name) {
  return std::any_of(ran.rows().begin(), ran.rows().end(),
                     [name](const core::StageRow& row) {
                       return std::string(row.name) == name;
                     });
}

// Whether @p step recorded its snapshot in a frame that waits for @p values
// and no other, each at the copies' stage, and timed the tiles.
::testing::AssertionResult RecordedWaitingFor(
    const Step& step, const std::vector<core::TimelinePoint>& values) {
  if (!step.recorded || step.fresh) {
    return ::testing::AssertionFailure()
           << "not recorded" << (step.fresh ? "" : ", yet not fresh");
  }
  if (step.waits.size() != values.size()) {
    return ::testing::AssertionFailure()
           << step.waits.size() << " waits, not " << values.size();
  }
  for (const core::TimelinePoint& value : values) {
    const auto wait =
        std::find_if(step.waits.begin(), step.waits.end(),
                     [&](const gfx::windowing::FrameWait& w) {
                       return w.point.semaphore == value.semaphore;
                     });
    if (wait == step.waits.end()) {
      return ::testing::AssertionFailure() << "no wait on a timeline";
    }
    if (wait->point.value != value.value) {
      return ::testing::AssertionFailure()
             << "waits for " << wait->point.value << ", not " << value.value;
    }
    if (wait->stages != VK_PIPELINE_STAGE_TRANSFER_BIT) {
      return ::testing::AssertionFailure()
             << "waits at stages " << wait->stages << ", not TRANSFER";
    }
  }
  if (!has_row(step.ran, "tiles")) {
    return ::testing::AssertionFailure() << "the tiles are not timed";
  }
  return ::testing::AssertionSuccess();
}

// Whether @p step skipped its snapshot for good: nothing recorded, waited for
// or timed, and the snapshot no longer fresh.
::testing::AssertionResult Skipped(const Step& step) {
  if (step.recorded) return ::testing::AssertionFailure() << "recorded";
  if (step.fresh) return ::testing::AssertionFailure() << "still fresh";
  if (!step.waits.empty()) {
    return ::testing::AssertionFailure() << step.waits.size() << " waits";
  }
  if (!step.ran.empty()) {
    return ::testing::AssertionFailure() << "a stage timed";
  }
  return ::testing::AssertionSuccess();
}

// @p snapshot's set's luma pass's ready value (source::PictureSet::ready).
core::TimelinePoint pass_of(const LiveSnapshot& snapshot) {
  return {snapshot.set->ready.get(), snapshot.set->ready_value};
}

// Whether @p step left its snapshot for a later frame: nothing recorded,
// waited for or timed, and the snapshot still fresh.
::testing::AssertionResult Deferred(const Step& step) {
  if (step.recorded) return ::testing::AssertionFailure() << "recorded";
  if (!step.fresh) return ::testing::AssertionFailure() << "no longer fresh";
  if (!step.waits.empty()) {
    return ::testing::AssertionFailure() << step.waits.size() << " waits";
  }
  if (!step.ran.empty()) {
    return ::testing::AssertionFailure() << "a stage timed";
  }
  return ::testing::AssertionSuccess();
}

// Submits @p cmd, ended, to @p device's queue as gfx's frame loop submits a
// frame -- after @p waits, each at its stages, which the loop takes only once
// reached (core::TimelineWaits::Reached) -- and waits for it.
core::Status submit_frame(const core::Device& device, VkCommandBuffer cmd,
                          const std::vector<gfx::windowing::FrameWait>& waits) {
  std::vector<core::TimelinePoint> points;
  std::vector<VkSemaphore> semaphores;
  std::vector<std::uint64_t> values;
  std::vector<VkPipelineStageFlags> stages;
  for (const gfx::windowing::FrameWait& wait : waits) {
    points.push_back(wait.point);
    semaphores.push_back(wait.point.semaphore->handle());
    values.push_back(wait.point.value);
    stages.push_back(wait.stages);
  }
  VKC_TRY(core::check_timeline_points(device, points, {}, "submit_frame",
                                      core::TimelineWaits::Reached));
  VkTimelineSemaphoreSubmitInfo timeline{};
  timeline.sType = VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO;
  timeline.waitSemaphoreValueCount = static_cast<std::uint32_t>(values.size());
  timeline.pWaitSemaphoreValues = values.data();
  VkSubmitInfo submit{};
  submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit.pNext = &timeline;
  submit.waitSemaphoreCount = static_cast<std::uint32_t>(semaphores.size());
  submit.pWaitSemaphores = semaphores.data();
  submit.pWaitDstStageMask = stages.data();
  submit.commandBufferCount = 1;
  submit.pCommandBuffers = &cmd;
  VKC_ASSIGN(const core::Fence fence, core::Fence::create(device.handle()));
  const VkResult submitted = device.queue_submit(1, &submit, fence.handle());
  if (submitted != VK_SUCCESS) {
    return core::vk_error(submitted, "submit_frame: vkQueueSubmit");
  }
  return fence.wait();
}

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

// A kTileW x kTileH picture of one grey.
sensor::testing::HostNv12 flat_grey(std::uint8_t level) {
  return grey_nv12(
      kTileW, kTileH,
      std::vector<std::uint8_t>(std::size_t{kTileW} * kTileH, level));
}

// A set of one frame per luma, a picture of that grey; none where empty.
struct Set {
  std::shared_ptr<const sensor::FrameSet> frames;
  std::vector<std::weak_ptr<const void>> pixels;  // each frame's
  // Its luma pass's ready value, reached, as a source sets it on a set taken
  // with luma; null for one taken without.
  std::shared_ptr<const core::TimelineSemaphore> ready;
  std::uint64_t ready_value = 0;
  core::TimelinePoint pass() const { return {ready.get(), ready_value}; }
};

// The set as a source hands an array's set out: each frame's NV12 images its
// colour tile.
std::shared_ptr<const source::PictureSet> pictures_of(const Set& made) {
  auto set = std::make_shared<source::PictureSet>();
  for (const std::optional<sensor::RgbdFrame>& frame : made.frames->frames) {
    source::CameraPicture& picture = set->cameras.emplace_back();
    picture.present = frame.has_value();
  }
  set->rgbd = made.frames;
  set->ready = made.ready;
  set->ready_value = made.ready_value;
  return set;
}

std::shared_ptr<const LiveSnapshot> snapshot_of(const Set& set) {
  auto snapshot = std::make_shared<LiveSnapshot>();
  snapshot->set = pictures_of(set);
  const std::size_t n = set.frames->frames.size();
  snapshot->detections.resize(n);
  snapshot->checks.resize(n);
  snapshot->errors.resize(n);
  return snapshot;
}

// The mosaic of `n` kTileW x kTileH tiles on `device`, fitted to kExtent.
std::unique_ptr<viz::RigMosaic> make_mosaic(const core::Device& device,
                                            core::Allocator& allocator,
                                            std::size_t n) {
  core::Result<std::unique_ptr<viz::RigMosaic>> mosaic = viz::RigMosaic::create(
      device, allocator, color_layout(),
      std::vector<sensor::camera::ImageSize>(n, {kTileW, kTileH}));
  EXPECT_TRUE(mosaic.ok()) << mosaic.status().message();
  if (!mosaic.ok()) return nullptr;
  mosaic.value()->view().set_viewport(
      {0.0f, 0.0f},
      {static_cast<float>(kExtent.width), static_cast<float>(kExtent.height)});
  mosaic.value()->view().fit();
  return std::move(mosaic).value();
}

// The window's frame `number` on `device`: its mosaic step records the
// snapshot, fresh, into the mosaic (record_fresh); the mosaic is drawn; the
// frame is submitted after the waits the step gave it (submit_frame); and the
// target's pixels are returned. `step`, when given, gets what the step did;
// without it, the step must have recorded.
std::vector<std::uint8_t> render(const core::Device& device,
                                 core::Allocator& allocator,
                                 viz::RigMosaic& mosaic,
                                 gfx::RetireQueue& retire,
                                 const std::shared_ptr<const LiveSnapshot>& s,
                                 std::uint64_t number, Step* step = nullptr) {
  gfx::OffscreenTargetDesc desc;
  desc.extent = kExtent;
  desc.color_format = kFormat;
  core::Result<gfx::OffscreenTarget> target =
      gfx::OffscreenTarget::create(allocator, desc);
  core::Result<core::CommandPool> pool =
      core::CommandPool::create(device.handle(), device.queue_family());
  EXPECT_TRUE(target.ok() && pool.ok());
  if (!target.ok() || !pool.ok()) return {};
  core::Result<core::CommandBuffer> cmd = pool.value().allocate_primary();
  EXPECT_TRUE(cmd.ok());
  if (!cmd.ok()) return {};
  EXPECT_TRUE(
      cmd.value().begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT).ok());
  const VkCommandBuffer raw = cmd.value().handle();
  gfx::windowing::Frame frame;
  frame.cmd = raw;
  frame.number = number;
  Step mine;
  Step& out = step != nullptr ? *step : mine;
  out = Step{};
  const core::Result<bool> recorded =
      record_fresh(mosaic, retire, frame, s, out.fresh, out.ran);
  EXPECT_TRUE(recorded.ok()) << recorded.status().message();
  if (!recorded.ok()) return {};
  out.recorded = recorded.value();
  out.waits = frame.waits;
  if (step == nullptr) {
    EXPECT_TRUE(out.recorded) << "not recorded";
  }
  target.value().prepare(raw);
  const gfx::RenderTarget rt = target.value().target();
  rt.begin(raw, gfx::RenderTargetBeginInfo{});
  mosaic.draw(raw, kExtent);
  rt.end(raw);
  target.value().record_readback(raw);
  EXPECT_TRUE(cmd.value().end().ok());
  const core::Status submitted = submit_frame(device, raw, frame.waits);
  EXPECT_TRUE(submitted.ok()) << submitted.message();
  if (!submitted.ok()) return {};
  const auto* px = static_cast<const std::uint8_t*>(target.value().pixels());
  return {px, px + std::size_t{kExtent.width} * kExtent.height * 4};
}

// The grey at the middle of tile `i`, as drawn.
int tile_grey(const viz::RigMosaic& mosaic,
              const std::vector<std::uint8_t>& pixels, std::size_t i) {
  const gfx::ui::OverlayRegion region = mosaic.tile_region(i);
  const glm::vec2 middle =
      mosaic.view().image_to_target(region.origin + 0.5f * region.size);
  const auto x = static_cast<std::size_t>(middle.x);
  const auto y = static_cast<std::size_t>(middle.y);
  return pixels[(y * kExtent.width + x) * 4 + 1];
}

class AppLiveView : public core::test::VulkanDeviceTest {
 protected:
  core::DeviceRequirements requirements() const override {
    return gfx::device_requirements();
  }

  // Camera 0's picture is `first` when given. Taken with luma: its pass's
  // value is the fixture's, reached.
  Set make_set(const std::vector<std::optional<std::uint8_t>>& grey,
               std::optional<sensor::YuvImage> first = std::nullopt) {
    auto set = std::make_shared<sensor::FrameSet>();
    Set out;
    if (luma_done_ == nullptr) {
      core::Result<core::TimelineSemaphore> made =
          core::TimelineSemaphore::create(device(), 1);
      EXPECT_TRUE(made.ok()) << made.status().message();
      if (made.ok()) {
        luma_done_ =
            std::make_shared<core::TimelineSemaphore>(std::move(made).value());
      }
    }
    out.ready = luma_done_;
    out.ready_value = 1;
    for (const std::optional<std::uint8_t>& level : grey) {
      if (!level) {
        set->frames.emplace_back();
        out.pixels.emplace_back();
        continue;
      }
      sensor::RgbdFrame frame;
      if (first && set->frames.empty()) {
        frame.color = *first;
      } else {
        core::Result<sensor::YuvImage> picture =
            sensor::testing::device_picture(device(), allocator(),
                                            flat_grey(*level));
        EXPECT_TRUE(picture.ok()) << picture.status().message();
        if (picture.ok()) frame.color = std::move(picture).value();
      }
      auto held = std::make_shared<int>(*level);
      out.pixels.emplace_back(held);
      frame.pixels = std::move(held);
      set->frames.emplace_back(std::move(frame));
    }
    out.frames = std::move(set);
    return out;
  }

  std::vector<std::uint8_t> render(viz::RigMosaic& mosaic,
                                   gfx::RetireQueue& retire,
                                   const std::shared_ptr<const LiveSnapshot>& s,
                                   std::uint64_t number, Step* step = nullptr) {
    return app::render(device(), allocator(), mosaic, retire, s, number, step);
  }

  std::unique_ptr<viz::RigMosaic> make_mosaic(std::size_t n) {
    return app::make_mosaic(device(), allocator(), n);
  }

 private:
  std::shared_ptr<const core::TimelineSemaphore> luma_done_;
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
      record_snapshot(*mosaic, retire, VK_NULL_HANDLE, nullptr, 1).status();
  EXPECT_EQ(refused.domain(), Code::InvalidArgument);
  const Set three = make_set({10, 20, 30});
  refused =
      record_snapshot(*mosaic, retire, VK_NULL_HANDLE, snapshot_of(three), 1)
          .status();
  EXPECT_EQ(refused.domain(), Code::InvalidArgument);
  EXPECT_NE(refused.message().find("a set of 3 pictures for a mosaic of 2"),
            std::string::npos);
  EXPECT_EQ(retire.pending(), 0u);

  // The window's step records nothing that is not fresh, and keeps a
  // snapshot it fails to record fresh, with nothing waited for or timed.
  gfx::windowing::Frame frame;
  frame.number = 1;
  core::StageMetrics ran;
  bool fresh = false;
  const core::Result<bool> stale =
      record_fresh(*mosaic, retire, frame, snapshot_of(three), fresh, ran);
  ASSERT_TRUE(stale.ok()) << stale.status().message();
  EXPECT_FALSE(stale.value());
  fresh = true;
  const core::Result<bool> failed =
      record_fresh(*mosaic, retire, frame, snapshot_of(three), fresh, ran);
  EXPECT_EQ(failed.status().domain(), Code::InvalidArgument);
  EXPECT_TRUE(fresh);
  EXPECT_TRUE(frame.waits.empty());
  EXPECT_TRUE(ran.empty());
  EXPECT_EQ(retire.pending(), 0u);
}

// A decoder's picture that carries a ready value, as NVIDIA's do, is copied
// only once the value is reached, in a frame that waits for it on the device
// at the copy's stage, as gfx's frame loop takes it: before, the snapshot is
// not recorded, nothing is held or timed, it stays fresh, and every tile
// keeps its last picture, so the window records it at a later frame and the
// tile shows what the decoder wrote before setting the value, never what the
// picture held before. A picture without one is copied with it.
TEST_F(AppLiveView, CopiesADecodersPictureInAFrameThatWaitsForItsReadyValue) {
  std::unique_ptr<viz::RigMosaic> mosaic = make_mosaic(2);
  ASSERT_NE(mosaic, nullptr);
  core::Result<core::TimelineSemaphore> timeline =
      core::TimelineSemaphore::create(device());
  ASSERT_TRUE(timeline.ok()) << timeline.status().message();
  gfx::RetireQueue retire(timeline.value());
  const test::SetOnExit retired{timeline.value(), 3};
  core::Result<core::TimelineSemaphore> made =
      core::TimelineSemaphore::create(device());
  ASSERT_TRUE(made.ok()) << made.status().message();
  const auto ready =
      std::make_shared<core::TimelineSemaphore>(std::move(made).value());

  Set first = make_set({40, 120});
  std::vector<std::uint8_t> pixels =
      render(*mosaic, retire, snapshot_of(first), 1);
  ASSERT_FALSE(pixels.empty());

  // Camera 0's next picture, its decoder's copy under way until it sets 1.
  std::shared_ptr<core::Image> luma;
  core::Result<sensor::YuvImage> decoded =
      concurrent_picture(device(), allocator(), flat_grey(90), {}, &luma);
  ASSERT_TRUE(decoded.ok()) << decoded.status().message();
  decoded->ready = ready;
  decoded->ready_value = 1;
  const Set second = make_set({90, 200}, decoded.value());
  const std::shared_ptr<const LiveSnapshot> snapshot = snapshot_of(second);
  Step step;
  pixels = render(*mosaic, retire, snapshot, 2, &step);
  ASSERT_FALSE(pixels.empty());
  EXPECT_TRUE(Deferred(step));  // not yet
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 0), 40, 3);
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 1), 120, 3);
  EXPECT_EQ(mosaic->sets_since_picture(0), std::optional<std::uint64_t>(0));
  ASSERT_TRUE(timeline.value().signal(2).ok());
  retire.poll();
  EXPECT_EQ(retire.pending(), 0u);  // frame 2 holds nothing

  // The copy lands, then the decoder sets the value.
  const sensor::testing::HostNv12 written = flat_grey(150);
  ASSERT_TRUE(core::test::write_image(device(), allocator(), *luma,
                                      written.luma.data(), written.luma.size(),
                                      VK_IMAGE_LAYOUT_GENERAL)
                  .ok());
  ASSERT_TRUE(ready->signal(1).ok());
  pixels = render(*mosaic, retire, snapshot, 3, &step);
  ASSERT_FALSE(pixels.empty());
  EXPECT_TRUE(RecordedWaitingFor(step, {{ready.get(), 1}, second.pass()}));
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 0), 150, 3);
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 1), 200, 3);
  ASSERT_TRUE(timeline.value().signal(3).ok());
  retire.poll();
  EXPECT_EQ(retire.pending(), 0u);
}

// A set taken without luma, as the live worker takes them while the 3D view
// shows, holds pictures no luma pass took over; on NVIDIA, NVDEC's images
// still owned outside Vulkan, which the mosaic refuses. The window's step
// skips it whole, as on the images' return: nothing recorded, refused, held,
// waited for or timed, the snapshot no longer fresh, every tile keeping its
// last picture and its age; the next set taken with luma is copied.
TEST_F(AppLiveView, ShowsNoSetTakenWithoutLumaUntilOneTakenWithIt) {
  std::unique_ptr<viz::RigMosaic> mosaic = make_mosaic(2);
  ASSERT_NE(mosaic, nullptr);
  core::Result<core::TimelineSemaphore> timeline =
      core::TimelineSemaphore::create(device());
  ASSERT_TRUE(timeline.ok()) << timeline.status().message();
  gfx::RetireQueue retire(timeline.value());
  const test::SetOnExit retired{timeline.value(), 3};

  const Set first = make_set({40, 120});
  std::vector<std::uint8_t> pixels =
      render(*mosaic, retire, snapshot_of(first), 1);
  ASSERT_FALSE(pixels.empty());
  EXPECT_EQ(retire.pending(), 1u);

  // Camera 0's picture as NVDEC hands it out, before a pass takes it over.
  core::Result<sensor::YuvImage> decoded =
      concurrent_picture(device(), allocator(), flat_grey(90), {});
  ASSERT_TRUE(decoded.ok()) << decoded.status().message();
  decoded->queue_family = sensor::kQueueFamilyExternal;
  Set without = make_set({90, 200}, decoded.value());
  without.ready = nullptr;  // taken without luma
  Step step;
  pixels = render(*mosaic, retire, snapshot_of(without), 2, &step);
  ASSERT_FALSE(pixels.empty());
  EXPECT_TRUE(Skipped(step));
  EXPECT_TRUE(mosaic->refused().empty());
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 0), 40, 3);
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 1), 120, 3);
  EXPECT_EQ(mosaic->sets_since_picture(0), std::optional<std::uint64_t>(0));
  EXPECT_EQ(retire.pending(), 1u);  // frame 2 holds nothing

  const Set next = make_set({70, 210});
  pixels = render(*mosaic, retire, snapshot_of(next), 3, &step);
  ASSERT_FALSE(pixels.empty());
  EXPECT_TRUE(RecordedWaitingFor(step, {next.pass()}));
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 0), 70, 3);
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 1), 210, 3);
  ASSERT_TRUE(timeline.value().signal(3).ok());
  retire.poll();
  EXPECT_EQ(retire.pending(), 0u);
}

class AppLiveCrossFamily : public core::test::SharedDeviceTest {
 protected:
  core::DeviceRequirements compute_requirements() const override {
    return sensor::device_requirements();
  }
  core::DeviceRequirements graphics_requirements() const override {
    return gfx::device_requirements();
  }
};

// As calib_rig runs: the camera's decoded picture CONCURRENT across both
// families (sensor's color_queue_families), the live worker finding the board
// in it on the compute family, and the window drawing it, one texel a pixel,
// on the graphics family.
TEST_F(AppLiveCrossFamily, TheWindowDrawsWhatTheLiveWorkerReadOnAnotherFamily) {
  core::test::TwoFamilyDevice& two = two_family();
  const std::vector<std::uint32_t> families(two.families().begin(),
                                            two.families().end());
  // A 960 x 544 picture, the mosaic's own size, so the view fits it 1:1.
  constexpr std::uint32_t kW = 960;
  constexpr std::uint32_t kH = 544;
  sensor::camera::CameraModel lens = test::color_like_lens();
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
  core::Result<sensor::YuvImage> made =
      concurrent_picture(two.compute(), two.compute_allocator(),
                         grey_nv12(kW, kH, luma), families);
  ASSERT_TRUE(made.ok()) << made.status().message();
  const sensor::YuvImage picture = std::move(made).value();

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
  core::Result<std::unique_ptr<source::RigSource>> rig =
      source::RigSource::open(std::move(array).value(), two.compute(),
                              two.compute_allocator(), families);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  core::Result<std::unique_ptr<LiveWorker>> live = LiveWorker::create(
      std::move(rig).value(), {lens}, legacy_femto_board(), false, nullptr);
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
  const test::SetOnExit retired{timeline.value(), 1};

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
  const core::Result<std::optional<std::vector<core::TimelinePoint>>> recorded =
      record_snapshot(tiles, retire, raw, snapshot, 1);
  ASSERT_TRUE(recorded.ok()) << recorded.status().message();
  ASSERT_TRUE(recorded->has_value());
  // The pass's value, which the source set on the compute family's queue;
  // the picture carries none of its own.
  ASSERT_EQ(recorded.value()->size(), 1u);
  const core::TimelinePoint pass = recorded.value()->front();
  EXPECT_EQ(pass.semaphore, pass_of(*snapshot).semaphore);
  EXPECT_EQ(pass.value, pass_of(*snapshot).value);
  target.value().prepare(raw);
  const gfx::RenderTarget rt = target.value().target();
  rt.begin(raw, gfx::RenderTargetBeginInfo{});
  tiles.draw(raw, {kW, kH});
  rt.end(raw);
  target.value().record_readback(raw);
  ASSERT_TRUE(cmd.value().end().ok());
  const core::Status drawn = submit_frame(
      two.graphics(), raw, {{pass, VK_PIPELINE_STAGE_TRANSFER_BIT}});
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

// The device calib_rig makes, on whatever queues `SharedDevice::create`
// gives it: two families on MoltenVK, two queues of one family on NVIDIA.
// Skips where both libraries share one queue, as no GPU-side wait on a value
// the host sets later may be submitted there.
class AppLiveTwoQueues : public core::test::VulkanTest {
 protected:
  core::DeviceRequirements requirements() const override {
    return gfx::device_requirements();
  }
  // A cross-queue case, whose 3D-view preparation reads the picture through
  // descriptors, which synchronization validation tracks only at this level.
  core::test::Validation validation() const override {
    return core::test::Validation::ShaderAccesses;
  }
  void SetUp() override {
    core::test::VulkanTest::SetUp();
    if (base_setup_incomplete()) return;
    core::Result<std::unique_ptr<core::test::TwoFamilyDevice>> made =
        core::test::TwoFamilyDevice::create(sensor::device_requirements(),
                                            gfx::device_requirements(),
                                            active_validation());
    ASSERT_TRUE(made.ok()) << made.status().message();
    two_ = std::move(made).value();
    if (two_->shared().plan() == core::QueuePlan::SharedQueue) {
      GTEST_SKIP() << "one queue for both libraries: "
                   << two_->shared().summary();
    }
  }
  core::test::TwoFamilyDevice& two() { return *two_; }

 private:
  std::unique_ptr<core::test::TwoFamilyDevice> two_;
};

// As calib_rig runs in the Images view with a camera whose picture shows as
// a grey tile (nvJPEG's planes, a capture card's V210; here host luma): the
// live worker's luma pass writes the tile on the compute queue, and the
// window copies it into the mosaic on the graphics queue in a frame that
// waits on the device for the value the source set after the pass.
TEST_F(AppLiveTwoQueues, TheWindowCopiesAGreyTileAfterItsLumaPass) {
  core::test::TwoFamilyDevice& two = this->two();
  const std::vector<std::uint32_t> families(two.families().begin(),
                                            two.families().end());
  sensor::camera::CameraModel lens;
  lens.size = {kTileW, kTileH};
  lens.intrinsics = {30.0, 30.0, 16.0, 12.0};
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
  core::Result<std::unique_ptr<source::RigSource>> rig =
      source::RigSource::open(std::move(array).value(), two.compute(),
                              two.compute_allocator(), families);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  core::Result<std::unique_ptr<LiveWorker>> live = LiveWorker::create(
      std::move(rig).value(), {lens}, legacy_femto_board(), false, nullptr);
  ASSERT_TRUE(live.ok()) << live.status().message();
  std::unique_ptr<viz::RigMosaic> mosaic =
      make_mosaic(two.graphics(), two.graphics_allocator(), 1);
  ASSERT_NE(mosaic, nullptr);
  core::Result<core::TimelineSemaphore> timeline =
      core::TimelineSemaphore::create(two.graphics());
  ASSERT_TRUE(timeline.ok()) << timeline.status().message();
  gfx::RetireQueue retire(timeline.value());
  const test::SetOnExit retired{timeline.value(), 1};

  auto luma = std::make_shared<std::vector<std::uint8_t>>(
      std::size_t{kTileW} * kTileH, 90);
  sensor::RgbdFrame frame;
  frame.color.layout = sensor::YuvLayout::Luma;
  frame.color.width = kTileW;
  frame.color.height = kTileH;
  frame.color.stride[0] = kTileW;
  frame.color.host = luma->data();
  frame.color_camera = lens;
  frame.timestamp_ns = 1'760'000'000'000'000'000ull;
  frame.pixels = luma;
  camera.push(std::move(frame));
  std::shared_ptr<const LiveSnapshot> snapshot;
  const auto give_up =
      std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (!snapshot && std::chrono::steady_clock::now() < give_up) {
    snapshot = live.value()->take_snapshot();
    if (!snapshot) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_NE(snapshot, nullptr);
  EXPECT_TRUE(live.value()->take_errors().empty());
  ASSERT_NE(snapshot->set->cameras[0].tile, nullptr);  // a grey tile
  ASSERT_NE(snapshot->set->ready, nullptr);

  Step step;
  const std::vector<std::uint8_t> pixels =
      render(two.graphics(), two.graphics_allocator(), *mosaic, retire,
             snapshot, 1, &step);
  ASSERT_FALSE(pixels.empty());
  EXPECT_TRUE(RecordedWaitingFor(step, {pass_of(*snapshot)}));
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 0), 90, 3);

  live.value()->stop();
  snapshot.reset();
  ASSERT_TRUE(timeline.value().signal(1).ok());
  retire.poll();
  EXPECT_EQ(retire.pending(), 0u);
}

// As calib_rig runs with the 3D view shown: a camera's decoded picture
// carries a ready value its decoder sets later, as NVIDIA's do. The live
// worker, not detecting, takes the set with no luma pass and publishes it at
// once, reading nothing; the 3D view's worker, offered the set, prepares it
// on the compute queue only once the value is set, its batch waiting on the
// device; and the window, back on the images before then, does not show the
// set, its tile keeping the last set's taken with luma, nor the next set
// taken without luma; the next taken with luma is copied on the graphics
// queue in a frame that waits for its pass's value and its decoder's. The
// picture is one the mosaic takes, standing in for any decoder's: NVIDIA's
// own, taken with no luma pass, are still owned outside Vulkan or held in a
// buffer, and the mosaic refuses them.
TEST_F(AppLiveTwoQueues, TheImagesShowNoSetTakenWhileThe3DViewShowed) {
  core::test::TwoFamilyDevice& two = this->two();
  // CONCURRENT across two families; one family's twice collapses.
  const std::vector<std::uint32_t> families(two.families().begin(),
                                            two.families().end());
  core::Result<core::TimelineSemaphore> made =
      core::TimelineSemaphore::create(two.compute());
  ASSERT_TRUE(made.ok()) << made.status().message();
  const auto ready =
      std::make_shared<core::TimelineSemaphore>(std::move(made).value());
  core::Result<sensor::YuvImage> decoded = concurrent_picture(
      two.compute(), two.compute_allocator(), flat_grey(90), families);
  ASSERT_TRUE(decoded.ok()) << decoded.status().message();
  decoded->ready = ready;
  decoded->ready_value = 1;

  sensor::camera::CameraModel lens;
  lens.size = {kTileW, kTileH};
  lens.intrinsics = {30.0, 30.0, 16.0, 12.0};
  sensor::camera::CameraModel depth_lens;
  depth_lens.size = {16, 12};
  depth_lens.intrinsics = {15.0, 15.0, 8.0, 6.0};
  sensor::SensorInfo info;
  info.id = "SYN0";
  info.color = lens;
  info.depth = depth_lens;
  info.fps = 25;
  auto fake = std::make_unique<test::FakeSensor>(info);
  test::FakeSensor& camera = *fake;
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors;
  sensors.push_back(std::move(fake));
  core::Result<sensor::SensorArray> array =
      sensor::SensorArray::open(std::move(sensors), {});
  ASSERT_TRUE(array.ok()) << array.status().message();
  core::Result<std::unique_ptr<source::RigSource>> rig =
      source::RigSource::open(std::move(array).value(), two.compute(),
                              two.compute_allocator(), families);
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  CloudOptions every_set;
  every_set.live_every = 1;
  core::Result<std::unique_ptr<CloudWorker>> cloud = CloudWorker::create(
      two.compute(), two.compute_allocator(), 1, families, every_set);
  ASSERT_TRUE(cloud.ok()) << cloud.status().message();
  core::Result<std::unique_ptr<LiveWorker>> live =
      LiveWorker::create(std::move(rig).value(), {lens}, legacy_femto_board(),
                         false, nullptr, cloud.value().get());
  ASSERT_TRUE(live.ok()) << live.status().message();
  std::unique_ptr<viz::RigMosaic> mosaic =
      make_mosaic(two.graphics(), two.graphics_allocator(), 1);
  ASSERT_NE(mosaic, nullptr);
  core::Result<core::TimelineSemaphore> timeline =
      core::TimelineSemaphore::create(two.graphics());
  ASSERT_TRUE(timeline.ok()) << timeline.status().message();
  gfx::RetireQueue retire(timeline.value());
  const test::SetOnExit retired{timeline.value(), 3};
  const test::SetOnExit decoded_by_then{*ready, 1};  // before the workers stop

  auto depth = std::make_shared<std::vector<std::uint16_t>>(16 * 12, 1500);
  const auto push = [&](const sensor::YuvImage& picture, std::uint64_t tick) {
    sensor::RgbdFrame frame;
    frame.color = picture;
    frame.color_camera = lens;
    frame.depth = depth->data();
    frame.depth_camera = depth_lens;
    frame.metres_per_unit = 0.001f;
    frame.min_depth = 0.5f;
    frame.max_depth = 3.86f;
    frame.timestamp_ns = 1'760'000'000'000'000'000ull + tick * 40'000'000;
    frame.sequence = tick;
    frame.pixels = depth;
    camera.push(std::move(frame));
  };
  const auto give_up =
      std::chrono::steady_clock::now() + std::chrono::seconds(30);
  const auto next_snapshot = [&](std::uint64_t after) {
    std::shared_ptr<const LiveSnapshot> s;
    while (std::chrono::steady_clock::now() < give_up) {
      s = live.value()->take_snapshot();
      if (s != nullptr && s->number > after) return s;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return std::shared_ptr<const LiveSnapshot>();
  };
  const auto prepared = [&](std::uint64_t count) {
    while (cloud.value()->prepared() < count &&
           std::chrono::steady_clock::now() < give_up) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return cloud.value()->prepared() >= count;
  };

  // The images first: a set taken with luma, copied after its pass.
  core::Result<sensor::YuvImage> earlier = concurrent_picture(
      two.compute(), two.compute_allocator(), flat_grey(40), families);
  ASSERT_TRUE(earlier.ok()) << earlier.status().message();
  push(earlier.value(), 1);
  const std::shared_ptr<const LiveSnapshot> first = next_snapshot(0);
  ASSERT_NE(first, nullptr);
  ASSERT_NE(first->set->ready, nullptr);
  ASSERT_TRUE(prepared(1));
  Step step;
  std::vector<std::uint8_t> pixels =
      render(two.graphics(), two.graphics_allocator(), *mosaic, retire, first,
             1, &step);
  ASSERT_FALSE(pixels.empty());
  EXPECT_TRUE(RecordedWaitingFor(step, {pass_of(*first)}));
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 0), 40, 3);

  // The window shows the 3D view alone: the board is not searched for. A
  // set is taken after that, whatever take was under way, so the next is
  // taken with no luma pass.
  live.value()->set_detecting(false);
  push(earlier.value(), 2);
  const std::shared_ptr<const LiveSnapshot> second =
      next_snapshot(first->number);
  ASSERT_NE(second, nullptr);
  ASSERT_TRUE(prepared(2));
  (void)cloud.value()->take_snapshot();
  push(decoded.value(), 3);

  // Every check before the value is set is an EXPECT, so a failing one
  // leaves no batch waiting for it.
  std::shared_ptr<const LiveSnapshot> snapshot = next_snapshot(second->number);
  EXPECT_NE(snapshot, nullptr);
  if (snapshot != nullptr) {
    EXPECT_FALSE(has_row(snapshot->stages, "luma"));  // no luma pass
    EXPECT_EQ(snapshot->set->ready, nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    EXPECT_EQ(cloud.value()->prepared(), 2u);
    // Back on the images: not shown, the tile keeping the first set's.
    pixels = render(two.graphics(), two.graphics_allocator(), *mosaic, retire,
                    snapshot, 2, &step);
    EXPECT_FALSE(pixels.empty());
    EXPECT_TRUE(Skipped(step));
    if (!pixels.empty()) {
      EXPECT_NEAR(tile_grey(*mosaic, pixels, 0), 40, 3);
    }
    EXPECT_TRUE(mosaic->refused().empty());
  }

  // The decoder sets the value: the 3D view's worker reads the picture.
  ASSERT_TRUE(ready->signal(1).ok());
  ASSERT_NE(snapshot, nullptr);
  ASSERT_TRUE(prepared(3));
  EXPECT_TRUE(cloud.value()->take_errors().empty());
  std::shared_ptr<const CloudSnapshot> points = cloud.value()->take_snapshot();
  ASSERT_NE(points, nullptr);
  EXPECT_EQ(points->number, 3u);
  EXPECT_TRUE(points->color);
  ASSERT_TRUE(points->frames[0].has_value());
  EXPECT_NE(points->frames[0]->color, nullptr);

  // The images again: past a set that may be taken without luma, the next
  // is taken with it, and copied after its pass's value and its decoder's.
  live.value()->set_detecting(true);
  push(decoded.value(), 4);
  const std::shared_ptr<const LiveSnapshot> turning =
      next_snapshot(snapshot->number);
  ASSERT_NE(turning, nullptr);
  push(decoded.value(), 5);
  const std::shared_ptr<const LiveSnapshot> back =
      next_snapshot(turning->number);
  ASSERT_NE(back, nullptr);
  ASSERT_NE(back->set->ready, nullptr);
  pixels = render(two.graphics(), two.graphics_allocator(), *mosaic, retire,
                  back, 3, &step);
  ASSERT_FALSE(pixels.empty());
  EXPECT_TRUE(RecordedWaitingFor(step, {pass_of(*back), {ready.get(), 1}}));
  EXPECT_NEAR(tile_grey(*mosaic, pixels, 0), 90, 3);
  EXPECT_TRUE(live.value()->take_errors().empty());

  live.value()->stop();
  points.reset();
  snapshot.reset();
  cloud.value()->stop();
  ASSERT_TRUE(timeline.value().signal(3).ok());
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

// Each camera's panel takes its picture's presence, check, stillness and
// counters, as sensor keeps them: sets the source skipped, newest winning,
// and frames dropped are no loss; a frame that failed is.
TEST(AppLiveViewModels, GivesEachCameraItsLiveState) {
  LiveSnapshot snapshot;
  auto set = std::make_shared<source::PictureSet>();
  set->cameras.resize(2);
  set->cameras[0].present = true;
  snapshot.set = set;
  snapshot.checks.resize(2);
  snapshot.checks[0].verdict = ViewVerdict::Good;
  snapshot.stillness_px = {0.25, std::nullopt};
  snapshot.stats.unmatched = 12;  // sets skipped
  snapshot.stats.sensors.resize(2);
  snapshot.stats.sensors[0].dropped = 30;
  snapshot.stats.sensors[1].received = 42;
  snapshot.stats.sensors[1].failed = 2;
  const core::Result<std::vector<viz::CameraLive>> cameras =
      camera_lives(snapshot);
  ASSERT_TRUE(cameras.ok());
  ASSERT_EQ(cameras->size(), 2u);
  EXPECT_TRUE(cameras.value()[0].present);
  EXPECT_FALSE(cameras.value()[1].present);
  EXPECT_EQ(cameras.value()[0].check.verdict, ViewVerdict::Good);
  EXPECT_EQ(cameras.value()[0].stillness_px, 0.25);
  EXPECT_FALSE(cameras.value()[1].stillness_px.has_value());
  EXPECT_EQ(cameras.value()[1].frames.received, 42u);
  const quality::CameraSummary session;
  const auto frames_tone = [&](std::size_t i) {
    const core::Result<std::vector<viz::PanelLine>> lines =
        viz::camera_panel(cameras.value()[i], session, legacy_femto_board());
    EXPECT_TRUE(lines.ok());
    for (const viz::PanelLine& line : lines.value()) {
      if (line.label == "frames") return line.tone;
    }
    return viz::Tone::Muted;
  };
  EXPECT_EQ(frames_tone(0), viz::Tone::Normal);
  EXPECT_EQ(frames_tone(1), viz::Tone::Warn);
}

// The set panel's times: each camera's picture's less the set's, either way
// round, none where it has none; and the set's spread.
TEST(AppLiveViewModels, GivesTheSetItsTimes) {
  source::PictureSet set;
  set.timestamp_ns = 1'000'000'000;
  set.spread_ns = 16'600'000;
  set.cameras = {{true, nullptr, 1'000'000'000},
                 {true, nullptr, 1'000'000'000 - 16'600'000},
                 {false, nullptr, 0}};
  const core::Result<viz::SetLive> live = set_live(set);
  ASSERT_TRUE(live.ok());
  EXPECT_EQ(live->spread_ns, 16'600'000u);
  ASSERT_EQ(live->skew_ns.size(), 3u);
  EXPECT_EQ(live->skew_ns[0], std::optional<std::int64_t>(0));
  EXPECT_EQ(live->skew_ns[1], std::optional<std::int64_t>(-16'600'000));
  EXPECT_FALSE(live->skew_ns[2].has_value());
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
  capture::SessionCamera info;
  const sensor::camera::CameraModel lens = test::color_like_lens();
  info.depth = test::depth_like_lens();
  info.depth_to_color[3][0] = -0.032;  // the depth camera 32 mm to one side
  constexpr std::size_t kCameras = gfx::pipelines::kCategoricalPaletteSize + 3;
  constexpr std::size_t kReference = 2;
  std::vector<std::uint32_t> tints;
  for (std::size_t i = 0; i < kCameras; ++i) {
    const viz::SceneCamera camera =
        scene_camera(info, lens, i, i == kReference);
    EXPECT_EQ(camera.tint, gfx::pipelines::categorical_rgba8(i)) << i;
    EXPECT_EQ(camera.color_to_world.has_value(), i == kReference) << i;
    EXPECT_TRUE(camera.shown) << i;
    EXPECT_EQ(camera.color.size.width, lens.size.width) << i;
    EXPECT_EQ(camera.color.intrinsics.fx, lens.intrinsics.fx) << i;
    ASSERT_TRUE(camera.depth.has_value()) << i;
    EXPECT_EQ(camera.depth->intrinsics.fx, info.depth->intrinsics.fx) << i;
    EXPECT_EQ(camera.depth_to_color, info.depth_to_color) << i;
    tints.push_back(camera.tint);
  }
  EXPECT_EQ(scene_camera(info, lens, 0, true).color_to_world,
            sensor::camera::Mat4d(1.0));
  // A camera without depth, as a stereo pair's, is drawn as its lens alone.
  capture::SessionCamera pair_camera;
  EXPECT_FALSE(scene_camera(pair_camera, lens, 0, true).depth.has_value());
  std::sort(tints.begin(), tints.end());
  EXPECT_EQ(std::adjacent_find(tints.begin(), tints.end()), tints.end());
}

// At a result every camera is placed at its own pose, in rig order, whatever
// its id, the reference among them; each keeps its models, tint and toggle.
// Poses for another number of cameras move none.
TEST(AppLiveViewModels, PlacesEveryCameraAtItsPoseInRigOrder) {
  capture::SessionCamera info;
  const sensor::camera::CameraModel lens = test::color_like_lens();
  info.depth = test::depth_like_lens();
  for (const std::size_t n : {1u, 3u, 5u}) {
    SCOPED_TRACE(n);
    const std::size_t reference = n / 2;
    std::vector<viz::SceneCamera> cameras;
    std::vector<sensor::camera::Mat4d> poses;
    for (std::size_t i = 0; i < n; ++i) {
      cameras.push_back(scene_camera(info, lens, i, i == reference));
      cameras.back().shown = i % 2 == 0;
      cameras.back().depth_to_color[3][0] = -0.01 * static_cast<double>(i);
      // A turn about y and a step along x, each camera's own.
      const double turn = 0.3 * static_cast<double>(i);
      sensor::camera::Mat4d pose(1.0);
      pose[0][0] = std::cos(turn);
      pose[0][2] = -std::sin(turn);
      pose[2][0] = std::sin(turn);
      pose[2][2] = std::cos(turn);
      pose[3][0] = 0.5 * static_cast<double>(i);
      poses.push_back(i == reference ? sensor::camera::Mat4d(1.0) : pose);
    }
    const std::vector<viz::SceneCamera> before = cameras;

    std::vector<sensor::camera::Mat4d> extra = poses;
    extra.push_back(sensor::camera::Mat4d(1.0));
    const std::vector<sensor::camera::Mat4d> fewer(poses.begin(),
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
      EXPECT_EQ(cameras[i].color.intrinsics.fx, lens.intrinsics.fx);
      EXPECT_TRUE(cameras[i].depth.has_value());
    }
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::app
