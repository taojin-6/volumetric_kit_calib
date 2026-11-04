// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The source tier on a device: a sensor array of fake cameras (NV12 device
// images, host luma) and a fake stereo sensor of host luma pairs, each set's
// luma and half made by sensor's LumaPass and read back. Each set's pictures,
// times and spread; its tiles, held by the set past the next take; its luma
// pass's ready value, reached, and none for a set taken without luma; the
// half against a host reference; newest wins; a decoder's picture read only
// once its ready value is set, and the set's own value set only after the
// pass; the stereo source's refusals and its skew bound; a poll's failure
// against a picture's; the ChArUco board found in the GPU's half as in the
// CPU's; and sample main 4 of the design, reading a source.
// Runs under the validation layer when the environment asks
// (VKC_TEST_VALIDATION); skips without a Vulkan device.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "detect_helpers.hpp"
#include "fake_sensor.hpp"
#include "fake_stereo.hpp"
#include "set_on_exit.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/detect/board.hpp"
#include "volumetric_kit/calib/source/rig_source.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/core/testing/write_image.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/image.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"
#include "volumetric_kit/sensor/prep/picture_readback.hpp"
#include "volumetric_kit/sensor/testing/nv12_picture.hpp"

namespace volumetric_kit::calib::source {
namespace {

namespace camera = sensor::camera;
using Code = core::Status::Code;

constexpr std::uint32_t kWidth = 64;
constexpr std::uint32_t kHeight = 36;
constexpr std::uint64_t kStartNs = 1'760'000'000'000'000'000ull;
constexpr std::uint64_t kPeriodNs = 40'000'000;  // 25 fps
constexpr auto kPatience = std::chrono::seconds(30);

// A picture's bytes: each pixel a function of its place and `seed`.
std::vector<std::uint8_t> pattern(std::uint32_t width, std::uint32_t height,
                                  int seed) {
  std::vector<std::uint8_t> bytes(std::size_t{width} * height);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      bytes[std::size_t{y} * width + x] =
          static_cast<std::uint8_t>((x * 7 + y * 13 + seed * 31) % 251);
    }
  }
  return bytes;
}

// The half of packed rows `full` as the GPU makes it (test::halved), packed.
std::vector<std::uint8_t> half_of(const std::vector<std::uint8_t>& full,
                                  std::uint32_t width, std::uint32_t height) {
  const cv::Mat picture(static_cast<int>(height), static_cast<int>(width),
                        CV_8UC1, const_cast<std::uint8_t*>(full.data()));
  const cv::Mat half = test::halved(picture);
  return {half.data, half.data + half.total()};
}

// The rows of `view`, packed.
std::vector<std::uint8_t> rows_of(const detect::LumaView& view) {
  std::vector<std::uint8_t> bytes;
  if (view.data == nullptr) return bytes;
  for (std::uint32_t y = 0; y < view.height; ++y) {
    const std::uint8_t* row = view.data + std::size_t{y} * view.stride;
    bytes.insert(bytes.end(), row, row + view.width);
  }
  return bytes;
}

class SourceRig : public core::test::VulkanDeviceTest {
 protected:
  core::DeviceRequirements requirements() const override {
    return sensor::device_requirements();
  }

  // The bytes of a set's tile, read back on a readback of the test's own.
  std::vector<std::uint8_t> tile_bytes(const sensor::YuvImage& tile) {
    if (readback_ == nullptr) {
      core::Result<std::unique_ptr<sensor::PictureReadback>> made =
          sensor::PictureReadback::create(device(), allocator());
      EXPECT_TRUE(made.ok()) << made.status().message();
      if (!made.ok()) return {};
      readback_ = std::move(made).value();
    }
    sensor::YuvImage copy = tile;
    core::Result<sensor::PictureView> view =
        readback_->read(copy, sensor::PictureReadback::Planes::Luma);
    EXPECT_TRUE(view.ok()) << view.status().message();
    if (!view.ok()) return {};
    return rows_of({view->width, view->height, view->stride, view->luma});
  }

  // A stereo source over a fake pair of `kWidth` x `kHeight` cameras.
  std::unique_ptr<RigSource> stereo(
      test::FakeStereo** fake = nullptr,
      sensor::StereoInfo info = test::uncalibrated_pair_info()) {
    auto sensor = std::make_unique<test::FakeStereo>(std::move(info));
    if (fake != nullptr) *fake = sensor.get();
    StereoMember member{std::move(sensor), {"bm-1", "bm-2"}};
    member.sizes = {{{kWidth, kHeight}, {kWidth, kHeight}}};
    core::Result<std::unique_ptr<RigSource>> rig =
        RigSource::open(std::move(member), device(), allocator());
    EXPECT_TRUE(rig.ok()) << rig.status().message();
    return rig.ok() ? std::move(rig).value() : nullptr;
  }

  std::unique_ptr<sensor::PictureReadback> readback_;
};

// --- A stereo sensor --------------------------------------------------------

// A pair: both cameras, each picture's time, the spread; each one's luma on
// the host as sent and its half the GPU's, and its tile on the device, held by
// the set; the counters.
TEST_F(SourceRig, AStereoPairComesAsTwoGreyPictures) {
  test::FakeStereo* fake = nullptr;
  std::unique_ptr<RigSource> rig = stereo(&fake);
  ASSERT_NE(rig, nullptr);
  ASSERT_EQ(rig->cameras().size(), 2u);
  EXPECT_EQ(rig->cameras()[0].id, "bm-1");
  EXPECT_EQ(rig->cameras()[1].id, "bm-2");
  for (const SourceCamera& camera : rig->cameras()) {
    EXPECT_EQ(camera.size.width, kWidth);
    EXPECT_EQ(camera.size.height, kHeight);
    EXPECT_FALSE(camera.factory.has_value());
    EXPECT_FALSE(camera.depth);
  }
  EXPECT_EQ(rig->max_skew_ns(), std::optional<std::uint64_t>(16'666'666));
  ASSERT_TRUE(rig->start().ok());
  EXPECT_TRUE(fake->started());

  core::Result<std::optional<PictureSet>> none = rig->take();
  ASSERT_TRUE(none.ok()) << none.status().message();
  EXPECT_FALSE(none.value().has_value());

  const std::vector<std::uint8_t> left = pattern(kWidth, kHeight, 1);
  const std::vector<std::uint8_t> right = pattern(kWidth, kHeight, 2);
  fake->push(
      test::host_pair(kWidth, kHeight, left, right, kStartNs, -8'200'000, 77));
  core::StageMetrics stages;
  core::Result<std::optional<PictureSet>> taken = rig->take(&stages);
  ASSERT_TRUE(taken.ok()) << taken.status().message();
  ASSERT_TRUE(taken.value().has_value());
  const PictureSet& set = *taken.value();
  EXPECT_EQ(set.sequence, 77u);
  EXPECT_EQ(set.timestamp_ns, kStartNs);
  EXPECT_EQ(set.spread_ns, 8'200'000u);
  EXPECT_EQ(set.rgbd, nullptr);
  ASSERT_EQ(set.cameras.size(), 2u);
  EXPECT_TRUE(set.complete());
  EXPECT_EQ(set.cameras[0].timestamp_ns, kStartNs);
  EXPECT_EQ(set.cameras[1].timestamp_ns, kStartNs - 8'200'000);
  ASSERT_NE(set.owner, nullptr);
  std::vector<std::string> rows;
  for (const core::StageRow& row : stages.rows()) rows.emplace_back(row.name);
  EXPECT_EQ(rows, (std::vector<std::string>{"poll", "luma", "readback"}));

  ASSERT_EQ(rig->host().size(), 2u);
  const std::vector<std::uint8_t>* sent[2] = {&left, &right};
  for (std::size_t i = 0; i < 2; ++i) {
    SCOPED_TRACE("camera " + std::to_string(i));
    const HostLuma& host = rig->host()[i];
    EXPECT_TRUE(host.full_range);
    EXPECT_EQ(host.luma.width, kWidth);
    EXPECT_EQ(host.half.width, kWidth / 2);
    EXPECT_EQ(host.half.height, kHeight / 2);
    EXPECT_EQ(rows_of(host.luma), *sent[i]);
    EXPECT_EQ(rows_of(host.half), half_of(*sent[i], kWidth, kHeight));
    ASSERT_NE(set.cameras[i].tile, nullptr);
    EXPECT_EQ(set.cameras[i].tile->layout, sensor::YuvLayout::Luma);
    EXPECT_EQ(tile_bytes(*set.cameras[i].tile), *sent[i]);
  }
  const sensor::SensorArrayStats stats = rig->stats();
  EXPECT_EQ(stats.sets, 1u);
  ASSERT_EQ(stats.sensors.size(), 2u);
  EXPECT_EQ(stats.sensors[0].delivered, 1u);
  EXPECT_EQ(stats.sensors[1].delivered, 1u);
  rig->stop();
  EXPECT_FALSE(fake->started());
}

// A set kept past the next take still holds its own tiles, on the device:
// it points at nothing on the host the next take reuses, which ASan would
// see, and its tiles are not written again while it lives.
TEST_F(SourceRig, AKeptSetHoldsItsTilesPastTheNextTake) {
  test::FakeStereo* fake = nullptr;
  std::unique_ptr<RigSource> rig = stereo(&fake);
  ASSERT_NE(rig, nullptr);
  ASSERT_TRUE(rig->start().ok());
  const std::vector<std::uint8_t> first = pattern(kWidth, kHeight, 3);
  fake->push(test::host_pair(kWidth, kHeight, first, first, kStartNs, 0, 1));
  core::Result<std::optional<PictureSet>> kept = rig->take();
  ASSERT_TRUE(kept.ok() && kept.value().has_value());
  const auto set = std::make_shared<const PictureSet>(*kept.value());
  const std::weak_ptr<const void> owner = set->owner;

  for (int k = 0; k < 3; ++k) {
    const std::vector<std::uint8_t> next = pattern(kWidth, kHeight, 10 + k);
    fake->push(test::host_pair(kWidth, kHeight, next, next,
                               kStartNs + (k + 1) * kPeriodNs, 0, 2 + k));
    core::Result<std::optional<PictureSet>> later = rig->take();
    ASSERT_TRUE(later.ok() && later.value().has_value());
    EXPECT_EQ(rows_of(rig->host()[0].luma), next);
  }
  kept.value().reset();
  // Every field of the set kept, read after the takes that followed it.
  EXPECT_EQ(set->sequence, 1u);
  EXPECT_EQ(set->timestamp_ns, kStartNs);
  for (const CameraPicture& picture : set->cameras) {
    EXPECT_TRUE(picture.present);
    EXPECT_EQ(picture.timestamp_ns, kStartNs);
    ASSERT_NE(picture.tile, nullptr);
    EXPECT_EQ(tile_bytes(*picture.tile), first);
  }
  EXPECT_FALSE(owner.expired());
}

// Pairs come faster than they are taken: the newest is taken, the one it
// replaced counted dropped, not lost.
TEST_F(SourceRig, AStereoSourceTakesTheNewestPair) {
  test::FakeStereo* fake = nullptr;
  std::unique_ptr<RigSource> rig = stereo(&fake);
  ASSERT_NE(rig, nullptr);
  ASSERT_TRUE(rig->start().ok());
  for (std::uint64_t k = 0; k < 3; ++k) {
    fake->push(test::host_pair(kWidth, kHeight,
                               pattern(kWidth, kHeight, static_cast<int>(k)),
                               pattern(kWidth, kHeight, static_cast<int>(k)),
                               kStartNs + k * kPeriodNs, 0, k));
  }
  core::Result<std::optional<PictureSet>> taken = rig->take();
  ASSERT_TRUE(taken.ok() && taken.value().has_value());
  EXPECT_EQ(taken.value()->sequence, 2u);
  EXPECT_EQ(rows_of(rig->host()[1].luma), pattern(kWidth, kHeight, 2));
  core::Result<std::optional<PictureSet>> after = rig->take();
  ASSERT_TRUE(after.ok());
  EXPECT_FALSE(after.value().has_value());
  const sensor::SensorArrayStats stats = rig->stats();
  EXPECT_EQ(stats.sets, 1u);
  EXPECT_EQ(stats.sensors[0].dropped, 2u);
  EXPECT_EQ(stats.sensors[0].failed, 0u);
}

// Taken without luma, a set has its times and no tiles, and the host has no
// pictures: only the poll runs.
TEST_F(SourceRig, ASetTakenWithoutLumaHasItsTimesAlone) {
  test::FakeStereo* fake = nullptr;
  std::unique_ptr<RigSource> rig = stereo(&fake);
  ASSERT_NE(rig, nullptr);
  ASSERT_TRUE(rig->start().ok());
  fake->push(test::host_pair(kWidth, kHeight, pattern(kWidth, kHeight, 1),
                             pattern(kWidth, kHeight, 1), kStartNs, 5'000'000,
                             9));
  core::StageMetrics stages;
  core::Result<std::optional<PictureSet>> taken = rig->take(&stages, false);
  ASSERT_TRUE(taken.ok() && taken.value().has_value());
  EXPECT_EQ(taken.value()->spread_ns, 5'000'000u);
  for (const CameraPicture& picture : taken.value()->cameras) {
    EXPECT_TRUE(picture.present);
    EXPECT_EQ(picture.tile, nullptr);
  }
  EXPECT_EQ(taken.value()->ready, nullptr);  // no pass to wait for
  EXPECT_TRUE(rig->host().empty());
  ASSERT_EQ(stages.rows().size(), 1u);
  EXPECT_STREQ(stages.rows()[0].name, "poll");
}

// A set taken with luma carries its pass's ready value, set after the pass on
// the device's queue and already reached when the set comes, for a reader on
// another queue to wait for there; each set a higher value on the source's
// one timeline, held by the set.
TEST_F(SourceRig, ASetTakenWithLumaCarriesItsPassReadyValue) {
  test::FakeStereo* fake = nullptr;
  std::unique_ptr<RigSource> rig = stereo(&fake);
  ASSERT_NE(rig, nullptr);
  ASSERT_TRUE(rig->start().ok());
  std::optional<PictureSet> sets[2];
  for (std::uint64_t k = 0; k < 2; ++k) {
    SCOPED_TRACE("set " + std::to_string(k));
    fake->push(test::host_pair(kWidth, kHeight,
                               pattern(kWidth, kHeight, static_cast<int>(k)),
                               pattern(kWidth, kHeight, 1), kStartNs, 0, k));
    core::Result<std::optional<PictureSet>> taken = rig->take();
    ASSERT_TRUE(taken.ok()) << taken.status().message();
    ASSERT_TRUE(taken.value().has_value());
    sets[k] = std::move(taken).value();
    ASSERT_NE(sets[k]->ready, nullptr);
    const core::Result<std::uint64_t> now = sets[k]->ready->value();
    ASSERT_TRUE(now.ok()) << now.status().message();
    EXPECT_GE(now.value(), sets[k]->ready_value);
  }
  EXPECT_EQ(sets[1]->ready, sets[0]->ready);
  EXPECT_GT(sets[1]->ready_value, sets[0]->ready_value);
  EXPECT_GT(sets[0]->ready_value, 0u);
}

// The sensor's failure is the take's, and says so through failed(); a picture
// of another size drops its set alone, uncounted, and the source goes on.
TEST_F(SourceRig, APollsFailureIsNotAPicturesFailure) {
  test::FakeStereo* fake = nullptr;
  std::unique_ptr<RigSource> rig = stereo(&fake);
  ASSERT_NE(rig, nullptr);
  ASSERT_TRUE(rig->start().ok());
  fake->push(test::host_pair(kWidth - 2, kHeight,
                             pattern(kWidth - 2, kHeight, 1),
                             pattern(kWidth - 2, kHeight, 1), kStartNs, 0, 1));
  core::Result<std::optional<PictureSet>> wrong = rig->take();
  ASSERT_FALSE(wrong.ok());
  EXPECT_EQ(wrong.status().domain(), Code::InvalidArgument);
  EXPECT_NE(wrong.status().message().find("camera bm-1 sent a 62x36 picture, "
                                          "not its 64x36"),
            std::string::npos)
      << wrong.status().message();
  EXPECT_FALSE(rig->failed());
  fake->push(test::host_pair(kWidth, kHeight, pattern(kWidth, kHeight, 1),
                             pattern(kWidth, kHeight, 1), kStartNs, 0, 2));
  core::Result<std::optional<PictureSet>> right = rig->take();
  ASSERT_TRUE(right.ok() && right.value().has_value());
  EXPECT_EQ(rig->stats().sets, 1u);

  fake->fail();
  core::Result<std::optional<PictureSet>> gone = rig->take();
  ASSERT_FALSE(gone.ok());
  EXPECT_EQ(gone.status().message(), "the stereo card is gone");
  EXPECT_TRUE(rig->failed());
}

TEST_F(SourceRig, AStereoSourceRefusesWhatItCannotName) {
  const auto refusal = [&](StereoMember member) {
    core::Result<std::unique_ptr<RigSource>> rig =
        RigSource::open(std::move(member), device(), allocator());
    EXPECT_FALSE(rig.ok());
    return rig.ok() ? std::string() : rig.status().message();
  };
  const auto fake = [] {
    return std::make_unique<test::FakeStereo>(test::uncalibrated_pair_info());
  };
  const std::array<camera::ImageSize, 2> sizes = {
      {{kWidth, kHeight}, {kWidth, kHeight}}};
  EXPECT_EQ(refusal({nullptr, {"bm-1", "bm-2"}, sizes}),
            "RigSource: no stereo sensor");
  EXPECT_EQ(refusal({fake(), {"bm-1", "bm-1"}, sizes}),
            "RigSource: camera bm-1 is named twice");
  EXPECT_EQ(refusal({fake(), {"", "bm-2"}, sizes}),
            "RigSource: a stereo camera's id is empty");
  EXPECT_EQ(refusal({fake(), {"bm-1", "bm-2"}, {}}),
            "RigSource: camera bm-1 has no size: the sensor was opened "
            "without a calibration");

  // A calibrated pair's sizes come from its calibration.
  sensor::StereoInfo calibrated = test::uncalibrated_pair_info();
  calibrated.calibration.left = test::color_like_lens();
  calibrated.calibration.right = test::color_like_lens();
  core::Result<std::unique_ptr<RigSource>> rig = RigSource::open(
      {std::make_unique<test::FakeStereo>(calibrated), {"bm-1", "bm-2"}},
      device(), allocator());
  ASSERT_TRUE(rig.ok()) << rig.status().message();
  EXPECT_EQ(rig.value()->cameras()[1].size.width,
            test::color_like_lens().size.width);
  EXPECT_FALSE(rig.value()->cameras()[1].factory.has_value());
}

// --- A sensor array ---------------------------------------------------------

class SourceArray : public SourceRig {
 protected:
  // Fake cameras, camera 0 the primary; each one's picture `pictures[i]`.
  void make_array(const std::vector<sensor::YuvImage>& pictures) {
    pictures_ = pictures;
    sensors_.clear();
    fakes_.clear();
    for (std::size_t i = 0; i < pictures.size(); ++i) {
      sensor::SensorInfo info;
      info.id = "SYN" + std::to_string(i);
      info.color = test::color_like_lens();
      info.color->size = {pictures[i].width, pictures[i].height};
      if (i % 2 == 0) info.depth = test::depth_like_lens();
      info.role =
          i == 0 ? sensor::SyncRole::Primary : sensor::SyncRole::Secondary;
      info.clock = sensor::ClockDomain::Host;
      info.fps = 25;
      auto fake = std::make_unique<test::FakeSensor>(info);
      fakes_.push_back(fake.get());
      sensors_.push_back(std::move(fake));
    }
  }

  std::unique_ptr<RigSource> open() {
    core::Result<sensor::SensorArray> array =
        sensor::SensorArray::open(std::move(sensors_), {});
    EXPECT_TRUE(array.ok()) << array.status().message();
    if (!array.ok()) return nullptr;
    core::Result<std::unique_ptr<RigSource>> rig =
        RigSource::open(std::move(array).value(), device(), allocator());
    EXPECT_TRUE(rig.ok()) << rig.status().message();
    return rig.ok() ? std::move(rig).value() : nullptr;
  }

  // Trigger `tick`'s frame from each camera but `missing`, camera i
  // `skew_ns[i]` late.
  void push(std::uint64_t tick, const std::vector<std::int64_t>& skew_ns = {},
            std::optional<std::size_t> missing = {}) {
    std::vector<std::pair<std::size_t, sensor::RgbdFrame>> frames;
    for (std::size_t i = 0; i < fakes_.size(); ++i) {
      if (missing == i) continue;
      sensor::RgbdFrame frame;
      frame.color = pictures_[i];
      const std::int64_t skew = i < skew_ns.size() ? skew_ns[i] : 0;
      frame.timestamp_ns = static_cast<std::uint64_t>(
          static_cast<std::int64_t>(kStartNs + tick * kPeriodNs) + skew);
      frame.sequence = 100 + tick;
      frames.emplace_back(i, std::move(frame));
    }
    for (auto& [i, frame] : frames) fakes_[i]->push(std::move(frame));
  }

  // The next set, polling until one comes.
  std::optional<PictureSet> next(RigSource& rig) {
    const auto give_up = std::chrono::steady_clock::now() + kPatience;
    while (std::chrono::steady_clock::now() < give_up) {
      core::Result<std::optional<PictureSet>> taken = rig.take();
      EXPECT_TRUE(taken.ok()) << taken.status().message();
      if (!taken.ok()) return std::nullopt;
      if (taken.value()) return std::move(taken).value();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ADD_FAILURE() << "no set";
    return std::nullopt;
  }

  // A host luma picture of `luma`.
  static sensor::YuvImage host_luma(const std::vector<std::uint8_t>& luma,
                                    std::uint32_t width, std::uint32_t height) {
    sensor::YuvImage picture;
    picture.layout = sensor::YuvLayout::Luma;
    picture.width = width;
    picture.height = height;
    picture.stride[0] = width;
    picture.host = luma.data();
    return picture;
  }

  std::vector<sensor::YuvImage> pictures_;
  std::vector<test::FakeSensor*> fakes_;
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors_;
};

// Each set: every camera's luma and half, its time and the set's spread; a
// host picture's tile on the device; the set its frames came in.
TEST_F(SourceArray, AnArraysSetComesAsEveryCamerasGreyPicture) {
  const std::vector<std::uint8_t> a = pattern(kWidth, kHeight, 4);
  const std::vector<std::uint8_t> b = pattern(kWidth + 16, kHeight + 8, 5);
  make_array(
      {host_luma(a, kWidth, kHeight), host_luma(b, kWidth + 16, kHeight + 8)});
  std::unique_ptr<RigSource> rig = open();
  ASSERT_NE(rig, nullptr);
  EXPECT_FALSE(rig->max_skew_ns().has_value());
  ASSERT_EQ(rig->cameras().size(), 2u);
  EXPECT_EQ(rig->cameras()[0].id, "SYN0");
  EXPECT_TRUE(rig->cameras()[0].factory.has_value());
  EXPECT_TRUE(rig->cameras()[0].depth);
  EXPECT_FALSE(rig->cameras()[1].depth);
  EXPECT_EQ(rig->cameras()[1].size.width, kWidth + 16);
  ASSERT_TRUE(rig->start().ok());
  push(1, {0, 3'000'000});
  const std::optional<PictureSet> set = next(*rig);
  ASSERT_TRUE(set.has_value());
  EXPECT_EQ(set->sequence, 101u);
  EXPECT_EQ(set->timestamp_ns, kStartNs + kPeriodNs);
  EXPECT_EQ(set->spread_ns, 3'000'000u);
  ASSERT_NE(set->rgbd, nullptr);
  EXPECT_EQ(set->rgbd->frames.size(), 2u);
  EXPECT_EQ(set->cameras[1].timestamp_ns, kStartNs + kPeriodNs + 3'000'000);
  const std::vector<std::uint8_t>* sent[2] = {&a, &b};
  const std::uint32_t widths[2] = {kWidth, kWidth + 16};
  const std::uint32_t heights[2] = {kHeight, kHeight + 8};
  for (std::size_t i = 0; i < 2; ++i) {
    SCOPED_TRACE("camera " + std::to_string(i));
    EXPECT_EQ(rows_of(rig->host()[i].luma), *sent[i]);
    EXPECT_EQ(rows_of(rig->host()[i].half),
              half_of(*sent[i], widths[i], heights[i]));
    ASSERT_NE(set->cameras[i].tile, nullptr);
    EXPECT_EQ(tile_bytes(*set->cameras[i].tile), *sent[i]);
  }
}

// The array's newest complete set wins; one missing a middle secondary still
// comes, that camera marked absent, with no host picture, and each camera
// after it with its own picture.
TEST_F(SourceArray, TakesTheNewestSetAndOneMissingACamera) {
  const std::vector<std::uint8_t> luma[3] = {pattern(kWidth, kHeight, 6),
                                             pattern(kWidth, kHeight, 16),
                                             pattern(kWidth, kHeight, 26)};
  make_array({host_luma(luma[0], kWidth, kHeight),
              host_luma(luma[1], kWidth, kHeight),
              host_luma(luma[2], kWidth, kHeight)});
  std::unique_ptr<RigSource> rig = open();
  ASSERT_NE(rig, nullptr);
  ASSERT_TRUE(rig->start().ok());
  for (std::uint64_t tick = 1; tick <= 3; ++tick) push(tick);
  const std::optional<PictureSet> newest = next(*rig);
  ASSERT_TRUE(newest.has_value());
  EXPECT_EQ(newest->timestamp_ns, kStartNs + 3 * kPeriodNs);
  core::Result<std::optional<PictureSet>> none = rig->take();
  ASSERT_TRUE(none.ok());
  EXPECT_FALSE(none.value().has_value());
  EXPECT_GT(rig->stats().unmatched, 0u);

  push(4, {}, 1);
  const std::optional<PictureSet> short_one = next(*rig);
  ASSERT_TRUE(short_one.has_value());
  EXPECT_FALSE(short_one->complete());
  EXPECT_EQ(short_one->count(), 2u);
  EXPECT_FALSE(short_one->cameras[1].present);
  EXPECT_EQ(short_one->cameras[1].tile, nullptr);
  EXPECT_EQ(rig->host()[1].luma.data, nullptr);
  for (const std::size_t i : {std::size_t{0}, std::size_t{2}}) {
    SCOPED_TRACE("camera " + std::to_string(i));
    EXPECT_TRUE(short_one->cameras[i].present);
    EXPECT_EQ(rows_of(rig->host()[i].luma), luma[i]);
    ASSERT_NE(short_one->cameras[i].tile, nullptr);
    EXPECT_EQ(tile_bytes(*short_one->cameras[i].tile), luma[i]);
  }
}

// The decoder's NV12 images show as the colour tile: no grey tile is held.
TEST_F(SourceArray, NoGreyTileWhereTheDecodersImagesShowColour) {
  const std::vector<std::uint8_t> luma = pattern(kWidth, kHeight, 7);
  sensor::YuvImage nv12;
  nv12.layout = sensor::YuvLayout::Nv12;
  nv12.width = kWidth;
  nv12.height = kHeight;
  core::ImageDesc desc;
  desc.extent = {kWidth, kHeight};
  desc.format = VK_FORMAT_R8_UNORM;
  desc.usage =
      VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  desc.with_view = false;
  core::Result<core::Image> y = allocator().create_image(desc);
  ASSERT_TRUE(y.ok());
  ASSERT_TRUE(core::test::write_image(device(), allocator(), y.value(),
                                      luma.data(), luma.size(),
                                      VK_IMAGE_LAYOUT_GENERAL)
                  .ok());
  desc.extent = {kWidth / 2, kHeight / 2};
  desc.format = VK_FORMAT_R8G8_UNORM;
  core::Result<core::Image> uv = allocator().create_image(desc);
  ASSERT_TRUE(uv.ok());
  const std::vector<std::uint8_t> neutral(kWidth * kHeight / 2, 128);
  ASSERT_TRUE(core::test::write_image(device(), allocator(), uv.value(),
                                      neutral.data(), neutral.size(),
                                      VK_IMAGE_LAYOUT_GENERAL)
                  .ok());
  nv12.image[0] = std::make_shared<const core::Image>(std::move(y).value());
  nv12.image[1] = std::make_shared<const core::Image>(std::move(uv).value());
  make_array({nv12});
  std::unique_ptr<RigSource> rig = open();
  ASSERT_NE(rig, nullptr);
  ASSERT_TRUE(rig->start().ok());
  push(1);
  const std::optional<PictureSet> set = next(*rig);
  ASSERT_TRUE(set.has_value());
  EXPECT_EQ(set->cameras[0].tile, nullptr);
  EXPECT_EQ(set->owner, nullptr);
  ASSERT_NE(set->rgbd, nullptr);
  EXPECT_TRUE(set->rgbd->frames[0]->has_color());
  EXPECT_EQ(rows_of(rig->host()[0].luma), luma);
}

// A decoder's picture that carries a ready value, as NVIDIA's do, is read by
// the luma pass only once the value is set: the take waits for it on the
// device, with no host wait on the value, and then reads the picture. The
// set's own value comes after the pass: while the take waits, the source's
// timeline stays at the last set's.
TEST_F(SourceArray, ReadsADecodersPictureOnlyOnceItsReadyValueIsSet) {
  const std::vector<std::uint8_t> luma = pattern(kWidth, kHeight, 9);
  sensor::testing::HostNv12 host;
  host.width = kWidth;
  host.height = kHeight;
  host.luma = luma;
  host.chroma.assign(std::size_t{kWidth} * kHeight / 2, 128);
  core::Result<sensor::YuvImage> picture =
      sensor::testing::device_picture(device(), allocator(), host);
  ASSERT_TRUE(picture.ok()) << picture.status().message();
  core::Result<core::TimelineSemaphore> made =
      core::TimelineSemaphore::create(device());
  ASSERT_TRUE(made.ok()) << made.status().message();
  const auto ready =
      std::make_shared<core::TimelineSemaphore>(std::move(made).value());
  picture->ready = ready;
  picture->ready_value = 1;
  make_array({picture.value()});
  std::unique_ptr<RigSource> rig = open();
  ASSERT_NE(rig, nullptr);
  ASSERT_TRUE(rig->start().ok());
  // First a picture decoded by the take: the source's timeline and value.
  ASSERT_TRUE(ready->signal(1).ok());
  push(1);
  const std::optional<PictureSet> earlier = next(*rig);
  ASSERT_TRUE(earlier.has_value());
  ASSERT_NE(earlier->ready, nullptr);
  const std::uint64_t passed = earlier->ready_value;
  pictures_[0].ready_value = 2;
  push(2);
  using Taken = core::Result<std::optional<PictureSet>>;
  const auto give_up = std::chrono::steady_clock::now() + kPatience;
  std::future<Taken> taken = std::async(std::launch::async, [&]() -> Taken {
    while (std::chrono::steady_clock::now() < give_up) {
      Taken next = rig->take();
      if (!next.ok() || next.value()) return next;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return core::Status::not_found("no set within the test's patience");
  });
  // A check failing before the value is set leaves no take waiting for it:
  // the guard sets it before the future joins the take.
  const test::SetOnExit decoded_by_then{*ready, 2};
  // The array has the frame, and the take its set; it waits on the device.
  while (fakes_[0]->queued() != 0 &&
         std::chrono::steady_clock::now() < give_up) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  const bool waited =
      taken.wait_for(std::chrono::seconds(0)) == std::future_status::timeout;
  // Nothing set for the set yet: its value comes after the pass.
  const core::Result<std::uint64_t> during = earlier->ready->value();
  const bool set = ready->signal(2).ok();
  ASSERT_TRUE(set);
  EXPECT_TRUE(waited);
  ASSERT_TRUE(during.ok()) << during.status().message();
  EXPECT_EQ(during.value(), passed);
  Taken result = taken.get();
  ASSERT_TRUE(result.ok()) << result.status().message();
  ASSERT_TRUE(result.value().has_value());
  EXPECT_EQ(rows_of(rig->host()[0].luma), luma);
  EXPECT_EQ(rows_of(rig->host()[0].half), half_of(luma, kWidth, kHeight));
  const PictureSet& later = *result.value();
  EXPECT_EQ(later.ready, earlier->ready);
  EXPECT_EQ(later.ready_value, passed + 1);
  const core::Result<std::uint64_t> now = later.ready->value();
  ASSERT_TRUE(now.ok()) << now.status().message();
  EXPECT_GE(now.value(), later.ready_value);
}

TEST_F(SourceArray, RefusesACameraWithoutColour) {
  sensor::SensorInfo info;
  info.id = "NOCOLOR";
  info.fps = 25;
  sensors_.push_back(std::make_unique<test::FakeSensor>(info));
  core::Result<sensor::SensorArray> array =
      sensor::SensorArray::open(std::move(sensors_), {});
  ASSERT_TRUE(array.ok()) << array.status().message();
  core::Result<std::unique_ptr<RigSource>> rig =
      RigSource::open(std::move(array).value(), device(), allocator());
  ASSERT_FALSE(rig.ok());
  EXPECT_EQ(rig.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(rig.status().message(),
            "RigSource: camera NOCOLOR has no colour model");
}

// A camera disconnected fails the take, as its poll failed.
TEST_F(SourceArray, ACameraDisconnectedFailsTheTake) {
  const std::vector<std::uint8_t> luma = pattern(kWidth, kHeight, 8);
  make_array({host_luma(luma, kWidth, kHeight)});
  std::unique_ptr<RigSource> rig = open();
  ASSERT_NE(rig, nullptr);
  ASSERT_TRUE(rig->start().ok());
  fakes_[0]->disconnect();
  core::Result<std::optional<PictureSet>> taken = rig->take();
  ASSERT_FALSE(taken.ok());
  EXPECT_EQ(taken.status().domain(), Code::IoError);
  EXPECT_TRUE(rig->failed());
}

// --- The ChArUco board in the GPU's half ------------------------------------

// The rig's board at 4K, found in the half the GPU made as in the half the
// CPU made before (OpenCV's INTER_AREA): every corner within 0.01 px. One
// pair, each picture with noise of its own; the halves' bytes are held to
// test::halved's at small sizes above.
TEST_F(SourceRig, FindsTheBoardInTheGpusHalfAsInTheCpusHalf) {
  const compute::Board board = test::femto_board();
  const camera::CameraModel lens = test::color_like_lens();
  test::FakeStereo* fake = nullptr;
  sensor::StereoInfo info = test::uncalibrated_pair_info();
  auto sensor = std::make_unique<test::FakeStereo>(info);
  fake = sensor.get();
  StereoMember member{std::move(sensor), {"bm-1", "bm-2"}};
  member.sizes = {{lens.size, lens.size}};
  core::Result<std::unique_ptr<RigSource>> made =
      RigSource::open(std::move(member), device(), allocator());
  ASSERT_TRUE(made.ok()) << made.status().message();
  RigSource& rig = *made.value();
  ASSERT_TRUE(rig.start().ok());
  const camera::Mat4d pose =
      test::board_facing_camera(board, {0.0, 0.0, 2.1}, 25.0, 2.0);
  const cv::Mat render = test::render_board(lens, pose, board);
  const cv::Mat left = test::camera_image(render, {0.8, 2.5, 2});
  const cv::Mat right = test::camera_image(render, {0.8, 2.5, 8});
  fake->push(test::host_pair(
      lens.size.width, lens.size.height,
      std::vector<std::uint8_t>(left.data, left.data + left.total()),
      std::vector<std::uint8_t>(right.data, right.data + right.total()),
      kStartNs, 0, 1));
  core::Result<std::optional<PictureSet>> taken = rig.take();
  ASSERT_TRUE(taken.ok() && taken.value().has_value());
  double worst = 0.0;
  std::size_t corners = 0;
  bool halves_differ = false;
  for (std::size_t i = 0; i < 2; ++i) {
    const HostLuma& host = rig.host()[i];
    const core::Result<Detection> gpu =
        detect::detect_board(host.luma, board, {&host.half});
    ASSERT_TRUE(gpu.ok()) << gpu.status().message();
    // The CPU's half, as calib 0.3.0's live view made it before searching
    // it: the picture's even part, INTER_AREA at exactly half the size.
    const cv::Mat full(
        static_cast<int>(host.luma.height), static_cast<int>(host.luma.width),
        CV_8UC1, const_cast<std::uint8_t*>(host.luma.data), host.luma.stride);
    cv::Mat old_half;
    cv::resize(full(cv::Rect(0, 0, full.cols & ~1, full.rows & ~1)), old_half,
               cv::Size(full.cols / 2, full.rows / 2), 0.0, 0.0,
               cv::INTER_AREA);
    const detect::LumaView old_view{static_cast<std::uint32_t>(old_half.cols),
                                    static_cast<std::uint32_t>(old_half.rows),
                                    old_half.step[0], old_half.data};
    halves_differ = halves_differ || rows_of(old_view) != rows_of(host.half);
    const core::Result<Detection> cpu =
        detect::detect_board(host.luma, board, {&old_view});
    ASSERT_TRUE(cpu.ok()) << cpu.status().message();
    const std::vector<DetectedPoint>& a = gpu.value().observation.points;
    const std::vector<DetectedPoint>& b = cpu.value().observation.points;
    ASSERT_EQ(a.size(), b.size());
    ASSERT_GT(a.size(), 0u);
    for (std::size_t p = 0; p < a.size(); ++p) {
      ASSERT_EQ(a[p].id, b[p].id);
      worst = std::max(worst, std::hypot(a[p].pixel.x - b[p].pixel.x,
                                         a[p].pixel.y - b[p].pixel.y));
    }
    corners += a.size();
  }
  std::printf("GPU half against CPU half: %zu corners, worst %.3g px, %s\n",
              corners, worst,
              halves_differ ? "halves differ" : "halves byte-equal");
  EXPECT_LE(worst, 0.01);
}

// --- Sample main 4 ----------------------------------------------------------

// The design's main 4, "Reading a source", stopping after `sets` sets.
core::Status watch(std::unique_ptr<sensor::IStereoSensor> pair,
                   const core::Device& device, core::Allocator& allocator,
                   const compute::Board& board, int sets) {
  VKC_ASSIGN(
      std::unique_ptr<RigSource> rig,
      RigSource::open(
          {std::move(pair), {"bm-1", "bm-2"}, {{{1280, 720}, {1280, 720}}}},
          device, allocator));
  VKC_TRY(rig->start());
  for (int seen = 0; seen < sets;) {
    VKC_ASSIGN(std::optional<PictureSet> set, rig->take());
    if (!set) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    for (std::size_t i = 0; i < set->cameras.size(); ++i) {
      const HostLuma& picture = rig->host()[i];
      VKC_ASSIGN(const Detection found,
                 detect::detect_board(picture.luma, board, {&picture.half}));
      std::printf("%s: %zu corners\n", rig->cameras()[i].id.c_str(),
                  found.observation.points.size());
    }
    ++seen;
  }
  return {};
}

TEST_F(SourceRig, MainFourReadsASource) {
  const compute::Board board = test::femto_board();
  camera::CameraModel lens = test::color_like_lens();
  lens.size = {1280, 720};
  lens.intrinsics.fx /= 3.0;
  lens.intrinsics.fy /= 3.0;
  lens.intrinsics.cx /= 3.0;
  lens.intrinsics.cy /= 3.0;
  const cv::Mat render = test::render_board(
      lens, test::board_facing_camera(board, {0.0, 0.0, 1.3}, 10.0, 5.0),
      board);
  const std::vector<std::uint8_t> bytes(render.data,
                                        render.data + render.total());
  auto fake =
      std::make_unique<test::FakeStereo>(test::uncalibrated_pair_info());
  fake->push(test::host_pair(1280, 720, bytes, bytes, kStartNs, 0, 1));
  const core::Status watched =
      watch(std::move(fake), device(), allocator(), board, 1);
  EXPECT_TRUE(watched.ok()) << watched.message();
}

}  // namespace
}  // namespace volumetric_kit::calib::source
