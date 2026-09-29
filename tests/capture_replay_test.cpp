// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/capture/session_replay.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "capture_test_util.hpp"
#include "volumetric_kit/calib/capture/session.hpp"

namespace volumetric_kit::calib::capture {
namespace {

using testing::make_camera;
using testing::pattern_image;
using testing::ScratchDir;

// A session of two cameras and three captures; camera c's image of capture k
// is pattern_image(size, 10 * k + c).
std::vector<CameraInfo> record(const ScratchDir& dir) {
  const std::vector<CameraInfo> cameras = {
      make_camera("CL2A141000N", SyncRole::Primary, 40, 30),
      make_camera("CL2A14100A4", SyncRole::Secondary, 20, 16)};
  auto writer = SessionWriter::create(dir.str(), cameras);
  EXPECT_TRUE(writer.ok()) << writer.status().message();
  for (unsigned k = 0; k < 3; ++k) {
    std::vector<LumaImage> images;
    for (unsigned c = 0; c < cameras.size(); ++c) {
      images.push_back(
          pattern_image(cameras[c].lens->image_size(), 10 * k + c));
    }
    EXPECT_TRUE(writer->add(100 + k, 1, images).ok());
  }
  return cameras;
}

TEST(CaptureReplay, HandsOutEveryCaptureOnceInOrder) {
  const ScratchDir dir;
  const std::vector<CameraInfo> cameras = record(dir);
  auto replay = SessionReplay::open(dir.str());
  ASSERT_TRUE(replay.ok()) << replay.status().message();
  ASSERT_EQ(replay->cameras().size(), 2u);
  EXPECT_EQ(replay->cameras()[0].serial, "CL2A141000N");
  EXPECT_TRUE(replay->cameras()[1].lens.has_value());
  EXPECT_EQ(find_primary(replay->cameras()), std::optional<std::size_t>{0});

  // Not started: nothing yet, and not over.
  auto before = replay->poll_set();
  ASSERT_TRUE(before.ok());
  EXPECT_FALSE(before->has_value());
  EXPECT_FALSE(replay->exhausted());

  ASSERT_TRUE(replay->start().ok());
  for (unsigned k = 0; k < 3; ++k) {
    auto set = replay->poll_set();
    ASSERT_TRUE(set.ok()) << set.status().message();
    ASSERT_TRUE(set->has_value());
    EXPECT_EQ((*set)->trigger_ns, 100u + k);
    EXPECT_TRUE((*set)->complete());
    for (unsigned c = 0; c < 2; ++c) {
      auto image = replay->read_luma(c);
      ASSERT_TRUE(image.ok()) << image.status().message();
      EXPECT_EQ(
          image->pixels,
          pattern_image(cameras[c].lens->image_size(), 10 * k + c).pixels);
    }
  }
  auto after = replay->poll_set();
  ASSERT_TRUE(after.ok());
  EXPECT_FALSE(after->has_value());
  EXPECT_TRUE(replay->exhausted());
}

TEST(CaptureReplay, StartRewindsToTheFirstCapture) {
  const ScratchDir dir;
  record(dir);
  auto replay = SessionReplay::open(dir.str());
  ASSERT_TRUE(replay.ok());
  ASSERT_TRUE(replay->start().ok());
  while (!replay->exhausted()) ASSERT_TRUE(replay->poll_set().ok());

  ASSERT_TRUE(replay->start().ok());
  EXPECT_FALSE(replay->exhausted());
  auto set = replay->poll_set();
  ASSERT_TRUE(set.ok());
  ASSERT_TRUE(set->has_value());
  EXPECT_EQ((*set)->trigger_ns, 100u);
}

TEST(CaptureReplay, ReadLumaNeedsAPolledSetAndACamera) {
  const ScratchDir dir;
  record(dir);
  auto replay = SessionReplay::open(dir.str());
  ASSERT_TRUE(replay.ok());
  EXPECT_EQ(replay->read_luma(0).status().domain(),
            Status::Code::InvalidArgument);  // not started

  ASSERT_TRUE(replay->start().ok());
  EXPECT_EQ(replay->read_luma(0).status().domain(),
            Status::Code::InvalidArgument);  // started, nothing polled
  ASSERT_TRUE(replay->poll_set().ok());
  EXPECT_TRUE(replay->read_luma(1).ok());
  EXPECT_EQ(replay->read_luma(2).status().domain(),
            Status::Code::InvalidArgument);  // no third camera

  replay->stop();
  EXPECT_EQ(replay->read_luma(0).status().domain(),
            Status::Code::InvalidArgument);  // stopped
}

TEST(CaptureReplay, AnEmptySessionIsExhaustedOnStart) {
  const ScratchDir dir;
  ASSERT_TRUE(
      SessionWriter::create(
          dir.str(), {make_camera("CL2A141000N", SyncRole::Primary, 8, 8)})
          .ok());
  auto replay = SessionReplay::open(dir.str());
  ASSERT_TRUE(replay.ok());
  ASSERT_TRUE(replay->start().ok());
  EXPECT_TRUE(replay->exhausted());
}

TEST(CaptureReplay, OpeningAMissingSessionIsNotFound) {
  const ScratchDir dir;
  EXPECT_EQ(SessionReplay::open(dir.str()).status().domain(),
            Status::Code::NotFound);
}

}  // namespace
}  // namespace volumetric_kit::calib::capture
