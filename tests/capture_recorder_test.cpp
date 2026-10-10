// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/capture/capture_recorder.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "capture_test_util.hpp"
#include "volumetric_kit/calib/capture/session.hpp"

namespace volumetric_kit::calib::capture {
namespace {

using testing::flat_image;
using testing::make_camera;
using testing::ScratchDir;

std::vector<SessionCamera> rig_cameras() {
  return {make_camera("CL2A141000N", 24, 16),
          make_camera("CL2A141000G", 12, 8)};
}

// A session of rig_cameras(), and sets fed to a recorder of it: camera i's
// image is flat at `level + i`.
struct Rig {
  ScratchDir dir;
  std::vector<SessionCamera> cameras = rig_cameras();
  // Set to make the next set's image of camera 0 one column too wide.
  bool wrong_size = false;

  CaptureRecorder recorder(std::uint32_t frames) {
    auto writer = SessionWriter::create(dir.str(), cameras, 0);
    EXPECT_TRUE(writer.ok()) << writer.status().message();
    auto made = CaptureRecorder::create(std::move(writer).value(), {frames});
    EXPECT_TRUE(made.ok()) << made.status().message();
    return std::move(made).value();
  }

  // Feed one set at `level`, with a frame from each camera `present` names;
  // returns what on_set returned.
  core::Result<std::optional<std::size_t>> feed(CaptureRecorder& r, int level,
                                                std::uint64_t trigger_ns,
                                                std::vector<bool> present = {
                                                    true, true}) {
    std::vector<LumaImage> luma;
    std::vector<const LumaImage*> images(present.size(), nullptr);
    luma.reserve(present.size());
    for (std::size_t i = 0; i < present.size(); ++i) {
      ImageSize size = cameras[i % cameras.size()].lens.image_size();
      if (i == 0 && wrong_size) size.width += 1;
      luma.push_back(flat_image(size, static_cast<std::uint8_t>(level + i)));
      if (present[i]) images[i] = &luma.back();
    }
    return r.on_set(trigger_ns, images);
  }

  std::uint8_t pixel(std::size_t camera, std::size_t index) {
    auto image = read_session_image(dir.str(), cameras[camera].serial, index);
    EXPECT_TRUE(image.ok()) << image.status().message();
    return image.ok() ? image->pixels.front() : 0;
  }
};

TEST(CaptureRecorder, IgnoresSetsUntilArmed) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(1);
  auto result = rig.feed(recorder, 10, 1);
  ASSERT_TRUE(result.ok());
  EXPECT_FALSE(result->has_value());
  EXPECT_EQ(recorder.collected(), 0u);
  EXPECT_TRUE(recorder.writer().session().captures.empty());

  // Nor does a set with no images, which is how a caller that reads nothing
  // while disarmed passes one, count as skipped.
  ASSERT_TRUE(rig.feed(recorder, 10, 2, {false, false}).ok());
  EXPECT_EQ(recorder.skipped(), 0u);
}

TEST(CaptureRecorder, AveragesTheNextCompleteSetsIntoOneCapture) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(3);
  recorder.arm();
  EXPECT_TRUE(recorder.armed());
  ASSERT_FALSE(rig.feed(recorder, 10, 500)->has_value());
  ASSERT_FALSE(rig.feed(recorder, 11, 540)->has_value());
  EXPECT_EQ(recorder.collected(), 2u);
  auto written = rig.feed(recorder, 13, 580);
  ASSERT_TRUE(written.ok()) << written.status().message();
  ASSERT_TRUE(written->has_value());
  EXPECT_EQ(**written, 0u);
  EXPECT_FALSE(recorder.armed());

  // (10 + 11 + 13) / 3 = 11.33 -> 11; camera 1 reads one level higher.
  EXPECT_EQ(rig.pixel(0, 0), 11);
  EXPECT_EQ(rig.pixel(1, 0), 12);
  const Session& session = recorder.writer().session();
  ASSERT_EQ(session.captures.size(), 1u);
  EXPECT_EQ(session.captures[0].trigger_ns, 500u);  // the first set's
  EXPECT_EQ(session.captures[0].frames_averaged, 3u);
}

TEST(CaptureRecorder, RoundsTheAverageToTheNearestLevel) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(2);
  recorder.arm();
  ASSERT_TRUE(rig.feed(recorder, 10, 1).ok());
  ASSERT_TRUE(rig.feed(recorder, 11, 2).ok());  // 10.5 -> 11
  EXPECT_EQ(rig.pixel(0, 0), 11);
}

TEST(CaptureRecorder, SkipsASetMissingACamera) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(2);
  recorder.arm();
  ASSERT_FALSE(rig.feed(recorder, 20, 1)->has_value());
  ASSERT_FALSE(rig.feed(recorder, 90, 2, {true, false})->has_value());
  EXPECT_EQ(recorder.skipped(), 1u);
  EXPECT_EQ(recorder.collected(), 1u);
  auto written = rig.feed(recorder, 22, 3);
  ASSERT_TRUE(written.ok());
  ASSERT_TRUE(written->has_value());
  EXPECT_EQ(rig.pixel(0, 0), 21);  // the skipped set's 90 is not in it
}

TEST(CaptureRecorder, ArmingTwiceCollectsOneCapture) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(2);
  recorder.arm();
  ASSERT_TRUE(rig.feed(recorder, 10, 1).ok());
  recorder.arm();  // a second press mid-capture changes nothing
  EXPECT_EQ(recorder.collected(), 1u);
  auto written = rig.feed(recorder, 10, 2);
  ASSERT_TRUE(written.ok());
  EXPECT_TRUE(written->has_value());
  EXPECT_EQ(recorder.writer().session().captures.size(), 1u);
}

TEST(CaptureRecorder, UndoWhenIdleRemovesTheLastCapture) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(1);
  recorder.arm();
  ASSERT_TRUE(rig.feed(recorder, 10, 1)->has_value());
  recorder.arm();
  ASSERT_TRUE(rig.feed(recorder, 20, 2)->has_value());
  ASSERT_EQ(recorder.writer().session().captures.size(), 2u);

  ASSERT_TRUE(recorder.undo().ok());
  ASSERT_EQ(recorder.writer().session().captures.size(), 1u);
  EXPECT_EQ(rig.pixel(0, 0), 10);  // the first capture is the one kept
}

TEST(CaptureRecorder, UndoWhileArmedOnlyDisarms) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(3);
  recorder.arm();
  ASSERT_TRUE(rig.feed(recorder, 10, 1).ok());
  ASSERT_TRUE(recorder.undo().ok());
  EXPECT_FALSE(recorder.armed());
  EXPECT_EQ(recorder.collected(), 0u);
  EXPECT_TRUE(recorder.writer().session().captures.empty());

  // With no capture and nothing armed, there is nothing to take back.
  EXPECT_EQ(recorder.undo().domain(), core::Status::Code::NotFound);
}

TEST(CaptureRecorder, RefusesAnImageTheWrongSize) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(2);
  recorder.arm();
  ASSERT_TRUE(rig.feed(recorder, 10, 1).ok());  // one collected
  rig.wrong_size = true;
  EXPECT_EQ(rig.feed(recorder, 10, 2).status().domain(),
            core::Status::Code::InvalidArgument);
  // The capture being collected is dropped, and nothing was written.
  EXPECT_FALSE(recorder.armed());
  EXPECT_EQ(recorder.collected(), 0u);
  EXPECT_TRUE(recorder.writer().session().captures.empty());
}

TEST(CaptureRecorder, RefusesASetOfAnotherRig) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(1);
  EXPECT_EQ(rig.feed(recorder, 10, 1, {true, true, true}).status().domain(),
            core::Status::Code::InvalidArgument);
}

TEST(CaptureRecorder, CreateRefusesNoFramesToAverage) {
  const ScratchDir dir;
  auto writer = SessionWriter::create(dir.str(), rig_cameras(), 0);
  ASSERT_TRUE(writer.ok());
  EXPECT_EQ(
      CaptureRecorder::create(std::move(writer).value(), {0}).status().domain(),
      core::Status::Code::InvalidArgument);
}

}  // namespace
}  // namespace volumetric_kit::calib::capture
