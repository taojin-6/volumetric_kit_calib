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

// A rig whose camera i reads back a flat image at `level + i`, counting reads;
// the test drives on_set with sets it builds itself.
class FakeRig final : public IRigSource {
 public:
  explicit FakeRig(std::vector<CameraInfo> cameras)
      : cameras_(std::move(cameras)) {}

  const std::vector<CameraInfo>& cameras() const noexcept override {
    return cameras_;
  }
  Status start() override { return {}; }
  void stop() noexcept override {}
  Result<std::optional<SetInfo>> poll_set() override { return no_set(); }
  Result<LumaImage> read_luma(std::size_t camera) override {
    ++reads;
    if (fail_camera && *fail_camera == camera) {
      return Status::io_error("camera unplugged");
    }
    ImageSize size = cameras_[camera].lens->image_size();
    if (wrong_size) size.width += 1;
    return flat_image(size, static_cast<std::uint8_t>(level + camera));
  }

  int level = 0;
  std::size_t reads = 0;
  std::optional<std::size_t> fail_camera;
  bool wrong_size = false;

 private:
  std::vector<CameraInfo> cameras_;
};

std::vector<CameraInfo> rig_cameras() {
  return {make_camera("CL2A141000N", SyncRole::Primary, 24, 16),
          make_camera("CL2A141000G", SyncRole::Secondary, 12, 8)};
}

SetInfo set_of(std::uint64_t trigger_ns, std::vector<bool> present) {
  SetInfo set;
  set.trigger_ns = trigger_ns;
  set.present = std::move(present);
  return set;
}

struct Rig {
  ScratchDir dir;
  FakeRig source{rig_cameras()};

  CaptureRecorder recorder(std::uint32_t frames) {
    auto writer = SessionWriter::create(dir.str(), source.cameras());
    EXPECT_TRUE(writer.ok()) << writer.status().message();
    auto made = CaptureRecorder::create(std::move(writer).value(), {frames});
    EXPECT_TRUE(made.ok()) << made.status().message();
    return std::move(made).value();
  }

  // Feed one set at `level`; returns what on_set returned.
  Result<std::optional<std::size_t>> feed(CaptureRecorder& r, int level,
                                          std::uint64_t trigger_ns,
                                          std::vector<bool> present = {true,
                                                                       true}) {
    source.level = level;
    return r.on_set(set_of(trigger_ns, std::move(present)), source);
  }

  std::uint8_t pixel(std::size_t camera, std::size_t index) {
    auto image =
        read_session_image(dir.str(), source.cameras()[camera].serial, index);
    EXPECT_TRUE(image.ok()) << image.status().message();
    return image.ok() ? image->pixels.front() : 0;
  }
};

TEST(CaptureRecorder, ReadsNothingUntilArmed) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(3);
  auto result = rig.feed(recorder, 10, 1);
  ASSERT_TRUE(result.ok());
  EXPECT_FALSE(result->has_value());
  EXPECT_EQ(rig.source.reads, 0u);
  EXPECT_TRUE(recorder.writer().session().captures.empty());
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
  EXPECT_EQ(rig.pixel(0, 0), 21);   // the skipped set's 90 is not in it
  EXPECT_EQ(rig.source.reads, 4u);  // two sets, two cameras; none skipped
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
  EXPECT_EQ(recorder.undo().domain(), Status::Code::NotFound);
}

TEST(CaptureRecorder, AReadFailureDropsTheCaptureAndDisarms) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(2);
  recorder.arm();
  rig.source.fail_camera = 1;
  auto failed = rig.feed(recorder, 10, 1);
  EXPECT_EQ(failed.status().domain(), Status::Code::IoError);
  EXPECT_FALSE(recorder.armed());
  EXPECT_TRUE(recorder.writer().session().captures.empty());
}

TEST(CaptureRecorder, RefusesAnImageTheWrongSize) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(1);
  recorder.arm();
  rig.source.wrong_size = true;
  EXPECT_EQ(rig.feed(recorder, 10, 1).status().domain(),
            Status::Code::InvalidArgument);
  EXPECT_FALSE(recorder.armed());
}

TEST(CaptureRecorder, RefusesASetOfAnotherRig) {
  Rig rig;
  CaptureRecorder recorder = rig.recorder(1);
  EXPECT_EQ(rig.feed(recorder, 10, 1, {true, true, true}).status().domain(),
            Status::Code::InvalidArgument);
}

TEST(CaptureRecorder, CreateRefusesNoFramesToAverage) {
  const ScratchDir dir;
  auto writer = SessionWriter::create(dir.str(), rig_cameras());
  ASSERT_TRUE(writer.ok());
  EXPECT_EQ(
      CaptureRecorder::create(std::move(writer).value(), {0}).status().domain(),
      Status::Code::InvalidArgument);
}

}  // namespace
}  // namespace volumetric_kit::calib::capture
