// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// What the capture tier's API allocates, accepting or refusing, can run out
// of memory: it must say OutOfMemory, never throw. The session's directory
// and sync file do not exist, so a capture queued fails on the writer's
// thread and writes nothing.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "allocation_failure.hpp"
#include "volumetric_kit/calib/capture/session_writer.hpp"
#include "volumetric_kit/core/base/check.hpp"

namespace volumetric_kit::calib::capture {
namespace {

namespace camera = recon::camera;
namespace vkc = volumetric_kit::core;
using Code = vkc::Status::Code;
using test::expect_oom_when_reporting;

// Enough for a sweep: its first call, the census, one for each allocation it
// fails and one failing none.
constexpr std::size_t kCalls = 33;

camera::CameraModel model(std::uint32_t width, std::uint32_t height) {
  camera::CameraModel m;
  m.size = {width, height};
  m.intrinsics = {0.8 * width, 0.8 * width, 0.5 * width, 0.5 * height};
  return m;
}

// Paths too long for any small-string buffer, under a parent that is not
// there.
SessionInfo make_info() {
  SessionInfo info;
  info.directory = "/vc_capture_oom_no_such_parent/2026-10-13_094107";
  info.sync_path = "/vc_capture_oom_no_such_parent/femto_mega_sync.json";
  info.reference = "CL2A141000N";
  info.board = legacy_femto_board();
  info.depth_mode.name = "nfov_unbinned";
  info.depth_mode.min_m = 0.5f;
  info.depth_mode.max_m = 3.86f;
  for (const char* serial : {"CL2A14100A4", "CL2A141000N"}) {
    SessionCamera c;
    c.info.id = serial;
    c.info.color = model(64, 48);
    c.info.depth = model(16, 12);
    c.info.fps = 25;
    info.cameras.push_back(std::move(c));
  }
  return info;
}

CaptureJob make_job() {
  CaptureJob job;
  for (int i = 0; i < 2; ++i) {
    CameraCapture part;
    part.jpeg = {0xFF, 0xD8, 0xFF, 0xD9};
    part.depth = std::make_shared<const std::vector<std::uint16_t>>(
        std::size_t{16} * 12, std::uint16_t{1000});
    part.detection.image_size = {64, 48};
    part.detection.observation.points = {{0, {10.0, 10.0}, 1.0}};
    part.check.verdict = ViewVerdict::Partial;
    job.cameras.push_back(std::move(part));
  }
  job.cameras[1].origin_tag = DetectedMarker{
      0, {{{30.0, 20.0}, {40.0, 20.0}, {40.0, 30.0}, {30.0, 30.0}}}};
  return job;
}

// What `fn` takes, made before any allocation fails; each call takes the
// next, by a move, which allocates nothing.
template <class T>
class Supply {
 public:
  explicit Supply(const T& each) : items_(kCalls, each) {}
  T take() {
    VKC_CHECK(!items_.empty(), "a sweep took more than kCalls calls");
    T item = std::move(items_.back());
    items_.pop_back();
    return item;
  }

 private:
  std::vector<T> items_;
};

TEST(CaptureOom, CreateAcceptingASession) {
  Supply<SessionInfo> infos(make_info());
  expect_oom_when_reporting(
      [&infos] { return SessionWriter::create(infos.take()); }, Code::Ok);
}

TEST(CaptureOom, CreateRefusingASession) {
  SessionInfo info = make_info();
  info.cameras[1].info.fps = 15;
  Supply<SessionInfo> infos(info);
  expect_oom_when_reporting(
      [&infos] { return SessionWriter::create(infos.take()); },
      Code::InvalidArgument);
}

TEST(CaptureOom, CreateRefusingAnOriginTag) {
  SessionInfo info = make_info();
  info.origin_tag.id = 587;  // past 36h11's codes
  Supply<SessionInfo> infos(info);
  expect_oom_when_reporting(
      [&infos] { return SessionWriter::create(infos.take()); },
      Code::InvalidArgument);
}

TEST(CaptureOom, CreateRefusingWhatReconRefuses) {
  SessionInfo info = make_info();
  info.cameras[0].info.color->intrinsics.fx =
      std::numeric_limits<double>::quiet_NaN();
  Supply<SessionInfo> infos(info);
  expect_oom_when_reporting(
      [&infos] { return SessionWriter::create(infos.take()); },
      Code::InvalidArgument);
}

TEST(CaptureOom, SubmitQueuingACapture) {
  vkc::Result<std::unique_ptr<SessionWriter>> writer =
      SessionWriter::create(make_info(), {2 * kCalls});
  ASSERT_TRUE(writer.ok()) << writer.status().message();
  Supply<CaptureJob> jobs(make_job());
  expect_oom_when_reporting([&] { return (*writer)->submit(jobs.take()); },
                            Code::Ok);
}

TEST(CaptureOom, SubmitRefusingACapture) {
  vkc::Result<std::unique_ptr<SessionWriter>> writer =
      SessionWriter::create(make_info());
  ASSERT_TRUE(writer.ok()) << writer.status().message();
  CaptureJob job = make_job();
  job.cameras[1].detection.image_size = {32, 24};
  Supply<CaptureJob> jobs(job);
  expect_oom_when_reporting([&] { return (*writer)->submit(jobs.take()); },
                            Code::InvalidArgument);
}

TEST(CaptureOom, SubmitRefusingAnotherTag) {
  vkc::Result<std::unique_ptr<SessionWriter>> writer =
      SessionWriter::create(make_info());
  ASSERT_TRUE(writer.ok()) << writer.status().message();
  CaptureJob job = make_job();
  job.cameras[1].origin_tag->id = 3;
  Supply<CaptureJob> jobs(job);
  expect_oom_when_reporting([&] { return (*writer)->submit(jobs.take()); },
                            Code::InvalidArgument);
}

// The name fits libc++'s small-string buffer, so only a library with a
// smaller one (libstdc++) allocates for it.
TEST(CaptureOom, SessionStamp) {
  const auto time =
      std::chrono::system_clock::time_point(std::chrono::seconds(1760348467));
  test::sweep_allocations([&](std::size_t after) {
    SCOPED_TRACE(after);
    std::optional<vkc::Result<std::string>> stamp;
    EXPECT_NO_THROW({
      const test::FailAllocation failure(after);
      stamp.emplace(session_stamp(time));
    });
    ASSERT_TRUE(stamp.has_value());
    if (!test::allocation_failure.injected) {
      EXPECT_TRUE(stamp->ok()) << stamp->status().message();
      return;
    }
    EXPECT_EQ(stamp->status().domain(), Code::OutOfMemory);
  });
}

}  // namespace
}  // namespace volumetric_kit::calib::capture
