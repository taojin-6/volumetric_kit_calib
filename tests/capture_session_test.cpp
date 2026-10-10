// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/capture/session.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "capture_test_util.hpp"

namespace volumetric_kit::calib::capture {
namespace {

using testing::flat_image;
using testing::make_camera;
using testing::pattern_image;
using testing::ScratchDir;

bool exists(const std::string& path) {
  std::error_code ec;
  return std::filesystem::exists(path, ec);
}

void write_file(const std::string& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << text;
}

// Two cameras of different sizes, so a mixed-up image or lens shows.
std::vector<CameraInfo> two_cameras() {
  return {make_camera("CL2A141000N", SyncRole::Primary, 64, 48),
          make_camera("CL2A141000G", SyncRole::Secondary, 32, 24)};
}

std::vector<LumaImage> images_for(const std::vector<CameraInfo>& cameras,
                                  unsigned seed) {
  std::vector<LumaImage> images;
  for (const CameraInfo& camera : cameras) {
    images.push_back(pattern_image(camera.lens->image_size(), seed++));
  }
  return images;
}

void expect_same_lens(const PinholeBrownConrady& a,
                      const PinholeBrownConrady& b) {
  EXPECT_EQ(a.image_size().width, b.image_size().width);
  EXPECT_EQ(a.image_size().height, b.image_size().height);
  EXPECT_EQ(a.intrinsics().fx, b.intrinsics().fx);
  EXPECT_EQ(a.intrinsics().fy, b.intrinsics().fy);
  EXPECT_EQ(a.intrinsics().cx, b.intrinsics().cx);
  EXPECT_EQ(a.intrinsics().cy, b.intrinsics().cy);
  EXPECT_EQ(a.distortion().k1, b.distortion().k1);
  EXPECT_EQ(a.distortion().k2, b.distortion().k2);
  EXPECT_EQ(a.distortion().p1, b.distortion().p1);
  EXPECT_EQ(a.distortion().p2, b.distortion().p2);
  EXPECT_EQ(a.distortion().k3, b.distortion().k3);
}

TEST(CaptureSession, WritesAndReadsBackASession) {
  const ScratchDir dir;
  const std::vector<CameraInfo> cameras = two_cameras();
  auto writer = SessionWriter::create(dir.str(), cameras);
  ASSERT_TRUE(writer.ok()) << writer.status().message();
  auto first = writer->add(1000, 5, images_for(cameras, 1));
  auto second = writer->add(2000, 1, images_for(cameras, 7));
  ASSERT_TRUE(first.ok()) << first.status().message();
  ASSERT_TRUE(second.ok()) << second.status().message();
  EXPECT_EQ(*first, 0u);
  EXPECT_EQ(*second, 1u);

  auto session = read_session(dir.str());
  ASSERT_TRUE(session.ok()) << session.status().message();
  ASSERT_EQ(session->cameras.size(), 2u);
  for (std::size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(session->cameras[i].serial, cameras[i].serial);
    EXPECT_EQ(session->cameras[i].role, cameras[i].role);
    ASSERT_TRUE(session->cameras[i].lens.has_value());
    // Exactly: the JSON must round-trip every double.
    expect_same_lens(*session->cameras[i].lens, *cameras[i].lens);
  }
  ASSERT_EQ(session->captures.size(), 2u);
  EXPECT_EQ(session->captures[0].index, 0u);
  EXPECT_EQ(session->captures[0].trigger_ns, 1000u);
  EXPECT_EQ(session->captures[0].frames_averaged, 5u);
  EXPECT_EQ(session->captures[1].index, 1u);
  EXPECT_EQ(session->captures[1].trigger_ns, 2000u);
  EXPECT_EQ(session->captures[1].frames_averaged, 1u);
}

TEST(CaptureSession, StoresEachImageLosslesslyUnderTheLegacyName) {
  const ScratchDir dir;
  const std::vector<CameraInfo> cameras = two_cameras();
  auto writer = SessionWriter::create(dir.str(), cameras);
  ASSERT_TRUE(writer.ok()) << writer.status().message();
  const std::vector<LumaImage> images = images_for(cameras, 3);
  ASSERT_TRUE(writer->add(42, 1, images).ok());

  // The name the legacy camera_calibration scripts glob for.
  EXPECT_EQ(session_image_path(dir.str(), "CL2A141000G", 0),
            dir / "CL2A141000G/calib_img_CL2A141000G_0.png");
  for (std::size_t i = 0; i < cameras.size(); ++i) {
    EXPECT_TRUE(exists(session_image_path(dir.str(), cameras[i].serial, 0)));
    auto read = read_session_image(dir.str(), cameras[i].serial, 0);
    ASSERT_TRUE(read.ok()) << read.status().message();
    EXPECT_EQ(read->size.width, images[i].size.width);
    EXPECT_EQ(read->size.height, images[i].size.height);
    EXPECT_EQ(read->pixels, images[i].pixels);
  }
}

TEST(CaptureSession, RemoveLastDeletesTheCaptureAndReusesItsIndex) {
  const ScratchDir dir;
  const std::vector<CameraInfo> cameras = two_cameras();
  auto writer = SessionWriter::create(dir.str(), cameras);
  ASSERT_TRUE(writer.ok());
  ASSERT_TRUE(writer->add(1, 1, images_for(cameras, 1)).ok());
  ASSERT_TRUE(writer->add(2, 1, images_for(cameras, 2)).ok());

  ASSERT_TRUE(writer->remove_last().ok());
  ASSERT_EQ(writer->session().captures.size(), 1u);
  for (const CameraInfo& camera : cameras) {
    EXPECT_TRUE(exists(session_image_path(dir.str(), camera.serial, 0)));
    EXPECT_FALSE(exists(session_image_path(dir.str(), camera.serial, 1)));
  }
  auto session = read_session(dir.str());
  ASSERT_TRUE(session.ok());
  EXPECT_EQ(session->captures.size(), 1u);

  // Contiguous indices, as the legacy scripts' pairing by name needs.
  auto again = writer->add(3, 1, images_for(cameras, 3));
  ASSERT_TRUE(again.ok());
  EXPECT_EQ(*again, 1u);
}

TEST(CaptureSession, RemoveLastWithNoCaptureIsNotFound) {
  const ScratchDir dir;
  auto writer = SessionWriter::create(dir.str(), two_cameras());
  ASSERT_TRUE(writer.ok());
  EXPECT_EQ(writer->remove_last().domain(), core::Status::Code::NotFound);
}

TEST(CaptureSession, CreateRefusesCamerasASessionCannotHold) {
  const ScratchDir dir;
  EXPECT_EQ(SessionWriter::create(dir.str(), {}).status().domain(),
            core::Status::Code::InvalidArgument);

  std::vector<CameraInfo> no_lens = two_cameras();
  no_lens[1].lens.reset();
  EXPECT_EQ(SessionWriter::create(dir.str(), no_lens).status().domain(),
            core::Status::Code::InvalidArgument);

  std::vector<CameraInfo> repeated = two_cameras();
  repeated[1].serial = repeated[0].serial;
  EXPECT_EQ(SessionWriter::create(dir.str(), repeated).status().domain(),
            core::Status::Code::InvalidArgument);

  for (const char* serial : {"", ".", "..", "a/b", "a\\b", "a b", "../x"}) {
    std::vector<CameraInfo> bad = two_cameras();
    bad[0].serial = serial;
    EXPECT_EQ(SessionWriter::create(dir.str(), bad).status().domain(),
              core::Status::Code::InvalidArgument)
        << "serial \"" << serial << "\"";
  }
  // Nothing was created for any refused session.
  EXPECT_FALSE(exists(dir / "session.json"));
}

TEST(CaptureSession, CreateRefusesADirectoryThatHoldsASession) {
  const ScratchDir dir;
  ASSERT_TRUE(SessionWriter::create(dir.str(), two_cameras()).ok());
  EXPECT_EQ(SessionWriter::create(dir.str(), two_cameras()).status().domain(),
            core::Status::Code::InvalidArgument);
}

TEST(CaptureSession, AddRefusesImagesThatDoNotFitTheCameras) {
  const ScratchDir dir;
  const std::vector<CameraInfo> cameras = two_cameras();
  auto writer = SessionWriter::create(dir.str(), cameras);
  ASSERT_TRUE(writer.ok());

  std::vector<LumaImage> one = images_for(cameras, 1);
  one.pop_back();
  EXPECT_EQ(writer->add(1, 1, one).status().domain(),
            core::Status::Code::InvalidArgument);

  std::vector<LumaImage> swapped = images_for(cameras, 1);
  std::swap(swapped[0], swapped[1]);  // each the other camera's size
  EXPECT_EQ(writer->add(1, 1, swapped).status().domain(),
            core::Status::Code::InvalidArgument);

  std::vector<LumaImage> short_image = images_for(cameras, 1);
  short_image[0].pixels.pop_back();
  EXPECT_EQ(writer->add(1, 1, short_image).status().domain(),
            core::Status::Code::InvalidArgument);

  EXPECT_EQ(writer->add(1, 0, images_for(cameras, 1)).status().domain(),
            core::Status::Code::InvalidArgument);

  // Nothing was written for any refused capture.
  EXPECT_TRUE(writer->session().captures.empty());
  for (const CameraInfo& camera : cameras) {
    EXPECT_FALSE(exists(session_image_path(dir.str(), camera.serial, 0)));
  }
}

TEST(CaptureSession, ReadSessionOfAMissingDirectoryIsNotFound) {
  const ScratchDir dir;
  EXPECT_EQ(read_session(dir.str()).status().domain(),
            core::Status::Code::NotFound);
}

// A valid session.json, and single-edit breakages of it, each refused.
TEST(CaptureSession, ReadSessionRefusesMalformedFiles) {
  const ScratchDir dir;
  ASSERT_TRUE(SessionWriter::create(dir.str(), two_cameras()).ok());
  const std::string lens =
      R"("lens": {"width": 64, "height": 48, "fx": 57.6, "fy": 57.6,
                  "cx": 31.5, "cy": 23.5, "k1": 0, "k2": 0, "p1": 0, "p2": 0,
                  "k3": 0})";
  const auto doc = [](const std::string& format, const std::string& role,
                      const std::string& lens_json,
                      const std::string& captures) {
    return R"({"format": )" + format + R"(, "version": 1, "cameras": [
               {"serial": "CL2A141000N", "role": )" +
           role + ", " + lens_json + R"(}], "captures": )" + captures + "}";
  };
  const std::string good_format = R"("volumetric_kit_calib.session")";
  const std::string good_capture =
      R"([{"index": 0, "trigger_ns": 5, "frames_averaged": 1}])";

  write_file(dir / "session.json",
             doc(good_format, R"("primary")", lens, good_capture));
  ASSERT_TRUE(read_session(dir.str()).ok())
      << read_session(dir.str()).status().message();

  const std::vector<std::string> broken = {
      "not json at all",
      "[1, 2, 3]",
      doc(R"("something.else")", R"("primary")", lens, good_capture),
      doc(good_format, R"("leader")", lens, good_capture),
      doc(good_format, R"("primary")", R"("lens": {"width": 64})",
          good_capture),
      doc(good_format, R"("primary")",
          R"("lens": {"width": 64, "height": 48, "fx": -1, "fy": 57.6,
                      "cx": 31.5, "cy": 23.5, "k1": 0, "k2": 0, "p1": 0,
                      "p2": 0, "k3": 0})",
          good_capture),
      doc(good_format, R"("primary")", lens,
          R"([{"index": 1, "trigger_ns": 5, "frames_averaged": 1},
              {"index": 1, "trigger_ns": 6, "frames_averaged": 1}])"),
      doc(good_format, R"("primary")", lens,
          R"([{"index": 0, "trigger_ns": 5, "frames_averaged": 0}])"),
      doc(good_format, R"("primary")", lens,
          R"([{"index": -1, "trigger_ns": 5, "frames_averaged": 1}])"),
      R"({"format": "volumetric_kit_calib.session", "version": 2,
          "cameras": [], "captures": []})",
      R"({"format": "volumetric_kit_calib.session", "version": 1,
          "cameras": [], "captures": []})",
  };
  for (const std::string& text : broken) {
    write_file(dir / "session.json", text);
    EXPECT_EQ(read_session(dir.str()).status().domain(),
              core::Status::Code::InvalidArgument)
        << text;
  }
}

TEST(CaptureSession, ReadSessionImageNamesWhatIsWrong) {
  const ScratchDir dir;
  const std::vector<CameraInfo> cameras = two_cameras();
  ASSERT_TRUE(SessionWriter::create(dir.str(), cameras).ok());
  EXPECT_EQ(
      read_session_image(dir.str(), cameras[0].serial, 0).status().domain(),
      core::Status::Code::NotFound);

  write_file(session_image_path(dir.str(), cameras[0].serial, 0), "not a png");
  EXPECT_EQ(
      read_session_image(dir.str(), cameras[0].serial, 0).status().domain(),
      core::Status::Code::IoError);
}

TEST(CaptureSession, SerialsAreKeptToPlainFileNames) {
  EXPECT_TRUE(is_session_serial("CL2A141000N"));
  EXPECT_TRUE(is_session_serial("bm-1"));
  EXPECT_TRUE(is_session_serial("cam_0.left"));
  EXPECT_FALSE(is_session_serial(""));
  EXPECT_FALSE(is_session_serial("."));
  EXPECT_FALSE(is_session_serial(".."));
  EXPECT_FALSE(is_session_serial("a/b"));
  EXPECT_FALSE(is_session_serial("a b"));
}

TEST(CaptureSession, AFlatImageRoundTrips) {
  // The recorder's averages are often near-flat; make sure the smallest PNG
  // stb writes still decodes to the same bytes.
  const ScratchDir dir;
  const std::vector<CameraInfo> cameras = two_cameras();
  auto writer = SessionWriter::create(dir.str(), cameras);
  ASSERT_TRUE(writer.ok());
  std::vector<LumaImage> images;
  for (const CameraInfo& camera : cameras) {
    images.push_back(flat_image(camera.lens->image_size(), 200));
  }
  ASSERT_TRUE(writer->add(9, 3, images).ok());
  auto read = read_session_image(dir.str(), cameras[1].serial, 0);
  ASSERT_TRUE(read.ok());
  EXPECT_EQ(read->pixels, images[1].pixels);
}

}  // namespace
}  // namespace volumetric_kit::calib::capture
