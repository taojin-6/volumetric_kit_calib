// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's Scan mode: every camera's picture saved at a press, no board.
// From host data, through the capture worker and the session writer: each
// V210 camera's words made an RGB PNG that decodes to the picture's colour
// (a neutral ramp to the code, a coloured one to sensor's v210_to_rgb8), the
// origin tag found in its luma and recorded, and the capture's file with no
// board's keys; a JPEG camera's JPEG byte for byte, its depth beside it, and
// no cap on the captures; a still that cannot be made failing its capture; a
// colour copy refused for its layout or size; and every capture queued
// written when the app quits. Then end to end on a device, through a fake
// DeckLink pair of V210 pictures in buffers: a press takes the next pair at
// once with nothing searched, its stills in colour; the clicker's "back"
// drops it; a switch to Poses after it closes the scan's session, and one
// back to Scan opens another. The device part runs under the validation
// layer when the environment asks (VKC_TEST_VALIDATION) and skips without a
// device.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include "fake_stereo.hpp"
#include "live_pair.hpp"
#include "scratch_dir.hpp"
#include "synthetic_rig.hpp"
#include "v210_pair.hpp"
#include "volumetric_kit/calib/app/calib_mode.hpp"
#include "volumetric_kit/calib/app/capture_worker.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/app/open_source.hpp"
#include "volumetric_kit/calib/app/preset.hpp"
#include "volumetric_kit/calib/app/rig_setup.hpp"
#include "volumetric_kit/calib/source/rig_source.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"
#include "volumetric_kit/sensor/io/image_io.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = sensor::camera;
namespace fs = std::filesystem;
using json = nlohmann::json;
using Code = core::Status::Code;
using test::read;
using test::ScratchDir;

// Small pictures, each row 40 groups of six pixels, padded as a card pads.
constexpr std::uint32_t kWidth = 240;
constexpr std::uint32_t kHeight = 136;
constexpr std::size_t kStride = 768;  // 640 bytes of words
constexpr camera::ImageSize kSize{kWidth, kHeight};
constexpr std::uint64_t kStartNs = 1'762'696'971'000'000'000ull;

// A neutral grey ramp: pixel x's luma 4 x (16 + x % 220), so its R'G'B' is
// (x % 220) / 219 of full scale, which rounds to no tie; Cb = Cr = 512.
test::V210Samples grey_ramp() {
  return test::v210_samples(
      kWidth, kHeight,
      [](std::uint32_t x, std::uint32_t) {
        return static_cast<std::uint16_t>(4 * (16 + x % 220));
      },
      [](std::uint32_t, std::uint32_t) {
        return std::make_pair(std::uint16_t{512}, std::uint16_t{512});
      });
}

// The ramp's byte at column x: 255 (x % 220) / 219, rounded.
std::uint8_t ramp_byte(std::uint32_t x) {
  const std::uint32_t v = x % 220;
  return static_cast<std::uint8_t>((510 * v + 219) / 438);
}

// A coloured picture: luma and both chroma moving across and down.
test::V210Samples colour_wash(int seed) {
  return test::v210_samples(
      kWidth, kHeight,
      [seed](std::uint32_t x, std::uint32_t y) {
        return static_cast<std::uint16_t>(64 + (x * 3 + y * 5 + seed) % 876);
      },
      [seed](std::uint32_t c, std::uint32_t y) {
        return std::make_pair(
            static_cast<std::uint16_t>(64 + (c * 7 + y + seed) % 896),
            static_cast<std::uint16_t>(960 - (c * 5 + y * 3) % 896));
      });
}

// A still's pixels as sensor decodes the PNG, R'G'B' bytes.
std::vector<std::uint8_t> decoded(const fs::path& path) {
  const core::Result<std::vector<std::uint32_t>> packed =
      sensor::io::load_color_packed(path.string(), kWidth, kHeight);
  EXPECT_TRUE(packed.ok()) << packed.status().message();
  std::vector<std::uint8_t> rgb;
  if (!packed.ok()) return rgb;
  rgb.reserve(packed->size() * 3);
  for (const std::uint32_t p : packed.value()) {
    rgb.push_back(static_cast<std::uint8_t>(p));
    rgb.push_back(static_cast<std::uint8_t>(p >> 8));
    rgb.push_back(static_cast<std::uint8_t>(p >> 16));
  }
  return rgb;
}

// Words as a capture's colour copy holds them: what the live worker copies
// out of the readback, the last row to its words' end.
ColourCopy colour_copy(const std::vector<std::uint8_t>& bytes) {
  ColourCopy copy;
  copy.words.assign(bytes.begin(),
                    bytes.begin() + static_cast<std::ptrdiff_t>(
                                        (kHeight - 1) * kStride +
                                        sensor::v210_row_bytes(kWidth)));
  sensor::YuvImage picture =
      test::device_v210_description(kWidth, kHeight, kStride);
  copy.picture = picture;
  return copy;
}

// A pair's camera record as the DeckLink arm gives it, its lens the
// stand-in a scan takes without a lens file.
capture::SessionCamera pair_camera(const char* id, const char* input) {
  capture::SessionCamera camera;
  camera.id = id;
  camera.model = "DeckLink 8K Pro";
  camera.size = kSize;
  camera.fps = 30;
  camera.input = input;
  return camera;
}

// The scan of a DeckLink pair, bm-5 on "SDI 2", bm-6 on "SDI 1", set up as
// calib_rig sets it (add_stereo_camera, apply_mode).
struct ScanRig {
  RigScenario scenario;
  CaptureSetup setup;
  LiveOptions live;
  RigMode window;
};

ScanRig scan_rig(const fs::path& root) {
  ScanRig rig;
  rig.scenario.mode = capture::SessionMode::Scan;
  rig.scenario.driver = "decklink";
  rig.scenario.pairs = {{"bm-5", "bm-6"}};
  OpenedSource opened;
  EXPECT_TRUE(
      add_stereo_camera(pair_camera("bm-5", "SDI 2"), rig.scenario, "", opened)
          .ok());
  EXPECT_TRUE(
      add_stereo_camera(pair_camera("bm-6", "SDI 1"), rig.scenario, "", opened)
          .ok());
  rig.setup.root = root.string();
  rig.setup.software = app_software();
  rig.setup.driver = "decklink";
  rig.setup.cameras = opened.cameras;
  rig.setup.lenses = opened.lenses;
  EXPECT_TRUE(apply_mode(rig.scenario, rig.setup, rig.live, rig.window).ok());
  return rig;
}

std::unique_ptr<CaptureWorker> make(const CaptureSetup& setup) {
  core::Result<std::unique_ptr<CaptureWorker>> made =
      CaptureWorker::create(setup);
  EXPECT_TRUE(made.ok()) << made.status().message();
  return made.ok() ? std::move(made).value() : nullptr;
}

// A scan's capture of the pair as the live worker hands it on: each camera's
// luma (blank but where a tag is drawn) and its V210 words, the pair's skew.
CaptureRequest pair_request(const std::vector<std::uint8_t>& left,
                            const std::vector<std::uint8_t>& right,
                            int tick = 0) {
  CaptureRequest out;
  out.board = legacy_femto_board();  // unread: a scan searches for none
  out.meta.time_ns = static_cast<std::int64_t>(kStartNs) + tick;
  out.meta.trigger_ns = kStartNs + static_cast<std::uint64_t>(tick);
  out.meta.sequence = 300 + static_cast<std::uint64_t>(tick);
  out.meta.spread_ms = 2.4;
  out.meta.skew_ns = 2'400'000;
  const std::vector<std::uint8_t>* words[2] = {&left, &right};
  for (std::size_t i = 0; i < 2; ++i) {
    CameraFrameCopy part;
    part.width = kWidth;
    part.height = kHeight;
    part.luma.assign(std::size_t{kWidth} * kHeight, 120);
    part.timestamp_ns = out.meta.trigger_ns + (i == 0 ? 0 : 2'400'000);
    part.sequence = out.meta.sequence;
    part.colour = colour_copy(*words[i]);
    out.cameras.push_back(std::move(part));
  }
  return out;
}

bool submit(CaptureWorker& worker, CaptureRequest r) {
  const core::Result<bool> queued = worker.submit(std::move(r));
  EXPECT_TRUE(queued.ok()) << queued.status().message();
  return queued.ok() && queued.value();
}

// Every event since the last take, once the worker is idle; each failure
// posted is a failure of the test.
std::vector<CaptureEvent> settle(CaptureWorker& worker) {
  worker.flush();
  for (const core::Status& error : worker.take_errors()) {
    ADD_FAILURE() << error.message();
  }
  return worker.take_events();
}

using Corners = std::array<cv::Point2d, 4>;

// The origin tag drawn into camera `i`'s luma in `r`, a print 60 px across.
Corners draw_origin_tag(CaptureRequest& r, std::size_t i) {
  CameraFrameCopy& part = r.cameras[i];
  cv::Mat luma(static_cast<int>(part.height), static_cast<int>(part.width),
               CV_8UC1, part.luma.data());
  return test::draw_marker(luma, kOriginTag.dictionary, kOriginTag.id,
                           test::marker_placement({70.4, 64.7}, 60.0, 9.0));
}

// --- From host data ---------------------------------------------------------

// Each camera's V210 words are converted and saved as an RGB PNG named for a
// scan, which decodes to the picture's colour: the grey ramp to its codes,
// the coloured wash to v210_to_rgb8's bytes. The origin tag drawn in one
// camera's luma is found and recorded; the capture's file holds each still's
// size and the pair's skew, and no board's keys; session.json says "scan".
TEST(AppScanHost, EachV210PictureIsSavedAsAColourPng) {
  const ScratchDir root;
  ScanRig rig = scan_rig(root.path());
  EXPECT_EQ(rig.setup.mode, capture::SessionMode::Scan);
  std::unique_ptr<CaptureWorker> worker = make(rig.setup);
  ASSERT_NE(worker, nullptr);
  const std::vector<std::uint8_t> left = test::pack_v210(grey_ramp(), kStride);
  const std::vector<std::uint8_t> right =
      test::pack_v210(colour_wash(3), kStride);
  CaptureRequest r = pair_request(left, right);
  const Corners truth = draw_origin_tag(r, 0);
  ASSERT_TRUE(submit(*worker, std::move(r)));
  const std::vector<CaptureEvent> events = settle(*worker);
  ASSERT_EQ(events.size(), 1u);
  ASSERT_EQ(events[0].kind, CaptureEvent::Kind::Saved)
      << events[0].status.message();
  EXPECT_EQ(events[0].meta.index, 0);
  EXPECT_EQ(events[0].tag_cameras, 1);
  EXPECT_EQ(events[0].usable_views, 0);
  ASSERT_EQ(events[0].pictures.size(), 2u);
  EXPECT_EQ(events[0].pictures[0].file, "images/bm-5/scan_bm-5_000.png");
  EXPECT_EQ(events[0].pictures[1].file, "images/bm-6/scan_bm-6_000.png");

  const fs::path dir = worker->directory();
  ASSERT_FALSE(dir.empty());
  const std::vector<std::uint8_t> ramp =
      decoded(dir / events[0].pictures[0].file);
  ASSERT_EQ(ramp.size(), std::size_t{kWidth} * kHeight * 3);
  for (std::uint32_t y = 0; y < kHeight; y += 45) {
    for (std::uint32_t x = 0; x < kWidth; ++x) {
      const std::size_t at = (std::size_t{y} * kWidth + x) * 3;
      ASSERT_EQ(ramp[at], ramp_byte(x)) << x << ", " << y;
      ASSERT_EQ(ramp[at + 1], ramp_byte(x)) << x << ", " << y;
      ASSERT_EQ(ramp[at + 2], ramp_byte(x)) << x << ", " << y;
    }
  }
  const sensor::YuvImage described =
      test::device_v210_description(kWidth, kHeight, kStride);
  EXPECT_EQ(ramp, test::v210_rgb(described, left).value());
  EXPECT_EQ(decoded(dir / events[0].pictures[1].file),
            test::v210_rgb(described, right).value());
  for (const capture::PictureFile& still : events[0].pictures) {
    EXPECT_EQ(still.bytes, fs::file_size(dir / still.file)) << still.file;
  }

  const json capture = json::parse(read(dir / "captures" / "000.json"));
  EXPECT_EQ(capture["skew_ns"], 2'400'000);
  EXPECT_EQ(capture["spread_ms"], 2.4);
  EXPECT_FALSE(capture.contains("stillness_px"));
  EXPECT_FALSE(capture.contains("usable_views"));
  const json& bm5 = capture["cameras"]["bm-5"];
  EXPECT_EQ(bm5["color"], "images/bm-5/scan_bm-5_000.png");
  EXPECT_EQ(bm5["color_bytes"], events[0].pictures[0].bytes);
  EXPECT_TRUE(bm5["depth"].is_null());
  EXPECT_FALSE(bm5.contains("detection"));
  EXPECT_FALSE(bm5.contains("check"));
  ASSERT_TRUE(bm5["origin_tag"].is_object());
  for (std::size_t k = 0; k < 4; ++k) {
    const cv::Point2d found(
        bm5["origin_tag"]["corners_px"][k][0].get<double>(),
        bm5["origin_tag"]["corners_px"][k][1].get<double>());
    EXPECT_LT(cv::norm(found - truth[k]), 0.5) << "corner " << k;
  }
  EXPECT_TRUE(capture["cameras"]["bm-6"]["origin_tag"].is_null());
  const json session = json::parse(read(dir / "session.json"));
  EXPECT_EQ(session["mode"], "scan");
  EXPECT_FALSE(session.contains("board"));
  EXPECT_EQ(session["rig"]["driver"], "decklink");
  EXPECT_EQ(session["cameras"][0]["input"], "SDI 2");
  EXPECT_EQ(session["cameras"][1]["input"], "SDI 1");
  EXPECT_EQ(session["cameras"][0]["lens"]["origin"], "stand-in");

  // A scan has no checks or Calibrate: its captures kept are its count.
  EXPECT_FALSE(worker->summary().has_value());
  const std::shared_ptr<const KeptCaptures> kept = worker->kept();
  ASSERT_NE(kept, nullptr);
  EXPECT_EQ(kept->mode, capture::SessionMode::Scan);
  ASSERT_EQ(kept->captures.size(), 1u);
  EXPECT_EQ(kept->captures[0]->depth,
            (std::vector<std::shared_ptr<const std::vector<std::uint16_t>>>(
                2, nullptr)));
}

// A Femto rig's scan: each camera's JPEG byte for byte, its depth beside it,
// as Lenses and Poses write them; a scan keeps no depth for Calibrate, and
// takes more captures than one Calibrate would adjust.
TEST(AppScanHost, AJpegCameraKeepsItsJpegAndItsDepth) {
  const ScratchDir root;
  CaptureSetup setup;
  setup.root = root.path().string();
  setup.software = app_software();
  setup.driver = "orbbec";
  setup.mode = capture::SessionMode::Scan;
  setup.depth_mode = depth_mode({640, 576}).value();
  setup.color_codec = "mjpeg";
  setup.max_kept = 1;
  for (std::size_t i = 0; i < 2; ++i) {
    capture::SessionCamera camera;
    camera.id = "SYN" + std::to_string(i);
    camera.model = "fake";
    camera.size = kSize;
    camera.color = stand_in_lens(kSize);
    camera.depth = test::depth_like_lens();
    camera.fps = 25;
    setup.cameras.push_back(camera);
    setup.lenses.push_back(stand_in_lens(kSize));
  }
  std::unique_ptr<CaptureWorker> worker = make(setup);
  ASSERT_NE(worker, nullptr);
  const auto request = [](int tick) {
    CaptureRequest r;
    r.board = legacy_femto_board();
    r.meta.trigger_ns = kStartNs + static_cast<std::uint64_t>(tick);
    for (std::size_t i = 0; i < 2; ++i) {
      CameraFrameCopy part;
      part.width = kWidth;
      part.height = kHeight;
      part.luma.assign(std::size_t{kWidth} * kHeight, 90);
      part.jpeg = {0xFF,
                   0xD8,
                   0xFF,
                   0xE0,
                   static_cast<std::uint8_t>(i),
                   static_cast<std::uint8_t>(tick),
                   0xFF,
                   0xD9};
      part.depth.assign(640 * 576, static_cast<std::uint16_t>(1000 + i));
      part.metres_per_unit = 0.001f;
      r.cameras.push_back(std::move(part));
    }
    return r;
  };
  for (int tick = 0; tick < 3; ++tick) {
    ASSERT_TRUE(submit(*worker, request(tick)));
  }
  const std::vector<CaptureEvent> events = settle(*worker);
  ASSERT_EQ(events.size(), 3u);
  for (const CaptureEvent& event : events) {
    ASSERT_EQ(event.kind, CaptureEvent::Kind::Saved) << event.status.message();
  }
  const fs::path dir = worker->directory();
  EXPECT_EQ(read(dir / "images" / "SYN1" / "scan_SYN1_002.jpg"),
            std::string({'\xFF', '\xD8', '\xFF', '\xE0', '\x01', '\x02', '\xFF',
                         '\xD9'}));
  EXPECT_EQ(events[2].pictures[1].file, "images/SYN1/scan_SYN1_002.jpg");
  EXPECT_TRUE(fs::exists(dir / "depth" / "SYN0" / "calib_depth_SYN0_000.png"));
  const json capture = json::parse(read(dir / "captures" / "001.json"));
  EXPECT_FALSE(capture.contains("skew_ns"));
  EXPECT_EQ(capture["cameras"]["SYN0"]["depth"],
            "depth/SYN0/calib_depth_SYN0_001.png");
  const std::shared_ptr<const KeptCaptures> kept = worker->kept();
  ASSERT_NE(kept, nullptr);
  ASSERT_EQ(kept->captures.size(), 3u);
  EXPECT_EQ(kept->captures[0]->depth[0], nullptr);
}

// A still that cannot be made fails its capture, naming the camera, and
// writes nothing of it; a colour copy that is not V210 at the camera's size
// is refused at once.
TEST(AppScanHost, AStillThatCannotBeMadeFailsItsCapture) {
  const ScratchDir root;
  ScanRig rig = scan_rig(root.path());
  std::unique_ptr<CaptureWorker> worker = make(rig.setup);
  ASSERT_NE(worker, nullptr);
  const std::vector<std::uint8_t> words =
      test::pack_v210(colour_wash(1), kStride);
  CaptureRequest r = pair_request(words, words);
  r.cameras[1].colour->words.resize(100);      // short of its rows
  r.cameras[1].colour->picture.stride[0] = 8;  // shorter than its row
  ASSERT_TRUE(submit(*worker, std::move(r)));
  std::vector<CaptureEvent> events = settle(*worker);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].kind, CaptureEvent::Kind::Failed);
  EXPECT_EQ(events[0].status.domain(), Code::InvalidArgument);
  EXPECT_EQ(
      events[0].status.message().rfind("capture: camera bm-6: still: ", 0), 0u)
      << events[0].status.message();
  EXPECT_TRUE(worker->directory().empty() ||
              !fs::exists(fs::path(worker->directory()) / "captures"));

  CaptureRequest grey = pair_request(words, words);
  grey.cameras[0].colour->picture.layout = sensor::YuvLayout::Luma;
  const core::Result<bool> refused = worker->submit(std::move(grey));
  ASSERT_FALSE(refused.ok());
  EXPECT_EQ(refused.status().message(),
            "capture: camera bm-5's colour is not V210 at its pictures' size");
  CaptureRequest small = pair_request(words, words);
  small.cameras[1].colour->picture.width = kWidth / 2;
  EXPECT_FALSE(worker->submit(std::move(small)).ok());
}

// Quitting the app destroys the capture worker: every capture queued is
// converted and written first, none lost.
TEST(AppScanHost, QuittingWritesEveryCaptureStillQueued) {
  const ScratchDir root;
  ScanRig rig = scan_rig(root.path());
  fs::path dir;
  {
    std::unique_ptr<CaptureWorker> worker = make(rig.setup);
    ASSERT_NE(worker, nullptr);
    const std::vector<std::uint8_t> words =
        test::pack_v210(colour_wash(2), kStride);
    for (int tick = 0; tick < 4; ++tick) {
      ASSERT_TRUE(submit(*worker, pair_request(words, words, tick)));
    }
  }
  for (const auto& entry : fs::directory_iterator(root.path())) {
    if (entry.is_directory()) dir = entry.path();
  }
  ASSERT_FALSE(dir.empty());
  const json session = json::parse(read(dir / "session.json"));
  ASSERT_EQ(session["captures"].size(), 4u);
  for (int index = 0; index < 4; ++index) {
    EXPECT_FALSE(session["captures"][index]["dropped"].get<bool>());
    for (const char* id : {"bm-5", "bm-6"}) {
      char file[64];
      std::snprintf(file, sizeof file, "images/%s/scan_%s_%03d.png", id, id,
                    index);
      EXPECT_TRUE(fs::exists(dir / file)) << file;
    }
  }
}

// --- End to end, on a device ------------------------------------------------

class AppScan : public core::test::VulkanDeviceTest {
 protected:
  core::DeviceRequirements requirements() const override {
    return sensor::device_requirements();
  }

  // `samples` packed and put in a buffer on the device.
  sensor::YuvImage on_device(const std::vector<std::uint8_t>& bytes) {
    core::Result<sensor::YuvImage> made = test::device_v210(
        device(), allocator(), bytes, kWidth, kHeight, kStride);
    EXPECT_TRUE(made.ok()) << made.status().message();
    return made.ok() ? std::move(made).value() : sensor::YuvImage{};
  }
};

// The owner's flow: a press, with no board anywhere, takes the next pair at
// once -- nothing searched, no stillness waited for -- and saves both
// pictures in colour; "back" drops it, kept on disk. A switch to Poses after
// it closes the scan's session; one back to Scan opens a new one at the next
// press.
TEST_F(AppScan, APressTakesTheNextPairInColour) {
  const ScratchDir root;
  ScanRig rig = scan_rig(root.path());
  ASSERT_TRUE(rig.live.scan);
  ASSERT_TRUE(rig.window.scan);
  std::unique_ptr<CaptureWorker> captures = make(rig.setup);
  ASSERT_NE(captures, nullptr);
  auto stereo =
      std::make_unique<test::FakeStereo>(test::uncalibrated_pair_info());
  test::FakeStereo& fake = *stereo;
  source::StereoMember member{std::move(stereo), {"bm-5", "bm-6"}};
  member.sizes = {kSize, kSize};
  core::Result<std::unique_ptr<source::RigSource>> source =
      source::RigSource::open(std::move(member), device(), allocator());
  ASSERT_TRUE(source.ok()) << source.status().message();
  core::Result<std::unique_ptr<LiveWorker>> started = LiveWorker::create(
      std::move(source).value(), rig.setup.lenses, board_seed(rig.setup.mode),
      false, captures.get(), nullptr, rig.live);
  ASSERT_TRUE(started.ok()) << started.status().message();
  const std::unique_ptr<LiveWorker> live = std::move(started).value();
  EXPECT_FALSE(live->rule_options().still);

  const std::vector<std::uint8_t> left = test::pack_v210(grey_ramp(), kStride);
  const std::vector<std::uint8_t> right =
      test::pack_v210(colour_wash(5), kStride);
  const sensor::YuvImage pictures[2] = {on_device(left), on_device(right)};
  std::uint64_t tick = 0;
  std::uint64_t seen = 0;
  const auto show = [&]() -> std::shared_ptr<const LiveSnapshot> {
    ++tick;
    fake.push(test::v210_pair(pictures[0], pictures[1],
                              kStartNs + tick * 33'333'333, -1'700'000, tick));
    std::shared_ptr<const LiveSnapshot> s = test::next_snapshot(*live, seen);
    if (s) seen = s->number;
    return s;
  };
  const auto press = [&]() -> std::shared_ptr<const LiveSnapshot> {
    if (!live->arm()) return nullptr;
    test::wait_armed(*live);
    return show();
  };

  // Not armed: each pair's grey tiles, nothing searched or taken.
  const std::shared_ptr<const LiveSnapshot> idle = show();
  ASSERT_NE(idle, nullptr);
  ASSERT_EQ(idle->detections.size(), 2u);
  EXPECT_TRUE(idle->detections[0].observation.empty());
  EXPECT_EQ(idle->decision.state, quality::CaptureState::Idle);
  ASSERT_NE(idle->set->cameras[0].tile, nullptr);
  EXPECT_EQ(idle->set->skew_ns, std::optional<std::int64_t>(-1'700'000));

  // A press takes the very next pair.
  std::shared_ptr<const LiveSnapshot> taken = press();
  ASSERT_NE(taken, nullptr);
  ASSERT_EQ(taken->decision.state, quality::CaptureState::Take)
      << taken->decision.message;
  EXPECT_EQ(taken->decision.message, "");
  EXPECT_FALSE(taken->stages.rows().empty());
  std::vector<CaptureEvent> events = settle(*captures);
  ASSERT_EQ(events.size(), 1u);
  ASSERT_EQ(events[0].kind, CaptureEvent::Kind::Saved)
      << events[0].status.message();
  EXPECT_EQ(events[0].meta.skew_ns, std::optional<std::int64_t>(-1'700'000));
  const fs::path first = captures->directory();
  const sensor::YuvImage described =
      test::device_v210_description(kWidth, kHeight, kStride);
  EXPECT_EQ(decoded(first / "images" / "bm-5" / "scan_bm-5_000.png"),
            test::v210_rgb(described, left).value());
  EXPECT_EQ(decoded(first / "images" / "bm-6" / "scan_bm-6_000.png"),
            test::v210_rgb(described, right).value());
  const json capture = json::parse(read(first / "captures" / "000.json"));
  EXPECT_EQ(capture["skew_ns"], -1'700'000);
  EXPECT_EQ(capture["cameras"]["bm-6"]["sequence"], tick);

  // The clicker's "back": dropped, kept on disk.
  ASSERT_TRUE(captures->drop_last().ok());
  events = settle(*captures);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].kind, CaptureEvent::Kind::Dropped);
  EXPECT_EQ(events[0].meta.index, 0);
  EXPECT_TRUE(
      json::parse(read(first / "session.json"))["captures"][0]["dropped"]
          .get<bool>());
  EXPECT_TRUE(fs::exists(first / "images" / "bm-5" / "scan_bm-5_000.png"));

  // Poses: the scan's session closes, the live view searches again.
  RigScenario next = rig.scenario;
  next.mode = capture::SessionMode::Extrinsics;
  core::Result<viz::BoardEditor> editor = switch_session(
      next, std::nullopt, rig.setup, rig.live, rig.window, *live, *captures);
  ASSERT_TRUE(editor.ok()) << editor.status().message();
  EXPECT_FALSE(rig.window.scan);
  EXPECT_FALSE(rig.live.scan);
  EXPECT_TRUE(live->rule_options().still);
  captures->flush();
  EXPECT_EQ(captures->kept(), nullptr);
  EXPECT_TRUE(captures->directory().empty());

  // Back to Scan: the next press opens a new session.
  next.mode = capture::SessionMode::Scan;
  editor = switch_session(next, std::nullopt, rig.setup, rig.live, rig.window,
                          *live, *captures);
  ASSERT_TRUE(editor.ok()) << editor.status().message();
  EXPECT_TRUE(rig.window.scan);
  ASSERT_NE(show(), nullptr);
  taken = press();
  ASSERT_NE(taken, nullptr);
  ASSERT_EQ(taken->decision.state, quality::CaptureState::Take)
      << taken->decision.message;
  events = settle(*captures);
  ASSERT_EQ(events.size(), 1u);
  ASSERT_EQ(events[0].kind, CaptureEvent::Kind::Saved)
      << events[0].status.message();
  const fs::path second = captures->directory();
  EXPECT_NE(second, first);
  EXPECT_EQ(json::parse(read(second / "session.json"))["mode"], "scan");
  EXPECT_TRUE(fs::exists(second / "images" / "bm-6" / "scan_bm-6_000.png"));
  live->stop();
  for (const core::Status& error : live->take_errors()) {
    ADD_FAILURE() << error.message();
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::app
