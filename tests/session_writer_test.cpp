// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Sessions on disk: the tree and its names, the files' bytes and JSON, what is
// refused, the queue, I/O failures, drops and shutdown. Synthetic cameras and
// frames: small images, a few corners, numbers with long binary expansions.

#include "volumetric_kit/calib/capture/session_writer.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"
#include "volumetric_kit/recon/io/image_io.hpp"

namespace volumetric_kit::calib::capture {
namespace {

namespace fs = std::filesystem;
namespace camera = recon::camera;
namespace vkc = volumetric_kit::core;
using json = nlohmann::json;
using Code = vkc::Status::Code;

constexpr std::uint32_t kColorW = 64;
constexpr std::uint32_t kColorH = 48;
constexpr std::uint32_t kDepthW = 16;
constexpr std::uint32_t kDepthH = 12;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// A fresh directory under the system's temporary one, removed with what it
// holds when the test ends.
class ScratchDir {
 public:
  ScratchDir() {
    static std::atomic<int> counter{0};
    const auto* test = ::testing::UnitTest::GetInstance()->current_test_info();
    std::error_code ec;
    path_ = fs::temp_directory_path(ec) /
            ("vc_capture_" + std::to_string(::getpid()) + "_" +
             test->test_suite_name() + "_" + test->name() + "_" +
             std::to_string(counter++));
    fs::remove_all(path_, ec);
    fs::create_directories(path_, ec);
  }
  ~ScratchDir() {
    std::error_code ec;
    // A test that made a directory read-only may fail before restoring it.
    for (auto it = fs::recursive_directory_iterator(path_, ec);
         it != fs::recursive_directory_iterator(); it.increment(ec)) {
      if (it->is_directory(ec)) {
        fs::permissions(it->path(), fs::perms::owner_all, fs::perm_options::add,
                        ec);
      }
    }
    fs::remove_all(path_, ec);
  }
  ScratchDir(const ScratchDir&) = delete;
  ScratchDir& operator=(const ScratchDir&) = delete;

  std::string operator/(const std::string& name) const {
    return (path_ / name).string();
  }

 private:
  fs::path path_;
};

std::string read(const std::string& path) {
  vkc::Result<std::string> bytes = vkc::read_file(path);
  EXPECT_TRUE(bytes.ok()) << bytes.status().message();
  return bytes.ok() ? *bytes : std::string();
}

void write(const std::string& path, const std::string& bytes) {
  const vkc::Status written =
      vkc::write_file(path, bytes, vkc::WriteMode::Replace);
  ASSERT_TRUE(written.ok()) << written.message();
}

json parse(const std::string& path) { return json::parse(read(path)); }

// Every file under `root`, by its path relative to it.
std::set<std::string> files_under(const std::string& root) {
  std::set<std::string> files;
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (entry.is_regular_file()) {
      files.insert(fs::relative(entry.path(), root).generic_string());
    }
  }
  return files;
}

// Every file under `root` with its bytes.
std::map<std::string, std::string> snapshot(const std::string& root) {
  std::map<std::string, std::string> files;
  for (const std::string& name : files_under(root)) {
    files[name] = read(root + "/" + name);
  }
  return files;
}

// --- Synthetic rig ----------------------------------------------------------

camera::CameraModel color_model(int seed) {
  camera::CameraModel m;
  m.size = {kColorW, kColorH};
  m.intrinsics = {52.1234567890123 + seed, 51.98765432109876,
                  31.5 + 0.3141592653589793, 23.5 - 0.2718281828459045};
  m.distortion = {0.0731199458241463,
                  -0.1004820391535759,
                  -0.00046360507258214056,
                  -5.1091061322949827e-05,
                  0.040595322847366333,
                  0.0,
                  0.0,
                  0.0};
  return m;
}

// Like the Femto Mega's depth lens: a rational model with k4-k6 set.
camera::CameraModel depth_model(int seed) {
  camera::CameraModel m;
  m.size = {kDepthW, kDepthH};
  m.intrinsics = {12.6 + 0.01 * seed, 12.61, 7.83, 5.91};
  m.distortion = {29.29, 14.52, 3.1e-05, -2.2e-05, 0.61, 29.63, 23.99, 3.77};
  return m;
}

SessionCamera make_camera(const std::string& serial, int seed,
                          recon::sensor::SyncRole role) {
  SessionCamera c;
  c.info.id = serial;
  c.info.model = "Orbbec Femto Mega";
  c.info.color = color_model(seed);
  c.info.depth = depth_model(seed);
  c.info.depth_to_color = camera::matrix_from_rodrigues(
      {{0.0012 * seed, -0.0034, 0.0021}, {-0.0325, -0.0005, 0.0022}});
  c.info.role = role;
  c.info.fps = 25;
  c.firmware = "1.2.9";
  c.ip_address = "192.168.1." + std::to_string(10 + seed);
  c.sync_mode =
      role == recon::sensor::SyncRole::Primary ? "primary" : "secondary-synced";
  c.depth_delay_us = 160 * seed;
  c.color_lens_model = "brown-conrady";
  c.depth_lens_model = "brown-conrady-k6";
  return c;
}

Streams small_streams() {
  Streams s;
  s.color_width = kColorW;
  s.color_height = kColorH;
  s.color_codec = "mjpeg";
  s.depth_width = kDepthW;
  s.depth_height = kDepthH;
  s.depth_mode = "nfov_unbinned";
  s.metres_per_unit = 0.001f;
  s.min_depth_m = 0.5f;
  s.max_depth_m = 3.86f;
  s.fps = 25;
  return s;
}

// Two cameras, the second the reference, so neither order nor index 0 is
// taken for it. The sync file exists, holding "abc".
SessionInfo make_info(const ScratchDir& dir) {
  SessionInfo info;
  info.directory = dir / "2026-10-13_094107";
  info.software = {"calib-sha", "recon-sha", "gfx-sha",
                   "core-sha",  "5.0.0",     "2.10.6"};
  info.sync_path = dir / "sync.json";
  write(info.sync_path, "abc");
  info.reference = "CL2A141000N";
  info.board = legacy_femto_board();
  info.board.square_size_m = 0.1505;
  info.board.planarity_bow_m = 0.0005;
  info.square_measured = true;
  info.streams = small_streams();
  info.cameras = {
      make_camera("CL2A14100A4", 1, recon::sensor::SyncRole::Secondary),
      make_camera("CL2A141000N", 2, recon::sensor::SyncRole::Primary)};
  return info;
}

CameraCapture make_part(int seed) {
  CameraCapture part;
  // Not a decodable JPEG: the writer keeps bytes, whatever they are.
  part.jpeg = {0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x00, 0x0A};
  for (int i = 0; i < 200 + seed; ++i) {
    part.jpeg.push_back(static_cast<std::uint8_t>(i * 7 + seed));
  }
  part.jpeg.push_back(0xFF);
  part.jpeg.push_back(0xD9);
  part.depth.resize(std::size_t{kDepthW} * kDepthH);
  for (std::size_t i = 0; i < part.depth.size(); ++i) {
    part.depth[i] = static_cast<std::uint16_t>(i * 977 + seed * 13);
  }
  part.depth[0] = 0;  // no return
  part.depth[1] = 65535;
  part.timestamp_ns = 1760348472003911000u + static_cast<std::uint64_t>(seed);
  part.sequence = 1234 + static_cast<std::uint64_t>(seed);
  part.detection.image_size = {kColorW, kColorH};
  part.detection.observation.camera_id = "ignored";
  part.detection.observation.points = {
      {0, {12.4 + 0.1 + 0.2, 9.1 / 3.0}, 1.0},
      {1, {20.000000000000004, 9.300000000000001}, 0.75},
      {5, {1.0 / 7.0, 1e-300}, 1.0 / 3.0}};
  DetectedMarker marker;
  marker.id = 3;
  marker.corners = {{{10.1, 5.2},
                     {14.3, 5.200000000000001},
                     {14.0, 9.9},
                     {10.0, 9.7 + seed}}};
  part.detection.markers = {marker};
  part.check.corners = 3;
  part.check.markers = 1;
  part.check.collinear = false;
  part.check.distance_m = 2.8000000000000003;
  part.check.tilt_deg = 12.0;
  part.check.cell = 4;
  part.check.verdict = ViewVerdict::Partial;
  return part;
}

CaptureJob make_job(int seed = 0) {
  CaptureJob job;
  job.meta.index = 99;  // the writer's to set
  job.meta.time_ns = 1760348472004123456;
  job.meta.trigger_ns = std::numeric_limits<std::uint64_t>::max() - 1;
  job.meta.sequence = 1234;
  job.meta.spread_ms = 1.4000000000000001;
  job.meta.stillness_px = 0.08;
  job.cameras = {make_part(seed), make_part(seed + 1)};
  job.cameras[1].check.verdict = ViewVerdict::Collinear;
  job.cameras[1].check.collinear = true;
  return job;
}

std::unique_ptr<SessionWriter> make_writer(SessionInfo info,
                                           std::size_t capacity = 4) {
  vkc::Result<std::unique_ptr<SessionWriter>> writer =
      SessionWriter::create(std::move(info), {capacity});
  EXPECT_TRUE(writer.ok()) << writer.status().message();
  return writer.ok() ? std::move(writer).value() : nullptr;
}

// Submit `job`, which must be queued.
void submit(SessionWriter& writer, CaptureJob job) {
  vkc::Result<bool> queued = writer.submit(std::move(job));
  ASSERT_TRUE(queued.ok()) << queued.status().message();
  ASSERT_TRUE(*queued);
}

// Carry out every request, and take their outcomes.
std::vector<WriteResult> finish(SessionWriter& writer) {
  writer.flush();
  return writer.take_results();
}

void expect_written(const WriteResult& result, WriteResult::Kind kind,
                    int index) {
  EXPECT_EQ(result.kind, kind);
  EXPECT_TRUE(result.status.ok()) << result.status.message();
  EXPECT_EQ(result.index, index);
}

// session.json's captures, as (index, dropped).
std::vector<std::pair<int, bool>> listed(const SessionInfo& info) {
  std::vector<std::pair<int, bool>> captures;
  const json session = parse(info.directory + "/session.json");
  for (const json& c : session["captures"]) {
    const int index = c["index"].get<int>();
    captures.emplace_back(index, c["dropped"].get<bool>());
    char file[32];
    std::snprintf(file, sizeof file, "captures/%03d.json", index);
    EXPECT_EQ(c["file"], file);
  }
  return captures;
}

bool is_root() { return ::geteuid() == 0; }

// The writer of a FIFO at `path`: the writer's thread, reading the sync file
// at its first capture, waits on the FIFO until this opens and fills it.
class FifoGate {
 public:
  explicit FifoGate(std::string path) : path_(std::move(path)) {
    EXPECT_EQ(::mkfifo(path_.c_str(), 0600), 0) << std::strerror(errno);
  }
  ~FifoGate() { open_with("abc"); }
  FifoGate(const FifoGate&) = delete;
  FifoGate& operator=(const FifoGate&) = delete;

  // Let the reader through with `bytes`: waits up to 10 s for it to open.
  void open_with(const std::string& bytes) {
    if (opened_) return;
    opened_ = true;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(10);
    int fd = -1;
    while (fd < 0 && std::chrono::steady_clock::now() < deadline) {
      fd = ::open(path_.c_str(), O_WRONLY | O_NONBLOCK);  // ENXIO: no reader
      if (fd < 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_GE(fd, 0) << "no reader opened " << path_;
    EXPECT_EQ(::write(fd, bytes.data(), bytes.size()),
              static_cast<ssize_t>(bytes.size()));
    ::close(fd);
  }

 private:
  std::string path_;
  bool opened_ = false;
};

// --- The tree and its files -------------------------------------------------

TEST(SessionWriter, WritesNothingBeforeTheFirstCapture) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  {
    auto writer = make_writer(info);
  }
  EXPECT_FALSE(fs::exists(info.directory));
}

TEST(SessionWriter, WritesTheTreeWithZeroPaddedNames) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  submit(*writer, make_job(0));
  submit(*writer, make_job(1));
  const std::vector<WriteResult> results = finish(*writer);
  ASSERT_EQ(results.size(), 2u);
  expect_written(results[0], WriteResult::Kind::Capture, 0);
  expect_written(results[1], WriteResult::Kind::Capture, 1);

  const std::set<std::string> expected = {
      "session.json",
      "factory_calibration.json",
      "images/CL2A14100A4/calib_img_CL2A14100A4_000.jpg",
      "images/CL2A14100A4/calib_img_CL2A14100A4_001.jpg",
      "images/CL2A141000N/calib_img_CL2A141000N_000.jpg",
      "images/CL2A141000N/calib_img_CL2A141000N_001.jpg",
      "depth/CL2A14100A4/calib_depth_CL2A14100A4_000.png",
      "depth/CL2A14100A4/calib_depth_CL2A14100A4_001.png",
      "depth/CL2A141000N/calib_depth_CL2A141000N_000.png",
      "depth/CL2A141000N/calib_depth_CL2A141000N_001.png",
      "captures/000.json",
      "captures/001.json"};
  EXPECT_EQ(files_under(info.directory), expected);
  EXPECT_EQ(listed(info),
            (std::vector<std::pair<int, bool>>{{0, false}, {1, false}}));
}

TEST(SessionWriter, KeepsTheCamerasJpegBytes) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  const CaptureJob job = make_job(3);
  submit(*writer, job);
  finish(*writer);
  for (std::size_t i = 0; i < info.cameras.size(); ++i) {
    const std::string& serial = info.cameras[i].info.id;
    const std::string jpeg = read(info.directory + "/images/" + serial +
                                  "/calib_img_" + serial + "_000.jpg");
    EXPECT_EQ(jpeg, std::string(job.cameras[i].jpeg.begin(),
                                job.cameras[i].jpeg.end()));
  }
}

TEST(SessionWriter, WritesDepthAsSixteenBitPngReconReads) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  const CaptureJob job = make_job(5);
  submit(*writer, job);
  finish(*writer);
  for (std::size_t i = 0; i < info.cameras.size(); ++i) {
    const std::string& serial = info.cameras[i].info.id;
    vkc::Result<std::vector<std::uint16_t>> depth =
        recon::io::load_depth_u16(info.directory + "/depth/" + serial +
                                      "/calib_depth_" + serial + "_000.png",
                                  kDepthW, kDepthH);
    ASSERT_TRUE(depth.ok()) << depth.status().message();
    EXPECT_EQ(*depth, job.cameras[i].depth);
  }
}

TEST(SessionWriter, FactoryCalibrationReadsBackAsTheModels) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  submit(*writer, make_job());
  finish(*writer);

  vkc::Result<camera::ArrayCalibration> factory =
      camera::read_array_calibration(info.directory +
                                     "/factory_calibration.json");
  ASSERT_TRUE(factory.ok()) << factory.status().message();
  EXPECT_TRUE(factory->world.sensor.empty());
  ASSERT_EQ(factory->sensors.size(), info.cameras.size());
  for (const SessionCamera& c : info.cameras) {
    const camera::SensorCalibration* sensor =
        camera::find_sensor(*factory, c.info.id);
    ASSERT_NE(sensor, nullptr) << c.info.id;
    EXPECT_FALSE(sensor->color_to_world);  // unposed
    ASSERT_TRUE(sensor->color && sensor->depth && sensor->depth_to_color);
    for (const auto& [got, model] :
         {std::pair{&*sensor->color, &*c.info.color},
          std::pair{&*sensor->depth, &*c.info.depth}}) {
      EXPECT_EQ(got->source, camera::IntrinsicsSource::Factory);
      const camera::CameraModel& m = got->model;
      EXPECT_EQ(m.size.width, model->size.width);
      EXPECT_EQ(m.size.height, model->size.height);
      EXPECT_EQ(m.intrinsics.fx, model->intrinsics.fx);
      EXPECT_EQ(m.intrinsics.fy, model->intrinsics.fy);
      EXPECT_EQ(m.intrinsics.cx, model->intrinsics.cx);
      EXPECT_EQ(m.intrinsics.cy, model->intrinsics.cy);
      const camera::RationalDistortion& d = m.distortion;
      const camera::RationalDistortion& e = model->distortion;
      EXPECT_EQ(d.k1, e.k1);
      EXPECT_EQ(d.k2, e.k2);
      EXPECT_EQ(d.p1, e.p1);
      EXPECT_EQ(d.p2, e.p2);
      EXPECT_EQ(d.k3, e.k3);
      EXPECT_EQ(d.k4, e.k4);
      EXPECT_EQ(d.k5, e.k5);
      EXPECT_EQ(d.k6, e.k6);
    }
    // To the Rodrigues conversion's round-off (array_calibration.hpp).
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        EXPECT_NEAR((*sensor->depth_to_color)[col][row],
                    c.info.depth_to_color[col][row], 1e-12);
      }
    }
  }
}

// --- The JSON ---------------------------------------------------------------

TEST(SessionWriter, SessionJsonHoldsWhatWasGivenExactly) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  submit(*writer, make_job());
  finish(*writer);

  const std::string text = read(info.directory + "/session.json");
  const json doc = json::parse(text);
  EXPECT_EQ(doc["format"], "volumetric_kit_calib.session");
  EXPECT_EQ(doc["version"], 1);
  EXPECT_EQ(doc["software"],
            json::parse(R"({"calib": "calib-sha", "recon": "recon-sha",
                            "gfx": "gfx-sha", "core": "core-sha",
                            "opencv": "5.0.0", "orbbec_sdk": "2.10.6"})"));
  EXPECT_EQ(doc["rig"]["sync_file"], info.sync_path);
  // SHA-256 of "abc" (FIPS 180-2, appendix B.1).
  EXPECT_EQ(doc["rig"]["sync_sha256"],
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(doc["rig"]["reference"], "CL2A141000N");

  const json& board = doc["board"];
  EXPECT_EQ(board["type"], "charuco");
  EXPECT_EQ(board["squares"], json::array({5, 3}));
  EXPECT_EQ(board["square_m"].get<double>(), 0.1505);
  EXPECT_EQ(board["marker_m"].get<double>(), 0.112);
  EXPECT_EQ(board["dictionary"], "DICT_4X4_100");
  EXPECT_EQ(board["first_marker_id"], 0);
  EXPECT_EQ(board["legacy_pattern"], true);
  EXPECT_EQ(board["square_measured"], true);
  EXPECT_TRUE(board["square_tolerance_m"].is_null());
  EXPECT_EQ(board["planarity_bow_m"].get<double>(), 0.0005);

  const json& streams = doc["streams"];
  EXPECT_EQ(streams["color"],
            json::parse(R"({"width": 64, "height": 48, "codec": "mjpeg"})"));
  const json& depth = streams["depth"];
  EXPECT_EQ(depth["width"], kDepthW);
  EXPECT_EQ(depth["height"], kDepthH);
  EXPECT_EQ(depth["mode"], "nfov_unbinned");
  // A float as the shortest decimal that reads back to it.
  EXPECT_EQ(static_cast<float>(depth["metres_per_unit"].get<double>()), 0.001f);
  EXPECT_EQ(static_cast<float>(depth["min_m"].get<double>()), 0.5f);
  EXPECT_EQ(static_cast<float>(depth["max_m"].get<double>()), 3.86f);
  EXPECT_NE(text.find("\"metres_per_unit\": 0.001,"), std::string::npos);
  EXPECT_NE(text.find("\"max_m\": 3.86"), std::string::npos);
  EXPECT_EQ(streams["fps"], 25);

  ASSERT_EQ(doc["cameras"].size(), 2u);
  EXPECT_EQ(doc["cameras"][0], json::parse(R"({
      "serial": "CL2A14100A4", "model": "Orbbec Femto Mega",
      "firmware": "1.2.9", "ip": "192.168.1.11", "role": "secondary",
      "sync_mode": "secondary-synced", "depth_delay_us": 160,
      "color_lens_model": "brown-conrady",
      "depth_lens_model": "brown-conrady-k6"})"));
  EXPECT_EQ(doc["cameras"][1]["serial"], "CL2A141000N");
  EXPECT_EQ(doc["cameras"][1]["role"], "primary");
  EXPECT_EQ(doc["factory_calibration"], "factory_calibration.json");

  // ISO 8601, UTC, to the millisecond.
  const std::string created = doc["created"];
  ASSERT_EQ(created.size(), 24u) << created;
  EXPECT_EQ(created[10], 'T');
  EXPECT_EQ(created.back(), 'Z');
}

TEST(SessionWriter, CaptureJsonHoldsWhatWasGivenExactly) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  CaptureJob job = make_job(2);
  job.cameras[0].check.pnp_rms_px = 0.21000000000000002;
  job.meta.live_vs_full_px = 0.12;
  submit(*writer, job);
  finish(*writer);

  const json doc = parse(info.directory + "/captures/000.json");
  EXPECT_EQ(doc["format"], "volumetric_kit_calib.capture");
  EXPECT_EQ(doc["version"], 1);
  EXPECT_EQ(doc["index"], 0);  // the writer's, not the job's 99
  EXPECT_EQ(doc["time"], "2025-10-13T09:41:12.004Z");
  EXPECT_EQ(doc["trigger_ns"].get<std::uint64_t>(), job.meta.trigger_ns);
  EXPECT_EQ(doc["sequence"].get<std::uint64_t>(), job.meta.sequence);
  EXPECT_EQ(doc["spread_ms"].get<double>(), job.meta.spread_ms);
  EXPECT_EQ(doc["stillness_px"].get<double>(), job.meta.stillness_px);
  EXPECT_EQ(doc["live_vs_full_px"].get<double>(), 0.12);
  // A Partial view counts, a Collinear one does not.
  EXPECT_EQ(doc["usable_views"], 1);

  ASSERT_EQ(doc["cameras"].size(), 2u);
  for (std::size_t i = 0; i < info.cameras.size(); ++i) {
    const std::string& serial = info.cameras[i].info.id;
    const CameraCapture& part = job.cameras[i];
    const json& c = doc["cameras"][serial];
    EXPECT_EQ(c["color"],
              "images/" + serial + "/calib_img_" + serial + "_000.jpg");
    EXPECT_EQ(c["depth"],
              "depth/" + serial + "/calib_depth_" + serial + "_000.png");
    EXPECT_EQ(c["color_bytes"], part.jpeg.size());
    EXPECT_EQ(c["timestamp_ns"].get<std::uint64_t>(), part.timestamp_ns);
    EXPECT_EQ(c["sequence"].get<std::uint64_t>(), part.sequence);

    const json& d = c["detection"];
    EXPECT_EQ(d["method"], "full");
    EXPECT_EQ(d["image_size"], json::array({kColorW, kColorH}));
    EXPECT_EQ(d["pixel_convention"], "opencv");
    const std::vector<DetectedPoint>& points =
        part.detection.observation.points;
    ASSERT_EQ(d["ids"].size(), points.size());
    for (std::size_t p = 0; p < points.size(); ++p) {
      EXPECT_EQ(d["ids"][p], points[p].id);
      // Every double back exactly.
      EXPECT_EQ(d["corners_px"][p][0].get<double>(), points[p].pixel.x);
      EXPECT_EQ(d["corners_px"][p][1].get<double>(), points[p].pixel.y);
      EXPECT_EQ(d["weights"][p].get<double>(), points[p].weight);
    }
    ASSERT_EQ(d["marker_ids"], json::array({3}));
    for (std::size_t k = 0; k < 4; ++k) {
      const camera::Vec2d& corner = part.detection.markers[0].corners[k];
      EXPECT_EQ(d["marker_corners_px"][0][k][0].get<double>(), corner.x);
      EXPECT_EQ(d["marker_corners_px"][0][k][1].get<double>(), corner.y);
    }

    const json& check = c["check"];
    EXPECT_EQ(check["corners"], part.check.corners);
    EXPECT_EQ(check["markers"], part.check.markers);
    EXPECT_EQ(check["collinear"], part.check.collinear);
    EXPECT_EQ(check["distance_m"].get<double>(), *part.check.distance_m);
    EXPECT_EQ(check["tilt_deg"].get<double>(), *part.check.tilt_deg);
    EXPECT_EQ(check["cell"], *part.check.cell);
    EXPECT_EQ(check["verdict"], to_string(part.check.verdict));
  }
  EXPECT_EQ(doc["cameras"]["CL2A14100A4"]["check"]["pnp_rms_px"].get<double>(),
            0.21000000000000002);
  EXPECT_TRUE(doc["cameras"]["CL2A141000N"]["check"]["pnp_rms_px"].is_null());
}

TEST(SessionWriter, TimesBeforeTheEpochRoundDown) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  CaptureJob job = make_job();
  job.meta.time_ns = -1;
  job.meta.live_vs_full_px.reset();
  submit(*writer, std::move(job));
  finish(*writer);
  const json doc = parse(info.directory + "/captures/000.json");
  EXPECT_EQ(doc["time"], "1969-12-31T23:59:59.999Z");
  EXPECT_TRUE(doc["live_vs_full_px"].is_null());
}

// FIPS 180-2's vectors, and lengths about the 56- and 64-byte block edges,
// through what session.json records of the sync file.
TEST(SessionWriter, RecordsTheSyncFilesSha256) {
  const std::vector<std::pair<std::string, std::string>> vectors = {
      {"", "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"},
      {"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
       "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"},
      {std::string(55, 'a'),
       "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318"},
      {std::string(56, 'a'),
       "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a"},
      {std::string(64, 'a'),
       "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb"},
      {std::string(119, 'a'),
       "31eba51c313a5c08226adf18d4a359cfdfd8d2e816b13f4af952f7ea6584dcfb"},
      {std::string(1000000, 'a'),
       "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0"}};
  for (const auto& [bytes, digest] : vectors) {
    const ScratchDir dir;
    const SessionInfo info = make_info(dir);
    write(info.sync_path, bytes);
    auto writer = make_writer(info);
    ASSERT_TRUE(writer);
    submit(*writer, make_job());
    finish(*writer);
    EXPECT_EQ(parse(info.directory + "/session.json")["rig"]["sync_sha256"],
              digest)
        << bytes.size() << " bytes";
  }
}

// --- Never overwrite --------------------------------------------------------

TEST(SessionWriter, RefusesAnExistingSessionDirectory) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  ASSERT_TRUE(fs::create_directory(info.directory));
  write(info.directory + "/note.txt", "an earlier run's");
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  submit(*writer, make_job());
  submit(*writer, make_job());
  const std::vector<WriteResult> results = finish(*writer);
  ASSERT_EQ(results.size(), 2u);
  for (const WriteResult& result : results) {
    EXPECT_EQ(result.status.domain(), Code::InvalidArgument)
        << result.status.message();
    EXPECT_EQ(result.index, -1);
  }
  EXPECT_EQ(
      snapshot(info.directory),
      (std::map<std::string, std::string>{{"note.txt", "an earlier run's"}}));
}

TEST(SessionWriter, RefusesAnExistingFileAndNeverReusesItsIndex) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  submit(*writer, make_job());
  finish(*writer);
  // The second camera's colour image of capture 1, there before it.
  const std::string planted =
      info.directory + "/images/CL2A141000N/calib_img_CL2A141000N_001.jpg";
  write(planted, "planted");

  submit(*writer, make_job());
  submit(*writer, make_job());
  const std::vector<WriteResult> results = finish(*writer);
  ASSERT_EQ(results.size(), 2u);
  EXPECT_EQ(results[0].status.domain(), Code::InvalidArgument)
      << results[0].status.message();
  EXPECT_NE(results[0].status.message().find(planted), std::string::npos)
      << results[0].status.message();
  expect_written(results[1], WriteResult::Kind::Capture, 2);
  EXPECT_EQ(read(planted), "planted");
  EXPECT_FALSE(fs::exists(info.directory + "/captures/001.json"));
  EXPECT_EQ(listed(info),
            (std::vector<std::pair<int, bool>>{{0, false}, {2, false}}));
}

// --- Failures ---------------------------------------------------------------

TEST(SessionWriter, AMissingParentOrSyncFileIsNotFound) {
  {
    const ScratchDir dir;
    SessionInfo info = make_info(dir);
    info.directory = dir / "no/such/parent";
    auto writer = make_writer(info);
    ASSERT_TRUE(writer);
    submit(*writer, make_job());
    const std::vector<WriteResult> results = finish(*writer);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].status.domain(), Code::NotFound);
  }
  {
    const ScratchDir dir;
    SessionInfo info = make_info(dir);
    info.sync_path = dir / "missing.json";
    auto writer = make_writer(info);
    ASSERT_TRUE(writer);
    submit(*writer, make_job());
    const std::vector<WriteResult> results = finish(*writer);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].status.domain(), Code::NotFound);
    EXPECT_FALSE(fs::exists(info.directory));
  }
}

TEST(SessionWriter, AFailedWriteIsAStatusAndTheSessionGoesOn) {
  if (is_root()) GTEST_SKIP() << "root writes into read-only directories";
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  submit(*writer, make_job());
  finish(*writer);

  // A camera's image directory refuses the write.
  const std::string images = info.directory + "/images/CL2A14100A4";
  fs::permissions(images, fs::perms::owner_write, fs::perm_options::remove);
  submit(*writer, make_job());
  std::vector<WriteResult> results = finish(*writer);
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].status.domain(), Code::IoError)
      << results[0].status.message();
  EXPECT_EQ(results[0].index, -1);
  fs::permissions(images, fs::perms::owner_write, fs::perm_options::add);

  // The session's directory refuses session.json, written last: the
  // capture's own files are there, but it is not listed.
  fs::permissions(info.directory, fs::perms::owner_write,
                  fs::perm_options::remove);
  submit(*writer, make_job());
  results = finish(*writer);
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].status.domain(), Code::IoError)
      << results[0].status.message();
  fs::permissions(info.directory, fs::perms::owner_write,
                  fs::perm_options::add);
  EXPECT_TRUE(fs::exists(info.directory + "/captures/002.json"));
  EXPECT_EQ(listed(info), (std::vector<std::pair<int, bool>>{{0, false}}));

  submit(*writer, make_job());
  results = finish(*writer);
  ASSERT_EQ(results.size(), 1u);
  expect_written(results[0], WriteResult::Kind::Capture, 3);
  EXPECT_EQ(listed(info),
            (std::vector<std::pair<int, bool>>{{0, false}, {3, false}}));
}

TEST(SessionWriter, ASessionThatCannotStartTriesAgainAtTheNextCapture) {
  if (is_root()) GTEST_SKIP() << "root writes into read-only directories";
  const ScratchDir dir;
  SessionInfo info = make_info(dir);
  // The parent refuses the session's directory, then allows it.
  const std::string parent = dir / "out";
  ASSERT_TRUE(fs::create_directory(parent));
  info.directory = parent + "/session";
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  fs::permissions(parent, fs::perms::owner_write, fs::perm_options::remove);
  submit(*writer, make_job());
  std::vector<WriteResult> results = finish(*writer);
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].status.domain(), Code::IoError)
      << results[0].status.message();
  fs::permissions(parent, fs::perms::owner_write, fs::perm_options::add);

  submit(*writer, make_job());
  results = finish(*writer);
  ASSERT_EQ(results.size(), 1u);
  // Nothing of the failed capture was written, so its index was not used.
  expect_written(results[0], WriteResult::Kind::Capture, 0);
}

// --- Drops ------------------------------------------------------------------

TEST(SessionWriter, ADropRewritesOnlySessionJson) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  submit(*writer, make_job(0));
  submit(*writer, make_job(1));
  finish(*writer);
  std::map<std::string, std::string> before = snapshot(info.directory);

  ASSERT_TRUE(writer->drop_last().ok());
  std::vector<WriteResult> results = finish(*writer);
  ASSERT_EQ(results.size(), 1u);
  expect_written(results[0], WriteResult::Kind::Drop, 1);
  std::map<std::string, std::string> after = snapshot(info.directory);
  EXPECT_NE(after["session.json"], before["session.json"]);
  before.erase("session.json");
  after.erase("session.json");
  EXPECT_EQ(after, before);  // the dropped capture's files kept, unchanged
  EXPECT_EQ(listed(info),
            (std::vector<std::pair<int, bool>>{{0, false}, {1, true}}));

  // Each drop takes the latest left; then there is none. A capture after
  // them takes a new index.
  ASSERT_TRUE(writer->drop_last().ok());
  ASSERT_TRUE(writer->drop_last().ok());
  submit(*writer, make_job());
  results = finish(*writer);
  ASSERT_EQ(results.size(), 3u);
  expect_written(results[0], WriteResult::Kind::Drop, 0);
  EXPECT_EQ(results[1].kind, WriteResult::Kind::Drop);
  EXPECT_EQ(results[1].status.domain(), Code::NotFound);
  EXPECT_EQ(results[1].index, -1);
  expect_written(results[2], WriteResult::Kind::Capture, 2);
  EXPECT_EQ(listed(info), (std::vector<std::pair<int, bool>>{
                              {0, true}, {1, true}, {2, false}}));
}

TEST(SessionWriter, ADropBeforeAnyCaptureIsNotFound) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  ASSERT_TRUE(writer->drop_last().ok());
  const std::vector<WriteResult> results = finish(*writer);
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].status.domain(), Code::NotFound);
  EXPECT_FALSE(fs::exists(info.directory));
}

TEST(SessionWriter, ADropThatCannotBeWrittenKeepsTheCapture) {
  if (is_root()) GTEST_SKIP() << "root writes into read-only directories";
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  submit(*writer, make_job());
  finish(*writer);

  fs::permissions(info.directory, fs::perms::owner_write,
                  fs::perm_options::remove);
  ASSERT_TRUE(writer->drop_last().ok());
  std::vector<WriteResult> results = finish(*writer);
  fs::permissions(info.directory, fs::perms::owner_write,
                  fs::perm_options::add);
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].status.domain(), Code::IoError);
  EXPECT_EQ(listed(info), (std::vector<std::pair<int, bool>>{{0, false}}));

  ASSERT_TRUE(writer->drop_last().ok());
  results = finish(*writer);
  ASSERT_EQ(results.size(), 1u);
  expect_written(results[0], WriteResult::Kind::Drop, 0);
}

// --- The queue and the thread -----------------------------------------------

TEST(SessionWriter, RefusesACaptureWhenTheQueueIsFull) {
  const ScratchDir dir;
  SessionInfo info = make_info(dir);
  info.sync_path = dir / "sync.fifo";
  FifoGate gate(info.sync_path);
  auto writer = make_writer(info, /*capacity=*/2);
  ASSERT_TRUE(writer);

  // The first capture holds the thread at the sync file.
  submit(*writer, make_job(0));
  submit(*writer, make_job(1));
  EXPECT_EQ(writer->pending(), 2u);
  vkc::Result<bool> refused = writer->submit(make_job(2));
  ASSERT_TRUE(refused.ok()) << refused.status().message();
  EXPECT_FALSE(*refused);
  // A drop is not a capture: the queue takes it.
  EXPECT_TRUE(writer->drop_last().ok());
  EXPECT_EQ(writer->pending(), 2u);

  gate.open_with("rig");
  const std::vector<WriteResult> results = finish(*writer);
  EXPECT_EQ(writer->pending(), 0u);
  ASSERT_EQ(results.size(), 3u);
  expect_written(results[0], WriteResult::Kind::Capture, 0);
  expect_written(results[1], WriteResult::Kind::Capture, 1);
  expect_written(results[2], WriteResult::Kind::Drop, 1);
  EXPECT_EQ(listed(info),
            (std::vector<std::pair<int, bool>>{{0, false}, {1, true}}));
  EXPECT_FALSE(fs::exists(info.directory + "/captures/002.json"));
}

TEST(SessionWriter, DrainsTheQueueAtShutdown) {
  const ScratchDir dir;
  SessionInfo info = make_info(dir);
  info.sync_path = dir / "sync.fifo";
  FifoGate gate(info.sync_path);
  {
    auto writer = make_writer(info);
    ASSERT_TRUE(writer);
    for (int i = 0; i < 3; ++i) submit(*writer, make_job(i));
    ASSERT_TRUE(writer->drop_last().ok());
    // Let the thread go only once the destructor is waiting on it.
    std::thread late([&gate] {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      gate.open_with("rig");
    });
    writer.reset();
    late.join();
  }
  EXPECT_EQ(listed(info), (std::vector<std::pair<int, bool>>{
                              {0, false}, {1, false}, {2, true}}));
}

TEST(SessionWriter, TakesRequestsFromSeveralThreads) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info, /*capacity=*/64);
  ASSERT_TRUE(writer);
  constexpr int kEach = 6;
  std::vector<std::thread> threads;
  for (int t = 0; t < 2; ++t) {
    threads.emplace_back([&writer, t] {
      for (int i = 0; i < kEach; ++i) submit(*writer, make_job(10 * t + i));
    });
  }
  std::vector<WriteResult> results;
  for (int polls = 0; polls < 20; ++polls) {
    std::vector<WriteResult> taken = writer->take_results();
    results.insert(results.end(), taken.begin(), taken.end());
    (void)writer->pending();
  }
  for (std::thread& thread : threads) thread.join();
  const std::vector<WriteResult> rest = finish(*writer);
  results.insert(results.end(), rest.begin(), rest.end());

  ASSERT_EQ(results.size(), 2u * kEach);
  std::set<int> indices;
  for (const WriteResult& result : results) {
    EXPECT_TRUE(result.status.ok()) << result.status.message();
    indices.insert(result.index);
  }
  EXPECT_EQ(indices.size(), 2u * kEach);
  EXPECT_EQ(*indices.begin(), 0);
  EXPECT_EQ(*indices.rbegin(), 2 * kEach - 1);
  EXPECT_EQ(listed(info).size(), 2u * kEach);
}

// --- What is refused up front -----------------------------------------------

TEST(SessionWriter, RefusesASessionItCannotWrite) {
  const ScratchDir dir;
  const auto refused = [](SessionInfo info, Code code, const char* why) {
    vkc::Result<std::unique_ptr<SessionWriter>> writer =
        SessionWriter::create(std::move(info));
    EXPECT_FALSE(writer.ok()) << why;
    if (writer.ok()) return;
    EXPECT_EQ(writer.status().domain(), code)
        << why << ": " << writer.status().message();
    EXPECT_NE(writer.status().message().find(why), std::string::npos)
        << why << ": " << writer.status().message();
  };
  const SessionInfo good = make_info(dir);
  {
    SessionInfo info = good;
    info.directory.clear();
    refused(info, Code::InvalidArgument, "directory");
  }
  {
    SessionInfo info = good;
    info.sync_path.clear();
    refused(info, Code::InvalidArgument, "sync_path");
  }
  {
    SessionInfo info = good;
    info.board.rows = 0;
    refused(info, Code::InvalidArgument, "board");
  }
  {
    SessionInfo info = good;
    info.board = Target{};
    info.board.kind = TargetKind::Checkerboard;
    info.board.rows = 6;
    info.board.cols = 9;
    info.board.square_size_m = 0.025;
    refused(info, Code::Unsupported, "checkerboard");
  }
  {
    SessionInfo info = good;
    info.streams.depth_width = 0;
    refused(info, Code::InvalidArgument, "sizes");
  }
  {
    SessionInfo info = good;
    info.streams.fps = 0;
    refused(info, Code::InvalidArgument, "fps");
  }
  {
    SessionInfo info = good;
    info.streams.metres_per_unit = std::numeric_limits<float>::infinity();
    refused(info, Code::InvalidArgument, "metres_per_unit");
  }
  {
    SessionInfo info = good;
    info.streams.min_depth_m = 4.0f;
    refused(info, Code::InvalidArgument, "min_depth_m");
  }
  {
    SessionInfo info = good;
    info.cameras.clear();
    refused(info, Code::InvalidArgument, "a camera");
  }
  {
    SessionInfo info = good;
    info.cameras[0].info.id = "../escape";
    refused(info, Code::InvalidArgument, "plain file name");
  }
  {
    SessionInfo info = good;
    info.cameras[0].info.id = "..";
    refused(info, Code::InvalidArgument, "plain file name");
  }
  {
    SessionInfo info = good;
    info.cameras[0].info.id = info.cameras[1].info.id;
    refused(info, Code::InvalidArgument, "repeated");
  }
  {
    SessionInfo info = good;
    info.cameras[1].info.depth.reset();
    refused(info, Code::InvalidArgument, "no depth model");
  }
  {
    SessionInfo info = good;
    info.cameras[1].info.color->size.width += 1;
    refused(info, Code::InvalidArgument, "colour model is 65x48");
  }
  {
    SessionInfo info = good;
    info.cameras[0].info.color->intrinsics.fx = kNaN;
    refused(info, Code::InvalidArgument, "factory calibration");
  }
  {
    SessionInfo info = good;
    info.cameras[0].info.depth_to_color[0][0] = 2.0;  // not a rotation
    refused(info, Code::InvalidArgument, "factory calibration");
  }
  {
    SessionInfo info = good;
    info.reference = "CL2A1410000";
    refused(info, Code::InvalidArgument, "reference");
  }
  {
    vkc::Result<std::unique_ptr<SessionWriter>> writer =
        SessionWriter::create(good, {0});
    ASSERT_FALSE(writer.ok());
    EXPECT_EQ(writer.status().domain(), Code::InvalidArgument);
  }
}

TEST(SessionWriter, RefusesACaptureThatDoesNotFitTheSession) {
  const ScratchDir dir;
  const SessionInfo info = make_info(dir);
  auto writer = make_writer(info);
  ASSERT_TRUE(writer);
  const auto refused = [&writer](CaptureJob job, const char* why) {
    vkc::Result<bool> queued = writer->submit(std::move(job));
    EXPECT_FALSE(queued.ok()) << why;
    if (queued.ok()) return;
    EXPECT_EQ(queued.status().domain(), Code::InvalidArgument) << why;
    EXPECT_NE(queued.status().message().find(why), std::string::npos)
        << why << ": " << queued.status().message();
  };
  {
    CaptureJob job = make_job();
    job.cameras.pop_back();
    refused(std::move(job), "1 parts for 2 cameras");
  }
  {
    CaptureJob job = make_job();
    job.cameras[1].jpeg.clear();
    refused(std::move(job), "camera CL2A141000N: no JPEG");
  }
  {
    CaptureJob job = make_job();
    job.cameras[0].depth.pop_back();
    refused(std::move(job), "191 depth samples for a 16x12 depth image");
  }
  {
    CaptureJob job = make_job();
    job.cameras[0].detection.image_size = {kColorW / 2, kColorH / 2};
    refused(std::move(job), "a detection in a 32x24 image");
  }
  {
    CaptureJob job = make_job();
    job.cameras[0].detection.observation.points[1].pixel.y = kNaN;
    refused(std::move(job), "corner 1 is not finite");
  }
  {
    CaptureJob job = make_job();
    job.cameras[0].detection.observation.points[2].weight = kNaN;
    refused(std::move(job), "corner 5 is not finite");
  }
  {
    CaptureJob job = make_job();
    job.cameras[1].detection.markers[0].corners[2].x = kNaN;
    refused(std::move(job), "marker 3 is not finite");
  }
  {
    CaptureJob job = make_job();
    job.cameras[0].check.tilt_deg = kNaN;
    refused(std::move(job), "a check's measure is not finite");
  }
  {
    CaptureJob job = make_job();
    job.cameras[0].check.verdict = static_cast<ViewVerdict>(99);
    refused(std::move(job), "not a ViewVerdict");
  }
  {
    CaptureJob job = make_job();
    job.meta.live_vs_full_px = kNaN;
    refused(std::move(job), "a set's measure is not finite");
  }
  {
    CaptureJob job = make_job();
    job.meta.spread_ms = kNaN;
    refused(std::move(job), "a set's measure is not finite");
  }
  EXPECT_EQ(writer->pending(), 0u);
  EXPECT_TRUE(finish(*writer).empty());
  EXPECT_FALSE(fs::exists(info.directory));
}

// --- The directory's name ---------------------------------------------------

TEST(SessionStamp, NamesTheLocalDateAndTime) {
  const char* old = std::getenv("TZ");
  const std::string saved = old != nullptr ? old : "";
  ::setenv("TZ", "UTC", 1);
  ::tzset();
  const auto time = std::chrono::system_clock::time_point(
      std::chrono::seconds(1760348467) + std::chrono::milliseconds(999));
  vkc::Result<std::string> stamp = session_stamp(time);
  ::setenv("TZ", "UTC-2", 1);  // POSIX: two hours east of UTC
  ::tzset();
  vkc::Result<std::string> east = session_stamp(time);
  if (old != nullptr) {
    ::setenv("TZ", saved.c_str(), 1);
  } else {
    ::unsetenv("TZ");
  }
  ::tzset();
  ASSERT_TRUE(stamp.ok()) << stamp.status().message();
  EXPECT_EQ(*stamp, "2025-10-13_094107");
  ASSERT_TRUE(east.ok()) << east.status().message();
  EXPECT_EQ(*east, "2025-10-13_114107");
}

}  // namespace
}  // namespace volumetric_kit::calib::capture
