// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's depth checks at Calibrate, on a device: the kept captures' raw
// depth, rendered here through a Femto-like depth lens from each camera's true
// pose -- a floor, a box and each capture's board -- with exact detections,
// for 2 to 5 cameras whose ids sort in another order than the rig's, one
// camera's depth turned off its colour camera's axes. An aligned rig passes
// every camera and pair; a board printed 0.3 % large, a board's depth 5 mm
// deep, a camera's depth 3 mm far and depth in half-millimetre units each
// read as they should; a camera without depth is insufficient; each
// capture's frames are let go before the next; a stop between captures stops
// the run, and a cancel writes nothing; a failure of the checks is posted and
// the results written without them; and the checks give the same while the 3D
// view's worker prepares on the same device. Under synchronization validation;
// skips without a Vulkan device, and the last case also with one queue family.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "calibrate_worker_support.hpp"
#include "depth_scene.hpp"
#include "kept_scene.hpp"
#include "result_checks.hpp"
#include "scratch_dir.hpp"
#include "synthetic_scene.hpp"
#include "volumetric_kit/calib/app/calib_rig.hpp"
#include "volumetric_kit/calib/app/calibrate_worker.hpp"
#include "volumetric_kit/calib/app/cloud_worker.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/gfx/core/device_requirements.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace camera = recon::camera;
namespace fs = std::filesystem;
namespace sensor = recon::sensor;
using camera::Mat3d;
using camera::Mat4d;
using camera::Vec3d;
using json = nlohmann::json;
using quality::DepthSummary;
using quality::DepthVerdict;
using quality::PairCheck;
using test::entries;
using test::ScratchDir;
using test::value_of;

// The floor, 5 cm below a board laid on it, so a board's depth read deeper
// than its face stays in front of it; and a box on it, off the boards.
constexpr double kFloorZ = -0.05;
const Vec3d kBoxLo(1.05, -1.15, kFloorZ);
const Vec3d kBoxHi(1.35, -0.85, 0.35);
// The printed board's white margin around its chessboard (metres).
constexpr double kMarginM = 0.03;

// How far the figures stray on exact detections. The preparation gives each
// pinhole pixel the depth of the raw pixel nearest where the lens maps it, so
// an oblique board's points sit up to half a raw pixel off their rays, about
// 1 mm RMS: each view's plane turns by up to 0.4 degrees, and a camera's
// median angle and mean ratio, over 14 to 16 views, stray by these; the
// board's ratio, pooled over every camera, half as far.
constexpr double kCameraRatio = 1e-3;
constexpr double kPooledRatio = 5e-4;
constexpr double kAngleDeg = 0.2;

// What a depth camera beside its colour camera turned 5.5 degrees down sees
// through, to show the checks take each camera's own transform.
Mat4d turned_depth_to_color() {
  Mat4d m(test::rotation({1.0, -0.03, 0.02}, -5.5));
  m[3] = glm::dvec4(0.0318, -0.0019, 0.0042, 1.0);
  return m;
}

// The rig's depth lens's rays, row-major (test::lens_rays), made once.
const std::vector<Vec3d>& depth_rays() {
  static const std::vector<Vec3d> rays =
      test::lens_rays(test::depth_like_lens());
  return rays;
}

// What every depth camera sees in one capture: the floor, the box, and the
// board as printed, `scale` times the target, its face read `depth_mm` deep.
test::Surface capture_surface(const compute::Board& board,
                              const Mat4d& board_to_world, double scale,
                              double depth_mm) {
  const Mat4d world_to_board = camera::rigid_inverse(board_to_world);
  const cv::Point3d c = board.centre();
  const double lo = -kMarginM * scale;
  const double hi_x = (2.0 * c.x + kMarginM) * scale;
  const double hi_y = (2.0 * c.y + kMarginM) * scale;
  const test::Plane floor{{0.0, 0.0, kFloorZ}, {0.0, 0.0, 1.0}};
  const test::Plane face{{0.0, 0.0, depth_mm / 1000.0}, {0.0, 0.0, 1.0}};
  return [=](const Vec3d& origin, const Vec3d& direction) {
    double nearest = 0.0;
    const auto take = [&](double s) {
      if (s > 0.0 && (nearest == 0.0 || s < nearest)) nearest = s;
    };
    take(test::hit_plane(floor, origin, direction));
    take(test::hit_box(kBoxLo, kBoxHi, origin, direction));
    const Vec3d from = Vec3d(world_to_board * glm::dvec4(origin, 1.0));
    const Vec3d along = Mat3d(world_to_board) * direction;
    const double s = test::hit_plane(face, from, along);
    const Vec3d at = from + s * along;
    if (s > 0.0 && at.x >= lo && at.x <= hi_x && at.y >= lo && at.y <= hi_y) {
      take(s);
    }
    return nearest;
  };
}

// A scene whose kept captures carry each camera's raw depth.
struct DepthCase {
  std::size_t cameras = 3;
  std::size_t reference = 0;
  std::size_t placements = 16;
  double board_scale = 1.0;     // the print, over the target
  double board_depth_mm = 0.0;  // how deep the depth reads the board's face
  std::optional<std::size_t> biased;  // reads every depth 3 mm far
  std::optional<std::size_t> without_depth;
  float metres_per_unit = 0.001f;
};

// The camera whose depth is turned off its colour camera's axes: not the
// reference.
std::size_t turned_camera(const DepthCase& c) {
  return (c.reference + 1) % c.cameras;
}

// `c`'s kept captures, from exact detections: each camera's raw depth as it
// sees the capture's board, in the depth mode's units, from its true pose.
KeptCaptures depth_captures(const DepthCase& c, const std::string& directory) {
  test::Scene scene = test::make_kept_scene(c.cameras, c.placements, 0.0);
  if (c.board_scale != 1.0) {
    Target printed = legacy_femto_board();
    printed.square_size_m *= c.board_scale;
    printed.marker_size_m *= c.board_scale;
    scene.views =
        test::observe(scene.rig, value_of(compute::Board::create(printed)),
                      scene.plan, test::ObserveOptions{0.0, 1});
  }
  KeptCaptures kept = test::kept_captures(scene, c.reference, directory);
  kept.depth_mode = {"test", c.metres_per_unit, 0.25f, 6.0f};
  kept.sensors[turned_camera(c)].depth_to_color = turned_depth_to_color();
  for (std::size_t p = 0; p < kept.captures.size(); ++p) {
    auto capture = std::make_shared<KeptCapture>(*kept.captures[p]);
    const test::Surface surface =
        capture_surface(scene.board, scene.plan[p].board_to_world,
                        c.board_scale, c.board_depth_mm);
    for (std::size_t k = 0; k < c.cameras; ++k) {
      if (c.without_depth == k) continue;
      const Mat4d depth_to_world =
          camera::rigid_inverse(scene.rig[k].world_to_camera) *
          kept.sensors[k].depth_to_color;
      capture->depth[k] = std::make_shared<const std::vector<std::uint16_t>>(
          test::render_raw(depth_rays(), depth_to_world, surface,
                           c.metres_per_unit, c.biased == k ? 0.003 : 0.0));
    }
    kept.captures[p] = std::move(capture);
  }
  return kept;
}

// This allocator's live device memory, every heap's.
std::uint64_t live_bytes(const core::Allocator& allocator) {
  const core::MemoryStats stats = allocator.memory_stats();
  std::uint64_t bytes = 0;
  for (std::uint32_t h = 0; h < stats.heap_count; ++h) {
    bytes += stats.heaps[h].allocation_bytes;
  }
  return bytes;
}

class CalibrateDepthTest : public core::test::VulkanDeviceTest {
 protected:
  // calib_rig's compute device.
  core::DeviceRequirements requirements() const override {
    return value_of(rig_requirements(false)).compute;
  }
  // The kernels read the prepared depth through descriptors, which
  // synchronization validation tracks only at this level.
  core::test::Validation validation() const override {
    return core::test::Validation::ShaderAccesses;
  }

  std::unique_ptr<CalibrateDepth> checks(std::size_t cameras) {
    return value_of(CalibrateDepth::create(device(), allocator(), cameras));
  }

  // `kept` calibrated with its depth checked.
  CalibrateResult calibrated(const KeptCaptures& kept,
                             const CalibrateCallback& progress = {}) {
    const std::unique_ptr<CalibrateDepth> depth = checks(kept.cameras.size());
    return value_of(calibrate(kept, progress, depth.get()));
  }
};

// Every camera and every ordered pair passes on an aligned rig of 2 to 5
// cameras, whatever its reference; each view a plane, the board's scale 1,
// no offset or angle, every pair's median at 0, pooled and in each capture,
// and the report names each camera's verdict. One camera's depth is turned
// off its colour camera's axes, and its pairs agree as the others' do.
TEST_F(CalibrateDepthTest, AnAlignedRigOfTwoToFiveCamerasPasses) {
  struct Case {
    std::size_t cameras;
    std::size_t reference;
  };
  for (const Case& c : {Case{2, 0}, Case{3, 1}, Case{4, 0}, Case{5, 3}}) {
    SCOPED_TRACE(std::to_string(c.cameras) + " cameras, reference " +
                 std::to_string(c.reference));
    DepthCase scene;
    scene.cameras = c.cameras;
    scene.reference = c.reference;
    const KeptCaptures kept = depth_captures(scene, "unwritten");
    const CalibrateResult r = calibrated(kept);
    EXPECT_TRUE(r.depth_failure.ok()) << r.depth_failure.message();
    ASSERT_TRUE(r.depth.has_value());
    const DepthSummary& d = *r.depth;

    std::size_t views = 0;
    for (const std::shared_ptr<const KeptCapture>& capture : kept.captures) {
      for (const Detection& view : capture->views) {
        if (!view.observation.empty()) ++views;
      }
    }
    EXPECT_EQ(d.views.size(), views);
    for (std::size_t v = 1; v < d.views.size(); ++v) {
      EXPECT_LE(d.views[v - 1].capture, d.views[v].capture);
    }
    EXPECT_NEAR(d.ratio.mean, 1.0, kPooledRatio);
    EXPECT_EQ(d.scale, DepthVerdict::Ok);
    ASSERT_EQ(d.cameras.size(), c.cameras);
    for (std::size_t k = 0; k < c.cameras; ++k) {
      SCOPED_TRACE(kept.cameras[k].id);
      const quality::CameraDepth& cam = d.cameras[k];
      EXPECT_GT(cam.views, 0u);
      EXPECT_NEAR(cam.ratio.mean, 1.0, kCameraRatio);
      EXPECT_NEAR(cam.offset_mm, 0.0, 0.5);
      EXPECT_LT(cam.angle_deg, kAngleDeg);
      EXPECT_EQ(cam.verdict, DepthVerdict::Ok);
    }
    ASSERT_EQ(d.pairs.size(), c.cameras * (c.cameras - 1));
    for (const PairCheck& p : d.pairs) {
      SCOPED_TRACE(std::to_string(p.agreement.from) + " -> " +
                   std::to_string(p.agreement.to));
      EXPECT_EQ(p.verdict, DepthVerdict::Ok);
      EXPECT_NEAR(p.agreement.signed_median_mm, 0.0, 0.3);
    }
    ASSERT_EQ(d.captures.size(), kept.captures.size());
    for (std::size_t i = 0; i < d.captures.size(); ++i) {
      EXPECT_EQ(d.captures[i].capture, kept.captures[i]->index);
      EXPECT_EQ(d.captures[i].pairs.size(), d.pairs.size());
    }

    const json report = json::parse(r.files.report);
    EXPECT_TRUE(report["depth_unchecked"].is_null());
    for (const solve::RigCamera& cam : kept.cameras) {
      EXPECT_EQ(report["depth"]["cameras"][cam.id]["verdict"], "ok");
    }
  }
}

// A board printed 0.3 % large reads 1.003 in every camera, past the scale's
// bound, so every camera fails.
TEST_F(CalibrateDepthTest, FindsABoardPrintedLarge) {
  DepthCase scene;
  scene.board_scale = 1.003;
  const CalibrateResult r = calibrated(depth_captures(scene, "unwritten"));
  ASSERT_TRUE(r.depth.has_value()) << r.depth_failure.message();
  EXPECT_NEAR(r.depth->ratio.mean, 1.003, kPooledRatio);
  EXPECT_EQ(r.depth->scale, DepthVerdict::Off);
  for (const quality::CameraDepth& cam : r.depth->cameras) {
    EXPECT_NEAR(cam.ratio.mean, 1.003, kCameraRatio);
    EXPECT_EQ(cam.scale, DepthVerdict::Off);
    EXPECT_EQ(cam.verdict, DepthVerdict::Off);
  }
}

// The board's depth read 5 mm deep is a 5 mm offset in every camera, within
// the plane's bound, at no angle.
TEST_F(CalibrateDepthTest, FindsTheBoardsDepthFiveMillimetresDeep) {
  DepthCase scene;
  scene.board_depth_mm = 5.0;
  const CalibrateResult r = calibrated(depth_captures(scene, "unwritten"));
  ASSERT_TRUE(r.depth.has_value()) << r.depth_failure.message();
  for (const quality::CameraDepth& cam : r.depth->cameras) {
    EXPECT_NEAR(cam.offset_mm, 5.0, 0.5);
    EXPECT_LT(cam.angle_deg, kAngleDeg);
    EXPECT_EQ(cam.plane, DepthVerdict::Ok);
  }
}

// A camera whose depth reads 3 mm far: every pair into it reads -3 mm, every
// pair from it farther, and the others 0; each within the pairs' bound.
TEST_F(CalibrateDepthTest, ReadsADepthBiasOnEveryPairIntoItsCamera) {
  DepthCase scene;
  scene.cameras = 4;
  scene.biased = 2;
  const CalibrateResult r = calibrated(depth_captures(scene, "unwritten"));
  ASSERT_TRUE(r.depth.has_value()) << r.depth_failure.message();
  for (const PairCheck& p : r.depth->pairs) {
    const quality::PairAgreement& a = p.agreement;
    SCOPED_TRACE(std::to_string(a.from) + " -> " + std::to_string(a.to));
    if (a.to == 2) {
      EXPECT_NEAR(a.signed_median_mm, -3.0, 0.3);
    } else if (a.from == 2) {
      EXPECT_GT(a.signed_median_mm, 1.0);
    } else {
      EXPECT_NEAR(a.signed_median_mm, 0.0, 0.3);
    }
    EXPECT_EQ(p.verdict, DepthVerdict::Ok);
  }
}

// Depth in half-millimetre units gives the same metres as in millimetres.
TEST_F(CalibrateDepthTest, HalfMillimetreUnitsGiveTheSameMetres) {
  DepthCase mm;
  DepthCase half = mm;
  half.metres_per_unit = 0.0005f;
  const CalibrateResult a = calibrated(depth_captures(mm, "unwritten"));
  const CalibrateResult b = calibrated(depth_captures(half, "unwritten"));
  ASSERT_TRUE(a.depth.has_value()) << a.depth_failure.message();
  ASSERT_TRUE(b.depth.has_value()) << b.depth_failure.message();
  ASSERT_EQ(a.depth->cameras.size(), b.depth->cameras.size());
  for (std::size_t k = 0; k < a.depth->cameras.size(); ++k) {
    const quality::CameraDepth& x = a.depth->cameras[k];
    const quality::CameraDepth& y = b.depth->cameras[k];
    EXPECT_EQ(x.views, y.views);
    EXPECT_NEAR(x.ratio.mean, y.ratio.mean, 3e-4);
    EXPECT_NEAR(x.offset_mm, y.offset_mm, 0.3);
    EXPECT_NEAR(x.angle_deg, y.angle_deg, 0.05);
    EXPECT_EQ(y.verdict, DepthVerdict::Ok);
  }
  ASSERT_EQ(a.depth->pairs.size(), b.depth->pairs.size());
  for (std::size_t i = 0; i < a.depth->pairs.size(); ++i) {
    EXPECT_NEAR(a.depth->pairs[i].agreement.signed_median_mm,
                b.depth->pairs[i].agreement.signed_median_mm, 0.3);
    EXPECT_EQ(b.depth->pairs[i].verdict, DepthVerdict::Ok);
  }
}

// A camera with no depth in any capture, or with no depth model, takes no
// part: it and its pairs are insufficient, pooled and in each capture, and
// the others pass; nothing fails.
TEST_F(CalibrateDepthTest, ACameraWithoutDepthIsInsufficient) {
  DepthCase scene;
  scene.without_depth = 1;
  KeptCaptures kept = depth_captures(scene, "unwritten");
  for (const bool modelled : {true, false}) {
    SCOPED_TRACE(modelled ? "no depth" : "no depth model either");
    if (!modelled) kept.sensors[2].depth.reset();
    const CalibrateResult r = calibrated(kept);
    EXPECT_TRUE(r.depth_failure.ok()) << r.depth_failure.message();
    ASSERT_TRUE(r.depth.has_value());
    const auto missing = [&](std::size_t k) {
      return k == 1 || (!modelled && k == 2);
    };
    for (std::size_t k = 0; k < kept.cameras.size(); ++k) {
      const quality::CameraDepth& cam = r.depth->cameras[k];
      EXPECT_EQ(cam.views == 0, missing(k)) << k;
      EXPECT_EQ(cam.verdict,
                missing(k) ? DepthVerdict::Insufficient : DepthVerdict::Ok)
          << k;
    }
    const auto expect_pairs = [&](const std::vector<PairCheck>& pairs) {
      for (const PairCheck& p : pairs) {
        const bool out = missing(p.agreement.from) || missing(p.agreement.to);
        EXPECT_EQ(p.agreement.overlap == 0, out);
        EXPECT_EQ(p.verdict,
                  out ? DepthVerdict::Insufficient : DepthVerdict::Ok);
      }
    };
    expect_pairs(r.depth->pairs);
    for (const quality::CapturePairs& c : r.depth->captures) {
      expect_pairs(c.pairs);
    }
  }
}

// Each capture's frames are let go before the next is prepared, so the
// preparations reuse their outputs: from the first capture checked to the
// last, the device memory grows by less than one camera's prepared depth.
// The checks report each capture, from 0 to all of them.
TEST_F(CalibrateDepthTest, LetsEachCapturesFramesGo) {
  DepthCase scene;
  scene.cameras = 2;
  scene.placements = 8;
  const KeptCaptures kept = depth_captures(scene, "unwritten");
  std::vector<std::size_t> checked;
  std::vector<std::uint64_t> bytes;
  const CalibrateResult r = calibrated(kept, [&](const CalibrateProgress& at) {
    if (at.stage != CalibrateStage::Checking) return;
    EXPECT_EQ(at.total, kept.captures.size());
    checked.push_back(at.done);
    bytes.push_back(live_bytes(allocator()));
  });
  ASSERT_TRUE(r.depth.has_value()) << r.depth_failure.message();
  ASSERT_EQ(checked.size(), kept.captures.size() + 1);
  for (std::size_t i = 0; i < checked.size(); ++i) EXPECT_EQ(checked[i], i);
  const camera::ImageSize size = test::depth_like_lens().size;
  const std::uint64_t frame = std::uint64_t{size.width} * size.height * 4;
  EXPECT_LT(bytes.back() - bytes[1], frame);
}

// A throw from the progress between captures stops the run there: no capture
// is checked after it, and the run returns what was thrown.
TEST_F(CalibrateDepthTest, AStopBetweenCapturesStopsTheRun) {
  DepthCase scene;
  scene.cameras = 2;
  scene.placements = 8;
  const KeptCaptures kept = depth_captures(scene, "unwritten");
  const std::unique_ptr<CalibrateDepth> depth = checks(2);
  std::vector<std::size_t> checked;
  const core::Result<CalibrateResult> stopped = calibrate(
      kept,
      [&](const CalibrateProgress& at) {
        if (at.stage != CalibrateStage::Checking) return;
        checked.push_back(at.done);
        if (at.done == 2) throw std::runtime_error("stop here");
      },
      depth.get());
  ASSERT_FALSE(stopped.ok());
  EXPECT_NE(stopped.status().message().find("stop here"), std::string::npos)
      << stopped.status().message();
  EXPECT_EQ(checked, (std::vector<std::size_t>{0, 1, 2}));
  // The checks are as they were: the next run passes.
  const core::Result<CalibrateResult> again = calibrate(kept, {}, depth.get());
  ASSERT_TRUE(again.ok()) << again.status().message();
  EXPECT_TRUE(again->depth.has_value()) << again->depth_failure.message();
}

std::unique_ptr<CalibrateWorker> depth_worker(core::Device& device,
                                              core::Allocator& allocator,
                                              std::size_t cameras) {
  return value_of(CalibrateWorker::create(
      value_of(CalibrateDepth::create(device, allocator, cameras))));
}

// A cancel while the worker checks the depth stops it at its next capture:
// nothing is written, published or posted.
TEST_F(CalibrateDepthTest, ACancelDuringTheChecksWritesNothing) {
  const ScratchDir session;
  DepthCase scene;
  scene.cameras = 2;
  scene.placements = 30;
  const KeptCaptures kept = depth_captures(scene, session.path().string());
  const auto gate = std::make_shared<test::Gate>();
  const std::unique_ptr<CalibrateWorker> worker =
      depth_worker(device(), allocator(), 2);
  const test::LetGoAtTheEnd let_go{*gate};
  ASSERT_TRUE(*worker->request(test::held_at_its_end(kept, gate)));
  ASSERT_TRUE(test::wait_for([&] {
    const std::optional<CalibrateProgress> at = worker->progress();
    return !at || (at->stage == CalibrateStage::Checking && at->done >= 1 &&
                   at->done < at->total);
  }));
  const std::optional<CalibrateProgress> at = worker->progress();
  ASSERT_TRUE(at.has_value());
  EXPECT_EQ(at->stage, CalibrateStage::Checking);
  EXPECT_EQ(at->total, kept.captures.size());
  EXPECT_TRUE(worker->cancel());
  const std::optional<CalibrateProgress> cancelled = worker->progress();
  ASSERT_TRUE(gate->held());
  const std::optional<CalibrateProgress> ended = worker->progress();
  ASSERT_TRUE(cancelled.has_value());
  ASSERT_TRUE(ended.has_value());
  EXPECT_EQ(ended->done, cancelled->done);
  gate->let_go();
  ASSERT_TRUE(test::idle(*worker));
  EXPECT_EQ(worker->take_result(), nullptr);
  EXPECT_TRUE(worker->take_errors().empty());
  EXPECT_TRUE(entries(session.path()).empty());
}

// A failure of the depth checks -- a capture's depth not its camera's size
// -- is posted, naming it, and the results are written and published without
// them, the report saying why.
TEST_F(CalibrateDepthTest, AFailureOfTheChecksIsPostedAndTheResultsWritten) {
  const ScratchDir session;
  DepthCase scene;
  scene.cameras = 2;
  scene.placements = 8;
  KeptCaptures kept = depth_captures(scene, session.path().string());
  auto bad = std::make_shared<KeptCapture>(*kept.captures[2]);
  bad->depth[1] =
      std::make_shared<const std::vector<std::uint16_t>>(5, std::uint16_t{1});
  kept.captures[2] = bad;
  const std::unique_ptr<CalibrateWorker> worker =
      depth_worker(device(), allocator(), 2);
  ASSERT_TRUE(*worker->request(test::shared(kept)));
  ASSERT_TRUE(test::idle(*worker));
  const std::vector<core::Status> errors = worker->take_errors();
  const std::string why = "depth checks: capture " +
                          std::to_string(bad->index) + ": camera " +
                          kept.cameras[1].id +
                          "'s depth has 5 samples for its 640 x 576 depth "
                          "camera";
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].domain(), core::Status::Code::InvalidArgument);
  EXPECT_EQ(errors[0].message(), "Calibrate: " + why);
  const std::shared_ptr<const CalibrateResult> result = worker->take_result();
  ASSERT_NE(result, nullptr);
  EXPECT_FALSE(result->depth.has_value());
  EXPECT_EQ(result->depth_failure.message(), why);
  const fs::path dir(result->directory);
  EXPECT_EQ(entries(dir), (std::set<std::string>{quality::kArrayCalibrationFile,
                                                 quality::kReportFile,
                                                 quality::kResidualsFile}));
  const json report = json::parse(test::read(dir / quality::kReportFile));
  EXPECT_TRUE(report["depth"].is_null());
  EXPECT_EQ(report["depth_unchecked"], why);
}

// --- Beside the 3D view's worker ------------------------------------------

class CalibrateDepthBesideTheCloud : public core::test::SharedDeviceTest {
 protected:
  core::DeviceRequirements compute_requirements() const override {
    return value_of(rig_requirements(false)).compute;
  }
  core::DeviceRequirements graphics_requirements() const override {
    return gfx::device_requirements();
  }
  core::test::Validation validation() const override {
    return core::test::Validation::ShaderAccesses;
  }
};

// The 3D view's live sets, of the kept depth alone.
std::shared_ptr<const sensor::FrameSet> live_set(const KeptCaptures& kept,
                                                 std::size_t capture,
                                                 std::uint64_t sequence) {
  auto set = std::make_shared<sensor::FrameSet>();
  set->sequence = sequence;
  const KeptCapture& from = *kept.captures[capture % kept.captures.size()];
  for (std::size_t k = 0; k < kept.cameras.size(); ++k) {
    sensor::RgbdFrame frame;
    frame.depth = from.depth[k]->data();
    frame.pixels = from.depth[k];
    frame.metres_per_unit = kept.depth_mode.metres_per_unit;
    frame.depth_camera = *kept.sensors[k].depth;
    frame.min_depth = kept.depth_mode.min_m;
    frame.max_depth = kept.depth_mode.max_m;
    frame.depth_to_color = kept.sensors[k].depth_to_color;
    frame.sequence = sequence;
    set->frames.emplace_back(std::move(frame));
  }
  return set;
}

// The checks give the same, to the bit, while the 3D view's worker prepares
// live sets on the same device and allocator from another thread, its
// frames shared with the renderer's family as calib_rig makes them.
TEST_F(CalibrateDepthBesideTheCloud, ChecksTheSameWhileTheCloudPrepares) {
  core::test::TwoFamilyDevice& two = two_family();
  DepthCase scene;
  const KeptCaptures kept = depth_captures(scene, "unwritten");
  const std::unique_ptr<CalibrateDepth> depth = value_of(CalibrateDepth::create(
      two.compute(), two.compute_allocator(), kept.cameras.size()));
  const CalibrateResult alone = value_of(calibrate(kept, {}, depth.get()));
  ASSERT_TRUE(alone.depth.has_value()) << alone.depth_failure.message();

  CloudOptions every;
  every.live_every = 1;
  const std::unique_ptr<CloudWorker> cloud = value_of(CloudWorker::create(
      two.compute(), two.compute_allocator(), kept.cameras.size(),
      {two.families()[0], two.families()[1]}, every));
  std::atomic<bool> stopping{false};
  std::thread offering([&] {
    for (std::uint64_t s = 1; !stopping; ++s) {
      cloud->offer(live_set(kept, s, s));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  const core::Result<CalibrateResult> beside = calibrate(kept, {}, depth.get());
  const std::uint64_t prepared = cloud->prepared();
  stopping = true;
  offering.join();
  cloud->stop();
  ASSERT_TRUE(beside.ok()) << beside.status().message();
  ASSERT_TRUE(beside->depth.has_value()) << beside->depth_failure.message();
  EXPECT_GT(prepared, 0u);
  EXPECT_TRUE(cloud->take_errors().empty());
  EXPECT_EQ(json::parse(beside->files.report)["depth"],
            json::parse(alone.files.report)["depth"]);
}

}  // namespace
}  // namespace volumetric_kit::calib::app
