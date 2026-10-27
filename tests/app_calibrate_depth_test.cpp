// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib_rig's depth checks at Calibrate, on a device: the kept captures' raw
// depth, rendered here through a Femto-like depth lens from each camera's true
// pose -- a floor, a box and each capture's board -- with exact detections,
// for 2 to 5 cameras whose ids sort in another order than the rig's, one
// camera's depth turned off its colour camera's axes. An aligned rig passes
// every camera and pair; a board printed 0.3 % large, a board's depth 5 mm
// deep, a camera's depth 3 mm far, cameras' depth 4 mm far and 1 mm near
// along their axes (each camera's depth offset, into the results' file) and
// depth in half-millimetre units each read as they should; a camera's depth
// 8 mm far in one capture fails that capture's own pairs alone; a camera
// without depth is insufficient; each capture's frames are let go before the
// next; a stop between captures stops the run; a failure of the checks is
// posted and the results written without them; a tag world's checks are the
// same as the reference's; and the checks give the same while the 3D view's
// worker prepares on the same device. Under synchronization validation; skips
// without a Vulkan device, and the last case also with one queue family.

#include <gtest/gtest.h>

#include <algorithm>
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
#include "volumetric_kit/recon/camera/array_calibration.hpp"

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
  // How far each camera's depth reads along its axis (mm, + far); none past
  // the last given.
  std::vector<double> bias_mm;
  std::optional<std::size_t> biased_capture;  // the biases in this one alone
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
      const double bias_mm =
          k < c.bias_mm.size() && (!c.biased_capture || c.biased_capture == p)
              ? c.bias_mm[k]
              : 0.0;
      capture->depth[k] = std::make_shared<const std::vector<std::uint16_t>>(
          test::render_raw(depth_rays(), depth_to_world, surface,
                           c.metres_per_unit, bias_mm / 1000.0));
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
      EXPECT_NEAR(1000.0 * cam.depth_offset_m, 0.0, 0.2);
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

// A board printed 0.3 % large reads 1.003 in every camera of 2 to 5, past
// the scale's bound, so every camera fails.
TEST_F(CalibrateDepthTest, FindsABoardPrintedLarge) {
  for (std::size_t n = 2; n <= test::kKeptSceneMaxCameras; ++n) {
    SCOPED_TRACE(std::to_string(n) + " cameras");
    DepthCase scene;
    scene.cameras = n;
    scene.board_scale = 1.003;
    const CalibrateResult r = calibrated(depth_captures(scene, "unwritten"));
    ASSERT_TRUE(r.depth.has_value()) << r.depth_failure.message();
    EXPECT_NEAR(r.depth->ratio.mean, 1.003, kPooledRatio);
    EXPECT_EQ(r.depth->scale, DepthVerdict::Off);
    ASSERT_EQ(r.depth->cameras.size(), n);
    for (const quality::CameraDepth& cam : r.depth->cameras) {
      EXPECT_NEAR(cam.ratio.mean, 1.003, kCameraRatio);
      EXPECT_EQ(cam.scale, DepthVerdict::Off);
      EXPECT_EQ(cam.verdict, DepthVerdict::Off);
    }
  }
}

// The board's depth read 5 mm deep is a 5 mm offset in every camera of 2 to
// 5, within the plane's bound, at no angle.
TEST_F(CalibrateDepthTest, FindsTheBoardsDepthFiveMillimetresDeep) {
  for (std::size_t n = 2; n <= test::kKeptSceneMaxCameras; ++n) {
    SCOPED_TRACE(std::to_string(n) + " cameras");
    DepthCase scene;
    scene.cameras = n;
    scene.board_depth_mm = 5.0;
    const CalibrateResult r = calibrated(depth_captures(scene, "unwritten"));
    ASSERT_TRUE(r.depth.has_value()) << r.depth_failure.message();
    ASSERT_EQ(r.depth->cameras.size(), n);
    for (const quality::CameraDepth& cam : r.depth->cameras) {
      EXPECT_NEAR(cam.offset_mm, 5.0, 0.5);
      EXPECT_LT(cam.angle_deg, kAngleDeg);
      EXPECT_EQ(cam.plane, DepthVerdict::Ok);
    }
  }
}

// The last of 2 to 5 cameras reads its depth 3 mm far: every pair into it
// reads -3 mm, every pair from it about +3 mm (as far as both rays' angles to
// the surface let it), and the others 0; each within the pairs' bound.
TEST_F(CalibrateDepthTest, ReadsADepthBiasOnEveryPairIntoItsCamera) {
  for (std::size_t n = 2; n <= test::kKeptSceneMaxCameras; ++n) {
    SCOPED_TRACE(std::to_string(n) + " cameras");
    DepthCase scene;
    scene.cameras = n;
    scene.bias_mm.assign(n, 0.0);
    scene.bias_mm[n - 1] = 3.0;
    const CalibrateResult r = calibrated(depth_captures(scene, "unwritten"));
    ASSERT_TRUE(r.depth.has_value()) << r.depth_failure.message();
    ASSERT_EQ(r.depth->pairs.size(), n * (n - 1));
    for (const PairCheck& p : r.depth->pairs) {
      const quality::PairAgreement& a = p.agreement;
      SCOPED_TRACE(std::to_string(a.from) + " -> " + std::to_string(a.to));
      if (a.to == n - 1) {
        EXPECT_NEAR(a.signed_median_mm, -3.0, 0.3);
      } else if (a.from == n - 1) {
        EXPECT_NEAR(a.signed_median_mm, 3.0, 0.5);
      } else {
        EXPECT_NEAR(a.signed_median_mm, 0.0, 0.3);
      }
      EXPECT_EQ(p.verdict, DepthVerdict::Ok);
    }
  }
}

// How obliquely each depth view of `c`'s scene sees its board: the angle
// between the board's normal and the ray from the depth camera to the
// chessboard's centre (degrees), from the true poses.
std::vector<double> incidences_deg(const DepthCase& c, const KeptCaptures& kept,
                                   const DepthSummary& d) {
  const test::Scene scene = test::make_kept_scene(c.cameras, c.placements, 0.0);
  const cv::Point3d centre = scene.board.centre();
  std::vector<double> out;
  for (const quality::BoardDepthCheck& view : d.views) {
    const auto p = static_cast<std::size_t>((view.capture - 1) / 2);
    const Mat4d& board_to_world = scene.plan[p].board_to_world;
    const Vec3d origin(
        (camera::rigid_inverse(scene.rig[view.camera].world_to_camera) *
         kept.sensors[view.camera].depth_to_color)[3]);
    const Vec3d at(board_to_world * glm::dvec4(centre.x, centre.y, 0.0, 1.0));
    const Vec3d normal = Mat3d(board_to_world) * Vec3d(0.0, 0.0, 1.0);
    const double cosine =
        std::abs(glm::dot(normal, glm::normalize(at - origin)));
    out.push_back(glm::degrees(std::acos(std::min(cosine, 1.0))));
  }
  return out;
}

// Of 4 cameras, one reads its depth 4 mm far along its axis (the one whose
// depth camera is turned off its colour camera's axes) and one 1 mm near:
// each camera's depth offset is its bias within 0.2 mm, the others' 0, over
// boards seen at 28 to 68.5 degrees, where the offsets along the normals
// understate the bias by up to half; every view's axial offset is within
// 1 mm of it (the preparation's nearest-pixel depth strays a view's plane),
// every plane passes, and the results' calibration file carries each offset
// as recon reads it back. The checks read raw depth, so the offsets are the
// depth's own.
TEST_F(CalibrateDepthTest, FindsEachCamerasAxialDepthOffset) {
  DepthCase scene;
  scene.cameras = 4;
  scene.bias_mm = {0.0, 4.0, -1.0, 0.0};
  ASSERT_EQ(turned_camera(scene), 1u);
  const KeptCaptures kept = depth_captures(scene, "unwritten");
  const CalibrateResult r = calibrated(kept);
  ASSERT_TRUE(r.depth.has_value()) << r.depth_failure.message();
  const DepthSummary& d = *r.depth;
  const camera::ArrayCalibration file =
      value_of(camera::parse_array_calibration(r.files.array_calibration));
  EXPECT_EQ(json::parse(r.files.array_calibration)["version"], 3);
  ASSERT_EQ(d.cameras.size(), 4u);
  for (std::size_t k = 0; k < 4; ++k) {
    SCOPED_TRACE(kept.cameras[k].id);
    const quality::CameraDepth& cam = d.cameras[k];
    EXPECT_NEAR(1000.0 * cam.depth_offset_m, scene.bias_mm[k], 0.2);
    EXPECT_LT(cam.scatter_mm, 0.5);
    EXPECT_EQ(cam.plane, DepthVerdict::Ok);
    const camera::SensorCalibration* sensor =
        camera::find_sensor(file, kept.cameras[k].id);
    ASSERT_NE(sensor, nullptr);
    EXPECT_EQ(sensor->depth_offset_m, cam.depth_offset_m);
  }
  const std::vector<double> incidence = incidences_deg(scene, kept, d);
  ASSERT_EQ(incidence.size(), d.views.size());
  double least = 90.0;
  double most = 0.0;
  for (std::size_t v = 0; v < d.views.size(); ++v) {
    const quality::BoardDepthCheck& view = d.views[v];
    SCOPED_TRACE("camera " + std::to_string(view.camera) + ", capture " +
                 std::to_string(view.capture) + ", " +
                 std::to_string(incidence[v]) + " degrees");
    EXPECT_NEAR(view.axial_offset_mm, scene.bias_mm[view.camera], 1.0);
    least = std::min(least, incidence[v]);
    most = std::max(most, incidence[v]);
  }
  // The scene's views span 28 to 68.5 degrees.
  EXPECT_LT(least, 30.0);
  EXPECT_GT(most, 65.0);
}

// A camera whose depth reads 8 mm far in one capture alone: that capture's
// own pairs into and from it fail, its pairs into it at -8 mm, and every
// other pair of it, every pair of every other capture and every pooled pair
// passes at 0. Each pair's overlap over the captures is its pooled overlap.
TEST_F(CalibrateDepthTest, JudgesEachCapturesPairsOnTheirOwn) {
  DepthCase scene;
  scene.bias_mm = {0.0, 0.0, 8.0};
  scene.biased_capture = 3;
  const KeptCaptures kept = depth_captures(scene, "unwritten");
  const CalibrateResult r = calibrated(kept);
  ASSERT_TRUE(r.depth.has_value()) << r.depth_failure.message();
  const DepthSummary& d = *r.depth;
  for (const PairCheck& p : d.pairs) {
    SCOPED_TRACE(std::to_string(p.agreement.from) + " -> " +
                 std::to_string(p.agreement.to));
    EXPECT_NEAR(p.agreement.signed_median_mm, 0.0, 0.5);
    EXPECT_EQ(p.verdict, DepthVerdict::Ok);
  }
  ASSERT_EQ(d.captures.size(), kept.captures.size());
  std::vector<std::size_t> overlap(d.pairs.size(), 0);
  for (std::size_t i = 0; i < d.captures.size(); ++i) {
    SCOPED_TRACE("capture " + std::to_string(d.captures[i].capture));
    EXPECT_EQ(d.captures[i].capture, kept.captures[i]->index);
    ASSERT_EQ(d.captures[i].pairs.size(), d.pairs.size());
    for (std::size_t j = 0; j < d.pairs.size(); ++j) {
      const PairCheck& p = d.captures[i].pairs[j];
      const quality::PairAgreement& a = p.agreement;
      SCOPED_TRACE(std::to_string(a.from) + " -> " + std::to_string(a.to));
      ASSERT_EQ(a.from, d.pairs[j].agreement.from);
      ASSERT_EQ(a.to, d.pairs[j].agreement.to);
      overlap[j] += a.overlap;
      const bool biased = i == *scene.biased_capture;
      if (biased && a.to == 2) {
        EXPECT_NEAR(a.signed_median_mm, -8.0, 0.5);
      } else if (!biased || a.from != 2) {
        EXPECT_NEAR(a.signed_median_mm, 0.0, 0.5);
      }
      EXPECT_EQ(p.verdict, biased && (a.from == 2 || a.to == 2)
                               ? DepthVerdict::Off
                               : DepthVerdict::Ok);
    }
  }
  for (std::size_t j = 0; j < d.pairs.size(); ++j) {
    EXPECT_EQ(overlap[j], d.pairs[j].agreement.overlap) << j;
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

// A throw from the progress between captures, as the worker's cancel throws
// (CalibrateWorker.ACancelWritesNothing), stops the run there: no capture is
// checked after it, and the run returns what was thrown, not a failure of the
// checks.
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

// With the origin tag seen, the poses are in its world, and the depth
// section is the one the same captures without the tag give, to the bit.
TEST_F(CalibrateDepthTest, ChecksTheSameDepthInATagWorld) {
  DepthCase c;
  c.reference = 1;
  const KeptCaptures plain = depth_captures(c, "unwritten");
  const KeptCaptures tagged = test::with_origin_tag(
      plain, test::make_kept_scene(c.cameras, c.placements, 0.0));
  const CalibrateResult without = calibrated(plain);
  const CalibrateResult with = calibrated(tagged);
  ASSERT_FALSE(without.world.origin.tag);
  ASSERT_TRUE(with.world.origin.tag) << with.world.origin.reason;
  ASSERT_TRUE(with.depth.has_value()) << with.depth_failure.message();
  EXPECT_NE(with.color_to_world, without.color_to_world);
  EXPECT_EQ(json::parse(with.files.report)["depth"],
            json::parse(without.files.report)["depth"]);
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
