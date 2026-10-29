// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The depth checks' board-plane fit on a device, against references in double
// made here -- quality::in_board_region's membership, both rounds' gates and
// least-squares planes, OpenCV's eigen solve: tilted planes on rigs of 1, 3, 4
// and 5 cameras of mixed sizes, with two regions on one depth, in one batch;
// the region's rule pixel by pixel; members counted with depth or without,
// wherever the region lies -- past the image's edges, partly or wholly behind
// the camera, or over more pixels than one pass takes; a plane turned near
// the seed's limit, to 1e-4 degrees; a box on the plane;
// 20 % outliers either side, and outliers on one edge; settings other than
// the defaults; two runs to the bit; too few inliers, or inliers on one line;
// the requirements; and refusals. Skips where no device is present.
//
// The device works in float and the references in double, so a pixel whose
// ray meets the board's plane within kBand of the inset outline, or a point
// within kBand of a gate, may be decided either way: counts may differ by
// those alone, which each test keeps to a handful. Where every value is a
// multiple of a power of two (dyadic_view), nothing rounds, and the counts
// and the plane are exact.

#include "volumetric_kit/calib/depth/depth_checks.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include "board_member_cases_comp.spv.hpp"
#include "depth_fixture.hpp"
#include "depth_scene.hpp"
#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_kernel.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"

namespace volumetric_kit::calib::depth {
namespace {

namespace camera = sensor::camera;
using camera::Mat3d;
using camera::Mat4d;
using camera::Vec2d;
using camera::Vec3d;
using quality::BoardRegion;
using quality::DepthCheckOptions;
using quality::DepthPlane;
using test::binned;
using test::check;
using test::expect_refused;
using test::hit_box;
using test::hit_plane;
using test::kMaxDepth;
using test::kMinDepth;
using test::measured;
using test::nfov;
using test::Pinhole;
using test::Plane;
using test::ray_of;
using test::render;
using test::rotation;
using test::Surface;
using test::value_of;
using test::wide;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
// How near the inset outline, or a gate, float and double may decide apart
// (metres): ten times the float arithmetic's error a few metres out.
constexpr double kBand = 1e-5;

// A depth camera 3 cm beside its colour camera, turned 5.5 degrees down.
Mat4d depth_to_color() {
  Mat4d m(rotation({1.0, -0.03, 0.02}, -5.5));
  m[3] = glm::dvec4(0.0318, -0.0019, 0.0042, 1.0);
  return m;
}

// The solved board's plane moved `offset_mm` into the board and turned
// `turn_deg` about `axis` at the chessboard's centre.
Plane plane_off(double offset_mm, const Vec3d& axis, double turn_deg) {
  const cv::Point3d c = test::femto_board().centre();
  return {Vec3d(c.x, c.y, offset_mm / 1000.0),
          rotation(axis, turn_deg) * Vec3d(0.0, 0.0, 1.0)};
}

// The plane's distance from the board at `at`, along the board's z
// (millimetres): board_depth_check's offset.
double offset_at(const Vec3d& point, const Vec3d& normal, const Vec2d& at) {
  return 1000.0 * (point.z -
                   (normal.x * (at.x - point.x) + normal.y * (at.y - point.y)) /
                       normal.z);
}
double offset_at(const DepthPlane& plane, const Vec2d& at) {
  return offset_at(plane.point, plane.normal, at);
}

Vec2d board_centre() {
  const cv::Point3d c = test::femto_board().centre();
  return {c.x, c.y};
}

// A board's view: the region its fit takes, its camera's depth, and the
// board in the colour camera.
struct View {
  Pinhole pinhole;
  BoardRegion region;
  std::vector<float> depth;
  Mat4d board_to_color{1.0};
};

// The rig's board with its centre at `centre` in a colour camera, turned
// `tilt_deg` and rolled `roll_deg` (test::board_facing_camera), seen by a
// depth camera of `pinhole` beside it whose depth is `surface`.
View view_of(const Pinhole& pinhole, const Vec3d& centre, double tilt_deg,
             double roll_deg, const Surface& surface,
             const DepthCheckOptions& options = {}) {
  const compute::Board board = test::femto_board();
  View view;
  view.pinhole = pinhole;
  view.board_to_color =
      test::board_facing_camera(board, centre, tilt_deg, roll_deg);
  view.region = value_of(quality::board_region(board, view.board_to_color,
                                               depth_to_color(), options));
  view.depth = render(pinhole, view.region.depth_to_board, surface);
  return view;
}

// A depth camera 1 m in front of the board and square to it, 512 x 640, with a
// pinhole whose rays meet the plane at multiples of 2^-9 m, and the region of
// columns 100 to 400 and rows 300 to 550: every value the device computes is
// exact in float. Its depth is `depth` everywhere.
constexpr std::uint32_t kDyadicWidth = 512;
constexpr std::size_t kDyadicMembers = 301 * 251;
View dyadic_view(float depth) {
  View view;
  view.pinhole = {{kDyadicWidth, 640}, {512.0, 512.0, 256.0, 320.0}};
  view.region.depth_to_board[3] = glm::dvec4(0.25, 0.125, -1.0, 1.0);
  view.region.board_to_color = depth_to_color();
  view.region.min =
      Vec2d(0.25 + (100.0 - 256.0) / 512.0, 0.125 + (300.0 - 320.0) / 512.0);
  view.region.max =
      Vec2d(0.25 + (400.0 - 256.0) / 512.0, 0.125 + (550.0 - 320.0) / 512.0);
  view.depth.assign(std::size_t{kDyadicWidth} * 640, depth);
  return view;
}

// Whether a ray meeting the plane at `at` is within kBand of the outline.
bool near_outline(const BoardRegion& region, const Vec2d& at) {
  const auto near = [](double a, double b) { return std::abs(a - b) < kBand; };
  const bool across_x =
      at.y >= region.min.y - kBand && at.y <= region.max.y + kBand;
  const bool across_y =
      at.x >= region.min.x - kBand && at.x <= region.max.x + kBand;
  return (across_x && (near(at.x, region.min.x) || near(at.x, region.max.x))) ||
         (across_y && (near(at.y, region.min.y) || near(at.y, region.max.y)));
}

// The least-squares plane through `points` (board frame), valid by the
// device's rule: enough of them, their least spread clear of the middle one.
DepthPlane least_squares(const std::vector<Vec3d>& points,
                         std::size_t min_inliers) {
  DepthPlane plane;
  plane.inliers = points.size();
  if (points.size() < min_inliers) return plane;
  Vec3d mean(0.0);
  for (const Vec3d& p : points) mean += p;
  mean /= static_cast<double>(points.size());
  cv::Matx33d spread = cv::Matx33d::zeros();
  for (const Vec3d& p : points) {
    const Vec3d d = p - mean;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) spread(i, j) += d[i] * d[j];
    }
  }
  spread *= 1.0 / static_cast<double>(points.size());
  cv::Mat values;
  cv::Mat vectors;
  cv::eigen(spread, values, vectors);  // descending, vectors in rows
  const double e0 = values.at<double>(0);
  const double e1 = values.at<double>(1);
  const double e2 = values.at<double>(2);
  if (!(e1 - e2 > 1e-5 * e0)) return plane;
  Vec3d normal(vectors.at<double>(2, 0), vectors.at<double>(2, 1),
               vectors.at<double>(2, 2));
  if (normal.z < 0.0) normal = -normal;
  plane.valid = true;
  plane.point = mean;
  plane.normal = normal;
  plane.rms_m = std::sqrt(std::max(e2, 0.0));
  return plane;
}

// Both rounds of a view's fit in double, and the decisions float may take
// otherwise.
struct Reference {
  DepthPlane plane;  ///< What the fit gives.
  DepthPlane first;  ///< The first round's plane.
  /// Pixels whose ray meets the plane within kBand of the outline.
  std::size_t outline_band = 0;
  std::size_t gate_band = 0;  ///< Points within kBand of either gate.
};

Reference reference_fit(const View& view, const DepthCheckOptions& options) {
  const BoardRegion& region = view.region;
  const Pinhole& p = view.pinhole;
  const Vec3d origin(region.depth_to_board[3]);
  const Mat3d turn(region.depth_to_board);
  Reference out;
  std::vector<Vec3d> points;
  std::size_t members = 0;
  std::size_t out_of_range = 0;
  for (std::uint32_t v = 0; v < p.size.height; ++v) {
    for (std::uint32_t u = 0; u < p.size.width; ++u) {
      const Vec3d ray = ray_of(p, u, v);
      const Vec3d direction = turn * ray;
      const double s = -origin.z / direction.z;
      if (s > 0.0 && std::isfinite(s) &&
          near_outline(region, Vec2d(origin + s * direction))) {
        ++out.outline_band;
      }
      if (!quality::in_board_region(region, ray)) continue;
      ++members;
      const double z = view.depth[std::size_t{v} * p.size.width + u];
      if (!measured(z)) {
        ++out_of_range;
        continue;
      }
      points.push_back(origin + z * direction);
    }
  }
  const auto gated = [&](double gate, const auto& distance) {
    std::vector<Vec3d> in;
    for (const Vec3d& point : points) {
      const double d = std::abs(distance(point));
      if (std::abs(d - gate) < kBand) ++out.gate_band;
      if (d < gate) in.push_back(point);
    }
    return in;
  };
  out.first = least_squares(
      gated(options.seed_mm / 1000.0, [](const Vec3d& q) { return q.z; }),
      options.min_inliers);
  out.first.members = members;
  out.first.out_of_range = out_of_range;
  out.plane = out.first;
  if (!out.first.valid) return out;
  const DepthPlane& first = out.first;
  out.plane =
      least_squares(gated(std::max(options.min_gate_mm / 1000.0,
                                   options.sigma_gate * first.rms_m),
                          [&](const Vec3d& q) {
                            return glm::dot(first.normal, q - first.point);
                          }),
                    options.min_inliers);
  out.plane.members = members;
  out.plane.out_of_range = out_of_range;
  return out;
}

std::size_t apart(std::size_t a, std::size_t b) {
  return a > b ? a - b : b - a;
}

// The device's plane against the reference's: the counts equal but for the
// decisions float may take otherwise, the plane within `offset_mm` at the
// board's centre and `angle_deg`, and the RMS within 0.05 mm.
void expect_like(const DepthPlane& got, const Reference& want, double offset_mm,
                 double angle_deg) {
  EXPECT_LE(apart(got.members, want.plane.members), want.outline_band);
  EXPECT_LE(apart(got.out_of_range, want.plane.out_of_range),
            want.outline_band);
  EXPECT_LE(apart(got.inliers, want.plane.inliers),
            want.outline_band + want.gate_band);
  ASSERT_EQ(got.valid, want.plane.valid);
  if (!got.valid) return;
  EXPECT_NEAR(offset_at(got, board_centre()),
              offset_at(want.plane, board_centre()), offset_mm);
  EXPECT_LE(compute::angle_deg(got.normal, want.plane.normal), angle_deg);
  EXPECT_NEAR(got.rms_m, want.plane.rms_m, 5e-5);
  EXPECT_NEAR(glm::length(got.normal), 1.0, 1e-6);
  EXPECT_GE(got.normal.z, 0.0);
}

// Mirrors Case in shaders/board_member_cases.comp under std430.
struct MemberCase {
  glm::mat4 depth_to_board{1.0f};
  glm::vec4 k{0.0f};
  glm::vec4 rect{0.0f};
  glm::uvec2 size{0u};
};
static_assert(sizeof(MemberCase) == 104 && offsetof(MemberCase, k) == 64 &&
                  offsetof(MemberCase, rect) == 80 &&
                  offsetof(MemberCase, size) == 96,
              "mirrors Case in shaders/board_member_cases.comp");

class PlaneFitTest : public test::DepthChecksTest {
 protected:
  // `view`'s fit, its depth in `buffer`.
  static PlaneRegion region_of(const View& view, const core::Buffer& buffer) {
    PlaneRegion region;
    region.image.depth = &buffer;
    region.image.intrinsics = view.pinhole.k;
    region.image.size = view.pinhole.size;
    region.image.min_depth_m = kMinDepth;
    region.image.max_depth_m = kMaxDepth;
    region.image.depth_to_color = depth_to_color();
    Mat4d color_to_world(rotation({0.0, 1.0, 0.0}, 30.0));
    color_to_world[3] = glm::dvec4(0.5, -0.2, 1.0, 1.0);
    region.image.depth_to_world = color_to_world * depth_to_color();
    region.region = view.region;
    return region;
  }

  // Each view's fit, its depth uploaded into `buffers`.
  std::vector<PlaneRegion> regions_of(const std::vector<View>& views,
                                      std::vector<core::Buffer>& buffers) {
    buffers.clear();
    buffers.reserve(views.size());
    std::vector<PlaneRegion> regions;
    for (const View& view : views) {
      buffers.push_back(upload(view.depth));
      regions.push_back(region_of(view, buffers.back()));
    }
    return regions;
  }

  // The device's answer, 1 or 0, for each pixel of a `size` image with
  // pinhole `k` in `region`, through the tier's rule.
  std::vector<std::uint32_t> device_members(const BoardRegion& region,
                                            const glm::vec4& k,
                                            const camera::ImageSize& size) {
    core::ComputeKernel kernel;
    core::KernelSetBuilder builder(device());
    check(builder.add(kernel, "board_member_cases",
                      vc_test_board_member_cases_comp_spv,
                      vc_test_board_member_cases_comp_spv_size, 2));
    const core::DescriptorPool pool = value_of(builder.build());
    MemberCase c;
    c.depth_to_board = glm::mat4(region.depth_to_board);
    c.k = k;
    c.rect = glm::vec4(region.min.x, region.min.y, region.max.x, region.max.y);
    c.size = glm::uvec2(size.width, size.height);
    const std::uint32_t pixels = size.width * size.height;
    const core::Buffer in =
        value_of(core::device_storage_buffer(allocator(), sizeof(c)));
    const core::Buffer out = value_of(core::device_storage_buffer(
        allocator(), pixels * sizeof(std::uint32_t)));
    kernel.set.write_storage_buffer(0, in.handle(), 0, sizeof(c));
    kernel.set.write_storage_buffer(1, out.handle(), 0, VK_WHOLE_SIZE);
    std::vector<std::uint32_t> members(pixels, 7u);
    core::CommandBatch batch(device(), allocator());
    check(batch.upload(in, 0, &c, sizeof(c)));
    check(batch.dispatch(kernel, nullptr, 0, core::group_count(pixels, 64),
                         device().caps().limits().maxComputeWorkGroupCount[0]));
    check(
        batch.readback(out, 0, pixels * sizeof(std::uint32_t), members.data()));
    check(batch.submit());
    return members;
  }
};

TEST(DepthChecksRequirements, AreTheCoresComputeBaselineAlone) {
  const core::DeviceRequirements got = DepthChecks::device_requirements();
  const core::DeviceRequirements baseline;
  EXPECT_EQ(got.api_version, baseline.api_version);
  EXPECT_EQ(got.api_version, VK_API_VERSION_1_2);
  EXPECT_EQ(got.queue_flags, VkQueueFlags{VK_QUEUE_COMPUTE_BIT});
  EXPECT_FALSE(got.needs_present);
  EXPECT_TRUE(got.extensions.empty());
  EXPECT_TRUE(got.optional_extensions.empty());
  EXPECT_EQ(std::memcmp(&got.features, &baseline.features,
                        sizeof(VkPhysicalDeviceFeatures)),
            0);
  EXPECT_TRUE(got.timeline_semaphore);
  EXPECT_FALSE(got.scalar_block_layout);
  EXPECT_FALSE(got.dynamic_rendering);
  EXPECT_EQ(got.feature_chain, nullptr);
}

// Each camera's depth is a plane a few millimetres off its solved board and
// turned about a degree; the fit finds it within 0.05 mm and 0.01 degrees,
// for every camera of rigs of 1, 3, 4 and 5 of mixed sizes, each rig in one
// batch, with a second region, inset farther, on the first camera's depth.
// (The fixture's device enabled the core's baseline alone.)
TEST_F(PlaneFitTest, FitsTiltedPlanesOnRigsOfAnySize) {
  const compute::Board board = test::femto_board();
  const DepthCheckOptions options;
  const Pinhole pinholes[] = {nfov(), binned(), nfov(), wide(), nfov()};
  for (const std::size_t cameras : {1u, 3u, 4u, 5u}) {
    SCOPED_TRACE(std::to_string(cameras) + " cameras");
    std::vector<View> views;
    std::vector<Plane> truths;
    for (std::size_t c = 0; c < cameras; ++c) {
      const double t = static_cast<double>(c);
      const Plane truth =
          plane_off(2.0 + 1.5 * t, {1.0, 0.4 * t - 0.6, 0.1}, 0.4 + 0.3 * t);
      truths.push_back(truth);
      views.push_back(view_of(
          pinholes[c], {0.04 * t - 0.08, 0.03 * t - 0.05, 1.45 + 0.2 * t},
          -25.0 + 13.0 * t, c % 2 == 0 ? 0.0 : 90.0,
          [&](const Vec3d& o, const Vec3d& d) {
            return hit_plane(truth, o, d);
          }));
    }
    DepthCheckOptions deeper = options;
    deeper.inset_m = 0.06;
    View inner = views[0];
    inner.region = value_of(quality::board_region(board, inner.board_to_color,
                                                  depth_to_color(), deeper));
    std::vector<core::Buffer> buffers;
    std::vector<PlaneRegion> regions = regions_of(views, buffers);
    PlaneRegion second = regions[0];
    second.region = inner.region;
    regions.push_back(second);
    views.push_back(inner);
    truths.push_back(truths[0]);

    const std::vector<DepthPlane> planes =
        value_of(checks_->fit_planes(regions, options));
    ASSERT_EQ(planes.size(), cameras + 1);
    for (std::size_t r = 0; r <= cameras; ++r) {
      SCOPED_TRACE("region " + std::to_string(r));
      const Reference want = reference_fit(views[r], options);
      EXPECT_LT(want.outline_band, want.plane.members / 100);
      EXPECT_EQ(want.gate_band, 0u);
      ASSERT_TRUE(want.plane.valid);
      expect_like(planes[r], want, 0.05, 0.01);
      const Plane& truth = truths[r];
      EXPECT_NEAR(offset_at(planes[r], board_centre()),
                  offset_at(truth.point, truth.normal, board_centre()), 0.05);
      EXPECT_LE(compute::angle_deg(planes[r].normal, truth.normal), 0.01);
      EXPECT_LT(planes[r].rms_m, 5e-5);
      EXPECT_EQ(planes[r].out_of_range, 0u);
      EXPECT_EQ(planes[r].inliers, planes[r].members);
    }
    EXPECT_LT(planes[cameras].members, planes[0].members);
  }
}

// The tier's rule, on the device, is the host's for every pixel: exactly
// where nothing rounds, edges included; elsewhere wherever float cannot
// round across the outline; and for no pixel of a camera facing away, one in
// the board's plane, or intrinsics not a number.
TEST_F(PlaneFitTest, MembershipIsTheHostsRuleForEveryPixel) {
  const auto host = [](const BoardRegion& region, const Pinhole& p) {
    std::vector<std::uint32_t> members;
    for (std::uint32_t v = 0; v < p.size.height; ++v) {
      for (std::uint32_t u = 0; u < p.size.width; ++u) {
        members.push_back(
            quality::in_board_region(region, ray_of(p, u, v)) ? 1u : 0u);
      }
    }
    return members;
  };
  const auto k_of = [](const Pinhole& p) {
    return glm::vec4(p.k.fx, p.k.fy, p.k.cx, p.k.cy);
  };

  const View exact = dyadic_view(1.0f);
  const std::vector<std::uint32_t> device =
      device_members(exact.region, k_of(exact.pinhole), exact.pinhole.size);
  EXPECT_EQ(device, host(exact.region, exact.pinhole));
  std::size_t count = 0;
  for (const std::uint32_t m : device) count += m;
  EXPECT_EQ(count, kDyadicMembers);

  const Pinhole p = nfov();
  for (const double roll : {0.0, 90.0}) {
    for (const double tilt : {-30.0, 0.0, 35.0}) {
      SCOPED_TRACE("roll " + std::to_string(roll) + ", tilt " +
                   std::to_string(tilt));
      const View view = view_of(p, {0.05, 0.08, 1.7}, tilt, roll,
                                [](const Vec3d&, const Vec3d&) { return 0.0; });
      const std::vector<std::uint32_t> got =
          device_members(view.region, k_of(p), p.size);
      const std::vector<std::uint32_t> want = host(view.region, p);
      const Vec3d origin(view.region.depth_to_board[3]);
      const Mat3d turn(view.region.depth_to_board);
      std::size_t members = 0;
      std::size_t banded = 0;
      for (std::uint32_t v = 0; v < p.size.height; ++v) {
        for (std::uint32_t u = 0; u < p.size.width; ++u) {
          const std::size_t i = std::size_t{v} * p.size.width + u;
          members += want[i];
          const Vec3d d = turn * ray_of(p, u, v);
          if (near_outline(view.region, Vec2d(origin - origin.z / d.z * d))) {
            ++banded;
            continue;
          }
          EXPECT_EQ(got[i], want[i]) << "pixel " << u << ", " << v;
        }
      }
      EXPECT_GT(members, 10000u);
      EXPECT_LT(banded, members / 1000);
    }
  }

  BoardRegion away = exact.region;
  away.depth_to_board[1] = glm::dvec4(0.0, -1.0, 0.0, 0.0);
  away.depth_to_board[2] = glm::dvec4(0.0, 0.0, -1.0, 0.0);
  BoardRegion in_plane = exact.region;
  in_plane.depth_to_board[3].z = 0.0;
  glm::vec4 no_focal = k_of(exact.pinhole);
  no_focal.x = kNaN;
  for (const auto& [region, k] : {std::pair{away, k_of(exact.pinhole)},
                                  std::pair{in_plane, k_of(exact.pinhole)},
                                  std::pair{exact.region, no_focal}}) {
    const std::vector<std::uint32_t> none =
        device_members(region, k, exact.pinhole.size);
    EXPECT_EQ(std::count(none.begin(), none.end(), 0u),
              static_cast<std::ptrdiff_t>(none.size()));
  }
  for (const std::uint32_t m : host(away, exact.pinhole)) EXPECT_EQ(m, 0u);
  for (const std::uint32_t m : host(in_plane, exact.pinhole)) EXPECT_EQ(m, 0u);
}

// A pixel is a member by its ray, whatever its depth: holes, NaN and depth
// either side of the range leave the members as they are and are counted
// out of range, the range's ends inside it, and holes still are with a range
// from 0; and where nothing rounds, the plane through the board itself comes
// out exact.
TEST_F(PlaneFitTest, CountsEveryMemberWithDepthOrWithout) {
  const View whole = dyadic_view(1.0f);
  View holed = whole;
  std::size_t holes = 0;
  std::size_t missing = 0;  // of the holes, those 0 or NaN
  for (std::size_t i = 0; i < holed.depth.size(); ++i) {
    float& z = holed.depth[i];
    if (i % 7 == 0) {
      z = 0.0f;  // none
    } else if (i % 11 == 0) {
      z = kNaN;
    } else if (i % 13 == 0) {
      z = 0.1f;  // nearer than the range
    } else if (i % 17 == 0) {
      z = 9.0f;  // farther
    }
    const std::uint32_t u = static_cast<std::uint32_t>(i % kDyadicWidth);
    const std::uint32_t v = static_cast<std::uint32_t>(i / kDyadicWidth);
    if (z != 1.0f && u >= 100 && u <= 400 && v >= 300 && v <= 550) {
      ++holes;
      if (!(z > 0.0f)) ++missing;
    }
  }
  std::vector<core::Buffer> buffers;
  std::vector<PlaneRegion> regions = regions_of({whole, holed}, buffers);
  // The range's ends are inside it.
  PlaneRegion nearest = regions[0];
  nearest.image.min_depth_m = 1.0;
  nearest.image.max_depth_m = 2.0;
  PlaneRegion farthest = regions[0];
  farthest.image.min_depth_m = 0.5;
  farthest.image.max_depth_m = 1.0;
  PlaneRegion past = regions[0];
  past.image.min_depth_m = std::nextafter(1.0f, 2.0f);
  // From 0, past the far depth: the holes alone are out of range.
  PlaneRegion open = regions[1];
  open.image.min_depth_m = 0.0;
  open.image.max_depth_m = 10.0;
  regions.insert(regions.end(), {nearest, farthest, past, open});

  const std::vector<DepthPlane> planes =
      value_of(checks_->fit_planes(regions, {}));
  ASSERT_EQ(planes.size(), 6u);
  const Vec2d centre = 0.5 * (whole.region.min + whole.region.max);
  for (const std::size_t r : {0u, 2u, 3u}) {
    SCOPED_TRACE("region " + std::to_string(r));
    const DepthPlane& plane = planes[r];
    EXPECT_EQ(plane.members, kDyadicMembers);
    EXPECT_EQ(plane.out_of_range, 0u);
    EXPECT_EQ(plane.inliers, kDyadicMembers);
    ASSERT_TRUE(plane.valid);
    EXPECT_EQ(plane.point, Vec3d(centre, 0.0));
    EXPECT_EQ(plane.normal, Vec3d(0.0, 0.0, 1.0));
    EXPECT_EQ(plane.rms_m, 0.0);
  }
  EXPECT_EQ(planes[1].members, kDyadicMembers);
  EXPECT_GT(holes, 20000u);
  EXPECT_EQ(planes[1].out_of_range, holes);
  EXPECT_EQ(planes[1].inliers, kDyadicMembers - holes);
  EXPECT_TRUE(planes[1].valid);
  EXPECT_EQ(planes[4].members, kDyadicMembers);
  EXPECT_EQ(planes[4].out_of_range, kDyadicMembers);
  EXPECT_EQ(planes[4].inliers, 0u);
  EXPECT_FALSE(planes[4].valid);
  // The near and far depths are in range there, but past the seed.
  EXPECT_EQ(planes[5].members, kDyadicMembers);
  EXPECT_GT(missing, 10000u);
  EXPECT_LT(missing, holes);
  EXPECT_EQ(planes[5].out_of_range, missing);
  EXPECT_EQ(planes[5].inliers, kDyadicMembers - holes);
  EXPECT_TRUE(planes[5].valid);
}

// The moments walk the box round a region's projected corners, and every
// member is in it wherever the region lies. Square to a 1024 x 640 camera,
// where nothing rounds: a region past the image on every side counts every
// pixel, more than one pass of the workgroups takes (1024 of 256), so each
// thread walks several; one past its corner counts the pixels inside; one
// beside it counts none. Along a floor: a region partly behind the camera,
// and one wholly behind, count as the host's rule does.
TEST_F(PlaneFitTest, CountsEveryMemberWhereverTheRegionLies) {
  const DepthCheckOptions options;
  const Pinhole pinhole = {{1024, 640}, {512.0, 512.0, 512.0, 320.0}};
  // The region of columns u0 to u1 and rows v0 to v1, edges included, as the
  // camera 1 m from the board sees them, square to it.
  const auto square = [&](double u0, double v0, double u1, double v1) {
    View view;
    view.pinhole = pinhole;
    view.region.depth_to_board[3] = glm::dvec4(0.25, 0.125, -1.0, 1.0);
    view.region.board_to_color = depth_to_color();
    view.region.min =
        Vec2d(0.25 + (u0 - 512.0) / 512.0, 0.125 + (v0 - 320.0) / 512.0);
    view.region.max =
        Vec2d(0.25 + (u1 - 512.0) / 512.0, 0.125 + (v1 - 320.0) / 512.0);
    view.depth.assign(std::size_t{1024} * 640, 1.0f);
    return view;
  };
  // A floor 0.3 m below the camera, the board's y along its view: the
  // region of x in [-0.5, 0.5] and y in [y0, y1] (metres).
  const auto floor = [&](double y0, double y1) {
    View view;
    view.pinhole = pinhole;
    Mat4d m(1.0);
    m[1] = glm::dvec4(0.0, 0.0, -1.0, 0.0);
    m[2] = glm::dvec4(0.0, 1.0, 0.0, 0.0);
    m[3] = glm::dvec4(0.0, 0.0, 0.3, 1.0);
    view.region.depth_to_board = m;
    view.region.board_to_color = depth_to_color();
    view.region.min = Vec2d(-0.5, y0);
    view.region.max = Vec2d(0.5, y1);
    view.depth = render(pinhole, m, [](const Vec3d& o, const Vec3d& d) {
      return hit_plane({Vec3d(0.0), Vec3d(0.0, 0.0, 1.0)}, o, d);
    });
    return view;
  };
  const std::vector<View> views = {square(-20.0, -10.0, 1030.0, 650.0),
                                   square(-50.0, 600.0, 100.0, 700.0),
                                   square(1100.0, 100.0, 1200.0, 200.0),
                                   floor(-0.5, 2.0), floor(-2.0, -0.5)};
  std::vector<core::Buffer> buffers;
  const std::vector<DepthPlane> planes =
      value_of(checks_->fit_planes(regions_of(views, buffers), options));
  ASSERT_EQ(planes.size(), 5u);

  const std::size_t whole = std::size_t{1024} * 640;
  EXPECT_EQ(planes[0].members, whole);
  EXPECT_EQ(planes[0].out_of_range, 0u);
  EXPECT_EQ(planes[0].inliers, whole);
  ASSERT_TRUE(planes[0].valid);
  EXPECT_NEAR(planes[0].point.z, 0.0, 1e-6);
  EXPECT_NEAR(planes[0].normal.z, 1.0, 1e-9);
  EXPECT_EQ(planes[1].members, 101u * 40u);  // columns 0-100, rows 600-639
  EXPECT_EQ(planes[1].inliers, 101u * 40u);
  EXPECT_EQ(planes[2].members, 0u);
  EXPECT_FALSE(planes[2].valid);
  for (const std::size_t r : {3u, 4u}) {
    SCOPED_TRACE("region " + std::to_string(r));
    const Reference want = reference_fit(views[r], options);
    EXPECT_LE(want.outline_band, want.plane.members / 100);
    expect_like(planes[r], want, 0.05, 0.01);
  }
  EXPECT_GT(planes[3].members, 100000u);
  EXPECT_TRUE(planes[3].valid);
  EXPECT_EQ(planes[4].members, 0u);
}

// The solve converges: a plane turned 3.5 degrees about the board's diagonal,
// near the seed's limit (25 of its 30 mm at the corners), seen obliquely, is
// the reference's to 1e-4 degrees. The float solve agrees to about 1e-6
// degrees there; a single Jacobi sweep would leave about 1e-3.
TEST_F(PlaneFitTest, ConvergesOnAPlaneTurnedNearTheSeedsLimit) {
  const DepthCheckOptions options;
  const Plane truth = plane_off(1.0, {1.0, 1.0, 0.0}, 3.5);
  const View view = view_of(
      nfov(), {0.0, 0.0, 1.5}, 40.0, 45.0,
      [&](const Vec3d& o, const Vec3d& d) { return hit_plane(truth, o, d); });
  std::vector<core::Buffer> buffers;
  const std::vector<DepthPlane> planes =
      value_of(checks_->fit_planes(regions_of({view}, buffers), options));
  const Reference want = reference_fit(view, options);
  ASSERT_TRUE(want.plane.valid);
  EXPECT_EQ(want.plane.inliers, want.plane.members);
  expect_like(planes[0], want, 0.001, 1e-4);
}

// The plane of box_view.
Plane box_plane() { return plane_off(0.0, {0.2, 1.0, 0.0}, 0.5); }

// box_plane with a box 15 mm high standing on it, about 4 % of the region,
// and one 60 mm high beside it.
View box_view() {
  const Plane truth = box_plane();
  const Vec3d lo(0.30, 0.15, -0.015);
  const Vec3d hi(0.42, 0.25, 0.0);
  const Vec3d tall_lo(0.08, 0.06, -0.06);
  const Vec3d tall_hi(0.16, 0.14, 0.0);
  return view_of(nfov(), {0.02, -0.03, 1.6}, 10.0, 0.0,
                 [&](const Vec3d& o, const Vec3d& d) {
                   double nearest = hit_plane(truth, o, d);
                   for (const double box : {hit_box(lo, hi, o, d),
                                            hit_box(tall_lo, tall_hi, o, d)}) {
                     if (box > 0.0 && (nearest == 0.0 || box < nearest))
                       nearest = box;
                   }
                   return nearest;
                 });
}

// The box 15 mm high: the first round takes its top and is pulled off the
// plane, and the second, gated at three times the first's RMS, leaves it out.
// The box 60 mm high is past the first round's seed.
TEST_F(PlaneFitTest, TheSecondRoundRemovesABoxOnThePlane) {
  const DepthCheckOptions options;
  const Plane truth = box_plane();
  const View view = box_view();
  std::vector<core::Buffer> buffers;
  const std::vector<DepthPlane> planes =
      value_of(checks_->fit_planes(regions_of({view}, buffers), options));
  const Reference want = reference_fit(view, options);
  EXPECT_LT(want.outline_band, want.plane.members / 100);
  ASSERT_TRUE(want.first.valid);
  // The first round took the box, and was pulled off the plane by it.
  EXPECT_GT(want.first.inliers, want.plane.inliers + 300);
  const double truth_mm = offset_at(truth.point, truth.normal, board_centre());
  EXPECT_GT(std::abs(offset_at(want.first, board_centre()) - truth_mm), 0.2);
  expect_like(planes[0], want, 0.05, 0.01);
  EXPECT_NEAR(offset_at(planes[0], board_centre()), truth_mm, 0.05);
  EXPECT_LE(compute::angle_deg(planes[0].normal, truth.normal), 0.01);
}

// Where a view's outliers lie, each 10 to 25 mm off its plane: none; a fifth
// of the pixels, spread evenly, alternately either side; or every pixel whose
// ray meets the board in the region's first fifth along its x, toward the
// camera, as fingers on the board's edge.
enum class Outliers { None, BothSides, OneEdge };

View outlier_view(const Pinhole& p, const Vec3d& centre, double tilt,
                  double roll, const Plane& truth, Outliers outliers) {
  View view = view_of(
      p, centre, tilt, roll,
      [&](const Vec3d& o, const Vec3d& d) { return hit_plane(truth, o, d); });
  if (outliers == Outliers::None) return view;
  const Vec3d origin(view.region.depth_to_board[3]);
  const Mat3d turn(view.region.depth_to_board);
  const double edge =
      view.region.min.x + 0.2 * (view.region.max.x - view.region.min.x);
  for (std::uint32_t v = 0; v < p.size.height; ++v) {
    for (std::uint32_t u = 0; u < p.size.width; ++u) {
      const Vec3d d = turn * ray_of(p, u, v);
      double sign = -1.0;  // toward the camera
      if (outliers == Outliers::BothSides) {
        if ((u + 2 * v) % 5 != 0) continue;
        sign = ((u / 5 + v) % 2 == 0) ? 1.0 : -1.0;
      } else if (!(origin.x - origin.z / d.z * d.x < edge)) {
        continue;
      }
      const std::size_t i = std::size_t{v} * p.size.width + u;
      const std::uint32_t hash =
          static_cast<std::uint32_t>(i) * 2654435761u;  // Knuth's
      const double off_m = (10.0 + 15.0 * (hash >> 8) / 16777216.0) / 1000.0;
      // Along the ray, to `off_m` from the plane.
      view.depth[i] = static_cast<float>(
          view.depth[i] + sign * off_m / glm::dot(truth.normal, d));
    }
  }
  return view;
}

// The outlier tests' plane, and their view of it.
Plane outlier_plane() { return plane_off(4.0, {0.3, 1.0, 0.0}, 0.8); }
View outlier_view(Outliers outliers) {
  return outlier_view(nfov(), {-0.03, 0.02, 1.8}, -15.0, 90.0, outlier_plane(),
                      outliers);
}

// 20 % outliers 10 to 25 mm off the plane, on either side, move the fit less
// than 0.2 mm from the fit of the same depth without them. That bound holds by
// symmetry, the first round's plane alone within it; what the second round
// does is held to the reference, which it moves by more than expect_like
// allows.
TEST_F(PlaneFitTest, TwentyPercentOutliersMoveThePlaneLittle) {
  const DepthCheckOptions options;
  const View clean = outlier_view(Outliers::None);
  const View noisy = outlier_view(Outliers::BothSides);
  std::vector<core::Buffer> buffers;
  const std::vector<DepthPlane> planes = value_of(
      checks_->fit_planes(regions_of({clean, noisy}, buffers), options));
  ASSERT_TRUE(planes[0].valid);
  ASSERT_TRUE(planes[1].valid);
  const Vec2d centre = board_centre();
  EXPECT_NEAR(offset_at(planes[1], centre), offset_at(planes[0], centre), 0.2);
  const Reference want = reference_fit(noisy, options);
  EXPECT_LT(want.outline_band, want.plane.members / 100);
  EXPECT_LT(want.gate_band, 20u);
  const std::size_t outliers =
      want.plane.members - want.plane.out_of_range - want.plane.inliers;
  EXPECT_GT(outliers, 0u);
  EXPECT_GT(want.first.inliers, want.plane.inliers);
  EXPECT_TRUE(std::abs(offset_at(want.first, centre) -
                       offset_at(want.plane, centre)) > 0.05 ||
              compute::angle_deg(want.first.normal, want.plane.normal) > 0.01);
  expect_like(planes[1], want, 0.05, 0.01);
}

// The known limit, depth_checks.hpp's TODO: outliers 10 to 25 mm in front of
// the region's first fifth, as fingers on the board's edge, raise the first
// round's RMS so far that the second keeps them all, and the plane moves by
// their share of their mean offset, about 3.5 mm, and turns past the default
// max_angle_deg. Pinned, so a second gate robust to them changes this test on
// purpose.
TEST_F(PlaneFitTest, OutliersOnOneSideMoveThePlaneByTheirShare) {
  const DepthCheckOptions options;
  const View clean = outlier_view(Outliers::None);
  const View fingers = outlier_view(Outliers::OneEdge);
  std::vector<core::Buffer> buffers;
  const std::vector<DepthPlane> planes = value_of(
      checks_->fit_planes(regions_of({clean, fingers}, buffers), options));
  ASSERT_TRUE(planes[0].valid);
  const Reference want = reference_fit(fingers, options);
  EXPECT_LT(want.outline_band, want.plane.members / 100);
  EXPECT_LT(want.gate_band, 20u);
  ASSERT_TRUE(want.first.valid);
  // Every point in range, the outliers among them, in both rounds.
  EXPECT_EQ(want.first.inliers, want.plane.members - want.plane.out_of_range);
  EXPECT_EQ(want.plane.inliers, want.first.inliers);
  expect_like(planes[1], want, 0.05, 0.01);
  const Vec2d centre = board_centre();
  const double moved =
      offset_at(planes[1], centre) - offset_at(planes[0], centre);
  EXPECT_GT(moved, -4.0);
  EXPECT_LT(moved, -3.0);
  EXPECT_GT(compute::angle_deg(planes[1].normal, planes[0].normal),
            options.max_angle_deg);
}

// The fit takes its settings from the options it is given, as the reference
// does: a seed of 12 mm takes fewer outliers into the first round; the second
// round's gate is min_gate_mm, 20, where sigma_gate, 1.5, times the first
// round's RMS is below it, and that product where min_gate_mm, 2, is. On the
// outliers either side, each moves the inliers hundreds off the defaults'.
TEST_F(PlaneFitTest, TakesItsSettingsFromTheOptions) {
  const std::vector<View> views = {outlier_view(Outliers::OneEdge),
                                   outlier_view(Outliers::BothSides)};
  std::vector<core::Buffer> buffers;
  const std::vector<PlaneRegion> regions = regions_of(views, buffers);
  DepthCheckOptions seed;
  seed.seed_mm = 12.0;
  DepthCheckOptions floor;
  floor.min_gate_mm = 20.0;
  floor.sigma_gate = 1.5;
  DepthCheckOptions scale = floor;
  scale.min_gate_mm = 2.0;
  const Reference usual = reference_fit(views[1], {});
  ASSERT_TRUE(usual.first.valid);
  const double rms_mm = 1000.0 * usual.first.rms_m;
  EXPECT_LT(1.5 * rms_mm, floor.min_gate_mm);
  EXPECT_GT(3.0 * rms_mm, floor.min_gate_mm);
  EXPECT_GT(1.5 * rms_mm, scale.min_gate_mm);
  for (const DepthCheckOptions& options : {seed, floor, scale}) {
    SCOPED_TRACE("seed " + std::to_string(options.seed_mm) + " mm, gate " +
                 std::to_string(options.min_gate_mm) + " mm, sigma " +
                 std::to_string(options.sigma_gate));
    const std::vector<DepthPlane> planes =
        value_of(checks_->fit_planes(regions, options));
    for (std::size_t r = 0; r < views.size(); ++r) {
      const Reference want = reference_fit(views[r], options);
      ASSERT_TRUE(want.plane.valid);
      expect_like(planes[r], want, 0.05, 0.01);
      if (r == 1) {
        EXPECT_GT(apart(want.plane.inliers, usual.plane.inliers), 500u);
      }
    }
  }
}

// The same depth twice, 640 x 576 with 4 regions, gives the same planes to
// the bit.
TEST_F(PlaneFitTest, TwoRunsAgreeToTheBit) {
  std::vector<View> views;
  for (int c = 0; c < 4; ++c) {
    const double t = c;
    views.push_back(outlier_view(nfov(), {0.03 * t - 0.05, 0.02, 1.5 + 0.3 * t},
                                 20.0 - 12.0 * t, c % 2 == 0 ? 0.0 : 90.0,
                                 plane_off(3.0 - t, {1.0, t, 0.0}, 0.3 * t),
                                 Outliers::BothSides));
  }
  std::vector<core::Buffer> buffers;
  const std::vector<PlaneRegion> regions = regions_of(views, buffers);
  const std::vector<DepthPlane> a = value_of(checks_->fit_planes(regions, {}));
  const std::vector<DepthPlane> b = value_of(checks_->fit_planes(regions, {}));
  ASSERT_EQ(a.size(), 4u);
  ASSERT_EQ(b.size(), 4u);
  for (std::size_t r = 0; r < 4; ++r) {
    SCOPED_TRACE("region " + std::to_string(r));
    ASSERT_TRUE(a[r].valid);
    EXPECT_EQ(a[r].valid, b[r].valid);
    EXPECT_EQ(a[r].members, b[r].members);
    EXPECT_EQ(a[r].out_of_range, b[r].out_of_range);
    EXPECT_EQ(a[r].inliers, b[r].inliers);
    EXPECT_EQ(std::memcmp(&a[r].point, &b[r].point, sizeof(Vec3d)), 0);
    EXPECT_EQ(std::memcmp(&a[r].normal, &b[r].normal, sizeof(Vec3d)), 0);
    EXPECT_EQ(std::memcmp(&a[r].rms_m, &b[r].rms_m, sizeof(double)), 0);
  }
}

// Fewer inliers than min_inliers fit no plane, and the second round does not
// run, so the plane keeps the first's counts; inliers on one line fit none
// however many.
TEST_F(PlaneFitTest, TooFewInliersOrOneLineFitNoPlane) {
  // Depth on a 10 x 10 block of the region alone.
  View block = dyadic_view(0.0f);
  for (std::uint32_t v = 400; v < 410; ++v) {
    for (std::uint32_t u = 200; u < 210; ++u) {
      block.depth[v * kDyadicWidth + u] = 1.0f;
    }
  }
  // Depth on one row of it alone.
  View row = dyadic_view(0.0f);
  for (std::uint32_t u = 0; u < kDyadicWidth; ++u) {
    row.depth[450 * kDyadicWidth + u] = 1.0f;
  }
  // And on one row of a turned board's, which float rounding leaves a sliver
  // of spread off its line.
  const Plane board = plane_off(2.0, {1.0, 0.5, 0.0}, 0.7);
  View turned = view_of(
      nfov(), {0.01, 0.02, 1.6}, 20.0, 0.0,
      [&](const Vec3d& o, const Vec3d& d) { return hit_plane(board, o, d); });
  const std::size_t width = turned.pinhole.size.width;
  for (std::size_t i = 0; i < turned.depth.size(); ++i) {
    if (i / width != 330) turned.depth[i] = 0.0f;
  }
  std::vector<core::Buffer> buffers;
  const std::vector<PlaneRegion> regions =
      regions_of({block, row, turned}, buffers);

  DepthCheckOptions options;
  options.min_inliers = 100;
  std::vector<DepthPlane> planes =
      value_of(checks_->fit_planes(regions, options));
  EXPECT_TRUE(planes[0].valid);
  EXPECT_EQ(planes[0].inliers, 100u);
  options.min_inliers = 101;
  planes = value_of(checks_->fit_planes(regions, options));
  const DepthPlane& few = planes[0];
  EXPECT_FALSE(few.valid);
  EXPECT_EQ(few.members, kDyadicMembers);
  EXPECT_EQ(few.out_of_range, kDyadicMembers - 100);
  EXPECT_EQ(few.inliers, 100u);
  EXPECT_TRUE(std::isnan(few.point.x));
  EXPECT_TRUE(std::isnan(few.normal.z));
  EXPECT_TRUE(std::isnan(few.rms_m));

  options.min_inliers = 3;
  planes = value_of(checks_->fit_planes(regions, options));
  EXPECT_TRUE(planes[0].valid);
  const DepthPlane& line = planes[1];
  EXPECT_FALSE(line.valid);
  EXPECT_EQ(line.members, kDyadicMembers);
  EXPECT_EQ(line.inliers, 301u);
  const Reference sliver = reference_fit(turned, options);
  EXPECT_FALSE(sliver.plane.valid);
  EXPECT_GT(sliver.plane.inliers, 150u);
  EXPECT_FALSE(planes[2].valid);
  EXPECT_LE(apart(planes[2].inliers, sliver.plane.inliers),
            sliver.outline_band + sliver.gate_band);
}

// Every refusal names the region, or the setting, and changes nothing: the
// next call fits as before. No regions fit nothing.
TEST_F(PlaneFitTest, RefusesWhatItCannotRead) {
  const View view = dyadic_view(1.0f);
  const core::Buffer buffer = upload(view.depth);
  const PlaneRegion good = region_of(view, buffer);
  const auto fits = [&] {
    const std::vector<DepthPlane> planes =
        value_of(checks_->fit_planes({good}, {}));
    ASSERT_EQ(planes.size(), 1u);
    EXPECT_TRUE(planes[0].valid);
    EXPECT_EQ(planes[0].inliers, kDyadicMembers);
  };
  const auto refused = [&](const PlaneRegion& bad, const std::string& says) {
    core::Result<std::vector<DepthPlane>> planes = std::vector<DepthPlane>{};
    EXPECT_NO_THROW(planes = checks_->fit_planes({good, bad}, {}));
    expect_refused(planes.status(), "fit_planes", "region 1");
    expect_refused(planes.status(), "fit_planes", says);
    fits();
  };

  PlaneRegion bad = good;
  bad.image.depth = nullptr;
  refused(bad, "no depth buffer");
  bad = good;
  bad.image.size.width = 0;
  refused(bad, "zero side");
  bad = good;
  bad.image.intrinsics.fx = 0.0;
  refused(bad, "focal lengths");
  bad = good;
  bad.image.intrinsics.cy = std::numeric_limits<double>::quiet_NaN();
  refused(bad, "principal point");
  for (const auto& [lo, hi] :
       {std::pair{2.0, 1.0}, std::pair{1.0, 1.0}, std::pair{-0.1, 1.0},
        std::pair{std::numeric_limits<double>::quiet_NaN(), 1.0},
        std::pair{0.5, std::numeric_limits<double>::infinity()}}) {
    bad = good;
    bad.image.min_depth_m = lo;
    bad.image.max_depth_m = hi;
    refused(bad, "range");
  }
  bad = good;
  bad.image.depth_to_world[0] *= 1.01;
  refused(bad, "region 1's depth_to_world");
  bad = good;
  bad.image.depth_to_color[3].x = std::numeric_limits<double>::quiet_NaN();
  refused(bad, "region 1's depth_to_color");
  bad = good;
  bad.region.max.x = bad.region.min.x;
  refused(bad, "rectangle");
  bad = good;
  bad.region.depth_to_board[2] *= 2.0;
  refused(bad, "depth_to_board");
  // A depth one row short of its size.
  bad = good;
  bad.image.size.height += 1;
  refused(bad, "the binding reads");
  // More than one binding may cover.
  bad = good;
  bad.image.size = {65536, 65536};
  refused(bad, "maxStorageBufferRange");
  // Not a storage buffer.
  core::BufferDesc transfer;
  transfer.size = view.depth.size() * sizeof(float);
  transfer.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  const core::Buffer unbindable = value_of(allocator().create_buffer(transfer));
  bad = good;
  bad.image.depth = &unbindable;
  refused(bad, "region 1's depth is not a storage buffer");

  DepthCheckOptions seed;
  seed.seed_mm = 0.0;
  expect_refused(checks_->fit_planes({good}, seed).status(), "fit_planes",
                 "options.seed_mm");
  expect_refused(checks_->fit_planes({}, seed).status(), "fit_planes",
                 "options.seed_mm");
  DepthCheckOptions few;
  few.min_inliers = 2;
  expect_refused(checks_->fit_planes({good}, few).status(), "fit_planes",
                 "options.min_inliers");

  // More regions than one dispatch of the solve can take, refused before
  // any is read.
  const std::uint32_t most =
      device().caps().limits().maxComputeWorkGroupCount[0];
  if (most < (1u << 20)) {
    const std::vector<PlaneRegion> many(std::size_t{most} + 1);
    expect_refused(checks_->fit_planes(many, {}).status(), "fit_planes",
                   "more than one dispatch can solve");
  }

  const core::Result<std::vector<DepthPlane>> none =
      checks_->fit_planes({}, {});
  ASSERT_TRUE(none.ok()) << none.status().message();
  EXPECT_TRUE(none->empty());
  fits();
}

}  // namespace
}  // namespace volumetric_kit::calib::depth
