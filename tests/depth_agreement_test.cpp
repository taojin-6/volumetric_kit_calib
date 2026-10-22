// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The depth checks' pair agreement on a device, against references in double
// made here: rigs of 1, 3, 4 and 5 cameras of mixed sizes posed about a plane
// with a box on it, every ordered pair within a bin of 0 and as the reference
// has it; a camera posed 3 mm off; the window and the edge gate over a box
// with flying pixels round it, counted exactly; the edge gate's scale with
// depth; each camera's own range; histograms accumulated in either order and
// as one capture, to the bit, and each capture's own in the same pass; the
// percentiles of known histograms, and of
// counts past float's 24 bits; empty slots; the stride; a pair with nothing
// in its window; a capture that could overflow a count; and refusals. Skips
// where no device is present.
//
// Most tests pose every camera alike (same_pose): each sample then meets its
// own pixel, and its difference is the two images' difference there, exact in
// float, so the histograms are known. Elsewhere the device works in float and
// the reference in double, so a sample one of whose decisions lies within the
// float's error of its threshold -- the image's edge, a pixel's half, the edge
// gate, the window's edge -- may be decided either way: counts may differ by
// those alone, and a statistic by those and the samples within that error of
// a bin edge beside it.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include "depth_fixture.hpp"
#include "depth_scene.hpp"
#include "volumetric_kit/calib/depth/depth_checks.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"

namespace volumetric_kit::calib::depth {
namespace {

namespace camera = recon::camera;
using camera::Mat3d;
using camera::Mat4d;
using camera::Vec3d;
using quality::DepthCheckOptions;
using quality::PairAgreement;
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
using test::ray_of;
using test::render;
using test::rotation;
using test::Surface;
using test::value_of;
using test::wide;

// How near a threshold float and double may decide apart: in pixels (the
// image's edge, a pixel's half), metres (the edge gate) and bins.
constexpr double kPixelBand = 2e-4;
constexpr double kDepthBand = 1e-5;
constexpr double kBinBand = 0.02;

std::size_t apart(std::size_t a, std::size_t b) {
  return a > b ? a - b : b - a;
}

// A camera of a rig: its depth's pinhole, its depth, where its depth camera
// is in the scene, and its depth's range.
struct Camera {
  Pinhole pinhole;
  std::vector<float> depth;
  Mat4d depth_to_world{1.0};
  double min_depth = kMinDepth;
  double max_depth = kMaxDepth;
};

// Whether `cam` measured depth `z`, by the kernels' rule.
bool measured_by(const Camera& cam, double z) {
  return measured(z, cam.min_depth, cam.max_depth);
}

// A depth camera at `eye` looking at `target`, its x axis square to the
// scene's y (down), then turned `roll_deg` about its axis.
Mat4d looking_at(const Vec3d& eye, const Vec3d& target, double roll_deg) {
  const Vec3d z = glm::normalize(target - eye);
  const Vec3d x = glm::normalize(glm::cross(Vec3d(0.0, 1.0, 0.0), z));
  Mat4d m(Mat3d(x, glm::cross(z, x), z) * rotation({0.0, 0.0, 1.0}, roll_deg));
  m[3] = glm::dvec4(eye, 1.0);
  return m;
}

// The scene's plane, z = 0; the cameras are on its negative side.
double plane(const Vec3d& origin, const Vec3d& direction) {
  return hit_plane({{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}, origin, direction);
}

// The plane with a box 15 cm high standing on it, toward the cameras: its
// edges step the depth by more than the edge gate, and its top lies past the
// window from the plane.
double plane_and_box(const Vec3d& origin, const Vec3d& direction) {
  const double floor = plane(origin, direction);
  const double box =
      hit_box({-0.15, -0.1, -0.15}, {0.1, 0.12, 0.0}, origin, direction);
  return box > 0.0 && (floor == 0.0 || box < floor) ? box : floor;
}

Camera posed(const Pinhole& pinhole, const Mat4d& depth_to_world,
             const Surface& surface) {
  return {pinhole, render(pinhole, depth_to_world, surface), depth_to_world};
}

// Camera c of a rig of `count` round the scene, about 2 m off and 15 to 25
// degrees oblique to the plane, each with its own pinhole, pose and roll.
Camera rig_camera(std::size_t c, std::size_t count, const Surface& surface) {
  const Pinhole pinholes[] = {nfov(), binned(), nfov(), wide(), nfov()};
  const double rolls[] = {0.0, 90.0, 12.0, -20.0, 35.0};
  const double t = static_cast<double>(c);
  const double a =
      2.0 * glm::pi<double>() * t / static_cast<double>(count) + 0.4;
  const Vec3d eye(0.45 + 0.4 * std::cos(a), -0.3 + 0.3 * std::sin(a),
                  -1.9 - 0.1 * t);
  const Vec3d target(0.03 * t - 0.05, 0.02 * t - 0.03, 0.0);
  return posed(pinholes[c], looking_at(eye, target, rolls[c]), surface);
}

// A `width` x `height` depth camera at the scene's origin, its depth `depth`
// but for row and column 0, which have none: a sample of either lies on the
// image's edge, where the device's rounding -- its division need not be
// correctly rounded -- may carry it off. Every other pixel lies well inside.
constexpr std::uint32_t kWidth = 64;
constexpr std::uint32_t kHeight = 48;
// Its pixels with depth, at 64 x 48.
constexpr std::size_t kInterior = std::size_t{kWidth - 1} * (kHeight - 1);
Camera same_pose(std::vector<float> depth, std::uint32_t height = kHeight,
                 std::uint32_t width = kWidth) {
  Camera cam;
  cam.pinhole = {{width, height}, {64.0, 64.0, 32.0, 16.0}};
  cam.depth = std::move(depth);
  if (cam.depth.empty()) return cam;
  for (std::uint32_t v = 0; v < height; ++v) {
    for (std::uint32_t u = 0; u < width; ++u) {
      if (u == 0 || v == 0) cam.depth[std::size_t{v} * width + u] = 0.0f;
    }
  }
  return cam;
}

// Whether pixel (u, v) of `cam`, of measured depth `z`, is at an edge by
// `ratio`; sets `near` when a neighbour's step lies within kDepthBand of the
// limit.
bool at_edge(const Camera& cam, std::uint32_t u, std::uint32_t v, double z,
             double ratio, bool& near) {
  const std::uint32_t w = cam.pinhole.size.width;
  const std::uint32_t h = cam.pinhole.size.height;
  const double limit = ratio * z;
  bool edge = false;
  const auto step = [&](std::uint32_t x, std::uint32_t y) {
    const double n = cam.depth[std::size_t{y} * w + x];
    if (!measured_by(cam, n)) return;
    const double d = std::abs(n - z);
    if (std::abs(d - limit) < kDepthBand) near = true;
    if (d > limit) edge = true;
  };
  if (u > 0) step(u - 1, v);
  if (u + 1 < w) step(u + 1, v);
  if (v > 0) step(u, v - 1);
  if (v + 1 < h) step(u, v + 1);
  return edge;
}

// One pair's agreement in double over the depth the device reads, and the
// samples float may decide otherwise.
struct Reference {
  std::size_t overlap = 0;
  std::size_t in_window = 0;
  std::vector<std::size_t> bins;  ///< The window's, from the lowest.
  /// The samples float may count otherwise: in the overlap, or the window.
  std::size_t band = 0;
  /// At each bin edge, from the window's lowest, the samples float may bin
  /// on its other side.
  std::vector<std::size_t> edges;
};

long half_bins(const DepthCheckOptions& options) {
  return std::lround(options.window_mm / options.bin_mm);
}

Reference reference_pair(const Camera& from, const Camera& to,
                         const DepthCheckOptions& options) {
  const Mat4d to_from =
      camera::rigid_inverse(to.depth_to_world) * from.depth_to_world;
  const double bin = options.bin_mm / 1000.0;
  const long half = half_bins(options);
  const Pinhole& p = from.pinhole;
  const Pinhole& t = to.pinhole;
  const double w = t.size.width;
  const double h = t.size.height;
  const auto close = [](double a, double b) {
    return std::abs(a - b) < kPixelBand;
  };
  Reference out;
  out.bins.assign(static_cast<std::size_t>(2 * half), 0);
  out.edges.assign(static_cast<std::size_t>(2 * half + 1), 0);
  for (std::uint32_t v = 0; v < p.size.height; v += options.agreement_stride) {
    for (std::uint32_t u = 0; u < p.size.width; u += options.agreement_stride) {
      bool near = false;
      const double z = from.depth[std::size_t{v} * p.size.width + u];
      if (!measured_by(from, z) ||
          at_edge(from, u, v, z, options.edge_ratio, near)) {
        out.band += near ? 1 : 0;
        continue;
      }
      const Vec3d q(to_from * glm::dvec4(z * ray_of(p, u, v), 1.0));
      if (!(q.z > 0.0)) continue;
      const double x = t.k.fx * q.x / q.z + t.k.cx;
      const double y = t.k.fy * q.y / q.z + t.k.cy;
      near = near || close(x, 0.0) || close(x, w) || close(y, 0.0) ||
             close(y, h) || close(x - std::floor(x), 0.5) ||
             close(y - std::floor(y), 0.5);
      if (!(x >= 0.0 && x < w && y >= 0.0 && y < h)) {
        out.band += near ? 1 : 0;
        continue;
      }
      const auto tu = std::min(static_cast<std::uint32_t>(std::floor(x + 0.5)),
                               t.size.width - 1);
      const auto tv = std::min(static_cast<std::uint32_t>(std::floor(y + 0.5)),
                               t.size.height - 1);
      const double there = to.depth[std::size_t{tv} * t.size.width + tu];
      if (!measured_by(to, there) ||
          at_edge(to, tu, tv, there, options.edge_ratio, near)) {
        out.band += near ? 1 : 0;
        continue;
      }
      ++out.overlap;
      const double at = (q.z - there) / bin;
      const double edge = std::round(at);
      if (std::abs(at - edge) < kBinBand && std::abs(edge) <= half) {
        // Within the window's edge, float may count it otherwise.
        near = near || std::abs(edge) == half;
        ++out.edges[static_cast<std::size_t>(edge + static_cast<double>(half))];
      }
      out.band += near ? 1 : 0;
      const double k = std::floor(at);
      if (k >= static_cast<double>(-half) && k < static_cast<double>(half)) {
        ++out.in_window;
        ++out.bins[static_cast<std::size_t>(k + static_cast<double>(half))];
      }
    }
  }
  return out;
}

// A quantile by the device's rule, in bins, the bin it lies in, and that
// bin's count.
struct Quantile {
  double at = 0.0;
  std::size_t bin = 0;
  std::size_t count = 0;
};

// The q-quantile of `bins`, in bins from bin `origin`'s lower edge: in the
// first bin where the running count reaches q n, as far into it as the rest
// of q n takes of its count.
Quantile quantile(const std::vector<std::size_t>& bins, double q, long origin) {
  std::size_t n = 0;
  for (const std::size_t c : bins) n += c;
  const double target = q * static_cast<double>(n);
  std::size_t below = 0;
  for (std::size_t b = 0; b < bins.size(); ++b) {
    const std::size_t c = bins[b];
    if (c != 0 && static_cast<double>(below + c) >= target) {
      return {
          static_cast<double>(static_cast<long>(b) - origin) +
              (target - static_cast<double>(below)) / static_cast<double>(c),
          b, c};
    }
    below += c;
  }
  return {std::numeric_limits<double>::quiet_NaN(), 0, 0};
}

// `bins` folded at 0: the magnitudes' histogram.
std::vector<std::size_t> folded(const std::vector<std::size_t>& bins) {
  const std::size_t half = bins.size() / 2;
  std::vector<std::size_t> out(half);
  for (std::size_t j = 0; j < half; ++j) {
    out[j] = bins[half + j] + bins[half - 1 - j];
  }
  return out;
}

// The device's pair against the reference's: the counts equal but for the
// band, and each statistic within what the band's samples, and those that
// float may bin across its bin's edges, could move it: each a bin over its
// bin's count.
void expect_like(const PairAgreement& got, const Reference& want,
                 const DepthCheckOptions& options) {
  EXPECT_LE(apart(got.overlap, want.overlap), want.band);
  EXPECT_LE(apart(got.in_window, want.in_window), want.band);
  ASSERT_GT(want.in_window, 0u);
  const std::size_t half = want.bins.size() / 2;
  // `edges` at a bin's two edges, each counted from the window's lowest.
  const auto near = [&](double got_mm, const Quantile& q,
                        const std::vector<std::size_t>& at_edges) {
    const double moved =
        static_cast<double>(want.band + at_edges[q.bin] + at_edges[q.bin + 1]);
    const double slack =
        (moved / static_cast<double>(q.count) + 1e-3) * options.bin_mm;
    EXPECT_NEAR(got_mm, q.at * options.bin_mm, slack);
  };
  near(got.signed_median_mm, quantile(want.bins, 0.5, half_bins(options)),
       want.edges);
  // Folded, edge j is the edges at half + j and half - j.
  std::vector<std::size_t> folded_edges(half + 1);
  for (std::size_t j = 0; j <= half; ++j) {
    folded_edges[j] = want.edges[half + j] + (j > 0 ? want.edges[half - j] : 0);
  }
  const std::vector<std::size_t> magnitudes = folded(want.bins);
  near(got.median_abs_mm, quantile(magnitudes, 0.5, 0), folded_edges);
  near(got.p90_abs_mm, quantile(magnitudes, 0.9, 0), folded_edges);
}

// The statistics of known bins, against the device's: exactly the counts,
// and the quantiles to float's rounding.
void expect_stats(const PairAgreement& got,
                  const std::vector<std::size_t>& bins,
                  const DepthCheckOptions& options) {
  std::size_t n = 0;
  for (const std::size_t c : bins) n += c;
  EXPECT_EQ(got.in_window, n);
  const double slack = 5e-4 * options.bin_mm;
  EXPECT_NEAR(got.signed_median_mm,
              quantile(bins, 0.5, half_bins(options)).at * options.bin_mm,
              slack);
  const std::vector<std::size_t> magnitudes = folded(bins);
  EXPECT_NEAR(got.median_abs_mm,
              quantile(magnitudes, 0.5, 0).at * options.bin_mm, slack);
  EXPECT_NEAR(got.p90_abs_mm, quantile(magnitudes, 0.9, 0).at * options.bin_mm,
              slack);
}

// Every field of two pairs alike, to the bit.
void expect_same(const PairAgreement& a, const PairAgreement& b) {
  EXPECT_EQ(a.from, b.from);
  EXPECT_EQ(a.to, b.to);
  EXPECT_EQ(a.capture, b.capture);
  EXPECT_EQ(a.overlap, b.overlap);
  EXPECT_EQ(a.in_window, b.in_window);
  EXPECT_EQ(a.signed_median_mm, b.signed_median_mm);
  EXPECT_EQ(a.median_abs_mm, b.median_abs_mm);
  EXPECT_EQ(a.p90_abs_mm, b.p90_abs_mm);
}

// A pair with nothing in its window, so no statistics.
void expect_no_statistics(const PairAgreement& pair) {
  EXPECT_EQ(pair.in_window, 0u);
  EXPECT_TRUE(std::isnan(pair.signed_median_mm));
  EXPECT_TRUE(std::isnan(pair.median_abs_mm));
  EXPECT_TRUE(std::isnan(pair.p90_abs_mm));
}

// A pair with no samples.
void expect_unmeasured(const PairAgreement& pair) {
  EXPECT_EQ(pair.overlap, 0u);
  expect_no_statistics(pair);
}

// Smooth depth about 2 m, within 2 cm of it, so no step reaches the edge
// gate: camera c's in capture k.
std::vector<float> smooth(std::size_t c, std::size_t k,
                          std::uint32_t height = kHeight) {
  std::vector<float> depth(std::size_t{kWidth} * height);
  const double cc = static_cast<double>(c);
  const double kk = static_cast<double>(k);
  for (std::uint32_t v = 0; v < height; ++v) {
    for (std::uint32_t u = 0; u < kWidth; ++u) {
      depth[std::size_t{v} * kWidth + u] =
          static_cast<float>(2.0 + 0.004 * cc - 0.003 * kk +
                             0.012 * std::sin(0.37 * u + 1.3 * cc + 0.7 * kk) *
                                 std::cos(0.23 * v - cc));
    }
  }
  return depth;
}

class AgreementTest : public test::DepthChecksTest {
 protected:
  static DepthImage image_of(const Camera& cam, const core::Buffer& buffer) {
    DepthImage image;
    image.depth = &buffer;
    image.intrinsics = cam.pinhole.k;
    image.size = cam.pinhole.size;
    image.min_depth_m = cam.min_depth;
    image.max_depth_m = cam.max_depth;
    image.depth_to_world = cam.depth_to_world;
    return image;
  }

  // Each camera's depth uploaded into `buffers`, as a slot; an empty camera
  // (no depth) leaves its slot empty.
  std::vector<std::optional<DepthImage>> images_of(
      const std::vector<Camera>& cameras, std::vector<core::Buffer>& buffers) {
    buffers.clear();
    buffers.reserve(cameras.size());
    std::vector<std::optional<DepthImage>> images;
    for (const Camera& cam : cameras) {
      if (cam.depth.empty()) {
        images.emplace_back();
        continue;
      }
      buffers.push_back(upload(cam.depth));
      images.emplace_back(image_of(cam, buffers.back()));
    }
    return images;
  }

  // `cameras` as one capture, into new histograms of `options`, summarized.
  std::vector<PairAgreement> agree(const std::vector<Camera>& cameras,
                                   const DepthCheckOptions& options = {}) {
    std::vector<core::Buffer> buffers;
    const std::vector<std::optional<DepthImage>> images =
        images_of(cameras, buffers);
    const std::unique_ptr<AgreementHistograms> histograms =
        value_of(checks_->histograms(cameras.size(), options));
    check(checks_->accumulate(*histograms, images));
    return value_of(checks_->summarize(*histograms));
  }
};

// Each camera posed as it is: every ordered pair of rigs of 1, 3, 4 and 5
// cameras of mixed sizes, rolls and obliquities agrees within a bin of 0,
// and as the reference has it -- the plane's samples in the window, the
// box's occlusions past it. A rig of 1 has no pairs.
TEST_F(AgreementTest, AlignedCamerasAgreeWithinABinOnRigsOfAnySize) {
  const DepthCheckOptions options;
  for (const std::size_t count : {1u, 3u, 4u, 5u}) {
    SCOPED_TRACE(std::to_string(count) + " cameras");
    std::vector<Camera> rig;
    for (std::size_t c = 0; c < count; ++c) {
      rig.push_back(rig_camera(c, count, plane_and_box));
    }
    const std::vector<PairAgreement> pairs = agree(rig, options);
    ASSERT_EQ(pairs.size(), count * (count - 1));
    std::size_t i = 0;
    for (std::size_t from = 0; from < count; ++from) {
      for (std::size_t to = 0; to < count; ++to) {
        if (from == to) continue;
        SCOPED_TRACE("from " + std::to_string(from) + " to " +
                     std::to_string(to));
        const PairAgreement& got = pairs[i++];
        EXPECT_EQ(got.from, from);
        EXPECT_EQ(got.to, to);
        EXPECT_FALSE(got.capture.has_value());
        const Reference want = reference_pair(rig[from], rig[to], options);
        EXPECT_GT(want.in_window, 20000u);
        EXPECT_LT(want.band, want.in_window / 100);
        expect_like(got, want, options);
        EXPECT_LE(std::abs(got.signed_median_mm), options.bin_mm);
      }
    }
  }
}

// Camera 1's pose 3 mm off along the plane's normal, away from the cameras:
// it and camera 0, both square to the plane, see each other 3 mm off, each
// way with its sign, within 0.1 mm; cameras 0 and 2 still agree; and every
// pair is as the reference has it.
TEST_F(AgreementTest, FindsACameraPosedThreeMillimetresOff) {
  const DepthCheckOptions options;
  std::vector<Camera> rig = {
      posed(nfov(), looking_at({0.1, -0.05, -2.0}, {0.1, -0.05, 0.0}, 0.0),
            plane),
      posed(binned(), looking_at({-0.25, 0.1, -1.7}, {-0.25, 0.1, 0.0}, 90.0),
            plane),
      posed(wide(), looking_at({0.6, -0.4, -1.9}, {0.0, 0.0, 0.0}, 15.0),
            plane)};
  rig[1].depth_to_world[3].z += 0.003;
  const std::vector<PairAgreement> pairs = agree(rig, options);
  ASSERT_EQ(pairs.size(), 6u);
  // By from, then to: (0, 1), (0, 2), (1, 0), (1, 2), (2, 0), (2, 1).
  EXPECT_NEAR(pairs[2].signed_median_mm, 3.0, 0.1);
  EXPECT_NEAR(pairs[0].signed_median_mm, -3.0, 0.1);
  EXPECT_LE(std::abs(pairs[1].signed_median_mm), options.bin_mm);
  EXPECT_LE(std::abs(pairs[4].signed_median_mm), options.bin_mm);
  EXPECT_GT(pairs[3].signed_median_mm, 2.5);
  EXPECT_LT(pairs[5].signed_median_mm, -2.5);
  for (const PairAgreement& got : pairs) {
    SCOPED_TRACE("from " + std::to_string(got.from) + " to " +
                 std::to_string(got.to));
    expect_like(got, reference_pair(rig[got.from], rig[got.to], options),
                options);
  }
}

// A box 15 cm in front of a plane at 2 m, in camera 1's depth alone, with a
// ring of flying pixels halfway between: the window takes the plane and
// leaves out the box; the edge gate drops the ring and the pixels either
// side of it, in both directions. Opened, the gate lets the ring into the
// window; widened, the window takes the box. The cameras' own pixels'
// differences are 0, -7.5 cm or -15 cm.
TEST_F(AgreementTest, TheWindowExcludesABoxAndTheEdgeGateItsFlyingPixels) {
  std::vector<float> box(std::size_t{kWidth} * kHeight, 2.0f);
  for (std::uint32_t v = 15; v < 33; ++v) {
    for (std::uint32_t u = 19; u < 45; ++u) {
      const bool inside = v >= 16 && v < 32 && u >= 20 && u < 44;
      box[std::size_t{v} * kWidth + u] = inside ? 1.85f : 1.925f;
    }
  }
  const std::vector<Camera> cameras = {
      same_pose(std::vector<float>(std::size_t{kWidth} * kHeight, 2.0f)),
      same_pose(box)};
  // kInterior pixels with depth: the box's 384, 76 of them on its border,
  // the ring's 84, and 88 of the plane's beside the ring.
  const auto expect_counts = [&](const DepthCheckOptions& options,
                                 std::size_t overlap, std::size_t in_window) {
    const std::vector<PairAgreement> pairs = agree(cameras, options);
    ASSERT_EQ(pairs.size(), 2u);
    for (const PairAgreement& got : pairs) {
      SCOPED_TRACE("from " + std::to_string(got.from));
      EXPECT_EQ(got.overlap, overlap);
      const Reference want =
          reference_pair(cameras[got.from], cameras[got.to], options);
      EXPECT_EQ(want.overlap, overlap);
      EXPECT_EQ(want.in_window, in_window);
      expect_stats(got, want.bins, options);
    }
  };
  DepthCheckOptions options;
  // The plane's ungated pixels in the window, at 0; the box's 308 inside its
  // border past it.
  expect_counts(options, kInterior - 84 - 76 - 88, kInterior - 384 - 84 - 88);
  const std::vector<PairAgreement> gated = agree(cameras, options);
  EXPECT_DOUBLE_EQ(gated[0].signed_median_mm, 0.5 * options.bin_mm);
  EXPECT_DOUBLE_EQ(gated[1].signed_median_mm, 0.5 * options.bin_mm);
  DepthCheckOptions open = options;
  open.edge_ratio = 0.5;
  expect_counts(open, kInterior, kInterior - 384);
  DepthCheckOptions wide_window = options;
  wide_window.window_mm = 200.0;
  expect_counts(wide_window, kInterior - 84 - 76 - 88,
                kInterior - 84 - 76 - 88);
}

// The edge gate is a fraction of the pixel's depth: at the default 3 %, a
// 10 cm step 5 m out is no edge, and every pixel counts; a 4 cm step 1 m out
// is one, and the two columns beside it are dropped in both directions.
TEST_F(AgreementTest, TheEdgeGateScalesWithDepth) {
  struct Case {
    float near;
    float far;
    std::size_t overlap;
  };
  const DepthCheckOptions options;
  for (const Case& c : {Case{5.0f, 5.1f, kInterior},
                        Case{1.0f, 1.04f, kInterior - 2 * (kHeight - 1)}}) {
    SCOPED_TRACE(c.near);
    std::vector<float> step(std::size_t{kWidth} * kHeight, c.near);
    for (std::uint32_t v = 0; v < kHeight; ++v) {
      for (std::uint32_t u = kWidth / 2; u < kWidth; ++u) {
        step[std::size_t{v} * kWidth + u] = c.far;
      }
    }
    const std::vector<Camera> cameras = {same_pose(step), same_pose(step)};
    for (const PairAgreement& got : agree(cameras, options)) {
      SCOPED_TRACE("from " + std::to_string(got.from));
      EXPECT_EQ(got.overlap, c.overlap);
      EXPECT_EQ(got.in_window, c.overlap);
      EXPECT_EQ(
          reference_pair(cameras[got.from], cameras[got.to], options).overlap,
          c.overlap);
    }
  }
}

// Each camera's own range decides which of its depths were measured, where a
// sample starts, where it lands, and among the neighbours an edge is judged
// by. Camera 0 measures [0.25, 2.01] m and camera 1 [1.99, 6] m, posed alike
// over a plane at 2 m. Each holds depths no camera measures -- not a number,
// infinite, negative, below 0.25 m, above 6 m -- and depths only the other
// measures (2.02 m in camera 0, 1.98 m in camera 1, twice, so a pixel judged
// by the wrong camera's range cannot cancel one counted by the right one's),
// which drop their pixel in both directions; depths 2 cm off inside its own
// range (1.98 m in camera 0, 2.02 m in camera 1), which count; and a step
// only the other camera measures (2.6 m in camera 0, 1 m in camera 1), which
// gates no neighbour. (Posed alike, each pixel is a sample one way and the
// pixel met the other, so the two directions count alike.)
TEST_F(AgreementTest, EachCamerasRangeIsItsOwn) {
  constexpr float kInf = std::numeric_limits<float>::infinity();
  std::vector<float> depth[2];
  depth[0].assign(std::size_t{kWidth} * kHeight, 2.0f);
  depth[1] = depth[0];
  // Each at a pixel of its own on a grid 4 apart, so none shares a neighbour.
  std::size_t next = 0;
  const auto put = [&](std::size_t c, float z) {
    const std::size_t u = 4 + 4 * (next % 14);
    const std::size_t v = 4 + 4 * (next / 14);
    ++next;
    depth[c][v * kWidth + u] = z;
  };
  for (const float z : {std::numeric_limits<float>::quiet_NaN(), kInf, -kInf,
                        -2.0f, 0.2f, 7.0f}) {
    put(0, z);
    put(1, z);
  }
  put(0, 2.02f);
  put(0, 2.02f);
  put(1, 1.98f);
  put(1, 1.98f);
  put(0, 1.98f);
  put(1, 2.02f);
  put(0, 2.6f);
  put(1, 1.0f);
  std::vector<Camera> cameras = {same_pose(depth[0]), same_pose(depth[1])};
  cameras[0].max_depth = 2.01;
  cameras[1].min_depth = 1.99;
  // Each camera's 6 unmeasured, its 2 only the other measures, and its step.
  constexpr std::size_t kOverlap = kInterior - 2 * 9;
  const DepthCheckOptions options;
  const std::vector<PairAgreement> pairs = agree(cameras, options);
  ASSERT_EQ(pairs.size(), 2u);
  for (const PairAgreement& got : pairs) {
    SCOPED_TRACE("from " + std::to_string(got.from));
    EXPECT_EQ(got.overlap, kOverlap);
    const Reference want =
        reference_pair(cameras[got.from], cameras[got.to], options);
    EXPECT_EQ(want.overlap, kOverlap);
    EXPECT_EQ(want.in_window, kOverlap);
    expect_stats(got, want.bins, options);
  }
}

// Integer counts add alike in any order: two captures of three cameras,
// accumulated either way round, leave the histograms one capture of both --
// the second below the first, twice the height -- leaves, to the bit; one
// capture alone leaves fewer.
TEST_F(AgreementTest, AccumulatingTwiceIsOneHistogramOfBoth) {
  std::vector<Camera> first;
  std::vector<Camera> second;
  std::vector<Camera> both;
  for (std::size_t c = 0; c < 3; ++c) {
    first.push_back(same_pose(smooth(c, 0)));
    second.push_back(same_pose(smooth(c, 1)));
    std::vector<float> stacked = first.back().depth;
    stacked.insert(stacked.end(), second.back().depth.begin(),
                   second.back().depth.end());
    both.push_back(same_pose(stacked, 2 * kHeight));
  }
  using Images = std::vector<std::optional<DepthImage>>;
  std::vector<core::Buffer> first_buffers;
  std::vector<core::Buffer> second_buffers;
  const Images first_images = images_of(first, first_buffers);
  const Images second_images = images_of(second, second_buffers);
  const auto accumulated = [&](const std::vector<const Images*>& captures) {
    const std::unique_ptr<AgreementHistograms> histograms =
        value_of(checks_->histograms(3));
    for (const auto* images : captures) {
      check(checks_->accumulate(*histograms, *images));
    }
    return value_of(checks_->summarize(*histograms));
  };
  const std::vector<PairAgreement> forward =
      accumulated({&first_images, &second_images});
  const std::vector<PairAgreement> backward =
      accumulated({&second_images, &first_images});
  const std::vector<PairAgreement> once = agree(both);
  const std::vector<PairAgreement> alone = accumulated({&first_images});
  ASSERT_EQ(forward.size(), 6u);
  ASSERT_EQ(backward.size(), 6u);
  ASSERT_EQ(once.size(), 6u);
  for (std::size_t i = 0; i < 6; ++i) {
    SCOPED_TRACE(i);
    expect_same(forward[i], backward[i]);
    expect_same(forward[i], once[i]);
    EXPECT_EQ(forward[i].overlap, 2 * kInterior);
    EXPECT_EQ(alone[i].overlap, kInterior);
    EXPECT_NE(alone[i].signed_median_mm, forward[i].signed_median_mm);
    expect_like(once[i],
                reference_pair(both[once[i].from], both[once[i].to], {}), {});
  }
}

// A capture's own histograms, given to accumulate, are counted in the same
// pass as the pooled ones: each holds the capture alone, to the bit, emptied
// of the one before whether this one measures every pair, some or none, and
// the pooled ones hold every capture as without them. Own histograms that
// are the pooled ones, or differ in cameras or settings, are refused, leaving
// both as they were; a rig of one camera has none to count.
TEST_F(AgreementTest, CountsACaptureAloneInTheSamePass) {
  using Images = std::vector<std::optional<DepthImage>>;
  std::vector<Camera> first;
  std::vector<Camera> second;
  for (std::size_t c = 0; c < 3; ++c) {
    first.push_back(same_pose(smooth(c, 0)));
    second.push_back(same_pose(smooth(c, 1)));
  }
  std::vector<core::Buffer> first_buffers;
  std::vector<core::Buffer> second_buffers;
  const Images a = images_of(first, first_buffers);
  const Images b = images_of(second, second_buffers);
  const Images some = {b[0], {}, b[2]};
  const Images none = {a[0], {}, {}};
  const std::vector<const Images*> captures = {&a, &b, &some, &none};
  const auto alone = [&](const Images& images, std::int64_t capture) {
    const std::unique_ptr<AgreementHistograms> histograms =
        value_of(checks_->histograms(3));
    check(checks_->accumulate(*histograms, images));
    return value_of(checks_->summarize(*histograms, capture));
  };
  // Every pair alike, to the bit; one with nothing in its window has no
  // statistics in either.
  const auto expect_all_same = [](const std::vector<PairAgreement>& got,
                                  const std::vector<PairAgreement>& want) {
    ASSERT_EQ(got.size(), want.size());
    for (std::size_t i = 0; i < got.size(); ++i) {
      SCOPED_TRACE(i);
      if (want[i].in_window > 0) {
        expect_same(got[i], want[i]);
        continue;
      }
      EXPECT_EQ(got[i].capture, want[i].capture);
      EXPECT_EQ(got[i].overlap, want[i].overlap);
      expect_no_statistics(got[i]);
    }
  };

  const std::unique_ptr<AgreementHistograms> pooled =
      value_of(checks_->histograms(3));
  const std::unique_ptr<AgreementHistograms> own =
      value_of(checks_->histograms(3));
  const std::unique_ptr<AgreementHistograms> without =
      value_of(checks_->histograms(3));
  for (std::size_t k = 0; k < captures.size(); ++k) {
    SCOPED_TRACE("capture " + std::to_string(k));
    const auto index = static_cast<std::int64_t>(k);
    check(checks_->accumulate(*pooled, *captures[k], own.get()));
    check(checks_->accumulate(*without, *captures[k]));
    const std::vector<PairAgreement> its =
        value_of(checks_->summarize(*own, index));
    expect_all_same(its, alone(*captures[k], index));
    expect_all_same(value_of(checks_->summarize(*pooled)),
                    value_of(checks_->summarize(*without)));
    if (captures[k] == &none) {
      for (const PairAgreement& pair : its) expect_unmeasured(pair);
    }
  }
  // 0 to 1 in the first two captures, 0 to 2 in the first three.
  const std::vector<PairAgreement> all = value_of(checks_->summarize(*pooled));
  EXPECT_EQ(all[0].overlap, 2 * kInterior);
  EXPECT_EQ(all[1].overlap, 3 * kInterior);

  const std::vector<PairAgreement> pooled_before =
      value_of(checks_->summarize(*pooled));
  const std::vector<PairAgreement> own_before =
      value_of(checks_->summarize(*own));
  const auto refused = [&](AgreementHistograms* bad) {
    expect_refused(checks_->accumulate(*pooled, a, bad), "accumulate",
                   "the capture's own histograms");
    expect_all_same(value_of(checks_->summarize(*pooled)), pooled_before);
    expect_all_same(value_of(checks_->summarize(*own)), own_before);
  };
  refused(pooled.get());
  refused(value_of(checks_->histograms(2)).get());
  DepthCheckOptions coarse;
  coarse.bin_mm = 0.2;
  refused(value_of(checks_->histograms(3, coarse)).get());
  DepthCheckOptions strided;
  strided.agreement_stride = 2;
  refused(value_of(checks_->histograms(3, strided)).get());
  DepthCheckOptions gated;
  gated.edge_ratio = 0.05;
  refused(value_of(checks_->histograms(3, gated)).get());

  const std::unique_ptr<AgreementHistograms> one =
      value_of(checks_->histograms(1));
  const std::unique_ptr<AgreementHistograms> one_own =
      value_of(checks_->histograms(1));
  check(checks_->accumulate(*one, {a[0]}, one_own.get()));
  EXPECT_TRUE(value_of(checks_->summarize(*one_own)).empty());
}

// A histogram known bin by bin, and samples past its window: camera 1's
// depth is camera 0's plus each sample's difference at a bin's centre, 5 m
// out, with the edge gate opened.
struct Known {
  DepthCheckOptions options;
  std::vector<std::pair<long, std::size_t>> bins;  ///< Bin from 0, count.
  std::size_t outside = 0;
};

// The percentiles are the known histograms' quantiles by the rule, to
// float's rounding, whatever their shape: spread and skewed, with samples in
// the window's outermost bins; one bin; split at 0, whose median is 0;
// bins of 0.5 mm over 1 mm; one bin either side; and the most bins, 4096.
TEST_F(AgreementTest, ItsPercentilesAreTheKnownHistogramsQuantiles) {
  DepthCheckOptions defaults;
  defaults.edge_ratio = 1.0;
  DepthCheckOptions coarse = defaults;
  coarse.window_mm = 1.0;
  coarse.bin_mm = 0.5;
  DepthCheckOptions two = defaults;
  two.window_mm = 0.1;
  DepthCheckOptions most = defaults;
  most.window_mm = 204.8;
  const std::vector<Known> known = {
      {defaults,
       {{-40, 30},
        {-12, 55},
        {-3, 120},
        {-1, 200},
        {0, 260},
        {2, 410},
        {3, 380},
        {5, 150},
        {9, 90},
        {31, 40},
        {-1000, 3},
        {999, 4}},
       25},
      {defaults, {{7, 500}}, 0},
      {defaults, {{-1, 300}, {0, 300}}, 0},
      {coarse, {{-2, 10}, {-1, 3}, {0, 6}, {1, 1}}, 7},
      {two, {{-1, 5}, {0, 2}}, 3},
      {most, {{-2048, 11}, {2047, 13}, {100, 50}}, 0}};
  for (std::size_t k = 0; k < known.size(); ++k) {
    SCOPED_TRACE(k);
    const Known& case_ = known[k];
    const DepthCheckOptions& options = case_.options;
    const double bin = options.bin_mm / 1000.0;
    const long half = half_bins(options);
    std::vector<double> differences;
    std::vector<std::size_t> bins(static_cast<std::size_t>(2 * half), 0);
    for (const auto& [at, count] : case_.bins) {
      bins[static_cast<std::size_t>(at + half)] += count;
      differences.insert(differences.end(), count,
                         (static_cast<double>(at) + 0.5) * bin);
    }
    for (std::size_t i = 0; i < case_.outside; ++i) {
      const double past = options.window_mm / 1000.0 + 0.01 * (1 + i % 3);
      differences.push_back(i % 2 == 0 ? past : -past);
    }
    // Row by row from row 1 and column 1, as kWidth - 1 a row.
    const auto rows = static_cast<std::uint32_t>(
        1 + (differences.size() + kWidth - 2) / (kWidth - 1));
    std::vector<float> zero(std::size_t{kWidth} * rows, 5.0f);
    std::vector<float> one(zero.size(), 0.0f);  // no depth past the samples
    for (std::size_t i = 0; i < differences.size(); ++i) {
      one[(1 + i / (kWidth - 1)) * kWidth + 1 + i % (kWidth - 1)] =
          static_cast<float>(5.0 + differences[i]);
    }
    const std::vector<PairAgreement> pairs =
        agree({same_pose(zero, rows), same_pose(one, rows)}, options);
    ASSERT_EQ(pairs.size(), 2u);
    const PairAgreement& got = pairs[1];  // from 1 to 0
    EXPECT_EQ(got.overlap, differences.size());
    expect_stats(got, bins, options);
  }
  // Known values: one bin's samples spread across it; a split at 0.
  const auto at = [&](const Known& case_) {
    std::vector<float> zero(std::size_t{kWidth} * kHeight, 5.0f);
    std::vector<float> one(zero.size(), 0.0f);
    std::size_t i = 0;
    for (const auto& [bin_at, count] : case_.bins) {
      for (std::size_t n = 0; n < count; ++n, ++i) {
        one[(1 + i / (kWidth - 1)) * kWidth + 1 + i % (kWidth - 1)] =
            static_cast<float>(5.0 +
                               (static_cast<double>(bin_at) + 0.5) * 1e-4);
      }
    }
    return agree({same_pose(zero), same_pose(one)}, case_.options)[1];
  };
  const PairAgreement seventh = at(known[1]);
  EXPECT_NEAR(seventh.signed_median_mm, 0.75, 1e-6);
  EXPECT_NEAR(seventh.median_abs_mm, 0.75, 1e-6);
  EXPECT_NEAR(seventh.p90_abs_mm, 0.79, 1e-6);
  EXPECT_EQ(at(known[2]).signed_median_mm, 0.0);
}

// The quantiles' bins are found in integers, so counts past float's 24 bits
// take the right one: 2^24 samples at 1.05 mm and 2^24 + 2 at 6.05 mm put
// the two middle samples, and so the median, in the second bin, where a walk
// in float -- (2^25 + 2) / 2 rounding to 2^24 -- stops at the first. Camera
// 1's depth lies 1.05 mm behind camera 0's over half a 1025 x 1025 image and
// 6.05 mm over the other half, accumulated 32 times, then 6.05 mm behind at
// 2 pixels of one more capture.
TEST_F(AgreementTest, ItsQuantilesTakeTheRightBinPastFloatsCounts) {
  constexpr std::uint32_t kSide = 1025;
  std::vector<float> near(std::size_t{kSide} * kSide, 5.0f);
  std::vector<float> far(near.size());
  for (std::uint32_t v = 0; v < kSide; ++v) {
    for (std::uint32_t u = 0; u < kSide; ++u) {
      far[std::size_t{v} * kSide + u] = v <= kSide / 2 ? 5.00105f : 5.00605f;
    }
  }
  std::vector<float> two(std::size_t{kWidth} * kHeight, 0.0f);
  two[10 * kWidth + 10] = 5.00605f;
  two[20 * kWidth + 30] = 5.00605f;
  std::vector<core::Buffer> buffers;
  const std::vector<std::optional<DepthImage>> halves = images_of(
      {same_pose(near, kSide, kSide), same_pose(far, kSide, kSide)}, buffers);
  std::vector<core::Buffer> more_buffers;
  const std::vector<std::optional<DepthImage>> more = images_of(
      {same_pose(std::vector<float>(two.size(), 5.0f)), same_pose(two)},
      more_buffers);
  const DepthCheckOptions options;
  const std::unique_ptr<AgreementHistograms> histograms =
      value_of(checks_->histograms(2, options));
  for (int capture = 0; capture < 32; ++capture) {
    check(checks_->accumulate(*histograms, halves));
  }
  check(checks_->accumulate(*histograms, more));
  const std::vector<PairAgreement> pairs =
      value_of(checks_->summarize(*histograms));
  ASSERT_EQ(pairs.size(), 2u);
  constexpr std::size_t kEach = std::size_t{1} << 24;
  const long half = half_bins(options);
  for (const PairAgreement& got : pairs) {
    SCOPED_TRACE("from " + std::to_string(got.from));
    // From 1, the differences are +1.05 and +6.05 mm; from 0, their
    // negatives, in bins -11 and -61.
    const bool from_far = got.from == 1;
    std::vector<std::size_t> bins(static_cast<std::size_t>(2 * half), 0);
    bins[static_cast<std::size_t>(half + (from_far ? 10 : -11))] = kEach;
    bins[static_cast<std::size_t>(half + (from_far ? 60 : -61))] = kEach + 2;
    EXPECT_EQ(got.overlap, 2 * kEach + 2);
    expect_stats(got, bins, options);
    EXPECT_NEAR(got.signed_median_mm, from_far ? 6.0 : -6.0, 1e-4);
    EXPECT_NEAR(got.median_abs_mm, 6.0, 1e-4);
  }
}

// An empty slot takes no part: with slots 1 and 3 of 5 empty, the pairs of
// cameras 0, 2 and 4 are those a rig of the three alone gives, to the bit,
// and every pair of camera 1 or 3 has no samples; so with slot 1 of 3. A
// capture of one camera, or none, adds nothing.
TEST_F(AgreementTest, AnEmptySlotIsSkipped) {
  std::vector<Camera> five;
  for (std::size_t c = 0; c < 5; ++c) five.push_back(same_pose(smooth(c, 0)));
  const std::vector<PairAgreement> three = agree({five[0], five[2], five[4]});
  std::vector<Camera> gaps = five;
  gaps[1].depth.clear();
  gaps[3].depth.clear();
  const std::vector<PairAgreement> pairs = agree(gaps);
  ASSERT_EQ(pairs.size(), 20u);
  ASSERT_EQ(three.size(), 6u);
  const auto among = [](std::size_t c) { return c / 2; };  // 0, 2, 4
  for (const PairAgreement& got : pairs) {
    SCOPED_TRACE("from " + std::to_string(got.from) + " to " +
                 std::to_string(got.to));
    if (got.from % 2 == 1 || got.to % 2 == 1) {
      expect_unmeasured(got);
      continue;
    }
    const std::size_t from = among(got.from);
    const std::size_t to = among(got.to);
    PairAgreement want = three[from * 2 + (to < from ? to : to - 1)];
    want.from = got.from;
    want.to = got.to;
    expect_same(got, want);
    EXPECT_EQ(got.overlap, kInterior);
  }

  std::vector<Camera> one_gap = {five[0], five[1], five[2]};
  one_gap[1].depth.clear();
  const std::vector<PairAgreement> of_three = agree(one_gap);
  ASSERT_EQ(of_three.size(), 6u);
  std::vector<PairAgreement> two = agree({five[0], five[2]});
  ASSERT_EQ(two.size(), 2u);
  two[0].to = 2;
  two[1].from = 2;
  expect_unmeasured(of_three[0]);  // 0 to 1
  expect_same(of_three[1], two[0]);
  expect_unmeasured(of_three[2]);  // 1 to 0
  expect_unmeasured(of_three[3]);  // 1 to 2
  expect_same(of_three[4], two[1]);
  expect_unmeasured(of_three[5]);  // 2 to 1

  std::vector<core::Buffer> buffers;
  const std::vector<std::optional<DepthImage>> images =
      images_of({five[0], five[1], five[2]}, buffers);
  const std::unique_ptr<AgreementHistograms> histograms =
      value_of(checks_->histograms(3));
  check(checks_->accumulate(*histograms, {images[0], {}, {}}));
  check(checks_->accumulate(*histograms, {{}, {}, {}}));
  for (const PairAgreement& pair : value_of(checks_->summarize(*histograms))) {
    expect_unmeasured(pair);
  }
}

// The stride samples every stride-th column and row from the first: camera
// 1's depth matches camera 0's at pixels whose column and row are even, and
// lies 15 cm off elsewhere, with the edge gate opened. (Row and column 0 have
// no depth.)
TEST_F(AgreementTest, TakesEveryStrideThPixel) {
  std::vector<float> off(std::size_t{kWidth} * kHeight, 2.15f);
  for (std::uint32_t v = 0; v < kHeight; v += 2) {
    for (std::uint32_t u = 0; u < kWidth; u += 2) {
      off[std::size_t{v} * kWidth + u] = 2.0f;
    }
  }
  const std::vector<Camera> cameras = {
      same_pose(std::vector<float>(std::size_t{kWidth} * kHeight, 2.0f)),
      same_pose(off)};
  struct Case {
    std::uint32_t stride;
    std::size_t overlap;
    std::size_t in_window;
  };
  // 63 x 47 pixels with depth, 31 x 23 of them even; 21 x 15 samples at
  // stride 3, 10 x 7 of them even; and one at stride 40, (40, 40).
  for (const Case& c : {Case{1, kInterior, 713}, Case{2, 713, 713},
                        Case{3, 315, 70}, Case{40, 1, 1}}) {
    SCOPED_TRACE(c.stride);
    DepthCheckOptions options;
    options.edge_ratio = 1.0;
    options.agreement_stride = c.stride;
    const std::unique_ptr<AgreementHistograms> histograms =
        value_of(checks_->histograms(2, options));
    EXPECT_EQ(histograms->options().agreement_stride, c.stride);
    EXPECT_EQ(histograms->cameras(), 2u);
    for (const PairAgreement& got : agree(cameras, options)) {
      EXPECT_EQ(got.overlap, c.overlap);
      EXPECT_EQ(got.in_window, c.in_window);
    }
  }
}

// A pair whose every sample lies past the window has an overlap, but no
// statistics: camera 1's depth lies 15 cm behind camera 0's everywhere.
TEST_F(AgreementTest, APairWithNothingInItsWindowHasNoStatistics) {
  const std::vector<Camera> cameras = {
      same_pose(std::vector<float>(std::size_t{kWidth} * kHeight, 2.0f)),
      same_pose(std::vector<float>(std::size_t{kWidth} * kHeight, 2.15f))};
  const std::vector<PairAgreement> pairs = agree(cameras);
  ASSERT_EQ(pairs.size(), 2u);
  for (const PairAgreement& got : pairs) {
    SCOPED_TRACE("from " + std::to_string(got.from));
    EXPECT_EQ(got.overlap, kInterior);
    expect_no_statistics(got);
  }
}

// A capture that could carry a pair's counts past 2^32 - 1 is refused, and
// leaves the histograms and their bound as they were: two 8192 x 8192
// cameras with no depth add 2^26 samples a pair a capture, so the 64th
// capture is refused; one of 8191 x 8193, 2^26 - 1 samples, brings each pair
// to 2^32 - 1 exactly and is taken; and one sample more is refused. Skips
// where one binding cannot hold such a depth.
TEST_F(AgreementTest, RefusesACaptureThatCouldPassACountsLimit) {
  constexpr std::uint32_t kSide = 8192;
  const VkDeviceSize bytes = VkDeviceSize{kSide} * kSide * sizeof(float);
  if (core::max_storage_buffer_range(device()) < bytes) {
    GTEST_SKIP() << "one binding cannot hold an 8192 x 8192 depth";
  }
  core::Result<core::Buffer> made =
      core::device_storage_buffer(allocator(), bytes);
  if (!made.ok()) GTEST_SKIP() << made.status().message();
  const core::Buffer empty = *std::move(made);
  {
    core::CommandBatch batch(device(), allocator());
    check(batch.fill(empty, 0, bytes, 0));
    check(batch.submit());
  }
  Camera big = same_pose({});
  big.pinhole.size = {kSide, kSide};
  const DepthImage image = image_of(big, empty);
  const std::unique_ptr<AgreementHistograms> histograms =
      value_of(checks_->histograms(2));
  for (int capture = 0; capture < 63; ++capture) {
    check(checks_->accumulate(*histograms, {image, image}));
  }
  const std::string past =
      "the pair from camera 0 to camera 1 could pass 2^32 - 1";
  expect_refused(checks_->accumulate(*histograms, {image, image}), "accumulate",
                 past);
  Camera rest = same_pose({});
  rest.pinhole.size = {kSide - 1, kSide + 1};
  check(checks_->accumulate(*histograms,
                            {image_of(rest, empty), image_of(rest, empty)}));
  Camera one = same_pose({});
  one.pinhole.size = {1, 1};
  expect_refused(checks_->accumulate(
                     *histograms, {image_of(one, empty), image_of(one, empty)}),
                 "accumulate", past);
  for (const PairAgreement& pair : value_of(checks_->summarize(*histograms))) {
    expect_unmeasured(pair);
  }
}

// Every refusal names what it refuses and changes nothing: the histograms
// hold what they held, and the next capture adds to them as before.
TEST_F(AgreementTest, RefusesWhatItCannotTake) {
  const std::vector<Camera> cameras = {same_pose(smooth(0, 0)),
                                       same_pose(smooth(1, 0))};
  std::vector<core::Buffer> buffers;
  const std::vector<std::optional<DepthImage>> good =
      images_of(cameras, buffers);

  // histograms
  expect_refused(checks_->histograms(0).status(), "histograms", "no cameras");
  const auto refused_option = [&](auto set, const std::string& says) {
    DepthCheckOptions options;
    set(options);
    expect_refused(checks_->histograms(2, options).status(), "histograms",
                   says);
  };
  refused_option([](DepthCheckOptions& o) { o.agreement_stride = 0; },
                 "options.agreement_stride");
  refused_option([](DepthCheckOptions& o) { o.window_mm = 0.0; },
                 "options.window_mm");
  refused_option([](DepthCheckOptions& o) { o.bin_mm = -0.1; },
                 "options.bin_mm");
  refused_option(
      [](DepthCheckOptions& o) {
        o.edge_ratio = std::numeric_limits<double>::quiet_NaN();
      },
      "options.edge_ratio");
  refused_option([](DepthCheckOptions& o) { o.window_mm = 204.9; },
                 "even number of bins");  // 4098 bins
  refused_option(
      [](DepthCheckOptions& o) {
        o.window_mm = 0.15;  // 3 bins
      },
      "even number of bins");
  expect_refused(checks_->histograms(std::size_t{1} << 20).status(),
                 "histograms", "more pairs than one binding holds");
  // Two bins a pair, 8 bytes, but accumulate's block for each, 144 bytes
  // (agreement_blocks.hpp's PairBlock), bounds them: refused at the first
  // count whose blocks one binding cannot hold.
  DepthCheckOptions two_bins;
  two_bins.window_mm = 0.1;
  const VkDeviceSize range = core::max_storage_buffer_range(device());
  std::size_t crowd = 2;
  while (VkDeviceSize{crowd} * (crowd - 1) * 144 <= range) ++crowd;
  expect_refused(checks_->histograms(crowd, two_bins).status(), "histograms",
                 "more pairs than one binding holds");

  // accumulate
  const std::unique_ptr<AgreementHistograms> histograms =
      value_of(checks_->histograms(2));
  check(checks_->accumulate(*histograms, good));
  const std::vector<PairAgreement> before =
      value_of(checks_->summarize(*histograms, 7));
  ASSERT_EQ(before.size(), 2u);
  EXPECT_EQ(before[0].capture, std::optional<std::int64_t>(7));
  EXPECT_EQ(before[0].overlap, kInterior);
  const auto refused = [&](const std::vector<std::optional<DepthImage>>& bad,
                           const std::string& says) {
    core::Status status;
    EXPECT_NO_THROW(status = checks_->accumulate(*histograms, bad));
    expect_refused(status, "accumulate", says);
    const std::vector<PairAgreement> after =
        value_of(checks_->summarize(*histograms, 7));
    for (std::size_t i = 0; i < before.size(); ++i) {
      expect_same(after[i], before[i]);
    }
  };
  refused({good[0]}, "1 image slots for 2 cameras");
  refused({good[0], good[1], good[1]}, "3 image slots for 2 cameras");
  std::optional<DepthImage> bad = good[1];
  bad->depth = nullptr;
  refused({good[0], bad}, "camera 1 has no depth buffer");
  bad = good[1];
  bad->intrinsics.fx = 0.0;
  refused({good[0], bad}, "focal lengths");
  bad = good[1];
  bad->min_depth_m = 2.0;
  bad->max_depth_m = 1.0;
  refused({good[0], bad}, "range");
  bad = good[0];
  bad->depth_to_world[0] *= 1.01;
  refused({bad, good[1]}, "camera 0's depth_to_world");
  bad = good[1];
  bad->size.height += 1;
  refused({good[0], bad}, "the binding reads");
  core::BufferDesc transfer;
  transfer.size = std::size_t{kWidth} * kHeight * sizeof(float);
  transfer.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  const core::Buffer unbindable = value_of(allocator().create_buffer(transfer));
  bad = good[1];
  bad->depth = &unbindable;
  refused({good[0], bad}, "camera 1's depth is not a storage buffer");

  // Histograms of another device, which neither call takes.
  core::Device other = value_of(core::Device::create(
      instance(), physical(), DepthChecks::device_requirements()));
  {
    core::Allocator other_allocator =
        value_of(core::Allocator::create(instance().handle(), other));
    const std::unique_ptr<DepthChecks> other_checks =
        value_of(DepthChecks::create(other, other_allocator));
    const std::unique_ptr<AgreementHistograms> foreign =
        value_of(other_checks->histograms(2));
    expect_refused(checks_->accumulate(*foreign, good), "accumulate",
                   "another device");
    expect_refused(checks_->summarize(*foreign).status(), "summarize",
                   "another device");
    const std::unique_ptr<AgreementHistograms> own =
        value_of(checks_->histograms(2));
    expect_refused(checks_->accumulate(*histograms, good, foreign.get()),
                   "accumulate", "the capture's own histograms");
    expect_refused(checks_->accumulate(*foreign, good, own.get()), "accumulate",
                   "another device");
  }

  check(checks_->accumulate(*histograms, good));
  const std::vector<PairAgreement> twice =
      value_of(checks_->summarize(*histograms));
  EXPECT_EQ(twice[0].overlap, 2 * before[0].overlap);
  EXPECT_FALSE(twice[0].capture.has_value());
}

}  // namespace
}  // namespace volumetric_kit::calib::depth
