// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The depth tier runs out of memory anywhere in building its checks or in
// refusing a fit or a pair's histograms or capture, and returns OutOfMemory,
// never throwing; the next call is as before. The Vulkan loader, layers and
// driver are spared (test::SpareVulkan). Skips where no device is present.
//
// TODO: Sweep a successful fit, histograms, capture and summary too once the
// core's Allocator::create_buffer frees its VkBuffer when a host allocation
// fails after vkCreateBuffer. It throws std::bad_alloc there, which the calls
// return as OutOfMemory, and leaks the buffer, which the layer reports as the
// device is destroyed.

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <glm/glm.hpp>

#include "allocation_failure.hpp"
#include "volumetric_kit/calib/depth/depth_checks.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"

namespace volumetric_kit::calib::depth {
namespace {

using Code = core::Status::Code;
using quality::DepthCheckOptions;
using quality::DepthPlane;
using quality::PairAgreement;
using test::expect_oom_anywhere;
using test::expect_oom_when_reporting;

class DepthOom : public core::test::VulkanDeviceTest {
 protected:
  // A 64 x 48 depth camera square to the board 1 m away, seeing it at 1 m,
  // and a region of 512 of its pixels.
  void SetUp() override {
    VulkanDeviceTest::SetUp();
    if (base_setup_incomplete()) return;
    const std::vector<float> depth(64 * 48, 1.0f);
    const VkDeviceSize bytes = depth.size() * sizeof(float);
    core::Result<core::Buffer> buffer =
        core::device_storage_buffer(allocator(), bytes);
    ASSERT_TRUE(buffer.ok()) << buffer.status().message();
    buffer_ = *std::move(buffer);
    core::CommandBatch batch(device(), allocator());
    ASSERT_TRUE(batch.upload(buffer_, 0, depth.data(), bytes).ok());
    ASSERT_TRUE(batch.submit().ok());
    good_.image.depth = &buffer_;
    good_.image.intrinsics = {64.0, 64.0, 31.5, 23.5};
    good_.image.size = {64, 48};
    good_.image.min_depth_m = 0.25;
    good_.image.max_depth_m = 6.0;
    good_.region.depth_to_board[3] = glm::dvec4(0.25, 0.125, -1.0, 1.0);
    good_.region.min = {0.0, 0.0};
    good_.region.max = {0.5, 0.25};
  }

  core::Result<std::unique_ptr<DepthChecks>> create() {
    return DepthChecks::create(device(), allocator());
  }

  // `checks` fits `regions`, each as good_ alone fits.
  static void expect_fits(DepthChecks& checks,
                          const std::vector<PlaneRegion>& regions) {
    const core::Result<std::vector<DepthPlane>> planes =
        checks.fit_planes(regions);
    ASSERT_TRUE(planes.ok()) << planes.status().message();
    ASSERT_EQ(planes->size(), regions.size());
    for (const DepthPlane& plane : *planes) {
      EXPECT_TRUE(plane.valid);
      EXPECT_EQ(plane.members, 32u * 16u);
      EXPECT_EQ(plane.inliers, plane.members);
      EXPECT_NEAR(plane.normal.z, 1.0, 1e-6);
      EXPECT_NEAR(plane.point.z, 0.0, 1e-6);
    }
  }

  core::Buffer buffer_;
  PlaneRegion good_;
};

// Building the checks runs out of memory anywhere as OutOfMemory.
TEST_F(DepthOom, Create) {
  const test::SpareVulkan spare;
  expect_oom_anywhere([&] { return create(); });
}

// Each refusal, its message built, runs out of memory as OutOfMemory, and the
// next fit is as before. Refusals reach no device call.
TEST_F(DepthOom, FitRefusals) {
  core::Result<std::unique_ptr<DepthChecks>> made = create();
  ASSERT_TRUE(made.ok()) << made.status().message();
  DepthChecks& checks = **made;
  core::BufferDesc transfer;
  transfer.size = 64 * 48 * sizeof(float);
  transfer.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  core::Result<core::Buffer> unbindable = allocator().create_buffer(transfer);
  ASSERT_TRUE(unbindable.ok()) << unbindable.status().message();

  std::vector<std::vector<PlaneRegion>> refused;
  PlaneRegion bad = good_;
  bad.image.depth = nullptr;
  refused.push_back({good_, bad});
  bad = good_;
  bad.image.intrinsics.fx = 0.0;
  refused.push_back({good_, bad});
  bad = good_;
  bad.image.max_depth_m = bad.image.min_depth_m;
  refused.push_back({good_, bad});
  bad = good_;
  bad.image.depth_to_world[0] *= 1.01;
  refused.push_back({good_, bad});
  bad = good_;
  bad.region.max.x = bad.region.min.x;
  refused.push_back({good_, bad});
  bad = good_;
  bad.image.size.height += 1;
  refused.push_back({good_, bad});
  bad = good_;
  bad.image.depth = &*unbindable;
  refused.push_back({good_, bad});
  for (const std::vector<PlaneRegion>& regions : refused) {
    expect_oom_when_reporting([&] { return checks.fit_planes(regions); },
                              Code::InvalidArgument);
  }
  DepthCheckOptions seed;
  seed.seed_mm = 0.0;
  const std::vector<PlaneRegion> one = {good_};
  expect_oom_when_reporting([&] { return checks.fit_planes(one, seed); },
                            Code::InvalidArgument);
  expect_fits(checks, one);
}

// Each of the agreement's refusals, its message built, runs out of memory as
// OutOfMemory, and leaves the histograms empty; the next capture adds to them.
// Refusals reach no device call. (No cameras is refused with a message short
// enough to allocate nothing.)
TEST_F(DepthOom, AgreementRefusals) {
  core::Result<std::unique_ptr<DepthChecks>> made = create();
  ASSERT_TRUE(made.ok()) << made.status().message();
  DepthChecks& checks = **made;
  DepthCheckOptions odd;
  odd.window_mm = 0.15;  // 3 bins
  expect_oom_when_reporting([&] { return checks.histograms(2, odd); },
                            Code::InvalidArgument);

  core::Result<std::unique_ptr<AgreementHistograms>> made_histograms =
      checks.histograms(2);
  ASSERT_TRUE(made_histograms.ok()) << made_histograms.status().message();
  AgreementHistograms& histograms = **made_histograms;
  core::BufferDesc transfer;
  transfer.size = 64 * 48 * sizeof(float);
  transfer.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  core::Result<core::Buffer> unbindable = allocator().create_buffer(transfer);
  ASSERT_TRUE(unbindable.ok()) << unbindable.status().message();
  const std::optional<DepthImage> good = good_.image;
  std::vector<std::vector<std::optional<DepthImage>>> refused;
  refused.push_back({good});
  std::optional<DepthImage> bad = good;
  bad->depth = nullptr;
  refused.push_back({good, bad});
  bad = good;
  bad->intrinsics.fx = 0.0;
  refused.push_back({good, bad});
  bad = good;
  bad->depth_to_world[0] *= 1.01;
  refused.push_back({bad, good});
  bad = good;
  bad->size.height += 1;
  refused.push_back({good, bad});
  bad = good;
  bad->depth = &*unbindable;
  refused.push_back({good, bad});
  for (const std::vector<std::optional<DepthImage>>& images : refused) {
    expect_oom_when_reporting(
        [&]() -> core::Result<bool> {
          VKC_TRY(checks.accumulate(histograms, images));
          return true;
        },
        Code::InvalidArgument);
  }
  core::Result<std::vector<PairAgreement>> pairs = checks.summarize(histograms);
  ASSERT_TRUE(pairs.ok()) << pairs.status().message();
  for (const PairAgreement& pair : *pairs) EXPECT_EQ(pair.overlap, 0u);
  ASSERT_TRUE(checks.accumulate(histograms, {good, good}).ok());
  pairs = checks.summarize(histograms);
  ASSERT_TRUE(pairs.ok()) << pairs.status().message();
  for (const PairAgreement& pair : *pairs) EXPECT_GT(pair.overlap, 0u);
}

}  // namespace
}  // namespace volumetric_kit::calib::depth
