// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file depth_fixture.hpp
/// @brief The depth tier's device tests' fixture: the depth checks on a
///        validated device, and depth uploaded as recon's frame preparation
///        leaves it. Its users link volumetric_kit::calib_depth and
///        volumetric_kit::core_test_support.

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "result_checks.hpp"
#include "volumetric_kit/calib/depth/depth_checks.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"

namespace volumetric_kit::calib::test {

/// The prepared depth's range in the depth tier's tests (metres).
inline constexpr double kMinDepth = 0.25;
inline constexpr double kMaxDepth = 6.0;  ///< See @ref kMinDepth.

/// @brief The kernels' rule for a measured depth, in double.
/// @param z          A depth (metres; 0 for none).
/// @param min_depth  The camera's range, ends included.
/// @param max_depth  See @p min_depth.
/// @return Whether @p z is positive and in the range; false for a NaN.
inline bool measured(double z, double min_depth = kMinDepth,
                     double max_depth = kMaxDepth) {
  return z > 0.0 && z >= min_depth && z <= max_depth;
}

/// @brief A test of @ref depth::DepthChecks built on the shared device.
///
/// It runs under synchronization validation with its tracking of what
/// shaders access through their descriptors, wherever the layer has it: the
/// batch's barriers are the core's, and the layer checks the kernels' reads
/// and writes of their storage buffers against them.
///
/// @code
/// class AgreementTest : public test::DepthChecksTest {};
///
/// TEST_F(AgreementTest, Agrees) {
///   const core::Buffer depth = upload(std::vector<float>(64 * 48, 2.0f));
///   // ... checks_->accumulate(...)
/// }
/// @endcode
class DepthChecksTest : public core::test::VulkanDeviceTest {
 protected:
  /// @return `ShaderAccesses`.
  core::test::Validation validation() const override {
    return core::test::Validation::ShaderAccesses;
  }

  /// @brief Build the checks, unless the base skipped or failed.
  void SetUp() override {
    VulkanDeviceTest::SetUp();
    if (base_setup_incomplete()) return;
    core::Result<std::unique_ptr<depth::DepthChecks>> checks =
        depth::DepthChecks::create(device(), allocator());
    ASSERT_TRUE(checks.ok()) << checks.status().message();
    checks_ = *std::move(checks);
  }

  /// @brief @p depth on the device, as recon's frame preparation leaves a
  ///        frame's.
  /// @param depth  Row-major depth (metres; 0 for none).
  /// @return A device-local storage buffer holding it.
  core::Buffer upload(const std::vector<float>& depth) {
    const VkDeviceSize bytes = depth.size() * sizeof(float);
    core::Buffer buffer =
        value_of(core::device_storage_buffer(allocator(), bytes));
    core::CommandBatch batch(device(), allocator());
    check(batch.upload(buffer, 0, depth.data(), bytes));
    check(batch.submit());
    return buffer;
  }

  std::unique_ptr<depth::DepthChecks> checks_;  ///< The checks under test.
};

}  // namespace volumetric_kit::calib::test
