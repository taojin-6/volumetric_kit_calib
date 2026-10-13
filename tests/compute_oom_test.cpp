// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Invalid input can run out of memory while validation builds a message or
// adds context. The public bridge must report that as OutOfMemory too.

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <type_traits>

#include <gtest/gtest.h>

#include "allocation_failure.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/cv_pose.hpp"

namespace volumetric_kit::calib::compute {
namespace {

using Code = core::Status::Code;
namespace camera = recon::camera;

// Sweep every allocation in a rejected call, including with_context and
// Result construction, without depending on string allocation counts or
// small-string buffer sizes. The first uninjected call also checks retry.
template <class Fn>
void expect_oom_when_reporting(const Fn& fn, Code expected) {
  const auto original = fn();
  ASSERT_EQ(original.status().domain(), expected);
  for (std::size_t after = 0; after < 32; ++after) {
    SCOPED_TRACE(after);
    std::optional<std::invoke_result_t<const Fn&>> result;
    EXPECT_NO_THROW({
      const test::FailAllocation failure(after);
      result.emplace(fn());
    });
    ASSERT_TRUE(result.has_value());
    if (!test::allocation_failure.injected) {
      EXPECT_GT(after, 0u) << "the refusal must exercise allocation failure";
      EXPECT_EQ(result->status().domain(), expected);
      EXPECT_EQ(result->status().message(), original.status().message());
      return;
    }
    EXPECT_EQ(result->status().domain(), Code::OutOfMemory);
  }
  FAIL() << "did not reach a refusal without an injected allocation failure";
}

TEST(ComputeOom, BoardTargetValidation) {
  Target target = legacy_femto_board();
  target.rows = 0;
  expect_oom_when_reporting([&] { return Board::create(target); },
                            Code::InvalidArgument);
}

TEST(ComputeOom, BoardUnsupportedKind) {
  Target target = legacy_femto_board();
  target.kind = TargetKind::Checkerboard;
  target.dictionary = MarkerDictionary::None;
  target.marker_size_m = 0.0;
  target.legacy_pattern = false;
  expect_oom_when_reporting([&] { return Board::create(target); },
                            Code::Unsupported);
}

TEST(ComputeOom, BoardFloatValidation) {
  Target target = legacy_femto_board();
  target.square_size_m = std::numeric_limits<double>::min();
  target.marker_size_m = target.square_size_m / 2.0;
  expect_oom_when_reporting([&] { return Board::create(target); },
                            Code::InvalidArgument);
}

TEST(ComputeOom, CameraModelValidationAndContext) {
  const camera::CameraModel model{};
  expect_oom_when_reporting([&] { return cv_camera(model); },
                            Code::InvalidArgument);
}

TEST(ComputeOom, CameraSizeValidation) {
  camera::CameraModel model{};
  model.size = {
      static_cast<std::uint32_t>(std::numeric_limits<int>::max()) + 1u, 480};
  model.intrinsics = {500.0, 500.0, 320.0, 240.0};
  expect_oom_when_reporting([&] { return cv_camera(model); },
                            Code::InvalidArgument);
}

TEST(ComputeOom, PoseValidationAndContext) {
  camera::Mat4d transform(1.0);
  transform[0][0] = 2.0;
  expect_oom_when_reporting([&] { return cv_pose(transform); },
                            Code::InvalidArgument);
}

}  // namespace
}  // namespace volumetric_kit::calib::compute
