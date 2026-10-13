// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Invalid input can run out of memory while validation builds a message or
// adds context. The public bridge must report that as OutOfMemory too.

#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "allocation_failure.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/cv_pose.hpp"

namespace volumetric_kit::calib::compute {
namespace {

using Code = core::Status::Code;
using test::expect_oom_when_reporting;
namespace camera = recon::camera;

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
