// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A refusal can run out of memory while it builds its message. Detection
// must report that as OutOfMemory too.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "allocation_failure.hpp"
#include "volumetric_kit/calib/detect/charuco.hpp"

namespace volumetric_kit::calib::detect {
namespace {

using Code = core::Status::Code;
using test::expect_oom_when_reporting;

TEST(DetectOom, ImageValidation) {
  const compute::Board board =
      compute::Board::create(legacy_femto_board()).value();
  const std::vector<std::uint8_t> pixels(64, 0);
  const LumaView narrow{8, 8, 7, pixels.data()};
  expect_oom_when_reporting(
      [&] { return detect_charuco(narrow, board, Mode::Hybrid); },
      Code::InvalidArgument);
}

TEST(DetectOom, CornerIdValidation) {
  const compute::Board board =
      compute::Board::create(legacy_femto_board()).value();
  Detection detection;
  detection.image_size = {3840, 2160};
  detection.observation.points = {{0, {100.0, 100.0}, 1.0},
                                  {123456789, {200.0, 100.0}, 1.0}};
  expect_oom_when_reporting([&] { return usable_for_pose(detection, board); },
                            Code::InvalidArgument);
}

}  // namespace
}  // namespace volumetric_kit::calib::detect
