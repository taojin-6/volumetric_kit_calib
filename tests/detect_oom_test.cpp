// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A refusal can run out of memory while it builds its message. Detection
// must report that as OutOfMemory too. A search itself is not swept: on Arm,
// OpenCV's detectors call KleidiCV's HAL (warpPerspective and resize), whose
// allocations end the program when they fail, since an exception cannot cross
// its C code; detect_tag_test returns OpenCV's own StsNoMem as OutOfMemory.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "allocation_failure.hpp"
#include "volumetric_kit/calib/detect/charuco.hpp"
#include "volumetric_kit/calib/detect/tag.hpp"

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
  expect_oom_when_reporting([&] { return detect_tag(narrow, kOriginTag); },
                            Code::InvalidArgument);
}

TEST(DetectOom, TagRefusal) {
  const std::vector<std::uint8_t> pixels(64, 0);
  OriginTag past = kOriginTag;
  past.id = 587;
  expect_oom_when_reporting(
      [&] { return detect_tag({8, 8, 8, pixels.data()}, past); },
      Code::InvalidArgument);
}

}  // namespace
}  // namespace volumetric_kit::calib::detect
