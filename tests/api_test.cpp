// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The stable API (README.md's table) as a program built on calib uses it: the
// sample mains, including only the table's headers (and the core's), run on
// synthetic data. A break in a call they make fails this build; the rest of
// the table is held in review (DECISIONS.md, 2026-11-01).

#include <cstddef>
#include <cstdio>

#include <gtest/gtest.h>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace calib = volumetric_kit::calib;
namespace core = volumetric_kit::core;

namespace {

// Main 1, entering a board: what the Board window does with the user's
// fields.
core::Status enter_board(std::size_t& corners) {
  calib::Target board = calib::legacy_checkerboard();  // 13 x 8 inner corners
  board.square_size_m = 0.01994;                       // measured
  VKC_TRY(calib::check_target(board));
  VKC_ASSIGN(const calib::compute::Board b,
             calib::compute::Board::create(board));
  corners = b.object_points().size();
  std::printf("%zu corners\n", corners);  // 104
  return {};
}

TEST(Api, EntersABoard) {
  std::size_t corners = 0;
  const core::Status entered = enter_board(corners);
  ASSERT_TRUE(entered.ok()) << entered.message();
  EXPECT_EQ(corners, 104u);
}

}  // namespace
