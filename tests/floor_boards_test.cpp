// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The boards on the floor of a world whose +z is up: a board lying flat on
// the floor reads 0 degrees and 0 mm, one tilted 3 degrees reads 3, and
// boards on stands, tilted past 5 degrees, 20 mm off the floor or face down
// are left out; and the refusals.

#include "volumetric_kit/calib/quality/floor_boards.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <glm/gtc/matrix_transform.hpp>

#include "result_checks.hpp"
#include "synthetic_rig.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;
using camera::Mat4d;
using camera::Vec3d;

// The board face up, its centre at `centre`, turned `tilt_deg` about the
// world's x axis: its x along the world's, its y down the world's -y, its z
// into the floor.
Mat4d face_up(const compute::Board& board, const Vec3d& centre,
              double tilt_deg = 0.0) {
  const Mat4d turn =
      glm::rotate(Mat4d(1.0), glm::radians(tilt_deg), Vec3d(1.0, 0.0, 0.0)) *
      glm::rotate(Mat4d(1.0), glm::radians(180.0), Vec3d(1.0, 0.0, 0.0));
  const cv::Point3d middle = board.centre();
  Mat4d pose = turn;
  pose[3] = glm::dvec4(
      centre - camera::Mat3d(turn) * Vec3d(middle.x, middle.y, middle.z), 1.0);
  return pose;
}

std::vector<solve::SolvedBoard> captures_of(std::size_t count) {
  std::vector<solve::SolvedBoard> boards(count);
  for (std::size_t b = 0; b < count; ++b) {
    boards[b].capture = static_cast<std::int64_t>(10 + b);
  }
  return boards;
}

TEST(FloorBoards, ReadsEachBoardOnTheFloorAndLeavesOutTheRest) {
  const compute::Board board = test::femto_board();
  const std::vector<Mat4d> poses = {
      face_up(board, {0.3, -0.2, 0.0}),          // 10: flat on the floor
      face_up(board, {-0.5, 0.4, 0.0}, 3.0),     // 11: tilted 3 degrees
      face_up(board, {0.0, 0.0, 0.5}),           // 12: on a stand
      face_up(board, {0.0, 0.0, 0.015}),         // 13: 15 mm up
      face_up(board, {0.0, 0.0, -0.025}),        // 14: 25 mm down
      face_up(board, {0.0, 0.0, 0.0}, 6.0),      // 15: tilted 6 degrees
      face_up(board, {0.0, 0.0, 0.0}, 180.0),    // 16: face down
      face_up(board, {0.2, 0.2, -0.004}, -4.5),  // 17: 4.5 degrees, 4 mm down
  };
  const std::vector<FloorBoard> floor =
      test::value_of(floor_boards(board, captures_of(poses.size()), poses));
  ASSERT_EQ(floor.size(), 4u);
  EXPECT_EQ(floor[0].capture, 10);
  EXPECT_NEAR(floor[0].angle_deg, 0.0, 1e-9);
  EXPECT_NEAR(floor[0].height_m, 0.0, 1e-12);
  EXPECT_EQ(floor[1].capture, 11);
  EXPECT_NEAR(floor[1].angle_deg, 3.0, 1e-9);
  EXPECT_NEAR(floor[1].height_m, 0.0, 1e-12);
  EXPECT_EQ(floor[2].capture, 13);
  EXPECT_NEAR(floor[2].height_m, 0.015, 1e-12);
  EXPECT_EQ(floor[3].capture, 17);
  EXPECT_NEAR(floor[3].angle_deg, 4.5, 1e-9);
  EXPECT_NEAR(floor[3].height_m, -0.004, 1e-12);

  // Stricter options leave the tilted and the raised out.
  FloorBoardOptions strict;
  strict.max_angle_deg = 1.0;
  strict.max_height_m = 0.01;
  const std::vector<FloorBoard> flat = test::value_of(
      floor_boards(board, captures_of(poses.size()), poses, strict));
  ASSERT_EQ(flat.size(), 1u);
  EXPECT_EQ(flat[0].capture, 10);
  EXPECT_TRUE(test::value_of(floor_boards(board, {}, {})).empty());
}

TEST(FloorBoards, RefusesWhatItCannotRead) {
  const compute::Board board = test::femto_board();
  const std::vector<Mat4d> one = {face_up(board, Vec3d(0.0))};
  const auto refused = [&](const std::vector<solve::SolvedBoard>& boards,
                           const std::vector<Mat4d>& poses,
                           const FloorBoardOptions& options,
                           const std::string& says) {
    SCOPED_TRACE(says);
    test::expect_refused(floor_boards(board, boards, poses, options).status(),
                         "floor_boards", says);
  };
  refused(captures_of(2), one, {}, "1 poses for 2 boards");
  std::vector<Mat4d> scaled = one;
  scaled[0][0][0] = 1.5;
  refused(captures_of(1), scaled, {}, "the board of capture 10");
  for (const double bad : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    FloorBoardOptions angle;
    angle.max_angle_deg = bad;
    refused(captures_of(1), one, angle,
            "options.max_angle_deg must be positive and finite");
    FloorBoardOptions height;
    height.max_height_m = bad;
    refused(captures_of(1), one, height,
            "options.max_height_m must be positive and finite");
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
