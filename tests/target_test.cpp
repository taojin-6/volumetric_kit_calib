// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The board: the rig's, and what check_target accepts and refuses.

#include "volumetric_kit/calib/core/target.hpp"

#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// The status is an InvalidArgument whose message names `field`.
::testing::AssertionResult RefusedNaming(const core::Status& status,
                                         const std::string& field) {
  if (status.ok()) return ::testing::AssertionFailure() << "accepted";
  if (status.domain() != core::Status::Code::InvalidArgument) {
    return ::testing::AssertionFailure()
           << "refused as " << core::to_string(status.domain()) << ": "
           << status.message();
  }
  if (status.message().find(field) == std::string::npos) {
    return ::testing::AssertionFailure()
           << "message does not name " << field << ": " << status.message();
  }
  return ::testing::AssertionSuccess();
}

Target checkerboard() {
  Target board;
  board.kind = TargetKind::Checkerboard;
  board.rows = 6;
  board.cols = 9;
  board.square_size_m = 0.025;
  return board;
}

TEST(Target, LegacyFemtoBoardIsTheRigsBoard) {
  const Target board = legacy_femto_board();
  EXPECT_EQ(board.kind, TargetKind::ChArUco);
  // OpenCV's CharucoBoard(Size(5, 3), ...) in squares; inner corners here.
  EXPECT_EQ(board.cols, 4);
  EXPECT_EQ(board.rows, 2);
  EXPECT_EQ(board.square_size_m, 0.15);
  EXPECT_EQ(board.marker_size_m, 0.112);
  EXPECT_EQ(board.dictionary, MarkerDictionary::Aruco4x4_100);
  EXPECT_EQ(board.first_marker_id, 0);
  EXPECT_TRUE(board.legacy_pattern);
  EXPECT_FALSE(board.square_size_tolerance_m);
  EXPECT_FALSE(board.planarity_bow_m);
  EXPECT_TRUE(check_target(board).ok()) << check_target(board).message();
}

// A default board names no layout, markers or tolerances, so a check refuses
// it until one is filled in.
TEST(Target, StartsEmptyAndIsRefused) {
  const Target board;
  EXPECT_EQ(board.dictionary, MarkerDictionary::None);
  EXPECT_EQ(board.first_marker_id, 0);
  EXPECT_FALSE(board.legacy_pattern);
  EXPECT_FALSE(board.square_size_tolerance_m);
  EXPECT_FALSE(board.planarity_bow_m);
  EXPECT_TRUE(RefusedNaming(check_target(board), "rows"));
}

TEST(Target, NeedsAFeatureGrid) {
  for (const int rows : {0, -1}) {
    Target board = legacy_femto_board();
    board.rows = rows;
    EXPECT_TRUE(RefusedNaming(check_target(board), "rows")) << rows;
  }
  Target board = legacy_femto_board();
  board.cols = 0;
  EXPECT_TRUE(RefusedNaming(check_target(board), "cols"));
}

TEST(Target, NeedsAFinitePositiveSquare) {
  for (const double size : {0.0, -0.15, kNaN, kInf}) {
    Target board = legacy_femto_board();
    board.square_size_m = size;
    EXPECT_TRUE(RefusedNaming(check_target(board), "square_size_m")) << size;
  }
}

TEST(Target, KnownTolerancesAreFiniteAndNotNegative) {
  Target board = legacy_femto_board();
  board.square_size_tolerance_m = 0.0;
  board.planarity_bow_m = 0.0;
  EXPECT_TRUE(check_target(board).ok());
  board.square_size_tolerance_m = 0.0002;
  board.planarity_bow_m = 0.0005;
  EXPECT_TRUE(check_target(board).ok());
  for (const double bad : {-1e-6, kNaN, kInf}) {
    Target tolerance = board;
    tolerance.square_size_tolerance_m = bad;
    EXPECT_TRUE(
        RefusedNaming(check_target(tolerance), "square_size_tolerance_m"))
        << bad;
    Target bow = board;
    bow.planarity_bow_m = bad;
    EXPECT_TRUE(RefusedNaming(check_target(bow), "planarity_bow_m")) << bad;
  }
}

TEST(Target, BoardWithMarkersNamesItsDictionary) {
  Target board = legacy_femto_board();
  board.dictionary = MarkerDictionary::None;
  EXPECT_TRUE(RefusedNaming(check_target(board), "dictionary"));
}

TEST(Target, MarkerFitsInsideItsSquare) {
  for (const double size : {0.0, -0.1, 0.15, 0.2, kNaN, kInf}) {
    Target board = legacy_femto_board();
    board.marker_size_m = size;
    EXPECT_TRUE(RefusedNaming(check_target(board), "marker_size_m")) << size;
  }
}

TEST(Target, MarkerIdsStayInsideTheDictionary) {
  // 5 x 3 squares carry 7 markers, so 4x4_100's ids 0-99 hold ids 93-99.
  Target board = legacy_femto_board();
  board.first_marker_id = 93;
  EXPECT_TRUE(check_target(board).ok());
  board.first_marker_id = 94;
  EXPECT_TRUE(RefusedNaming(check_target(board), "first_marker_id"));
  board.first_marker_id = -1;
  EXPECT_TRUE(RefusedNaming(check_target(board), "first_marker_id"));
  board.first_marker_id = std::numeric_limits<int>::max();
  EXPECT_TRUE(RefusedNaming(check_target(board), "first_marker_id"));

  // 6 x 4 squares: an even count, half of them markers.
  board = legacy_femto_board();
  board.cols = 5;
  board.rows = 3;
  board.first_marker_id = 88;
  EXPECT_TRUE(check_target(board).ok());
  board.first_marker_id = 89;
  EXPECT_TRUE(RefusedNaming(check_target(board), "first_marker_id"));

  // A dictionary's own size: AprilTag 36h11 holds 587 codes.
  board = legacy_femto_board();
  board.dictionary = MarkerDictionary::AprilTag36h11;
  board.first_marker_id = 580;
  EXPECT_TRUE(check_target(board).ok());
  board.first_marker_id = 581;
  EXPECT_TRUE(RefusedNaming(check_target(board), "first_marker_id"));
}

TEST(Target, AprilGridCarriesOneTagPerFeature) {
  Target grid;
  grid.kind = TargetKind::AprilGrid;
  grid.rows = 6;
  grid.cols = 6;
  grid.square_size_m = 0.088 * 1.3;  // tag size plus spacing
  grid.marker_size_m = 0.088;
  grid.dictionary = MarkerDictionary::AprilTag16h5;  // 30 codes, 36 tags
  EXPECT_TRUE(RefusedNaming(check_target(grid), "first_marker_id"));
  grid.dictionary = MarkerDictionary::AprilTag36h11;
  EXPECT_TRUE(check_target(grid).ok()) << check_target(grid).message();
  grid.first_marker_id = 587 - 36;
  EXPECT_TRUE(check_target(grid).ok());
  grid.first_marker_id = 587 - 35;
  EXPECT_TRUE(RefusedNaming(check_target(grid), "first_marker_id"));
}

TEST(Target, PatternWithoutMarkersTakesNone) {
  for (const TargetKind kind :
       {TargetKind::Checkerboard, TargetKind::SymmetricCircleGrid,
        TargetKind::AsymmetricCircleGrid, TargetKind::Screen}) {
    Target board = checkerboard();
    board.kind = kind;
    EXPECT_TRUE(check_target(board).ok()) << static_cast<int>(kind);

    Target dictionary = board;
    dictionary.dictionary = MarkerDictionary::Aruco4x4_50;
    EXPECT_TRUE(RefusedNaming(check_target(dictionary), "dictionary"));
    Target marker = board;
    marker.marker_size_m = 0.01;
    EXPECT_TRUE(RefusedNaming(check_target(marker), "marker_size_m"));
    Target first = board;
    first.first_marker_id = 1;
    EXPECT_TRUE(RefusedNaming(check_target(first), "first_marker_id"));
  }
}

TEST(Target, OnlyChArUcoTakesTheLegacyPattern) {
  Target board = checkerboard();
  board.legacy_pattern = true;
  EXPECT_TRUE(RefusedNaming(check_target(board), "legacy_pattern"));
  Target grid;
  grid.kind = TargetKind::AprilGrid;
  grid.rows = 2;
  grid.cols = 2;
  grid.square_size_m = 0.1;
  grid.marker_size_m = 0.08;
  grid.dictionary = MarkerDictionary::AprilTag36h11;
  grid.legacy_pattern = true;
  EXPECT_TRUE(RefusedNaming(check_target(grid), "legacy_pattern"));
  EXPECT_TRUE(check_target(legacy_femto_board()).ok());
}

}  // namespace
}  // namespace volumetric_kit::calib
