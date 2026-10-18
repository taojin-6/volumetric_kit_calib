// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The board: the rig's, what check_target accepts and refuses, and when two
// boards are the same. The origin tag: the rig's, and what check_origin_tag
// refuses.

#include "volumetric_kit/calib/core/target.hpp"

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "refused_naming.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

using test::RefusedNaming;

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
  EXPECT_EQ(board.dictionary, MarkerDictionary::ArUco4x4_100);
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

// Each dictionary holds as many codes as OpenCV's predefined one: a one-tag
// AprilGrid takes the last id and refuses the next.
TEST(Target, EachDictionaryHoldsItsCodes) {
  struct DictionaryCodes {
    MarkerDictionary dictionary;
    int codes;
  };
  constexpr DictionaryCodes kSizes[] = {
      {MarkerDictionary::ArUco4x4_50, 50},
      {MarkerDictionary::ArUco4x4_100, 100},
      {MarkerDictionary::ArUco4x4_250, 250},
      {MarkerDictionary::ArUco4x4_1000, 1000},
      {MarkerDictionary::ArUco5x5_50, 50},
      {MarkerDictionary::ArUco5x5_100, 100},
      {MarkerDictionary::ArUco5x5_250, 250},
      {MarkerDictionary::ArUco5x5_1000, 1000},
      {MarkerDictionary::ArUco6x6_50, 50},
      {MarkerDictionary::ArUco6x6_100, 100},
      {MarkerDictionary::ArUco6x6_250, 250},
      {MarkerDictionary::ArUco6x6_1000, 1000},
      {MarkerDictionary::ArUco7x7_50, 50},
      {MarkerDictionary::ArUco7x7_100, 100},
      {MarkerDictionary::ArUco7x7_250, 250},
      {MarkerDictionary::ArUco7x7_1000, 1000},
      {MarkerDictionary::ArUcoOriginal, 1024},
      {MarkerDictionary::AprilTag16h5, 30},
      {MarkerDictionary::AprilTag25h9, 35},
      {MarkerDictionary::AprilTag36h10, 2320},
      {MarkerDictionary::AprilTag36h11, 587},
      {MarkerDictionary::ArUcoMip36h12, 250},
  };
  // One entry per dictionary after None; ArUcoMip36h12 is the last.
  static_assert(sizeof(kSizes) / sizeof(kSizes[0]) ==
                    static_cast<int>(MarkerDictionary::ArUcoMip36h12),
                "a MarkerDictionary is missing from the table");
  for (const DictionaryCodes& size : kSizes) {
    Target tag;
    tag.kind = TargetKind::AprilGrid;
    tag.rows = 1;
    tag.cols = 1;
    tag.square_size_m = 0.1;
    tag.marker_size_m = 0.08;
    tag.dictionary = size.dictionary;
    tag.first_marker_id = size.codes - 1;
    EXPECT_TRUE(check_target(tag).ok())
        << size.codes << ": " << check_target(tag).message();
    tag.first_marker_id = size.codes;
    EXPECT_TRUE(RefusedNaming(check_target(tag), "first_marker_id"))
        << size.codes;
  }
}

// A value outside an enum is refused, naming its field.
TEST(Target, RefusesAValueOutsideItsEnum) {
  Target kind = checkerboard();
  kind.kind = static_cast<TargetKind>(42);
  EXPECT_TRUE(RefusedNaming(check_target(kind), "kind"));

  Target dictionary = legacy_femto_board();
  dictionary.dictionary = static_cast<MarkerDictionary>(99);
  EXPECT_TRUE(RefusedNaming(check_target(dictionary), "dictionary"));
}

// The names calib's files write: a stable lowercase one for a pattern,
// OpenCV's for a dictionary, and "unknown" outside each enum.
TEST(Target, NamesItsPatternAndDictionary) {
  EXPECT_STREQ(to_string(TargetKind::Checkerboard), "checkerboard");
  EXPECT_STREQ(to_string(TargetKind::ChArUco), "charuco");
  EXPECT_STREQ(to_string(TargetKind::SymmetricCircleGrid),
               "symmetric_circle_grid");
  EXPECT_STREQ(to_string(TargetKind::AsymmetricCircleGrid),
               "asymmetric_circle_grid");
  EXPECT_STREQ(to_string(TargetKind::AprilGrid), "april_grid");
  EXPECT_STREQ(to_string(TargetKind::Screen), "screen");
  EXPECT_STREQ(to_string(static_cast<TargetKind>(42)), "unknown");

  struct DictionaryName {
    MarkerDictionary dictionary;
    const char* name;
  };
  constexpr DictionaryName kNames[] = {
      {MarkerDictionary::None, "none"},
      {MarkerDictionary::ArUco4x4_50, "DICT_4X4_50"},
      {MarkerDictionary::ArUco4x4_100, "DICT_4X4_100"},
      {MarkerDictionary::ArUco4x4_250, "DICT_4X4_250"},
      {MarkerDictionary::ArUco4x4_1000, "DICT_4X4_1000"},
      {MarkerDictionary::ArUco5x5_50, "DICT_5X5_50"},
      {MarkerDictionary::ArUco5x5_100, "DICT_5X5_100"},
      {MarkerDictionary::ArUco5x5_250, "DICT_5X5_250"},
      {MarkerDictionary::ArUco5x5_1000, "DICT_5X5_1000"},
      {MarkerDictionary::ArUco6x6_50, "DICT_6X6_50"},
      {MarkerDictionary::ArUco6x6_100, "DICT_6X6_100"},
      {MarkerDictionary::ArUco6x6_250, "DICT_6X6_250"},
      {MarkerDictionary::ArUco6x6_1000, "DICT_6X6_1000"},
      {MarkerDictionary::ArUco7x7_50, "DICT_7X7_50"},
      {MarkerDictionary::ArUco7x7_100, "DICT_7X7_100"},
      {MarkerDictionary::ArUco7x7_250, "DICT_7X7_250"},
      {MarkerDictionary::ArUco7x7_1000, "DICT_7X7_1000"},
      {MarkerDictionary::ArUcoOriginal, "DICT_ARUCO_ORIGINAL"},
      {MarkerDictionary::AprilTag16h5, "DICT_APRILTAG_16h5"},
      {MarkerDictionary::AprilTag25h9, "DICT_APRILTAG_25h9"},
      {MarkerDictionary::AprilTag36h10, "DICT_APRILTAG_36h10"},
      {MarkerDictionary::AprilTag36h11, "DICT_APRILTAG_36h11"},
      {MarkerDictionary::ArUcoMip36h12, "DICT_ARUCO_MIP_36h12"},
  };
  static_assert(sizeof(kNames) / sizeof(kNames[0]) ==
                    static_cast<int>(MarkerDictionary::ArUcoMip36h12) + 1,
                "a MarkerDictionary is missing from the table");
  for (const DictionaryName& entry : kNames) {
    EXPECT_STREQ(to_string(entry.dictionary), entry.name);
  }
  EXPECT_STREQ(to_string(static_cast<MarkerDictionary>(99)), "unknown");
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

// A ChArUco board has a marker in every other square, from the second; an
// AprilGrid one per tag; other patterns none.
TEST(Target, CountsItsMarkers) {
  EXPECT_EQ(marker_count(legacy_femto_board()), 7);  // 5 x 3 squares
  Target even = legacy_femto_board();
  even.rows = 3;  // 5 x 4 squares
  EXPECT_EQ(marker_count(even), 10);
  Target grid;
  grid.kind = TargetKind::AprilGrid;
  grid.rows = 6;
  grid.cols = 5;
  EXPECT_EQ(marker_count(grid), 30);
  EXPECT_EQ(marker_count(checkerboard()), 0);
  Target unknown = legacy_femto_board();
  unknown.kind = static_cast<TargetKind>(99);
  EXPECT_EQ(marker_count(unknown), 0);
}

TEST(Target, PatternWithoutMarkersTakesNone) {
  for (const TargetKind kind :
       {TargetKind::Checkerboard, TargetKind::SymmetricCircleGrid,
        TargetKind::AsymmetricCircleGrid, TargetKind::Screen}) {
    Target board = checkerboard();
    board.kind = kind;
    EXPECT_TRUE(check_target(board).ok()) << static_cast<int>(kind);

    Target dictionary = board;
    dictionary.dictionary = MarkerDictionary::ArUco4x4_50;
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

// Two boards are equal only when every field is: each field changed alone
// makes them differ.
TEST(Target, EqualOnlyWhenEveryFieldIs) {
  const Target board = legacy_femto_board();
  EXPECT_TRUE(board == legacy_femto_board());
  EXPECT_FALSE(board != legacy_femto_board());
  std::vector<Target> changed(10, board);
  changed[0].kind = TargetKind::AprilGrid;
  changed[1].rows = 3;
  changed[2].cols = 5;
  changed[3].square_size_m = 0.1505;
  changed[4].marker_size_m = 0.11;
  changed[5].dictionary = MarkerDictionary::ArUco4x4_250;
  changed[6].first_marker_id = 10;
  changed[7].legacy_pattern = false;
  changed[8].square_size_tolerance_m = 0.0;
  changed[9].planarity_bow_m = 0.0005;
  for (std::size_t i = 0; i < changed.size(); ++i) {
    EXPECT_FALSE(changed[i] == board) << i;
    EXPECT_TRUE(changed[i] != board) << i;
  }
  // A tolerance known differs from one known at another value.
  Target a = board;
  Target b = board;
  a.planarity_bow_m = 0.0;
  b.planarity_bow_m = 0.001;
  EXPECT_TRUE(a != b);
  b.planarity_bow_m = 0.0;
  EXPECT_TRUE(a == b);
}

::testing::AssertionResult TagRefusedNaming(const OriginTag& tag,
                                            const std::string& field) {
  return test::RefusedNamingPath(check_origin_tag(tag),
                                 core::Status::Code::InvalidArgument,
                                 "origin_tag." + field);
}

TEST(OriginTag, IsTheRigsTag) {
  static_assert(kOriginTag.dictionary == MarkerDictionary::AprilTag36h11);
  static_assert(kOriginTag.id == 0);
  static_assert(kOriginTag.edge_m == 0.15);
  EXPECT_TRUE(check_origin_tag(kOriginTag).ok())
      << check_origin_tag(kOriginTag).message();
}

TEST(OriginTag, StartsEmptyAndIsRefused) {
  const OriginTag tag;
  EXPECT_EQ(tag.dictionary, MarkerDictionary::None);
  EXPECT_EQ(tag.id, 0);
  EXPECT_EQ(tag.edge_m, 0.0);
  EXPECT_TRUE(TagRefusedNaming(tag, "dictionary"));
}

TEST(OriginTag, NamesADictionary) {
  OriginTag tag = kOriginTag;
  tag.dictionary = MarkerDictionary::None;
  EXPECT_TRUE(TagRefusedNaming(tag, "dictionary"));
  tag.dictionary = static_cast<MarkerDictionary>(99);
  EXPECT_TRUE(TagRefusedNaming(tag, "dictionary"));
  tag.dictionary = MarkerDictionary::ArUco4x4_50;
  EXPECT_TRUE(check_origin_tag(tag).ok());
}

// The id is one of the dictionary's codes: 36h11's 587, 16h5's 30.
TEST(OriginTag, IdIsOneTheDictionaryHolds) {
  OriginTag tag = kOriginTag;
  tag.id = -1;
  EXPECT_TRUE(TagRefusedNaming(tag, "id"));
  tag.id = 586;
  EXPECT_TRUE(check_origin_tag(tag).ok());
  tag.id = 587;
  EXPECT_TRUE(TagRefusedNaming(tag, "id"));
  tag.dictionary = MarkerDictionary::AprilTag16h5;
  tag.id = 29;
  EXPECT_TRUE(check_origin_tag(tag).ok());
  tag.id = 30;
  EXPECT_TRUE(TagRefusedNaming(tag, "id"));
}

TEST(OriginTag, NeedsAFinitePositiveEdge) {
  for (const double edge : {0.0, -0.15, kNaN, kInf}) {
    OriginTag tag = kOriginTag;
    tag.edge_m = edge;
    EXPECT_TRUE(TagRefusedNaming(tag, "edge_m")) << edge;
  }
}

}  // namespace
}  // namespace volumetric_kit::calib
