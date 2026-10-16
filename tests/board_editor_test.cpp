// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The board editor: edits checked as the detector checks a board, the
// fields as the editor shows them, and the board locked after the session's
// first capture.

#include "volumetric_kit/calib/viz/board_editor.hpp"

#include <limits>
#include <string>
#include <utility>

#include <gtest/gtest.h>

namespace volumetric_kit::calib::viz {
namespace {

using Code = core::Status::Code;

BoardEditor femto_editor() {
  core::Result<BoardEditor> editor = BoardEditor::create(legacy_femto_board());
  EXPECT_TRUE(editor.ok()) << editor.status().message();
  return std::move(editor).value();
}

TEST(BoardEditor, StartsUnlockedWithItsBoard) {
  const BoardEditor editor = femto_editor();
  EXPECT_FALSE(editor.locked());
  EXPECT_FALSE(editor.square_measured());
  EXPECT_EQ(editor.board().square_size_m, 0.15);
  EXPECT_EQ(editor.board().dictionary, MarkerDictionary::ArUco4x4_100);
  const BoardEditor measured =
      BoardEditor::create(legacy_femto_board(), true).value();
  EXPECT_TRUE(measured.square_measured());
}

// The editor takes what the detector's board takes: a ChArUco board that
// check_target accepts and whose lengths OpenCV's floats hold.
TEST(BoardEditor, TakesACharucoBoardTheChecksAccept) {
  Target checkerboard;
  checkerboard.kind = TargetKind::Checkerboard;
  checkerboard.rows = 6;
  checkerboard.cols = 9;
  checkerboard.square_size_m = 0.025;
  EXPECT_EQ(BoardEditor::create(checkerboard).status().domain(),
            Code::Unsupported);
  Target bad = legacy_femto_board();
  bad.marker_size_m = 0.2;  // larger than its square
  const core::Status refused = BoardEditor::create(bad).status();
  EXPECT_EQ(refused.domain(), Code::InvalidArgument);
  EXPECT_NE(refused.message().find("target.marker_size_m"), std::string::npos)
      << refused.message();

  // check_target takes these; a float cannot hold them.
  const struct {
    double square_m;
    double marker_m;
    const char* field;
  } beyond_float[] = {
      {1e-300, 5e-301, "target.square_size_m"},      // below float's range
      {1e-39, 5e-40, "target.square_size_m"},        // a subnormal float
      {0.15, 0.15 - 1e-12, "target.marker_size_m"},  // rounds to the square
  };
  for (const auto& c : beyond_float) {
    Target board = legacy_femto_board();
    board.square_size_m = c.square_m;
    board.marker_size_m = c.marker_m;
    ASSERT_TRUE(check_target(board).ok()) << c.square_m;
    const core::Status s = BoardEditor::create(board).status();
    EXPECT_EQ(s.domain(), Code::InvalidArgument) << c.square_m;
    EXPECT_NE(s.message().find(c.field), std::string::npos) << s.message();
    BoardEditor editor = femto_editor();
    EXPECT_EQ(editor.edit(board, true).domain(), Code::InvalidArgument);
    EXPECT_EQ(editor.board().square_size_m, 0.15);
  }
}

// Before the first capture an edit is taken, and one the checks refuse
// changes nothing.
TEST(BoardEditor, EditsTheBoardBeforeTheFirstCapture) {
  BoardEditor editor = femto_editor();
  Target measured = editor.board();
  measured.square_size_m = 0.1505;
  ASSERT_TRUE(editor.edit(measured, true).ok());
  EXPECT_EQ(editor.board().square_size_m, 0.1505);
  EXPECT_TRUE(editor.square_measured());

  Target bad = editor.board();
  bad.first_marker_id = 99;  // past DICT_4X4_100's codes
  EXPECT_EQ(editor.edit(bad, false).domain(), Code::InvalidArgument);
  EXPECT_EQ(editor.board().first_marker_id, 0);
  EXPECT_TRUE(editor.square_measured());
  Target circles;
  circles.kind = TargetKind::SymmetricCircleGrid;
  circles.rows = 4;
  circles.cols = 11;
  circles.square_size_m = 0.02;
  EXPECT_EQ(editor.edit(circles, true).domain(), Code::Unsupported);
  EXPECT_EQ(editor.board().kind, TargetKind::ChArUco);
}

// The editor shows squares, one more than the corners each way, and
// millimetres.
TEST(BoardFields, ShowSquaresAndMillimetres) {
  const BoardFields fields = board_fields(legacy_femto_board());
  EXPECT_EQ(fields.squares_across, 5);
  EXPECT_EQ(fields.squares_down, 3);
  EXPECT_DOUBLE_EQ(fields.square_mm, 150.0);
  EXPECT_DOUBLE_EQ(fields.marker_mm, 112.0);

  const Target board = apply_fields(legacy_femto_board(), {9, 7, 30.0, 22.5});
  EXPECT_EQ(board.cols, 8);
  EXPECT_EQ(board.rows, 6);
  EXPECT_DOUBLE_EQ(board.square_size_m, 0.03);
  EXPECT_DOUBLE_EQ(board.marker_size_m, 0.0225);
  EXPECT_EQ(board.dictionary, MarkerDictionary::ArUco4x4_100);
  EXPECT_TRUE(board.legacy_pattern);
}

// Fields left as shown give the board back exactly, though millimetres and
// back need not round-trip.
TEST(BoardFields, KeepWhatTheUserLeftExactly) {
  Target board = legacy_femto_board();
  board.square_size_m = 0.1532;  // to millimetres and back is not exact
  board.marker_size_m = 0.1109;  // nor is this
  ASSERT_NE(board.square_size_m * 1000.0 / 1000.0, board.square_size_m);
  ASSERT_NE(board.marker_size_m * 1000.0 / 1000.0, board.marker_size_m);
  board.first_marker_id = 3;
  board.square_size_tolerance_m = 0.0002;
  const Target same = apply_fields(board, board_fields(board));
  EXPECT_EQ(same.cols, board.cols);
  EXPECT_EQ(same.rows, board.rows);
  EXPECT_EQ(same.square_size_m, board.square_size_m);
  EXPECT_EQ(same.marker_size_m, board.marker_size_m);
  EXPECT_EQ(same.first_marker_id, 3);
  EXPECT_EQ(same.square_size_tolerance_m, board.square_size_tolerance_m);
}

// Counts at int's ends are held to its range, never overflowed, and the
// editor refuses a board of fewer than 2 squares each way.
TEST(BoardFields, HoldCountsToIntsRange) {
  constexpr int kMin = std::numeric_limits<int>::min();
  constexpr int kMax = std::numeric_limits<int>::max();
  Target wide = legacy_femto_board();
  wide.cols = kMax;
  EXPECT_EQ(board_fields(wide).squares_across, kMax);

  BoardEditor editor = femto_editor();
  BoardFields fields = board_fields(editor.board());
  fields.squares_across = kMin;
  const Target least = apply_fields(editor.board(), fields);
  EXPECT_EQ(least.cols, kMin);
  EXPECT_EQ(editor.edit(least, false).domain(), Code::InvalidArgument);
  fields.squares_across = 1;
  EXPECT_EQ(apply_fields(editor.board(), fields).cols, 0);
  EXPECT_EQ(editor.edit(apply_fields(editor.board(), fields), false).domain(),
            Code::InvalidArgument);
  fields.squares_across = kMax;
  EXPECT_EQ(apply_fields(editor.board(), fields).cols, kMax - 1);
  EXPECT_EQ(editor.board().cols, 4);
}

TEST(BoardEditor, LocksTheBoardAfterTheFirstCapture) {
  BoardEditor editor = femto_editor();
  editor.lock();
  EXPECT_TRUE(editor.locked());
  Target measured = editor.board();
  measured.square_size_m = 0.1505;
  const core::Status refused = editor.edit(measured, true);
  EXPECT_EQ(refused.domain(), Code::InvalidArgument);
  EXPECT_NE(refused.message().find("locked"), std::string::npos)
      << refused.message();
  EXPECT_EQ(editor.board().square_size_m, 0.15);
  EXPECT_FALSE(editor.square_measured());
  editor.lock();  // again: nothing changes
  EXPECT_TRUE(editor.locked());
  EXPECT_EQ(editor.edit(editor.board(), false).domain(), Code::InvalidArgument);
}

}  // namespace
}  // namespace volumetric_kit::calib::viz
