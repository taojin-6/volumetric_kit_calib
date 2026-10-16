// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The board editor: edits checked as the detector and the session writer
// check a board, and the board locked after the session's first capture.

#include "volumetric_kit/calib/viz/board_editor.hpp"

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
  Target circles = editor.board();
  circles.kind = TargetKind::SymmetricCircleGrid;
  EXPECT_EQ(editor.edit(circles, true).domain(), Code::Unsupported);
  EXPECT_EQ(editor.board().kind, TargetKind::ChArUco);
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
