// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/viz/board_editor.hpp"

#include <string>

#include "imgui.h"
#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::viz {
namespace {

// The detector and the session writer take ChArUco boards alone.
core::Status check_board(const Target& board) {
  if (board.kind != TargetKind::ChArUco) {
    return core::Status::unsupported(std::string("BoardEditor: a ") +
                                     to_string(board.kind) +
                                     " board; the rig's is charuco");
  }
  return check_target(board).with_context("BoardEditor");
}

}  // namespace

BoardEditor::BoardEditor(const Target& board, bool square_measured) noexcept
    : board_(board), square_measured_(square_measured) {}

core::Result<BoardEditor> BoardEditor::create(const Target& board,
                                              bool square_measured) {
  return compute::guarded("BoardEditor", [&]() -> core::Result<BoardEditor> {
    VKC_TRY(check_board(board));
    return BoardEditor(board, square_measured);
  });
}

core::Status BoardEditor::edit(const Target& board, bool square_measured) {
  return compute::guarded("BoardEditor", [&]() -> core::Status {
    if (locked_) {
      return core::Status::invalid_argument(
          "BoardEditor: the board is locked after the first capture");
    }
    VKC_TRY(check_board(board));
    board_ = board;
    square_measured_ = square_measured;
    return {};
  });
}

core::Status draw_board_editor(BoardEditor& editor) {
  const bool locked = editor.locked();
  if (locked) ImGui::TextUnformatted("locked after the first capture");
  ImGui::BeginDisabled(locked);
  Target board = editor.board();
  bool measured = editor.square_measured();
  bool changed = false;
  constexpr ImGuiInputTextFlags kOnEnter = ImGuiInputTextFlags_EnterReturnsTrue;

  // OpenCV and the print count squares, one more than the corners each way.
  int squares[2] = {board.cols + 1, board.rows + 1};
  if (ImGui::InputInt2("squares across, down", squares, kOnEnter)) {
    board.cols = squares[0] - 1;
    board.rows = squares[1] - 1;
    changed = true;
  }
  double square_mm = board.square_size_m * 1000.0;
  if (ImGui::InputDouble("square (mm)", &square_mm, 0.0, 0.0, "%.2f",
                         kOnEnter)) {
    board.square_size_m = square_mm / 1000.0;
    changed = true;
  }
  double marker_mm = board.marker_size_m * 1000.0;
  if (ImGui::InputDouble("marker (mm)", &marker_mm, 0.0, 0.0, "%.2f",
                         kOnEnter)) {
    board.marker_size_m = marker_mm / 1000.0;
    changed = true;
  }
  if (ImGui::BeginCombo("dictionary", to_string(board.dictionary))) {
    // Every dictionary but None, which a ChArUco board cannot take:
    // ArUcoMip36h12 is the enum's last.
    for (int d = static_cast<int>(MarkerDictionary::ArUco4x4_50);
         d <= static_cast<int>(MarkerDictionary::ArUcoMip36h12); ++d) {
      const auto dictionary = static_cast<MarkerDictionary>(d);
      const bool selected = dictionary == board.dictionary;
      if (ImGui::Selectable(to_string(dictionary), selected) && !selected) {
        board.dictionary = dictionary;
        changed = true;
      }
      if (selected) ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }
  if (ImGui::InputInt("first marker id", &board.first_marker_id, 1, 10,
                      kOnEnter)) {
    changed = true;
  }
  changed |= ImGui::Checkbox("legacy layout", &board.legacy_pattern);
  changed |= ImGui::Checkbox("square measured", &measured);
  ImGui::EndDisabled();

  if (!changed || locked) return {};
  return editor.edit(board, measured);
}

}  // namespace volumetric_kit::calib::viz
