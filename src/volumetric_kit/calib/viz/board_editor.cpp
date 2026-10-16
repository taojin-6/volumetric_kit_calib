// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/viz/board_editor.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

#include "imgui.h"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::viz {
namespace {

// A board the detector takes: one its Board builds, the rule's one home.
core::Status check_board(const Target& board) {
  return compute::Board::create(board).status().with_context("BoardEditor");
}

// `value` held to int's range.
int to_int(std::int64_t value) noexcept {
  return static_cast<int>(std::clamp<std::int64_t>(
      value, std::numeric_limits<int>::min(), std::numeric_limits<int>::max()));
}

// A size in metres from the millimetres shown; the board's own when the
// field is as shown, as millimetres and back need not round-trip exactly.
double to_metres(double mm, double board_m) noexcept {
  return mm == board_m * 1000.0 ? board_m : mm / 1000.0;
}

}  // namespace

BoardFields board_fields(const Target& board) noexcept {
  // OpenCV and the print count squares, one more than the corners each way.
  return {to_int(std::int64_t{board.cols} + 1),
          to_int(std::int64_t{board.rows} + 1), board.square_size_m * 1000.0,
          board.marker_size_m * 1000.0};
}

Target apply_fields(const Target& board, const BoardFields& fields) noexcept {
  Target edited = board;
  edited.cols = to_int(std::int64_t{fields.squares_across} - 1);
  edited.rows = to_int(std::int64_t{fields.squares_down} - 1);
  edited.square_size_m = to_metres(fields.square_mm, board.square_size_m);
  edited.marker_size_m = to_metres(fields.marker_mm, board.marker_size_m);
  return edited;
}

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
  ImGui::PushID(&editor);
  if (locked) ImGui::TextUnformatted("locked after the first capture");
  ImGui::BeginDisabled(locked);
  BoardFields fields = board_fields(editor.board());
  bool measured = editor.square_measured();
  bool changed = false;
  constexpr ImGuiInputTextFlags kOnEnter = ImGuiInputTextFlags_EnterReturnsTrue;

  int squares[2] = {fields.squares_across, fields.squares_down};
  if (ImGui::InputInt2("squares across, down", squares, kOnEnter)) {
    fields.squares_across = squares[0];
    fields.squares_down = squares[1];
    changed = true;
  }
  changed |= ImGui::InputDouble("square (mm)", &fields.square_mm, 0.0, 0.0,
                                "%.2f", kOnEnter);
  changed |= ImGui::InputDouble("marker (mm)", &fields.marker_mm, 0.0, 0.0,
                                "%.2f", kOnEnter);
  Target board = apply_fields(editor.board(), fields);
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
  ImGui::PopID();

  if (!changed || locked) return {};
  return editor.edit(board, measured);
}

}  // namespace volumetric_kit::calib::viz
