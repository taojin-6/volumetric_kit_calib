// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file board_editor.hpp
/// @brief The board's description, edited before a session's first capture
///        and locked from then on: every capture's detections, and the solve,
///        rest on it.

#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/viz/export.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::viz {

/// @brief The ChArUco board a session is captured with, and whether its
///        square size was measured: editable until @ref lock, at the
///        session's first capture.
///
/// Every edit is checked by building the detector's board
/// (`compute::Board::create`), so the board it holds is always one the
/// detector, and so the session writer, take. A value; not thread-safe.
///
/// @code
/// VKC_ASSIGN(BoardEditor editor, BoardEditor::create(legacy_femto_board()));
/// Target measured = editor.board();
/// measured.square_size_m = 0.1505;  // four squares with calipers, / 4
/// VKC_TRY(editor.edit(measured, true));
/// editor.lock();  // the first capture is taken
/// info.board = editor.board();
/// info.square_measured = editor.square_measured();
/// @endcode
class VC_VIZ_API BoardEditor {
 public:
  /// @brief An editor holding @p board, unlocked.
  /// @param board            A ChArUco board the detector takes.
  /// @param square_measured  Whether its square size was measured, rather
  ///                         than the print's nominal size.
  /// @return The editor; what `compute::Board::create` refuses
  ///         (`InvalidArgument` naming the field, or `Unsupported` for a
  ///         board that is not ChArUco); or `OutOfMemory`.
  static core::Result<BoardEditor> create(const Target& board,
                                          bool square_measured = false);

  /// @return The board.
  const Target& board() const noexcept { return board_; }
  /// @return Whether its square size was measured.
  bool square_measured() const noexcept { return square_measured_; }
  /// @return Whether the board is locked: a capture has been taken.
  bool locked() const noexcept { return locked_; }

  /// @brief Replace the board, before the first capture.
  /// @param board            The new board, as @ref create takes it.
  /// @param square_measured  Whether its square size was measured.
  /// @return OK; `InvalidArgument` once locked; what @ref create refuses; or
  ///         `OutOfMemory`. A refused edit changes nothing.
  core::Status edit(const Target& board, bool square_measured);

  /// @brief Lock the board: the session's first capture is taken. Locking
  ///        again does nothing.
  void lock() noexcept { locked_ = true; }

 private:
  BoardEditor(const Target& board, bool square_measured) noexcept;

  Target board_;
  bool square_measured_ = false;
  bool locked_ = false;
};

/// @brief A board's fields as @ref draw_board_editor shows them: squares, as
///        OpenCV and the print count them, rather than corners, and
///        millimetres rather than metres.
///
/// @code
/// BoardFields fields = board_fields(editor.board());
/// fields.square_mm = 150.5;  // four squares with calipers, / 4
/// VKC_TRY(editor.edit(apply_fields(editor.board(), fields), true));
/// @endcode
struct BoardFields {
  int squares_across = 0;  ///< Squares across: the corners across plus one.
  int squares_down = 0;    ///< Squares down: the corners down plus one.
  double square_mm = 0.0;  ///< A square's edge (millimetres).
  double marker_mm = 0.0;  ///< A marker's edge (millimetres).
};

/// @brief @p board's fields as the editor shows them.
/// @param board  The board.
/// @return Its squares across and down, held to `int`'s range, and its square
///         and marker sizes in millimetres.
///
/// @code
/// const BoardFields fields = board_fields(legacy_femto_board());  // 5 x 3
/// @endcode
VC_VIZ_API BoardFields board_fields(const Target& board) noexcept;

/// @brief @p board with @p fields applied: one corner fewer than squares each
///        way, held to `int`'s range, and the sizes in metres; a size left as
///        @ref board_fields gave it, and every other field, kept exactly. The
///        result is unchecked: @ref BoardEditor::edit checks it, and refuses
///        fewer than 2 squares across or down.
/// @param board   The board the fields edit.
/// @param fields  The fields, as the user left them.
/// @return The board.
///
/// @code
/// VKC_TRY(editor.edit(apply_fields(editor.board(), fields), measured));
/// @endcode
VC_VIZ_API Target apply_fields(const Target& board,
                               const BoardFields& fields) noexcept;

/// @brief Draw @p editor's fields as ImGui widgets -- the squares across and
///        down, the square and marker sizes (millimetres), the dictionary,
///        the first marker's id, the legacy layout, and whether the square
///        was measured -- and apply what the user changes through
///        @ref BoardEditor::edit. Once locked, the widgets are disabled. The
///        widgets' ImGui ids are scoped to @p editor, so two editors can
///        share a window.
/// @param editor  The editor.
/// @return OK when nothing changed or the change was taken; else why
///         @ref BoardEditor::edit refused it, for the caller to show.
/// @pre An ImGui frame is active, with its context current.
///
/// @code
/// ImGui::Begin("Board");
/// if (const vkc::Status s = draw_board_editor(editor); !s.ok()) {
///   show(s.message());
/// }
/// ImGui::End();
/// @endcode
VC_VIZ_API core::Status draw_board_editor(BoardEditor& editor);

}  // namespace volumetric_kit::calib::viz
