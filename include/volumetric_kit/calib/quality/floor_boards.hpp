// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file floor_boards.hpp
/// @brief The solved boards that lie on the floor of the origin tag's world,
///        each with its tilt from the tag's z and its height above the tag's
///        plane: what the floor says of the tag's z, reported with no
///        verdict.

#include <cstdint>
#include <vector>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/quality/export.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::quality {

/// @brief What counts as a board on the floor.
///
/// @code
/// FloorBoardOptions options;
/// options.max_height_m = 0.01;  // only boards within 10 mm of the floor
/// @endcode
struct FloorBoardOptions {
  /// The printed face's normal from the world's +z, at most (degrees).
  double max_angle_deg = 5.0;
  /// The board's centre from the plane z = 0, at most (metres).
  double max_height_m = 0.02;
};

/// @brief A solved board that lies on the floor.
///
/// @code
/// for (const FloorBoard& b : floor) log(b.capture, b.angle_deg, b.height_m);
/// @endcode
struct FloorBoard {
  std::int64_t capture = 0;  ///< Its capture (`solve::SolvedBoard::capture`).
  /// Its printed face's normal (`compute::face_normal`) from the world's +z
  /// (degrees).
  double angle_deg = 0.0;
  /// Its centre's z (`compute::centre_in`), its height above the plane
  /// z = 0 (metres).
  double height_m = 0.0;
};

/// @brief The solved boards that lie on the floor of a world whose +z is up
///        and whose floor is z = 0, as the origin tag's is.
/// @param board            The board the solve placed.
/// @param boards           The solve's boards (`solve::RigSolution::boards`).
/// @param boards_to_world  Each one's T_wb in the world
///                         (`solve::AnchoredRig::boards_to_world`), in
///                         @p boards' order.
/// @param options          What counts as on the floor.
/// @return The boards whose face is within `max_angle_deg` of +z and whose
///         centre is within `max_height_m` of z = 0, in @p boards' order;
///         `InvalidArgument` for poses not as many as @p boards, a pose that
///         is not rigid, or an option that is not positive and finite; or
///         `OutOfMemory`. Each message starts `floor_boards: `.
///
/// @code
/// VKC_ASSIGN(const std::vector<FloorBoard> floor,
///            floor_boards(board, solution.boards, rig.boards_to_world));
/// @endcode
VC_QUALITY_API core::Result<std::vector<FloorBoard>> floor_boards(
    const compute::Board& board, const std::vector<solve::SolvedBoard>& boards,
    const std::vector<recon::camera::Mat4d>& boards_to_world,
    const FloorBoardOptions& options = {});

}  // namespace volumetric_kit::calib::quality
