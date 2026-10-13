// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file spanning_tree.hpp
/// @brief Which captures the rig's cameras share, and the tree along which
///        the cameras are placed from the reference.

#include <cstddef>
#include <string>
#include <vector>

#include "volumetric_kit/calib/solve/export.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::solve {

/// @brief For each pair of cameras, how many captures hold a view of the
///        board in both: K x K and symmetric, its diagonal each camera's own
///        count, indexed `[a][b]`.
using Covisibility = std::vector<std::vector<int>>;

/// @brief The captures each pair of cameras shares.
/// @param cameras   How many cameras the rig has (K).
/// @param captures  For each capture, the cameras (indices below K) with a
///                  view of the board in it.
/// @return The counts; `InvalidArgument` for a camera index of K or more,
///         or one listed twice in a capture.
///
/// @code
/// // Capture 0 seen by cameras 0 and 1, capture 1 by 1 and 2.
/// VKC_ASSIGN(const Covisibility shared, covisibility(3, {{0, 1}, {1, 2}}));
/// // shared[0][1] == 1, shared[1][1] == 2, shared[0][2] == 0
/// @endcode
VC_SOLVE_API core::Result<Covisibility> covisibility(
    std::size_t cameras, const std::vector<std::vector<std::size_t>>& captures);

/// @brief One link of a @ref SpanningTree: the camera it places, and the
///        camera it is placed from.
struct TreeLink {
  std::size_t camera = 0;  ///< The camera the link places.
  std::size_t parent = 0;  ///< The camera it is placed from.
  int shared = 0;          ///< The captures the two share.
  /// Fewer shared captures than `weak_below`: the link holds, but each
  /// capture short of it costs accuracy (millimetres per link).
  bool weak = false;
};

/// @brief The tree that places every camera from the reference through the
///        links that share the most captures.
///
/// @code
/// VKC_ASSIGN(const SpanningTree tree,
///            spanning_tree(shared, ids, reference));
/// for (const TreeLink& link : tree.links) {
///   if (link.weak) warn(ids[link.camera], link.shared);
/// }
/// @endcode
struct SpanningTree {
  std::size_t reference = 0;  ///< The root: the camera the world is.
  /// One link for each other camera, each after the link that places its
  /// parent: from the reference outwards, breadth first, siblings in the
  /// order of their ids.
  std::vector<TreeLink> links;
};

/// @brief The maximum spanning tree of @p covisibility, rooted at
///        @p reference: OpenCV's `buildMST` (Prim) on weights of minus the
///        shared captures.
///
/// Of two links that share as many captures, the tree takes the one whose
/// pair of ids sorts first (the smaller id of each pair, then the larger), so
/// the tree depends on the cameras' ids and not on their order.
/// @param covisibility  The captures each pair shares (@ref covisibility).
/// @param ids           Each camera's id, unique, in @p covisibility's order.
/// @param reference     The root's index.
/// @param weak_below    A link sharing fewer captures than this is weak.
/// @return The tree; `InvalidArgument` naming every camera that shares no
///         capture with the reference's tree, or for inputs that do not
///         agree (sizes, a matrix that is not symmetric or has a negative
///         count, repeated ids, a reference outside the rig, a negative
///         @p weak_below); or what OpenCV threw (@ref compute::guarded).
VC_SOLVE_API core::Result<SpanningTree> spanning_tree(
    const Covisibility& covisibility, const std::vector<std::string>& ids,
    std::size_t reference, int weak_below = 5);

}  // namespace volumetric_kit::calib::solve
