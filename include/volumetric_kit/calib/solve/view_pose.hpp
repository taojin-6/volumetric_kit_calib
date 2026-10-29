// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file view_pose.hpp
/// @brief One view's own pose of the board: IPPE's solutions, how well each
///        fits, and how far the board is turned from the camera.
///
/// Poses are sensor's `Mat4d`s, named by what they map: `board_to_camera` is
/// OpenCV's `[R | t]` from the board's frame (@ref compute::Board) to the
/// camera's (+x right, +y down, +z forward).

#include <optional>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/solve/export.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"

namespace volumetric_kit::calib::solve {

/// @brief One pose of the board that a view allows, and how well it fits.
///
/// @code
/// const PoseCandidate& pose = view.best;
/// if (pose.rms_px > 5.0) return;  // the view does not fit its own pose
/// @endcode
struct PoseCandidate {
  sensor::camera::Mat4d board_to_camera{1.0};  ///< T_cb.
  /// The RMS of each corner's Euclidean reprojection error (pixels).
  double rms_px = 0.0;
};

/// @brief The poses of the board a view allows: those of IPPE's two
///        solutions that are finite and put every corner seen in front of the
///        camera, the better fit first.
///
/// A planar board seen nearly face-on, or small, fits two poses almost
/// equally well, mirror images about the line of sight. @ref ambiguity says
/// how far apart their fits are.
///
/// @code
/// VKC_ASSIGN(const ViewPose pose, view_pose(cam, board, view));
/// if (pose.ambiguity() < 2.0) return;  // either pose could be the board's
/// const sensor::camera::Mat4d& board_to_camera = pose.best.board_to_camera;
/// @endcode
struct VC_SOLVE_API ViewPose {
  PoseCandidate best;                        ///< The better fit.
  std::optional<PoseCandidate> alternative;  ///< The other, if it is valid.

  /// @return The alternative's RMS over the best's (at least 1): infinite
  ///         without an alternative, or when only the best fits exactly;
  ///         1 when both fit exactly.
  double ambiguity() const noexcept;
};

/// @brief The poses of @p board that @p view allows: OpenCV's
///        `solvePnPGeneric` with `SOLVEPNP_IPPE` on the corners' rays
///        (@ref compute::undistort_points), each fitted through the camera's
///        whole lens.
/// @param camera  The camera, its size the image the view was detected in.
/// @param board   The board seen.
/// @param view    The corners seen; its weights are not used.
/// @return The poses; `InvalidArgument` for a view
///         @ref compute::usable_for_pose does not find usable, its message
///         saying why (@ref compute::describe); `Numerical` when no solution
///         is finite with every corner in front of the camera; a `Backend`
///         error naming `solvePnPGeneric` when IPPE fails, which a usable view
///         does only when IPPE throws (`solvePnPGeneric` swallows that,
///         running out of memory included, and keeps none or half of the
///         solutions); or what OpenCV threw (@ref compute::guarded).
VC_SOLVE_API core::Result<ViewPose> view_pose(const compute::CvCamera& camera,
                                              const compute::Board& board,
                                              const Observation& view);

/// @brief How far a board is turned from facing the camera: the angle
///        between its printed face's normal and the line from its centre to
///        the camera.
/// @param board            The board.
/// @param board_to_camera  Its pose, T_cb.
/// @return The angle (degrees): 0 face-on, 90 edge-on, and above 90 when the
///         camera sees the board's back.
VC_SOLVE_API double tilt_deg(
    const compute::Board& board,
    const sensor::camera::Mat4d& board_to_camera) noexcept;

}  // namespace volumetric_kit::calib::solve
