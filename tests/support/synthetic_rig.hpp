// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file synthetic_rig.hpp
/// @brief The one synthetic source for calib's tests: lenses like the Femto
///        Mega's, a four-camera rig, a plan of board placements, noisy
///        detections of it, and an exact-inverse renderer of the board.
///
/// World: z up, the floor at z = 0. Poses are recon's `Mat4d`s, named by what
/// they map (`world_to_camera` is OpenCV's extrinsic). Projections are exact:
/// a point is moved into the camera in double, then through the lens with
/// `cv::projectPoints`, with no Rodrigues round trip.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::test {

/// @brief A 4K colour lens with a Femto Mega's magnitudes: Brown-Conrady with
///        small k4-k6, so the rational denominator is exercised.
/// @return The model, which recon's `check_camera_model` accepts.
recon::camera::CameraModel color_like_lens();

/// @brief A 640x576 depth lens like the Femto Mega's NFOV unbinned one:
///        large k1 and k4 that nearly cancel.
/// @return The model, which recon's `check_camera_model` accepts.
recon::camera::CameraModel depth_like_lens();

/// @brief A 1280x720 lens with k1 = 0.4 alone: strong enough that
///        `cv::undistortPoints`' default 5 iterations leave 1.5 px at the
///        image's corners.
/// @return The model, which recon's `check_camera_model` accepts.
recon::camera::CameraModel strongly_distorted_lens();

/// @brief One colour camera of the synthetic rig.
struct SyntheticCamera {
  std::string id;                        ///< A stand-in serial.
  recon::camera::CameraModel model;      ///< Its lens, at 3840x2160.
  recon::camera::Mat4d world_to_camera;  ///< T_cw.
};

/// @brief Four 4K colour cameras about 2.5 m up in an outside-in ring of about
///        1.9 m, each looking near the middle of the floor with its own
///        variation of @ref color_like_lens; camera 0 is the reference.
/// @return The cameras.
std::vector<SyntheticCamera> synthetic_rig();

/// @brief Where a camera images points.
/// @param model      The camera's lens.
/// @param to_camera  The points' frame to the camera's.
/// @param points     The points.
/// @return Each point's pixel, OpenCV's convention (centres at integers).
std::vector<cv::Point2d> project(const recon::camera::CameraModel& model,
                                 const recon::camera::Mat4d& to_camera,
                                 const std::vector<cv::Point3d>& points);

/// @brief Points around the chessboard's outer edge, or a band outside it.
/// @param board     The board.
/// @param margin_m  How far outside the outer edge (metres); 0 for the edge.
/// @param per_side  Points on each side, starting at its corner.
/// @return The points in the board frame, on z = 0.
std::vector<cv::Point3d> board_outline(const compute::Board& board,
                                       double margin_m, int per_side);

/// @brief One placement of the board, and who sees it.
struct Placement {
  recon::camera::Mat4d board_to_world;  ///< T_wb.
  /// The rig's cameras (indices) that see the whole board (@ref PlanOptions).
  std::vector<std::size_t> cameras;
};

/// @brief What @ref synthetic_plan places, and what counts as seeing a board:
///        every bound holds at 16 points a side of @ref board_outline.
struct PlanOptions {
  std::size_t count = 60;       ///< Placements to make.
  std::uint32_t seed = 1;       ///< The random sequence's seed.
  std::size_t min_cameras = 2;  ///< Fewer cameras seeing a board drop it.
  /// The printed face's normal to the ray to the board's centre, at most.
  double max_tilt_deg = 70.0;
  double margin_px = 40.0;   ///< The outline this far inside the image.
  double min_depth_m = 0.3;  ///< The outline at least this far in front.
};

/// @brief Placements as a person makes them: 40 % on the floor, the rest on
///        stands at 0.5, 1.0 and 1.5 m, half of those tilted 5-15 degrees
///        toward a camera (none with no cameras); each at a random yaw, kept
///        when at least `min_cameras` see it.
/// @param rig      The cameras.
/// @param board    The board placed.
/// @param options  How many, and what counts as seeing one.
/// @return Up to `options.count` placements, the same for the same options.
std::vector<Placement> synthetic_plan(const std::vector<SyntheticCamera>& rig,
                                      const compute::Board& board,
                                      const PlanOptions& options = {});

/// @brief How @ref observe detects the board.
struct ObserveOptions {
  double noise_px = 0.3;   ///< Gaussian noise on each axis (pixels, sigma).
  std::uint32_t seed = 1;  ///< The noise's seed.
};

/// @brief What the rig's cameras detect of a plan: every corner of each
///        placement's board in each camera that sees it, where it projects
///        (@ref project), plus noise.
/// @param rig      The cameras.
/// @param board    The board placed.
/// @param plan     The placements.
/// @param options  The noise.
/// @return One observation for each placement and camera, in the plan's
///         order and then the cameras': `camera_id` the camera's id,
///         `frame_id` the placement's index, the corners in id order, each
///         weighted `1 / noise_px^2` (1 without noise). The same for the same
///         options.
std::vector<Observation> observe(const std::vector<SyntheticCamera>& rig,
                                 const compute::Board& board,
                                 const std::vector<Placement>& plan,
                                 const ObserveOptions& options = {});

/// @brief How @ref render_board draws.
struct RenderOptions {
  /// The board texture's resolution; the board's edges should fall on its
  /// texel edges, as the rig's board's do at 6000.
  double texture_px_per_m = 6000.0;
  double margin_m = 0.02;     ///< White margin around the chessboard.
  float black = 30.0f;        ///< A black square's level.
  float white = 220.0f;       ///< A white square's level.
  float background = 110.0f;  ///< Everything off the board.
  int supersampling = 3;      ///< Samples per pixel on each axis.
};

/// @brief The board as @p model images it at @p board_to_camera: a clean,
///        geometric render (no blur, noise or compression).
///
/// Each sample is a pixel's ray (@ref compute::undistort_points) meeting the
/// board's plane, so the lens is inverted exactly rather than approximated by a
/// forward warp. A sample is the board's mean over the bounding box of its
/// footprint there, from a summed-area table of the texture, so a pixel is
/// close to the board's mean over it whatever the texture's resolution.
/// Accuracy at the defaults (`SyntheticRig.PlacesSquaresWhereTheyProject`):
/// an edge sits where it projects to within 0.005 px on average along it,
/// front-on, turned or tilted, and within 0.02 px on any one pixel row; a
/// slanted edge's spread on a row narrows with more supersampling.
/// @param model            The lens.
/// @param board_to_camera  T_cb.
/// @param board            The board drawn.
/// @param options          Levels and sampling.
/// @return An 8-bit luma image of `model.size`.
cv::Mat render_board(const recon::camera::CameraModel& model,
                     const recon::camera::Mat4d& board_to_camera,
                     const compute::Board& board,
                     const RenderOptions& options = {});

}  // namespace volumetric_kit::calib::test
