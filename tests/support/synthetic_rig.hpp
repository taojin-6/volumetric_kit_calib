// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file synthetic_rig.hpp
/// @brief The one synthetic source for calib's tests: lenses like the Femto
///        Mega's, a four-camera rig, a plan of board placements, noisy
///        detections of it, an exact-inverse renderer of the board, a marker
///        drawn through a homography, and the origin tag on the floor and
///        what the rig sees of it.
///
/// World: z up, the floor at z = 0. Poses are recon's `Mat4d`s, named by what
/// they map (`world_to_camera` is OpenCV's extrinsic). Projections are exact:
/// a point is moved into the camera in double, then through the lens with
/// `cv::projectPoints`, with no Rodrigues round trip. @ref camera_image adds
/// a camera's blur and noise to a render.

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/detection.hpp"
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

/// @brief A marker lying face up on the floor, as the rig's origin tag does.
struct FloorTag {
  double edge_m = 0.15;  ///< Its black border's edge (metres), kOriginTag's.
  /// Its centre in the world; z = 0 lays it on the floor.
  recon::camera::Vec3d centre{0.0};
  double yaw_deg = 0.0;  ///< Its x axis's turn from the world's, about z.
};

/// @brief @p tag's corners in the world, in `DetectedMarker`'s order: OpenCV's
///        marker frame, x from the first corner to the second and z out of
///        its face, up. At the defaults that frame is the world's.
/// @param tag  The tag.
/// @return The corners.
std::array<cv::Point3d, 4> floor_tag_corners(const FloorTag& tag = {});

/// @brief One camera's sighting of a marker in one capture.
struct MarkerSighting {
  std::size_t camera = 0;    ///< The camera, by index in the rig.
  std::int64_t capture = 0;  ///< The capture.
  DetectedMarker marker;     ///< Its id and corners in the image.
};

/// @brief What the rig's cameras see of a marker that stays put for
///        @p captures captures.
///
/// A camera sees it when it faces the camera and its corners are in front of
/// the camera and `margin_px` inside its image; it then sees it in every
/// capture, at its corners' projections (@ref project) plus Gaussian noise
/// on each axis. Unlike @ref observe's, the noise is the same with every
/// standard library (Box-Muller over `std::mt19937`).
/// @param rig        The cameras.
/// @param id         The marker's id.
/// @param corners    Its corners in the world, in `DetectedMarker`'s order.
/// @param captures   Captures 0 to `captures - 1`.
/// @param options    The noise's sigma and seed.
/// @param margin_px  How far inside the image each corner must be.
/// @return The sightings by capture, then camera; the same for the same
///         arguments.
std::vector<MarkerSighting> observe_marker(
    const std::vector<SyntheticCamera>& rig, int id,
    const std::array<cv::Point3d, 4>& corners, std::size_t captures,
    const ObserveOptions& options = {}, double margin_px = 40.0);

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

/// @brief How @ref draw_marker draws.
struct MarkerOptions {
  float black = 30.0f;   ///< The black cells' level.
  float white = 220.0f;  ///< The white cells' and the margin's level.
  /// The white margin around the black border, in the marker's cells.
  double margin_cells = 2.0;
  int supersampling = 8;  ///< Samples per pixel on each axis.
};

/// @brief A printed marker drawn into @p image through a homography: a clean,
///        geometric render (no blur, noise or compression).
///
/// The print's frame puts the black border's outer corners at (0, 0),
/// (1, 0), (1, 1) and (0, 1), x right and y down as printed. Each pixel the
/// print reaches is the mean of S x S samples of OpenCV's marker image
/// (`generateImageMarker`, 100 px a cell), warped bilinearly; the border's
/// edges fall on that image's texel edges, so they sit where the homography
/// puts them, within 0.01 px on average along an edge and 0.02 px on any
/// pixel row (`SyntheticRig.DrawsAMarkerWhereItsHomographyPutsIt`). Pixels
/// it does not reach keep their level.
/// @param image           An 8-bit luma image, drawn into.
/// @param dictionary      The marker's dictionary, not `None`.
/// @param id              Its id there.
/// @param print_to_image  The print's frame to the image's pixels.
/// @param options         Levels, margin and sampling.
/// @return The black border's outer corners in the image's pixels, clockwise
///         from the top-left as printed: where a detector should find them.
std::array<cv::Point2d, 4> draw_marker(cv::Mat& image,
                                       MarkerDictionary dictionary, int id,
                                       const cv::Matx33d& print_to_image,
                                       const MarkerOptions& options = {});

/// @brief The rig's board, @ref legacy_femto_board, as compute builds it.
/// @return The board.
compute::Board femto_board();

/// @brief @p board in front of a camera: its centre at @p centre, its printed
///        face turned @p tilt_deg about its x axis from facing the camera,
///        then rolled @p roll_deg about the camera's z axis.
/// @param board     The board.
/// @param centre    Where the chessboard's centre is in the camera (metres).
/// @param tilt_deg  The turn about the board's x axis (degrees).
/// @param roll_deg  The roll about the camera's z axis (degrees).
/// @return T_cb.
recon::camera::Mat4d board_facing_camera(const compute::Board& board,
                                         const recon::camera::Vec3d& centre,
                                         double tilt_deg, double roll_deg);

/// @brief What @ref camera_image adds to a render.
struct CameraOptions {
  double blur_px = 0.8;  ///< The optics' Gaussian blur (sigma, px); 0 for none.
  /// Gaussian noise on each pixel (sigma, 8-bit levels); 0 for none.
  double noise_dn = 2.5;
  std::uint64_t seed = 1;  ///< The noise's seed.
};

/// @brief A camera's picture of @p render: blurred by its optics, then
///        noisy, rounded to 8 bits.
/// @param render   An 8-bit luma image, as @ref render_board draws.
/// @param options  The blur and noise.
/// @return A new image of the render's size, the same for the same options.
cv::Mat camera_image(const cv::Mat& render, const CameraOptions& options = {});

}  // namespace volumetric_kit::calib::test
