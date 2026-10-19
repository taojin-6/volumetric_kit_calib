// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file world_tag.hpp
/// @brief The world at the origin tag: the tag located from the solved rig,
///        its poses fixed, and the rig's outputs moved into the tag's frame.
///
/// The tag's frame is OpenCV's marker frame, centred on the tag: x from
/// @ref DetectedMarker's corner 0 to its corner 1, y from corner 3 to
/// corner 0, z out of the printed face, so up for a tag lying face up on the
/// floor, whose plane is then z = 0. In it the corners, in
/// @ref DetectedMarker's order, sit at (-s/2, s/2, 0), (s/2, s/2, 0),
/// (s/2, -s/2, 0) and (-s/2, -s/2, 0), s the tag's edge.
///
/// @ref origin_tag_frame works in the solution's gauge, the reference
/// camera's; @ref anchor_rig moves the outputs into the tag's world and never
/// the solution, so whatever is measured from the solution stays in the
/// reference's gauge.

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/export.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::solve {

/// @brief One camera's sighting of the origin tag in one capture.
///
/// @code
/// VKC_ASSIGN(const detect::MarkerSearch search,
///            detect::detect_tag(luma, kOriginTag));
/// if (search.found) {
///   views.push_back({cam.id, capture, cam.model.size, *search.found});
/// }
/// @endcode
struct TagView {
  std::string camera;        ///< Its camera's id, as @ref RigCamera::id.
  std::int64_t capture = 0;  ///< Its capture, as `Observation::frame_id`.
  /// The image the tag was found in: its camera's lens's size.
  recon::camera::ImageSize image_size;
  /// The tag's id and corners in the image's pixels, as `detect::detect_tag`
  /// finds them.
  DetectedMarker marker;
};

/// @brief The thresholds @ref origin_tag_frame applies; the outlier rule's
///        are `BundleAdjustOptions{}`'s.
///
/// @code
/// TagFrameOptions options;
/// options.min_cameras = 3;  // a world only from 3 cameras or more
/// @endcode
struct TagFrameOptions {
  /// Huber's delta on each corner's Euclidean residual (pixels).
  double huber_px = 1.0;
  /// Solves after outlier views are dropped, at most; 0 drops none.
  int outlier_rounds = 3;
  /// A view whose worst corner's residual is above this (pixels), and above
  /// `outlier_mads` robust deviations, is an outlier.
  double outlier_px = 3.0;
  /// The robust deviations, `1.4826` times the median absolute deviation of
  /// the kept views' residual components, above which a view's worst corner
  /// makes it an outlier, if above `outlier_px` too.
  double outlier_mads = 4.0;
  /// The cameras that must keep a view for a world; at least 2.
  int min_cameras = 2;
  /// The mean edge's departure from the tag's printed edge, as a fraction of
  /// it, above which @ref TagWorld::edge_warning is set.
  double edge_warn_fraction = 0.02;
};

/// @brief The RMS of a tag world's corner residuals (@ref TagWorld::rms_px)
///        above which @ref origin_tag_frame refuses it (pixels): a
///        @ref TagMiss::PoorFit, the reference camera staying the origin.
///
/// Cameras that disagree on where the tag is -- a pose or a lens off, or a
/// tag moved between captures -- leave their rays apart. With 3 cameras or
/// more the outlier rounds drop the views that disagree; with 2 they cannot
/// tell which is wrong, and their bound grows with the disagreement. On the
/// synthetic rig, at 0.3 px of noise a world fits at about 0.4 px RMS; two
/// cameras whose corners are (3, 1.5) px apart fit at 1.47 px, and
/// (5.6, 2.8) px apart at 3.08 px.
///
/// @code
/// if (origin.miss == TagMiss::PoorFit) warn(origin.reason);  // names the RMS
/// @endcode
inline constexpr double kMaxTagWorldRmsPx = 3.0;

/// @brief Why @ref origin_tag_frame gave no tag world.
enum class TagMiss {
  NotSeen,    ///< No camera saw the tag.
  OneCamera,  ///< One camera alone saw it (@ref WorldOrigin::camera).
  /// Fewer than `min_cameras` cameras saw it, or keep a view after the
  /// outlier rounds.
  FewCameras,
  /// A camera that saw it is not above its face (@ref WorldOrigin::camera,
  /// the first such by id): the tag lies face down, or its corners are
  /// mirrored.
  CameraBelowTag,
  /// The corners could not be placed: rays that meet at no point, a corner
  /// not in front of a camera that saw it, a solve that did not converge, or
  /// corners that make no square.
  Numerical,
  /// The corners were placed, but fit the views used worse than
  /// @ref kMaxTagWorldRmsPx: the reason gives the RMS.
  PoorFit,
};

/// @brief The name calib's results files give @p miss.
/// @param miss  A miss.
/// @return `"not_seen"`, `"one_camera"`, `"few_cameras"`,
///         `"camera_below_tag"`, `"numerical"` or `"poor_fit"`; `"unknown"`
///         for a value @ref TagMiss does not name.
///
/// @code
/// if (!origin.tag) log(to_string(origin.miss), origin.reason);
/// @endcode
VC_SOLVE_API const char* to_string(TagMiss miss) noexcept;

/// @brief One view's fit at the tag's corners.
///
/// @code
/// for (const TagViewFit& v : world.views) {
///   if (!v.used) report(v.camera, v.capture, v.worst_px, v.bound_px);
/// }
/// @endcode
struct TagViewFit {
  std::string camera;        ///< Its camera's id.
  std::int64_t capture = 0;  ///< Its capture.
  /// The RMS of its corners' residuals at the corners placed (pixels).
  double rms_px = 0.0;
  double worst_px = 0.0;  ///< Its worst corner's residual there (pixels).
  bool used = true;       ///< False when it was dropped as an outlier.
  /// The solves run before it was dropped; 0 for a view used.
  int round = 0;
  /// The bound its worst corner broke (pixels); 0 for a view used.
  double bound_px = 0.0;
};

/// @brief One camera of the rig, as the tag's world sees it.
///
/// @code
/// for (const TagCamera& c : world.cameras) log(c.id, c.height_m);
/// @endcode
struct TagCamera {
  std::string id;  ///< Its id.
  int views = 0;   ///< Its views of the tag.
  int used = 0;    ///< Of them, those used.
  /// Its colour camera's centre above the tag's plane (metres): z in the
  /// tag's world.
  double height_m = 0.0;
};

/// @brief The tag, located: the world it fixes, and how well it fits.
///
/// @code
/// VKC_ASSIGN(const WorldOrigin origin,
///            origin_tag_frame(cameras, solution, views, kOriginTag));
/// if (origin.tag && origin.tag->edge_warning) {
///   warn(origin.tag->edge_m, origin.tag->edge_error_percent);
/// }
/// @endcode
struct TagWorld {
  /// The reference camera's colour frame, the solution's world, to the
  /// tag's.
  recon::camera::Mat4d reference_to_world{1.0};
  /// The corners placed, in the tag's world, in @ref DetectedMarker's order.
  std::array<recon::camera::Vec3d, 4> corners_m{};
  /// The edges (metres): corner k to corner k + 1, the last back to 0.
  std::array<double, 4> edges_m{};
  /// The diagonals (metres): corner 0 to 2, and 1 to 3.
  std::array<double, 2> diagonals_m{};
  double edge_m = 0.0;  ///< The mean edge (metres), the frame's square's.
  /// The mean edge's departure from the tag's printed edge (percent).
  double edge_error_percent = 0.0;
  /// Whether that departure is above `edge_warn_fraction`.
  bool edge_warning = false;
  /// The corner farthest from the tag's plane: its distance (metres).
  double out_of_plane_m = 0.0;
  /// The RMS of the used views' corners' residuals (pixels).
  double rms_px = 0.0;
  double worst_px = 0.0;  ///< Their worst (pixels).
  /// Every view, used or dropped, by capture, then camera id.
  std::vector<TagViewFit> views;
  std::vector<TagCamera> cameras;  ///< Every camera, in the rig's order.
  int solves = 0;  ///< The solves run: 1 and an outlier round's each.
};

/// @brief Where the world is: at the tag, or, without one, at the reference
///        camera, the solution's own world.
///
/// @code
/// VKC_ASSIGN(const WorldOrigin origin,
///            origin_tag_frame(cameras, solution, views, kOriginTag));
/// const recon::camera::Mat4d w = origin.reference_to_world();
/// @endcode
struct WorldOrigin {
  /// The tag sought, as @ref origin_tag_frame took it; its dictionary `None`
  /// when none was sought.
  OriginTag sought;
  /// The tag's world; empty when the reference camera stays the origin.
  std::optional<TagWorld> tag;
  /// Why there is no tag world; set only when @ref tag is empty.
  TagMiss miss = TagMiss::NotSeen;
  /// The camera a `OneCamera` or `CameraBelowTag` miss names; else empty.
  std::string camera;
  /// The miss in words, for a report; empty with a tag world, and when no
  /// tag was sought.
  std::string reason;

  /// @return The reference camera's frame to the world: @ref tag's
  ///         `reference_to_world`, or exactly the identity without one.
  recon::camera::Mat4d reference_to_world() const noexcept {
    return tag ? tag->reference_to_world : recon::camera::Mat4d(1.0);
  }
};

/// @brief Locate @p tag from @p solution, the rig's poses fixed, and the
///        world it fixes; or say why it fixes none.
///
/// The views are taken by capture, then camera id, so the result does not
/// depend on their order. Fewer than 2 cameras seeing the tag, or fewer
/// than `min_cameras`, is a miss before any solve. Then, in the solution's
/// gauge:
/// 1. **Rays.** Each corner's pixel is undistorted
///    (`compute::undistort_points`, converged).
/// 2. **Corners.** Each corner is triangulated from every view kept by a
///    linear N-view DLT (`cv::SVD`), then the 12 corner coordinates are
///    refined together on `cv::LevMarq`, minimising Huber's cost of each
///    corner's reprojection through its camera's whole lens.
/// 3. **Outlier rounds.** After each solve, up to `outlier_rounds` times, the
///    views whose worst corner is above `bundle_adjust`'s corner bound
///    (`outlier_px`, or `outlier_mads` robust deviations if more) are
///    dropped, and the corners are solved again from the views left. The
///    rounds stop when none is dropped; fewer than `min_cameras` cameras
///    left is a miss.
/// 4. **Frame.** A square of the corners' mean edge, the model above, is
///    fitted to them by SVD (Procrustes), a proper rotation and a
///    translation: no tag size is assumed. The tag's printed edge is only
///    compared with the mean edge.
/// 5. **Guard.** Each camera that saw the tag must be above its plane
///    (z > 0 in its world).
/// 6. **Fit.** The used views' corners must fit at @ref kMaxTagWorldRmsPx
///    or better.
///
/// @param cameras   The rig's cameras, as @ref bundle_adjust took them.
/// @param solution  @ref bundle_adjust's solution for them.
/// @param views     Every sighting of the tag, at most one per camera and
///                  capture; none when it was not seen.
/// @param tag       The tag sought, as `check_origin_tag` accepts it: its id
///                  must be each view's, and its edge is the one compared.
/// @param options   The thresholds.
/// @return The world's origin, @p tag as its @ref WorldOrigin::sought: a tag
///         world, or the reference with the miss;
///         or `InvalidArgument` for an option out of range (naming it), a
///         @p tag `check_origin_tag` refuses, cameras @ref initialise would
///         refuse, a @p solution that does not fit them, or a view naming
///         another camera or marker, of an image not its lens's size, with a
///         corner not finite, or a second of its camera and capture; or what
///         OpenCV threw (@ref compute::guarded). Each message starts
///         `origin_tag_frame: `.
///
/// @code
/// VKC_ASSIGN(const WorldOrigin origin,
///            origin_tag_frame(cameras, solution, views, kOriginTag));
/// VKC_ASSIGN(const AnchoredRig rig, anchor_rig(solution, origin));
/// @endcode
VC_SOLVE_API core::Result<WorldOrigin> origin_tag_frame(
    const std::vector<RigCamera>& cameras, const RigSolution& solution,
    const std::vector<TagView>& views, const OriginTag& tag,
    const TagFrameOptions& options = {});

/// @brief A solved rig's outputs in its world's frame: what is written and
///        shown.
///
/// @code
/// VKC_ASSIGN(const AnchoredRig rig, anchor_rig(solution, origin));
/// VKC_TRY(recon::camera::write_array_calibration(
///     path, rig.calibration, core::WriteMode::CreateNew));
/// @endcode
struct AnchoredRig {
  /// The solution's calibration in the world: each sensor's
  /// `color_to_world` `W C`, W the origin's `reference_to_world`.
  recon::camera::ArrayCalibration calibration;
  /// Each camera's colour frame to the world, `W C`, in the rig's order: the
  /// inverse of its T_cw, so its T_cw in the world is `T_cw W^-1`.
  std::vector<recon::camera::Mat4d> color_to_world;
  /// Each solved board's T_wb in the world, `W T_wb`, in
  /// @ref RigSolution::boards' order.
  std::vector<recon::camera::Mat4d> boards_to_world;
};

/// @brief @p solution's outputs moved into @p origin's world, applied last:
///        @p solution itself stays in the reference camera's gauge.
///
/// Every camera sees every point where it did: `T_cw W^-1` of `W X` is
/// `T_cw X`. Without a tag world W is the identity and the outputs are the
/// solution's own, its calibration's world the reference's id. With one the
/// calibration's world is the tag, recon's `world.apriltag`: the
/// @ref apriltag_family of the tag sought's dictionary, its id, and its
/// printed edge as `size_m`.
/// @param solution  @ref bundle_adjust's solution.
/// @param origin    Its world's origin, as @ref origin_tag_frame gave it.
/// @return The outputs; `InvalidArgument` for a solution with no camera, a
///         pose or board not finite, or a sensor not posed, a
///         `reference_to_world` that is not rigid, or a tag world whose tag
///         sought `check_origin_tag` refuses or is of no AprilTag family; or
///         `OutOfMemory`. Each message starts `anchor_rig: `.
///
/// @code
/// VKC_ASSIGN(const AnchoredRig rig, anchor_rig(solution, origin));
/// place(cameras[c].id, rig.color_to_world[c]);
/// @endcode
VC_SOLVE_API core::Result<AnchoredRig> anchor_rig(const RigSolution& solution,
                                                  const WorldOrigin& origin);

}  // namespace volumetric_kit::calib::solve
