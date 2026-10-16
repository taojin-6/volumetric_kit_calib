// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file initialise.hpp
/// @brief The first rig: every camera and every board placed from the
///        views alone, the bundle adjustment's starting point; and its rules
///        for the rig and the views, which the session's checks share.
///
/// The solver's state, which the adjustment refines:
/// - `world_to_camera` (T_cw) for each colour camera, OpenCV's extrinsic;
/// - `board_to_world` (T_wb) for each capture;
/// - the world is the reference camera's colour frame (+x right, +y down,
///   +z forward), so the reference's T_cw is the identity. Scale comes from
///   the board's squares.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/usable_for_pose.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/solve/export.hpp"
#include "volumetric_kit/calib/solve/spanning_tree.hpp"
#include "volumetric_kit/calib/solve/view_pose.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::solve {

/// @brief One camera the solve places: a colour camera, named as its
///        observations name it.
///
/// @code
/// RigCamera cam;
/// cam.id = info.id;        // a recon SensorInfo: the serial
/// cam.model = *info.color;  // the factory model at the streamed mode
/// @endcode
struct RigCamera {
  std::string id;  ///< As `Observation::camera_id`: unique, not empty.
  /// The lens, at the size of the images its views were detected in.
  recon::camera::CameraModel model;
};

/// @brief Refuse a rig's camera ids that are none, empty or repeated.
/// @param ids  Each camera's id, in the rig's order.
/// @return OK; or `InvalidArgument` for the first fault, e.g. `the rig has no
///         cameras`, `camera 2 has no id` or `camera id "A" appears twice`
///         (a caller adds its name with `with_context`); or `OutOfMemory`.
///
/// @code
/// VKC_TRY(check_camera_ids(serials).with_context("CaptureRule"));
/// @endcode
VC_SOLVE_API core::Status check_camera_ids(const std::vector<std::string>& ids);

/// @brief A rig's cameras as the solve takes them: each one's id and lens,
///        in the rig's order.
///
/// @code
/// VKC_ASSIGN(const RigLenses rig, check_rig(cameras, primary));
/// const compute::CvCamera& lens = rig.lenses[c];  // camera c, rig.ids[c]
/// @endcode
struct RigLenses {
  std::vector<std::string> ids;           ///< Each camera's id.
  std::vector<compute::CvCamera> lenses;  ///< Each camera's lens.
};

/// @brief Check a rig and its reference, and put each lens in OpenCV's
///        types.
/// @param cameras    The rig's cameras.
/// @param reference  The camera the world is, by index in @p cameras.
/// @return The ids and lenses; `InvalidArgument` for ids
///         @ref check_camera_ids refuses, a reference outside the rig, or a
///         camera model @ref compute::cv_camera refuses (`camera ID: ` and its
///         message), a caller adding its name with `with_context`; or
///         `OutOfMemory`.
///
/// @code
/// core::Result<RigLenses> rig = check_rig(cameras, primary);
/// if (!rig.ok()) return rig.status().with_context("SessionChecks");
/// @endcode
VC_SOLVE_API core::Result<RigLenses> check_rig(
    const std::vector<RigCamera>& cameras, std::size_t reference);

/// @brief The thresholds @ref initialise applies.
///
/// @code
/// InitialiseOptions options;
/// options.weak_below = 10;  // warn below 10 shared captures
/// @endcode
struct InitialiseOptions {
  /// A view whose own pose fits worse than this (RMS, pixels) is left out.
  double max_pnp_rms_px = 5.0;
  /// A view whose poses' @ref ViewPose::ambiguity is below this places no
  /// camera; at least 1.
  double min_ambiguity = 2.0;
  /// A view tilted more than this (degrees, @ref tilt_deg) is flagged.
  double oblique_deg = 70.0;
  /// A tree link sharing fewer captures than this is weak.
  int weak_below = 5;
  /// A capture's relative pose more than this (degrees, below 90) from the
  /// link's medoid rotation does not place the camera.
  double inlier_deg = 3.0;
  /// A corner's reprojection error counts at most this (pixels) when a
  /// board's candidate poses are scored.
  double truncation_px = 10.0;
};

/// @brief What @ref initialise does with a view.
enum class ViewFate {
  Used,  ///< It initialises the rig.
  /// Its two poses fit almost alike, so it places no camera. Its capture's
  /// board comes from it only when the capture has no view used; the
  /// adjustment (@ref bundle_adjust) takes it.
  Ambiguous,
  /// Left out: @ref compute::usable_for_pose refused it
  /// (@ref ViewRecord::use says why).
  NotUsable,
  /// Left out: IPPE gave no pose with every corner in front of the camera.
  NoPose,
  /// Left out: its own pose fits worse than `max_pnp_rms_px`.
  PoorFit,
};

/// @brief What @ref initialise found in one view, so a view left out can be
///        named and recaptured.
///
/// The measures from the view's own pose are empty when it has none.
///
/// @code
/// for (const ViewRecord& view : rig.views) {
///   if (view.fate == ViewFate::PoorFit) report(view.camera, view.capture);
/// }
/// @endcode
struct ViewRecord {
  std::size_t camera = 0;    ///< Its camera's index in the rig.
  std::int64_t capture = 0;  ///< Its capture, as `Observation::frame_id`.
  ViewFate fate = ViewFate::NotUsable;  ///< What the solve does with it.
  /// @ref compute::usable_for_pose's answer.
  compute::ViewUse use = compute::ViewUse::TooFewCorners;
  std::optional<double> pnp_rms_px;  ///< Its best pose's RMS (pixels).
  /// Its poses' @ref ViewPose::ambiguity.
  std::optional<double> ambiguity;
  std::optional<double> tilt_deg;  ///< Its best pose's tilt (degrees).
  /// Tilted more than `oblique_deg`: kept, but its fit is worth reporting
  /// apart.
  bool oblique = false;
};

/// @brief Where a capture put the board.
///
/// @code
/// for (const BoardPose& board : rig.boards) {
///   const recon::camera::Mat4d board_to_camera =
///       rig.world_to_camera[c] * board.board_to_world;  // T_cb in camera c
/// }
/// @endcode
struct BoardPose {
  std::int64_t capture = 0;  ///< The capture, as `Observation::frame_id`.
  recon::camera::Mat4d board_to_world{1.0};  ///< T_wb.
};

/// @brief A link of the tree (@ref TreeLink) as @ref initialise placed its
///        camera.
///
/// @code
/// for (const PlacedLink& link : rig.links) {
///   const int outliers = link.shared - link.inliers;
///   if (outliers > 0) report(link.camera, outliers);  // captures to check
/// }
/// @endcode
struct PlacedLink : TreeLink {
  /// The shared captures whose relative pose agreed and placed the camera.
  int inliers = 0;
};

/// @brief The first rig: every camera and board placed from the views.
///
/// @code
/// VKC_ASSIGN(const InitialRig rig,
///            initialise(cameras, board, observations, primary));
/// const recon::camera::Mat4d& world_to_camera = rig.world_to_camera[1];
/// @endcode
struct InitialRig {
  std::size_t reference = 0;  ///< The camera the world is.
  /// T_cw for each camera, in the rig's order; the reference's is exactly
  /// the identity.
  std::vector<recon::camera::Mat4d> world_to_camera;
  /// The board in each capture with a view used or ambiguous, in the
  /// captures' order; a capture with neither has no pose.
  std::vector<BoardPose> boards;
  /// How the cameras were placed: @ref SpanningTree::links, in its order.
  std::vector<PlacedLink> links;
  /// One record for each observation, in the observations' order.
  std::vector<ViewRecord> views;
};

/// @brief Place every camera and board from the views: each view's own pose
///        (@ref view_pose), the tree of the links that share the most captures
///        (@ref spanning_tree), each link's robust relative pose, and each
///        board's best candidate pose.
///
/// 1. **Views.** A view is used when it is usable
///    (@ref compute::usable_for_pose), has a pose, fits it within
///    `max_pnp_rms_px`, and is not ambiguous (`min_ambiguity`), as
///    @ref view_fate says; its tilt is flagged above `oblique_deg`.
/// 2. **Tree.** @ref spanning_tree of the used views' @ref covisibility.
/// 3. **Cameras.** Each link's camera is placed from its parent, from the
///    reference outwards. Each capture both use gives a relative pose
///    `T_cb(camera) T_cb(parent)^-1`. Of those, the rotation with the least
///    median angle to the others is the medoid; the captures within
///    `inlier_deg` of it give the rotation, as their chordal mean (recon's
///    `nearest_rotation` of their sum), and the translation, as their
///    median on each axis.
/// 4. **Boards.** For each capture, each used view's poses, both of IPPE's,
///    through its camera's placement, are candidates; the one whose summed
///    squared reprojection error over the capture's used views, each corner
///    capped at `truncation_px`, is least is the board's. A capture whose
///    views are all ambiguous takes its board from them alike, so the
///    adjustment can take them.
///
/// The result depends on the cameras' ids, not on the order of the cameras
/// or of the observations, and is the same on every run.
/// @param cameras    The rig's cameras.
/// @param board      The board seen.
/// @param views      Every camera's view of the board in every capture: at
///                   most one per camera and capture.
/// @param reference  The camera the world is, by index in @p cameras.
/// @param options    The thresholds.
/// @return The first rig; `InvalidArgument` for a camera outside the tree
///         (naming it, and saying why when none of its views is used), an
///         option out of range (naming it), no cameras, a repeated or empty
///         id, a camera model @ref compute::cv_camera refuses, a view naming
///         a camera the rig does not have, two views of one capture by one
///         camera, or a reference outside the rig; @ref view_pose's `Backend`
///         error when IPPE fails on a usable view (naming the view); or what
///         OpenCV threw (@ref compute::guarded). Each message starts
///         `initialise: `.
VC_SOLVE_API core::Result<InitialRig> initialise(
    const std::vector<RigCamera>& cameras, const compute::Board& board,
    const std::vector<Observation>& views, std::size_t reference,
    const InitialiseOptions& options = {});

/// @brief Refuse options @ref initialise cannot take.
/// @param options  The options.
/// @param prefix   What each message starts with, before the option's name.
/// @return OK; `InvalidArgument` naming the first option out of range and
///         its range; or `OutOfMemory`.
///
/// @code
/// VKC_TRY(check_initialise_options(options.solve, "app: options.solve."));
/// @endcode
VC_SOLVE_API core::Status check_initialise_options(
    const InitialiseOptions& options,
    std::string_view prefix = "initialise: options.");

/// @brief What @ref initialise does with a view it has poses for: step 1's
///        rule, which the session's checks share.
/// @param pose     The view's poses (@ref view_pose).
/// @param options  The thresholds; @ref check_initialise_options accepts
///                 them.
/// @return `PoorFit` when its best pose fits worse than `max_pnp_rms_px`;
///         else `Ambiguous` when its @ref ViewPose::ambiguity is below
///         `min_ambiguity`; else `Used`.
///
/// @code
/// VKC_ASSIGN(const ViewPose pose, view_pose(cam, board, view));
/// if (view_fate(pose, options) == ViewFate::Ambiguous) hint("tilt the board");
/// @endcode
VC_SOLVE_API ViewFate view_fate(const ViewPose& pose,
                                const InitialiseOptions& options) noexcept;

}  // namespace volumetric_kit::calib::solve
