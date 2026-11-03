// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tracked_world.hpp
/// @brief A tracked camera's world placed in the origin tag's, fitted to the
///        camera's stills of the tag at the poses its session records for
///        them; and the registration file's contents from that.
///
/// A tracked camera (ARKit) poses each frame in a gravity-aligned world of
/// its own. Its stills of the origin tag fix the one rigid transform that
/// places that world in the tag's (world_tag.hpp's frame, z up out of the
/// printed face): `tracking_to_world = [Rz(yaw) A | t]`, where A turns the
/// tracking world's up onto +z. The tracking gives the tilt, so the fit
/// solves the yaw and the translation alone. A still's own tilt is weak
/// (degrees, face-on), and a hold's nearly parallel rays fix no size. With
/// the tilt given, yaw and position hold from one still, face-on included.
/// The corners are fitted to a square of the edge the rig measured, never
/// the printed one.

#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/solve/export.hpp"
#include "volumetric_kit/calib/solve/world_tag.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"
#include "volumetric_kit/sensor/camera/tracked_stills.hpp"

namespace volumetric_kit::calib::solve {

/// @brief One still of the origin tag from a tracked camera, at the pose its
///        session records for it.
///
/// @code
/// VKC_ASSIGN(const detect::MarkerSearch search,
///            detect::detect_tag(luma, kOriginTag));
/// if (search.found) {
///   views.push_back({still.image, size, still.model, still.cam_to_tracking,
///                    *search.found});
/// }
/// @endcode
struct TrackedTagView {
  std::string still;  ///< Its name, unique among the views: its file's.
  /// The image the tag was found in: its model's size.
  sensor::camera::ImageSize image_size;
  /// The camera's model for the still: a pinhole for ARKit, which states no
  /// lens.
  sensor::camera::CameraModel model;
  /// The camera's frame, in sensor's camera axes (+z forward, +y down), to
  /// the tracking world: the still's recorded pose (sensor's
  /// `TrackedStill::cam_to_tracking`). Rigid.
  sensor::camera::Mat4d cam_to_tracking{1.0};
  /// The tag's id and corners in the still's pixels, as `detect::detect_tag`
  /// finds them.
  DetectedMarker marker;
};

/// @brief What @ref tracked_world_from_tag takes the tracking world to be.
///
/// @code
/// TrackedWorldOptions options;   // ARKit's world: +y up
/// options.up = {0.0, 0.0, 1.0};  // a tracking world with z up
/// @endcode
struct TrackedWorldOptions {
  /// The tracking world's up, against gravity, in its own axes: ARKit's
  /// gravity-aligned world has +y up. Finite and not zero; its length is
  /// ignored.
  sensor::camera::Vec3d up{0.0, 1.0, 0.0};
};

/// @brief The RMS of a tracked fit's corner residuals
///        (@ref TrackedTagFit::rms_px) above which @ref tracked_world_from_tag
///        refuses it (pixels): a @ref TagMiss::PoorFit.
///
/// Above the rig's @ref kMaxTagWorldRmsPx, whose cameras are solved together
/// to a fraction of a pixel: each still carries its tracker's own pose
/// error. On synthetic holds, a tracker off by 0.1 degrees and 2 mm per
/// still fits at up to 9.2 px RMS from 1.2 m (5.7 px at p95), and 11.7 px
/// from 0.8 m; one still moved 10 cm fits at 18.9 px or more from as far as
/// 3 m, and 48 px from 1.2 m.
///
/// @code
/// if (found.miss == TagMiss::PoorFit) warn(found.reason);  // names the RMS
/// @endcode
inline constexpr double kMaxTrackedRmsPx = 12.0;

/// @brief One still's fit at the transform found.
///
/// @code
/// for (const TrackedStillFit& s : found.fit->stills) {
///   log(s.still, s.rms_px, s.range_m, s.off_normal_deg);
/// }
/// @endcode
struct TrackedStillFit {
  std::string still;      ///< Its name.
  double rms_px = 0.0;    ///< The RMS of its corners' residuals (pixels).
  double worst_px = 0.0;  ///< Its worst corner's residual (pixels).
  double range_m = 0.0;   ///< Its camera's distance from the tag's centre.
  /// The angle between its camera's ray to the tag's centre and the tag's
  /// normal (degrees): 0 face-on.
  double off_normal_deg = 0.0;
};

/// @brief The tracking world, placed in the tag's, and how well it fits.
///
/// @code
/// const sensor::camera::Mat4d cam_to_world =
///     found.fit->tracking_to_world * view.cam_to_tracking;
/// @endcode
struct TrackedTagFit {
  /// The tracking world to the tag's: `x_world = tracking_to_world
  /// x_tracking`. Rigid; it takes the tracking world's up to +z exactly.
  sensor::camera::Mat4d tracking_to_world{1.0};
  double rms_px = 0.0;    ///< The RMS of every still's corner residuals.
  double worst_px = 0.0;  ///< Their worst (pixels).
  double edge_m = 0.0;    ///< The square's edge the corners were fitted to.
  std::vector<TrackedStillFit> stills;  ///< Every still, by name.
};

/// @brief Where the tracking world sits in the tag's, or why it was not
///        placed.
///
/// @code
/// VKC_ASSIGN(const TrackedWorld found,
///            tracked_world_from_tag(views, kOriginTag, 0.14725));
/// if (!found.fit) warn(to_string(found.miss), found.reason);
/// @endcode
struct TrackedWorld {
  /// The tag sought, as @ref tracked_world_from_tag took it.
  OriginTag sought;
  std::optional<TrackedTagFit> fit;  ///< Empty on a miss.
  /// Why there is no fit; set only when @ref fit is empty: `NotSeen` (no
  /// still), `CameraBelowTag`, `Numerical` or `PoorFit`.
  TagMiss miss = TagMiss::NotSeen;
  std::string still;   ///< The still a `CameraBelowTag` miss names.
  std::string reason;  ///< The miss in words; empty with a fit.
};

/// @brief Place the tracking world in @p tag's from @p views, their poses
///        fixed; or say why it cannot be placed.
///
/// The views are taken by still name, so the result does not depend on
/// their order. In the tag's frame, its corners a square of @p edge_m in
/// `DetectedMarker`'s order (world_tag.hpp):
/// 1. **Seed.** Each still alone, in closed form, on its undistorted
///    corners: with the tilt given the tag's plane is level, its corners'
///    rays meet it at a scale of their level parts, and the square's corners
///    are a turn, that scale and a shift of them (a least-squares
///    similarity), which give a yaw and a translation. The seed is their
///    median over the stills. A still whose corners make no convex
///    quadrilateral, or whose rays meet the plane on both sides of its
///    camera, is a `Numerical` miss.
/// 2. **Fit.** The yaw and the translation by least squares on `cv::LevMarq`:
///    the sum of every still's squared corner residuals, through its whole
///    lens, from the seed. No still is dropped: a hold's stills share one
///    pose, and a bad one shows in its @ref TrackedStillFit.
/// 3. **Guard.** Each still's camera must be above the tag's plane.
/// 4. **Fit.** The corners must fit at @ref kMaxTrackedRmsPx or better.
///
/// @param views    Every still the tag was found in; none when it was not.
/// @param tag      The tag sought, as `check_origin_tag` accepts it: its id
///                 must be each view's. Its printed edge is not used.
/// @param edge_m   The tag's edge as the rig located it (`TagWorld::edge_m`):
///                 finite, above 0.
/// @param options  The tracking world's up.
/// @return The placement, @p tag as its @ref TrackedWorld::sought: a fit, or
///         a miss; or `InvalidArgument` for an `up` not finite or zero, an
///         @p edge_m not finite and above 0, a @p tag `check_origin_tag`
///         refuses, or a view with a still named before, a model
///         `check_camera_model` refuses, an image not its model's size, a
///         pose not rigid, another marker or a corner not finite; or what
///         OpenCV threw (@ref compute::guarded). Each message starts
///         `tracked_world_from_tag: `.
VC_SOLVE_API core::Result<TrackedWorld> tracked_world_from_tag(
    const std::vector<TrackedTagView>& views, const OriginTag& tag,
    double edge_m, const TrackedWorldOptions& options = {});

/// @brief The registration of @p stills' session from @p found: what the
///        caller writes with sensor's `write_tracked_registration`.
///
/// `tracking_to_world` is the fit's, `rms_px` its RMS, `views` its stills
/// and `edge_m` the edge it was fitted to. `sensor`, `session` and
/// `world_epoch` are @p stills'.
/// @param stills  The stills the views were found in.
/// @param world   The array's world (`ArrayCalibration::world`): the tag
///                sought, with its family (@ref apriltag_family), its id, and
///                its printed edge as `size_m`.
/// @param found   @ref tracked_world_from_tag's placement from those stills.
/// @return The registration, which `validate_tracked_registration` accepts;
///         or `InvalidArgument` for a miss (giving its reason), a world that
///         is not the tag sought (naming what differs), or @p stills that
///         `validate_tracked_stills` refuses; or `OutOfMemory`. Each message
///         starts `register_tracked: `.
///
/// @code
/// VKC_ASSIGN(const sensor::camera::TrackedRegistration reg,
///            register_tracked(stills, array.world, found));
/// VKC_TRY(sensor::camera::write_tracked_registration(
///     path, reg, core::WriteMode::CreateNew));
/// @endcode
VC_SOLVE_API core::Result<sensor::camera::TrackedRegistration> register_tracked(
    const sensor::camera::TrackedStills& stills,
    const sensor::camera::WorldFrame& world, const TrackedWorld& found);

}  // namespace volumetric_kit::calib::solve
