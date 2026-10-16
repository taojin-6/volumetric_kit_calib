// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file bundle_adjust.hpp
/// @brief The rig's extrinsics refined from the first rig: a bundle
///        adjustment on OpenCV's `cv::LevMarq`, its outlier rounds, guards,
///        covariance and intrinsics sensitivity.
///
/// The parameters are each non-reference camera's world-to-camera transform
/// (T_cw) and each capture's board-to-world transform (T_wb), as a Rodrigues
/// vector and a translation; the reference camera is fixed at the identity
/// (the gauge), and the board's squares give the scale. A corner's residual
/// is its projection, `cv::projectPoints` of `R_cw (R_wb X + t_wb) + t_cw`
/// through the camera's whole lens, minus where it was detected. The lenses
/// are not adjusted.

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/solve/export.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::solve {

/// @brief A corner's two residuals' derivatives by a transform's six
///        parameters: the Rodrigues vector's three, then the translation's.
using CornerJacobian = cv::Matx<double, 2, 6>;

/// @brief A view's corners at a rig's state: each one's residual and depth,
///        and the residuals' derivatives by the camera's and the board's
///        transforms.
///
/// @code
/// VKC_ASSIGN(const ViewResiduals at,
///            view_residuals(cam, board, view, world_to_camera,
///            board_to_world));
/// const recon::camera::Vec2d& e = at.residuals[0];  // pixels
/// const CornerJacobian& by_camera = at.camera_jacobian[0];
/// @endcode
struct ViewResiduals {
  /// Each corner's projection minus its detection (pixels), in the view's
  /// order.
  std::vector<recon::camera::Vec2d> residuals;
  /// Each corner's depth in the camera (metres): z of `T_cw T_wb X`.
  std::vector<double> depths;
  /// Each corner's residual by `world_to_camera`'s parameters: OpenCV's
  /// `dp/drvec` and `dp/dtvec`.
  std::vector<CornerJacobian> camera_jacobian;
  /// Each corner's residual by `board_to_world`'s parameters: `dp/dX_w`,
  /// which is `dp/dtvec R_cw`, through the 3 x 9 derivative of
  /// `cv::Rodrigues(r_wb)` and the identity.
  std::vector<CornerJacobian> board_jacobian;
};

/// @brief The residuals the adjustment minimises for one view, and their
///        derivatives.
///
/// Accurate at every rotation: `cv::Rodrigues` is taken only from vector to
/// matrix, whose derivative has no singularity below 2 pi, and never from
/// matrix to vector. A corner behind the camera projects as OpenCV projects
/// it; its depth says so. A pixel not a number gives a residual not a
/// number.
/// @param camera           The view's camera.
/// @param board            The board seen.
/// @param view             The corners seen; their weights are not used.
/// @param world_to_camera  T_cw.
/// @param board_to_world   T_wb.
/// @return The residuals and derivatives; `InvalidArgument` for a corner id
///         the board does not have or a transform not finite; or what OpenCV
///         threw (@ref compute::guarded).
VC_SOLVE_API core::Result<ViewResiduals> view_residuals(
    const compute::CvCamera& camera, const compute::Board& board,
    const Observation& view,
    const recon::camera::RodriguesTransform& world_to_camera,
    const recon::camera::RodriguesTransform& board_to_world);

/// @brief The thresholds and amounts @ref bundle_adjust applies.
///
/// @code
/// BundleAdjustOptions options;
/// options.outlier_rounds = 0;  // keep every corner
/// @endcode
struct BundleAdjustOptions {
  /// Huber's delta on each corner's Euclidean residual (pixels): a corner
  /// fitting worse counts linearly, not squared.
  double huber_px = 1.0;
  /// A corner nearer its camera than this (metres), or behind it, as a solve
  /// begins is dropped; a step that would bring a kept one nearer is refused
  /// (@ref bundle_adjust, step 2).
  double min_depth_m = 0.05;
  /// `cv::LevMarq`'s iterations in one solve, at most; at least 1.
  int max_iterations = 100;
  /// Solves after outliers are dropped, at most; 0 drops none.
  int outlier_rounds = 3;
  /// A corner whose residual is above this (pixels), and above
  /// `outlier_mads` robust deviations, is an outlier.
  double outlier_px = 3.0;
  /// The robust deviations, `1.4826` times the median absolute deviation of
  /// the kept corners' residual components, above which a corner is an
  /// outlier, if above `outlier_px` too.
  double outlier_mads = 4.0;
  /// A view whose RMS is above this many times the median view's, and above
  /// `outlier_view_px`, is an outlier: its RMS over its corners that are not
  /// outliers, or over all of them when every one is.
  double outlier_view_ratio = 3.0;
  /// The RMS (pixels) a view must be above to be an outlier.
  double outlier_view_px = 1.0;
  /// Each link of the tree must keep at least this many shared captures
  /// after the outliers are dropped, with enough positive-weight corners
  /// in each camera's view to fix a pose; at least 1.
  int min_shared = 3;
  /// Each camera's 6 x 6 block of the normal matrix, with the boards
  /// eliminated, must have a condition number below this; above 1.
  double max_condition = 1e12;
  /// Captures adjusted, at most; at least 1.
  std::size_t max_captures = 150;
  /// The focal lengths' error, as a fraction of them, whose effect
  /// @ref IntrinsicsSensitivity predicts.
  double focal_fraction = 0.003;
  /// The principal point's error (pixels) whose effect
  /// @ref IntrinsicsSensitivity predicts.
  double principal_px = 2.0;
};

/// @brief Why the adjustment left a corner or a view out.
enum class Dropped {
  /// The corner was nearer its camera than `min_depth_m`, or behind it, as a
  /// solve began.
  TooNear,
  /// The corner's residual was above the round's outlier bound.
  Outlier,
  /// The view's RMS was above the round's view bound.
  OutlierView,
  /// The view's corners still kept no longer fix a pose
  /// (@ref compute::usable_for_pose), so none of them is kept.
  Unusable,
};

/// @brief A corner or a view the adjustment left out, and why, so it can be
///        named and recaptured.
///
/// @code
/// for (const Rejection& r : solution.rejections) {
///   if (r.corner < 0) report(cameras[r.camera].id, r.capture, r.value);
/// }
/// @endcode
struct Rejection {
  std::size_t camera = 0;    ///< Its camera's index in the rig.
  std::int64_t capture = 0;  ///< Its capture, as `Observation::frame_id`.
  int corner = -1;           ///< The corner's id; -1 for a whole view.
  /// The solves run before it was left out: 0 before the first.
  int round = 0;
  Dropped reason = Dropped::Outlier;  ///< Why.
  /// The corner's depth (metres) for `TooNear`, its residual (pixels) for
  /// `Outlier`, the view's RMS (pixels) for `OutlierView`; 0 for `Unusable`.
  double value = 0.0;
  /// The bound it broke, in @ref value's units; 0 for `Unusable`.
  double bound = 0.0;
};

/// @brief One corner of an adjusted view at the solution.
///
/// @code
/// for (const CornerResidual& c : solution.residuals) {
///   if (!c.dropped) sum += glm::dot(c.residual, c.residual);
/// }
/// @endcode
struct CornerResidual {
  std::size_t camera = 0;    ///< Its camera's index in the rig.
  std::int64_t capture = 0;  ///< Its capture, as `Observation::frame_id`.
  int corner = 0;            ///< Its id on the board.
  recon::camera::Vec2d pixel{0.0};  ///< Where it was detected.
  /// Its projection minus its detection (pixels); not a number for a corner
  /// behind its camera.
  recon::camera::Vec2d residual{0.0};
  double weight = 1.0;  ///< Its detection's weight, as given.
  /// Huber's weight on it at the solution: 1 within `huber_px`, and
  /// `huber_px / |residual|` beyond.
  double huber_weight = 1.0;
  bool dropped = false;  ///< Left out (@ref RigSolution::rejections).
};

/// @brief A capture's board at the solution.
///
/// @code
/// const recon::camera::Mat4d board_to_world =
///     recon::camera::matrix_from_rodrigues(solved.board_to_world);
/// @endcode
struct SolvedBoard {
  std::int64_t capture = 0;  ///< The capture, as `Observation::frame_id`.
  /// T_wb, its rotation's angle at most pi.
  recon::camera::RodriguesTransform board_to_world;
};

/// @brief How uncertain a camera's solved pose is from the corners' noise
///        alone.
///
/// @code
/// const PoseCovariance& sigma = solution.covariance[c];
/// log(cameras[c].id, sigma.rotation_deg, sigma.centre_mm);  // 1 sigma
/// @endcode
struct PoseCovariance {
  /// The covariance of T_cw's six parameters (rvec, then tvec): the
  /// residuals' variance times the camera's block of the normal matrix's
  /// inverse, by its Schur complement over the boards. Zero for the
  /// reference.
  cv::Matx66d covariance = cv::Matx66d::zeros();
  /// One sigma of the rotation (degrees): the root of the trace of its
  /// covariance as a small rotation.
  double rotation_deg = 0.0;
  /// One sigma of the camera's centre, `-R^T t` (millimetres): the root of
  /// the trace of its covariance.
  double centre_mm = 0.0;
};

/// @brief How far a camera's solved pose moves, to first order.
///
/// @code
/// const PoseShift& moved = solution.sensitivity[j].focal[c];
/// log(cameras[c].id, moved.rotation_deg, glm::length(moved.centre_mm));
/// @endcode
struct PoseShift {
  double rotation_deg = 0.0;            ///< The rotation's turn (degrees).
  recon::camera::Vec3d centre_mm{0.0};  ///< The centre's move (millimetres).
};

/// @brief What one camera's intrinsics being off does to every camera's
///        solved pose, to first order: the bias budget beside the noise
///        (@ref PoseCovariance).
///
/// Each shift is `-(J^T W J)^-1 J^T W J_int d_int`, with `J_int` from
/// `cv::projectPoints`' `dpdf` and `dpdc`. To first order, an error the other
/// way moves each camera the other way as far.
///
/// @code
/// const IntrinsicsSensitivity& off = solution.sensitivity[c];
/// const PoseShift& itself = off.focal[c];  // camera c's f 0.3 % long
/// @endcode
struct IntrinsicsSensitivity {
  /// Each camera's shift, in the rig's order, when this camera's `fx` and
  /// `fy` are `focal_fraction` of themselves larger.
  std::vector<PoseShift> focal;
  /// Each camera's shift when this camera's `cx` is `principal_px` larger.
  std::vector<PoseShift> principal_x;
  /// Each camera's shift when this camera's `cy` is `principal_px` larger.
  std::vector<PoseShift> principal_y;
};

/// @brief The adjusted rig.
///
/// @code
/// VKC_ASSIGN(const RigSolution solution,
///            bundle_adjust(cameras, board, views, rig));
/// VKC_TRY(recon::camera::write_array_calibration(
///     path, solution.calibration, core::WriteMode::CreateNew));
/// @endcode
struct RigSolution {
  std::size_t reference = 0;  ///< The camera the world is.
  /// T_cw for each camera, in the rig's order, each rotation's angle at
  /// most pi; the reference's exactly zero.
  std::vector<recon::camera::RodriguesTransform> world_to_camera;
  /// The same poses as recon's calibration: the world the reference's id,
  /// each camera's `color_to_world` the inverse of recon's
  /// `matrix_from_rodrigues` of its T_cw (the reference's exactly the
  /// identity), and its colour lens as given, marked factory: the adjustment
  /// solves no intrinsics. Sensors in id order; no depth cameras.
  recon::camera::ArrayCalibration calibration;
  /// The board in each capture adjusted, in the captures' order.
  std::vector<SolvedBoard> boards;
  /// Every corner of every view adjusted, in the observations' order.
  std::vector<CornerResidual> residuals;
  /// What was left out, in the order it was.
  std::vector<Rejection> rejections;
  /// Each camera's uncertainty, in the rig's order.
  std::vector<PoseCovariance> covariance;
  /// The residuals' variance: `sum w |e|^2 / (m - n)` over the kept
  /// corners, `w` each one's weight times its Huber weight, `m` the
  /// residuals of those whose weight is positive and `n` the parameters.
  double variance = 0.0;
  /// What each camera's intrinsics being off does, in the rig's order.
  std::vector<IntrinsicsSensitivity> sensitivity;
  /// The tree that placed the first rig (@ref InitialRig::links).
  std::vector<PlacedLink> links;
  int solves = 0;      ///< The solves run: 1 and an outlier round's each.
  int iterations = 0;  ///< `cv::LevMarq`'s iterations, over every solve.
  /// The energy, `sum weight * huber(|e|)`, as the first solve began.
  double initial_energy = 0.0;
  double final_energy = 0.0;  ///< The energy as the last solve ended.
  double seconds = 0.0;       ///< How long the adjustment took.
};

/// @brief Refine @p start: every non-reference camera and every board, from
///        the views @p start uses or left as ambiguous.
///
/// 1. **Views.** A view of @p start's `Used` or `Ambiguous` fate is adjusted,
///    on its capture's board; every other is left out (@p start says why).
/// 2. **Solves.** `cv::LevMarq` with a dense normal-equations callback
///    (Cholesky) minimises `sum weight * huber(|e|)` over the kept corners,
///    each corner's Huber weight its IRLS weight in `J^T W J` and `J^T W e`.
///    It stops after `max_iterations`, or when the energy's relative change
///    or the step's squared norm falls below 1e-12. As each solve begins,
///    corners nearer their cameras than `min_depth_m` are dropped and every
///    rotation's angle is brought within pi. A step that would bring a kept
///    corner nearer than `min_depth_m`, or make a value not finite, has
///    infinite energy, so `cv::LevMarq` tries a shorter one.
/// 3. **Outlier rounds.** After each solve, up to `outlier_rounds` times:
///    the views whose RMS is above both `outlier_view_ratio` times the median
///    view's and `outlier_view_px`, and in the others the corners whose
///    residual is above both `outlier_px` and `outlier_mads` robust
///    deviations, are dropped, and the rig is solved again from where it was.
///    A view's RMS leaves out its outlier corners, unless all are, so one bad
///    corner costs its view nothing more. A view whose kept corners no longer
///    fix a pose is dropped whole. The rounds stop when none is dropped.
/// 4. **Guards.** Each link of @p start's tree keeps `min_shared` captures
///    in which both its cameras' kept, positive-weight corners fix a pose
///    (@ref compute::usable_for_pose); each camera's 6 x 6 block of the normal
///    matrix, the boards eliminated, has a condition number below
///    `max_condition`.
/// 5. **Uncertainty.** The covariance (@ref PoseCovariance) and the
///    intrinsics sensitivity (@ref IntrinsicsSensitivity), at the solution.
///
/// The result depends on the cameras' ids, not on the order of the cameras or
/// of the observations, and, but for @ref RigSolution::seconds, is the same
/// on every run.
/// @param cameras  The rig's cameras, as @ref initialise took them.
/// @param board    The board seen.
/// @param views    The observations, as @ref initialise took them; corner
///                 weights are used as given.
/// @param start    @ref initialise's rig for them, or a rig like it: its
///                 reference at exactly the identity, each view's record in
///                 the observations' order.
/// @param options  The thresholds and amounts.
/// @return The solution; `InvalidArgument` for an option out of range
///         (naming it), cameras @ref initialise would refuse, a @p start that
///         does not fit them or @p views (naming what does not), a view
///         adjusted that is not usable, a weight negative or not finite, more
///         captures than `max_captures`, or a link left short of
///         `min_shared` captures (naming its cameras); `Numerical` when a
///         solve does not converge or meets a value that is not finite, for
///         fewer weighted residuals than parameters, for a board or a camera
///         poorly constrained (naming its capture or its id), or for the
///         cameras poorly constrained together, or for sensitivity diagnostics
///         that are not finite; or what OpenCV threw (@ref compute::guarded).
///         Each message starts `bundle_adjust: `.
VC_SOLVE_API core::Result<RigSolution> bundle_adjust(
    const std::vector<RigCamera>& cameras, const compute::Board& board,
    const std::vector<Observation>& views, const InitialRig& start,
    const BundleAdjustOptions& options = {});

}  // namespace volumetric_kit::calib::solve
