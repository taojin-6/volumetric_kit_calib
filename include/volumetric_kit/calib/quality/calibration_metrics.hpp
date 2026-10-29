// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file calibration_metrics.hpp
/// @brief What a solved rig is worth: its reprojection error before and
///        after the adjustment, what the adjustment left out, how well each
///        camera's view of a capture predicts the others' (cross-camera
///        transfer), how far each camera moves when one capture is left out,
///        whether the principal points fit better 2.5 px on, and where the
///        cameras are.
///
/// Every error is in pixels, per corner, unweighted: the Euclidean norm of a
/// corner's projection minus its detection. The adjustment's own
/// uncertainty -- each camera's covariance and its sensitivity to the
/// intrinsics -- is in its @ref solve::RigSolution. The checks of depth are
/// not here. Geometric and on the host: each function takes the solve's
/// detections and poses, a few thousand corners.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/quality/export.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"

namespace volumetric_kit::calib::quality {

/// @brief A group of corners' errors.
///
/// @code
/// const ErrorStats& camera = metrics.reprojection.solved.cameras[c];
/// show(camera.rms_px, camera.p90_px, camera.mean_px);  // mean_px: a bias
/// @endcode
struct ErrorStats {
  std::size_t count = 0;  ///< The corners.
  /// The root of the mean squared error; every statistic is not a number
  /// when @ref count is 0.
  double rms_px = std::numeric_limits<double>::quiet_NaN();
  /// The 90th percentile, interpolated linearly between ranks (numpy's
  /// default).
  double p90_px = std::numeric_limits<double>::quiet_NaN();
  double max_px = std::numeric_limits<double>::quiet_NaN();  ///< The largest.
  /// The mean residual vector: a bias hint, where a lens or a pose is off.
  sensor::camera::Vec2d mean_px{std::numeric_limits<double>::quiet_NaN()};
};

/// @brief How many corners err how far.
///
/// @code
/// for (std::size_t i = 0; i < h.counts.size(); ++i) {
///   bar(i * h.bin_px, h.counts[i]);
/// }
/// @endcode
struct Histogram {
  double bin_px = 0.0;  ///< Each bin's width.
  /// Bin i counts the errors in `[i bin_px, (i + 1) bin_px)`; the last, one
  /// past `MetricsOptions::histogram_bins`, every error beyond them.
  std::vector<std::size_t> counts;
};

/// @brief One capture's errors.
///
/// @code
/// for (const CaptureErrors& c : metrics.reprojection.solved.captures) {
///   if (c.errors.max_px > 2.0) recheck(c.capture);
/// }
/// @endcode
struct CaptureErrors {
  std::int64_t capture = 0;  ///< The capture, as `Observation::frame_id`.
  ErrorStats errors;         ///< Its corners' errors, every camera's.
};

/// @brief One view's errors.
///
/// @code
/// for (const ViewErrors& v : metrics.reprojection.solved.views) {
///   if (!v.oblique) show(ids[v.camera], v.capture, v.errors.rms_px);
/// }
/// @endcode
struct ViewErrors {
  std::size_t camera = 0;    ///< Its camera's index in the rig.
  std::int64_t capture = 0;  ///< Its capture, as `Observation::frame_id`.
  /// Tilted more than the solve's `oblique_deg` (@ref solve::ViewRecord).
  bool oblique = false;
  ErrorStats errors;  ///< Its corners' errors.
};

/// @brief The reprojection errors at one state of the rig, over the corners
///        the adjustment kept.
///
/// @code
/// const ReprojectionStats& solved = metrics.reprojection.solved;
/// log(solved.overall.rms_px, solved.without_oblique.rms_px);
/// @endcode
struct ReprojectionStats {
  ErrorStats overall;  ///< Every corner kept.
  /// The corners of the views not oblique (@ref ViewErrors::oblique); its
  /// legacy RMSE is its `rms_px / sqrt(2)`.
  ErrorStats without_oblique;
  /// The root of the mean squared residual component, x and y alike, as the
  /// legacy solve reports it: `overall.rms_px / sqrt(2)`.
  double legacy_rmse_px = std::numeric_limits<double>::quiet_NaN();
  std::vector<ErrorStats> cameras;  ///< Each camera's, in the rig's order.
  /// Each capture's with a corner kept, in ascending order.
  std::vector<CaptureErrors> captures;
  /// Each view's with a corner kept, in the observations' order.
  std::vector<ViewErrors> views;
  Histogram histogram;  ///< Of every corner kept.
};

/// @brief The reprojection errors before and after the adjustment, over the
///        same corners.
///
/// @code
/// const Reprojection& r = metrics.reprojection;
/// log(r.initial.legacy_rmse_px, r.solved.legacy_rmse_px,
///     r.legacy_reduction_percent);
/// @endcode
struct Reprojection {
  ReprojectionStats initial;  ///< At the first rig (@ref solve::initialise).
  ReprojectionStats solved;   ///< At the solution.
  /// How much less the legacy RMSE is after: `100 (1 - solved / initial)`;
  /// not a number when the initial is 0.
  double legacy_reduction_percent = std::numeric_limits<double>::quiet_NaN();
};

/// @brief What the adjustment left out for one reason in one round.
///
/// @code
/// for (const RejectionCount& r : metrics.rejections) {
///   log(r.round, solve::to_string(r.reason), r.corners, r.views);
/// }
/// @endcode
struct RejectionCount {
  int round = 0;  ///< As @ref solve::Rejection::round.
  solve::Dropped reason = solve::Dropped::Outlier;  ///< Why.
  std::size_t corners = 0;                          ///< The corners left out.
  std::size_t views = 0;  ///< The whole views left out.
};

/// @brief One capture's board, posed from one camera's view of it, as
///        another camera sees it.
///
/// @code
/// for (const TransferError& e : transfer.errors) {
///   if (e.error_px > 10.0) recheck(e.capture, ids[e.from], ids[e.to]);
/// }
/// @endcode
struct TransferError {
  std::int64_t capture = 0;  ///< The capture, as `Observation::frame_id`.
  std::size_t from = 0;      ///< The camera whose view posed the board.
  std::size_t to = 0;        ///< The camera it was carried into.
  /// The mean, over every corner @ref to detected, of the distance from its
  /// detection to where the board carried into it projects it; infinite when
  /// one does not project (behind the camera).
  double error_px = 0.0;
};

/// @brief One ordered pair of cameras' transfer, over the captures both
///        pose.
///
/// @code
/// const PairTransfer& p = transfer.pairs[from][to];
/// if (p.count > 0) cell(from, to, p.mean_px);
/// @endcode
struct PairTransfer {
  std::size_t count = 0;  ///< The captures.
  /// Their errors' mean; not a number for no captures.
  double mean_px = std::numeric_limits<double>::quiet_NaN();
};

/// @brief How well each camera's view of a capture predicts the others',
///        through the solved poses: the legacy `verify_rig.py`'s check,
///        independent of the adjustment's own residuals.
///
/// @code
/// VKC_ASSIGN(const Transfer t,
///            cross_camera_transfer(cameras, board, views, poses));
/// log(t.median_px, t.p90_px, t.worst_px);
/// @endcode
struct Transfer {
  /// Every capture's every ordered pair of cameras that both pose its board,
  /// captures in ascending order, then `from` and `to` in the rig's.
  std::vector<TransferError> errors;
  /// The errors' median, interpolated as @ref ErrorStats::p90_px; every
  /// statistic is not a number for no errors.
  double median_px = std::numeric_limits<double>::quiet_NaN();
  double p90_px =
      std::numeric_limits<double>::quiet_NaN();  ///< 90th percentile.
  double worst_px = std::numeric_limits<double>::quiet_NaN();  ///< The largest.
  /// Each ordered pair's, `pairs[from][to]`, the rig's order on both.
  std::vector<std::vector<PairTransfer>> pairs;
};

/// @brief How far a camera's solved pose moved.
///
/// @code
/// const PoseChange& moved = left.cameras[c];
/// log(moved.centre_mm, moved.rotation_deg);
/// @endcode
struct PoseChange {
  double centre_mm = 0.0;     ///< Its centre's move (millimetres).
  double rotation_deg = 0.0;  ///< Its rotation's turn (degrees).
};

/// @brief The rig re-solved without one capture.
///
/// @code
/// for (const LeftOut& left : loo.captures) {
///   if (!left.refused.empty()) note(left.capture, left.refused);
/// }
/// @endcode
struct LeftOut {
  std::int64_t capture = 0;  ///< The capture, as `Observation::frame_id`.
  /// Why the rig without it could not be solved, e.g. a link left short of
  /// `min_shared` captures (@ref solve::bundle_adjust's message); empty
  /// when it was.
  std::string refused;
  /// Each camera's move from the solution, in the rig's order; empty when
  /// refused.
  std::vector<PoseChange> cameras;
  /// The re-solve's RMS over the corners it kept; not a number when
  /// refused.
  double rms_px = std::numeric_limits<double>::quiet_NaN();
  /// Their legacy RMSE (@ref ReprojectionStats::legacy_rmse_px): the legacy
  /// solve's figure for the rig without this capture.
  double legacy_rmse_px = std::numeric_limits<double>::quiet_NaN();
};

/// @brief How far one camera moves, at most, when one capture is left out,
///        and which capture moves it so.
///
/// @code
/// const CameraStability& s = loo.cameras[c];
/// if (s.centre_mm > 3.0 && s.centre_capture) recapture(*s.centre_capture);
/// @endcode
struct CameraStability {
  double centre_mm = 0.0;  ///< The largest move of its centre.
  /// The capture whose leaving out moved it so; empty when none moved it.
  std::optional<std::int64_t> centre_capture;
  double rotation_deg = 0.0;  ///< The largest turn of its rotation.
  /// The capture whose leaving out turned it so; empty when none turned it.
  std::optional<std::int64_t> rotation_capture;
};

/// @brief How much the solution rests on each capture: the rig re-solved
///        without each one in turn.
///
/// @code
/// VKC_ASSIGN(const LeaveOneOut loo,
///            leave_one_out(cameras, board, views, rig, solution));
/// const CameraStability& most = loo.cameras[c];
/// if (most.centre_capture) recapture(*most.centre_capture);
/// @endcode
struct LeaveOneOut {
  /// Each capture adjusted, in ascending order.
  std::vector<LeftOut> captures;
  std::vector<CameraStability> cameras;  ///< In the rig's order.
};

/// @brief Which principal points the A/B test favours: the side whose RMS is
///        less by the margin (@ref principal_point_verdict).
enum class PrincipalPointVerdict {
  AsGiven,  ///< The lenses as given.
  Shifted,  ///< The principal points shifted.
  /// Neither side's RMS is less by the margin, or either is not a finite
  /// number.
  Inconclusive,
};

/// @brief The name the report gives @p verdict.
/// @param verdict  A verdict.
/// @return `"as_given"`, `"shifted"` or `"inconclusive"`; `"unknown"` for a
///         value @ref PrincipalPointVerdict does not name.
///
/// @code
/// show(to_string(metrics.principal_point.verdict));
/// @endcode
VC_QUALITY_API const char* to_string(PrincipalPointVerdict verdict) noexcept;

/// @brief One side of the principal-point A/B test.
///
/// @code
/// const PrincipalPointArm& a = ab.shifted;
/// log(a.rms_px, a.transfer_median_px);
/// @endcode
struct PrincipalPointArm {
  /// The RMS over the corners both sides' adjustments kept.
  double rms_px = std::numeric_limits<double>::quiet_NaN();
  /// Their legacy RMSE (@ref ReprojectionStats::legacy_rmse_px).
  double legacy_rmse_px = std::numeric_limits<double>::quiet_NaN();
  /// The cross-camera transfer's median, with this side's lenses and poses.
  double transfer_median_px = std::numeric_limits<double>::quiet_NaN();
  /// Its 90th percentile.
  double transfer_p90_px = std::numeric_limits<double>::quiet_NaN();
};

/// The A/B verdict's margin unless told otherwise, as a fraction of the
/// larger side's RMS (@ref principal_point_verdict).
inline constexpr double kPrincipalMarginFraction = 0.02;
/// The A/B verdict's margin's cap unless told otherwise (pixels).
inline constexpr double kPrincipalMarginPx = 0.05;

/// @brief The side of the principal-point A/B test whose RMS is less by the
///        margin: `min(margin_fraction * max(a, s), margin_px)`, for the RMS
///        `a` as given and `s` shifted. The transfer is information, not
///        taken.
/// @param as_given         The lenses as given.
/// @param shifted          The principal points shifted.
/// @param margin_fraction  The margin's fraction of the larger RMS; in
///                         (0, 1).
/// @param margin_px        The margin's cap (pixels); positive and finite.
/// @return `Shifted` when `a - s` is positive and at least the margin,
///         `AsGiven` when `s - a` is, else `Inconclusive`, as it is when
///         either RMS is not a finite number.
///
/// @code
/// ab.verdict = principal_point_verdict(ab.as_given, ab.shifted);
/// @endcode
VC_QUALITY_API PrincipalPointVerdict principal_point_verdict(
    const PrincipalPointArm& as_given, const PrincipalPointArm& shifted,
    double margin_fraction = kPrincipalMarginFraction,
    double margin_px = kPrincipalMarginPx) noexcept;

/// @brief The principal-point A/B test's gap relative to the larger RMS:
///        `(a - s) / max(a, s)`, for the RMS `a` as given and `s` shifted.
/// @param as_given  The lenses as given.
/// @param shifted   The principal points shifted.
/// @return The gap, positive when the shifted side's RMS is less; NaN when
///         either RMS is not a finite number, or both are 0.
///
/// @code
/// show(100.0 * principal_point_gap(ab.as_given, ab.shifted), "%");
/// @endcode
VC_QUALITY_API double principal_point_gap(
    const PrincipalPointArm& as_given,
    const PrincipalPointArm& shifted) noexcept;

/// @brief Whether every camera's principal point fits better shifted: the
///        rig re-solved with `cx` and `cy` @ref shift_px larger on every
///        camera, against the solution.
///
/// A Femto Mega's SDK gives its 4K intrinsics as a 640x360 group's times 6,
/// with no half-pixel term; if that group's pixel centres are at integers,
/// the 4K principal point is 2.5 px low.
///
/// @code
/// VKC_ASSIGN(const PrincipalPointAB ab,
///            principal_point_ab(cameras, board, views, rig, solution));
/// log(to_string(ab.verdict), ab.as_given.rms_px, ab.shifted.rms_px);
/// @endcode
struct PrincipalPointAB {
  double shift_px = 0.0;  ///< The shift tried, on `cx` and `cy` (pixels).
  /// Why the shifted rig could not be solved (@ref solve::bundle_adjust's
  /// message); empty when it was. When refused, @ref corners is 0, the arms
  /// are not numbers and the verdict is `Inconclusive`.
  std::string refused;
  /// The corners both sides' adjustments kept, which the RMS is over.
  std::size_t corners = 0;
  PrincipalPointArm as_given;  ///< The lenses as given: the solution.
  PrincipalPointArm shifted;   ///< The principal points shifted, re-solved.
  /// Which side's RMS is less by the margin (@ref principal_point_verdict).
  PrincipalPointVerdict verdict = PrincipalPointVerdict::Inconclusive;
};

/// @brief Where the solved cameras are, in the world (the reference's colour
///        frame).
///
/// @code
/// const RigGeometry& g = metrics.geometry;
/// log(ids[1], g.centres_m[1], g.baselines_m[0][1]);  // against a tape
/// @endcode
struct RigGeometry {
  /// Each camera's centre, `-R^T t` of its T_cw (metres), in the rig's order.
  std::vector<sensor::camera::Vec3d> centres_m;
  /// The distance between each two cameras' centres (metres),
  /// `baselines_m[a][b]`.
  std::vector<std::vector<double>> baselines_m;
};

/// The principal points' shift the A/B test tries unless told otherwise
/// (pixels), on `cx` and `cy`.
inline constexpr double kPrincipalShiftPx = 2.5;

/// @brief The settings behind the metrics, and those the report records.
///
/// @code
/// MetricsOptions options;
/// options.bundle_adjust = adjust_options;  // as the solve took them
/// @endcode
struct MetricsOptions {
  /// What @ref solve::initialise took: the report records it.
  solve::InitialiseOptions initialise;
  /// What @ref solve::bundle_adjust took: the report records it, and the
  /// re-solves (@ref leave_one_out, @ref principal_point_ab) take it.
  solve::BundleAdjustOptions bundle_adjust;
  /// The principal points' shift the A/B test tries (pixels); finite, not 0.
  double principal_shift_px = kPrincipalShiftPx;
  /// The A/B verdict's margin as a fraction of the larger RMS
  /// (@ref principal_point_verdict); in (0, 1).
  double principal_margin_fraction = kPrincipalMarginFraction;
  /// The A/B verdict's margin's cap (pixels); positive and finite.
  double principal_margin_px = kPrincipalMarginPx;
  double histogram_bin_px = 0.1;  ///< The histograms' bin width; positive.
  int histogram_bins = 30;        ///< Their bins before the last; at least 1.
};

/// @brief Everything @ref calibration_metrics measures.
///
/// @code
/// VKC_ASSIGN(const CalibrationMetrics m,
///            calibration_metrics(cameras, board, views, rig, solution));
/// log(m.reprojection.solved.legacy_rmse_px, m.transfer.median_px);
/// @endcode
struct CalibrationMetrics {
  MetricsOptions options;     ///< The settings they were measured with.
  Reprojection reprojection;  ///< Before and after the adjustment.
  /// The views @ref solve::initialise left out, which the adjustment did not
  /// take (@ref solve::adjusts), in the observations' order, so each can be
  /// named and recaptured.
  std::vector<solve::ViewRecord> views_left_out;
  /// What the adjustment left out, by round, then by reason, in
  /// @ref solve::Dropped's order.
  std::vector<RejectionCount> rejections;
  Transfer transfer;                 ///< At the solution.
  LeaveOneOut leave_one_out;         ///< Each capture left out.
  PrincipalPointAB principal_point;  ///< The principal-point A/B test.
  RigGeometry geometry;              ///< Where the cameras are.
};

/// @brief How far through the leave-one-out re-solves: called on the
///        calling thread after each one.
/// @param done   The re-solves done.
/// @param total  The re-solves in all.
using Progress = std::function<void(std::size_t done, std::size_t total)>;

/// @brief How well each camera's view of a capture predicts the others',
///        through @p world_to_camera: the legacy `verify_rig.py`'s check.
///
/// For each capture, each view @ref compute::usable_for_pose accepts is
/// posed on its own (@ref solve::view_pose's better fit, T_cb). For each
/// ordered pair of cameras that both pose the capture's board, A's pose is
/// carried into B, `T_cw(B) T_cw(A)^-1 T_cb(A)`, and every corner B detected
/// is projected through B's lens; the pair's error is the mean distance from
/// each to its detection. A view with no pose in front of its camera poses
/// nothing. Corner weights are not used.
/// @param cameras          The rig's cameras.
/// @param board            The board seen.
/// @param views            Every camera's view of the board in every capture:
///                         at most one per camera and capture.
/// @param world_to_camera  Each camera's T_cw, in the rig's order.
/// @return The transfer; `InvalidArgument` for no cameras, a repeated or
///         empty id, a lens @ref compute::cv_camera refuses, a pose for each
///         camera missing or not finite, a view naming a camera the rig does
///         not have, or two views of one capture by one camera;
///         @ref solve::view_pose's `Backend` error when IPPE fails; or what
///         OpenCV threw (@ref compute::guarded). Each message starts
///         `cross_camera_transfer: `.
VC_QUALITY_API core::Result<Transfer> cross_camera_transfer(
    const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
    const std::vector<Observation>& views,
    const std::vector<sensor::camera::RodriguesTransform>& world_to_camera);

/// @brief The rig re-solved without each capture the adjustment took, in
///        turn, warm: from @p solution's poses and the corners it kept, with
///        @p start's tree and view records less the capture's.
///
/// Each re-solve is one solve: @p options with no outlier rounds and no
/// uncertainty step (@ref solve::BundleAdjustOptions::uncertainty). Each
/// camera's move is its centre's and its rotation's from @p solution's, and
/// each re-solve's RMS is over the corners it kept. A re-solve
/// @ref solve::bundle_adjust refuses as `InvalidArgument` or `Numerical` --
/// a link left short of `min_shared` captures, say -- is recorded as
/// refused, and moves nothing.
/// @param cameras   The rig's cameras, as the solve took them.
/// @param board     The board seen.
/// @param views     The observations, as the solve took them.
/// @param start     @ref solve::initialise's rig, which the adjustment
///                  started from.
/// @param solution  The adjustment's solution.
/// @param options   The adjustment's options, which each re-solve takes.
/// @param progress  Called after each re-solve; may be empty.
/// @return The re-solves; `InvalidArgument` for an option out of range
///         (naming it) or a @p solution that does not fit the others (naming
///         what does not); another error of a re-solve's, such as
///         `OutOfMemory` or OpenCV's (@ref compute::guarded); or what
///         @p progress threw. Each message starts `leave_one_out: `.
VC_QUALITY_API core::Result<LeaveOneOut> leave_one_out(
    const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
    const std::vector<Observation>& views, const solve::InitialRig& start,
    const solve::RigSolution& solution,
    const solve::BundleAdjustOptions& options = {},
    const Progress& progress = {});

/// @brief The principal-point A/B test: the rig re-solved, warm from
///        @p solution, with every camera's `cx` and `cy` @p shift_px larger,
///        and both sides' RMS and cross-camera transfer
///        (@ref cross_camera_transfer).
///
/// The re-solve is one solve of the corners @p solution kept, as
/// @ref leave_one_out's are, so both sides' RMS is over the same corners. A
/// re-solve @ref solve::bundle_adjust refuses as `InvalidArgument` or
/// `Numerical` is recorded as refused (@ref PrincipalPointAB::refused), as
/// @ref leave_one_out records one.
/// @param cameras   The rig's cameras, as the solve took them.
/// @param board     The board seen.
/// @param views     The observations, as the solve took them.
/// @param start     @ref solve::initialise's rig, which the adjustment
///                  started from.
/// @param solution  The adjustment's solution.
/// @param options   The adjustment's options, which the re-solve takes.
/// @param shift_px  The shift (pixels); finite, not 0.
/// @param margin_fraction  The verdict's margin's fraction of the larger RMS
///                         (@ref principal_point_verdict); in (0, 1).
/// @param margin_px        The verdict's margin's cap (pixels); positive and
///                         finite.
/// @return The test; `InvalidArgument` for an option, a shift or a margin out
///         of range (naming it), or when @p solution does not fit the others;
///         or another error of the re-solve's, or the transfer's. Each
///         message starts `principal_point_ab: `.
VC_QUALITY_API core::Result<PrincipalPointAB> principal_point_ab(
    const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
    const std::vector<Observation>& views, const solve::InitialRig& start,
    const solve::RigSolution& solution,
    const solve::BundleAdjustOptions& options = {},
    double shift_px = kPrincipalShiftPx,
    double margin_fraction = kPrincipalMarginFraction,
    double margin_px = kPrincipalMarginPx);

/// @brief Everything a solved rig is measured by.
///
/// - **Reprojection.** Over the corners @p solution kept, at @p start's
///   poses and at @p solution's: overall, without the oblique views, per
///   camera, per capture and per view, the legacy RMSE and a histogram.
/// - **Rejections.** The views @p start left out, and @p solution's
///   rejections, counted by round and reason.
/// - **Transfer** (@ref cross_camera_transfer), **leave one out**
///   (@ref leave_one_out, with @p progress) and the **principal-point A/B
///   test** (@ref principal_point_ab), each as its function gives it.
/// - **Geometry.** The cameras' centres and baselines.
/// @param cameras   The rig's cameras, as the solve took them.
/// @param board     The board seen.
/// @param views     The observations, as the solve took them.
/// @param start     @ref solve::initialise's rig, which the adjustment
///                  started from.
/// @param solution  The adjustment's solution.
/// @param options   The settings.
/// @param progress  As @ref leave_one_out's; may be empty.
/// @return The metrics; `InvalidArgument` for an option out of range (naming
///         it) or a @p solution that does not fit the others (naming what
///         does not); or a part's error. Each message starts
///         `calibration_metrics: `.
VC_QUALITY_API core::Result<CalibrationMetrics> calibration_metrics(
    const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
    const std::vector<Observation>& views, const solve::InitialRig& start,
    const solve::RigSolution& solution, const MetricsOptions& options = {},
    const Progress& progress = {});

}  // namespace volumetric_kit::calib::quality
