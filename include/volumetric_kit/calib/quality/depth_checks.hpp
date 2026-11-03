// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file depth_checks.hpp
/// @brief What each camera's depth says of the solved rig: the board's scale
///        and plane from depth, and how well each ordered pair of cameras'
///        depth agrees, with a verdict on each.
///
/// The device measures and the host does the geometry. For each view of a
/// board, the device fits a plane (@ref DepthPlane) to the depth pixels in
/// the view's @ref BoardRegion, and for each ordered pair of cameras it
/// summarizes how far one camera's depth lies from the other's
/// (@ref PairAgreement); only those few numbers come back. Here, on the host,
/// with the solver's detections and poses: the views a solved rig's checks
/// fit and the region each fit takes (@ref depth_views, @ref board_region),
/// the corners' rays met with the fitted plane (@ref board_depth_check), and
/// the verdicts over every view and pair (@ref summarize_depth_checks). Any
/// number of cameras.

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/quality/export.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"

namespace volumetric_kit::calib::quality {

/// How far inside the chessboard's outer edge the board region stays unless
/// told otherwise (metres), clear of the depth's flying pixels at the
/// board's edge.
inline constexpr double kDepthInsetM = 0.02;

/// The fewest samples a pair is judged on unless told otherwise
/// (@ref DepthCheckOptions::min_overlap).
inline constexpr std::size_t kMinOverlap = 1000;

/// The fewest inliers a plane is fitted from unless told otherwise
/// (@ref DepthCheckOptions::min_inliers).
inline constexpr std::size_t kMinPlaneInliers = 100;

/// The most bins a pair's histogram of depth differences has
/// (@ref DepthCheckOptions::window_mm); DECISIONS.md, "Depth checks are
/// calib's kernels", says why.
inline constexpr std::size_t kMaxAgreementBins = 4096;

// TODO: The verdict bounds, the fit's gates and the agreement's window and
// edge gate are first values, to be set from a real session (H5).
/// @brief The depth checks' settings, one object for every step: the region
///        each plane fit takes, the fit's gates, how the pairs' depth is
///        compared, the verdicts' bounds, and what the report records. Give
///        the same to @ref board_region, to the depth tier's kernels and to
///        @ref summarize_depth_checks.
///
/// @code
/// DepthCheckOptions options;
/// options.max_scatter_mm = 2.0;
/// @endcode
struct DepthCheckOptions {
  /// How far inside the chessboard's outer edge the board region stays
  /// (metres, @ref board_region); finite and not negative.
  double inset_m = kDepthInsetM;
  /// The plane fit's first round takes the region's depth points within
  /// this of the solved board's plane (millimetres).
  double seed_mm = 30.0;
  /// Its second round takes those within `max(min_gate_mm, sigma_gate rms)`
  /// of the first round's plane, `rms` being the first round's RMS distance
  /// from it (millimetres).
  double min_gate_mm = 5.0;
  double sigma_gate = 3.0;  ///< See @ref min_gate_mm.
  /// The fewest inliers a plane is fitted from (@ref DepthPlane::valid); at
  /// least 3.
  std::size_t min_inliers = kMinPlaneInliers;
  /// A pair's differences in [-window_mm, window_mm) are binned
  /// (@ref PairAgreement::in_window), at @ref bin_mm: an even number of
  /// bins, `2 window_mm / bin_mm`, at most @ref kMaxAgreementBins
  /// (millimetres).
  double window_mm = 100.0;
  double bin_mm = 0.1;  ///< See @ref window_mm.
  /// A pair's sample is dropped when its pixel in either camera has a
  /// 4-neighbour with depth in range differing from its own by more than
  /// this fraction of it: a flying pixel at a depth step.
  double edge_ratio = 0.03;
  /// A pair takes every this-th pixel of each row and column of the moved
  /// camera's depth, from the first; at least 1.
  std::uint32_t agreement_stride = 1;
  /// The fewest samples a pair is judged on (@ref PairCheck::verdict); at
  /// least 1.
  std::size_t min_overlap = kMinOverlap;
  /// The scale's bound on `|mean ratio - 1|` (@ref CameraDepth::scale).
  double max_scale_error = 0.002;
  /// The plane's bound on its views' scatter once the camera's depth offset
  /// is taken off (millimetres, @ref CameraDepth::scatter_mm): a pair's
  /// bound on its signed median, since what is left is depth error another
  /// camera would see.
  double max_scatter_mm = 5.0;
  /// The plane's bound on the median angle (degrees).
  double max_angle_deg = 1.0;
  /// A pair's bound on its signed median (millimetres,
  /// @ref PairCheck::verdict).
  double max_signed_median_mm = 5.0;
  /// A pair's bound on its median magnitude (millimetres).
  double max_median_abs_mm = 10.0;
};

/// @brief Whether @p options are in range: the inset finite and not
///        negative, `min_overlap` and `agreement_stride` at least 1,
///        `min_inliers` at least 3, every other setting positive and finite,
///        and the window a whole number of bins, at most
///        @ref kMaxAgreementBins in all.
///
/// What every step that takes the options refuses, as its own refusal.
/// @param options  The settings.
/// @return OK; or `InvalidArgument` naming the first setting out of range,
///         e.g. `options.seed_mm must be positive and finite`; or
///         `OutOfMemory`.
///
/// @code
/// VKC_TRY(check_depth_options(options).with_context("fit_planes"));
/// @endcode
VC_QUALITY_API core::Status check_depth_options(
    const DepthCheckOptions& options);

/// @brief The depth pixels one view's plane fit takes: those whose ray meets
///        the solved board's plane inside its chessboard, inset.
///
/// The board frame is the solved board's (@ref compute::Board): z = 0 is its
/// plane. A pixel belongs by its ray alone (@ref in_board_region), with depth
/// or without.
///
/// @code
/// VKC_ASSIGN(const BoardRegion region,
///            board_region(board, board_to_color, info.depth_to_color,
///                         options));
/// upload(Mat4f(region.depth_to_board), region.min, region.max);  // device
/// @endcode
struct BoardRegion {
  /// T_cb: the solved board in the colour camera, which carries the fitted
  /// plane into the colour camera's frame (@ref board_depth_check).
  sensor::camera::Mat4d board_to_color{1.0};
  /// The depth camera's frame to the board's: `T_cb^-1 depth_to_color`.
  sensor::camera::Mat4d depth_to_board{1.0};
  /// The inset chessboard's corner nearest the board frame's origin, on
  /// z = 0 (metres).
  sensor::camera::Vec2d min{0.0};
  sensor::camera::Vec2d max{0.0};  ///< Its far corner (metres).
};

/// @brief The region one view's plane fit takes.
/// @param board           The board seen.
/// @param board_to_color  T_cb: the solved board in the view's colour camera,
///                        e.g. `T_cw T_wb` of the solution.
/// @param depth_to_color  The camera's depth-to-colour transform (sensor's
///                        `SensorInfo::depth_to_color`).
/// @param options         The settings; its inset must leave some of the
///                        board.
/// @return The region; or `InvalidArgument` for a transform that is not
///         rigid (sensor's `check_rigid`), an option out of range (naming it,
///         as @ref summarize_depth_checks does) or an inset that leaves none
///         of the board, its message starting `board_region: `.
VC_QUALITY_API core::Result<BoardRegion> board_region(
    const compute::Board& board, const sensor::camera::Mat4d& board_to_color,
    const sensor::camera::Mat4d& depth_to_color,
    const DepthCheckOptions& options = {});

/// @brief Whether @p region is one a plane can be fitted in: both transforms
///        rigid (sensor's `check_rigid`), and its rectangle finite and not
///        empty.
/// @param region  The region.
/// @return OK; or `InvalidArgument` naming what is wrong; or `OutOfMemory`.
///
/// @code
/// VKC_TRY(check_board_region(region).with_context("fit_planes"));
/// @endcode
VC_QUALITY_API core::Status check_board_region(const BoardRegion& region);

/// @brief Whether a depth camera's ray belongs to @p region: it meets the
///        board's plane in front of the camera, inside the inset rectangle
///        (edges included).
///
/// The reference rule: the device classifies the pixels, and its tests check
/// it against this. A pixel's ray is its unprojection at depth 1,
/// `((u - cx) / fx, (v - cy) / fy, 1)` for the prepared depth's pinhole.
/// @param region  The region.
/// @param ray     A direction from the depth camera's centre, in its frame;
///                any positive length.
/// @return Whether it belongs; false for a ray along the plane or not
///         finite.
///
/// @code
/// EXPECT_EQ(device_member[i], in_board_region(region, ray_of(i)));
/// @endcode
VC_QUALITY_API bool in_board_region(const BoardRegion& region,
                                    const sensor::camera::Vec3d& ray) noexcept;

/// @brief One view of a solved rig whose depth a plane is fitted to.
///
/// @code
/// for (const DepthView& v : views) {
///   regions.push_back({images[v.camera], v.region});  // the device's fit
///   checked.push_back(board_depth_check(cameras, board, observations[v.view],
///                                       v.region, planes[i]));
/// }
/// @endcode
struct DepthView {
  std::size_t camera = 0;    ///< Its camera's index in the rig.
  std::int64_t capture = 0;  ///< Its capture, as `Observation::frame_id`.
  /// Its observation's index among those the solve took: its record's in
  /// `solve::InitialRig::views`.
  std::size_t view = 0;
  /// Its fit's region (@ref board_region): at the solution's board, its
  /// `board_to_color` the solved `T_cw T_wb`.
  BoardRegion region;
};

/// @brief The views a solved rig's depth checks fit a plane to: each one the
///        adjustment adjusted (`solve::adjusts`) and did not leave out whole,
///        on its capture's solved board.
/// @param board           The board seen.
/// @param start           `solve::initialise`'s rig, which @p solution
///                        refined: a record per observation.
/// @param solution        `solve::bundle_adjust`'s solution of it.
/// @param depth_to_color  Each camera's depth-to-colour transform (sensor's
///                        `SensorInfo::depth_to_color`), in the rig's order.
/// @param options         The settings; @ref board_region takes them.
/// @return The views, by capture, then camera; `InvalidArgument` for a
///         solution whose cameras are not as many as @p depth_to_color, a
///         record naming a camera it does not have, or what
///         @ref board_region refuses; or `OutOfMemory`. Each message starts
///         `depth_views: `.
///
/// @code
/// VKC_ASSIGN(const std::vector<DepthView> views,
///            depth_views(board, start, solution, depth_to_color));
/// @endcode
VC_QUALITY_API core::Result<std::vector<DepthView>> depth_views(
    const compute::Board& board, const solve::InitialRig& start,
    const solve::RigSolution& solution,
    const std::vector<sensor::camera::Mat4d>& depth_to_color,
    const DepthCheckOptions& options = {});

/// @brief A plane fitted to one view's depth in its @ref BoardRegion, as the
///        device gives it, in the region's board frame.
///
/// Its counts satisfy `inliers + out_of_range <= members`.
///
/// @code
/// DepthPlane plane;  // from the device's readback
/// plane.valid = true;
/// plane.point = centroid;
/// plane.normal = normal;
/// @endcode
struct DepthPlane {
  /// Whether enough members fitted a plane; when not, @ref point,
  /// @ref normal and @ref rms_m mean nothing.
  bool valid = false;
  /// A point on the plane, the inliers' centroid (metres, board frame).
  sensor::camera::Vec3d point{std::numeric_limits<double>::quiet_NaN()};
  /// The plane's normal (board frame): either sign, of any finite length
  /// but 0.
  sensor::camera::Vec3d normal{std::numeric_limits<double>::quiet_NaN()};
  /// The inliers' RMS distance from the plane (metres).
  double rms_m = std::numeric_limits<double>::quiet_NaN();
  /// The depth pixels in the region (@ref in_board_region), with depth or
  /// without.
  std::size_t members = 0;
  /// Those with no depth, or depth outside the camera's range.
  std::size_t out_of_range = 0;
  std::size_t inliers = 0;  ///< Those the final fit took.
};

/// @brief One view's board measured against its depth.
///
/// @code
/// VKC_ASSIGN(const BoardDepthCheck check,
///            board_depth_check(cameras, board, view, region, plane));
/// for (const double r : check.ratios) log(r * board.target().square_size_m);
/// @endcode
struct BoardDepthCheck {
  std::size_t camera = 0;    ///< Its camera's index in the rig.
  std::int64_t capture = 0;  ///< Its capture, as `Observation::frame_id`.
  DepthPlane plane;          ///< The plane fitted to its depth.
  /// For each two adjacent corners both detected whose rays meet the plane in
  /// front of the camera: the distance between the two meetings over the
  /// board's square size. In row-major order of the first corner's id, the
  /// one to its right before the one below. Empty when the plane is not
  /// valid.
  std::vector<double> ratios;
  /// How far the plane lies from the solved board at the chessboard's centre
  /// (@ref compute::Board::centre), along the board's z, into the board
  /// (millimetres): positive when the depth puts the board farther from a
  /// camera seeing its face. Not a number when the plane is not valid, or is
  /// perpendicular to the board.
  double offset_mm = std::numeric_limits<double>::quiet_NaN();
  /// @ref offset_mm made axial: how far the depth reads along its camera's
  /// own axis to put the plane there, `offset_mm d_z / (n . d)`, with `d`
  /// the unit ray from the depth camera to the chessboard's centre and `n`
  /// the board's z, both in the depth camera's frame (millimetres; positive
  /// when the depth reads too far). A board seen square on the axis gives
  /// the offset itself; seen at 60 degrees, about twice it. Not a number
  /// when the offset is not, or the centre is not in front of the depth
  /// camera.
  double axial_offset_mm = std::numeric_limits<double>::quiet_NaN();
  /// The angle between the plane and the solved board (degrees), in
  /// [0, 90]; not a number when the plane is not valid.
  double angle_deg = std::numeric_limits<double>::quiet_NaN();
};

/// @brief One view's board scale and plane against its depth: each
///        corner's ray (@ref compute::undistort_points) met with the fitted
///        plane, and the plane against the solved board.
///
/// The ratios do not depend on the solved board: the plane comes to the
/// colour camera through `T_cb T_cb^-1 depth_to_color`. The offset, its
/// axial form and the angle do; the axial form takes the depth camera from
/// the region's `depth_to_board`.
/// @param cameras  The rig's cameras.
/// @param board    The board seen.
/// @param view     The view's detection, at its camera's lens's size.
/// @param region   The region its plane was fitted in.
/// @param plane    The plane.
/// @return The check; `InvalidArgument` for ids @ref solve::check_camera_ids
///         refuses, a view naming no camera of the rig, a lens
///         @ref compute::cv_camera refuses, a corner id the board does not
///         have or repeated, a region that is not rigid or whose rectangle is
///         empty or not finite, or a plane whose counts do not add up or
///         that is valid with a point or normal not finite or a normal of 0;
///         or what OpenCV threw (@ref compute::guarded). Each message starts
///         `board_depth_check: `.
VC_QUALITY_API core::Result<BoardDepthCheck> board_depth_check(
    const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
    const Observation& view, const BoardRegion& region,
    const DepthPlane& plane);

/// @brief How far one camera's depth lies from another's where both see the
///        same surface, as the device gives it: each of @ref from's depth
///        pixels moved into @ref to's depth camera and compared with the depth
///        @ref to measured there.
///
/// Each sample's difference is the moved point's depth in @ref to's camera
/// minus @ref to's measured depth: positive when @ref from puts the surface
/// farther from @ref to than @ref to does.
///
/// @code
/// PairAgreement pair;  // from the device's readback
/// pair.from = 1;
/// pair.to = 0;
/// pair.overlap = counts.overlap;
/// @endcode
struct PairAgreement {
  std::size_t from = 0;  ///< The camera whose depth pixels are moved.
  std::size_t to = 0;    ///< The camera they are compared with; not @ref from.
  /// The capture they are of, as `Observation::frame_id`; empty when pooled
  /// over every capture checked.
  std::optional<std::int64_t> capture;
  /// The samples that met a pixel of @ref to's with depth, past the device's
  /// edge gate.
  std::size_t overlap = 0;
  /// Those whose difference is inside the device's window; at most
  /// @ref overlap.
  std::size_t in_window = 0;
  /// The median difference over the samples in the window (millimetres);
  /// not a number for none.
  double signed_median_mm = std::numeric_limits<double>::quiet_NaN();
  /// The median magnitude over the samples in the window (millimetres).
  double median_abs_mm = std::numeric_limits<double>::quiet_NaN();
  /// The 90th percentile of the magnitudes over the samples in the window
  /// (millimetres); no verdict rests on it.
  double p90_abs_mm = std::numeric_limits<double>::quiet_NaN();
};

/// @brief A depth check's verdict.
enum class DepthVerdict {
  Ok,            ///< Within its bounds.
  Off,           ///< Beyond a bound.
  Insufficient,  ///< Too little was measured to say.
};

/// @brief The name the report gives @p verdict.
/// @param verdict  A verdict.
/// @return `"ok"`, `"off"` or `"insufficient"`; `"unknown"` for a value
///         @ref DepthVerdict does not name.
///
/// @code
/// show(to_string(summary.scale));
/// @endcode
VC_QUALITY_API const char* to_string(DepthVerdict verdict) noexcept;

/// @brief A camera's depth verdict from its scale's and its plane's.
/// @param scale  Its board scale's verdict (@ref CameraDepth::scale).
/// @param plane  Its plane's (@ref CameraDepth::plane).
/// @return `Off` when either is `Off`; `Ok` when both are `Ok`; else
///         `Insufficient`.
///
/// @code
/// camera.verdict = camera_verdict(camera.scale, camera.plane);
/// @endcode
VC_QUALITY_API DepthVerdict camera_verdict(DepthVerdict scale,
                                           DepthVerdict plane) noexcept;

/// @brief A set of ratios' mean and spread.
///
/// @code
/// const RatioStats& r = summary.ratio;
/// log(r.mean * square_mm, r.sd * square_mm, r.count);  // 150.5 +- 0.6 mm
/// @endcode
struct RatioStats {
  std::size_t count = 0;  ///< The ratios.
  /// Their mean; not a number for none.
  double mean = std::numeric_limits<double>::quiet_NaN();
  /// Their sample standard deviation (over `count - 1`); not a number for
  /// fewer than 2.
  double sd = std::numeric_limits<double>::quiet_NaN();
};

/// @brief One camera's board checks over every capture.
///
/// @code
/// const CameraDepth& c = summary.cameras[i];
/// if (c.plane == DepthVerdict::Ok) offset[i] = c.depth_offset_m;
/// else warn(ids[i], c.depth_offset_m, c.scatter_mm, c.angle_deg);
/// @endcode
struct CameraDepth {
  std::size_t views = 0;  ///< Its views whose plane is valid.
  RatioStats ratio;       ///< Their ratios, pooled.
  /// Its board scale: `Insufficient` for no ratios, else `Off` when the mean
  /// is beyond `max_scale_error` from 1.
  DepthVerdict scale = DepthVerdict::Insufficient;
  /// The median of their offsets along the boards' normals (millimetres);
  /// not a number for none. Not judged: @ref depth_offset_m takes it off.
  double offset_mm = std::numeric_limits<double>::quiet_NaN();
  /// The median of their angles (degrees); not a number for none.
  double angle_deg = std::numeric_limits<double>::quiet_NaN();
  /// How far its depth reads along its axis, too far when positive: the
  /// median of their axial offsets (@ref BoardDepthCheck::axial_offset_mm),
  /// in metres, sensor's `SensorCalibration::depth_offset_m`, which
  /// @ref format_results writes when @ref plane is `Ok`. Not a number for
  /// none.
  double depth_offset_m = std::numeric_limits<double>::quiet_NaN();
  /// How far their axial offsets lie from @ref depth_offset_m: the median of
  /// `|a - depth offset|` over their axial offsets `a` (millimetres); not a
  /// number for fewer than 2.
  double scatter_mm = std::numeric_limits<double>::quiet_NaN();
  /// Its plane against the solved boards once @ref depth_offset_m is taken
  /// off: `Insufficient` for no valid view; else `Off` when the median angle
  /// is beyond `max_angle_deg` or the depth offset is one sensor's
  /// calibration file does not carry (its `validate_array_calibration`
  /// refuses it, as it does one not a number); else `Insufficient` when the
  /// scatter is not a number (one view), and `Off` when it is beyond
  /// `max_scatter_mm`.
  DepthVerdict plane = DepthVerdict::Insufficient;
  /// Its depth's: @ref camera_verdict of @ref scale and @ref plane.
  DepthVerdict verdict = DepthVerdict::Insufficient;
};

/// @brief One ordered pair's agreement and its verdict.
///
/// @code
/// for (const PairCheck& p : summary.pairs) {
///   const PairAgreement& a = p.agreement;
///   if (p.verdict == DepthVerdict::Off) warn(a.from, a.to);
/// }
/// @endcode
struct PairCheck {
  PairAgreement agreement;  ///< As measured.
  /// `Insufficient` when its overlap is below `min_overlap`; else `Off` when
  /// fewer than half its samples are in the window (the median magnitude
  /// over every sample is beyond it), its signed median is beyond
  /// `max_signed_median_mm` or its median magnitude beyond
  /// `max_median_abs_mm`.
  DepthVerdict verdict = DepthVerdict::Insufficient;
};

/// @brief Every ordered pair's agreement in one capture.
///
/// @code
/// for (const CapturePairs& c : summary.captures) show(c.capture, c.pairs);
/// @endcode
struct CapturePairs {
  std::int64_t capture = 0;  ///< The capture, as `Observation::frame_id`.
  /// Every ordered pair, as @ref DepthSummary::pairs.
  std::vector<PairCheck> pairs;
};

/// @brief The depth checks over a Calibrate's captures, with their verdicts.
///
/// @code
/// VKC_ASSIGN(const DepthSummary s,
///            summarize_depth_checks(cameras.size(), checks, pairs, options));
/// log(to_string(s.scale), s.ratio.mean, s.ratio.sd);
/// @endcode
struct DepthSummary {
  DepthCheckOptions options;  ///< The settings the verdicts took.
  /// Every view checked, by capture, then camera.
  std::vector<BoardDepthCheck> views;
  std::vector<CameraDepth> cameras;  ///< Each camera's, in the rig's order.
  RatioStats ratio;  ///< Every camera's ratios, pooled: the board's scale.
  /// Its verdict, by @ref CameraDepth::scale's rule.
  DepthVerdict scale = DepthVerdict::Insufficient;
  /// Every ordered pair, pooled over every capture: `cameras (cameras - 1)`
  /// of them, by `from`, then `to`.
  std::vector<PairCheck> pairs;
  /// Each capture with a pair measured on its own, in ascending order.
  std::vector<CapturePairs> captures;
};

/// @brief Each camera's and the board's verdicts over every capture, and
///        every ordered pair's.
///
/// A pair not given (a camera with no frame, say) is listed with an overlap
/// of 0, so insufficient.
/// @param cameras  The rig's cameras; at least 1.
/// @param views    Each view's check (@ref board_depth_check); at most one
///                 per camera and capture.
/// @param pairs    Each ordered pair's agreement, pooled (no capture) or
///                 one capture's; at most one per pair and capture.
/// @param options  The settings, as @ref board_region took them.
/// @return The summary; or `InvalidArgument` for no cameras, an option out
///         of range (naming it), or a view or pair naming a camera the rig
///         does not have, repeated, a pair from a camera to itself or with
///         more samples in the window than overlap; or `OutOfMemory`. Each
///         message starts `summarize_depth_checks: `.
VC_QUALITY_API core::Result<DepthSummary> summarize_depth_checks(
    std::size_t cameras, const std::vector<BoardDepthCheck>& views,
    const std::vector<PairAgreement>& pairs,
    const DepthCheckOptions& options = {});

}  // namespace volumetric_kit::calib::quality
