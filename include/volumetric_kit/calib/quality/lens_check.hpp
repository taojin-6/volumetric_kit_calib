// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file lens_check.hpp
/// @brief Whether a lens holds on a camera's residuals: their RMS, the share
///        of corners dropped and the 0.1 px cell floor the lens gates'
///        (`quality/lens_gate.hpp`), each cell's mean held to Student's t. On
///        views of any board, each posed through the lens held fixed
///        (@ref check_lens), or on a solve's residuals
///        (@ref check_lens_residuals, @ref check_rig_lens).
///
/// Which to use: a lens session's checkerboard views, @ref check_lens; a pose
/// session, @ref check_rig_lens on its rig solve; a solve's residuals read
/// back, @ref check_lens_residuals. A lens's own fit is @ref judge_lens's to
/// gate. A view's own pose takes up any error of the lens that is close to
/// affine over the board, so on the rig's 5 x 3 ChArUco board @ref check_lens
/// shows only a lens off by pixels; the rig solve shares each capture's board
/// pose between the cameras, so its residuals keep most of what a view's own
/// pose would take up (DECISIONS.md, 2026-11-02, a lens checked on views of
/// any board). A good verdict holds only as far out as the corners reach
/// (@ref LensCheck::reach).

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/quality/export.hpp"
#include "volumetric_kit/calib/quality/lens_gate.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"

namespace volumetric_kit::calib::quality {

/// The cells of the grid the lens checks judge: the lens gates' 8 x 6.
inline constexpr int kLensCells = kCoverageCols * kCoverageRows;

/// @brief The limits the lens checks hold a lens's residuals to, and what
///        @ref check_lens poses: the RMS, the share dropped and the cell floor
///        the lens gates' (@ref LensGateOptions), each cell held to Student's
///        t at `min_cell_corners`.
///
/// @code
/// LensCheckOptions options;
/// options.min_corners = 8;  // whole 5 x 3 ChArUco views alone
/// @endcode
struct LensCheckOptions {
  /// The RMS of the corners kept, at most this at 3840 wide, scaled by width:
  /// the held-out views' limit.
  double max_rms_px = LensGateOptions{}.max_holdout_rms_px;
  /// The corners not kept, as a share, at most; in [0, 1]: the held-out
  /// corners' limit.
  double max_dropped = LensGateOptions{}.max_holdout_dropped;
  /// The corners not kept past `max_dropped` read suspect when they lie in at
  /// least this many views, and insufficient when in fewer; at least 1. One
  /// board dropped, a glitch or a blur, cannot tell the lens from its
  /// detection, and on a 5 x 3 ChArUco board's 8 corners it is past 1% in any
  /// session under 100 captures.
  int min_dropped_views = 2;
  /// In every cell of the 8 x 6 grid with at least `min_cell_corners`
  /// corners, their mean residual at most this, or t standard errors of that
  /// mean if larger (@ref LensCell::limit_px): the bias gate's floor.
  double max_cell_bias_px = LensGateOptions{}.max_cell_bias_px;
  /// At least 2. 6, the fewest whose limit at 0.15 px of corner noise stays
  /// within the RMS limit; noise alone fails such a cell more rarely than the
  /// bias gate's of 30 (DECISIONS.md).
  int min_cell_corners = 6;
  /// @ref check_lens alone: a view with fewer corners is not posed, nor one
  /// left with fewer once those past the bound go; at least 4. A pose takes 6
  /// of a view's numbers, so 6 corners leave 6 to show the lens: a view of
  /// the rig's 5 x 3 ChArUco board (8) keeps its pose with 2 corners lost,
  /// hidden or past the bound.
  int min_corners = 6;
  /// @ref check_lens alone: each view's outlier rounds, as a held-out view's
  /// in @ref solve::solve_lens; at least 0.
  int outlier_rounds = solve::LensOptions{}.outlier_rounds;
};

/// @brief One cell of the 8 x 6 grid: its corners' mean residual, and the
///        limit that mean is held to.
///
/// @code
/// const LensCell& cell = check.cells[row * kCoverageCols + col];
/// if (!std::isnan(cell.limit_px)) draw_arrow(col, row, cell.mean_px);
/// @endcode
struct LensCell {
  int corners = 0;  ///< The corners kept in it.
  /// Their mean residual, projected less detected (pixels); not a number in a
  /// cell of fewer than `min_cell_corners`.
  sensor::camera::Vec2d mean_px{std::numeric_limits<double>::quiet_NaN()};
  /// The limit on its length: `max_cell_bias_px`, or t standard errors of the
  /// mean if larger, the error taken as a vector; t is Student's for the
  /// cell's corners at the two-sided 0.27% of a normal's 3 (4.53 at 8
  /// corners, 3.28 at 30), as the pair row check's. Not a number where the
  /// mean is.
  double limit_px = std::numeric_limits<double>::quiet_NaN();
};

/// @brief A lens checked on a camera's residuals, and what the limits made
///        of it.
///
/// @code
/// VKC_ASSIGN(const LensCheck check,
///            check_rig_lens("bm-1", cameras, solution));
/// if (check.verdict != LensCheck::Verdict::Good) log(check.reason);
/// @endcode
struct LensCheck {
  /// What the check found.
  enum class Verdict : std::uint8_t {
    /// Within every limit, a judged cell in every part of the image: as far
    /// out as the corners reach (`reach`).
    Good,
    /// A judged cell, the RMS or the share of corners dropped past its limit;
    /// or, for @ref check_rig_lens, a camera sharing boards with it so.
    Suspect,
    /// Within every limit, but some part of the image without a judged cell,
    /// or the corners dropped past their share from too few views.
    Insufficient,
  };

  LensCheckOptions options;                 ///< The settings the verdict took.
  std::string camera;                       ///< The camera's id, as given.
  Verdict verdict = Verdict::Insufficient;  ///< The limits'.
  /// Why, in words: each limit passed, with its numbers, or the parts of the
  /// image short of corners (where to capture); empty when good.
  std::string reason;
  /// The RMS of the corners kept (pixels); not a number with none.
  double rms_px = std::numeric_limits<double>::quiet_NaN();
  /// The 95th percentile of their residuals' lengths (`compute::quantile`);
  /// reported, not gated. Not a number with none.
  double p95_px = std::numeric_limits<double>::quiet_NaN();
  /// The views judged: those posed, or the captures with a residual judged.
  std::size_t views = 0;
  /// The corners kept, their residual finite: in a view posed and within the
  /// bound, or not dropped by the solve.
  std::size_t corners = 0;
  /// The corners not kept: past the bound or behind the camera in a view
  /// posed, and every corner of a view left with no pose; or dropped by the
  /// solve, or behind the camera.
  std::size_t dropped = 0;
  /// @ref check_rig_lens alone: the camera's corners left out, kept or not,
  /// of captures in which no other camera kept a corner, their board posed
  /// by this camera alone; 0 otherwise.
  std::size_t unshared = 0;
  /// The corner bound (@ref solve::PosedLensViews::bound_px); not a number
  /// for a solve's residuals, whose own rule dropped its corners.
  double bound_px = std::numeric_limits<double>::quiet_NaN();
  /// How far the corners kept get toward each image corner and side
  /// (@ref solve::pixel_reach); all 0 with none. A good verdict holds this
  /// far out: a lens error beyond does not show. A lens fit's gates ask
  /// @ref solve::kLensReach of its corners; a pose session's corners, in the
  /// pair's shared view, reach less.
  solve::LensReach reach{};
  /// Each cell of the 8 x 6 grid, row-major from the top left.
  std::array<LensCell, kLensCells> cells{};
};

/// @brief A verdict's name, as a report writes it.
/// @param verdict  The verdict.
/// @return `"good"`, `"suspect"` or `"insufficient"`; `"unknown"` for a
///         value @ref LensCheck::Verdict does not name.
///
/// @code
/// std::printf("%s: %s\n", check.camera.c_str(), to_string(check.verdict));
/// @endcode
VC_QUALITY_API const char* to_string(LensCheck::Verdict verdict) noexcept;

/// @brief Whether @p camera_id's lens holds on residuals of its corners, as
///        any solve gives them: each corner's pixel, residual and whether the
///        solve dropped it.
///
/// 1. **Corners.** Those not dropped whose residual is finite are kept; the
///    rest are dropped.
/// 2. **Limits.** The corners kept: their RMS within `max_rms_px`, scaled by
///    width; the corners dropped within `max_dropped` of all, a limit passed
///    when they lie in `min_dropped_views` views or more; and in each cell of
///    the 8 x 6 grid with `min_cell_corners`, the mean residual within its
///    limit (@ref LensCell::limit_px).
/// 3. **Verdict.** Any limit passed: `Suspect`, naming each with its numbers
///    and each cell past its limit by the part of the image it lies in
///    ("top-right"). Else some part of a 3 x 3 split of the image, each cell
///    in the part its centre lies in, without a judged cell, or the corners
///    dropped past `max_dropped` from fewer views: `Insufficient`, naming
///    those parts, where to capture. Else `Good`. A limit passed comes first:
///    no capture elsewhere mends it.
///
/// @param camera_id  The camera, its id for the check.
/// @param size       Its image's size, the residuals' pixels within it.
/// @param residuals  Its corners alone: their @ref solve::CornerResidual's
///                   `capture`, `pixel`, `residual` and `dropped` are read.
/// @param options    The limits; `min_corners` and `outlier_rounds` are
///                   checked, not used.
/// @return The check; `InvalidArgument` for an option out of range (naming
///         it), an empty id, a size with a side 0, or a pixel not finite or
///         outside the image (naming the camera and capture). Each message
///         starts `check_lens_residuals: `.
///
/// @code
/// // bm-5's rows of a pose session's residuals.csv (quality/results.hpp),
/// // read back as solve::CornerResidual.
/// VKC_ASSIGN(const LensCheck check,
///            check_lens_residuals("bm-5", {3840, 2160}, bm5_rows));
/// @endcode
VC_QUALITY_API core::Result<LensCheck> check_lens_residuals(
    std::string_view camera_id, sensor::camera::ImageSize size,
    const std::vector<solve::CornerResidual>& residuals,
    const LensCheckOptions& options = {});

/// @brief Whether @p camera_id's lens holds on a rig solve: its residuals in
///        @p solution, judged as @ref check_lens_residuals judges them, and
///        then beside the cameras it shares boards with.
///
/// 1. **Shared captures.** The solve's cameras share each capture's board
///    pose, so a lens off shows in its residuals where a view's own pose
///    would take it up. Only the camera's corners of captures in which
///    another camera kept a corner are judged; the rest, their board posed
///    by this camera alone, are counted in `unshared`.
/// 2. **Its own residuals,** as @ref check_lens_residuals judges them,
///    `Insufficient` asking for boards another camera sees too.
/// 3. **Boards shared.** Two cameras that both kept corners of a capture
///    share its board. A lens off shows in the residuals of every camera
///    sharing its boards, and not always most in its own: so a camera that
///    shares boards with one whose own residuals read `Suspect` reads
///    `Suspect` too, naming it. Such a verdict says the lenses disagree;
///    @ref check_lens on a lens session says which is off.
///
/// The corners the solve dropped count against `max_dropped`, whatever
/// dropped them (@ref solve::RigSolution::rejections says what).
/// @param camera_id  The camera.
/// @param cameras    The rig's cameras, as the solve took them.
/// @param solution   @ref solve::bundle_adjust's solution for them.
/// @param options    The limits.
/// @return The check; `InvalidArgument` for an id the rig does not have, a
///         solution of another rig, or as @ref check_lens_residuals refuses
///         any camera's residuals judged. Each message starts
///         `check_rig_lens: `.
///
/// @code
/// for (const solve::RigCamera& cam : cameras) {
///   VKC_ASSIGN(const LensCheck check,
///              check_rig_lens(cam.id, cameras, solution));
///   log(cam.id, to_string(check.verdict), check.reason);
/// }
/// @endcode
VC_QUALITY_API core::Result<LensCheck> check_rig_lens(
    std::string_view camera_id, const std::vector<solve::RigCamera>& cameras,
    const solve::RigSolution& solution, const LensCheckOptions& options = {});

/// @brief Whether @p lens holds on @p camera_id's views of any board, posed
///        through it held fixed.
///
/// 1. **Poses.** @ref solve::pose_lens_views: each view of `min_corners` or
///    more that fixes a pose is posed on its own through the lens, as
///    @ref solve::solve_lens poses a held-out view, its corners past the
///    adjustment's bound dropped.
/// 2. **Limits and verdict.** The residuals, as @ref check_lens_residuals
///    judges them.
///
/// @param camera_id  The camera; views of others are ignored.
/// @param lens       The lens checked, at the views' size.
/// @param board      The board seen, either kind: its ids need hold in one
///                   view only.
/// @param views      Any cameras' views, at most one per camera and capture.
/// @param options    What it poses, and the limits.
/// @return The check; `InvalidArgument` for an option out of range (naming
///         it), or what @ref solve::pose_lens_views refuses (an empty id, a
///         lens sensor refuses, two views of one capture, a corner not
///         finite); or what OpenCV threw (@ref compute::guarded). Each
///         message starts `check_lens: `.
///
/// @code
/// VKC_ASSIGN(const LensCheck check,
///            check_lens("bm-1", lens, board, checkerboard_views));
/// @endcode
VC_QUALITY_API core::Result<LensCheck> check_lens(
    std::string_view camera_id, const sensor::camera::CameraModel& lens,
    const compute::Board& board, const std::vector<Observation>& views,
    const LensCheckOptions& options = {});

}  // namespace volumetric_kit::calib::quality
