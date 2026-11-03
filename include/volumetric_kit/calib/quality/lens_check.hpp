// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file lens_check.hpp
/// @brief Whether a lens holds on views of any board: each view posed through
///        the lens held fixed, and the residuals held to the lens gates'
///        hold-out and bias limits (`quality/lens_gate.hpp`). So a pose
///        session's ChArUco views, treated as a checkerboard's, say whether
///        the lens it uses is good.
///
/// A view's pose takes 6 of its numbers and the rest show the lens: 8 corners,
/// the rig's 5 x 3 ChArUco board's, are enough to check a lens, never to fit
/// one. A pose also takes up any error of the lens that is close to affine
/// over the board, so a small board shows only a lens well off
/// (DECISIONS.md, 2026-11-02, a lens checked on views of any board).

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
#include "volumetric_kit/calib/solve/lens.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"

namespace volumetric_kit::calib::quality {

/// The cells of the grid @ref check_lens judges: the lens gates' 8 x 6.
inline constexpr int kLensCells = kCoverageCols * kCoverageRows;

/// @brief What @ref check_lens poses, and the limits it holds a lens to: the
///        lens gates' own, as @ref LensGateOptions sets them.
///
/// @code
/// LensCheckOptions options;
/// options.min_corners = 8;  // whole 5 x 3 ChArUco views alone
/// @endcode
struct LensCheckOptions {
  /// The views' RMS through the lens, at most this at 3840 wide, scaled by
  /// width: the held-out views' limit.
  double max_rms_px = LensGateOptions{}.max_holdout_rms_px;
  /// The corners past the bound, or in a view left with no pose, as a share,
  /// at most; in [0, 1]: the held-out corners' limit.
  double max_dropped = LensGateOptions{}.max_holdout_dropped;
  /// In every cell of the 8 x 6 grid with at least `min_cell_corners`
  /// corners, their mean residual at most this, or 3 standard errors of that
  /// mean if larger: the bias gate's.
  double max_cell_bias_px = LensGateOptions{}.max_cell_bias_px;
  int min_cell_corners = LensGateOptions{}.min_cell_corners;  ///< At least 2.
  /// A view with fewer corners is not posed, nor one left with fewer once
  /// those past the bound go; at least 4. A pose takes 6 of a view's numbers,
  /// so 6 corners leave 6 to show the lens: a view of the rig's 5 x 3 ChArUco
  /// board (8) keeps its pose with 2 corners lost, hidden or past the bound.
  int min_corners = 6;
  /// Each view's outlier rounds, as a held-out view's in
  /// @ref solve::solve_lens; at least 0.
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
  /// The limit on its length: `max_cell_bias_px`, or 3 standard errors of the
  /// mean if larger, the error taken as a vector; not a number where the mean
  /// is.
  double limit_px = std::numeric_limits<double>::quiet_NaN();
};

/// @brief A lens checked on a camera's views, and what the limits made of
///        it.
///
/// @code
/// VKC_ASSIGN(const LensCheck check,
///            check_lens("bm-1", lens, board, charuco_views));
/// if (check.verdict != LensCheck::Verdict::Good) log(check.reason);
/// @endcode
struct LensCheck {
  /// What the check found.
  enum class Verdict : std::uint8_t {
    Good,  ///< Within every limit, a judged cell in every part of the image.
    /// A judged cell, the RMS or the share of corners dropped past its limit.
    Suspect,
    /// Within every limit, but some part of the image without a judged cell.
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
  std::size_t views = 0;  ///< The views posed.
  /// The corners kept: in a view posed, within the bound, their residual
  /// finite.
  std::size_t corners = 0;
  /// The corners not kept: past the bound or behind the camera in a view
  /// posed, and every corner of a view left with no pose.
  std::size_t dropped = 0;
  /// The corner bound (@ref solve::PosedLensViews::bound_px).
  double bound_px = 0.0;
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

/// @brief Whether @p lens holds on @p camera_id's views of any board, posed
///        through it held fixed.
///
/// 1. **Poses.** @ref solve::pose_lens_views: each view of `min_corners` or
///    more that fixes a pose is posed on its own through the lens, as
///    @ref solve::solve_lens poses a held-out view, its corners past the
///    adjustment's bound dropped.
/// 2. **Limits.** The corners kept are held to the lens gates' limits
///    (@ref judge_lens): their RMS within `max_rms_px`, scaled by width; the
///    corners dropped within `max_dropped` of all; and in each cell of the
///    8 x 6 grid with `min_cell_corners`, the mean residual within its limit
///    (@ref LensCell::limit_px), as the bias gate holds a fit's.
/// 3. **Verdict.** Any limit passed: `Suspect`, naming each with its numbers
///    and each cell past its limit by the part of the image it lies in
///    ("top-right"). Else some part of a 3 x 3 split of the image, each cell
///    in the part its centre lies in, without a judged cell: `Insufficient`,
///    naming those parts, where to capture. Else `Good`. A limit passed comes
///    first: no capture elsewhere mends it.
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
VC_QUALITY_API core::Result<LensCheck> check_lens(
    std::string_view camera_id, const sensor::camera::CameraModel& lens,
    const compute::Board& board, const std::vector<Observation>& views,
    const LensCheckOptions& options = {});

}  // namespace volumetric_kit::calib::quality
