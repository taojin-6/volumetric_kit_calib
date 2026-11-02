// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file lens_gate.hpp
/// @brief What a solved lens is worth, and the files of a lens-only results
///        directory: sensor's calibration file with lenses and no poses, the
///        report and the residuals.
///
/// A wrong lens can fit its views well, since each view's own pose absorbs
/// most of the error. So beside the RMS the gates look for what no pose can
/// absorb: a pattern left in the residuals over the image, and a lens model
/// that turns back inside the frame.

#include <array>
#include <string>
#include <vector>

#include "volumetric_kit/calib/quality/export.hpp"
#include "volumetric_kit/calib/quality/results.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"

namespace volumetric_kit::calib::quality {

/// @brief The numbers @ref judge_lens holds a fit to.
///
/// @code
/// LensGateOptions options;
/// options.max_holdout_rms_px = 0.4;
/// @endcode
struct LensGateOptions {
  /// The held-out views' RMS at most this at 3840 wide, scaled by width.
  double max_holdout_rms_px = 0.5;
  double max_holdout_ratio = 1.3;  ///< That RMS over the fit's, at most.
  /// In every cell of an 8 x 6 grid over the image with at least
  /// `min_cell_corners` fitted corners, their mean residual at most this, or
  /// 3 standard errors of that mean if larger: noise alone cannot fail a
  /// right lens.
  double max_cell_bias_px = 0.1;
  int min_cell_corners = 30;  ///< At least 2.
  /// Reach (@ref solve::LensReach) of the fitted corners kept toward each
  /// image corner and side, at least; in [0, 1]. Past the corners a fit
  /// reached, nothing holds its curve.
  double min_reach = solve::kLensReach;
  /// The ratio is judged only with the held-out RMS above this at 3840 wide,
  /// scaled by width; at least 0. Below it a fit is down at the detector's
  /// noise, where the ratio of two small numbers says nothing.
  double min_ratio_holdout_rms_px = 0.1;
  /// The held-out corners past the fit's corner bound, as a share, at most;
  /// in [0, 1]. A glitch costs nothing, a wrong lens does not hide.
  double max_holdout_dropped = 0.01;
};

/// @brief A fit and what the gates made of it.
///
/// @code
/// VKC_ASSIGN(const LensVerdict verdict, judge_lens(std::move(fit)));
/// for (const std::string& why : verdict.failed) log(why);
/// @endcode
struct LensVerdict {
  solve::LensFit fit;  ///< What was judged.
  bool pass = false;   ///< No gate missed.
  /// Each gate missed, with its numbers.
  std::vector<std::string> failed;
  double worst_cell_bias_px = 0;  ///< The largest cell's mean residual.
  /// The fitted corner reaching farthest toward each image corner (top-left,
  /// top-right, bottom-left, bottom-right; @ref solve::LensReach): how far
  /// off axis it is through the fit (degrees).
  std::array<double, 4> corner_reach_deg{};
  std::array<double, 2> side_reach_deg{};  ///< And each side (left, right).
  /// Where the lens model turns back or meets a pole of its denominator, off
  /// axis (degrees); 90: never.
  double fold_deg = 90.0;
  bool frame_folds = false;  ///< Some image corner lies past the fold.
  solve::LensReach reach{};  ///< How far the fitted corners kept get.
  /// The column and row of the largest cell of the 8 x 6 grid; -1 with no
  /// cell judged.
  std::array<int, 2> worst_cell{-1, -1};
  int holdout_corners = 0;  ///< The held-out views' corners scored or dropped.
  int holdout_dropped = 0;  ///< Those past the fit's corner bound.
};

/// @brief The gates, from the fit alone: its held-out and fitted residuals
///        and its lens.
///
/// 1. **Hold-out.** `holdout_rms_px` within `max_holdout_rms_px` scaled by
///    width: the views and the detection are good enough. A fit with no view
///    held out misses it.
/// 2. **Hold-out against fit.** `holdout_rms_px / rms_px` within
///    `max_holdout_ratio`: no overfit. Judged only above
///    `min_ratio_holdout_rms_px`, scaled by width.
/// 3. **Held-out outliers.** At most `max_holdout_dropped` of the held-out
///    corners past the fit's corner bound, which both RMS values leave out.
/// 4. **Bias.** The cells' mean residuals, over the fitted corners kept
///    (@ref LensGateOptions::max_cell_bias_px): a wrong minimum, or a curve
///    bent at the edge, leaves a pattern RMS ratios miss.
/// 5. **Fold.** sensor's `fold_radius2` gives the undistorted radius `r` where
///    the radial model turns back, `rho = r N(r^2) / D(r^2)` the largest
///    distorted radius it reaches, and each image corner's normalized distorted
///    radius, `((u - cx) / fx, (v - cy) / fy)`, must lie below `rho`: past it a
///    pixel has no inverse. A pole of `D` bounds no radius, yet a fit's curve
///    goes on past one when `N` nearly cancels it, so each image corner must
///    also read nearer the axis than the fold through the lens.
/// 6. **Reach.** The fitted corners kept reach `min_reach` toward every image
///    corner and side: outlier rounds can take it from a session that was
///    ready, and past the corners nothing holds a curve.
///
/// @param fit      The fit, from @ref solve::solve_lens or made alike.
/// @param options  The numbers.
/// @return The verdict, whose `fit` is @p fit; `InvalidArgument` for an option
///         out of range (naming it), a lens sensor refuses, or a residual of a
///         capture @p fit has no view of; or what OpenCV threw
///         (@ref compute::guarded). Each message starts `judge_lens: `.
VC_QUALITY_API core::Result<LensVerdict> judge_lens(
    solve::LensFit fit, const LensGateOptions& options = {});

/// @brief Format a lens-only results directory's files.
///
/// **`array_calibration.json`**: sensor's file, version 2, with colour lenses
/// alone: each camera of @p lenses, marked calibrated, whatever its verdict, so
/// a trial pose can use it, and those of @p base's other cameras as they are.
/// @p base's poses, world and other fields are dropped: poses solved with old
/// lenses are void. Empty when no lens is left to write: there is no file.
///
/// **`report.json`**: version 2 with its `lenses` section alone
/// (@ref results.hpp). **`residuals.csv`**: every fitted and held-out corner
/// of @p lenses, in order, as `format_results` writes them.
/// @param lenses       The cameras judged.
/// @param unsolved     Cameras not solved, listed in the report.
/// @param base         The file the lenses came from, or null.
/// @param report_only  Cameras of @p lenses fitted for the report and never
///                     written (factory lenses), nor carried from @p base.
/// @return The files; `InvalidArgument` for an id empty or repeated across
///         @p lenses and @p unsolved, or in @p report_only and not @p lenses;
///         sensor's error for a calibration its writer refuses; or
///         `OutOfMemory`. Each message starts `format_lens_results: `.
///
/// @code
/// VKC_ASSIGN(const ResultFiles files,
///            format_lens_results({verdict}, {}, nullptr));
/// VKC_TRY(core::write_file(dir + "/" + kArrayCalibrationFile,
///                          files.array_calibration,
///                          core::WriteMode::CreateNew));
/// @endcode
VC_QUALITY_API core::Result<ResultFiles> format_lens_results(
    const std::vector<LensVerdict>& lenses,
    const std::vector<std::string>& unsolved,
    const sensor::camera::ArrayCalibration* base,
    const std::vector<std::string>& report_only = {});

}  // namespace volumetric_kit::calib::quality
