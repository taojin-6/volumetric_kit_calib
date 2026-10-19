// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file results.hpp
/// @brief The files a Calibrate press leaves in its results directory, as
///        text: recon's calibration file of the solution, the report and the
///        residuals.
///
/// The brain formats them and writes nothing: the application creates the
/// directory and writes each file with the core's
/// `write_file(WriteMode::CreateNew)`.
///
/// **`report.json`** (`"format": "volumetric_kit_calib.report"`,
/// `"version": 1`), cameras named by id, each figure once:
/// - `settings`: `reference`, `cameras`, and the `initialise`,
///   `bundle_adjust` and `metrics` options the metrics were measured with
///   (@ref CalibrationMetrics::options);
/// - `solver`: `solves`, `iterations`, `initial_energy`, `final_energy`,
///   `variance`, `seconds` (to the microsecond, with six decimals); `tree`:
///   each link's `camera`, `parent`, `shared`, `inliers` and `weak`;
/// - `reprojection`: `initial` and `solved`, each `overall`,
///   `without_oblique`, `legacy_rmse_px`, `histogram` (the counts, in bins of
///   `settings.metrics.histogram_bin_px`), `cameras` (by id), `captures` and
///   `views`, every group `{count, rms_px, p90_px, max_px, mean_px: [x, y]}`;
///   and `legacy_reduction_percent`;
/// - `rejections`: `views_left_out`, each view @ref solve::initialise left
///   out (`camera`, `capture`, `fate`, `use`, `pnp_rms_px`, `ambiguity`,
///   `tilt_deg`); `counts` of the adjustment's, by round and reason; and its
///   `list`, each `camera`, `capture`, `corner` (-1 for a view), `round`,
///   `reason`, `value`, `bound`;
/// - `transfer`: `median_px`, `p90_px`, `worst_px`, `pairs` (`from`, `to`,
///   `count`, `mean_px`, for each pair with a capture) and `errors`
///   (`capture`, `from`, `to`, `error_px`);
/// - `leave_one_out`: `cameras` (by id: `centre_mm`, `centre_capture`,
///   `rotation_deg`, `rotation_capture`) and `captures` (`capture`,
///   `refused`, `rms_px`, `legacy_rmse_px`, and `cameras` by id:
///   `centre_mm`, `rotation_deg`);
/// - `covariance`: by id, `rotation_deg`, `centre_mm` and the 6 x 6
///   `matrix`, rows of (rvec, tvec);
/// - `sensitivity`: by the id whose intrinsics are off (by
///   `settings.bundle_adjust`'s `focal_fraction` and `principal_px`):
///   `focal`, `principal_x` and `principal_y`, each by the id moved,
///   `{rotation_deg, centre_mm: [x, y, z]}`;
/// - `principal_point_ab` (its shift `settings.metrics.principal_shift_px`,
///   its verdict's margin `principal_margin_fraction` and
///   `principal_margin_px`): `refused`, `corners`, `as_given` and `shifted`
///   (`rms_px`, `legacy_rmse_px`, `transfer_median_px`, `transfer_p90_px`),
///   `verdict` (@ref to_string(PrincipalPointVerdict)'s name);
/// - `geometry`: `centres_m` by id, in the reference camera's colour frame,
///   and `baselines_m`, each pair once (`a`, `b`, `m`);
/// - `world` (@ref ResultWorld): `origin`, `"tag"` or `"reference"`;
///   `reference`, the reference camera's id; `uncertainty_frame`,
///   `"reference"`, the frame of the covariance, the sensitivities, the
///   leave-one-out moves and `geometry`, whatever the world;
///   `reference_to_world`, its 4 x 4 rows; `tag`, the tag sought
///   (`dictionary`, `family` as `apriltag_family` names it, empty for none,
///   `id`, `edge_mm`, and `refinement`, its corners' as
///   `detect::kTagCornerRefinement` names it), `null` when none was; `miss`,
///   `miss_camera` and `reason`, why there is no tag world
///   (`solve::to_string(TagMiss)`'s name, the camera it names, its words),
///   each `null` with one; `located`, the
///   tag world (`solve::TagWorld`), `null` without one: `rms_px`, `max_px`,
///   `solves`, `edges_mm`, `mean_edge_mm`, `edge_error_percent`,
///   `edge_warning`, `diagonals_mm`, `out_of_plane_mm`, `corners_m`,
///   `cameras` (by id: `views`, `used`, `dropped`, `height_m`) and `views`
///   (`camera`, `capture`, `used`, `rms_px`, `worst_px`, `round`,
///   `bound_px`); and `floor_boards` (`capture`, `angle_deg`, `height_mm`);
/// - `depth` (@ref DepthSummary), `null` when the depth was not checked:
///   `settings` (its @ref DepthCheckOptions, by their names); `ratio`
///   (`count`, `mean`, `sd`) and its `scale` verdict; `cameras` (by id:
///   `views`, `ratio`, `scale`, `offset_mm`, `angle_deg`, `plane`,
///   `verdict`); `views` (`camera`, `capture`, `valid`, `members`,
///   `out_of_range`, `inliers`, `rms_mm`, `ratios`, `offset_mm`,
///   `angle_deg`); `pairs`, pooled, and `captures` (`capture`, `pairs`), each
///   pair `from`, `to`, `overlap`, `in_window`, `signed_median_mm`,
///   `median_abs_mm`, `p90_abs_mm`, `verdict`. Verdicts are
///   @ref to_string(DepthVerdict)'s names; an invalid plane's `rms_mm` is
///   `null`;
/// - `depth_unchecked`: why `depth` is `null` though the depth was to be
///   checked, such as the device's failure; `null` otherwise.
///
/// Numbers are written in the shortest form that reads back to the same
/// double; one that is not finite, or not measured, is `null`, as are a
/// `centre_capture` or `rotation_capture` when no capture moved the camera,
/// and `refused` when the re-solve ran.
///
/// **`residuals.csv`**: the header
/// `capture,serial,corner_id,u,v,du,dv,weight,dropped`, then every corner
/// the adjustment took, in @ref solve::RigSolution::residuals' order: its
/// detection (u, v), its residual (du, dv; `nan` behind its camera), its
/// weight, and 1 when it was left out, else 0.

#include <string>
#include <vector>

#include "volumetric_kit/calib/quality/calibration_metrics.hpp"
#include "volumetric_kit/calib/quality/depth_checks.hpp"
#include "volumetric_kit/calib/quality/export.hpp"
#include "volumetric_kit/calib/quality/floor_boards.hpp"
#include "volumetric_kit/calib/solve/bundle_adjust.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/calib/solve/world_tag.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::quality {

/// The results directory's calibration file: recon's, posed.
inline constexpr const char* kArrayCalibrationFile = "array_calibration.json";
inline constexpr const char* kReportFile = "report.json";  ///< The report.
/// Every corner's residual.
inline constexpr const char* kResidualsFile = "residuals.csv";

/// @brief The text of each file of a results directory.
///
/// @code
/// VKC_ASSIGN(const ResultFiles files,
///            format_results(cameras, solution, metrics, &depth));
/// VKC_TRY(core::write_file(dir + "/" + kReportFile, files.report,
///                          core::WriteMode::CreateNew));
/// @endcode
struct ResultFiles {
  /// @ref kArrayCalibrationFile: recon's `format_array_calibration` of the
  /// solution's calibration in the world (`solve::anchor_rig`), whose `pose`
  /// is each camera's T_cw there and whose `world` is the tag or the
  /// reference camera.
  std::string array_calibration;
  std::string report;     ///< @ref kReportFile.
  std::string residuals;  ///< @ref kResidualsFile.
};

/// @brief The world a solved rig's results are in, and the boards on its
///        floor.
///
/// @code
/// ResultWorld world;
/// VKC_ASSIGN(world.origin,
///            solve::origin_tag_frame(cameras, solution, views, kOriginTag));
/// @endcode
struct ResultWorld {
  /// Where it is (`solve::origin_tag_frame`); by default the reference
  /// camera, no tag sought.
  solve::WorldOrigin origin;
  /// The boards on its floor (@ref floor_boards); empty without a tag world.
  std::vector<FloorBoard> floor_boards;
};

/// @brief Format a solved rig's results files.
/// @param cameras   The rig's cameras, as the solve took them.
/// @param solution  The adjustment's solution.
/// @param metrics   @ref calibration_metrics of it.
/// @param depth     @ref summarize_depth_checks of its depth; null when the
///                  depth was not checked.
/// @param depth_unchecked  Why the depth was not checked though it was to
///                  be, which the report writes as `depth_unchecked`; empty,
///                  written `null`, when it was checked or not asked for.
///                  Not written with @p depth.
/// @param world     Its world; by default the reference camera's, whose
///                  calibration file is the solution's own.
/// @return The files; `InvalidArgument` for no cameras, a repeated or empty
///         id, or a @p solution, @p metrics, @p depth or tag world that names
///         a camera the rig does not have or has another camera count, or a
///         calibration whose sensors are not the cameras (naming what);
///         what `solve::anchor_rig` refuses; recon's error for a calibration
///         its writer refuses; or `OutOfMemory`. Each message starts
///         `format_results: `.
VC_QUALITY_API core::Result<ResultFiles> format_results(
    const std::vector<solve::RigCamera>& cameras,
    const solve::RigSolution& solution, const CalibrationMetrics& metrics,
    const DepthSummary* depth = nullptr,
    const std::string& depth_unchecked = {}, const ResultWorld& world = {});

}  // namespace volumetric_kit::calib::quality
