// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file pair_row_check.hpp
/// @brief A stereo pair's rectified row error (Δy): how far apart the rows of
///        one board corner land in the pair's two rectified views, cell by
///        cell over the view, through the rectification the runtime uses.
///
/// A stereo matcher searches along rows, so a pair whose rows disagree gives
/// depth that is wrong where they do. The pair is rectified by sensor's own
/// `stereo_calibration` and `rectify_stereo`, the functions its
/// `StereoRectifier` uses, at the runtime's size: so the check measures the
/// runtime as it is. Named for its rows: `quality::PairCheck` is an ordered
/// pair's depth agreement (`quality/depth_checks.hpp`).

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/quality/export.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"

namespace volumetric_kit::calib::quality {

/// The cells of the grid over the rectified view: 3 x 3.
inline constexpr int kPairRowCells = 9;

/// @brief The size @ref check_pair_rows rectifies at, and the gate it holds a
///        pair to.
///
/// @code
/// PairRowCheckOptions options;
/// options.rectified = {960, 544};  // the runtime's, from core's config
/// @endcode
struct PairRowCheckOptions {
  /// The runtime's rectified size: its stereo matcher's, as core's config
  /// resolves it. Required: a side of 0 is refused.
  sensor::camera::ImageSize rectified{0, 0};
  double alpha = 0.0;  ///< As the runtime's rectifier; in [0, 1].
  /// In every cell of a 3 x 3 grid over the rectified view with at least
  /// `min_cell_corners` corners, the mean Δy at most this, or 3 standard
  /// errors of that mean if larger: noise alone cannot fail a right pair.
  double max_cell_mean_px = 0.15;
  std::size_t min_cell_corners = 8;  ///< At least 2.
  /// Cells with `min_cell_corners`, at least, one in each outer column among
  /// them; in [1, 9].
  int min_cells = 7;
};

/// @brief A pair's rows, and what the gate made of them.
///
/// Δy is a corner's row in the left rectified view less its row in the right
/// one, in rectified pixels. A cell is the left view's.
///
/// @code
/// VKC_ASSIGN(const PairRowCheck rows,
///            check_pair_rows(rig.calibration, "bm-1", "bm-2", views,
///                            options));
/// if (rows.verdict != PairRowCheck::Verdict::Pass) log(rows.reason);
/// @endcode
struct PairRowCheck {
  /// What the check found.
  enum class Verdict : std::uint8_t {
    Pass,          ///< Every cell judged within its limit, enough of them.
    Fail,          ///< Some cell judged past its limit.
    Insufficient,  ///< Too few cells judged, or an outer column without one.
    Swapped,       ///< Only the pair named the other way round rectifies.
    Refused,       ///< Neither way rectifies (a vertical pair, say).
  };

  PairRowCheckOptions options;  ///< The settings the verdict took.
  std::string left;             ///< The left camera's id, as given.
  std::string right;            ///< The right camera's id, as given.
  Verdict verdict = Verdict::Insufficient;  ///< The gate's.
  /// Why, in words: the cells past their limits, the cells short of corners
  /// (where to capture), the order to name the pair in, or the
  /// rectification's refusal; empty for a pass.
  std::string reason;
  std::size_t captures = 0;  ///< Captures with a corner counted.
  /// Corners counted: seen by both cameras in one capture, and inside both
  /// rectified views.
  std::size_t corners = 0;
  /// Each cell's corners, row-major from the top left.
  std::array<std::size_t, kPairRowCells> cell_corners{};
  /// Each cell's mean Δy (pixels); not a number in a cell with fewer than
  /// `min_cell_corners`.
  std::array<double, kPairRowCells> cell_mean_px{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
  /// Each judged cell's limit: `max_cell_mean_px`, or 3 standard errors of
  /// its mean if larger; not a number where the mean is.
  std::array<double, kPairRowCells> cell_limit_px = cell_mean_px;
  /// The median of every counted corner's |Δy| (pixels); reported, not
  /// gated. Not a number with none counted.
  double dy_median_px = std::numeric_limits<double>::quiet_NaN();
  /// The 95th percentile of |Δy| (`compute::quantile`); reported, not gated.
  double dy_p95_px = std::numeric_limits<double>::quiet_NaN();
  double baseline_m = 0.0;  ///< The rectified pair's; 0 unless rectified.
};

/// @brief A verdict's name, as the report writes it.
/// @param verdict  The verdict.
/// @return `"pass"`, `"fail"`, `"insufficient"`, `"swapped"` or `"refused"`.
VC_QUALITY_API const char* to_string(PairRowCheck::Verdict verdict) noexcept;

/// @brief A pair's rows: every board corner both cameras saw in one capture,
///        through the pair's rectification, as Δy.
///
/// The pair is sensor's `stereo_calibration(solved, left, right)`, rectified
/// by `rectify_stereo` at `options.rectified` and `options.alpha`. Each corner
/// is undistorted exactly (`compute::undistort_points`, to 1e-10 px:
/// OpenCV's default five iterations stop short of a fisheye's lens), turned by
/// its camera's rectifying rotation (OpenCV's `R1` or `R2`) and projected by
/// the rectified camera. A corner outside either rectified view is not
/// counted.
///
/// **The gate.** Cells of a 3 x 3 grid over the left rectified view with at
/// least `min_cell_corners` corners are judged: each mean Δy within its limit
/// (@ref PairRowCheck::cell_limit_px). Any past it: `Fail`, naming those
/// cells. Else fewer than `min_cells` judged, or an outer column with none:
/// `Insufficient`, naming the cells short of corners, where to capture. Else
/// `Pass`. The median and 95th percentile are reported, not gated: corner
/// noise alone sets them, and a fault that harms the matcher is local.
///
/// **Left and right.** When `rectify_stereo` refuses the pair as named, the
/// pair named the other way round is tried: `Swapped` if that rectifies,
/// else `Refused` with the first refusal. Neither measures a row.
///
/// @param solved   The solved calibration: both cameras posed, with colour
///                 lenses (`solve::anchor_rig`'s, say).
/// @param left     The left camera's id, as the runtime's config names it.
/// @param right    The right camera's id.
/// @param views    Every camera's views of a board whose corner ids name one
///                 corner in every camera (a ChArUco board's, not a
///                 checkerboard's), `frame_id` the capture; other cameras'
///                 are ignored.
/// @param options  The size, and the gate.
/// @return The check; `InvalidArgument` for an option out of range (naming
///         it), what `stereo_calibration` refuses (an id not in @p solved,
///         unposed or without a colour lens, the same id twice), or a camera
///         with two views of one capture; or what OpenCV threw
///         (@ref compute::guarded). Each message starts `check_pair_rows: `.
VC_QUALITY_API core::Result<PairRowCheck> check_pair_rows(
    const sensor::camera::ArrayCalibration& solved, std::string_view left,
    std::string_view right, const std::vector<Observation>& views,
    const PairRowCheckOptions& options);

}  // namespace volumetric_kit::calib::quality
