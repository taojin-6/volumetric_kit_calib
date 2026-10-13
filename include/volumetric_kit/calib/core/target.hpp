// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file target.hpp
/// @brief Calibration target (board) description, its check, and the rig's
///        board.
///
/// Carries the geometry the detector and solver need, plus the physical
/// tolerances the quality tier propagates: the measured square size and its
/// tolerance scale every metric extrinsic, and board non-planarity (bow) leaks
/// straight into the intrinsics (see DESIGN.md "Target fidelity").

#include <optional>

#include "volumetric_kit/calib/core/export.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib {

/// @brief The pattern family of a calibration target.
enum class TargetKind {
  Checkerboard,          ///< Symmetric checkerboard; best corner accuracy,
                         ///< full board must be visible, 180-degree
                         ///< orientation ambiguity.
  ChArUco,               ///< Checkerboard + ArUco markers; robust to
                         ///< occlusion/partial views, unambiguous orientation.
  SymmetricCircleGrid,   ///< Circles on a square grid; blur-robust, but the
                         ///< projected centroid is perspective/distortion-
                         ///< biased.
  AsymmetricCircleGrid,  ///< Circles with alternate rows offset by half a
                         ///< pitch (OpenCV's asymmetric grid); same bias.
  AprilGrid,             ///< Grid of AprilTags.
  Screen,                ///< Pattern shown on a calibrated display: flatness
                         ///< solved, geometry known to the pixel pitch.
};

/// @brief The dictionary a board's markers come from: one of OpenCV's
///        predefined ArUco and AprilTag dictionaries.
///
/// A marker is decoded only against the dictionary it was printed from, so a
/// board with markers names its own. The ArUco ones are named by their bits and
/// code count, as OpenCV's `DICT_4X4_100` is `ArUco4x4_100`. The values are not
/// OpenCV's `PredefinedDictionaryType` (`None` comes first), so a bridge maps
/// them by name, never by value.
enum class MarkerDictionary {
  None,           ///< No markers: a pattern that has none.
  ArUco4x4_50,    ///< ArUco, 4x4 bits, 50 codes.
  ArUco4x4_100,   ///< ArUco, 4x4 bits, 100 codes.
  ArUco4x4_250,   ///< ArUco, 4x4 bits, 250 codes.
  ArUco4x4_1000,  ///< ArUco, 4x4 bits, 1000 codes.
  ArUco5x5_50,    ///< ArUco, 5x5 bits, 50 codes.
  ArUco5x5_100,   ///< ArUco, 5x5 bits, 100 codes.
  ArUco5x5_250,   ///< ArUco, 5x5 bits, 250 codes.
  ArUco5x5_1000,  ///< ArUco, 5x5 bits, 1000 codes.
  ArUco6x6_50,    ///< ArUco, 6x6 bits, 50 codes.
  ArUco6x6_100,   ///< ArUco, 6x6 bits, 100 codes.
  ArUco6x6_250,   ///< ArUco, 6x6 bits, 250 codes.
  ArUco6x6_1000,  ///< ArUco, 6x6 bits, 1000 codes.
  ArUco7x7_50,    ///< ArUco, 7x7 bits, 50 codes.
  ArUco7x7_100,   ///< ArUco, 7x7 bits, 100 codes.
  ArUco7x7_250,   ///< ArUco, 7x7 bits, 250 codes.
  ArUco7x7_1000,  ///< ArUco, 7x7 bits, 1000 codes.
  ArUcoOriginal,  ///< The original ArUco library's, 1024 codes.
  AprilTag16h5,   ///< AprilTag 16h5, 30 codes.
  AprilTag25h9,   ///< AprilTag 25h9, 35 codes.
  AprilTag36h10,  ///< AprilTag 36h10, 2320 codes.
  AprilTag36h11,  ///< AprilTag 36h11, 587 codes.
  ArUcoMip36h12,  ///< ArUco MIP 36h12, 250 codes.
};

/// @brief A calibration board: pattern, layout, markers, and physical
///        tolerances.
///
/// `rows` and `cols` count the grid of features the detector reports, which is
/// what `DetectedPoint::id` indexes: inner corners for a checkerboard or
/// ChArUco board, circles for a circle grid, tags for an AprilGrid. OpenCV's
/// `CharucoBoard`, and the legacy stereo and bundle-adjustment scripts, count
/// squares instead, one more on each axis; `findChessboardCorners` and the
/// legacy intrinsics script count inner corners, as here.
///
/// @code
/// // The rig's 5 x 3-square ChArUco board: 4 x 2 inner corners.
/// Target board = legacy_femto_board();
/// board.square_size_m = 0.1505;    // measured with calipers
/// board.planarity_bow_m = 0.0005;  // measured; tolerance left unknown
/// VKC_TRY(check_target(board));
/// @endcode
struct Target {
  TargetKind kind = TargetKind::ChArUco;
  int rows = 0;  ///< Feature rows (see above; inner corners for ChArUco).
  int cols = 0;  ///< Feature columns (see above; inner corners for ChArUco).
  double square_size_m = 0.0;  ///< Measured square/spacing size (metres).
  /// ArUco/AprilTag marker size (metres); 0 for a pattern without markers.
  double marker_size_m = 0.0;
  /// The dictionary of a ChArUco board's or AprilGrid's markers; `None` for a
  /// pattern without markers.
  MarkerDictionary dictionary = MarkerDictionary::None;
  /// The id of the board's first marker, from which the rest are numbered in
  /// OpenCV's order; 0 for a pattern without markers.
  int first_marker_id = 0;
  /// OpenCV's legacy ChArUco layout (`CharucoBoard::setLegacyPattern`), which
  /// boards printed before OpenCV 4.6 use. It differs only with an even number
  /// of square rows, where it starts the top-left square white. ChArUco only.
  bool legacy_pattern = false;
  /// 1-sigma tolerance on @ref square_size_m (metres) -- print/thermal error;
  /// propagated to rig-scale uncertainty. Empty when unknown.
  std::optional<double> square_size_tolerance_m;
  /// Peak out-of-plane deviation of the physical board (metres); 0 when
  /// measured flat, empty when not measured.
  std::optional<double> planarity_bow_m;
};

/// @brief Whether @p target describes a board a detector can use.
///
/// Every board needs a @ref TargetKind, at least one feature row and column, a
/// finite positive square size, and known tolerances finite and not negative.
/// A ChArUco board or AprilGrid needs a @ref MarkerDictionary, a marker size
/// between 0 and the square size, and marker ids, from
/// @ref Target::first_marker_id on, that the dictionary holds: half the
/// squares, rounded down, for a ChArUco board, and every tag of an AprilGrid.
/// Any other pattern takes no dictionary, marker size or first id. Only a
/// ChArUco board takes @ref Target::legacy_pattern.
/// @param target  The board.
/// @return OK; or `Status::Code::InvalidArgument` naming the field.
VC_CORE_API core::Status check_target(const Target& target);

/// @brief The name calib's files write for a board's pattern.
/// @param kind  The pattern.
/// @return A stable lowercase name for @p kind (`"charuco"`,
///         `"checkerboard"`, ...); `"unknown"` for a value that is not a
///         @ref TargetKind.
VC_CORE_API const char* to_string(TargetKind kind) noexcept;

/// @brief The name calib's files write for a marker dictionary: OpenCV's.
/// @param dictionary  The dictionary.
/// @return OpenCV's name for @p dictionary (`"DICT_4X4_100"`,
///         `"DICT_APRILTAG_36h11"`, ...); `"none"` for
///         @ref MarkerDictionary::None; `"unknown"` for a value that is not a
///         @ref MarkerDictionary.
VC_CORE_API const char* to_string(MarkerDictionary dictionary) noexcept;

/// @brief The rig's board, as the legacy calibration scripts print it: ChArUco,
///        5 x 3 squares (4 x 2 inner corners) of 0.15 m, with 0.112 m markers
///        from `DICT_4X4_100` numbered from 0, in the legacy layout.
///
/// The square size is the nominal one, and the tolerances are unknown.
/// @return The board, which passes @ref check_target.
VC_CORE_API Target legacy_femto_board() noexcept;

}  // namespace volumetric_kit::calib
