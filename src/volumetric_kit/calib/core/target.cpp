// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/core/target.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace volumetric_kit::calib {
namespace {

// How many codes a dictionary holds, as OpenCV's predefined one does
// (board_test checks each against OpenCV); -1 for a value that is not a
// MarkerDictionary.
std::int64_t dictionary_size(MarkerDictionary dictionary) {
  switch (dictionary) {
    case MarkerDictionary::None:
      return 0;
    case MarkerDictionary::ArUco4x4_50:
    case MarkerDictionary::ArUco5x5_50:
    case MarkerDictionary::ArUco6x6_50:
    case MarkerDictionary::ArUco7x7_50:
      return 50;
    case MarkerDictionary::ArUco4x4_100:
    case MarkerDictionary::ArUco5x5_100:
    case MarkerDictionary::ArUco6x6_100:
    case MarkerDictionary::ArUco7x7_100:
      return 100;
    case MarkerDictionary::ArUco4x4_250:
    case MarkerDictionary::ArUco5x5_250:
    case MarkerDictionary::ArUco6x6_250:
    case MarkerDictionary::ArUco7x7_250:
    case MarkerDictionary::ArUcoMip36h12:
      return 250;
    case MarkerDictionary::ArUco4x4_1000:
    case MarkerDictionary::ArUco5x5_1000:
    case MarkerDictionary::ArUco6x6_1000:
    case MarkerDictionary::ArUco7x7_1000:
      return 1000;
    case MarkerDictionary::ArUcoOriginal:
      return 1024;
    case MarkerDictionary::AprilTag16h5:
      return 30;
    case MarkerDictionary::AprilTag25h9:
      return 35;
    case MarkerDictionary::AprilTag36h10:
      return 2320;
    case MarkerDictionary::AprilTag36h11:
      return 587;
  }
  return -1;
}

// Whether a pattern carries markers; empty for a value that is not a
// TargetKind.
std::optional<bool> carries_markers(TargetKind kind) {
  switch (kind) {
    case TargetKind::ChArUco:
    case TargetKind::AprilGrid:
      return true;
    case TargetKind::Checkerboard:
    case TargetKind::SymmetricCircleGrid:
    case TargetKind::AsymmetricCircleGrid:
    case TargetKind::Screen:
      return false;
  }
  return std::nullopt;
}

core::Status invalid(const std::string& field, const std::string& what) {
  return core::Status::invalid_argument("target." + field + " " + what);
}

core::Status check_tolerance(const std::optional<double>& value,
                             const char* field) {
  if (value && !(std::isfinite(*value) && *value >= 0.0)) {
    return invalid(field, "must be finite and not negative");
  }
  return {};
}

}  // namespace

core::Status check_target(const Target& target) {
  const std::optional<bool> has_markers = carries_markers(target.kind);
  if (!has_markers) return invalid("kind", "is not a TargetKind");
  if (target.rows < 1) return invalid("rows", "must be at least 1");
  if (target.cols < 1) return invalid("cols", "must be at least 1");
  if (!(std::isfinite(target.square_size_m) && target.square_size_m > 0.0)) {
    return invalid("square_size_m", "must be finite and positive");
  }
  VKC_TRY(check_tolerance(target.square_size_tolerance_m,
                          "square_size_tolerance_m"));
  VKC_TRY(check_tolerance(target.planarity_bow_m, "planarity_bow_m"));
  if (target.legacy_pattern && target.kind != TargetKind::ChArUco) {
    return invalid("legacy_pattern", "applies to a ChArUco board only");
  }

  if (!*has_markers) {
    constexpr const char* kUnset =
        "must be unset for a pattern without markers";
    if (target.dictionary != MarkerDictionary::None) {
      return invalid("dictionary", kUnset);
    }
    if (target.marker_size_m != 0.0) return invalid("marker_size_m", kUnset);
    if (target.first_marker_id != 0) return invalid("first_marker_id", kUnset);
    return {};
  }

  if (target.dictionary == MarkerDictionary::None) {
    return invalid("dictionary", "must name the markers' dictionary");
  }
  const std::int64_t codes = dictionary_size(target.dictionary);
  if (codes < 0) return invalid("dictionary", "is not a MarkerDictionary");
  if (!(std::isfinite(target.marker_size_m) && target.marker_size_m > 0.0 &&
        target.marker_size_m < target.square_size_m)) {
    return invalid("marker_size_m",
                   "must be finite, positive and below square_size_m");
  }
  if (target.first_marker_id < 0) {
    return invalid("first_marker_id", "must not be negative");
  }
  const std::int64_t markers = marker_count(target);
  if (target.first_marker_id + markers > codes) {
    return invalid("first_marker_id", std::to_string(target.first_marker_id) +
                                          " puts the board's " +
                                          std::to_string(markers) +
                                          " markers past the dictionary's " +
                                          std::to_string(codes) + " codes");
  }
  return {};
}

std::int64_t marker_count(const Target& target) noexcept {
  switch (target.kind) {
    case TargetKind::ChArUco:
      // One in every other square, from the second.
      return (std::int64_t{target.rows} + 1) * (std::int64_t{target.cols} + 1) /
             2;
    case TargetKind::AprilGrid:
      return std::int64_t{target.rows} * target.cols;
    case TargetKind::Checkerboard:
    case TargetKind::SymmetricCircleGrid:
    case TargetKind::AsymmetricCircleGrid:
    case TargetKind::Screen:
      return 0;
  }
  return 0;
}

const char* to_string(TargetKind kind) noexcept {
  switch (kind) {
    case TargetKind::Checkerboard:
      return "checkerboard";
    case TargetKind::ChArUco:
      return "charuco";
    case TargetKind::SymmetricCircleGrid:
      return "symmetric_circle_grid";
    case TargetKind::AsymmetricCircleGrid:
      return "asymmetric_circle_grid";
    case TargetKind::AprilGrid:
      return "april_grid";
    case TargetKind::Screen:
      return "screen";
  }
  return "unknown";
}

const char* to_string(MarkerDictionary dictionary) noexcept {
  switch (dictionary) {
    case MarkerDictionary::None:
      return "none";
    case MarkerDictionary::ArUco4x4_50:
      return "DICT_4X4_50";
    case MarkerDictionary::ArUco4x4_100:
      return "DICT_4X4_100";
    case MarkerDictionary::ArUco4x4_250:
      return "DICT_4X4_250";
    case MarkerDictionary::ArUco4x4_1000:
      return "DICT_4X4_1000";
    case MarkerDictionary::ArUco5x5_50:
      return "DICT_5X5_50";
    case MarkerDictionary::ArUco5x5_100:
      return "DICT_5X5_100";
    case MarkerDictionary::ArUco5x5_250:
      return "DICT_5X5_250";
    case MarkerDictionary::ArUco5x5_1000:
      return "DICT_5X5_1000";
    case MarkerDictionary::ArUco6x6_50:
      return "DICT_6X6_50";
    case MarkerDictionary::ArUco6x6_100:
      return "DICT_6X6_100";
    case MarkerDictionary::ArUco6x6_250:
      return "DICT_6X6_250";
    case MarkerDictionary::ArUco6x6_1000:
      return "DICT_6X6_1000";
    case MarkerDictionary::ArUco7x7_50:
      return "DICT_7X7_50";
    case MarkerDictionary::ArUco7x7_100:
      return "DICT_7X7_100";
    case MarkerDictionary::ArUco7x7_250:
      return "DICT_7X7_250";
    case MarkerDictionary::ArUco7x7_1000:
      return "DICT_7X7_1000";
    case MarkerDictionary::ArUcoOriginal:
      return "DICT_ARUCO_ORIGINAL";
    case MarkerDictionary::AprilTag16h5:
      return "DICT_APRILTAG_16h5";
    case MarkerDictionary::AprilTag25h9:
      return "DICT_APRILTAG_25h9";
    case MarkerDictionary::AprilTag36h10:
      return "DICT_APRILTAG_36h10";
    case MarkerDictionary::AprilTag36h11:
      return "DICT_APRILTAG_36h11";
    case MarkerDictionary::ArUcoMip36h12:
      return "DICT_ARUCO_MIP_36h12";
  }
  return "unknown";
}

Target legacy_femto_board() noexcept {
  Target board;
  board.kind = TargetKind::ChArUco;
  board.rows = 2;
  board.cols = 4;
  board.square_size_m = 0.15;
  board.marker_size_m = 0.112;
  board.dictionary = MarkerDictionary::ArUco4x4_100;
  board.first_marker_id = 0;
  board.legacy_pattern = true;
  return board;
}

}  // namespace volumetric_kit::calib
