// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tag.hpp
/// @brief The origin tag in an 8-bit luma image (luma.hpp): its four corners,
///        when the image holds exactly one print of it.

#include <optional>

#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/detect/export.hpp"
#include "volumetric_kit/calib/detect/luma.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::detect {

/// @brief What @ref detect_tag found of the tag it sought.
///
/// @code
/// VKC_ASSIGN(const MarkerSearch search, detect_tag(luma, kOriginTag));
/// if (search.found) keep(search.found->corners);
/// else if (search.matches > 1) warn("more than one origin tag in view");
/// @endcode
struct MarkerSearch {
  /// The tag, when the image holds exactly one marker with its id.
  std::optional<DetectedMarker> found;
  int matches = 0;  ///< How many markers with the tag's id the image holds.
};

/// @brief How @ref detect_tag refines the tag's corners, as calib's report
///        names it: OpenCV's `CORNER_REFINE_SUBPIX`.
///
/// @code
/// report.key("refinement").value(kTagCornerRefinement);
/// @endcode
inline constexpr const char* kTagCornerRefinement = "subpix";

/// @brief Find @p tag in @p image.
///
/// OpenCV's `ArucoDetector` on the tag's dictionary, its default parameters
/// but for the corner refinement (@ref kTagCornerRefinement), with no camera
/// model. Two prints of the tag give no corners, so neither is taken for the
/// other. No tag is not a failure: @ref MarkerSearch::found is then empty.
/// Calls may run on several threads at once.
///
/// OpenCV's image of an AprilTag is AprilTag's own (apriltag-imgs) turned
/// 180 degrees, so on a print of AprilTag's image the first corner is the
/// printed bottom-right.
/// @param image  The luma image, at least 1 x 1.
/// @param tag    The tag sought, as @ref check_origin_tag accepts it.
/// @return The search: the tag's corners in the image's pixels, in
///         @ref DetectedMarker's order, and how many markers with its id
///         were found. Or `InvalidArgument` for a @p tag
///         @ref check_origin_tag refuses, an image with no pixels, a null
///         `data`, a `stride` below `width`, or a side longer than OpenCV's
///         `int` holds; or what OpenCV threw (`compute::guarded`).
///
/// @code
/// VKC_ASSIGN(const MarkerSearch search, detect_tag(luma, kOriginTag));
/// if (search.found) {
///   const recon::camera::Vec2d first = search.found->corners[0];
/// }
/// @endcode
VC_DETECT_API core::Result<MarkerSearch> detect_tag(const LumaView& image,
                                                    const OriginTag& tag);

}  // namespace volumetric_kit::calib::detect
