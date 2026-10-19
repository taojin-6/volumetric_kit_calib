// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/detect/tag.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include "detail/luma_check.hpp"
#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::detect {

core::Result<MarkerSearch> detect_tag(const LumaView& image,
                                      const OriginTag& tag) {
  return compute::guarded("detect_tag", [&]() -> core::Result<MarkerSearch> {
    VKC_TRY(check_origin_tag(tag));
    VKC_TRY(detail::check_luma(image, "detect_tag"));
    const std::optional<cv::aruco::PredefinedDictionaryType> dictionary =
        compute::opencv_dictionary(tag.dictionary);
    // check_origin_tag refuses a tag without a dictionary.
    if (!dictionary) {
      return core::Status::invalid_argument(
          "origin_tag.dictionary must name the tag's dictionary");
    }
    cv::aruco::DetectorParameters parameters;
    parameters.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    const cv::aruco::ArucoDetector detector(
        cv::aruco::getPredefinedDictionary(*dictionary), parameters);
    // OpenCV reads the image and never writes it.
    const cv::Mat luma(static_cast<int>(image.height),
                       static_cast<int>(image.width), CV_8UC1,
                       const_cast<std::uint8_t*>(image.data), image.stride);
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    detector.detectMarkers(luma, corners, ids);

    MarkerSearch search;
    std::size_t found = 0;
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] != tag.id) continue;
      ++search.matches;
      found = i;
    }
    if (search.matches != 1) return search;
    DetectedMarker marker;
    marker.id = tag.id;
    for (std::size_t k = 0; k < marker.corners.size(); ++k) {
      marker.corners[k] = {corners[found][k].x, corners[found][k].y};
    }
    search.found = marker;
    return search;
  });
}

}  // namespace volumetric_kit::calib::detect
