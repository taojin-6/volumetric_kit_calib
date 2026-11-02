// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// What the board detectors share (charuco.cpp, board.cpp): a pixel of a
// picture halved, in the picture; corners found in the half refined in the
// picture; and the ChArUco search, in the picture or its half.

#include <vector>

#include <opencv2/core.hpp>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/detection.hpp"

namespace volumetric_kit::calib::detect::detail {

// A pixel of the picture halved, in the picture: the centre of the 2 x 2
// pixels averaged into it.
inline cv::Point2f full_resolution(const cv::Point2f& half) {
  return {2.0f * half.x + 0.5f, 2.0f * half.y + 0.5f};
}

// The distance from each corner to the nearest of `points` (a ChArUco
// board's: every marker corner found, the board's or not).
std::vector<double> nearest_of(
    const std::vector<cv::Point2f>& corners,
    const std::vector<std::vector<cv::Point2f>>& points);

// The distance from each corner to the nearest other corner (a
// checkerboard's).
std::vector<double> nearest_other(const std::vector<cv::Point2f>& corners);

// Each corner refined in `image` from its seed with cornerSubPix (at most 100
// iterations, to 0.01 px), its half-window half of `nearest[i]`, the distance
// to the nearest edge that would pull it, rounded down and held to 2-10 px;
// a corner that moves more than 2 px is dropped, with its id.
void refine_corners(const cv::Mat& image, const std::vector<double>& nearest,
                    std::vector<cv::Point2f>& corners, std::vector<int>& ids);

// The ChArUco board (board.charuco(), not null) in `full`: searched in `half`
// when given (`full` halved, each pixel the mean of 2 x 2 of its own), its
// markers reported in `full`'s pixels and its corners refined there; else
// searched in `full`. The detection's corners and markers by id, ascending;
// its image size is the caller's to fill.
Detection find_charuco(const cv::Mat& full, const cv::Mat* half,
                       const compute::Board& board);

}  // namespace volumetric_kit::calib::detect::detail
