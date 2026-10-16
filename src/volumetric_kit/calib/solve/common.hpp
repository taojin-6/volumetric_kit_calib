// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// What initialise and bundle_adjust share: private to the tier. Each message
// starts with the caller's `prefix` ("initialise: ", "bundle_adjust: ").

#include <cstddef>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/core/observation.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::calib::solve::detail {

// The rig's cameras: each one's id and lens in OpenCV's types, in the rig's
// order, and the order of their ids.
struct RigCameras {
  std::map<std::string, std::size_t> by_id;
  std::vector<std::string> ids;
  std::vector<compute::CvCamera> lenses;
  std::vector<std::size_t> by_rank;  // the cameras in id order
  std::vector<std::size_t> rank;     // each camera's place in that order
};

// `cameras`, or InvalidArgument for none, an id empty or repeated, or a lens
// compute::cv_camera refuses.
core::Result<RigCameras> read_rig_cameras(const std::vector<RigCamera>& cameras,
                                          const std::string& prefix);

// InvalidArgument when a camera has two views of one capture.
core::Status check_one_view_each(const std::vector<Observation>& views,
                                 const std::string& prefix);

// "<prefix><name> must be <rule>", as InvalidArgument: an options check's
// refusal, `prefix` naming the options ("initialise: options.").
core::Status refuse_option(std::string_view prefix, const char* name,
                           const char* rule);

// The median of `values`, which are not empty.
double median(std::vector<double> values);

}  // namespace volumetric_kit::calib::solve::detail
