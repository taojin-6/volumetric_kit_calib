// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "common.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <utility>

namespace volumetric_kit::calib::solve::detail {

core::Result<RigCameras> read_rig_cameras(const std::vector<RigCamera>& cameras,
                                          const std::string& prefix) {
  const std::size_t k = cameras.size();
  if (k == 0) {
    return core::Status::invalid_argument(prefix + "the rig has no cameras");
  }
  RigCameras out;
  out.ids.reserve(k);
  out.lenses.reserve(k);
  for (std::size_t c = 0; c < k; ++c) {
    const std::string& id = cameras[c].id;
    if (id.empty()) {
      return core::Status::invalid_argument(prefix + "camera " +
                                            std::to_string(c) + " has no id");
    }
    if (!out.by_id.emplace(id, c).second) {
      return core::Status::invalid_argument(prefix + "camera id \"" + id +
                                            "\" appears twice");
    }
    core::Result<compute::CvCamera> lens = compute::cv_camera(cameras[c].model);
    if (!lens.ok()) return lens.status().with_context(prefix + "camera " + id);
    out.ids.push_back(id);
    out.lenses.push_back(*std::move(lens));
  }
  out.rank.resize(k);
  out.by_rank.reserve(k);
  for (const auto& entry : out.by_id) {
    out.rank[entry.second] = out.by_rank.size();
    out.by_rank.push_back(entry.second);
  }
  return out;
}

core::Status check_one_view_each(const std::vector<Observation>& views,
                                 const std::string& prefix) {
  std::set<std::pair<std::int64_t, std::string>> seen;
  for (const Observation& view : views) {
    if (!seen.emplace(view.frame_id, view.camera_id).second) {
      return core::Status::invalid_argument(
          prefix + "camera " + view.camera_id + " has two views of capture " +
          std::to_string(view.frame_id));
    }
  }
  return {};
}

core::Status refuse_option(std::string_view prefix, const char* name,
                           const char* rule) {
  return core::Status::invalid_argument(std::string(prefix) + name +
                                        " must be " + rule);
}

double median(std::vector<double> values) {
  const std::size_t n = values.size();
  std::sort(values.begin(), values.end());
  return n % 2 == 1 ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

double outlier_bound(std::vector<double> components, double outlier_px,
                     double outlier_mads) {
  // 1.4826 times the median absolute deviation estimates a normal
  // distribution's standard deviation.
  constexpr double kMadToSigma = 1.4826;
  const double centre = median(components);
  for (double& c : components) c = std::abs(c - centre);
  return std::max(outlier_px,
                  outlier_mads * kMadToSigma * median(std::move(components)));
}

}  // namespace volumetric_kit::calib::solve::detail
