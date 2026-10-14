// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/solve/initialise.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/trigonometric.hpp>

#include "reprojection.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/solve/view_pose.hpp"

namespace volumetric_kit::calib::solve {
namespace {

namespace camera = recon::camera;

core::Status refuse_option(const char* name, const char* rule) {
  return core::Status::invalid_argument(std::string("initialise: options.") +
                                        name + " must be " + rule);
}

core::Status check_options(const InitialiseOptions& options) {
  if (!(options.max_pnp_rms_px > 0.0) ||
      !std::isfinite(options.max_pnp_rms_px)) {
    return refuse_option("max_pnp_rms_px", "positive and finite");
  }
  if (!(options.min_ambiguity >= 1.0) ||
      !std::isfinite(options.min_ambiguity)) {
    return refuse_option("min_ambiguity", "at least 1 and finite");
  }
  if (!(options.oblique_deg > 0.0 && options.oblique_deg <= 180.0)) {
    return refuse_option("oblique_deg", "in (0, 180]");
  }
  if (options.weak_below < 0) {
    return refuse_option("weak_below", "at least 0");
  }
  if (!(options.inlier_deg > 0.0 && options.inlier_deg < 90.0)) {
    return refuse_option("inlier_deg", "in (0, 90)");
  }
  if (!(options.truncation_px > 0.0) || !std::isfinite(options.truncation_px)) {
    return refuse_option("truncation_px", "positive and finite");
  }
  return {};
}

// The angle between two rotations (degrees), accurate near 0 and 180.
double angle_deg(const camera::Mat3d& a, const camera::Mat3d& b) noexcept {
  return glm::degrees(
      glm::length(camera::rodrigues_from_rotation(glm::transpose(a) * b)));
}

double median(std::vector<double> values) {
  const std::size_t n = values.size();
  std::sort(values.begin(), values.end());
  return n % 2 == 1 ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

// A robust mean of transforms that each estimate one: the rotation with the
// least median angle to the others (the first, on a tie) is the medoid; those
// within `inlier_deg` of it give the chordal mean of their rotations and the
// median of their translations on each axis.
struct RobustMean {
  camera::Mat4d transform{1.0};
  int inliers = 0;
};

RobustMean robust_mean(const std::vector<camera::Mat4d>& candidates,
                       double inlier_deg) {
  const std::size_t n = candidates.size();
  std::vector<camera::Mat3d> rotations;
  rotations.reserve(n);
  for (const camera::Mat4d& m : candidates) rotations.emplace_back(m);

  std::size_t medoid = 0;
  double least = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    std::vector<double> angles(n);
    for (std::size_t j = 0; j < n; ++j) {
      angles[j] = angle_deg(rotations[i], rotations[j]);
    }
    const double m = median(std::move(angles));
    if (m < least) {
      least = m;
      medoid = i;
    }
  }

  camera::Mat3d sum(0.0);
  std::vector<double> x, y, z;
  for (std::size_t i = 0; i < n; ++i) {
    if (!(angle_deg(rotations[i], rotations[medoid]) <= inlier_deg)) continue;
    sum += rotations[i];
    x.push_back(candidates[i][3][0]);
    y.push_back(candidates[i][3][1]);
    z.push_back(candidates[i][3][2]);
  }
  RobustMean mean;
  mean.inliers = static_cast<int>(x.size());
  // Each rotation is within inlier_deg (below 90) of the medoid, so each one's
  // symmetric part, relative to it, is positive definite, and so is the
  // sum's: the sum is invertible with a positive determinant, as
  // nearest_rotation needs.
  mean.transform = camera::Mat4d(camera::nearest_rotation(sum));
  mean.transform[3] = glm::dvec4(median(std::move(x)), median(std::move(y)),
                                 median(std::move(z)), 1.0);
  return mean;
}

// One used view: its observation, camera and poses.
struct UsedView {
  std::size_t observation = 0;
  std::size_t camera = 0;
  ViewPose pose;
};

// The views each capture uses, in the order of their cameras' ids.
struct Capture {
  std::int64_t id = 0;
  std::vector<UsedView> views;
};

const UsedView* find_view(const Capture& capture, std::size_t camera) noexcept {
  for (const UsedView& view : capture.views) {
    if (view.camera == camera) return &view;
  }
  return nullptr;
}

const char* name_of(ViewFate fate) noexcept {
  switch (fate) {
    case ViewFate::Used:
      return "Used";
    case ViewFate::Ambiguous:
      return "Ambiguous";
    case ViewFate::NotUsable:
      return "NotUsable";
    case ViewFate::NoPose:
      return "NoPose";
    case ViewFate::PoorFit:
      return "PoorFit";
  }
  return "?";
}

// Why each camera (in id order) with no view used has none: "; of SYN3's 45
// views none is used: 45 PoorFit", or "; SYN3 has no view". Empty when every
// camera has a view used.
std::string unused_cameras(const std::vector<ViewRecord>& records,
                           const std::vector<std::string>& ids,
                           const std::vector<std::size_t>& by_rank) {
  const std::size_t k = ids.size();
  // PoorFit is ViewFate's last value.
  constexpr std::size_t kFates =
      static_cast<std::size_t>(ViewFate::PoorFit) + 1;
  std::vector<std::vector<int>> counts(k, std::vector<int>(kFates, 0));
  for (const ViewRecord& record : records) {
    ++counts[record.camera][static_cast<std::size_t>(record.fate)];
  }
  std::string why;
  for (const std::size_t c : by_rank) {
    const std::vector<int>& count = counts[c];
    if (count[static_cast<std::size_t>(ViewFate::Used)] > 0) continue;
    int total = 0;
    for (const int n : count) total += n;
    if (total == 0) {
      why += "; " + ids[c] + " has no view";
      continue;
    }
    why += "; of " + ids[c] + "'s " + std::to_string(total) +
           (total == 1 ? " view" : " views") + " none is used:";
    const char* separator = " ";
    for (std::size_t f = 0; f < kFates; ++f) {
      if (count[f] == 0) continue;
      why += separator + std::to_string(count[f]) + " " +
             name_of(static_cast<ViewFate>(f));
      separator = ", ";
    }
  }
  return why;
}

}  // namespace

core::Result<InitialRig> initialise(const std::vector<RigCamera>& cameras,
                                    const compute::Board& board,
                                    const std::vector<Observation>& views,
                                    std::size_t reference,
                                    const InitialiseOptions& options) {
  return compute::guarded("initialise", [&]() -> core::Result<InitialRig> {
    VKC_TRY(check_options(options));
    const std::size_t k = cameras.size();
    if (k == 0) {
      return core::Status::invalid_argument(
          "initialise: the rig has no cameras");
    }
    if (reference >= k) {
      return core::Status::invalid_argument(
          "initialise: reference " + std::to_string(reference) +
          " is not one of the rig's " + std::to_string(k) + " cameras");
    }

    // The cameras by id, in OpenCV's types, and each one's rank in id order.
    std::map<std::string, std::size_t> by_id;
    std::vector<std::string> ids;
    std::vector<compute::CvCamera> lenses;
    ids.reserve(k);
    lenses.reserve(k);
    for (std::size_t c = 0; c < k; ++c) {
      const std::string& id = cameras[c].id;
      if (id.empty()) {
        return core::Status::invalid_argument("initialise: camera " +
                                              std::to_string(c) + " has no id");
      }
      if (!by_id.emplace(id, c).second) {
        return core::Status::invalid_argument("initialise: camera id \"" + id +
                                              "\" appears twice");
      }
      core::Result<compute::CvCamera> lens =
          compute::cv_camera(cameras[c].model);
      if (!lens.ok()) {
        return lens.status().with_context("initialise: camera " + id);
      }
      ids.push_back(id);
      lenses.push_back(*lens);
    }
    std::vector<std::size_t> rank(k);
    std::vector<std::size_t> by_rank;
    by_rank.reserve(k);
    for (const auto& entry : by_id) {
      rank[entry.second] = by_rank.size();
      by_rank.push_back(entry.second);
    }

    // 1. Each view: whether it is usable, its own poses, and its fate.
    InitialRig rig;
    rig.reference = reference;
    rig.views.resize(views.size());
    std::map<std::int64_t, Capture> captures;
    std::map<std::pair<std::int64_t, std::size_t>, std::size_t> seen;
    for (std::size_t i = 0; i < views.size(); ++i) {
      const Observation& view = views[i];
      const auto found = by_id.find(view.camera_id);
      if (found == by_id.end()) {
        return core::Status::invalid_argument(
            "initialise: observation " + std::to_string(i) +
            " names camera \"" + view.camera_id + "\", which the rig lacks");
      }
      const std::size_t c = found->second;
      if (!seen.emplace(std::make_pair(view.frame_id, c), i).second) {
        return core::Status::invalid_argument(
            "initialise: camera " + view.camera_id +
            " has two views of capture " + std::to_string(view.frame_id));
      }
      ViewRecord& record = rig.views[i];
      record.camera = c;
      record.capture = view.frame_id;

      VKC_ASSIGN(record.use,
                 compute::usable_for_pose(board, view, lenses[c].size));
      if (record.use != compute::ViewUse::Usable) {
        record.fate = ViewFate::NotUsable;
        continue;
      }
      core::Result<ViewPose> pose = view_pose(lenses[c], board, view);
      if (!pose.ok()) {
        if (pose.status().domain() != core::Status::Code::Numerical) {
          return pose.status().with_context(
              "initialise: camera " + view.camera_id + "'s view of capture " +
              std::to_string(view.frame_id));
        }
        record.fate = ViewFate::NoPose;
        continue;
      }
      record.pnp_rms_px = pose->best.rms_px;
      record.ambiguity = pose->ambiguity();
      record.tilt_deg = tilt_deg(board, pose->best.board_to_camera);
      record.oblique = *record.tilt_deg > options.oblique_deg;
      if (!(pose->best.rms_px <= options.max_pnp_rms_px)) {
        record.fate = ViewFate::PoorFit;
      } else if (!(*record.ambiguity >= options.min_ambiguity)) {
        record.fate = ViewFate::Ambiguous;
      } else {
        record.fate = ViewFate::Used;
        Capture& capture = captures[view.frame_id];
        capture.id = view.frame_id;
        capture.views.push_back({i, c, *std::move(pose)});
      }
    }
    for (auto& entry : captures) {
      std::vector<UsedView>& used = entry.second.views;
      std::sort(used.begin(), used.end(),
                [&](const UsedView& a, const UsedView& b) {
                  return rank[a.camera] < rank[b.camera];
                });
    }

    // 2. The tree of the links the used views share most.
    std::vector<std::vector<std::size_t>> seen_by;
    seen_by.reserve(captures.size());
    for (const auto& entry : captures) {
      std::vector<std::size_t>& list = seen_by.emplace_back();
      for (const UsedView& view : entry.second.views) {
        list.push_back(view.camera);
      }
    }
    VKC_ASSIGN(const Covisibility shared, covisibility(k, seen_by));
    core::Result<SpanningTree> tree =
        spanning_tree(shared, ids, reference, options.weak_below);
    if (!tree.ok()) {
      if (tree.status().domain() != core::Status::Code::InvalidArgument) {
        return tree.status().with_context("initialise");
      }
      // A camera outside the tree: the tree counts used views alone, so say
      // why a camera has none.
      return core::Status::invalid_argument(
          "initialise: " + tree.status().message() +
          unused_cameras(rig.views, ids, by_rank));
    }

    // 3. Each camera from its parent, from the reference outwards; a link
    // shares a capture at least, so each has a relative pose.
    rig.world_to_camera.assign(k, camera::Mat4d(1.0));
    rig.links.reserve(tree->links.size());
    for (const TreeLink& link : tree->links) {
      std::vector<camera::Mat4d> relative;  // parent to camera, per capture
      for (const auto& entry : captures) {
        const UsedView* child = find_view(entry.second, link.camera);
        const UsedView* parent = find_view(entry.second, link.parent);
        if (child == nullptr || parent == nullptr) continue;
        relative.push_back(
            child->pose.best.board_to_camera *
            camera::rigid_inverse(parent->pose.best.board_to_camera));
      }
      const RobustMean mean = robust_mean(relative, options.inlier_deg);
      rig.world_to_camera[link.camera] =
          mean.transform * rig.world_to_camera[link.parent];
      rig.links.push_back({link, mean.inliers});
    }

    // 4. Each capture's board: the candidate that best fits its used views.
    // TODO: CA5 decides how to seed, or drop, a capture whose views are all
    // Ambiguous; it has no board pose here, so the adjustment cannot use them.
    const double cap = options.truncation_px * options.truncation_px;
    for (const auto& entry : captures) {
      const Capture& capture = entry.second;
      std::optional<camera::Mat4d> best;
      double least = std::numeric_limits<double>::infinity();
      for (const UsedView& from : capture.views) {
        const camera::Mat4d camera_to_world =
            camera::rigid_inverse(rig.world_to_camera[from.camera]);
        for (const std::optional<PoseCandidate>& candidate :
             {std::optional<PoseCandidate>(from.pose.best),
              from.pose.alternative}) {
          if (!candidate) continue;
          const camera::Mat4d board_to_world =
              camera_to_world * candidate->board_to_camera;
          double score = 0.0;
          for (const UsedView& view : capture.views) {
            for (const double e2 : detail::squared_errors(
                     lenses[view.camera], board, views[view.observation],
                     rig.world_to_camera[view.camera] * board_to_world)) {
              score += std::min(e2, cap);
            }
          }
          if (!best || score < least) {
            least = score;
            best = board_to_world;
          }
        }
      }
      rig.boards.push_back({capture.id, *best});
    }
    return rig;
  });
}

}  // namespace volumetric_kit::calib::solve
