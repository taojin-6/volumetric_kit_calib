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

#include "common.hpp"
#include "reprojection.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/solve/view_pose.hpp"

namespace volumetric_kit::calib::solve {
namespace {

namespace camera = recon::camera;

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
      angles[j] = compute::rotation_angle_deg(rotations[i], rotations[j]);
    }
    const double m = compute::median(std::move(angles));
    if (m < least) {
      least = m;
      medoid = i;
    }
  }

  camera::Mat3d sum(0.0);
  std::vector<double> x, y, z;
  for (std::size_t i = 0; i < n; ++i) {
    if (!(compute::rotation_angle_deg(rotations[i], rotations[medoid]) <=
          inlier_deg)) {
      continue;
    }
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
  mean.transform[3] =
      glm::dvec4(compute::median(std::move(x)), compute::median(std::move(y)),
                 compute::median(std::move(z)), 1.0);
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

core::Status check_camera_ids(const std::vector<std::string>& ids) {
  return compute::guarded("check_camera_ids", [&]() -> core::Status {
    if (ids.empty()) {
      return core::Status::invalid_argument("the rig has no cameras");
    }
    std::map<std::string, std::size_t> seen;
    for (std::size_t c = 0; c < ids.size(); ++c) {
      if (ids[c].empty()) {
        return core::Status::invalid_argument("camera " + std::to_string(c) +
                                              " has no id");
      }
      if (!seen.emplace(ids[c], c).second) {
        return core::Status::invalid_argument("camera id \"" + ids[c] +
                                              "\" appears twice");
      }
    }
    return {};
  });
}

core::Result<RigLenses> check_rig(const std::vector<RigCamera>& cameras,
                                  std::size_t reference) {
  return compute::guarded("check_rig", [&]() -> core::Result<RigLenses> {
    RigLenses rig;
    rig.ids.reserve(cameras.size());
    rig.lenses.reserve(cameras.size());
    for (const RigCamera& camera : cameras) rig.ids.push_back(camera.id);
    VKC_TRY(check_camera_ids(rig.ids));
    if (reference >= cameras.size()) {
      return core::Status::invalid_argument(
          "reference " + std::to_string(reference) +
          " is not one of the rig's " + std::to_string(cameras.size()) +
          " cameras");
    }
    for (const RigCamera& camera : cameras) {
      core::Result<compute::CvCamera> lens = compute::cv_camera(camera.model);
      if (!lens.ok()) return lens.status().with_context("camera " + camera.id);
      rig.lenses.push_back(*lens);
    }
    return rig;
  });
}

core::Status check_initialise_options(const InitialiseOptions& options,
                                      const std::string& prefix) {
  return compute::guarded("check_initialise_options", [&]() -> core::Status {
    const auto refuse = [&](const char* name, const char* rule) {
      return core::Status::invalid_argument(prefix + name + " must be " + rule);
    };
    if (!(options.max_pnp_rms_px > 0.0) ||
        !std::isfinite(options.max_pnp_rms_px)) {
      return refuse("max_pnp_rms_px", "positive and finite");
    }
    if (!(options.min_ambiguity >= 1.0) ||
        !std::isfinite(options.min_ambiguity)) {
      return refuse("min_ambiguity", "at least 1 and finite");
    }
    if (!(options.oblique_deg > 0.0 && options.oblique_deg <= 180.0)) {
      return refuse("oblique_deg", "in (0, 180]");
    }
    if (options.weak_below < 0) return refuse("weak_below", "at least 0");
    if (!(options.inlier_deg > 0.0 && options.inlier_deg < 90.0)) {
      return refuse("inlier_deg", "in (0, 90)");
    }
    if (!(options.truncation_px > 0.0) ||
        !std::isfinite(options.truncation_px)) {
      return refuse("truncation_px", "positive and finite");
    }
    return {};
  });
}

ViewFate view_fate(const ViewPose& pose,
                   const InitialiseOptions& options) noexcept {
  if (!(pose.best.rms_px <= options.max_pnp_rms_px)) return ViewFate::PoorFit;
  if (!(pose.ambiguity() >= options.min_ambiguity)) return ViewFate::Ambiguous;
  return ViewFate::Used;
}

core::Result<InitialRig> initialise(const std::vector<RigCamera>& cameras,
                                    const compute::Board& board,
                                    const std::vector<Observation>& views,
                                    std::size_t reference,
                                    const InitialiseOptions& options) {
  return compute::guarded("initialise", [&]() -> core::Result<InitialRig> {
    VKC_TRY(check_initialise_options(options));
    core::Result<RigLenses> rig_lenses = check_rig(cameras, reference);
    if (!rig_lenses.ok()) return rig_lenses.status().with_context("initialise");
    VKC_TRY(detail::check_one_view_each(views, "initialise: "));
    const std::size_t k = cameras.size();
    const std::vector<std::string>& ids = rig_lenses->ids;
    const std::vector<compute::CvCamera>& lenses = rig_lenses->lenses;

    // The cameras by id, and each one's rank in id order.
    std::map<std::string, std::size_t> by_id;
    for (std::size_t c = 0; c < k; ++c) by_id.emplace(ids[c], c);
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
    std::map<std::int64_t, Capture> ambiguous;
    for (std::size_t i = 0; i < views.size(); ++i) {
      const Observation& view = views[i];
      const auto found = by_id.find(view.camera_id);
      if (found == by_id.end()) {
        return core::Status::invalid_argument(
            "initialise: observation " + std::to_string(i) +
            " names camera \"" + view.camera_id + "\", which the rig lacks");
      }
      const std::size_t c = found->second;
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
      record.fate = view_fate(*pose, options);
      if (record.fate == ViewFate::Used || record.fate == ViewFate::Ambiguous) {
        Capture& capture =
            (record.fate == ViewFate::Used ? captures
                                           : ambiguous)[view.frame_id];
        capture.id = view.frame_id;
        capture.views.push_back({i, c, *std::move(pose)});
      }
    }
    for (auto* by_capture : {&captures, &ambiguous}) {
      for (auto& entry : *by_capture) {
        std::vector<UsedView>& list = entry.second.views;
        std::sort(list.begin(), list.end(),
                  [&](const UsedView& a, const UsedView& b) {
                    return rank[a.camera] < rank[b.camera];
                  });
      }
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

    // 4. Each capture's board: the candidate that best fits its used views,
    // or, where it has none, its ambiguous views, which the adjustment takes.
    for (auto& entry : ambiguous) captures.insert(std::move(entry));
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
