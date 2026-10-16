// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/session_checks.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "checks.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;

core::Status refuse_option(const char* name, const char* rule) {
  return core::Status::invalid_argument(std::string("SessionChecks: options.") +
                                        name + " must be " + rule);
}

core::Status check_options(const SessionOptions& options) {
  VKC_TRY(
      detail::check_view_options(options.view, "SessionChecks: options.view."));
  VKC_TRY(solve::check_initialise_options(options.solve,
                                          "SessionChecks: options.solve."));
  if (!(options.duplicate_m > 0.0) || !std::isfinite(options.duplicate_m)) {
    return refuse_option("duplicate_m", "positive and finite");
  }
  if (!(options.duplicate_deg > 0.0 && options.duplicate_deg <= 180.0)) {
    return refuse_option("duplicate_deg", "in (0, 180]");
  }
  if (!(options.floor_share > 0.0 && options.floor_share <= 1.0)) {
    return refuse_option("floor_share", "in (0, 1]");
  }
  if (!(options.floor_deg >= 0.0 && options.floor_deg <= 180.0)) {
    return refuse_option("floor_deg", "in [0, 180]");
  }
  if (!(options.frontal_share >= 0.0 && options.frontal_share < 1.0)) {
    return refuse_option("frontal_share", "in [0, 1)");
  }
  if (options.warn_from < 1) return refuse_option("warn_from", "at least 1");
  if (options.strong_from < options.solve.weak_below) {
    return refuse_option("strong_from", "at least solve.weak_below");
  }
  if (options.ready_from < 1) return refuse_option("ready_from", "at least 1");
  return {};
}

}  // namespace

SessionChecks::SessionChecks(std::vector<std::string> ids,
                             std::vector<compute::CvCamera> cameras,
                             compute::Board board, std::size_t reference,
                             const SessionOptions& options)
    : ids_(std::move(ids)),
      cameras_(std::move(cameras)),
      board_(std::move(board)),
      reference_(reference),
      options_(options) {}

core::Result<SessionChecks> SessionChecks::create(
    const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
    std::size_t reference, const SessionOptions& options) {
  return compute::guarded(
      "SessionChecks", [&]() -> core::Result<SessionChecks> {
        core::Result<solve::RigLenses> rig =
            solve::check_rig(cameras, reference);
        if (!rig.ok()) return rig.status().with_context("SessionChecks");
        VKC_TRY(check_options(options));
        return SessionChecks(std::move(rig->ids), std::move(rig->lenses), board,
                             reference, options);
      });
}

core::Status SessionChecks::add(int index,
                                const std::vector<Detection>& detections) {
  return compute::guarded("SessionChecks::add", [&]() -> core::Status {
    if (index < 0) {
      return core::Status::invalid_argument(
          "SessionChecks::add: capture index " + std::to_string(index) +
          " is negative");
    }
    for (const Capture& capture : captures_) {
      if (capture.index == index) {
        return core::Status::invalid_argument("SessionChecks::add: capture " +
                                              std::to_string(index) +
                                              " is added already");
      }
    }
    if (detections.size() != cameras_.size()) {
      return core::Status::invalid_argument(
          "SessionChecks::add: " + std::to_string(detections.size()) +
          " detections for " + std::to_string(cameras_.size()) + " cameras");
    }
    Capture capture;
    capture.index = index;
    capture.views.resize(cameras_.size());
    for (std::size_t c = 0; c < cameras_.size(); ++c) {
      core::Result<CheckedView> checked =
          check_view(cameras_[c], board_, detections[c], options_.view);
      if (!checked.ok()) {
        return checked.status().with_context("SessionChecks::add: camera " +
                                             ids_[c]);
      }
      View& view = capture.views[c];
      view.usable = is_usable(checked->check.verdict);
      if (!view.usable) continue;
      view.fate = solve::view_fate(*checked->pose, options_.solve);
      view.frontal = checked->frontal;
      view.board_to_camera = checked->pose->best.board_to_camera;
      for (const DetectedPoint& point : detections[c].observation.points) {
        view.corners.push_back(point.pixel);
      }
    }
    captures_.push_back(std::move(capture));
    return {};
  });
}

core::Status SessionChecks::drop(int index) {
  return compute::guarded("SessionChecks::drop", [&]() -> core::Status {
    for (Capture& capture : captures_) {
      if (capture.index != index) continue;
      if (capture.dropped) {
        return core::Status::invalid_argument("SessionChecks::drop: capture " +
                                              std::to_string(index) +
                                              " is dropped already");
      }
      capture.dropped = true;
      return {};
    }
    return core::Status::not_found("SessionChecks::drop: no capture " +
                                   std::to_string(index));
  });
}

core::Result<SessionSummary> SessionChecks::summary() const {
  return compute::guarded(
      "SessionChecks::summary", [&]() -> core::Result<SessionSummary> {
        const std::size_t k = cameras_.size();
        SessionSummary summary;
        summary.cameras.resize(k);
        std::vector<const Capture*> kept;
        for (const Capture& capture : captures_) {
          if (capture.dropped) {
            ++summary.dropped;
          } else {
            kept.push_back(&capture);
          }
        }
        summary.kept = static_cast<int>(kept.size());

        // Views, coverage, and the floor-only and frontal warnings.
        std::vector<std::vector<std::size_t>> used_by;
        for (const Capture* capture : kept) {
          std::vector<std::size_t>& using_it = used_by.emplace_back();
          for (std::size_t c = 0; c < k; ++c) {
            if (capture->views[c].fate == solve::ViewFate::Used) {
              using_it.push_back(c);
            }
          }
        }
        for (std::size_t c = 0; c < k; ++c) {
          CameraSummary& cam = summary.cameras[c];
          std::vector<camera::Vec3d> normals;
          for (const Capture* capture : kept) {
            const View& view = capture->views[c];
            if (!view.usable) continue;
            ++cam.usable_views;
            if (view.fate == solve::ViewFate::Used) ++cam.used_views;
            if (view.fate == solve::ViewFate::Ambiguous) ++cam.ambiguous_views;
            if (view.frontal) ++cam.frontal_views;
            normals.push_back(compute::face_normal(view.board_to_camera));
            for (const camera::Vec2d& pixel : view.corners) {
              if (const std::optional<int> cell =
                      detail::grid_cell(pixel.x, pixel.y, cameras_[c].size,
                                        kCoverageCols, kCoverageRows)) {
                ++cam.coverage[static_cast<std::size_t>(*cell)];
              }
            }
          }
          const auto filled =
              std::count_if(cam.coverage.begin(), cam.coverage.end(),
                            [](int n) { return n > 0; });
          cam.coverage_filled = static_cast<double>(filled) /
                                static_cast<double>(cam.coverage.size());
          if (cam.usable_views < options_.warn_from) continue;
          const auto n = static_cast<double>(normals.size());
          std::size_t alike = 0;
          for (const camera::Vec3d& a : normals) {
            alike = std::max<std::size_t>(
                alike, static_cast<std::size_t>(
                           std::count_if(normals.begin(), normals.end(),
                                         [&](const camera::Vec3d& b) {
                                           return compute::angle_deg(a, b) <=
                                                  options_.floor_deg;
                                         })));
          }
          cam.floor_only =
              static_cast<double>(alike) / n >= options_.floor_share;
          cam.mostly_frontal = static_cast<double>(cam.frontal_views) / n >
                               options_.frontal_share;
        }

        // Which cameras share captures the solve uses, and the tree over those
        // the reference reaches.
        VKC_ASSIGN(summary.covisibility, solve::covisibility(k, used_by));
        VKC_ASSIGN(const std::vector<bool> reached,
                   solve::reached_from(summary.covisibility, reference_));
        std::vector<std::size_t> linked;  // reached cameras, in the rig's order
        for (std::size_t c = 0; c < k; ++c) {
          if (reached[c]) linked.push_back(c);
        }
        solve::Covisibility shared(linked.size(),
                                   std::vector<int>(linked.size(), 0));
        std::vector<std::string> ids;
        std::size_t root = 0;
        for (std::size_t a = 0; a < linked.size(); ++a) {
          if (linked[a] == reference_) root = a;
          ids.push_back(ids_[linked[a]]);
          for (std::size_t b = 0; b < linked.size(); ++b) {
            shared[a][b] = summary.covisibility[linked[a]][linked[b]];
          }
        }
        VKC_ASSIGN(
            const solve::SpanningTree tree,
            solve::spanning_tree(shared, ids, root, options_.solve.weak_below));
        bool strong_enough = true;
        for (const solve::TreeLink& link : tree.links) {
          PreviewLink preview;
          preview.parent = linked[link.parent];
          preview.shared = link.shared;
          preview.strength = link.weak ? LinkStrength::Weak
                             : link.shared >= options_.strong_from
                                 ? LinkStrength::Strong
                                 : LinkStrength::Fair;
          summary.cameras[linked[link.camera]].link = preview;
          strong_enough = strong_enough && link.shared >= options_.ready_from;
        }
        for (const std::size_t c : linked) summary.cameras[c].linked = true;
        summary.ready = k > 1 && linked.size() == k && strong_enough;

        // Captures that repeat an earlier one in every camera that sees them.
        const auto repeats = [&](const Capture& later, const Capture& earlier) {
          bool seen = false;
          for (std::size_t c = 0; c < k; ++c) {
            const View& a = later.views[c];
            if (!a.usable) continue;
            seen = true;
            const View& b = earlier.views[c];
            if (!b.usable ||
                !(glm::length(compute::centre_in(board_, a.board_to_camera) -
                              compute::centre_in(board_, b.board_to_camera)) <
                  options_.duplicate_m) ||
                !(compute::rotation_angle_deg(
                      camera::Mat3d(a.board_to_camera),
                      camera::Mat3d(b.board_to_camera)) <
                  options_.duplicate_deg)) {
              return false;
            }
          }
          return seen;
        };
        for (std::size_t i = 0; i < kept.size(); ++i) {
          for (std::size_t j = 0; j < i; ++j) {
            if (repeats(*kept[i], *kept[j])) {
              summary.duplicates.push_back({kept[i]->index, kept[j]->index});
              break;
            }
          }
        }
        return summary;
      });
}

}  // namespace volumetric_kit::calib::quality
