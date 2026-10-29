// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file kept_scene.hpp
/// @brief Calibrate's tests' input: the synthetic scene of 2 to 5 cameras
///        (synthetic_scene.hpp's @ref make_kept_scene) and its captures as the
///        capture worker keeps them (`app::KeptCaptures`), with or without the
///        origin tag. Its users link calib_app and vc_synthetic_rig.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "synthetic_scene.hpp"
#include "volumetric_kit/calib/app/capture_worker.hpp"
#include "volumetric_kit/calib/core/target.hpp"

namespace volumetric_kit::calib::test {

/// @brief A kept capture's index for a placement: odd, as if every other
///        capture were dropped.
/// @param placement  The placement, by index in @ref Scene::plan.
/// @return The capture's index.
inline int kept_index(std::size_t placement) {
  return static_cast<int>(2 * placement + 1);
}

/// @brief @p scene's first @p captures placements as the capture worker keeps
///        them: each camera's view, named for its camera and numbered
///        @ref kept_index (a camera that does not see the board has none),
///        an empty depth and no origin tag per camera, the rig's board
///        (@ref legacy_femto_board) and default solve thresholds.
/// @param scene      The scene.
/// @param reference  The reference camera, by rig index.
/// @param directory  The session's directory.
/// @param captures   Placements kept, from the first; all for more than
///                   there are.
/// @return The snapshot.
inline app::KeptCaptures kept_captures(const Scene& scene,
                                       std::size_t reference,
                                       const std::string& directory,
                                       std::size_t captures = SIZE_MAX) {
  app::KeptCaptures kept;
  kept.board = legacy_femto_board();
  kept.cameras = scene.cameras;
  for (const solve::RigCamera& cam : scene.cameras) {
    sensor::SensorInfo info;
    info.id = cam.id;
    info.model = "synthetic";
    info.color = cam.model;
    info.depth = depth_like_lens();
    info.depth_to_color[3] = glm::dvec4(-0.032, 0.0, 0.002, 1.0);
    kept.sensors.push_back(info);
  }
  kept.reference = reference;
  kept.depth_mode = {"nfov_unbinned", 0.001f, 0.25f, 2.88f};
  kept.directory = directory;
  const std::size_t n = std::min(captures, scene.plan.size());
  std::vector<std::shared_ptr<app::KeptCapture>> made;
  for (std::size_t p = 0; p < n; ++p) {
    auto capture = std::make_shared<app::KeptCapture>();
    capture->index = kept_index(p);
    for (const solve::RigCamera& cam : scene.cameras) {
      Detection view;
      view.image_size = cam.model.size;
      view.observation.camera_id = cam.id;
      view.observation.frame_id = capture->index;
      capture->views.push_back(view);
      capture->depth.push_back(
          std::make_shared<const std::vector<std::uint16_t>>());
      capture->tags.emplace_back();
    }
    made.push_back(std::move(capture));
  }
  for (const Observation& view : scene.views) {
    const auto p = static_cast<std::size_t>(view.frame_id);
    if (p >= n) continue;
    made[p]->views[index_of(scene.cameras, view.camera_id)].observation.points =
        view.points;
  }
  kept.captures.assign(made.begin(), made.end());
  return kept;
}

/// @brief @p kept with the origin tag, lying at the world's origin
///        (@ref FloorTag's defaults), in each camera's tags that sees it, in
///        every capture, as @ref observe_marker sees it.
/// @param kept      Captures of @p scene (@ref kept_captures); each is
///                  replaced by a copy with the tags.
/// @param scene     The scene.
/// @param noise_px  The corners' noise (@ref ObserveOptions::noise_px).
/// @param seed      The noise's seed.
/// @return The snapshot.
inline app::KeptCaptures with_origin_tag(app::KeptCaptures kept,
                                         const Scene& scene,
                                         double noise_px = 0.3,
                                         std::uint32_t seed = 1) {
  const std::vector<MarkerSighting> seen =
      observe_marker(scene.rig, kOriginTag.id, floor_tag_corners(),
                     kept.captures.size(), {noise_px, seed});
  for (std::size_t k = 0; k < kept.captures.size(); ++k) {
    auto copy = std::make_shared<app::KeptCapture>(*kept.captures[k]);
    for (const MarkerSighting& s : seen) {
      if (s.capture == static_cast<std::int64_t>(k)) {
        copy->tags[s.camera] = s.marker;
      }
    }
    kept.captures[k] = std::move(copy);
  }
  return kept;
}

}  // namespace volumetric_kit::calib::test
