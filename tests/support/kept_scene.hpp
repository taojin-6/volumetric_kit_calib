// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file kept_scene.hpp
/// @brief Calibrate's tests' input: the synthetic scene (synthetic_scene.hpp)
///        for 2 to 5 cameras whose ids sort in another order than the rig's,
///        and its captures as the capture worker keeps them
///        (`app::KeptCaptures`). Its users link calib_app and
///        vc_synthetic_rig.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <glm/trigonometric.hpp>

#include "synthetic_scene.hpp"
#include "volumetric_kit/calib/app/capture_worker.hpp"
#include "volumetric_kit/calib/core/target.hpp"

namespace volumetric_kit::calib::test {

/// The most cameras a kept scene has.
inline constexpr std::size_t kKeptSceneMaxCameras = 5;

/// @brief A kept scene's camera's id: sorted, the ids are in another order
///        than the rig's, and rig camera 0 sorts fourth.
/// @param c  The camera, by rig index; below @ref kKeptSceneMaxCameras.
/// @return Its id.
inline std::string kept_scene_id(std::size_t c) {
  static const char* const kIds[kKeptSceneMaxCameras] = {"CL8F", "CL2A", "CL9C",
                                                         "CL0B", "CL5E"};
  return kIds[c];
}

/// @brief @ref synthetic_rig's first @p count cameras, renamed
///        (@ref kept_scene_id); a fifth, for 5, is its first turned 130
///        degrees about the floor's z, on the ring between its second and
///        third, its principal point 3 px off the first's.
/// @param count  The cameras, 2 to @ref kKeptSceneMaxCameras.
/// @return The cameras.
inline std::vector<SyntheticCamera> kept_scene_rig(std::size_t count) {
  std::vector<SyntheticCamera> rig = synthetic_rig();
  SyntheticCamera fifth = rig.front();
  const double a = glm::radians(130.0);
  recon::camera::Mat4d turn(1.0);  // the camera moved about the floor's z
  turn[0][0] = std::cos(a);
  turn[0][1] = std::sin(a);
  turn[1][0] = -std::sin(a);
  turn[1][1] = std::cos(a);
  fifth.world_to_camera =
      fifth.world_to_camera * recon::camera::rigid_inverse(turn);
  fifth.model.intrinsics.cx += 3.0;
  fifth.model.intrinsics.cy -= 3.0;
  rig.push_back(fifth);
  rig.resize(count);
  for (std::size_t c = 0; c < rig.size(); ++c) rig[c].id = kept_scene_id(c);
  return rig;
}

/// @brief The scene @ref make_scene makes, of @ref kept_scene_rig's cameras.
/// @param cameras     The cameras, 2 to @ref kKeptSceneMaxCameras.
/// @param count       Placements to plan (@ref PlanOptions::count).
/// @param noise_px    The detections' noise (@ref ObserveOptions::noise_px).
/// @param seed        The plan's and the noise's seed.
/// @return The scene, the same for the same arguments.
inline Scene make_kept_scene(std::size_t cameras, std::size_t count = 40,
                             double noise_px = 0.3, std::uint32_t seed = 1) {
  return make_scene(kept_scene_rig(cameras), count, noise_px, seed, seed);
}

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
    recon::sensor::SensorInfo info;
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

}  // namespace volumetric_kit::calib::test
