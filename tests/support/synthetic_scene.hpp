// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file synthetic_scene.hpp
/// @brief The solve's tests' scene, from the synthetic rig
///        (synthetic_rig.hpp): the rig, a plan, what its cameras see and the
///        cameras as the solve takes them; and the measures the tests compare
///        rigs by. Its users link calib_solve and vc_synthetic_rig.

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <glm/trigonometric.hpp>

#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"

namespace volumetric_kit::calib::test {

/// @brief The synthetic rig, a plan of its placements and what its cameras
///        see.
///
/// @code
/// const Scene scene = make_scene(20);
/// initialise(scene.cameras, scene.board, scene.views, 0);
/// @endcode
struct Scene {
  std::vector<SyntheticCamera> rig;       ///< The cameras and their truths.
  compute::Board board;                   ///< The board placed.
  std::vector<Placement> plan;            ///< Where it was placed.
  std::vector<Observation> views;         ///< What the cameras saw of it.
  std::vector<solve::RigCamera> cameras;  ///< @ref rig as the solve takes it.
};

/// @brief @p rig's cameras as the solve takes them.
/// @param rig  The cameras.
/// @return Each one's id and lens, in @p rig's order.
inline std::vector<solve::RigCamera> rig_cameras(
    const std::vector<SyntheticCamera>& rig) {
  std::vector<solve::RigCamera> cameras;
  for (const SyntheticCamera& cam : rig) cameras.push_back({cam.id, cam.model});
  return cameras;
}

/// @brief @p rig, a plan for it and what it sees.
/// @param rig         The cameras.
/// @param count       Placements to plan (@ref PlanOptions::count).
/// @param noise_px    The detections' noise (@ref ObserveOptions::noise_px).
/// @param plan_seed   The plan's seed.
/// @param noise_seed  The noise's seed.
/// @return The scene, the same for the same arguments.
inline Scene make_scene(std::vector<SyntheticCamera> rig, std::size_t count,
                        double noise_px, std::uint32_t plan_seed,
                        std::uint32_t noise_seed) {
  Scene scene{std::move(rig), femto_board(), {}, {}, {}};
  PlanOptions plan_options;
  plan_options.count = count;
  plan_options.seed = plan_seed;
  scene.plan = synthetic_plan(scene.rig, scene.board, plan_options);
  scene.views = observe(scene.rig, scene.board, scene.plan,
                        ObserveOptions{noise_px, noise_seed});
  scene.cameras = rig_cameras(scene.rig);
  return scene;
}

/// @brief @ref synthetic_rig, a plan for it and what it sees.
/// @param count       Placements to plan (@ref PlanOptions::count).
/// @param noise_px    The detections' noise (@ref ObserveOptions::noise_px).
/// @param plan_seed   The plan's seed.
/// @param noise_seed  The noise's seed.
/// @return The scene, the same for the same arguments.
inline Scene make_scene(std::size_t count = 60, double noise_px = 0.3,
                        std::uint32_t plan_seed = 1,
                        std::uint32_t noise_seed = 1) {
  return make_scene(synthetic_rig(), count, noise_px, plan_seed, noise_seed);
}

/// @brief The camera named @p id.
/// @param cameras  The cameras.
/// @param id       The id.
/// @return Its index; 0, failing the test, when none has it.
inline std::size_t index_of(const std::vector<solve::RigCamera>& cameras,
                            const std::string& id) {
  for (std::size_t c = 0; c < cameras.size(); ++c) {
    if (cameras[c].id == id) return c;
  }
  ADD_FAILURE() << "no camera " << id;
  return 0;
}

/// @brief The angle between two transforms' rotations, accurate near 0 and
///        180 degrees.
/// @param a  A transform.
/// @param b  Another.
/// @return The angle (degrees).
inline double angle_deg(const recon::camera::Mat4d& a,
                        const recon::camera::Mat4d& b) {
  return glm::degrees(glm::length(recon::camera::rodrigues_from_rotation(
      glm::transpose(recon::camera::Mat3d(a)) * recon::camera::Mat3d(b))));
}

/// @brief Where a camera is.
/// @param world_to_camera  T_cw.
/// @return Its centre in the world.
inline recon::camera::Vec3d centre_of(
    const recon::camera::Mat4d& world_to_camera) {
  return recon::camera::Vec3d(recon::camera::rigid_inverse(world_to_camera)[3]);
}

/// @brief Each camera's true T_cw in the world of one of them, as the solve
///        places the rig.
/// @param scene      The scene.
/// @param reference  The camera whose frame is the world, by index in
///                   @ref Scene::rig.
/// @return Each camera's T_cw, in @ref Scene::rig's order.
inline std::vector<recon::camera::Mat4d> truths_in_world_of(
    const Scene& scene, std::size_t reference) {
  const recon::camera::Mat4d reference_to_world =
      recon::camera::rigid_inverse(scene.rig[reference].world_to_camera);
  std::vector<recon::camera::Mat4d> truth;
  for (const SyntheticCamera& cam : scene.rig) {
    truth.push_back(cam.world_to_camera * reference_to_world);
  }
  return truth;
}

}  // namespace volumetric_kit::calib::test
