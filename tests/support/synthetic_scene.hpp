// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file synthetic_scene.hpp
/// @brief The solve's tests' scene, from the synthetic rig
///        (synthetic_rig.hpp): the rig, of its 4 cameras or of 2 to 5 whose
///        ids sort in another order than the rig's, a plan, what its cameras
///        see and the cameras as the solve takes them; how many scenes a
///        sweep draws; and the measures the tests compare rigs by. Its users
///        link calib_solve and vc_synthetic_rig.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
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

/// @brief How many scenes a statistical sweep (a suite named `*Sweep`) draws:
///        all of DECISIONS.md's evidence when `VC_TEST_SWEEPS` is set, as the
///        `slow` tests set it (tests/CMakeLists.txt), and a few otherwise.
/// @param few  The default suite's count.
/// @param all  The sweep's.
/// @return @p all or @p few.
inline std::uint32_t sweep_size(std::uint32_t few, std::uint32_t all) {
  return std::getenv("VC_TEST_SWEEPS") != nullptr ? all : few;
}

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
