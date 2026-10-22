// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The board-plane fit's blocks on the host: shaders/plane_common.glsl's
// Region, Fit and Partial under std430, and the two kernels' push constants.
// The static_asserts pin the packing, so a drift is a compile error here
// rather than a misread block there.

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace volumetric_kit::calib::depth::detail {

// plane_common.glsl's Region.
struct RegionBlock {
  glm::mat4 depth_to_board{1.0f};
  glm::vec4 k{0.0f};
  glm::vec4 rect{0.0f};
  glm::uvec2 size{0u};
  glm::vec2 range{0.0f};
  std::uint32_t first_partial = 0;
  std::uint32_t groups = 0;
  glm::uvec2 origin{0u};
  glm::uvec2 extent{0u};
  std::uint32_t pad[2] = {0, 0};
};
static_assert(sizeof(RegionBlock) == 144 && offsetof(RegionBlock, k) == 64 &&
                  offsetof(RegionBlock, rect) == 80 &&
                  offsetof(RegionBlock, size) == 96 &&
                  offsetof(RegionBlock, range) == 104 &&
                  offsetof(RegionBlock, first_partial) == 112 &&
                  offsetof(RegionBlock, groups) == 116 &&
                  offsetof(RegionBlock, origin) == 120 &&
                  offsetof(RegionBlock, extent) == 128,
              "mirrors Region in shaders/plane_common.glsl");

// plane_common.glsl's Fit: what comes back per region.
struct FitBlock {
  glm::vec3 point{0.0f};
  float rms = 0.0f;
  glm::vec3 normal{0.0f};
  std::uint32_t valid = 0;
  std::uint32_t members = 0;
  std::uint32_t out_of_range = 0;
  std::uint32_t inliers = 0;
  float gate = 0.0f;
};
static_assert(sizeof(FitBlock) == 48 && offsetof(FitBlock, rms) == 12 &&
                  offsetof(FitBlock, normal) == 16 &&
                  offsetof(FitBlock, valid) == 28 &&
                  offsetof(FitBlock, members) == 32 &&
                  offsetof(FitBlock, gate) == 44,
              "mirrors Fit in shaders/plane_common.glsl");

// plane_common.glsl's Partial, which only the kernels read and write.
inline constexpr std::size_t kPartialBytes = 48;

// plane_moments.comp's push constants.
struct MomentsParams {
  std::uint32_t region = 0;
  std::uint32_t round = 0;
  float seed = 0.0f;
};
static_assert(sizeof(MomentsParams) == 12,
              "mirrors Params in shaders/plane_moments.comp");

// plane_solve.comp's push constants.
struct SolveParams {
  std::uint32_t round = 0;
  std::uint32_t min_inliers = 0;
  float min_gate = 0.0f;
  float sigma_gate = 0.0f;
};
static_assert(sizeof(SolveParams) == 16,
              "mirrors Params in shaders/plane_solve.comp");

static_assert(std::is_trivially_copyable_v<RegionBlock> &&
                  std::is_trivially_copyable_v<FitBlock>,
              "uploaded and read back as bytes");

}  // namespace volumetric_kit::calib::depth::detail
