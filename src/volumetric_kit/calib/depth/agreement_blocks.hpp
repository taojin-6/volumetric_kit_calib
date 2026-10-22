// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The pair agreement's blocks on the host: shaders/agreement.comp's Pair and
// shaders/agreement_stats.comp's Stats under std430, both kernels' push
// constants, and agreement.comp's limits. The static_asserts pin the packing,
// so a drift is a compile error here rather than a misread block there.

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

#include "volumetric_kit/calib/quality/depth_checks.hpp"

namespace volumetric_kit::calib::depth::detail {

// agreement.comp's workgroup (VC_AGREEMENT_LOCAL_SIZE).
inline constexpr std::uint32_t kAgreementLocalSize = 256;
// agreement_stats.comp's workgroup (VC_STATS_LOCAL_SIZE).
inline constexpr std::uint32_t kStatsLocalSize = 64;
static_assert(quality::kMaxAgreementBins == 4096,
              "mirrors VC_AGREEMENT_MAX_BINS in shaders/agreement.comp");

// agreement.comp's Pair: one ordered pair's cameras, as the host writes them.
struct PairBlock {
  glm::mat4 to_from{1.0f};
  glm::vec4 k_from{0.0f};
  glm::vec4 k_to{0.0f};
  glm::uvec2 size_from{0u};
  glm::uvec2 size_to{0u};
  glm::vec2 range_from{0.0f};
  glm::vec2 range_to{0.0f};
  std::uint32_t histogram = 0;
  std::uint32_t columns = 0;
  std::uint32_t samples = 0;
  std::uint32_t groups = 0;
};
static_assert(sizeof(PairBlock) == 144 && offsetof(PairBlock, k_from) == 64 &&
                  offsetof(PairBlock, k_to) == 80 &&
                  offsetof(PairBlock, size_from) == 96 &&
                  offsetof(PairBlock, size_to) == 104 &&
                  offsetof(PairBlock, range_from) == 112 &&
                  offsetof(PairBlock, range_to) == 120 &&
                  offsetof(PairBlock, histogram) == 128 &&
                  offsetof(PairBlock, groups) == 140,
              "mirrors Pair in shaders/agreement.comp");

// agreement_stats.comp's Stats: what comes back per pair.
struct StatsBlock {
  std::uint32_t overlap = 0;
  std::uint32_t in_window = 0;
  float signed_median = 0.0f;
  float median_abs = 0.0f;
  float p90_abs = 0.0f;
  std::uint32_t pad[3] = {0, 0, 0};
};
static_assert(sizeof(StatsBlock) == 32 &&
                  offsetof(StatsBlock, signed_median) == 8 &&
                  offsetof(StatsBlock, p90_abs) == 16,
              "mirrors Stats in shaders/agreement_stats.comp");

// agreement.comp's push constants.
struct AgreementParams {
  std::uint32_t pair = 0;
  std::uint32_t half_bins = 0;
  std::uint32_t stride = 0;
  float bin = 0.0f;
  float edge_ratio = 0.0f;
  std::uint32_t alone = 0;
};
static_assert(sizeof(AgreementParams) == 24,
              "mirrors Params in shaders/agreement.comp");

// agreement_stats.comp's push constants.
struct StatsParams {
  std::uint32_t pairs = 0;
  std::uint32_t half_bins = 0;
};
static_assert(sizeof(StatsParams) == 8,
              "mirrors Params in shaders/agreement_stats.comp");

static_assert(std::is_trivially_copyable_v<PairBlock> &&
                  std::is_trivially_copyable_v<StatsBlock>,
              "uploaded and read back as bytes");

}  // namespace volumetric_kit::calib::depth::detail
