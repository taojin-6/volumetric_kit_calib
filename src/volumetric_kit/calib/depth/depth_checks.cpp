// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/depth/depth_checks.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "plane_blocks.hpp"
#include "plane_moments_comp.spv.hpp"
#include "plane_solve_comp.spv.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"

namespace volumetric_kit::calib::depth {
namespace {

namespace camera = recon::camera;
using detail::FitBlock;
using detail::MomentsParams;
using detail::RegionBlock;
using detail::SolveParams;

// plane_moments.comp's workgroup (plane_common.glsl's VC_PLANE_LOCAL_SIZE):
// the pixels one of its workgroups takes in a pass.
constexpr std::uint32_t kLocalSize = 256;
// The most workgroups, and so slots of partial sums, one region's dispatch
// takes; past it each thread walks more pixels.
constexpr std::uint32_t kMaxRegionGroups = 1024;

constexpr const char* kCreate = "DepthChecks::create";
constexpr const char* kFit = "fit_planes";

// Refuses a depth image the kernels cannot read, naming `what`; on success
// sets `*bytes` to its depth's size.
core::Status check_image(const DepthImage& image, const std::string& what,
                         VkDeviceSize max_range, VkDeviceSize* bytes) {
  core::Status checked =
      camera::check_camera_model({image.size, image.intrinsics, {}});
  if (!checked.ok()) return std::move(checked).with_context(what);
  // Written so a NaN fails it.
  if (!(image.min_depth_m >= 0.0 && image.max_depth_m > image.min_depth_m) ||
      !std::isfinite(image.max_depth_m)) {
    return core::Status::invalid_argument(
        what + ": the range must be finite, not negative and in order");
  }
  for (const auto& [name, transform] :
       {std::pair{"depth_to_world", &image.depth_to_world},
        std::pair{"depth_to_color", &image.depth_to_color}}) {
    checked = camera::check_rigid(*transform);
    if (!checked.ok()) {
      return std::move(checked).with_context(what + "'s " + name);
    }
  }
  if (image.depth == nullptr) {
    return core::Status::invalid_argument(what + " has no depth buffer");
  }
  *bytes = VkDeviceSize{image.size.width} * image.size.height * sizeof(float);
  const std::string buffer = what + "'s depth";
  VKC_TRY(core::check_storage_buffer_range(buffer.c_str(), *bytes, max_range));
  return core::StorageInput(*image.depth).check(buffer.c_str(), *bytes);
}

quality::DepthPlane plane_of(const FitBlock& fit) noexcept {
  quality::DepthPlane plane;
  plane.valid = fit.valid != 0;
  plane.members = fit.members;
  plane.out_of_range = fit.out_of_range;
  plane.inliers = fit.inliers;
  if (plane.valid) {
    plane.point = camera::Vec3d(fit.point);
    plane.normal = camera::Vec3d(fit.normal);
    plane.rms_m = fit.rms;
  }
  return plane;
}

}  // namespace

core::DeviceRequirements DepthChecks::device_requirements() { return {}; }

DepthChecks::DepthChecks(const core::Device& device,
                         core::Allocator& allocator) noexcept
    : device_(&device), allocator_(&allocator) {}

DepthChecks::~DepthChecks() = default;

core::Result<std::unique_ptr<DepthChecks>> DepthChecks::create(
    const core::Device& device, core::Allocator& allocator) {
  return compute::guarded(
      kCreate, [&]() -> core::Result<std::unique_ptr<DepthChecks>> {
        const auto in_create = [](core::Status status) {
          return std::move(status).with_context(kCreate);
        };
        VKC_TRY(in_create(device.check_enabled(device_requirements())));
        std::unique_ptr<DepthChecks> checks(new DepthChecks(device, allocator));
        VkPushConstantRange moments_push{};
        moments_push.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        moments_push.size = sizeof(MomentsParams);
        VkPushConstantRange solve_push = moments_push;
        solve_push.size = sizeof(SolveParams);
        core::KernelSetBuilder builder(device);
        // Depth, regions, fits, partials; then regions, partials, fits.
        VKC_TRY(in_create(builder.add(
            checks->moments_, "plane_moments", vc_plane_moments_comp_spv,
            vc_plane_moments_comp_spv_size, 4, &moments_push)));
        VKC_TRY(in_create(
            builder.add(checks->solve_, "plane_solve", vc_plane_solve_comp_spv,
                        vc_plane_solve_comp_spv_size, 3, &solve_push)));
        core::Result<core::DescriptorPool> pool = builder.build();
        if (!pool.ok()) return in_create(pool.status());
        checks->pool_ = *std::move(pool);
        checks->max_groups_ =
            device.caps().limits().maxComputeWorkGroupCount[0];
        checks->max_range_ = core::max_storage_buffer_range(device);
        return checks;
      });
}

core::Result<std::vector<quality::DepthPlane>> DepthChecks::fit_planes(
    const std::vector<PlaneRegion>& regions,
    const quality::DepthCheckOptions& options) {
  return compute::guarded(
      kFit, [&]() -> core::Result<std::vector<quality::DepthPlane>> {
        const auto in_fit = [](core::Status status) {
          return std::move(status).with_context(kFit);
        };
        VKC_TRY(in_fit(quality::check_depth_options(options)));
        if (regions.empty()) return std::vector<quality::DepthPlane>{};
        if (regions.size() > max_groups_) {
          return core::Status::invalid_argument(
              std::string(kFit) + ": " + std::to_string(regions.size()) +
              " regions are more than one dispatch can solve (" +
              std::to_string(max_groups_) + ")");
        }
        const auto count = static_cast<std::uint32_t>(regions.size());

        // Check everything, and lay out each region's block and slots,
        // before anything is allocated or recorded.
        std::vector<RegionBlock> blocks(count);
        std::vector<VkDeviceSize> depth_bytes(count);
        std::uint32_t slots = 0;
        for (std::uint32_t r = 0; r < count; ++r) {
          const PlaneRegion& view = regions[r];
          const std::string what = "region " + std::to_string(r);
          VKC_TRY(in_fit(
              check_image(view.image, what, max_range_, &depth_bytes[r])));
          VKC_TRY(in_fit(
              quality::check_board_region(view.region).with_context(what)));
          const DepthImage& image = view.image;
          const camera::PinholeIntrinsics& k = image.intrinsics;
          RegionBlock& block = blocks[r];
          block.depth_to_board = glm::mat4(view.region.depth_to_board);
          block.k = glm::vec4(k.fx, k.fy, k.cx, k.cy);
          block.rect = glm::vec4(view.region.min.x, view.region.min.y,
                                 view.region.max.x, view.region.max.y);
          block.size = glm::uvec2(image.size.width, image.size.height);
          block.range = glm::vec2(image.min_depth_m, image.max_depth_m);
          block.first_partial = slots;
          // The binding range bounds the pixels below 2^30.
          block.groups =
              std::min(core::group_count(image.size.width * image.size.height,
                                         kLocalSize),
                       kMaxRegionGroups);
          slots += block.groups;
        }
        const VkDeviceSize region_bytes =
            VkDeviceSize{count} * sizeof(RegionBlock);
        const VkDeviceSize fit_bytes = VkDeviceSize{count} * sizeof(FitBlock);
        const VkDeviceSize partial_bytes =
            VkDeviceSize{slots} * detail::kPartialBytes;
        const core::Device& device = *device_;
        VKC_TRY(in_fit(core::ensure_device_scratch(
            device, *allocator_, regions_, region_bytes, max_range_,
            "calib.depth_regions")));
        VKC_TRY(in_fit(core::ensure_device_scratch(
            device, *allocator_, partials_, partial_bytes, max_range_,
            "calib.depth_partials")));
        VKC_TRY(in_fit(core::ensure_device_scratch(device, *allocator_, fits_,
                                                   fit_bytes, max_range_,
                                                   "calib.depth_fits")));
        VKC_TRY(in_fit(moment_sets_.reserve(device, moments_, count)));
        for (std::uint32_t r = 0; r < count; ++r) {
          const core::DescriptorSet& set = moment_sets_[r];
          set.write_storage_buffer(0, regions[r].image.depth->handle(), 0,
                                   depth_bytes[r]);
          set.write_storage_buffer(1, regions_.handle(), 0, region_bytes);
          set.write_storage_buffer(2, fits_.handle(), 0, fit_bytes);
          set.write_storage_buffer(3, partials_.handle(), 0, partial_bytes);
        }
        solve_.set.write_storage_buffer(0, regions_.handle(), 0, region_bytes);
        solve_.set.write_storage_buffer(1, partials_.handle(), 0,
                                        partial_bytes);
        solve_.set.write_storage_buffer(2, fits_.handle(), 0, fit_bytes);

        // Both rounds of every region in one submit: each round's moments,
        // region by region, then its planes; then the planes come back.
        core::CommandBatch batch(device, *allocator_);
        VKC_TRY(in_fit(batch.upload(regions_, 0, blocks.data(), region_bytes)));
        SolveParams solve;
        solve.min_inliers = static_cast<std::uint32_t>(std::min<std::size_t>(
            options.min_inliers, std::numeric_limits<std::uint32_t>::max()));
        solve.min_gate = static_cast<float>(options.min_gate_mm / 1000.0);
        solve.sigma_gate = static_cast<float>(options.sigma_gate);
        for (std::uint32_t round = 0; round < 2; ++round) {
          for (std::uint32_t r = 0; r < count; ++r) {
            MomentsParams moments;
            moments.region = r;
            moments.round = round;
            moments.seed = static_cast<float>(options.seed_mm / 1000.0);
            VKC_TRY(in_fit(batch.dispatch(moments_, moment_sets_[r], &moments,
                                          sizeof(moments), blocks[r].groups,
                                          max_groups_)));
          }
          solve.round = round;
          VKC_TRY(in_fit(batch.dispatch(solve_, &solve, sizeof(solve), count,
                                        max_groups_)));
        }
        std::vector<FitBlock> fits(count);
        VKC_TRY(in_fit(batch.readback(fits_, 0, fit_bytes, fits.data())));
        VKC_TRY(in_fit(batch.submit()));

        std::vector<quality::DepthPlane> planes;
        planes.reserve(count);
        for (const FitBlock& fit : fits) planes.push_back(plane_of(fit));
        return planes;
      });
}

}  // namespace volumetric_kit::calib::depth
