// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/depth/depth_checks.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "agreement_blocks.hpp"
#include "agreement_comp.spv.hpp"
#include "agreement_stats_comp.spv.hpp"
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
using detail::AgreementParams;
using detail::FitBlock;
using detail::MomentsParams;
using detail::PairBlock;
using detail::RegionBlock;
using detail::SolveParams;
using detail::StatsBlock;
using detail::StatsParams;

// plane_moments.comp's workgroup (plane_common.glsl's VC_PLANE_LOCAL_SIZE):
// the pixels one of its workgroups takes in a pass.
constexpr std::uint32_t kLocalSize = 256;
// The most workgroups, and so slots of partial sums, one region's dispatch
// takes; past it each thread walks more pixels.
constexpr std::uint32_t kMaxRegionGroups = 1024;

// The most workgroups one pair's agreement dispatch takes: each clears and
// flushes a histogram, so past this each thread walks more samples instead.
constexpr std::uint32_t kMaxPairGroups = 512;

constexpr const char* kCreate = "DepthChecks::create";
constexpr const char* kFit = "fit_planes";
constexpr const char* kHistograms = "histograms";
constexpr const char* kAccumulate = "accumulate";
constexpr const char* kSummarize = "summarize";

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

// The ordered pairs of `cameras`, by `from`, then `to`: pair (from, to)'s
// histogram.
std::size_t pair_count(std::size_t cameras) noexcept {
  return cameras * (cameras - 1);
}
std::uint32_t histogram_of(std::size_t cameras, std::size_t from,
                           std::size_t to) noexcept {
  return static_cast<std::uint32_t>(from * (cameras - 1) +
                                    (to < from ? to : to - 1));
}

// ceil(a / b), b > 0, without the overflow of (a + b - 1) / b.
std::uint32_t ceil_div(std::uint32_t a, std::uint32_t b) noexcept {
  return a / b + (a % b != 0 ? 1 : 0);
}

// The pixels plane_moments walks for `region` in an image of `size` and
// pinhole `k`: the box round the inset rectangle's corners projected, a pixel
// wider each side for float's rounding, clamped to the image -- none when it
// lies outside -- or the whole image when a corner is not in front of the
// camera. Every member's ray meets the rectangle in front of the camera, so
// projects inside it.
std::pair<glm::uvec2, glm::uvec2> region_pixels(
    const quality::BoardRegion& region, const camera::PinholeIntrinsics& k,
    const camera::ImageSize& size) {
  const glm::uvec2 whole(size.width, size.height);
  const camera::Mat4d board_to_depth =
      camera::rigid_inverse(region.depth_to_board);
  camera::Vec2d lo(std::numeric_limits<double>::infinity());
  camera::Vec2d hi(-std::numeric_limits<double>::infinity());
  for (const camera::Vec2d& corner :
       {region.min, camera::Vec2d(region.max.x, region.min.y), region.max,
        camera::Vec2d(region.min.x, region.max.y)}) {
    const glm::dvec4 p = board_to_depth * glm::dvec4(corner, 0.0, 1.0);
    if (!(p.z > 0.0)) return {glm::uvec2(0u), whole};
    const camera::Vec2d at(k.fx * p.x / p.z + k.cx, k.fy * p.y / p.z + k.cy);
    lo = glm::min(lo, at);
    hi = glm::max(hi, at);
  }
  glm::uvec2 origin(0u);
  glm::uvec2 extent(0u);
  for (int i = 0; i < 2; ++i) {
    const double last = static_cast<double>(whole[i]) - 1.0;
    const double first = std::max(std::floor(lo[i]) - 1.0, 0.0);
    const double end = std::min(std::ceil(hi[i]) + 1.0, last);
    // Written so a NaN fails it.
    if (!(first <= end)) return {glm::uvec2(0u), glm::uvec2(0u)};
    origin[i] = static_cast<std::uint32_t>(first);
    extent[i] = static_cast<std::uint32_t>(end - first) + 1u;
  }
  return {origin, extent};
}

// Whether two histograms' counts are binned alike: the same window, bins,
// edge gate and stride.
bool binned_alike(const quality::DepthCheckOptions& a,
                  const quality::DepthCheckOptions& b) noexcept {
  return a.window_mm == b.window_mm && a.bin_mm == b.bin_mm &&
         a.edge_ratio == b.edge_ratio &&
         a.agreement_stride == b.agreement_stride;
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

AgreementHistograms::AgreementHistograms(
    const core::Device& device, std::size_t cameras,
    const quality::DepthCheckOptions& options) noexcept
    : device_(&device), cameras_(cameras), options_(options) {}

AgreementHistograms::~AgreementHistograms() = default;

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
        VkPushConstantRange agreement_push = moments_push;
        agreement_push.size = sizeof(AgreementParams);
        VkPushConstantRange stats_push = moments_push;
        stats_push.size = sizeof(StatsParams);
        core::KernelSetBuilder builder(device);
        // Depth, regions, fits, partials; then regions, partials, fits.
        VKC_TRY(in_create(builder.add(
            checks->moments_, "plane_moments", vc_plane_moments_comp_spv,
            vc_plane_moments_comp_spv_size, 4, &moments_push)));
        VKC_TRY(in_create(
            builder.add(checks->solve_, "plane_solve", vc_plane_solve_comp_spv,
                        vc_plane_solve_comp_spv_size, 3, &solve_push)));
        // From's depth, to's, pairs, bins, overlaps, a capture's own bins and
        // overlaps; then bins, overlaps, statistics.
        VKC_TRY(in_create(
            builder.add(checks->agreement_, "agreement", vc_agreement_comp_spv,
                        vc_agreement_comp_spv_size, 7, &agreement_push)));
        VKC_TRY(in_create(builder.add(
            checks->stats_, "agreement_stats", vc_agreement_stats_comp_spv,
            vc_agreement_stats_comp_spv_size, 3, &stats_push)));
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
          std::tie(block.origin, block.extent) =
              region_pixels(view.region, k, image.size);
          // The binding range bounds the pixels below 2^30. A slot at least,
          // which a rectangle of none leaves empty.
          block.groups = std::clamp(
              core::group_count(block.extent.x * block.extent.y, kLocalSize),
              1u, kMaxRegionGroups);
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

core::Result<std::unique_ptr<AgreementHistograms>> DepthChecks::histograms(
    std::size_t cameras, const quality::DepthCheckOptions& options) {
  return compute::guarded(
      kHistograms, [&]() -> core::Result<std::unique_ptr<AgreementHistograms>> {
        const auto in_histograms = [](core::Status status) {
          return std::move(status).with_context(kHistograms);
        };
        VKC_TRY(in_histograms(quality::check_depth_options(options)));
        if (cameras == 0) {
          return core::Status::invalid_argument(std::string(kHistograms) +
                                                ": no cameras");
        }
        // Whole, and at most kMaxAgreementBins / 2, by check_depth_options.
        const auto half = static_cast<std::uint32_t>(
            std::lround(options.window_mm / options.bin_mm));
        const VkDeviceSize pair_bytes =
            VkDeviceSize{2} * half * sizeof(std::uint32_t);
        // Every pair's bins, and its block for accumulate, in one binding --
        // the block is larger than its statistics and its overlap, the other
        // bindings a pair has -- and a stats invocation each in one
        // dispatch; checked as cameras - 1 <= most / cameras, which cannot
        // overflow.
        static_assert(sizeof(PairBlock) >= sizeof(StatsBlock));
        const VkDeviceSize most = std::min(
            max_range_ / std::max<VkDeviceSize>(pair_bytes, sizeof(PairBlock)),
            VkDeviceSize{max_groups_} * detail::kStatsLocalSize);
        if (cameras - 1 > most / cameras) {
          return core::Status::invalid_argument(
              std::string(kHistograms) + ": " + std::to_string(cameras) +
              " cameras make more pairs than one binding holds or one "
              "dispatch summarizes (" +
              std::to_string(most) + ")");
        }
        const std::size_t pairs = pair_count(cameras);
        std::unique_ptr<AgreementHistograms> out(
            new AgreementHistograms(*device_, cameras, options));
        out->half_bins_ = half;
        out->samples_.assign(pairs, 0);
        if (pairs == 0) return out;
        const VkDeviceSize bin_bytes = pairs * pair_bytes;
        const VkDeviceSize overlap_bytes = pairs * sizeof(std::uint32_t);
        core::Result<core::Buffer> bins =
            core::device_storage_buffer(*allocator_, bin_bytes);
        if (!bins.ok()) return in_histograms(bins.status());
        out->bins_ = *std::move(bins);
        core::Result<core::Buffer> overlaps =
            core::device_storage_buffer(*allocator_, overlap_bytes);
        if (!overlaps.ok()) return in_histograms(overlaps.status());
        out->overlaps_ = *std::move(overlaps);
        core::CommandBatch batch(*device_, *allocator_);
        VKC_TRY(in_histograms(batch.fill(out->bins_, 0, bin_bytes, 0)));
        VKC_TRY(in_histograms(batch.fill(out->overlaps_, 0, overlap_bytes, 0)));
        VKC_TRY(in_histograms(batch.submit()));
        return out;
      });
}

core::Status DepthChecks::accumulate(
    AgreementHistograms& histograms,
    const std::vector<std::optional<DepthImage>>& images,
    AgreementHistograms* alone) {
  return compute::guarded(kAccumulate, [&]() -> core::Status {
    const auto in_accumulate = [](core::Status status) {
      return std::move(status).with_context(kAccumulate);
    };
    if (histograms.device_ != device_) {
      return core::Status::invalid_argument(
          std::string(kAccumulate) + ": the histograms are of another device");
    }
    const std::size_t cameras = histograms.cameras_;
    if (alone != nullptr &&
        (alone == &histograms || alone->device_ != device_ ||
         alone->cameras_ != cameras ||
         !binned_alike(alone->options_, histograms.options_))) {
      return core::Status::invalid_argument(
          std::string(kAccumulate) +
          ": the capture's own histograms must be others of this device, "
          "with the same cameras, window, bins, edge gate and stride");
    }
    if (images.size() != cameras) {
      return core::Status::invalid_argument(
          std::string(kAccumulate) + ": " + std::to_string(images.size()) +
          " image slots for " + std::to_string(cameras) + " cameras");
    }

    // Check everything, and lay out each pair's block, before anything is
    // allocated, recorded or counted.
    std::vector<VkDeviceSize> depth_bytes(cameras, 0);
    for (std::size_t c = 0; c < cameras; ++c) {
      if (!images[c]) continue;
      VKC_TRY(
          in_accumulate(check_image(*images[c], "camera " + std::to_string(c),
                                    max_range_, &depth_bytes[c])));
    }
    const quality::DepthCheckOptions& options = histograms.options_;
    const std::uint32_t stride = options.agreement_stride;
    std::vector<PairBlock> blocks;
    std::vector<std::pair<std::size_t, std::size_t>> measured;
    for (std::size_t from = 0; from < cameras; ++from) {
      for (std::size_t to = 0; to < cameras; ++to) {
        if (from == to || !images[from] || !images[to]) continue;
        const DepthImage& a = *images[from];
        const DepthImage& b = *images[to];
        PairBlock block;
        block.to_from = glm::mat4(camera::rigid_inverse(b.depth_to_world) *
                                  a.depth_to_world);
        block.k_from = glm::vec4(a.intrinsics.fx, a.intrinsics.fy,
                                 a.intrinsics.cx, a.intrinsics.cy);
        block.k_to = glm::vec4(b.intrinsics.fx, b.intrinsics.fy,
                               b.intrinsics.cx, b.intrinsics.cy);
        block.size_from = glm::uvec2(a.size.width, a.size.height);
        block.size_to = glm::uvec2(b.size.width, b.size.height);
        block.range_from = glm::vec2(a.min_depth_m, a.max_depth_m);
        block.range_to = glm::vec2(b.min_depth_m, b.max_depth_m);
        block.histogram = histogram_of(cameras, from, to);
        // The binding range bounds the pixels below 2^30.
        block.columns = ceil_div(a.size.width, stride);
        block.samples = block.columns * ceil_div(a.size.height, stride);
        block.groups = std::min(
            core::group_count(block.samples, detail::kAgreementLocalSize),
            kMaxPairGroups);
        if (histograms.samples_[block.histogram] + block.samples >
            std::numeric_limits<std::uint32_t>::max()) {
          return core::Status::invalid_argument(
              std::string(kAccumulate) + ": the pair from camera " +
              std::to_string(from) + " to camera " + std::to_string(to) +
              " could pass 2^32 - 1 samples, as many as its counts hold");
        }
        blocks.push_back(block);
        measured.emplace_back(from, to);
      }
    }
    const core::Device& device = *device_;
    core::CommandBatch batch(device, *allocator_);
    if (alone != nullptr && alone->bins_.size() > 0) {
      VKC_TRY(
          in_accumulate(batch.fill(alone->bins_, 0, alone->bins_.size(), 0)));
      VKC_TRY(in_accumulate(
          batch.fill(alone->overlaps_, 0, alone->overlaps_.size(), 0)));
    }
    // Each pair's bound on the capture's own counts, once they are emptied
    // and this capture counted.
    const auto count_alone = [&] {
      if (alone == nullptr) return;
      std::fill(alone->samples_.begin(), alone->samples_.end(), 0);
      for (const PairBlock& block : blocks) {
        alone->samples_[block.histogram] = block.samples;
      }
    };
    if (blocks.empty()) {
      VKC_TRY(in_accumulate(batch.submit()));
      count_alone();
      return core::Status{};
    }
    const auto count = static_cast<std::uint32_t>(blocks.size());

    const VkDeviceSize pair_bytes = VkDeviceSize{count} * sizeof(PairBlock);
    const VkDeviceSize bin_bytes = histograms.bins_.size();
    const VkDeviceSize overlap_bytes = histograms.overlaps_.size();
    VKC_TRY(in_accumulate(
        core::ensure_device_scratch(device, *allocator_, pairs_, pair_bytes,
                                    max_range_, "calib.depth_pairs")));
    VKC_TRY(in_accumulate(pair_sets_.reserve(device, agreement_, count)));
    // Without a capture's own histograms, the pooled ones are bound in their
    // place, and not written there.
    const AgreementHistograms& own = alone != nullptr ? *alone : histograms;
    for (std::uint32_t i = 0; i < count; ++i) {
      const auto [from, to] = measured[i];
      const core::DescriptorSet& set = pair_sets_[i];
      set.write_storage_buffer(0, images[from]->depth->handle(), 0,
                               depth_bytes[from]);
      set.write_storage_buffer(1, images[to]->depth->handle(), 0,
                               depth_bytes[to]);
      set.write_storage_buffer(2, pairs_.handle(), 0, pair_bytes);
      set.write_storage_buffer(3, histograms.bins_.handle(), 0, bin_bytes);
      set.write_storage_buffer(4, histograms.overlaps_.handle(), 0,
                               overlap_bytes);
      set.write_storage_buffer(5, own.bins_.handle(), 0, bin_bytes);
      set.write_storage_buffer(6, own.overlaps_.handle(), 0, overlap_bytes);
    }

    VKC_TRY(in_accumulate(batch.upload(pairs_, 0, blocks.data(), pair_bytes)));
    AgreementParams params;
    params.half_bins = histograms.half_bins_;
    params.stride = stride;
    params.bin = static_cast<float>(options.bin_mm / 1000.0);
    params.edge_ratio = static_cast<float>(options.edge_ratio);
    params.alone = alone != nullptr ? 1u : 0u;
    for (std::uint32_t i = 0; i < count; ++i) {
      params.pair = i;
      VKC_TRY(in_accumulate(batch.dispatch(agreement_, pair_sets_[i], &params,
                                           sizeof(params), blocks[i].groups,
                                           max_groups_)));
    }
    // Counted before the submit, so a failure leaves a bound still above
    // whatever the device added.
    for (const PairBlock& block : blocks) {
      histograms.samples_[block.histogram] += block.samples;
      if (alone != nullptr) alone->samples_[block.histogram] += block.samples;
    }
    VKC_TRY(in_accumulate(batch.submit()));
    count_alone();
    return core::Status{};
  });
}

core::Result<std::vector<quality::PairAgreement>> DepthChecks::summarize(
    const AgreementHistograms& histograms,
    const std::optional<std::int64_t>& capture) {
  return compute::guarded(
      kSummarize, [&]() -> core::Result<std::vector<quality::PairAgreement>> {
        const auto in_summarize = [](core::Status status) {
          return std::move(status).with_context(kSummarize);
        };
        if (histograms.device_ != device_) {
          return core::Status::invalid_argument(
              std::string(kSummarize) +
              ": the histograms are of another device");
        }
        const std::size_t cameras = histograms.cameras_;
        const auto pairs = static_cast<std::uint32_t>(pair_count(cameras));
        std::vector<StatsBlock> stats(pairs);
        if (pairs > 0) {
          const VkDeviceSize stats_bytes =
              VkDeviceSize{pairs} * sizeof(StatsBlock);
          const core::Device& device = *device_;
          VKC_TRY(in_summarize(core::ensure_device_scratch(
              device, *allocator_, summary_, stats_bytes, max_range_,
              "calib.depth_agreement")));
          stats_.set.write_storage_buffer(0, histograms.bins_.handle(), 0,
                                          histograms.bins_.size());
          stats_.set.write_storage_buffer(1, histograms.overlaps_.handle(), 0,
                                          histograms.overlaps_.size());
          stats_.set.write_storage_buffer(2, summary_.handle(), 0, stats_bytes);
          StatsParams params;
          params.pairs = pairs;
          params.half_bins = histograms.half_bins_;
          core::CommandBatch batch(device, *allocator_);
          VKC_TRY(in_summarize(batch.dispatch(
              stats_, &params, sizeof(params),
              core::group_count(pairs, detail::kStatsLocalSize), max_groups_)));
          VKC_TRY(in_summarize(
              batch.readback(summary_, 0, stats_bytes, stats.data())));
          VKC_TRY(in_summarize(batch.submit()));
        }

        const double bin_mm = histograms.options_.bin_mm;
        std::vector<quality::PairAgreement> out;
        out.reserve(pairs);
        for (std::size_t from = 0; from < cameras; ++from) {
          for (std::size_t to = 0; to < cameras; ++to) {
            if (from == to) continue;
            const StatsBlock& s = stats[histogram_of(cameras, from, to)];
            quality::PairAgreement pair;
            pair.from = from;
            pair.to = to;
            pair.capture = capture;
            pair.overlap = s.overlap;
            pair.in_window = s.in_window;
            if (s.in_window > 0) {
              pair.signed_median_mm = s.signed_median * bin_mm;
              pair.median_abs_mm = s.median_abs * bin_mm;
              pair.p90_abs_mm = s.p90_abs * bin_mm;
            }
            out.push_back(pair);
          }
        }
        return out;
      });
}

}  // namespace volumetric_kit::calib::depth
