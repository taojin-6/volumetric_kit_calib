// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file depth_checks.hpp
/// @brief The depth checks' device side: each view's board plane fitted on
///        the GPU to its camera's depth, and each ordered pair of cameras'
///        depth compared, read where recon's frame preparation left it.
///
/// The quality tier's `depth_checks.hpp` is the host side: the region each
/// fit takes, and what the planes and pairs say of the solved rig. Here runs
/// the dense work alone -- every depth pixel of each region classified, gated
/// and summed; every pixel of each camera's depth moved into each other
/// camera's and its difference binned -- with a few numbers per region and
/// per pair coming back. Any number of cameras and regions.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "volumetric_kit/calib/depth/export.hpp"
#include "volumetric_kit/calib/quality/depth_checks.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/compute_kernel.hpp"
#include "volumetric_kit/core/vulkan/descriptor.hpp"
#include "volumetric_kit/core/vulkan/device_requirements.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::core {
class Allocator;
class Device;
}  // namespace volumetric_kit::core

namespace volumetric_kit::calib::depth {

/// @brief One camera's prepared depth on the device, and where the camera is:
///        what the checks read of recon's `sensor::DeviceFrame` and the
///        solved rig, so this tier names no sensor type.
///
/// @code
/// const recon::DepthCameraParams& d = frame.depth_camera;  // a DeviceFrame
/// DepthImage image;
/// image.depth = frame.depth.get();
/// image.intrinsics = {d.fx, d.fy, d.cx, d.cy};
/// image.size = {d.width, d.height};
/// image.min_depth_m = d.min_depth;
/// image.max_depth_m = d.max_depth;
/// image.depth_to_color = info.depth_to_color;  // recon's SensorInfo
/// image.depth_to_world = color_to_world * info.depth_to_color;
/// @endcode
struct DepthImage {
  /// Row-major depth in metres, `size.width * size.height` floats from its
  /// start, 0 where there is none: a storage buffer in device-local memory
  /// that the checks' device can read in place, on its queue family (recon's
  /// `GpuFramePrep` on the same device leaves one), its writes complete.
  /// Borrowed for the call.
  const core::Buffer* depth = nullptr;
  /// The depth's pinhole, in its pixels, whose centres are at integers.
  recon::camera::PinholeIntrinsics intrinsics;
  recon::camera::ImageSize size;  ///< The depth image's size.
  /// The camera's range: depth nearer than this, or none, is out of range
  /// (metres); finite and not negative.
  double min_depth_m = 0.0;
  /// And farther than this; finite and above @ref min_depth_m.
  double max_depth_m = 0.0;
  /// The depth camera in the solved rig's world; rigid. `accumulate` reads
  /// it; `fit_planes` checks it but does not read it.
  recon::camera::Mat4d depth_to_world{1.0};
  /// The depth camera's frame to its colour camera's (recon's
  /// `SensorInfo::depth_to_color`), which `quality::board_region` takes;
  /// rigid. The checks check it but do not read it: a region's
  /// `depth_to_board` already holds it.
  recon::camera::Mat4d depth_to_color{1.0};
};

/// @brief One view's plane fit: a camera's depth, and the region of it the
///        fit takes.
///
/// @code
/// PlaneRegion view;
/// view.image = images[c];
/// VKC_ASSIGN(view.region,
///            quality::board_region(board, board_to_color,
///                                  view.image.depth_to_color, options));
/// @endcode
struct PlaneRegion {
  DepthImage image;  ///< The camera's depth.
  /// The pixels the fit takes, in `image`'s depth camera: what
  /// `quality::board_region` gives for its `depth_to_color`.
  quality::BoardRegion region;
};

class DepthChecks;

/// @brief Every ordered pair of a rig's cameras' depth differences, binned
///        on the device and summed over each capture given to
///        @ref DepthChecks::accumulate, which @ref DepthChecks::summarize
///        reads. Made empty by @ref DepthChecks::histograms, with the
///        settings every later call takes.
///
/// The counts are integers, added by integer atomics, so they come out the
/// same in any order: two captures accumulated one after the other leave the
/// histograms one capture holding both would. A pair holds at most
/// 2^32 - 1 samples, about 11,000 captures of 640 x 576 depth at stride 1.
///
/// @code
/// // Every capture pooled, and the chosen capture on its own.
/// VKC_ASSIGN(std::unique_ptr<AgreementHistograms> pooled,
///            checks->histograms(cameras, options));
/// VKC_ASSIGN(std::unique_ptr<AgreementHistograms> chosen,
///            checks->histograms(cameras, options));
/// for (const Capture& capture : captures) {
///   VKC_TRY(checks->accumulate(*pooled, capture.images));
///   if (capture.id == chosen_id) {
///     VKC_TRY(checks->accumulate(*chosen, capture.images));
///   }
/// }
/// VKC_ASSIGN(std::vector<quality::PairAgreement> pairs,
///            checks->summarize(*pooled));
/// VKC_ASSIGN(std::vector<quality::PairAgreement> own,
///            checks->summarize(*chosen, chosen_id));
/// pairs.insert(pairs.end(), own.begin(), own.end());
/// @endcode
class VC_DEPTH_API AgreementHistograms {
 public:
  /// @brief Free the histograms.
  /// @pre No call that reads or writes them is running, and the allocator
  ///      of the checks that made them is alive.
  ~AgreementHistograms();
  AgreementHistograms(const AgreementHistograms&) = delete;
  AgreementHistograms& operator=(const AgreementHistograms&) = delete;
  AgreementHistograms(AgreementHistograms&&) = delete;
  AgreementHistograms& operator=(AgreementHistograms&&) = delete;

  /// @return The rig's cameras: `cameras() (cameras() - 1)` ordered pairs.
  std::size_t cameras() const noexcept { return cameras_; }
  /// @return The settings the histograms were made with.
  const quality::DepthCheckOptions& options() const noexcept {
    return options_;
  }

 private:
  friend class DepthChecks;
  AgreementHistograms(const core::Device& device, std::size_t cameras,
                      const quality::DepthCheckOptions& options) noexcept;

  const core::Device* device_;
  std::size_t cameras_;
  quality::DepthCheckOptions options_;
  std::uint32_t half_bins_ = 0;  // bins either side of 0
  core::Buffer bins_;            // each pair's 2 half_bins_ counts
  core::Buffer overlaps_;        // each pair's overlap
  // Each pair's most samples so far, which bounds every count it holds.
  std::vector<std::uint64_t> samples_;
};

// TODO: Outliers on one side within the second round's gate -- fingers 10 to
// 25 mm in front of one edge's fifth of the board -- inflate the first
// round's RMS, so the second keeps them all: the plane moves by their share of
// their mean offset (3.5 mm there) and turns about 1.4 degrees, past the
// default max_angle_deg, so a hand on one edge in every capture marks a good
// camera Off. (On both sides they cancel.) Revisit the second gate's scale,
// e.g. one robust to them from a histogram of the first round's residuals,
// before the verdicts rely on the planes; the test
// OutliersOnOneSideMoveThePlaneByTheirShare pins today's bias.
/// @brief The depth checks' kernels on a device shared with recon: each
///        view's board plane fitted to its camera's depth, and each ordered
///        pair of cameras' depth compared, read in place.
///
/// A fit has two rounds. The first takes the region's points within
/// `seed_mm` of the solved board's plane; the second, those within
/// `max(min_gate_mm, sigma_gate rms)` of the first round's plane. Each round
/// fits the least-squares plane in the region's board frame, from moments
/// summed in a fixed order, so the same depth gives the same plane to the
/// bit. Both rounds of every region run in one submit, and 48 bytes per
/// region come back. A pair's comparison bins its differences into
/// @ref AgreementHistograms over any number of captures, and 32 bytes per
/// pair come back. Not thread-safe.
///
/// @code
/// VKC_ASSIGN(std::unique_ptr<DepthChecks> checks,
///            DepthChecks::create(device, allocator));
/// VKC_ASSIGN(std::vector<quality::DepthPlane> planes,
///            checks->fit_planes(views, options));
/// VKC_ASSIGN(quality::BoardDepthCheck check,
///            quality::board_depth_check(cameras, board, detections[0],
///                                       views[0].region, planes[0]));
/// @endcode
class VC_DEPTH_API DepthChecks {
 public:
  /// @brief What the checks' kernels need of a device: the core's compute
  ///        baseline -- Vulkan 1.2, a compute queue and timeline semaphores
  ///        -- and nothing more.
  ///
  /// The device a rig shares merges these with recon's and gfx's.
  /// @return The requirements, a default `core::DeviceRequirements`.
  ///
  /// @code
  /// VKC_ASSIGN(core::DeviceRequirements reqs,
  ///            core::merge(recon::device_requirements(),
  ///                        DepthChecks::device_requirements()));
  /// @endcode
  static core::DeviceRequirements device_requirements();

  /// @brief Build the kernels.
  /// @param device     The device recon's frames are prepared on, so their
  ///                   depth is read in place; it enabled
  ///                   @ref device_requirements, and must outlive the
  ///                   checks.
  /// @param allocator  Allocates the checks' device-only scratch; it must
  ///                   outlive them.
  /// @return The checks; `Unsupported` for a device that did not enable
  ///         @ref device_requirements; a kernel's or the descriptor pool's
  ///         failure; or `OutOfMemory`. Each message starts
  ///         `DepthChecks::create: `.
  static core::Result<std::unique_ptr<DepthChecks>> create(
      const core::Device& device, core::Allocator& allocator);

  /// @brief Free the kernels and the scratch.
  /// @pre No call is running.
  ~DepthChecks();
  DepthChecks(const DepthChecks&) = delete;
  DepthChecks& operator=(const DepthChecks&) = delete;
  DepthChecks(DepthChecks&&) = delete;
  DepthChecks& operator=(DepthChecks&&) = delete;

  /// @brief Fit each region's board plane to its camera's depth.
  ///
  /// A member is a pixel whose ray belongs to the region
  /// (`quality::in_board_region`), with depth or without; one whose depth is
  /// missing, not a number or outside the camera's range is out of range.
  /// The depth camera's pose is the region's `depth_to_board`.
  /// A plane is valid with at least `options.min_inliers` inliers that do
  /// not lie on one line. When the first round's plane is not valid, the
  /// second does not run, and the plane gives the first's counts.
  /// @param regions  The fits, any number; several may read one depth.
  /// @param options  The settings, as `quality::board_region` took them.
  /// @return Each region's plane, in its board frame, in the order given
  ///         (none for none); or `InvalidArgument` for options
  ///         `quality::check_depth_options` refuses, a region
  ///         `quality::check_board_region` refuses, or a depth image that
  ///         is empty or not a storage buffer in device-local memory of its
  ///         size, larger than one binding may be, with intrinsics not
  ///         finite or a focal length not positive, a range out of order or
  ///         a transform not rigid, or more regions than one dispatch can
  ///         solve, each naming the region; a device failure; or
  ///         `OutOfMemory`. Each message starts `fit_planes: `.
  core::Result<std::vector<quality::DepthPlane>> fit_planes(
      const std::vector<PlaneRegion>& regions,
      const quality::DepthCheckOptions& options = {});

  /// @brief Empty histograms of every ordered pair of @p cameras cameras'
  ///        depth differences, for @ref accumulate.
  /// @param cameras  The rig's cameras; at least 1 (which pairs none).
  /// @param options  The settings: the agreement's window, bins, edge gate
  ///                 and stride, which every @ref accumulate and
  ///                 @ref summarize of them take, and the rest the report
  ///                 records.
  /// @return The histograms, zeroed on the device; `InvalidArgument` for no
  ///         cameras, options `quality::check_depth_options` refuses, or
  ///         more pairs than one binding may hold or one dispatch
  ///         summarize; a device failure; or `OutOfMemory`. Each message
  ///         starts `histograms: `.
  core::Result<std::unique_ptr<AgreementHistograms>> histograms(
      std::size_t cameras, const quality::DepthCheckOptions& options = {});

  /// @brief Add one capture's depth differences to @p histograms: every
  ///        ordered pair of its cameras with depth, in one submit.
  ///
  /// A pair's samples are the pixels of the moved camera's depth (`from`)
  /// at every `agreement_stride`-th column and row, with depth in its range.
  /// Each moves into the other's depth frame (`to`) by
  /// `to.depth_to_world^-1 from.depth_to_world`, composed in double, and
  /// meets the pixel of `to`'s depth nearest where it projects (recon's
  /// pinhole). It counts in the pair's overlap when that pixel has depth in
  /// range and neither pixel is at an edge (`options.edge_ratio`). Its
  /// difference, its depth in `to`'s frame minus `to`'s depth there, is
  /// binned when inside the window.
  /// @param histograms  Of this device; their settings apply.
  /// @param images      Each camera's depth, in the rig's order, one slot
  ///                    per camera of @p histograms; an empty slot (a camera
  ///                    with no frame in the capture) takes no part. Several
  ///                    may read one buffer.
  /// @return OK; `InvalidArgument` for histograms of another device, a
  ///         slot count that is not their cameras', an image @ref fit_planes
  ///         would refuse, naming its camera, or a capture that could carry
  ///         a pair past 2^32 - 1 samples, each leaving the histograms as
  ///         they were; a device failure, after which their counts are
  ///         unknown; or `OutOfMemory`. Each message starts `accumulate: `.
  core::Status accumulate(AgreementHistograms& histograms,
                          const std::vector<std::optional<DepthImage>>& images);

  /// @brief What each ordered pair's histogram says: its overlap, its
  ///        samples in the window, and their signed median, median
  ///        magnitude and 90th percentile of magnitude.
  ///
  /// The q-quantile of binned samples lies in the bin where their running
  /// count, from the lowest, reaches q of them, as far into it as the rest
  /// of q takes of its count: its samples spread evenly across it. The
  /// magnitudes are the histogram folded at 0. Computed on the device, with
  /// 32 bytes per pair coming back.
  /// @param histograms  Of this device.
  /// @param capture     The one capture they hold, for
  ///                    `PairAgreement::capture`; empty when pooled.
  /// @return Every ordered pair, by `from`, then `to`, a pair never measured
  ///         with an overlap of 0; or `InvalidArgument` for histograms of
  ///         another device; a device failure; or `OutOfMemory`. Each message
  ///         starts `summarize: `.
  core::Result<std::vector<quality::PairAgreement>> summarize(
      const AgreementHistograms& histograms,
      const std::optional<std::int64_t>& capture = std::nullopt);

 private:
  DepthChecks(const core::Device& device, core::Allocator& allocator) noexcept;

  const core::Device* device_;
  core::Allocator* allocator_;
  core::ComputeKernel moments_;    // plane_moments.comp, a set per region
  core::ComputeKernel solve_;      // plane_solve.comp
  core::ComputeKernel agreement_;  // agreement.comp, a set per pair
  core::ComputeKernel stats_;      // agreement_stats.comp
  core::DescriptorPool pool_;      // the four kernels' own sets
  core::KernelSets moment_sets_;
  core::KernelSets pair_sets_;
  core::Buffer regions_;   // each region's block
  core::Buffer partials_;  // each region's workgroups' sums
  core::Buffer fits_;      // each region's plane, read back
  core::Buffer pairs_;     // each pair's block
  core::Buffer summary_;   // each pair's statistics, read back
  std::uint32_t max_groups_ = 0;
  VkDeviceSize max_range_ = 0;
};

}  // namespace volumetric_kit::calib::depth
