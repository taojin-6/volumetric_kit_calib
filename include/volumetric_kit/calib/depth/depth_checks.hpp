// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file depth_checks.hpp
/// @brief The depth checks' device side: each view's board plane fitted on
///        the GPU to its camera's depth, read where recon's frame preparation
///        left it.
///
/// The quality tier's `depth_checks.hpp` is the host side: the region each
/// fit takes, and what the planes say of the solved rig. Here runs the dense
/// work alone -- every depth pixel of each region classified, gated and
/// summed -- with a few numbers per region coming back. Any number of cameras
/// and regions.

#include <cstdint>
#include <memory>
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
  /// The depth camera in the solved rig's world; rigid.
  recon::camera::Mat4d depth_to_world{1.0};
  /// The depth camera's frame to its colour camera's (recon's
  /// `SensorInfo::depth_to_color`), which `quality::board_region` takes;
  /// rigid.
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

// TODO: Outliers on one side within the second round's gate -- fingers 10 to
// 25 mm in front of a fifth of the board -- inflate the first round's RMS, so
// the second keeps them and the plane moves by their share of their mean
// offset (3.5 mm there). Revisit the second gate's scale, e.g. one robust to
// them from a histogram of the first round's residuals, with a real session
// (H5).
/// @brief The depth checks' kernels on a device shared with recon: each
///        view's board plane fitted to its camera's depth, read in place.
///
/// A fit has two rounds. The first takes the region's points within
/// `seed_mm` of the solved board's plane; the second, those within
/// `max(min_gate_mm, sigma_gate rms)` of the first round's plane. Each round
/// fits the least-squares plane in the region's board frame, from moments
/// summed in a fixed order, so the same depth gives the same plane to the
/// bit. Both rounds of every region run in one submit, and 48 bytes per
/// region come back. Not thread-safe.
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

 private:
  DepthChecks(const core::Device& device, core::Allocator& allocator) noexcept;

  const core::Device* device_;
  core::Allocator* allocator_;
  core::ComputeKernel moments_;  // plane_moments.comp, a set per region
  core::ComputeKernel solve_;    // plane_solve.comp
  core::DescriptorPool pool_;    // the two kernels' own sets
  core::KernelSets moment_sets_;
  core::Buffer regions_;   // each region's block
  core::Buffer partials_;  // each region's workgroups' sums
  core::Buffer fits_;      // each region's plane, read back
  std::uint32_t max_groups_ = 0;
  VkDeviceSize max_range_ = 0;
};

}  // namespace volumetric_kit::calib::depth
