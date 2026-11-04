// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file rig_source.hpp
/// @brief Every camera calib reads, as one source of synced 8-bit grey
///        pictures with camera ids: a hardware-synced RGB-D array (the Femto
///        rig) or one stereo sensor (a DeckLink pair).
///
/// Public for the app and the tests, but outside README's stable API: it
/// carries sensor's driver-side types (`SensorArray`, `IStereoSensor`,
/// `YuvImage`, `FrameSet`), and it has one user, `calib_rig`
/// (DECISIONS.md, 2026-11-02). It names no driver: the application opens the
/// sensors and hands them over.

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/detect/luma.hpp"
#include "volumetric_kit/calib/source/export.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/stereo/stereo_sensor.hpp"
#include "volumetric_kit/sensor/yuv_image.hpp"

namespace volumetric_kit::core {
class Allocator;
class Device;
}  // namespace volumetric_kit::core

namespace volumetric_kit::calib::source {

/// @brief One camera of a source, known at open.
///
/// @code
/// for (const SourceCamera& camera : rig->cameras()) {
///   sizes.push_back(camera.size);  // the mosaic's tiles
/// }
/// @endcode
struct SourceCamera {
  std::string id;  ///< Its calibration key: a Femto's serial, `"bm-1"`.
  sensor::camera::ImageSize size;  ///< Its pictures' size.
  /// The device's own lens at that size (a Femto's colour lens); empty for a
  /// stereo sensor's cameras, whose lenses only calib makes.
  std::optional<sensor::camera::CameraModel> factory;
  /// Its sets carry depth (@ref PictureSet::rgbd): the 3D view and the depth
  /// checks.
  bool depth = false;
};

/// @brief One camera's picture in a set, as kept: on the device.
///
/// @code
/// const CameraPicture& picture = set.cameras[i];
/// if (picture.tile != nullptr) grey_tile(*picture.tile);
/// @endcode
struct CameraPicture {
  /// The set has this camera's picture; an array's set may miss a
  /// secondary's.
  bool present = false;
  /// Its 8-bit luma on the device (`YuvLayout::Luma` in a buffer), for the
  /// live view's grey tile; null where the decoder's NV12 images give a
  /// colour tile (@ref PictureSet::rgbd's picture), and for a set taken
  /// without luma. Held by @ref PictureSet::owner.
  const sensor::YuvImage* tile = nullptr;
  std::uint64_t timestamp_ns = 0;  ///< Its capture time.
};

/// @brief One synced set. May be kept and shared: it holds what it points
///        to, and nothing on the host that the next take() reuses.
///
/// @code
/// std::optional<PictureSet> set = rig->take().value();
/// if (set && set->complete()) keep = std::make_shared<PictureSet>(*set);
/// @endcode
struct PictureSet {
  /// The set's number as its source counts: an array's primary frame's
  /// sequence, a stereo frame's.
  std::uint64_t sequence = 0;
  /// The set's time: an array's trigger (its primary's frame's), a stereo
  /// pair's left picture's.
  std::uint64_t timestamp_ns = 0;
  /// The latest picture's time less the earliest's.
  std::uint64_t spread_ns = 0;
  std::vector<CameraPicture> cameras;  ///< In `RigSource::cameras` order.
  /// The RGB-D set it came from (JPEG, depth, the decoder's pictures); null
  /// for a stereo source.
  std::shared_ptr<const sensor::FrameSet> rgbd;
  /// Holds every device picture a tile points to. Let it go only once every
  /// device read of them has completed: the source writes them again after.
  std::shared_ptr<const void> owner;
  /// Set by a take with luma: the source's timeline, at @ref ready_value
  /// once the luma pass is done with the set's pictures on the device -- its
  /// grey tiles written, the decoder's images taken over. The source sets it
  /// on its device's queue after the pass, and it is reached when take()
  /// hands the set out. A reader on another queue (the window's copy into
  /// the mosaic) waits for it on the device, as the pass's fence makes no
  /// device write visible there. Null for a set taken without luma, whose
  /// pictures no pass took over.
  std::shared_ptr<const core::TimelineSemaphore> ready;
  std::uint64_t ready_value = 0;  ///< See @ref ready.

  /// @return How many cameras' pictures the set holds.
  std::size_t count() const noexcept {
    std::size_t n = 0;
    for (const CameraPicture& camera : cameras) n += camera.present ? 1 : 0;
    return n;
  }
  /// @return `true` when every camera's picture is here.
  bool complete() const noexcept { return count() == cameras.size(); }
};

/// @brief One camera's picture on the host from the last take(): valid until
///        the next take(); copy what you keep.
///
/// @code
/// const HostLuma& picture = rig->host()[i];
/// VKC_ASSIGN(Detection found,
///            detect::detect_board(picture.luma, board, {&picture.half}));
/// @endcode
struct HostLuma {
  /// 8-bit, full size; no pixels where the set has no picture of this
  /// camera.
  detect::LumaView luma;
  /// 2 x 2 means of @ref luma, rounded, made on the GPU: `(w / 2) x (h / 2)`,
  /// its pixel u at 2u + 0.5 there (`detect::DetectOptions::half`).
  detect::LumaView half;
  bool full_range = true;  ///< Y in 0..255; false: 16..235 (V210).
};

/// @brief A stereo sensor and its two cameras' ids, left then right, as the
///        opener names them.
///
/// @code
/// StereoMember pair{std::move(decklink), {"bm-1", "bm-2"}};
/// pair.sizes = {{{3840, 2160}, {3840, 2160}}};  // opened without a lens
/// @endcode
struct StereoMember {
  std::unique_ptr<sensor::IStereoSensor> sensor;  ///< The pair; not null.
  std::array<std::string, 2> ids;  ///< Left's, right's; distinct, not empty.
  /// Each camera's picture size, as the opener opened it: needed for a
  /// sensor opened without a calibration, whose `StereoInfo::calibration` is
  /// 0 x 0; 0 x 0 here takes the calibration's.
  std::array<sensor::camera::ImageSize, 2> sizes{};
};

/// @brief Every camera calib reads, as one source of synced 8-bit grey
///        pictures.
///
/// Each take() polls -- the array's next set, or the stereo sensor's newest
/// pair -- makes every picture's luma and its half in one GPU pass (sensor's
/// `LumaPass`, any layout a driver hands out), reads them back in one submit
/// (`PictureReadback`'s list read), and sets the set's ready value after them
/// on the device's queue (@ref PictureSet::ready). The set handed out lives on
/// the device; @ref host has its luma on the host until the next take. Not
/// thread-safe.
///
/// A camera whose picture is NV12 images (VideoToolbox's, NVDEC's) shows the
/// decoder's picture as a colour tile; any other picture -- nvJPEG's planes
/// in a buffer, a host YUYV, V210 -- shows its luma as a grey tile.
///
/// @warning The device and allocator must outlive the source and every set
///          it hands out. Let every set go before the source is destroyed,
///          so the frames go back to their sensors before they close.
///
/// @code
/// VKC_ASSIGN(std::unique_ptr<RigSource> rig,
///            RigSource::open(std::move(array), device, allocator));
/// VKC_TRY(rig->start());
/// VKC_ASSIGN(std::optional<PictureSet> set, rig->take());
/// if (set) detect_each(rig->host());  // a caller's own
/// @endcode
class VC_SOURCE_API RigSource {
 public:
  /// @brief A hardware-synced RGB-D array (the Femto rig): its sets, as the
  ///        array groups them.
  /// @param array      The array, opened and not started; taken whatever the
  ///                   outcome.
  /// @param device     The device its pictures are on; it enabled sensor's
  ///                   requirements.
  /// @param allocator  Its allocator, for the luma and the readback.
  /// @param families   The queue families that read the device luma (the
  ///                   renderer's beside @p device's), which makes it
  ///                   CONCURRENT; empty: @p device's alone.
  /// @return The source; `InvalidArgument` for an empty array or a sensor
  ///         without a colour camera (naming it); what `LumaPass::create`,
  ///         `PictureReadback::create` or the ready timeline's
  ///         `TimelineSemaphore::create` refuses; or `OutOfMemory`.
  static core::Result<std::unique_ptr<RigSource>> open(
      sensor::SensorArray&& array, const core::Device& device,
      core::Allocator& allocator,
      const std::vector<std::uint32_t>& families = {});

  /// @brief One stereo sensor (a DeckLink pair): its pairs, as its driver
  ///        syncs them.
  /// @param member     The sensor, opened and not started, and its cameras'
  ///                   ids and sizes; taken whatever the outcome.
  /// @param device     As the other overload.
  /// @param allocator  As the other overload.
  /// @param families   As the other overload.
  /// @return The source; `InvalidArgument` for no sensor, an empty id or the
  ///         same id twice, or a camera with no size from the member or the
  ///         calibration; what the other overload returns otherwise.
  static core::Result<std::unique_ptr<RigSource>> open(
      StereoMember member, const core::Device& device,
      core::Allocator& allocator,
      const std::vector<std::uint32_t>& families = {});

  /// @brief Stops the sensors, then closes them.
  ~RigSource();
  RigSource(const RigSource&) = delete;
  RigSource& operator=(const RigSource&) = delete;
  RigSource(RigSource&&) = delete;
  RigSource& operator=(RigSource&&) = delete;

  /// @return Each camera, in the order every set gives them.
  const std::vector<SourceCamera>& cameras() const noexcept;

  /// @brief Start the sensors. Idempotent.
  /// @return OK; or the sensors' failure.
  core::Status start();
  /// @brief Stop the sensors and drop what they hold. Idempotent.
  void stop() noexcept;

  /// @brief The newest set not yet taken, its luma made and read back.
  /// @param metrics  Optional: `"poll"`, then, with @p luma, sensor's
  ///                 `"luma"` and `"readback"` rows.
  /// @param luma     Whether to make and read back its luma. False while no
  ///                 one looks at it (the live view showing the 3D view):
  ///                 the set then has no tiles and no ready value, and
  ///                 @ref host no pictures.
  /// @return The set; empty when none is new or the source is not started;
  ///         the sensors' failure (a camera disconnected), after which
  ///         @ref failed is true; or the luma's, the readback's or the ready
  ///         value's failure, the set dropped and the source going on. What a
  ///         driver throws leaves take() as it leaves sensor's own polls.
  core::Result<std::optional<PictureSet>> take(
      core::StageMetrics* metrics = nullptr, bool luma = true);

  /// @return Whether the last take() failed to poll the sensors, rather
  ///         than to make or read back a set's luma.
  bool failed() const noexcept;

  /// @return The last take()'s host pictures, in cameras() order
  ///         (@ref HostLuma); none before a take with luma.
  const std::vector<HostLuma>& host() const noexcept;

  /// @return The counters since the last start: an array's own; for a
  ///         stereo sensor, the pairs handed out as `sets`, and the sensor's
  ///         counters for both cameras, which share them.
  sensor::SensorArrayStats stats() const;

  /// @return The source's own sync bound, nanoseconds: a stereo sensor's
  ///         `StereoInfo::max_skew_ns`, past which it hands out no pair;
  ///         empty for an array, which bounds no spread.
  std::optional<std::uint64_t> max_skew_ns() const noexcept;

 private:
  struct Impl;
  explicit RigSource(std::unique_ptr<Impl> impl) noexcept;
  std::unique_ptr<Impl> impl_;
};

}  // namespace volumetric_kit::calib::source
