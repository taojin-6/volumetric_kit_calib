// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file rig_source.hpp
/// @brief The capture contract: a rig of cameras, polled for sets of frames
///        that one sync trigger produced, whose pixels are read on demand.
///
/// A set says only which cameras delivered a frame for the trigger; the pixels
/// stay where the source keeps them (a live rig keeps them on the GPU) until
/// @ref IRigSource::read_luma asks for one camera's. Calibration reads few
/// frames at full resolution -- a capture's -- so this contract never makes a
/// source copy a frame nobody reads.
///
/// The image read is the camera's luma as captured, before undistortion, and
/// @ref CameraInfo::lens is the factory model of that image. Detection and the
/// solve then see the lens directly rather than an image resampled through it.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/calib/capture/export.hpp"
#include "volumetric_kit/calib/core/camera_model.hpp"
#include "volumetric_kit/calib/core/result.hpp"

namespace volumetric_kit::calib::capture {

/// @brief A camera's role in a hardware-synchronised rig.
enum class SyncRole {
  Standalone,  ///< Not synchronised: streams on its own clock.
  Primary,     ///< Drives the sync line; the rig's reference camera.
  Secondary,   ///< Captures on the primary's trigger.
};

/// @return A stable lowercase name for @p role (`"standalone"`, `"primary"`,
///         `"secondary"`), as sessions store it.
VC_CAPTURE_API const char* to_string(SyncRole role) noexcept;

/// @brief One camera of a rig, as its source describes it.
///
/// @code
/// for (const CameraInfo& camera : source.cameras()) {
///   std::printf("%s (%s)\n", camera.serial.c_str(), to_string(camera.role));
/// }
/// @endcode
struct CameraInfo {
  std::string serial;  ///< Stable identifier; names the camera's files.
  SyncRole role = SyncRole::Standalone;  ///< Its role in the rig.
  /// The factory model of the image @ref IRigSource::read_luma returns: its
  /// size, intrinsics and distortion. Set once @ref IRigSource::start has
  /// returned OK -- a live camera reports its lens with its first frame -- and
  /// empty before.
  std::optional<PinholeBrownConrady> lens;
};

/// @return The index of the one @ref SyncRole::Primary camera in @p cameras;
///         empty when there is none or more than one.
VC_CAPTURE_API std::optional<std::size_t> find_primary(
    const std::vector<CameraInfo>& cameras) noexcept;

/// @brief The frames one trigger produced across the rig.
///
/// @code
/// if (set.complete()) recorder_input.push_back(set);  // every camera is here
/// @endcode
struct SetInfo {
  /// The trigger's time on the rig's clock (ns); zero when the source has
  /// none.
  std::uint64_t trigger_ns = 0;
  /// Per camera, in @ref IRigSource::cameras order: whether its frame for
  /// this trigger arrived.
  std::vector<bool> present;

  /// @return How many cameras delivered a frame.
  std::size_t count() const noexcept;
  /// @return `true` when every camera delivered one (and there is at least
  ///         one camera).
  bool complete() const noexcept;
};

/// @brief An 8-bit single-channel image the caller owns: a camera's luma.
///
/// @code
/// const std::uint8_t centre =
///     image.pixels[(image.size.height / 2) * image.size.width +
///                  image.size.width / 2];
/// @endcode
struct LumaImage {
  ImageSize size;  ///< Width and height (pixels).
  /// Row-major, `size.width * size.height` bytes, rows tightly packed.
  std::vector<std::uint8_t> pixels;

  /// @return `true` when the image has no pixels.
  bool empty() const noexcept { return pixels.empty(); }
};

/// @brief A source of synchronised sets from a rig of cameras.
///
/// Polled, not called back, and dropping rather than queueing: a consumer
/// slower than the cameras gets the newest set, as recon's `ICameraCapture`
/// does for one camera. @ref poll_set says which cameras delivered;
/// @ref read_luma fetches one camera's pixels from the set last polled.
///
/// @code
/// VC_TRY(source.start());
/// for (;;) {
///   VC_ASSIGN(const std::optional<SetInfo> set, source.poll_set());
///   if (!set) {
///     if (source.exhausted()) break;  // a replay's end
///     continue;                       // a live rig's "not yet"
///   }
///   if (set->complete()) {
///     VC_ASSIGN(const LumaImage primary, source.read_luma(0));
///   }
/// }
/// @endcode
///
/// @warning Not thread-safe unless an implementation says otherwise. Start,
///          poll, read and stop from one thread.
class VC_CAPTURE_API IRigSource {
 public:
  virtual ~IRigSource() = default;

  IRigSource(const IRigSource&) = delete;
  IRigSource& operator=(const IRigSource&) = delete;

  /// @return The rig's cameras, in the order sets report them. Each one's
  ///         @ref CameraInfo::lens is set once @ref start has returned OK.
  virtual const std::vector<CameraInfo>& cameras() const noexcept = 0;

  /// @brief Begin producing sets. Idempotent: starting a running source is OK.
  /// @return OK once running (every camera's lens then known), or why the
  ///         source could not start.
  virtual Status start() = 0;

  /// @brief Stop producing sets. Idempotent, and safe on a source that never
  ///        started.
  virtual void stop() noexcept = 0;

  /// @brief Take the newest set not yet returned.
  /// @return The set; an empty optional (@ref no_set) when none is ready or
  ///         the source is not started -- not an error; or a device failure.
  virtual Result<std::optional<SetInfo>> poll_set() = 0;

  /// @brief Read camera @p camera's luma from the set @ref poll_set last
  ///        returned, at the size of its @ref CameraInfo::lens.
  /// @param camera  An index into @ref cameras.
  /// @return The image; @ref Status::Code::InvalidArgument when no set has
  ///         been polled since @ref start, @p camera is out of range, or that
  ///         camera is absent from the set; or a read failure.
  virtual Result<LumaImage> read_luma(std::size_t camera) = 0;

  /// @brief Whether this source will never hand out another set.
  ///
  /// An empty @ref poll_set means only "nothing this tick"; a finite source,
  /// such as a replay, overrides this to say its sequence is over. A live rig
  /// keeps the default: it is never exhausted, only stopped.
  /// @return `true` once no further @ref poll_set can return a set.
  virtual bool exhausted() const noexcept { return false; }

  /// @brief The "no set this tick" return. `return std::nullopt;` needs two
  ///        user-defined conversions and does not compile, and
  ///        `return Status{};` aborts (@ref Result refuses an OK status), so
  ///        implementations return this.
  /// @return An OK @ref Result holding an empty optional.
  static Result<std::optional<SetInfo>> no_set() {
    return std::optional<SetInfo>{};
  }

  /// @brief The "here is the set" return, the counterpart to @ref no_set.
  /// @param set  The set to hand over.
  /// @return An OK @ref Result holding @p set.
  static Result<std::optional<SetInfo>> some_set(SetInfo set) {
    return std::optional<SetInfo>{std::move(set)};
  }

 protected:
  IRigSource() = default;
  // Move operations stay available to implementations but are not part of the
  // polymorphic interface: callers hold a source behind a pointer.
  IRigSource(IRigSource&&) = default;
  IRigSource& operator=(IRigSource&&) = default;
};

}  // namespace volumetric_kit::calib::capture
