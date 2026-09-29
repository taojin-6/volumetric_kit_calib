// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/capture/capture_recorder.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace volumetric_kit::calib::capture {
namespace {

// The per-pixel sums are 32-bit: 255 * 2^24 still fits.
constexpr std::uint32_t kMaxFramesToAverage = 1u << 24;

}  // namespace

Result<CaptureRecorder> CaptureRecorder::create(SessionWriter writer,
                                                RecorderOptions options) {
  if (options.frames_to_average == 0 ||
      options.frames_to_average > kMaxFramesToAverage) {
    return Status::invalid_argument("frames_to_average must be 1 to " +
                                    std::to_string(kMaxFramesToAverage));
  }
  return CaptureRecorder{std::move(writer), options};
}

void CaptureRecorder::arm() noexcept {
  if (armed_) return;
  armed_ = true;
  collected_ = 0;
}

void CaptureRecorder::cancel() noexcept {
  armed_ = false;
  collected_ = 0;
  sums_.clear();
}

Result<std::optional<std::size_t>> CaptureRecorder::on_set(const SetInfo& set,
                                                           IRigSource& source) {
  const std::vector<CameraInfo>& cameras = writer_.session().cameras;
  if (set.present.size() != cameras.size()) {
    return Status::invalid_argument(
        "a set of " + std::to_string(set.present.size()) +
        " cameras for a session of " + std::to_string(cameras.size()));
  }
  if (!armed_) return std::optional<std::size_t>{};
  if (!set.complete()) {
    ++skipped_;
    return std::optional<std::size_t>{};
  }

  if (collected_ == 0) {
    first_trigger_ns_ = set.trigger_ns;
    sums_.assign(cameras.size(), {});
  }
  for (std::size_t i = 0; i < cameras.size(); ++i) {
    Result<LumaImage> image = source.read_luma(i);
    if (!image) {
      cancel();
      return image.status();
    }
    const ImageSize& want = cameras[i].lens->image_size();
    const std::size_t pixels = static_cast<std::size_t>(want.width) *
                               static_cast<std::size_t>(want.height);
    if (image->size.width != want.width || image->size.height != want.height ||
        image->pixels.size() != pixels) {
      cancel();
      return Status::invalid_argument(
          cameras[i].serial + ": the source read a " +
          std::to_string(image->size.width) + "x" +
          std::to_string(image->size.height) + " image, its lens is " +
          std::to_string(want.width) + "x" + std::to_string(want.height));
    }
    std::vector<std::uint32_t>& sum = sums_[i];
    if (sum.empty()) sum.assign(pixels, 0);
    for (std::size_t p = 0; p < pixels; ++p) sum[p] += image->pixels[p];
  }
  ++collected_;
  if (collected_ < options_.frames_to_average) {
    return std::optional<std::size_t>{};
  }

  // Every set is in; write their average, rounded to the nearest level.
  std::vector<LumaImage> images(cameras.size());
  const std::uint32_t n = collected_;
  for (std::size_t i = 0; i < cameras.size(); ++i) {
    images[i].size = cameras[i].lens->image_size();
    images[i].pixels.resize(sums_[i].size());
    for (std::size_t p = 0; p < sums_[i].size(); ++p) {
      images[i].pixels[p] =
          static_cast<std::uint8_t>((sums_[i][p] + n / 2) / n);
    }
  }
  const std::uint64_t trigger_ns = first_trigger_ns_;
  cancel();
  VC_ASSIGN(const std::size_t index, writer_.add(trigger_ns, n, images));
  return std::optional<std::size_t>{index};
}

Status CaptureRecorder::undo() {
  if (armed_) {
    cancel();
    return {};
  }
  return writer_.remove_last();
}

}  // namespace volumetric_kit::calib::capture
