// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/source/rig_source.hpp"

#include <algorithm>
#include <string>
#include <utility>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/sensor/prep/luma_pass.hpp"
#include "volumetric_kit/sensor/prep/picture_readback.hpp"

namespace volumetric_kit::calib::source {
namespace {

std::string size_text(std::uint32_t width, std::uint32_t height) {
  return std::to_string(width) + "x" + std::to_string(height);
}

// The decoder's NV12 images, which the live view copies as a colour tile.
bool colour_tile(const sensor::YuvImage& picture) {
  return picture.layout == sensor::YuvLayout::Nv12 &&
         picture.image[0] != nullptr && picture.image[1] != nullptr;
}

// The device luma a set's grey tiles point to, held for as long as the set.
struct Tiles {
  std::vector<sensor::DeviceLuma> luma;
};

detect::LumaView luma_view(const sensor::PictureView& view) {
  return {view.width, view.height, view.stride, view.luma};
}

}  // namespace

struct RigSource::Impl {
  // One of the two.
  std::optional<sensor::SensorArray> array;
  std::unique_ptr<sensor::IStereoSensor> stereo;

  std::vector<SourceCamera> cameras;
  std::unique_ptr<sensor::LumaPass> pass;
  std::unique_ptr<sensor::PictureReadback> readback;
  std::vector<HostLuma> host;
  bool failed = false;
  std::uint64_t pairs = 0;  // a stereo source's pairs handed out since start

  ~Impl() {
    if (array) array->stop();
    if (stereo) stereo->stop();
  }

  core::Status make_tools(const core::Device& device,
                          core::Allocator& allocator,
                          const std::vector<std::uint32_t>& families) {
    VKC_ASSIGN(pass, sensor::LumaPass::create(
                         device, allocator,
                         families.empty() ? nullptr : families.data(),
                         static_cast<std::uint32_t>(families.size())));
    VKC_ASSIGN(readback, sensor::PictureReadback::create(device, allocator));
    return {};
  }

  // `pictures[k]`'s luma made and read back: camera `camera[k]`'s. The host
  // pictures in `host`, by camera; each grey tile's device luma held by the
  // set's owner, its picture in `set`.
  // TODO: one submit for the pass and the read, with NV12 luma images read in
  // place, once sensor's LumaPass offers them.
  core::Status make_luma(const std::vector<sensor::YuvImage*>& pictures,
                         const std::vector<std::size_t>& camera,
                         const std::vector<bool>& grey, PictureSet& set,
                         core::StageMetrics* metrics) {
    host.assign(cameras.size(), HostLuma{});
    VKC_ASSIGN(std::vector<sensor::DeviceLuma> made,
               pass->run(pictures, metrics));
    std::vector<sensor::YuvImage*> list;
    list.reserve(2 * made.size());
    for (sensor::DeviceLuma& luma : made) {
      list.push_back(&luma.full);
      list.push_back(&luma.half);
    }
    core::Result<std::vector<sensor::PictureView>> read =
        readback->read(list, sensor::PictureReadback::Planes::Luma, metrics);
    if (!read.ok()) {
      host.assign(cameras.size(), HostLuma{});
      return read.status();
    }
    const std::vector<sensor::PictureView>& views = read.value();
    auto tiles = std::make_shared<Tiles>();
    tiles->luma.reserve(made.size());
    for (std::size_t k = 0; k < made.size(); ++k) {
      HostLuma& out = host[camera[k]];
      out.luma = luma_view(views[2 * k]);
      out.half = luma_view(views[2 * k + 1]);
      out.full_range = views[2 * k].full_range;
      if (!grey[k]) continue;  // its colour tile is the decoder's picture
      tiles->luma.push_back(std::move(made[k]));
      set.cameras[camera[k]].tile = &tiles->luma.back().full;
    }
    if (!tiles->luma.empty()) set.owner = std::move(tiles);
    return {};
  }

  core::Result<std::optional<PictureSet>> take_array(
      core::StageMetrics* metrics, bool luma) {
    std::optional<sensor::FrameSet> polled;
    {
      const core::StageScope scope(metrics, "poll");
      core::Result<std::optional<sensor::FrameSet>> next = array->poll_set();
      if (!next.ok()) {
        failed = true;
        return next.status();
      }
      polled = std::move(next).value();
    }
    failed = false;
    if (!polled) return std::optional<PictureSet>();
    return compute::guarded(
        "RigSource::take", [&]() -> core::Result<std::optional<PictureSet>> {
          PictureSet set;
          set.sequence = polled->sequence;
          set.timestamp_ns = polled->timestamp_ns;
          set.spread_ns = polled->spread_ns();
          set.cameras.resize(cameras.size());
          std::vector<sensor::YuvImage*> pictures;
          std::vector<std::size_t> camera;
          std::vector<bool> grey;
          for (std::size_t i = 0; i < polled->frames.size(); ++i) {
            std::optional<sensor::RgbdFrame>& frame = polled->frames[i];
            if (!frame) continue;
            set.cameras[i].present = true;
            set.cameras[i].timestamp_ns = frame->timestamp_ns;
            if (!luma) continue;
            sensor::YuvImage& picture = frame->color;
            if (picture.width == 0 || picture.height == 0) {
              return core::Status::invalid_argument("camera " + cameras[i].id +
                                                    "'s frame has no picture");
            }
            pictures.push_back(&picture);
            camera.push_back(i);
            grey.push_back(!colour_tile(picture));
          }
          if (luma) {
            // The pass takes each picture's queue family over; the set keeps
            // the pictures as it left them, for the tiles and the 3D view.
            VKC_TRY(make_luma(pictures, camera, grey, set, metrics));
          } else {
            host.clear();
          }
          set.rgbd = std::make_shared<const sensor::FrameSet>(
              std::move(polled).value());
          return std::optional<PictureSet>(std::move(set));
        });
  }

  core::Result<std::optional<PictureSet>> take_stereo(
      core::StageMetrics* metrics, bool luma) {
    std::optional<sensor::StereoFrame> polled;
    {
      const core::StageScope scope(metrics, "poll");
      core::Result<std::optional<sensor::StereoFrame>> next = stereo->poll();
      if (!next.ok()) {
        failed = true;
        return next.status();
      }
      polled = std::move(next).value();
    }
    failed = false;
    if (!polled) return std::optional<PictureSet>();
    return compute::guarded(
        "RigSource::take", [&]() -> core::Result<std::optional<PictureSet>> {
          sensor::StereoFrame& frame = *polled;
          PictureSet set;
          set.sequence = frame.sequence;
          set.timestamp_ns = frame.timestamp_ns;
          set.cameras.resize(2);
          set.cameras[0] = {true, nullptr, frame.timestamp_ns};
          const std::int64_t skew = frame.skew_ns;
          const std::uint64_t magnitude =
              skew < 0 ? std::uint64_t{0} - static_cast<std::uint64_t>(skew)
                       : static_cast<std::uint64_t>(skew);
          set.cameras[1] = {
              true, nullptr,
              frame.timestamp_ns + static_cast<std::uint64_t>(skew)};
          set.spread_ns = magnitude;
          if (!luma) {
            host.clear();
            ++pairs;
            return std::optional<PictureSet>(std::move(set));
          }
          sensor::YuvImage* sides[2] = {&frame.left, &frame.right};
          for (std::size_t i = 0; i < 2; ++i) {
            const sensor::camera::ImageSize& size = cameras[i].size;
            if (sides[i]->width != size.width ||
                sides[i]->height != size.height) {
              return core::Status::invalid_argument(
                  "camera " + cameras[i].id + " sent a " +
                  size_text(sides[i]->width, sides[i]->height) +
                  " picture, not its " + size_text(size.width, size.height));
            }
          }
          VKC_TRY(make_luma({sides[0], sides[1]}, {0, 1}, {true, true}, set,
                            metrics));
          ++pairs;
          return std::optional<PictureSet>(std::move(set));
        });
  }
};

RigSource::RigSource(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}

RigSource::~RigSource() = default;

core::Result<std::unique_ptr<RigSource>> RigSource::open(
    sensor::SensorArray&& array, const core::Device& device,
    core::Allocator& allocator, const std::vector<std::uint32_t>& families) {
  auto impl = std::make_unique<Impl>();
  impl->array.emplace(std::move(array));
  return compute::guarded(
      "RigSource::open", [&]() -> core::Result<std::unique_ptr<RigSource>> {
        sensor::SensorArray& taken = *impl->array;
        if (taken.size() == 0) {
          return core::Status::invalid_argument("RigSource: no cameras");
        }
        for (std::size_t i = 0; i < taken.size(); ++i) {
          const sensor::SensorInfo& info = taken.sensor(i).info();
          if (!info.color) {
            return core::Status::invalid_argument(
                "RigSource: camera " + info.id + " has no colour model");
          }
          SourceCamera camera;
          camera.id = info.id;
          camera.size = info.color->size;
          camera.factory = info.color;
          camera.depth = info.depth.has_value();
          impl->cameras.push_back(std::move(camera));
        }
        VKC_TRY(impl->make_tools(device, allocator, families));
        return std::unique_ptr<RigSource>(new RigSource(std::move(impl)));
      });
}

core::Result<std::unique_ptr<RigSource>> RigSource::open(
    StereoMember member, const core::Device& device, core::Allocator& allocator,
    const std::vector<std::uint32_t>& families) {
  auto impl = std::make_unique<Impl>();
  impl->stereo = std::move(member.sensor);
  return compute::guarded(
      "RigSource::open", [&]() -> core::Result<std::unique_ptr<RigSource>> {
        if (impl->stereo == nullptr) {
          return core::Status::invalid_argument("RigSource: no stereo sensor");
        }
        const std::array<std::string, 2>& ids = member.ids;
        if (ids[0].empty() || ids[1].empty()) {
          return core::Status::invalid_argument(
              "RigSource: a stereo camera's id is empty");
        }
        if (ids[0] == ids[1]) {
          return core::Status::invalid_argument("RigSource: camera " + ids[0] +
                                                " is named twice");
        }
        const sensor::StereoCalibration& pair =
            impl->stereo->info().calibration;
        const sensor::camera::ImageSize lensed[2] = {pair.left.size,
                                                     pair.right.size};
        for (std::size_t i = 0; i < 2; ++i) {
          SourceCamera camera;
          camera.id = ids[i];
          camera.size = member.sizes[i];
          if (camera.size.width == 0 || camera.size.height == 0) {
            camera.size = lensed[i];
          }
          if (camera.size.width == 0 || camera.size.height == 0) {
            return core::Status::invalid_argument(
                "RigSource: camera " + ids[i] +
                " has no size: the sensor was opened without a calibration");
          }
          impl->cameras.push_back(std::move(camera));
        }
        VKC_TRY(impl->make_tools(device, allocator, families));
        return std::unique_ptr<RigSource>(new RigSource(std::move(impl)));
      });
}

const std::vector<SourceCamera>& RigSource::cameras() const noexcept {
  return impl_->cameras;
}

core::Status RigSource::start() {
  impl_->failed = false;
  if (impl_->array) return impl_->array->start();
  VKC_TRY(impl_->stereo->start());
  impl_->pairs = 0;
  return {};
}

void RigSource::stop() noexcept {
  if (impl_->array) impl_->array->stop();
  if (impl_->stereo) impl_->stereo->stop();
}

core::Result<std::optional<PictureSet>> RigSource::take(
    core::StageMetrics* metrics, bool luma) {
  if (impl_->array) return impl_->take_array(metrics, luma);
  return impl_->take_stereo(metrics, luma);
}

bool RigSource::failed() const noexcept { return impl_->failed; }

const std::vector<HostLuma>& RigSource::host() const noexcept {
  return impl_->host;
}

sensor::SensorArrayStats RigSource::stats() const {
  if (impl_->array) return impl_->array->stats();
  sensor::SensorArrayStats stats;
  stats.sets = impl_->pairs;
  const sensor::SensorStats shared = impl_->stereo->stats();
  stats.sensors = {shared, shared};
  return stats;
}

std::optional<std::uint64_t> RigSource::max_skew_ns() const noexcept {
  if (impl_->stereo) return impl_->stereo->info().max_skew_ns;
  return std::nullopt;
}

}  // namespace volumetric_kit::calib::source
