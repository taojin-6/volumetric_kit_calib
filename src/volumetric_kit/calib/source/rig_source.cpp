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

// A V210 picture's words as read (`view`), described as `picture` is, on the
// host: what `v210_to_rgb8` takes.
sensor::YuvImage colour_view(const sensor::PictureView& view,
                             const sensor::YuvImage& picture) {
  sensor::YuvImage out;
  out.layout = sensor::YuvLayout::V210;
  out.host = view.luma;
  out.stride[0] = view.stride;
  out.width = view.width;
  out.height = view.height;
  out.kr = picture.kr;
  out.kb = picture.kb;
  out.full_range = view.full_range;
  out.chroma_location = picture.chroma_location;
  return out;
}

}  // namespace

struct RigSource::Impl {
  // One of the two.
  std::optional<sensor::SensorArray> array;
  std::unique_ptr<sensor::IStereoSensor> stereo;

  std::vector<SourceCamera> cameras;
  const core::Device* gpu = nullptr;  // the pictures', whose queue sets ready
  std::unique_ptr<sensor::LumaPass> pass;
  std::unique_ptr<sensor::PictureReadback> readback;
  // Each set's ready value (PictureSet::ready), set after its luma pass;
  // shared with the sets, which keep it alive while a reader waits on it.
  std::shared_ptr<core::TimelineSemaphore> ready;
  std::uint64_t signalled = 0;  // the last value set on it
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
    gpu = &device;
    VKC_ASSIGN(pass, sensor::LumaPass::create(
                         device, allocator,
                         families.empty() ? nullptr : families.data(),
                         static_cast<std::uint32_t>(families.size())));
    VKC_ASSIGN(readback, sensor::PictureReadback::create(device, allocator));
    VKC_ASSIGN(core::TimelineSemaphore timeline,
               core::TimelineSemaphore::create(device, 0));
    ready = std::make_shared<core::TimelineSemaphore>(std::move(timeline));
    return {};
  }

  // The pass's and the read's fences make no device write visible to another
  // queue: a value set after them on this queue gives a reader there -- the
  // window's copy of the tiles and of the images the pass took over -- a
  // semaphore to wait on that does, as the 3D view's worker sets one after
  // its preparation. Waited for, so it is reached when the set is handed out.
  core::Status set_ready(PictureSet& set) {
    const std::uint64_t value = signalled + 1;
    VKC_ASSIGN(core::Device::PendingSubmit done,
               gpu->submit_pending([](VkCommandBuffer) {}, {},
                                   {{ready.get(), value}}));
    signalled = value;
    VKC_TRY(done.wait());
    set.ready = ready;
    set.ready_value = value;
    return {};
  }

  // `pictures[k]`'s luma made and read back: camera `camera[k]`'s. The host
  // pictures in `host`, by camera; each grey tile's device luma held by the
  // set's owner, its picture in `set`; the set's ready value set after both.
  // Two submits (the pass, then the read) stay: one submit, and NV12 luma
  // images read in place, were measured on 2026-11-03 at under 0.5 ms and 5%
  // a set on the M5 Max and the lab box, below the bar for a change. With
  // `colour`, each V210 picture's words are read whole in the same read:
  // every plane of a luma picture is its luma, so the list reads the same.
  core::Status make_luma(const std::vector<sensor::YuvImage*>& pictures,
                         const std::vector<std::size_t>& camera,
                         const std::vector<bool>& grey, bool colour,
                         PictureSet& set, core::StageMetrics* metrics) {
    host.assign(cameras.size(), HostLuma{});
    VKC_ASSIGN(std::vector<sensor::DeviceLuma> made,
               pass->run(pictures, metrics));
    std::vector<sensor::YuvImage*> list;
    list.reserve(3 * made.size());
    for (sensor::DeviceLuma& luma : made) {
      list.push_back(&luma.full);
      list.push_back(&luma.half);
    }
    // Keyed on the layout, not the driver: a V210 picture's colour is its
    // words, read now while the frame is the driver's. One on the host has
    // none the readback copies, and none is read.
    std::vector<std::size_t> coloured;  // by index into `pictures`
    if (colour) {
      for (std::size_t k = 0; k < pictures.size(); ++k) {
        if (pictures[k]->layout != sensor::YuvLayout::V210 ||
            pictures[k]->device == nullptr) {
          continue;
        }
        list.push_back(pictures[k]);
        coloured.push_back(k);
      }
    }
    core::Result<std::vector<sensor::PictureView>> read =
        readback->read(list,
                       coloured.empty() ? sensor::PictureReadback::Planes::Luma
                                        : sensor::PictureReadback::Planes::All,
                       metrics);
    if (!read.ok()) {
      host.assign(cameras.size(), HostLuma{});
      return read.status();
    }
    const core::Status ready_set = set_ready(set);
    if (!ready_set.ok()) {
      host.assign(cameras.size(), HostLuma{});
      return ready_set;
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
    for (std::size_t c = 0; c < coloured.size(); ++c) {
      const std::size_t k = coloured[c];
      host[camera[k]].colour =
          colour_view(views[2 * made.size() + c], *pictures[k]);
    }
    if (!tiles->luma.empty()) set.owner = std::move(tiles);
    return {};
  }

  core::Result<std::optional<PictureSet>> take_array(
      core::StageMetrics* metrics, bool luma, bool colour) {
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
            VKC_TRY(make_luma(pictures, camera, grey, colour, set, metrics));
          } else {
            host.clear();
          }
          set.rgbd = std::make_shared<const sensor::FrameSet>(
              std::move(polled).value());
          return std::optional<PictureSet>(std::move(set));
        });
  }

  core::Result<std::optional<PictureSet>> take_stereo(
      core::StageMetrics* metrics, bool luma, bool colour) {
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
          set.skew_ns = skew;
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
          VKC_TRY(make_luma({sides[0], sides[1]}, {0, 1}, {true, true}, colour,
                            set, metrics));
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
    core::StageMetrics* metrics, bool luma, bool colour) {
  if (impl_->array) return impl_->take_array(metrics, luma, colour);
  return impl_->take_stereo(metrics, luma, colour);
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
