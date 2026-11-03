// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/live_view.hpp"

#include <cstddef>
#include <optional>
#include <string_view>
#include <utility>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/gfx/pipelines/color.hpp"

namespace volumetric_kit::calib::app {

core::Status record_snapshot(
    viz::RigMosaic& mosaic, gfx::RetireQueue& retire, VkCommandBuffer cmd,
    const std::shared_ptr<const LiveSnapshot>& snapshot,
    std::uint64_t frame_number) {
  return compute::guarded("record_snapshot", [&]() -> core::Status {
    if (snapshot == nullptr || snapshot->set == nullptr) {
      return core::Status::invalid_argument("record_snapshot: no snapshot");
    }
    const source::PictureSet& set = *snapshot->set;
    if (set.cameras.size() != mosaic.size()) {
      return core::Status::invalid_argument(
          "record_snapshot: a set of " + std::to_string(set.cameras.size()) +
          " pictures for a mosaic of " + std::to_string(mosaic.size()) +
          " tiles");
    }
    std::vector<const sensor::YuvImage*> pictures(set.cameras.size(), nullptr);
    for (std::size_t i = 0; i < pictures.size(); ++i) {
      if (!set.cameras[i].present) continue;
      // Its grey tile; else the decoder's picture, in colour.
      if (set.cameras[i].tile != nullptr) {
        pictures[i] = set.cameras[i].tile;
      } else if (set.rgbd != nullptr && i < set.rgbd->frames.size() &&
                 set.rgbd->frames[i] && set.rgbd->frames[i]->has_color()) {
        pictures[i] = &set.rgbd->frames[i]->color;
      }
    }
    // TODO: put each decoder picture's ready value among the frame's waits
    // (sensor's add_ready_point). The luma pass waits for it, so only a set
    // taken while the 3D view showed, with no pass, is copied without: on
    // NVIDIA's decoders, at the first frame back on the images.
    // Held, not freed: the tiles and the decoder's images go back once
    // nothing holds the set, after the GPU is done with the frame that copied
    // them. Held before the record, whose OutOfMemory may follow copies
    // already recorded.
    retire.push(frame_number, [held = snapshot->set] { (void)held; });
    return mosaic.record(cmd, pictures);
  });
}

core::Result<std::vector<viz::TileRefusal>> refusals_to_say(
    const std::vector<viz::TileRefusal>& refused,
    std::vector<std::string>& said) {
  using Refusals = std::vector<viz::TileRefusal>;
  return compute::guarded("refusals_to_say", [&]() -> core::Result<Refusals> {
    Refusals unsaid;
    for (const viz::TileRefusal& refusal : refused) {
      if (refusal.camera >= said.size()) said.resize(refusal.camera + 1);
      std::string& last = said[refusal.camera];
      if (refusal.status.message() == last) continue;
      last = refusal.status.message();
      unsaid.push_back(refusal);
    }
    return unsaid;
  });
}

core::Result<std::vector<viz::TileOverlay>> tile_overlays(
    const LiveSnapshot& snapshot, const std::vector<std::string>& ids,
    const quality::SessionSummary* session) {
  return compute::guarded(
      "tile_overlays", [&]() -> core::Result<std::vector<viz::TileOverlay>> {
        const std::size_t n = ids.size();
        if (snapshot.detections.size() != n || snapshot.checks.size() != n ||
            (session != nullptr && session->cameras.size() != n)) {
          return core::Status::invalid_argument(
              "tile_overlays: the snapshot, the ids and the session disagree "
              "on the cameras");
        }
        std::vector<viz::TileOverlay> tiles(n);
        for (std::size_t i = 0; i < n; ++i) {
          tiles[i].id = ids[i];
          tiles[i].check = snapshot.checks[i];
          if (!snapshot.detections[i].observation.points.empty() ||
              !snapshot.detections[i].markers.empty()) {
            tiles[i].detection = &snapshot.detections[i];
          }
          if (session != nullptr) tiles[i].session = &session->cameras[i];
        }
        return tiles;
      });
}

core::Result<viz::SetLive> set_live(const source::PictureSet& set) {
  return compute::guarded("set_live", [&]() -> core::Result<viz::SetLive> {
    viz::SetLive live;
    live.spread_ns = set.spread_ns;
    live.skew_ns.reserve(set.cameras.size());
    for (const source::CameraPicture& camera : set.cameras) {
      if (!camera.present) {
        live.skew_ns.emplace_back();
        continue;
      }
      // Two's complement: the difference of two times, either way round.
      live.skew_ns.emplace_back(
          static_cast<std::int64_t>(camera.timestamp_ns - set.timestamp_ns));
    }
    return live;
  });
}

core::Result<std::vector<viz::CameraLive>> camera_lives(
    const LiveSnapshot& snapshot) {
  return compute::guarded(
      "camera_lives", [&]() -> core::Result<std::vector<viz::CameraLive>> {
        const std::size_t n = snapshot.checks.size();
        std::vector<viz::CameraLive> cameras(n);
        for (std::size_t i = 0; i < n; ++i) {
          viz::CameraLive& camera = cameras[i];
          camera.present = snapshot.set != nullptr &&
                           i < snapshot.set->cameras.size() &&
                           snapshot.set->cameras[i].present;
          camera.check = snapshot.checks[i];
          if (i < snapshot.stillness_px.size()) {
            camera.stillness_px = snapshot.stillness_px[i];
          }
          if (i < snapshot.stats.sensors.size()) {
            camera.frames = snapshot.stats.sensors[i];
          }
        }
        return cameras;
      });
}

viz::TagCoverage tag_coverage(const KeptCaptures& kept) noexcept {
  viz::TagCoverage coverage;
  coverage.cameras = kept.cameras.size();
  for (std::size_t c = 0; c < coverage.cameras; ++c) {
    std::size_t views = 0;
    for (const std::shared_ptr<const KeptCapture>& capture : kept.captures) {
      if (capture != nullptr && c < capture->tags.size() && capture->tags[c]) {
        ++views;
      }
    }
    if (views > 0) ++coverage.seen_by;
    coverage.views += views;
  }
  return coverage;
}

core::Result<std::optional<core::TimelinePoint>> record_cloud(
    gfx::pipelines::DepthCloudPipeline& pipeline, gfx::RetireQueue& retire,
    VkCommandBuffer cmd, VkExtent2D extent, std::uint64_t frame_number,
    const std::shared_ptr<const CloudSnapshot>& snapshot,
    const std::vector<viz::SceneCamera>& cameras,
    const viz::CloudStyle& style) {
  return compute::guarded(
      "record_cloud",
      [&]() -> core::Result<std::optional<core::TimelinePoint>> {
        if (snapshot == nullptr) return std::optional<core::TimelinePoint>();
        VKC_ASSIGN(const std::vector<gfx::pipelines::DepthCloudDraw> draws,
                   viz::cloud_draws(snapshot->frames, cameras, style));
        if (draws.empty()) return std::optional<core::TimelinePoint>();
        // Held, not freed: sensor's prep reuses a prepared buffer only once no
        // frame holds it, after the GPU is done with the frame that drew it.
        // Held before the draws are recorded, so an allocation failure cannot
        // leave them recorded with nothing holding their buffers.
        retire.push(frame_number, [held = snapshot] { (void)held; });
        VKC_TRY(
            pipeline.submit(cmd, {extent, frame_number, draws.data(),
                                  static_cast<std::uint32_t>(draws.size())}));
        return std::optional<core::TimelinePoint>(snapshot->ready);
      });
}

bool cloud_live(bool frozen, viz::ScenePlacement placement,
                bool minimized) noexcept {
  return !frozen && placement != viz::ScenePlacement::Hidden && !minimized;
}

viz::SceneCamera scene_camera(const capture::SessionCamera& record,
                              const sensor::camera::CameraModel& lens,
                              std::size_t index, bool reference) {
  viz::SceneCamera camera;
  camera.color = lens;
  camera.depth = record.depth;
  camera.depth_to_color = record.depth_to_color;
  if (reference) camera.color_to_world = sensor::camera::Mat4d(1.0);
  camera.tint = gfx::pipelines::categorical_rgba8(index);
  return camera;
}

core::Status placed_cameras(
    std::vector<viz::SceneCamera>& cameras,
    const std::vector<sensor::camera::Mat4d>& color_to_world) {
  return compute::guarded("placed_cameras", [&]() -> core::Status {
    if (color_to_world.size() != cameras.size()) {
      return core::Status::invalid_argument(
          "placed_cameras: " + std::to_string(color_to_world.size()) +
          " poses for " + std::to_string(cameras.size()) + " cameras");
    }
    for (std::size_t i = 0; i < cameras.size(); ++i) {
      cameras[i].color_to_world = color_to_world[i];
    }
    return {};
  });
}

}  // namespace volumetric_kit::calib::app
