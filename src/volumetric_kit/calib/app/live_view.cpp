// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/live_view.hpp"

#include <cstddef>
#include <optional>
#include <string_view>
#include <utility>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/gfx/pipelines/color.hpp"

namespace volumetric_kit::calib::app {

namespace sensor = recon::sensor;

core::Status record_snapshot(
    viz::RigMosaic& mosaic, gfx::RetireQueue& retire, VkCommandBuffer cmd,
    const std::shared_ptr<const LiveSnapshot>& snapshot,
    std::uint64_t frame_number) {
  return compute::guarded("record_snapshot", [&]() -> core::Status {
    if (snapshot == nullptr || snapshot->set == nullptr) {
      return core::Status::invalid_argument("record_snapshot: no snapshot");
    }
    const sensor::FrameSet& set = *snapshot->set;
    if (set.frames.size() != mosaic.size()) {
      return core::Status::invalid_argument(
          "record_snapshot: a set of " + std::to_string(set.frames.size()) +
          " frames for a mosaic of " + std::to_string(mosaic.size()) +
          " tiles");
    }
    std::vector<const sensor::YuvImage*> pictures(set.frames.size(), nullptr);
    for (std::size_t i = 0; i < pictures.size(); ++i) {
      if (set.frames[i] && set.frames[i]->has_color()) {
        pictures[i] = &set.frames[i]->color;
      }
    }
    // Held, not freed: the decoder's images go back once nothing holds the
    // set, after the GPU is done with the frame that copied them. Held before
    // the record, whose OutOfMemory may follow copies already recorded.
    retire.push(frame_number, [held = snapshot->set] { (void)held; });
    return mosaic.record(cmd, pictures);
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

core::Result<std::vector<viz::CameraLive>> camera_lives(
    const LiveSnapshot& snapshot) {
  return compute::guarded(
      "camera_lives", [&]() -> core::Result<std::vector<viz::CameraLive>> {
        const std::size_t n = snapshot.checks.size();
        std::vector<viz::CameraLive> cameras(n);
        for (std::size_t i = 0; i < n; ++i) {
          viz::CameraLive& camera = cameras[i];
          camera.present = snapshot.set != nullptr &&
                           i < snapshot.set->frames.size() &&
                           snapshot.set->frames[i].has_value();
          camera.check = snapshot.checks[i];
          if (i < snapshot.stillness_px.size()) {
            camera.stillness_px = snapshot.stillness_px[i];
          }
          if (i < snapshot.stats.sensors.size()) {
            camera.frames = snapshot.stats.sensors[i];
          }
          if (i < snapshot.lost.size()) camera.lost = snapshot.lost[i];
        }
        return cameras;
      });
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
        // Held, not freed: recon reuses a prepared buffer only once no frame
        // holds it, after the GPU is done with the frame that drew it. Held
        // before the draws are recorded, so an allocation failure cannot
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

viz::SceneCamera scene_camera(const sensor::SensorInfo& info, std::size_t index,
                              bool reference) {
  VKC_CHECK(info.color.has_value(), "scene_camera: the sensor has no colour");
  viz::SceneCamera camera;
  camera.color = *info.color;
  camera.depth = info.depth;
  camera.depth_to_color = info.depth_to_color;
  // TODO: CA12b places every camera at Calibrate's solved poses.
  if (reference) camera.color_to_world = recon::camera::Mat4d(1.0);
  camera.tint = gfx::pipelines::categorical_rgba8(index);
  return camera;
}

}  // namespace volumetric_kit::calib::app
