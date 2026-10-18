// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/app/live_view.hpp"

#include <cstddef>
#include <optional>
#include <string_view>
#include <utility>

#include "volumetric_kit/calib/compute/guarded.hpp"

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

}  // namespace volumetric_kit::calib::app
