// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file live_view.hpp
/// @brief What the window (calib_rig's T0 thread) makes of a live snapshot:
///        its pictures copied into the rig mosaic, and the marks and panels
///        the viz tier draws over and beside it.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "volumetric_kit/calib/app/export.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"
#include "volumetric_kit/calib/viz/mosaic.hpp"
#include "volumetric_kit/calib/viz/overlay.hpp"
#include "volumetric_kit/calib/viz/panels.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/core/retire_queue.hpp"

namespace volumetric_kit::calib::app {

/// @brief Record @p snapshot's pictures into @p mosaic, and hold its set in
///        @p retire until frame @p frame_number retires, so the decoder's
///        images the copies read outlive them on the GPU.
/// @param mosaic        The rig's mosaic, one tile per camera.
/// @param retire        The window's retire queue, on its frame loop's
///                      timeline.
/// @param cmd           The frame's command buffer, recording, outside its
///                      rendering scope, on the renderer's graphics queue.
/// @param snapshot      The snapshot; each camera's picture with colour is
///                      written into its tile, a missing one keeps its last.
/// @param frame_number  The frame's number (`windowing::Frame::number`).
/// @return OK, with what the mosaic left out in its `refused`;
///         `InvalidArgument` for a null snapshot or one whose set has a frame
///         per camera missing or extra, with nothing recorded or held; the
///         mosaic's `OutOfMemory`.
///
/// @code
/// if (fresh) {
///   VKC_TRY(record_snapshot(*mosaic, retire, frame.cmd, latest,
///                           frame.number));
/// }
/// @endcode
VC_APP_API core::Status record_snapshot(
    viz::RigMosaic& mosaic, gfx::RetireQueue& retire, VkCommandBuffer cmd,
    const std::shared_ptr<const LiveSnapshot>& snapshot,
    std::uint64_t frame_number);

/// @brief What each tile of the mosaic shows over its picture.
/// @param snapshot  The snapshot.
/// @param ids       Each camera's id, which the overlays name; it must
///                  outlive them.
/// @param session   The session's checks, for each camera's coverage grid;
///                  null before the first capture.
/// @return One overlay per camera, pointing into @p snapshot and
///         @p session, which must outlive it; `InvalidArgument` when the
///         snapshot, @p ids and @p session disagree on the cameras; or
///         `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const auto tiles, tile_overlays(*latest, live->ids(), summary));
/// for (std::size_t i = 0; i < tiles.size(); ++i) {
///   VKC_ASSIGN(const viz::TileMarks marks, viz::tile_marks(tiles[i]));
///   viz::draw_tile_marks(mosaic->view(), mosaic->tile_region(i), marks);
/// }
/// @endcode
VC_APP_API core::Result<std::vector<viz::TileOverlay>> tile_overlays(
    const LiveSnapshot& snapshot, const std::vector<std::string>& ids,
    const quality::SessionSummary* session);

/// @brief What the camera panels show of each camera in @p snapshot.
/// @param snapshot  The snapshot.
/// @return One per camera, in rig order: its frame's presence, its view's
///         check, its stillness, its counters and lost frames; or
///         `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const auto cameras, camera_lives(*latest));
/// VKC_ASSIGN(const auto lines, viz::camera_panel(cameras[i], summary_i,
///                                                board));
/// @endcode
VC_APP_API core::Result<std::vector<viz::CameraLive>> camera_lives(
    const LiveSnapshot& snapshot);

}  // namespace volumetric_kit::calib::app
