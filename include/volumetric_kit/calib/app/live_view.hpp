// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file live_view.hpp
/// @brief What the window (calib_rig's T0 thread) makes of a live snapshot:
///        its pictures copied into the rig mosaic, and the marks and panels
///        the viz tier draws over and beside it; of the captures kept: the
///        origin tag's coverage; and of a 3D-view snapshot: each camera's
///        points, drawn in place at the rig's poses, which Calibrate's result
///        sets.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/app/capture_worker.hpp"
#include "volumetric_kit/calib/app/cloud_worker.hpp"
#include "volumetric_kit/calib/app/live_worker.hpp"
#include "volumetric_kit/calib/quality/session_checks.hpp"
#include "volumetric_kit/calib/viz/mosaic.hpp"
#include "volumetric_kit/calib/viz/overlay.hpp"
#include "volumetric_kit/calib/viz/panels.hpp"
#include "volumetric_kit/calib/viz/point_cloud.hpp"
#include "volumetric_kit/calib/viz/scene_view.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/core/retire_queue.hpp"
#include "volumetric_kit/gfx/pipelines/depth_cloud_pipeline.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"

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
core::Status record_snapshot(
    viz::RigMosaic& mosaic, gfx::RetireQueue& retire, VkCommandBuffer cmd,
    const std::shared_ptr<const LiveSnapshot>& snapshot,
    std::uint64_t frame_number);

/// @brief Which of the mosaic's refusals the window says: each camera's
///        once, until its message changes, as every set's picture from a
///        camera is refused alike.
/// @param refused  What the last record left out (`viz::RigMosaic::refused`).
/// @param said     Each camera's refusal said last, by camera; grown to fit,
///                 and set for each refusal returned.
/// @return The refusals not said before, in @p refused's order; or
///         `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const auto unsaid, refusals_to_say(mosaic->refused(), said));
/// for (const viz::TileRefusal& refusal : unsaid) show(refusal);
/// @endcode
core::Result<std::vector<viz::TileRefusal>> refusals_to_say(
    const std::vector<viz::TileRefusal>& refused,
    std::vector<std::string>& said);

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
core::Result<std::vector<viz::TileOverlay>> tile_overlays(
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
core::Result<std::vector<viz::CameraLive>> camera_lives(
    const LiveSnapshot& snapshot);

/// @brief How much of the origin tag the captures kept hold, as the session
///        panel shows it (`viz::origin_tag_line`).
/// @param kept  The captures kept (@ref CaptureWorker::kept): a dropped one
///              is not among them.
/// @return The rig's cameras, those that found the tag in a capture kept, and
///         their views of it (@ref KeptCapture::tags).
///
/// @code
/// if (auto kept = captures->kept()) {
///   VKC_ASSIGN(const viz::PanelLine line,
///              viz::origin_tag_line(tag_coverage(*kept)));
/// }
/// @endcode
viz::TagCoverage tag_coverage(const KeptCaptures& kept) noexcept;

/// @brief Record @p snapshot's points, each shown and placed camera's at its
///        pose now (`viz::cloud_draws`), and hold the snapshot in @p retire
///        until frame @p frame_number retires, so its buffers outlive the
///        draws on the GPU.
/// @param pipeline      gfx's depth-cloud pipeline, on the frame loop's
///                      timeline.
/// @param retire        The window's retire queue, on the same timeline.
/// @param cmd           The frame's command buffer, recording, inside its
///                      rendering scope.
/// @param extent        The target's size, in pixels.
/// @param frame_number  The frame's number (`windowing::Frame::number`).
/// @param snapshot      The snapshot; null draws nothing.
/// @param cameras       The rig's cameras: their poses, tints and toggles.
/// @param style         How and where the points are drawn.
/// @return What the frame's submission must wait for at
///         `VK_PIPELINE_STAGE_VERTEX_SHADER_BIT` (@ref CloudSnapshot::ready),
///         already reached; empty when nothing was drawn, and nothing held;
///         what `viz::cloud_draws` refuses, with nothing recorded or held;
///         or what gfx's `DepthCloudPipeline::submit` refuses, with nothing
///         recorded and the snapshot held until the frame retires.
///
/// @code
/// VKC_ASSIGN(const auto wait, record_cloud(*clouds, retire, frame.cmd, extent,
///                                          frame.number, cloud_latest,
///                                          cameras, style));
/// if (wait) {
///   frame.waits.push_back({*wait, VK_PIPELINE_STAGE_VERTEX_SHADER_BIT});
/// }
/// @endcode
core::Result<std::optional<core::TimelinePoint>> record_cloud(
    gfx::pipelines::DepthCloudPipeline& pipeline, gfx::RetireQueue& retire,
    VkCommandBuffer cmd, VkExtent2D extent, std::uint64_t frame_number,
    const std::shared_ptr<const CloudSnapshot>& snapshot,
    const std::vector<viz::SceneCamera>& cameras, const viz::CloudStyle& style);

/// @brief Whether the 3D view's worker prepares the sets offered
///        (@ref CloudWorker::set_live): only while the view is live and can
///        be seen.
/// @param frozen     Whether the view is frozen.
/// @param placement  Where the view sits.
/// @param minimized  Whether the window is minimized, drawing nothing.
/// @return False when frozen, hidden or minimized; true otherwise.
///
/// @code
/// cloud->set_live(cloud_live(frozen, placement, !frame));
/// @endcode
bool cloud_live(bool frozen, viz::ScenePlacement placement,
                bool minimized) noexcept;

/// @brief Sensor @p index's camera in the 3D view before the rig is solved
///        (@ref placed_cameras places them all): its factory models, its tint
///        gfx's @p index-th categorical colour, and placed, at the world's
///        origin, only when it is the reference.
/// @param info       The sensor: its colour model, and its depth model and
///                   factory depth-to-colour extrinsic.
/// @param index      Its index in the rig, which picks its tint.
/// @param reference  Whether it is the rig's reference camera.
/// @return The view's camera, shown.
/// @pre `info.color` holds a model.
///
/// @code
/// for (std::size_t i = 0; i < array.size(); ++i) {
///   cameras.push_back(
///       scene_camera(array.sensor(i).info(), i, i == array.primary()));
/// }
/// @endcode
viz::SceneCamera scene_camera(const recon::sensor::SensorInfo& info,
                              std::size_t index, bool reference);

/// @brief Place every camera of the 3D view at Calibrate's poses; each
///        keeps its models, tint and toggle.
/// @param cameras         The rig's cameras, in rig order.
/// @param color_to_world  Each one's colour frame to the world, in rig order
///                        (`CalibrateResult::color_to_world`).
/// @return OK; or `InvalidArgument`, with no camera moved, for as many poses
///         as cameras missing or extra; or `OutOfMemory`.
///
/// @code
/// if (auto result = calibrate->take_result()) {
///   VKC_TRY(placed_cameras(view_cameras, result->color_to_world));
/// }
/// @endcode
core::Status placed_cameras(
    std::vector<viz::SceneCamera>& cameras,
    const std::vector<recon::camera::Mat4d>& color_to_world);

}  // namespace volumetric_kit::calib::app
