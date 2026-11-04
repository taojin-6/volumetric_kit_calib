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
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/core/retire_queue.hpp"
#include "volumetric_kit/gfx/pipelines/depth_cloud_pipeline.hpp"
#include "volumetric_kit/gfx/windowing/frame_loop.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"

namespace volumetric_kit::calib::app {

/// @brief Record @p snapshot's pictures into @p mosaic, and hold its set in
///        @p retire until frame @p frame_number retires, so the pictures the
///        copies read, and the timelines the frame waits on, outlive them on
///        the GPU.
///
/// A picture that carries a ready value (`sensor::YuvImage::ready`, as
/// NVIDIA's decoders hand theirs out), and the set's own, set after its luma
/// pass on the source's queue (`source::PictureSet::ready`), are copied only
/// once each value is reached, and each value is returned for the frame to
/// wait for on the device: gfx's frame loop takes only values already
/// reached, and nothing here waits on the host. A snapshot with a value not
/// reached yet is recorded at a later frame. The window records through
/// record_fresh.
/// @param mosaic        The rig's mosaic, one tile per camera.
/// @param retire        The window's retire queue, on its frame loop's
///                      timeline.
/// @param cmd           The frame's command buffer, recording, outside its
///                      rendering scope, on the renderer's graphics queue.
/// @param snapshot      The snapshot; each camera's picture is written into
///                      its tile -- its grey tile (`source::CameraPicture::
///                      tile`), else its frame's colour -- and a missing one
///                      keeps its last.
/// @param frame_number  The frame's number (`windowing::Frame::number`).
/// @return What the frame's submission must wait for at
///         `VK_PIPELINE_STAGE_TRANSFER_BIT`: the set's ready value and that
///         of each picture handed to the mosaic, one a timeline (sensor's
///         `add_ready_point`), every one already reached; empty when none
///         carries one. What the mosaic left out is in its `refused`.
///         Nothing (`std::nullopt`) while a picture's ready value is not
///         reached, with nothing recorded or held: record the snapshot at a
///         later frame. `InvalidArgument` for a null snapshot or one whose
///         set has a picture per camera missing or extra, or a ready
///         timeline's error when its value cannot be read, with nothing
///         recorded or held; the mosaic's `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const auto ready, record_snapshot(*mosaic, retire, frame.cmd,
///                                              latest, frame.number));
/// if (ready) {  // else record the snapshot at a later frame
///   for (const core::TimelinePoint& point : *ready) {
///     frame.waits.push_back({point, VK_PIPELINE_STAGE_TRANSFER_BIT});
///   }
/// }
/// @endcode
core::Result<std::optional<std::vector<core::TimelinePoint>>> record_snapshot(
    viz::RigMosaic& mosaic, gfx::RetireQueue& retire, VkCommandBuffer cmd,
    const std::shared_ptr<const LiveSnapshot>& snapshot,
    std::uint64_t frame_number);

/// @brief The window's mosaic in a frame: while @p fresh, record @p snapshot
///        (record_snapshot), the frame waiting for each ready value it
///        returns at `VK_PIPELINE_STAGE_TRANSFER_BIT`, the record timed as
///        @p ran's `"tiles"` row, and @p fresh cleared.
///
/// A snapshot with a ready value not reached yet stays fresh, for a later
/// frame, with nothing recorded, held, waited for or timed, so the Stages
/// panel keeps the last record's time (`viz::hold_stages`). A snapshot whose
/// set was taken without luma (no `source::PictureSet::ready`: the 3D view
/// showed) is not recorded at all, and @p fresh is cleared: no luma pass took
/// its pictures over, and NVIDIA's -- NVDEC's images still owned outside
/// Vulkan, nvJPEG's planes in a buffer -- are ones the mosaic refuses. Every
/// tile keeps its last picture until a set taken with luma comes.
/// @param mosaic    The rig's mosaic, one tile per camera.
/// @param retire    The window's retire queue, on its frame loop's timeline.
/// @param frame     The frame, recording, outside its rendering scope; its
///                  `waits` gain the ready values.
/// @param snapshot  The latest snapshot; null records nothing.
/// @param fresh     Whether @p snapshot is still to be recorded: false
///                  records nothing, and a record or a skip clears it.
/// @param ran       This frame's run of the tiles; a record adds its row.
/// @return Whether it recorded @p snapshot, after which the window says the
///         mosaic's new refusals (refusals_to_say); false for one skipped;
///         record_snapshot's errors, with @p fresh kept; or `OutOfMemory`.
///
/// @code
/// core::StageMetrics ran;
/// if (mosaic_shown) {
///   VKC_ASSIGN(const bool recorded,
///              record_fresh(*mosaic, retire, frame, latest, fresh, ran));
///   // ... once recorded, say refusals_to_say(mosaic->refused(), said) ...
/// }
/// VKC_TRY(viz::hold_stages(ran, mosaic_shown, tiles, frame_stages));
/// @endcode
core::Result<bool> record_fresh(
    viz::RigMosaic& mosaic, gfx::RetireQueue& retire,
    gfx::windowing::Frame& frame,
    const std::shared_ptr<const LiveSnapshot>& snapshot, bool& fresh,
    core::StageMetrics& ran);

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

/// @brief What the set panel shows of @p set's times (`viz::set_panel`).
/// @param set  The set.
/// @return Each camera's picture's time less the set's, and its spread; or
///         `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const viz::SetLive set, set_live(*latest->set));
/// VKC_ASSIGN(const auto lines, viz::set_panel(set, latest->stats, ids,
///                                             saver));
/// @endcode
core::Result<viz::SetLive> set_live(const source::PictureSet& set);

/// @brief What the camera panels show of each camera in @p snapshot.
/// @param snapshot  The snapshot.
/// @return One per camera, in rig order: its picture's presence, its view's
///         check, its stillness and its counters; or `OutOfMemory`.
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

/// @brief Camera @p index of the rig in the 3D view before the rig is solved
///        (@ref placed_cameras places them all): its lens, its factory depth
///        model, its tint gfx's @p index-th categorical colour, and placed,
///        at the world's origin, only when it is the reference.
/// @param camera     The camera's record: its depth model and factory
///                   depth-to-colour extrinsic, if any.
/// @param lens       Its lens: the colour camera the view draws.
/// @param index      Its index in the rig, which picks its tint.
/// @param reference  Whether it is the rig's reference camera.
/// @return The view's camera, shown.
///
/// @code
/// for (std::size_t i = 0; i < opened.cameras.size(); ++i) {
///   cameras.push_back(scene_camera(opened.cameras[i], opened.lenses[i], i,
///                                  i == opened.reference));
/// }
/// @endcode
viz::SceneCamera scene_camera(const capture::SessionCamera& camera,
                              const sensor::camera::CameraModel& lens,
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
    const std::vector<sensor::camera::Mat4d>& color_to_world);

}  // namespace volumetric_kit::calib::app
