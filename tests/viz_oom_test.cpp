// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The viz tier runs out of memory anywhere in its models and the 3D view's
// geometry and draws, and returns OutOfMemory, never throwing; a board editor
// is then as it was.

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "allocation_failure.hpp"
#include "volumetric_kit/calib/viz/board_editor.hpp"
#include "volumetric_kit/calib/viz/mosaic.hpp"
#include "volumetric_kit/calib/viz/overlay.hpp"
#include "volumetric_kit/calib/viz/panels.hpp"
#include "volumetric_kit/calib/viz/point_cloud.hpp"
#include "volumetric_kit/calib/viz/scene_view.hpp"
#include "volumetric_kit/core/vulkan/image.hpp"

namespace volumetric_kit::calib::viz {
namespace {

using Code = core::Status::Code;
using test::expect_oom_anywhere;
using test::expect_oom_when_reporting;

TEST(VizOom, MosaicLayout) {
  MosaicLimits limits;
  limits.max_extent = {144, 112};  // paged
  const std::vector<sensor::camera::ImageSize> sizes(5, {64, 48});
  expect_oom_anywhere(
      [&] { return mosaic_layout(sizes, kMosaicGutter, limits); });
  const std::vector<sensor::camera::ImageSize> odd = {{65, 48}};
  expect_oom_when_reporting([&] { return mosaic_layout(odd); },
                            Code::InvalidArgument);
}

TEST(VizOom, TileWriteAndConversionRefusals) {
  sensor::YuvImage picture;
  picture.layout = sensor::YuvLayout::Nv12;
  picture.image[0] = std::make_shared<core::Image>();
  picture.image[1] = std::make_shared<core::Image>();
  picture.width = 64;
  picture.height = 48;
  picture.chroma_location = sensor::ChromaLocation::TopLeft;
  // Built outside the calls, whose own allocations alone are swept.
  const std::vector<const sensor::YuvImage*> unsited = {&picture};
  expect_oom_when_reporting([&] { return tile_conversion(unsited); },
                            Code::Unsupported);
  expect_oom_when_reporting([&] { return tile_write(picture, {32, 24}, {}); },
                            Code::InvalidArgument);
  sensor::YuvImage other = picture;
  picture.chroma_location = sensor::ChromaLocation::Center;
  other.chroma_location = sensor::ChromaLocation::Left;
  const std::vector<const sensor::YuvImage*> mixed = {&picture, &other};
  expect_oom_when_reporting([&] { return tile_conversion(mixed); },
                            Code::InvalidArgument);
  picture.image[1].reset();
  expect_oom_when_reporting([&] { return tile_write(picture, {64, 48}, {}); },
                            Code::InvalidArgument);
}

TEST(VizOom, TileMarks) {
  Detection found;
  found.observation.points.push_back({0, {10.0, 20.0}, 1.0});
  found.observation.points.push_back({1, {30.0, 20.0}, 1.0});
  found.markers.emplace_back();
  const quality::CameraSummary session;
  TileOverlay tile;
  tile.id = "CL2A141000N";
  tile.detection = &found;
  tile.session = &session;
  expect_oom_anywhere([&] { return tile_marks(tile); });
}

TEST(VizOom, Panels) {
  CameraLive camera;
  camera.present = true;
  camera.check.distance_m = 2.5;
  camera.lost = 1;
  expect_oom_anywhere([&] {
    return camera_panel(camera, quality::CameraSummary{}, legacy_femto_board());
  });

  sensor::FrameSet set;
  set.frames.resize(2);
  set.frames[0].emplace();
  const std::vector<std::string> ids = {"CL2A141000N", "CL2A14100A4"};
  expect_oom_anywhere([&] { return set_panel(set, {}, ids, {}); });
  expect_oom_when_reporting([&] { return set_panel(set, {}, {}, {}); },
                            Code::InvalidArgument);

  quality::SessionSummary summary;
  summary.cameras.resize(2);
  summary.cameras[0].linked = true;
  summary.cameras[1].link =
      quality::PreviewLink{0, 3, quality::LinkStrength::Weak};
  summary.cameras[1].floor_only = true;
  summary.duplicates.push_back({2, 1});
  summary.covisibility = {{3, 3}, {3, 3}};
  expect_oom_anywhere([&] { return session_panel(summary, ids); });
  expect_oom_when_reporting([&] { return session_panel(summary, {}); },
                            Code::InvalidArgument);
  // A rig of 1000 cameras, so even the shortest line is too long for any
  // small-string buffer.
  for (std::size_t seen_by = 0; seen_by <= 2; ++seen_by) {
    expect_oom_anywhere([&] { return origin_tag_line({1000, seen_by, 5}); });
  }
  expect_oom_when_reporting([] { return origin_tag_line({2, 3, 5}); },
                            Code::InvalidArgument);

  CaptureStatus status;
  status.decision.state = quality::CaptureState::Waiting;
  status.decision.message = "board moving (0.82 px)";
  status.last = CaptureMeta{};
  status.error = "disk full";
  expect_oom_anywhere([&] { return capture_panel(status); });

  core::StageMetrics stages;
  stages.add_cpu("poll", 1.0);
  expect_oom_anywhere([&] { return stage_frame_metrics(stages, 25.0); });
}

// Holding a stage's rows and adding them to the frame's.
TEST(VizOom, HoldStages) {
  core::StageMetrics ran;
  ran.add_cpu("tiles", 4.0);
  std::size_t oom = 0;
  test::sweep_allocations([&](std::size_t after) {
    core::StageMetrics held;
    core::StageMetrics stages;
    stages.add_cpu("poll", 1.0);
    std::optional<core::Status> status;
    EXPECT_NO_THROW({
      const test::FailAllocation failure(after);
      status.emplace(hold_stages(ran, true, held, stages));
    });
    ASSERT_TRUE(status.has_value());
    if (!test::allocation_failure.injected) {
      EXPECT_TRUE(status->ok());
      EXPECT_EQ(stages.rows().size(), 2u);
      return;
    }
    EXPECT_EQ(status->domain(), Code::OutOfMemory);
    ++oom;
  });
  EXPECT_GT(oom, 0u);
}

// Calibrate's lines: its progress, and its result's every kind of line --
// a camera moved and one not, the A/B test decided and refused, and the depth
// checks', made and not.
TEST(VizOom, CalibratePanels) {
  // Longer than any small-string buffer, so the line allocates.
  expect_oom_anywhere([] {
    return calibrate_progress_line("measuring the rig without a capture", 104,
                                   150, 1234.5);
  });
  quality::CalibrationMetrics metrics;
  const std::vector<std::string> ids = {"CL2A141000N", "CL2A14100A4"};
  for (auto* stats : {&metrics.reprojection.initial.cameras,
                      &metrics.reprojection.solved.cameras}) {
    stats->resize(2);
    (*stats)[0].count = 10;
    (*stats)[0].rms_px = 0.5;
  }
  metrics.transfer.median_px = 0.6;
  metrics.transfer.p90_px = 1.4;
  metrics.leave_one_out.cameras.resize(2);
  metrics.leave_one_out.cameras[1] = {1.5, 7, 0.05, 3};
  metrics.principal_point.as_given.rms_px = 0.31;
  metrics.principal_point.shifted.rms_px = 0.30;
  metrics.geometry.centres_m.resize(2);
  const std::string directory =
      "/sessions/2026-10-19_101500/results/000_2026-10-19_101500";
  expect_oom_anywhere(
      [&] { return calibration_panel(ids, metrics, 0, directory, 12, 3.5); });
  metrics.principal_point.refused = "a link left short of 3 shared captures";
  expect_oom_anywhere(
      [&] { return calibration_panel(ids, metrics, 0, directory, 12, 3.5); });
  expect_oom_when_reporting(
      [&] { return calibration_panel({}, metrics, 0, directory, 12, 3.5); },
      Code::InvalidArgument);

  // The depth lines: every kind, not checked, and a refusal.
  quality::DepthSummary depth;
  depth.cameras.resize(2);
  depth.cameras[0].views = 12;
  depth.cameras[0].verdict = quality::DepthVerdict::Ok;
  quality::PairCheck failing;
  failing.agreement = {0, 1, std::nullopt, 5000, 3000, 6.2, 7.1, 9.0};
  failing.verdict = quality::DepthVerdict::Off;
  quality::PairCheck passing;
  passing.agreement = {1, 0, std::nullopt, 5000, 4900, -0.2, 1.1, 2.0};
  passing.verdict = quality::DepthVerdict::Ok;
  depth.pairs = {failing, passing};
  depth.captures = {{5, depth.pairs}};
  expect_oom_anywhere([&] { return depth_panel(ids, &depth); });
  const core::Status lost =
      core::Status::io_error("depth checks: capture 5: the device was lost");
  expect_oom_anywhere([&] { return depth_panel(ids, nullptr, lost); });
  expect_oom_when_reporting([&] { return depth_panel({}, &depth); },
                            Code::InvalidArgument);

  // The world lines: a tag world with its edge warned of and boards on its
  // floor, the reference with a reason, and a refusal.
  quality::ResultWorld world;
  world.origin.sought = kOriginTag;
  solve::TagWorld& tag = world.origin.tag.emplace();
  tag.edge_m = 0.1552;
  tag.edge_error_percent = 3.47;
  tag.edge_warning = true;
  tag.cameras = {{ids[0], 6, 6, 2.41}, {ids[1], 6, 5, 2.6}};
  world.floor_boards = {{3, 1.1, 0.002}};
  expect_oom_anywhere([&] { return world_panel(ids, 0, world); });
  quality::ResultWorld reference;
  reference.origin.sought = kOriginTag;
  reference.origin.miss = solve::TagMiss::PoorFit;
  reference.origin.reason =
      "the tag's corners fit its views at 4.62582 px RMS, above the 3 px a "
      "world takes: the cameras disagree on where it is";
  expect_oom_anywhere([&] { return world_panel(ids, 1, reference); });
  expect_oom_when_reporting([&] { return world_panel(ids, 2, reference); },
                            Code::InvalidArgument);
  // The notices of a warned edge.
  const std::vector<PanelLine> warned = world_panel(ids, 0, world).value();
  expect_oom_anywhere(
      [&] { return calibration_notices(7, "results/007_a", warned); });
}

// A refusal that runs out of memory reports OutOfMemory, and the editor
// keeps its board.
TEST(VizOom, BoardEditorRefusals) {
  Target bad = legacy_femto_board();
  bad.marker_size_m = 0.2;
  expect_oom_when_reporting([&] { return BoardEditor::create(bad); },
                            Code::InvalidArgument);

  BoardEditor editor = BoardEditor::create(legacy_femto_board()).value();
  editor.lock();
  Target measured = editor.board();
  measured.square_size_m = 0.1505;
  std::size_t oom = 0;
  test::sweep_allocations([&](std::size_t after) {
    std::optional<core::Status> status;
    EXPECT_NO_THROW({
      const test::FailAllocation failure(after);
      status.emplace(editor.edit(measured, true));
    });
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(editor.board().square_size_m, 0.15);
    EXPECT_FALSE(editor.square_measured());
    if (!test::allocation_failure.injected) {
      EXPECT_EQ(status->domain(), Code::InvalidArgument);
      return;
    }
    EXPECT_EQ(status->domain(), Code::OutOfMemory);
    ++oom;
  });
  EXPECT_GT(oom, 0u);
}

// The 3D view's lines, through OpenCV's undistortion, and its turn's refusal.
TEST(VizOom, SceneLinesAndTurn) {
  sensor::camera::CameraModel lens;
  lens.size = {1280, 720};
  lens.intrinsics = {900.0, 901.0, 640.5, 359.5};
  lens.distortion.k1 = -0.12;
  lens.distortion.k2 = 0.03;
  std::vector<SceneCamera> cameras(2);
  for (SceneCamera& camera : cameras) {
    camera.color = lens;
    camera.depth = lens;
  }
  cameras[0].color_to_world = sensor::camera::Mat4d(1.0);
  SceneOptions options;
  options.depth_frusta = true;
  options.tag_edge_m = 0.15;
  const compute::Board board =
      compute::Board::create(legacy_femto_board()).value();
  const std::vector<sensor::camera::Mat4d> boards(2,
                                                  sensor::camera::Mat4d(1.0));
  expect_oom_anywhere(
      [&] { return scene_lines(cameras, &board, boards, options); });
  expect_oom_when_reporting(
      [&] { return scene_lines(cameras, nullptr, boards, options); },
      Code::InvalidArgument);
  expect_oom_when_reporting(
      [&] { return frustum_lines(lens, sensor::camera::Mat4d(1.0), 0.0, 0); },
      Code::InvalidArgument);
  expect_oom_anywhere([] { return tag_lines(0.15, 0); });
  expect_oom_when_reporting([] { return tag_lines(0.0, 0); },
                            Code::InvalidArgument);
  // A turn allocates nothing but its refusal.
  const std::vector<sensor::camera::Mat4d> none;
  expect_oom_when_reporting([&] { return display_rotation(none); },
                            Code::InvalidArgument);
  const std::vector<sensor::camera::Mat4d> one(1, sensor::camera::Mat4d(1.0));
  expect_oom_when_reporting(
      [&] { return display_rotation(one, sensor::camera::Vec3d(0.0)); },
      Code::InvalidArgument);
}

// The points' draws, and their refusals, from frames with no buffer.
TEST(VizOom, CloudDraws) {
  std::vector<SceneCamera> cameras(3);  // none placed: nothing to draw
  const std::vector<std::optional<sensor::DeviceFrame>> frames(
      3, sensor::DeviceFrame{});
  CloudStyle style;
  style.producer_family = 0;  // so a frame with no depth is what is refused
  style.renderer_family = 0;
  expect_oom_anywhere([&] { return cloud_draws(frames, cameras, style); });
  cameras.pop_back();
  expect_oom_when_reporting([&] { return cloud_draws(frames, cameras, style); },
                            Code::InvalidArgument);
  const sensor::camera::Mat4d identity(1.0);
  expect_oom_when_reporting(
      [&] { return cloud_draw(frames[0].value(), identity, identity, style); },
      Code::InvalidArgument);
}

}  // namespace
}  // namespace volumetric_kit::calib::viz
