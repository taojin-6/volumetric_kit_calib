// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The viz tier runs out of memory anywhere in its models and returns
// OutOfMemory, never throwing; a board editor is then as it was.

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "allocation_failure.hpp"
#include "volumetric_kit/calib/viz/board_editor.hpp"
#include "volumetric_kit/calib/viz/overlay.hpp"
#include "volumetric_kit/calib/viz/panels.hpp"
#include "volumetric_kit/calib/viz/tiles.hpp"
#include "volumetric_kit/core/vulkan/image.hpp"

namespace volumetric_kit::calib::viz {
namespace {

using Code = core::Status::Code;
using test::expect_oom_anywhere;
using test::expect_oom_when_reporting;

TEST(VizOom, TileUpdateRefusals) {
  recon::sensor::YuvImage picture;
  picture.layout = recon::sensor::YuvLayout::Nv12;
  picture.image[0] = std::make_shared<core::Image>();
  picture.image[1] = std::make_shared<core::Image>();
  picture.width = 64;
  picture.height = 48;
  picture.chroma_location = recon::sensor::ChromaLocation::TopLeft;
  expect_oom_when_reporting([&] { return tile_update(picture); },
                            Code::Unsupported);
  picture.image[1].reset();
  expect_oom_when_reporting([&] { return tile_update(picture); },
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
  camera.id = "N";
  camera.present = true;
  camera.check.distance_m = 2.5;
  camera.lost = 1;
  expect_oom_anywhere([&] {
    return camera_panel(camera, quality::CameraSummary{}, legacy_femto_board());
  });

  recon::sensor::FrameSet set;
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
  for (std::size_t after = 0;; ++after) {
    ASSERT_LT(after, 32u);
    std::optional<core::Status> status;
    EXPECT_NO_THROW({
      const test::FailAllocation failure(after);
      status.emplace(editor.edit(measured, true));
    });
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(editor.board().square_size_m, 0.15);
    EXPECT_FALSE(editor.square_measured());
    if (!test::allocation_failure.injected) {
      EXPECT_GT(after, 0u);
      EXPECT_EQ(status->domain(), Code::InvalidArgument);
      break;
    }
    EXPECT_EQ(status->domain(), Code::OutOfMemory);
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::viz
