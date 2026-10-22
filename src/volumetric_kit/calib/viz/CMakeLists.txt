# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_viz: Tier 4 -------------------------------------------------------
# The live view: every camera's decoded picture copied on the device into its
# tile of one mosaic, the marks over each tile, the panels and the board editor,
# on gfx's image pipeline, ImGui overlay and image view; and the 3D view: each
# camera's prepared depth drawn as points in place at the rig's poses, its
# frusta and the solved boards, on gfx's depth-cloud and primitive pipelines.
# Built with VC_WITH_RIG. A static library for calib_rig and the tests: no
# export macros (DECISIONS.md, 2026-10-22).
add_library(calib_viz STATIC)
add_library(volumetric_kit::calib_viz ALIAS calib_viz)

target_sources(
  calib_viz
  PRIVATE board_editor.cpp
          mosaic.cpp
          overlay.cpp
          panels.cpp
          point_cloud.cpp
          scene_view.cpp
          tone.cpp)

# The public headers name quality's checks and summaries, gfx's pipelines,
# overlay, views and metrics, and recon's pictures, sets, counters and prepared
# frames. gfx_pipelines brings gfx_camera, the views' tier.
target_link_libraries(
  calib_viz
  PUBLIC calib_quality volumetric_kit::gfx_pipelines volumetric_kit::gfx_ui
         volumetric_kit::recon_sensor_array volumetric_kit::recon_sensor_utils)

vc_target_warnings(calib_viz)
