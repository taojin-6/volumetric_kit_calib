# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_viz: Tier 4 -------------------------------------------------------
# The live view: each camera's decoded picture drawn in place from the device as
# a tile, the marks over it, the panels and the board editor, on gfx's image
# pipeline, ImGui overlay and image views. Built with VC_WITH_RIG and
# VC_WITH_OPENCV. Build-tree only, as gfx_ui is: neither is installed until gfx
# co-installs the ImGui headers.
include(GenerateExportHeader)

set(_vc_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(calib_viz)
add_library(volumetric_kit::calib_viz ALIAS calib_viz)

target_sources(calib_viz PRIVATE board_editor.cpp overlay.cpp panels.cpp
                                 tiles.cpp tone.cpp)

# The tier's own export macro and guard (see calib_core).
generate_export_header(
  calib_viz
  BASE_NAME
  VC_VIZ
  EXPORT_MACRO_NAME
  VC_VIZ_API
  EXPORT_FILE_NAME
  "${_vc_generated_inc}/volumetric_kit/calib/viz/export.hpp")

set_target_properties(
  calib_viz
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vc_type calib_viz TYPE)
if(_vc_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(calib_viz PUBLIC VC_VIZ_STATIC_DEFINE)
endif()

target_include_directories(calib_viz
                           PUBLIC "$<BUILD_INTERFACE:${_vc_generated_inc}>")

# The public headers name quality's checks and summaries, gfx's pipeline,
# overlay, views and metrics, and recon's pictures, sets and counters.
target_link_libraries(
  calib_viz
  PUBLIC calib_quality volumetric_kit::gfx_pipelines volumetric_kit::gfx_ui
         volumetric_kit::gfx_camera volumetric_kit::recon_sensor_array)

vc_target_warnings(calib_viz)
