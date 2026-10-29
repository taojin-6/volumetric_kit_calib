# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_depth: Tier 4 -----------------------------------------------------
# The depth checks' device side: each view's board plane fitted on the GPU to
# the depth sensor's frame preparation leaves on the device, and each ordered
# pair of cameras' depth compared, read in place, on the core's vulkan tier
# (DECISIONS.md, "Depth checks are calib's kernels"). Built with VC_WITH_RIG,
# beside viz: it takes the quality tier's region, plane, options and pair
# agreement, and names no sensor type and nothing of gfx.
include(GenerateExportHeader)

set(_vc_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(calib_depth)
add_library(volumetric_kit::calib_depth ALIAS calib_depth)

target_sources(calib_depth PRIVATE depth_checks.cpp)

# The tier's own export macro and guard (see calib_core).
generate_export_header(
  calib_depth
  BASE_NAME
  VC_DEPTH
  EXPORT_MACRO_NAME
  VC_DEPTH_API
  EXPORT_FILE_NAME
  "${_vc_generated_inc}/volumetric_kit/calib/depth/export.hpp")

set_target_properties(
  calib_depth
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vc_type calib_depth TYPE)
if(_vc_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(calib_depth PUBLIC VC_DEPTH_STATIC_DEFINE)
endif()

target_include_directories(calib_depth PUBLIC "${_vc_generated_inc}")

# The public header names quality's region, plane, options and pair agreement,
# and the core's device, allocator and buffers.
target_link_libraries(calib_depth PUBLIC calib_quality
                                         volumetric_kit::core_vulkan)

# The kernels include calib's own GLSL under src/, spelt as the C++ headers are,
# and sensor's pinhole (pinhole.glsl), which sensor_frame exports. The tests'
# kernels take the same (tests/CMakeLists.txt).
vkc_embed_shaders(
  calib_depth
  SYMBOL_PREFIX
  vc_
  INCLUDE_DIRS
  "${PROJECT_SOURCE_DIR}/src"
  INCLUDE_TARGETS
  volumetric_kit::sensor_frame
  SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/plane_moments.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/plane_solve.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/agreement.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/agreement_stats.comp")

vc_target_warnings(calib_depth)
