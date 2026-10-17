# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_depth: Tier 4 -----------------------------------------------------
# The depth checks' device side: each view's board plane fitted on the GPU to
# the depth recon's frame preparation leaves on the device, and each ordered
# pair of cameras' depth compared, read in place, on the core's vulkan tier
# (DECISIONS.md, "Depth checks are calib's kernels"). Built with VC_WITH_RIG and
# VC_WITH_OPENCV, beside viz: it takes the quality tier's region, plane, options
# and pair agreement, and names no recon sensor type and nothing of gfx.
include(GenerateExportHeader)
include(GNUInstallDirs)

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

target_include_directories(
  calib_depth PUBLIC "$<BUILD_INTERFACE:${_vc_generated_inc}>"
                     "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

# The public header names quality's region, plane, options and pair agreement,
# and the core's device, allocator and buffers.
target_link_libraries(calib_depth PUBLIC calib_quality
                                         volumetric_kit::core_vulkan)

# The kernels include recon's pinhole (pinhole.glsl) under the root recon_core
# exports, and calib's own GLSL under src/, spelt as the C++ headers are. The
# VC_SHADER_INCLUDE_DIRS property names both, for the tests' kernels.
get_target_property(_vc_recon_glsl volumetric_kit::recon_core
                    VR_SHADER_INCLUDE_DIR)
if(NOT _vc_recon_glsl)
  message(
    FATAL_ERROR
      "calib_depth needs volumetric_kit_recon at or after bdbe26e (PR #245), "
      "whose recon_core names its GLSL include root in VR_SHADER_INCLUDE_DIR; "
      "the recon this build made available has none.")
endif()
set(_vc_glsl_roots "${PROJECT_SOURCE_DIR}/src" "${_vc_recon_glsl}")
set_target_properties(calib_depth PROPERTIES VC_SHADER_INCLUDE_DIRS
                                             "${_vc_glsl_roots}")
vkc_embed_shaders(
  calib_depth
  SYMBOL_PREFIX
  vc_
  INCLUDE_DIRS
  ${_vc_glsl_roots}
  SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/plane_moments.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/plane_solve.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/agreement.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/agreement_stats.comp")

vc_target_warnings(calib_depth)

if(VC_INSTALL)
  install(
    TARGETS calib_depth
    EXPORT volumetric_kit_calibTargets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
  install(
    DIRECTORY "${_vc_generated_inc}/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING
    PATTERN "*.hpp")
endif()
