# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_source: off detect ------------------------------------------------
# Every camera calib reads as one source of synced 8-bit grey pictures with
# camera ids: a sensor array or one stereo sensor, each picture's luma and half
# made on the GPU by sensor's LumaPass and read back as one list. Built with
# VC_WITH_RIG: it takes sensor's array, stereo and prep tiers, so it needs the
# Vulkan SDK through sensor. It names no driver. Public, outside the stable API
# (DECISIONS.md, 2026-11-02).
include(GenerateExportHeader)

set(_vc_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(calib_source)
add_library(volumetric_kit::calib_source ALIAS calib_source)

target_sources(calib_source PRIVATE rig_source.cpp)

# The tier's own export macro and guard (see calib_core).
generate_export_header(
  calib_source
  BASE_NAME
  VC_SOURCE
  EXPORT_MACRO_NAME
  VC_SOURCE_API
  EXPORT_FILE_NAME
  "${_vc_generated_inc}/volumetric_kit/calib/source/export.hpp")

set_target_properties(
  calib_source
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vc_type calib_source TYPE)
if(_vc_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(calib_source PUBLIC VC_SOURCE_STATIC_DEFINE)
endif()

target_include_directories(calib_source PUBLIC "${_vc_generated_inc}")

# The public header names detect's LumaView, sensor's array, stereo sensor and
# pictures; the luma pass and the readback stay private.
target_link_libraries(
  calib_source
  PUBLIC calib_detect volumetric_kit::sensor_array volumetric_kit::sensor_stereo
  PRIVATE volumetric_kit::sensor_prep)

vc_target_warnings(calib_source)
