# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_quality: Tier 3 ---------------------------------------------------
# What the views and the captures are worth, before and after the solve: the
# live checks of each view and set, the capture rule, the session's checks, the
# solved rig's metrics, the depth checks' geometry and verdicts over the
# device's results, the boards on the origin tag's floor, and its results files'
# text. Pure and geometric, on core's detection records and solve's poses, view
# rule, tree, adjustment and world.
include(GenerateExportHeader)
include(GNUInstallDirs)

set(_vc_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(calib_quality)
add_library(volumetric_kit::calib_quality ALIAS calib_quality)

target_sources(
  calib_quality
  PRIVATE calibration_metrics.cpp
          capture_rule.cpp
          depth_checks.cpp
          floor_boards.cpp
          live_checks.cpp
          results.cpp
          session_checks.cpp)

# The tier's own export macro and guard (see calib_core).
generate_export_header(
  calib_quality
  BASE_NAME
  VC_QUALITY
  EXPORT_MACRO_NAME
  VC_QUALITY_API
  EXPORT_FILE_NAME
  "${_vc_generated_inc}/volumetric_kit/calib/quality/export.hpp")

set_target_properties(
  calib_quality
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vc_type calib_quality TYPE)
if(_vc_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(calib_quality PUBLIC VC_QUALITY_STATIC_DEFINE)
endif()

target_include_directories(
  calib_quality PUBLIC "$<BUILD_INTERFACE:${_vc_generated_inc}>"
                       "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

# The public headers name compute's board and camera and solve's rig cameras,
# poses, options, covisibility and solution; nothing calls OpenCV directly. The
# report names detect's tag corner refinement, privately.
target_link_libraries(
  calib_quality
  PUBLIC calib_solve
  PRIVATE calib_detect)
# nlohmann/json writes the report, privately, through calib's JSON writer
# (core/json_text.hpp): no public header names it.
target_include_directories(calib_quality SYSTEM
                           PRIVATE "${VC_JSON_INCLUDE_DIR}")

vc_target_warnings(calib_quality)

if(VC_INSTALL)
  install(
    TARGETS calib_quality
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
