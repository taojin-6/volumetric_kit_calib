# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_quality: Tier 3 ---------------------------------------------------
# What the views and the captures are worth, before and after the solve: the
# live checks of each view and set, the capture rule, and the session's checks.
# Pure and geometric, on detect's records and solve's poses and tree. Built with
# VC_WITH_OPENCV.
include(GenerateExportHeader)
include(GNUInstallDirs)

set(_vc_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(calib_quality)
add_library(volumetric_kit::calib_quality ALIAS calib_quality)

target_sources(calib_quality PRIVATE capture_rule.cpp live_checks.cpp
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

# The public headers name compute's board and camera and solve's rig cameras and
# covisibility; nothing calls OpenCV directly.
target_link_libraries(calib_quality PUBLIC calib_solve)

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
