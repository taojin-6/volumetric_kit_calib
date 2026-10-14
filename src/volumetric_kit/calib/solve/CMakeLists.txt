# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_solve: Tier 2 -----------------------------------------------------
# The rig solve on OpenCV: each view's own pose (IPPE), the cameras' spanning
# tree, the first rig, and the bundle adjustment that refines it (LevMarq).
# Built with VC_WITH_OPENCV.
include(GenerateExportHeader)
include(GNUInstallDirs)

set(_vc_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(calib_solve)
add_library(volumetric_kit::calib_solve ALIAS calib_solve)

target_sources(
  calib_solve PRIVATE bundle_adjust.cpp common.cpp initialise.cpp
                      reprojection.cpp spanning_tree.cpp view_pose.cpp)

# The tier's own export macro and guard (see calib_core).
generate_export_header(
  calib_solve
  BASE_NAME
  VC_SOLVE
  EXPORT_MACRO_NAME
  VC_SOLVE_API
  EXPORT_FILE_NAME
  "${_vc_generated_inc}/volumetric_kit/calib/solve/export.hpp")

set_target_properties(
  calib_solve
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vc_type calib_solve TYPE)
if(_vc_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(calib_solve PUBLIC VC_SOLVE_STATIC_DEFINE)
endif()

target_include_directories(
  calib_solve PUBLIC "$<BUILD_INTERFACE:${_vc_generated_inc}>"
                     "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

# The public headers name compute's board and camera; OpenCV's geometry module
# (PnP, projection, buildMST, LevMarq) stays private.
target_link_libraries(
  calib_solve
  PUBLIC calib_compute
  PRIVATE opencv_geometry)

vc_target_warnings(calib_solve)

if(VC_INSTALL)
  install(
    TARGETS calib_solve
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
