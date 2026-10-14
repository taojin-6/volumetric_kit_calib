# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_compute: Tier 1 ---------------------------------------------------
# The OpenCV bridge, calib's CPU maths backend: recon's camera model and rigid
# transforms in OpenCV's types, the ChArUco board and whether a view of it fixes
# a pose, the measures the solve and the checks share, and guarded(), which
# returns what OpenCV throws as a Status. Built with VC_WITH_OPENCV.
include(GenerateExportHeader)
include(GNUInstallDirs)

set(_vc_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(calib_compute)
add_library(volumetric_kit::calib_compute ALIAS calib_compute)

target_sources(
  calib_compute PRIVATE board.cpp cv_camera.cpp cv_pose.cpp guarded.cpp
                        measures.cpp usable_for_pose.cpp)

# The tier's own export macro and guard (see calib_core).
generate_export_header(
  calib_compute
  BASE_NAME
  VC_COMPUTE
  EXPORT_MACRO_NAME
  VC_COMPUTE_API
  EXPORT_FILE_NAME
  "${_vc_generated_inc}/volumetric_kit/calib/compute/export.hpp")

set_target_properties(
  calib_compute
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vc_type calib_compute TYPE)
if(_vc_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(calib_compute PUBLIC VC_COMPUTE_STATIC_DEFINE)
endif()

target_include_directories(
  calib_compute PUBLIC "$<BUILD_INTERFACE:${_vc_generated_inc}>"
                       "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

# The public headers name OpenCV's core types and its ChArUco board.
# opencv_geometry is private: only undistort_points uses it.
target_link_libraries(
  calib_compute
  PUBLIC calib_core opencv_core opencv_objdetect
  PRIVATE opencv_geometry)

vc_target_warnings(calib_compute)

if(VC_INSTALL)
  install(
    TARGETS calib_compute
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
