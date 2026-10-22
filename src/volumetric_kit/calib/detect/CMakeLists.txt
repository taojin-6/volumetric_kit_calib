# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_detect: Tier 2 ----------------------------------------------------
# The ChArUco board and the origin tag in a luma image, through OpenCV's
# detectors.
include(GenerateExportHeader)

set(_vc_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(calib_detect)
add_library(volumetric_kit::calib_detect ALIAS calib_detect)

target_sources(calib_detect PRIVATE charuco.cpp tag.cpp)

# The tier's own export macro and guard (see calib_core).
generate_export_header(
  calib_detect
  BASE_NAME
  VC_DETECT
  EXPORT_MACRO_NAME
  VC_DETECT_API
  EXPORT_FILE_NAME
  "${_vc_generated_inc}/volumetric_kit/calib/detect/export.hpp")

set_target_properties(
  calib_detect
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vc_type calib_detect TYPE)
if(_vc_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(calib_detect PUBLIC VC_DETECT_STATIC_DEFINE)
endif()

target_include_directories(calib_detect PUBLIC "${_vc_generated_inc}")

# The public header names compute's Board; imgproc (resize, cornerSubPix) is
# private.
target_link_libraries(
  calib_detect
  PUBLIC calib_compute
  PRIVATE opencv_imgproc)

vc_target_warnings(calib_detect)
