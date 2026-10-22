# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_capture: off core -------------------------------------------------
# Sessions on disk: the rig and each capture's RGB-D set, detections and checks,
# written on a thread of its own and never overwritten. Built with VC_WITH_RIG:
# it takes recon's sensor tier (SensorInfo) and io tier (the 16-bit PNG
# encoder), and recon's calibration file for the factory record, so it needs the
# Vulkan SDK through recon. Depends on calib_core alone among calib's tiers.
include(GenerateExportHeader)

set(_vc_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(calib_capture)
add_library(volumetric_kit::calib_capture ALIAS calib_capture)

target_sources(calib_capture PRIVATE session_files.cpp session_writer.cpp
                                     sha256.cpp)

# The tier's own export macro and guard (see calib_core).
generate_export_header(
  calib_capture
  BASE_NAME
  VC_CAPTURE
  EXPORT_MACRO_NAME
  VC_CAPTURE_API
  EXPORT_FILE_NAME
  "${_vc_generated_inc}/volumetric_kit/calib/capture/export.hpp")

set_target_properties(
  calib_capture
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vc_type calib_capture TYPE)
if(_vc_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(calib_capture PUBLIC VC_CAPTURE_STATIC_DEFINE)
endif()

target_include_directories(calib_capture PUBLIC "${_vc_generated_inc}")
# nlohmann/json writes the files, privately: no public header names it.
target_include_directories(calib_capture SYSTEM
                           PRIVATE "${VC_JSON_INCLUDE_DIR}")

# The public header names recon's SensorInfo; the PNG encoder and the threads
# stay private.
target_link_libraries(
  calib_capture
  PUBLIC calib_core volumetric_kit::recon_sensor
  PRIVATE volumetric_kit::recon_io Threads::Threads)

vc_target_warnings(calib_capture)
