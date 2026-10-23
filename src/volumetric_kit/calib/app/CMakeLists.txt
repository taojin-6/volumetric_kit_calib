# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_app: Tier 5 -------------------------------------------------------
# calib_rig's live view, 3D view, capture and Calibrate: the rig's preset check,
# the key map, the window's mode, the acquisition and live-detection worker
# (T1), the 3D view's frame preparation worker (T2), the capture worker (T3),
# the Calibrate worker (T4) and the window loop (T0) on gfx's GLFW glue, ImGui
# overlay and the viz tier's mosaic and scene. Built with VC_BUILD_APPS and the
# viz tier when gfx_glfw (an installed GLFW) and recon's Orbbec tier
# (VR_WITH_ORBBEC) exist (src/CMakeLists.txt). A static library, as calib_viz
# is.

# calib_rig loads its scenario as config::Application::CalibRig, which the core
# has from 0.11.0, below calib's floor, in its config tier
# (third_party/CMakeLists.txt turns it on).
vkc_require_core(${VC_CORE_MIN_VERSION} CONFIG)

# What a session records it was captured with (capture::Software): calib's and
# each fetched sibling's commit, read from its checkout at every build
# (software.cmake), so a rebuild after a commit or an edit records it, and the
# Orbbec SDK's version, from recon's configure.
include(FetchContent)
FetchContent_GetProperties(volumetric_kit_core)
FetchContent_GetProperties(volumetric_kit_recon)
FetchContent_GetProperties(volumetric_kit_gfx)
# TODO: take the Orbbec SDK's version from recon once its Orbbec tier exposes
# it; VR_ORBBEC_SDK_VERSION is recon's own configure variable.
get_directory_property(
  _vc_orbbec_sdk_version DIRECTORY "${volumetric_kit_recon_SOURCE_DIR}"
                                   DEFINITION VR_ORBBEC_SDK_VERSION)
set(_vc_software_cpp "${CMAKE_CURRENT_BINARY_DIR}/software.cpp")
set(_vc_software_command
    "${CMAKE_COMMAND}" "-DIN=${CMAKE_CURRENT_SOURCE_DIR}/software.cpp.in"
    "-DOUT=${_vc_software_cpp}" "-DCALIB_DIR=${PROJECT_SOURCE_DIR}"
    "-DCORE_DIR=${volumetric_kit_core_SOURCE_DIR}"
    "-DRECON_DIR=${volumetric_kit_recon_SOURCE_DIR}"
    "-DGFX_DIR=${volumetric_kit_gfx_SOURCE_DIR}"
    "-DORBBEC_SDK_VERSION=${_vc_orbbec_sdk_version}" -P
    "${CMAKE_CURRENT_SOURCE_DIR}/software.cmake")
# Once now, so the source exists when the build is generated, then before every
# build of the tier.
execute_process(COMMAND ${_vc_software_command} RESULT_VARIABLE _vc_software_rc)
if(NOT _vc_software_rc EQUAL 0)
  message(FATAL_ERROR "calib_app: software.cmake failed (${_vc_software_rc})")
endif()
add_custom_target(
  calib_app_software
  COMMAND ${_vc_software_command}
  BYPRODUCTS "${_vc_software_cpp}"
  VERBATIM)

add_library(calib_app STATIC)
add_library(volumetric_kit::calib_app ALIAS calib_app)

target_sources(
  calib_app
  PRIVATE calib_rig.cpp
          calibrate_worker.cpp
          capture_worker.cpp
          cloud_worker.cpp
          keys.cpp
          live_view.cpp
          live_worker.cpp
          preset.cpp
          rig_mode.cpp
          "${_vc_software_cpp}")
add_dependencies(calib_app calib_app_software)

# The public headers name the viz tier's mosaic, overlays and scene, the quality
# tier's metrics and results files, the capture tier's session records, the
# detector's board, recon's sensor array, picture readback, frame preparation
# and Orbbec options, gfx's retire queue and depth-cloud pipeline, and the
# core's scenario. The window, the shared device, the depth checks' kernels,
# OpenCV's thread pool and the threads stay private.
target_link_libraries(
  calib_app
  PUBLIC calib_viz
         calib_quality
         calib_capture
         calib_detect
         volumetric_kit::recon_sensor_array
         volumetric_kit::recon_sensor_utils
         volumetric_kit::recon_sensor_orbbec
         volumetric_kit::core_config
  PRIVATE calib_depth volumetric_kit::gfx_app volumetric_kit::gfx_glfw
          volumetric_kit::recon_core opencv_core Threads::Threads)

vc_target_warnings(calib_app)
