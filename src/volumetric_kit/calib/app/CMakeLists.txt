# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_app: Tier 5 -------------------------------------------------------
# calib_rig's live view, 3D view, capture, Calibrate and Scan's stills (V210
# made R'G'B' and PNG on the capture worker, with sensor's interface and image
# I/O tiers): the opener, the one code that names a driver, with an arm per
# driver built (Orbbec's preset check, DeckLink's pair); the per-mode table and
# the Setup panel's switch of mode and lens file; the key map, the window's
# mode, the acquisition and live-detection worker (T1) on the source tier, the
# 3D view's frame preparation worker (T2), the capture worker (T3), the
# Calibrate worker (T4) and the window loop (T0) on gfx's GLFW glue, ImGui
# overlay and the viz tier's mosaic and scene. Built with VC_BUILD_APPS and the
# viz tier when gfx_glfw (an installed GLFW) and a sensor driver tier exist:
# Orbbec's (VS_WITH_ORBBEC) or DeckLink's (VS_WITH_DECKLINK)
# (src/CMakeLists.txt). A static library, as calib_viz is.

# calib_rig loads its scenario as config::Application::CalibRig, whose operation
# (calibrate or calibrate_intrinsics) and DeckLink cameras the core has from
# 0.40.0, and its scan from 0.51.0, in its config tier
# (third_party/CMakeLists.txt turns it on).
vkc_require_core(${VC_APP_CORE_MIN_VERSION} CONFIG)

# What a session records it was captured with (capture::Software): calib's and
# each fetched sibling's commit, read from its checkout at every build
# (software.cmake), so a rebuild after a commit or an edit records it, and the
# Orbbec SDK's version, from sensor's configure.
include(FetchContent)
FetchContent_GetProperties(volumetric_kit_core)
FetchContent_GetProperties(volumetric_kit_sensor)
FetchContent_GetProperties(volumetric_kit_gfx)
# TODO: take the Orbbec SDK's version from sensor once its Orbbec tier exposes
# it; VS_ORBBEC_SDK_VERSION is sensor's own configure variable.
get_directory_property(
  _vc_orbbec_sdk_version DIRECTORY "${volumetric_kit_sensor_SOURCE_DIR}"
                                   DEFINITION VS_ORBBEC_SDK_VERSION)
set(_vc_software_cpp "${CMAKE_CURRENT_BINARY_DIR}/software.cpp")
set(_vc_software_command
    "${CMAKE_COMMAND}" "-DIN=${CMAKE_CURRENT_SOURCE_DIR}/software.cpp.in"
    "-DOUT=${_vc_software_cpp}" "-DCALIB_DIR=${PROJECT_SOURCE_DIR}"
    "-DCORE_DIR=${volumetric_kit_core_SOURCE_DIR}"
    "-DSENSOR_DIR=${volumetric_kit_sensor_SOURCE_DIR}"
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
  PRIVATE calib_mode.cpp
          calib_rig.cpp
          calibrate_worker.cpp
          capture_worker.cpp
          cloud_worker.cpp
          keys.cpp
          live_view.cpp
          live_worker.cpp
          open_source.cpp
          preset.cpp
          rig_mode.cpp
          rig_setup.cpp
          "${_vc_software_cpp}")
add_dependencies(calib_app calib_app_software)
# nlohmann/json reads a lens run's report for the Setup panel's picker,
# privately: no public header names it.
target_include_directories(calib_app SYSTEM PRIVATE "${VC_JSON_INCLUDE_DIR}")

# The public headers name the viz tier's mosaic, overlays, board editor and
# scene, the quality tier's metrics, lens checks and results files, the capture
# tier's session records, the detector's board, the source tier's pictures,
# sensor's array, frame preparation and calibration file, gfx's retire queue,
# depth-cloud pipeline and frame (the window's mosaic step), and the core's
# scenario. The window, the shared device and its requirements, the depth
# checks' kernels, the drivers (the opener's arms), OpenCV's thread pool and the
# threads stay private.
target_link_libraries(
  calib_app
  PUBLIC calib_viz
         calib_quality
         calib_capture
         calib_detect
         calib_source
         volumetric_kit::sensor_array
         volumetric_kit::sensor_prep
         volumetric_kit::core_config
         volumetric_kit::gfx_windowing
  PRIVATE calib_depth
          volumetric_kit::gfx_app
          volumetric_kit::gfx_glfw
          volumetric_kit::sensor_frame
          volumetric_kit::sensor_interface
          volumetric_kit::sensor_image_io
          opencv_core
          Threads::Threads)

# An arm per driver tier sensor built (src/CMakeLists.txt asks for one). The
# definitions are public, so a program or a test knows which scenarios
# check_scenario takes.
if(TARGET volumetric_kit::sensor_orbbec)
  target_sources(calib_app PRIVATE orbbec_arm.cpp)
  target_link_libraries(calib_app PRIVATE volumetric_kit::sensor_orbbec)
  target_compile_definitions(calib_app PUBLIC VC_WITH_ORBBEC)
endif()
if(TARGET volumetric_kit::sensor_decklink)
  target_sources(calib_app PRIVATE decklink_arm.cpp)
  target_link_libraries(calib_app PRIVATE volumetric_kit::sensor_decklink)
  target_compile_definitions(calib_app PUBLIC VC_WITH_DECKLINK)
endif()

vc_target_warnings(calib_app)
