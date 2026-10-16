# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_app: Tier 5 -------------------------------------------------------
# calib_rig's live view and capture: the rig's preset check, the key map, the
# acquisition and live-detection worker (T1), the capture worker (T3) and the
# window loop (T0) on gfx's GLFW glue, ImGui overlay and the viz tier's mosaic.
# Built with the viz tier when gfx_glfw (an installed GLFW) and recon's Orbbec
# tier (VR_WITH_ORBBEC) exist (src/CMakeLists.txt). Build-tree only, as
# calib_viz is.
include(GenerateExportHeader)

# calib_rig loads its scenario as config::Application::CalibRig, which the core
# has from 0.11.0, in its config tier (third_party/CMakeLists.txt turns it on).
vkc_require_core(0.11.0 CONFIG)

set(_vc_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

# What a session records it was captured with (capture::Software): each fetched
# sibling's commit, read from its checkout at configure time (empty where it is
# not a git checkout), and the Orbbec SDK's version, from recon's configure.
include(FetchContent)
function(_vc_checkout_commit out source_dir)
  set(_sha "")
  find_package(Git QUIET)
  if(GIT_FOUND AND EXISTS "${source_dir}")
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" -C "${source_dir}" rev-parse HEAD
      OUTPUT_VARIABLE _sha
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _rc
      ERROR_QUIET)
    if(NOT _rc EQUAL 0)
      set(_sha "")
    endif()
  endif()
  set(${out}
      "${_sha}"
      PARENT_SCOPE)
endfunction()
FetchContent_GetProperties(volumetric_kit_core)
FetchContent_GetProperties(volumetric_kit_recon)
FetchContent_GetProperties(volumetric_kit_gfx)
_vc_checkout_commit(VC_APP_CALIB_COMMIT "${PROJECT_SOURCE_DIR}")
_vc_checkout_commit(VC_APP_CORE_COMMIT "${volumetric_kit_core_SOURCE_DIR}")
_vc_checkout_commit(VC_APP_RECON_COMMIT "${volumetric_kit_recon_SOURCE_DIR}")
_vc_checkout_commit(VC_APP_GFX_COMMIT "${volumetric_kit_gfx_SOURCE_DIR}")
set(_vc_recon_dir "${volumetric_kit_recon_SOURCE_DIR}")
get_directory_property(
  VC_APP_ORBBEC_SDK_VERSION DIRECTORY "${_vc_recon_dir}" DEFINITION
                                      VR_ORBBEC_SDK_VERSION)
configure_file(software.cpp.in "${CMAKE_CURRENT_BINARY_DIR}/software.cpp" @ONLY)

add_library(calib_app)
add_library(volumetric_kit::calib_app ALIAS calib_app)

target_sources(
  calib_app
  PRIVATE calib_rig.cpp
          capture_worker.cpp
          keys.cpp
          live_view.cpp
          live_worker.cpp
          preset.cpp
          "${CMAKE_CURRENT_BINARY_DIR}/software.cpp")

# The tier's own export macro and guard (see calib_core).
generate_export_header(
  calib_app
  BASE_NAME
  VC_APP
  EXPORT_MACRO_NAME
  VC_APP_API
  EXPORT_FILE_NAME
  "${_vc_generated_inc}/volumetric_kit/calib/app/export.hpp")

set_target_properties(
  calib_app
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vc_type calib_app TYPE)
if(_vc_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(calib_app PUBLIC VC_APP_STATIC_DEFINE)
endif()

target_include_directories(calib_app
                           PUBLIC "$<BUILD_INTERFACE:${_vc_generated_inc}>")

# The public headers name the viz tier's mosaic and overlays, the capture tier's
# session records, the detector's board, recon's sensor array, picture readback
# and Orbbec options, gfx's retire queue, and the core's scenario. The window,
# the shared device, OpenCV's thread pool and the threads stay private.
target_link_libraries(
  calib_app
  PUBLIC calib_viz
         calib_capture
         calib_detect
         volumetric_kit::recon_sensor_array
         volumetric_kit::recon_sensor_utils
         volumetric_kit::recon_sensor_orbbec
         volumetric_kit::core_config
  PRIVATE volumetric_kit::gfx_app volumetric_kit::gfx_glfw
          volumetric_kit::recon_core opencv_core Threads::Threads)

vc_target_warnings(calib_app)
