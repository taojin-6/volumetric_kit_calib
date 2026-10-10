# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_capture: Tier 1, off core -----------------------------------------
# The capture contract (IRigSource: sets of frames one trigger apart, pixels
# read on demand), sessions on disk (session.json + one PNG per camera per
# capture), a replay source over a session, and the recorder that turns a key
# press into an averaged capture. Pure CPU: a live source that keeps its frames
# on the GPU implements IRigSource in a target of its own.
include(GenerateExportHeader)
include(GNUInstallDirs)

set(_vc_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(calib_capture)
add_library(volumetric_kit::calib_capture ALIAS calib_capture)

target_sources(
  calib_capture PRIVATE capture_recorder.cpp png_io.cpp rig_source.cpp
                        session.cpp session_replay.cpp)

# png_io.cpp compiles stb's implementation, which is third-party: build it with
# warnings off, apart from the tier's -Werror surface (png_io.cpp explains).
set_source_files_properties(png_io.cpp PROPERTIES COMPILE_OPTIONS -w)

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

# With a static archive, VC_CAPTURE_API must expand to nothing for consumers.
get_target_property(_vc_type calib_capture TYPE)
if(_vc_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(calib_capture PUBLIC VC_CAPTURE_STATIC_DEFINE)
endif()

target_compile_features(calib_capture PUBLIC cxx_std_17)

target_include_directories(
  calib_capture
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${_vc_generated_inc}>"
         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
# nlohmann/json and stb are PRIVATE SYSTEM includes: session.cpp parses and
# writes with them, and no public header names them (third_party explains).
target_include_directories(calib_capture SYSTEM PRIVATE "${VC_JSON_INCLUDE_DIR}"
                                                        "${VC_STB_INCLUDE_DIR}")

target_link_libraries(calib_capture PUBLIC calib_core)

vc_target_warnings(calib_capture)

if(VC_INSTALL)
  install(
    TARGETS calib_capture
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
