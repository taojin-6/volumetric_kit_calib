# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- calib_core: Tier 0 ------------------------------------------------------
# Foundation tier: the version API and the data model (board target,
# observation, view and capture records, lens and sensor profiles), with error
# handling from volumetric_kit_core's base tier. Every other tier depends on
# this; it depends only on the core's base tier and recon's camera tier, which
# its public headers name. Pure CPU, no GPU and no OpenCV/Ceres, so it stays
# usable from a tool or a device-less test.
include(GenerateExportHeader)
include(GNUInstallDirs)

set(_vc_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(calib_core)
add_library(volumetric_kit::calib_core ALIAS calib_core)

target_sources(calib_core PRIVATE target.cpp version.cpp)

# Generated headers: the VC_CORE_API export macro + the configured version
# header. The export header is per-tier (core/export.hpp + VC_CORE_API +
# VC_CORE_STATIC_DEFINE) so each future tier generates its own isolated
# macro/guard and they never collide on one path or mis-attribute symbols across
# shared libraries.
generate_export_header(
  calib_core
  BASE_NAME
  VC_CORE
  EXPORT_MACRO_NAME
  VC_CORE_API
  EXPORT_FILE_NAME
  "${_vc_generated_inc}/volumetric_kit/calib/core/export.hpp")
configure_file(
  "${PROJECT_SOURCE_DIR}/include/volumetric_kit/calib/version.hpp.in"
  "${_vc_generated_inc}/volumetric_kit/calib/version.hpp" @ONLY)

set_target_properties(
  calib_core
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

# With a static archive, VC_CORE_API must expand to nothing for consumers.
get_target_property(_vc_type calib_core TYPE)
if(_vc_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(calib_core PUBLIC VC_CORE_STATIC_DEFINE)
endif()

target_compile_features(calib_core PUBLIC cxx_std_17)

target_include_directories(
  calib_core
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${_vc_generated_inc}>"
         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

# Both are PUBLIC: the public headers name the core's Status / Result and
# recon's camera types in their signatures, so consumers compile against them
# too. The installed package re-finds recon, and the core through recon's
# package config.
target_link_libraries(calib_core PUBLIC volumetric_kit::core_base
                                        volumetric_kit::recon_camera)

vc_target_warnings(calib_core)

# Install the target into the shared export set + the headers this tier
# generates. The umbrella target, public-header tree, and package config are
# finalized by the parent (src/CMakeLists.txt).
if(VC_INSTALL)
  install(
    TARGETS calib_core
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
