# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Orchestrator for the component tiers. Each subdirectory owns its own target
# and build settings and installs itself into the shared
# volumetric_kit_calibTargets export set; the strict dependency rule (a tier may
# depend only on tiers to its left) is expressed by what each links.
include(GNUInstallDirs)

add_subdirectory(volumetric_kit/calib/core)
add_subdirectory(volumetric_kit/calib/capture)
# TODO: add_subdirectory() for compute / detect / solve / color / quality / viz
# / app as those tiers land (see DESIGN.md). compute needs the Vulkan/CUDA
# backends; detect/solve need OpenCV/Ceres; viz/app need volumetric_kit_gfx.

# --- calib: umbrella alias ---------------------------------------------------
# Consumers link `volumetric_kit::calib` and get whatever tiers are built.
add_library(calib INTERFACE)
add_library(volumetric_kit::calib ALIAS calib)
target_link_libraries(calib INTERFACE calib_core calib_capture)

# --- Install / export --------------------------------------------------------
# Tiers install their own targets into volumetric_kit_calibTargets (above); here
# we add the umbrella, ship the public header tree, and finalize the export set
# and package config.
if(VC_INSTALL)
  install(TARGETS calib EXPORT volumetric_kit_calibTargets)

  install(
    DIRECTORY "${PROJECT_SOURCE_DIR}/include/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING
    PATTERN "*.hpp")

  install(
    EXPORT volumetric_kit_calibTargets
    NAMESPACE volumetric_kit::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/volumetric_kit_calib")

  # The installed package re-finds the core at the MAJOR.MINOR this build
  # fetched: the core's version file accepts only the same minor before 1.0.
  get_target_property(_vc_core_version volumetric_kit::core_base VERSION)
  string(REGEX MATCH "^[0-9]+\\.[0-9]+" VC_CORE_MAJOR_MINOR
               "${_vc_core_version}")
  if(NOT VC_CORE_MAJOR_MINOR)
    message(FATAL_ERROR "volumetric_kit::core_base has no VERSION to require")
  endif()

  include(CMakePackageConfigHelpers)
  configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/volumetric_kit_calibConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/volumetric_kit_calibConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/volumetric_kit_calib")
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/volumetric_kit_calibConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion)
  install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/volumetric_kit_calibConfig.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/volumetric_kit_calibConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/volumetric_kit_calib")
endif()
