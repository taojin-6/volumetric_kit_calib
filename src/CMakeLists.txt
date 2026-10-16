# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Orchestrator for the component tiers. Each subdirectory owns its own target
# and build settings and installs itself into the shared
# volumetric_kit_calibTargets export set; the strict dependency rule (a tier may
# depend only on tiers to its left) is expressed by what each links.
include(GNUInstallDirs)

add_subdirectory(volumetric_kit/calib/core)
if(VC_WITH_OPENCV)
  add_subdirectory(volumetric_kit/calib/compute)
  add_subdirectory(volumetric_kit/calib/detect)
  add_subdirectory(volumetric_kit/calib/solve)
  add_subdirectory(volumetric_kit/calib/quality)
endif()
if(VC_WITH_RIG)
  add_subdirectory(volumetric_kit/calib/capture)
endif()
if(VC_WITH_VIZ)
  add_subdirectory(volumetric_kit/calib/viz)
endif()
# TODO: add_subdirectory() for color / app as those tiers land (see DESIGN.md).

# --- calib: umbrella alias ---------------------------------------------------
# Consumers link `volumetric_kit::calib` and get whatever tiers are built.
add_library(calib INTERFACE)
add_library(volumetric_kit::calib ALIAS calib)
target_link_libraries(calib INTERFACE calib_core)
if(VC_WITH_OPENCV)
  target_link_libraries(calib INTERFACE calib_compute calib_detect calib_solve
                                        calib_quality)
endif()
if(VC_WITH_RIG)
  target_link_libraries(calib INTERFACE calib_capture)
endif()
# calib_viz is build-tree only (viz/CMakeLists.txt), so it joins the umbrella
# for in-tree consumers without entering the installed export set.
if(VC_WITH_VIZ)
  target_link_libraries(calib INTERFACE $<BUILD_INTERFACE:calib_viz>)
endif()

# --- Install / export --------------------------------------------------------
# Tiers install their own targets into volumetric_kit_calibTargets (above); here
# we add the umbrella, ship the public header tree, and finalize the export set
# and package config.
if(VC_INSTALL)
  install(TARGETS calib EXPORT volumetric_kit_calibTargets)

  # A tier's headers ship only when the tier is built.
  set(_vc_unbuilt_headers)
  if(NOT VC_WITH_OPENCV)
    foreach(_vc_tier compute detect solve quality)
      list(APPEND _vc_unbuilt_headers PATTERN "${_vc_tier}" EXCLUDE)
    endforeach()
  endif()
  if(NOT VC_WITH_RIG)
    list(APPEND _vc_unbuilt_headers PATTERN "capture" EXCLUDE)
  endif()
  # viz is never installed (viz/CMakeLists.txt).
  list(APPEND _vc_unbuilt_headers PATTERN "viz" EXCLUDE)
  install(
    DIRECTORY "${PROJECT_SOURCE_DIR}/include/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING
    PATTERN "*.hpp" ${_vc_unbuilt_headers})

  install(
    EXPORT volumetric_kit_calibTargets
    NAMESPACE volumetric_kit::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/volumetric_kit_calib")

  # The installed package finds the core this build fetched, at its version: the
  # core's version file accepts only the same minor, no older, before 1.0.
  vkc_core_version(VC_CORE_VERSION)

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
