# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

cmake_minimum_required(VERSION 3.21)

project(
  volumetric_kit_calib
  VERSION 0.0.1
  DESCRIPTION
    "Calibration companion for volumetric_kit: diagnostics, acquisition guidance, and a requirements solver"
  HOMEPAGE_URL "https://github.com/taojin-6/volumetric_kit_calib"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Are we the top-level project, or pulled in via add_subdirectory/FetchContent?
# Defaults for "build the extras" and "generate install rules" follow from this.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(VC_IS_TOP_LEVEL ON)
else()
  set(VC_IS_TOP_LEVEL OFF)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# --- Options -----------------------------------------------------------------
option(VC_BUILD_EXAMPLES "Build example programs" ${VC_IS_TOP_LEVEL})
option(VC_BUILD_TESTS "Build tests" ${VC_IS_TOP_LEVEL})
option(VC_INSTALL "Generate install/export rules" ${VC_IS_TOP_LEVEL})
# No GPU options: calib's GPU work runs on volumetric_kit_core's vulkan tier, on
# a VkDevice shared with recon and gfx (DECISIONS.md), not on a backend of its
# own. On by default only at the top level: a consumer that pulls us in via
# FetchContent with a newer compiler should see our new warnings, not a broken
# build.
option(VC_WARNINGS_AS_ERRORS "Treat first-party compiler warnings as errors"
       ${VC_IS_TOP_LEVEL})
# OFF by default: OpenCV is a prerequisite you install, and a build without it
# still has the core tier.
option(
  VC_WITH_OPENCV
  "Build the OpenCV tiers, from compute on (requires an installed OpenCV 5)"
  OFF)

# Semicolon list of sanitizers (e.g. "address;undefined"); empty = off. Applied
# globally — see cmake/vc_sanitizers.cmake for why.
set(VC_SANITIZE
    ""
    CACHE STRING "Sanitizers to enable, e.g. address;undefined (empty = off)")

include(vc_warnings)
include(vc_sanitizers)

# --- Dependencies + library --------------------------------------------------
# The oldest volumetric_kit_core calib builds with (DECISIONS.md, 2026-10-12):
# checked in the core that arrives (third_party/) and again by the installed
# package config.
set(VC_CORE_MIN_VERSION 0.2.0)
# OpenCV 5, for the compute tier and those built on it (DECISIONS.md,
# 2026-10-12): found here so its imported targets reach every tier.
if(VC_WITH_OPENCV)
  find_package(OpenCV 5.0 CONFIG COMPONENTS core imgproc objdetect geometry)
  if(NOT OpenCV_FOUND)
    message(
      FATAL_ERROR
        "VC_WITH_OPENCV is ON but OpenCV 5 (core, imgproc, objdetect, "
        "geometry) was not found. Install it (brew install opencv on macOS) "
        "and, if it is not on CMake's search path, point the build at it "
        "with -DOpenCV_DIR=<dir holding OpenCVConfig.cmake>; or configure "
        "with -DVC_WITH_OPENCV=OFF.")
  endif()
endif()
add_subdirectory(third_party)
add_subdirectory(src)

if(VC_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# Guard: examples/ is optional and may be absent in a slimmed checkout.
if(VC_BUILD_EXAMPLES AND EXISTS
                         "${CMAKE_CURRENT_SOURCE_DIR}/examples/CMakeLists.txt")
  add_subdirectory(examples)
endif()
