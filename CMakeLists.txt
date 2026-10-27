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
# The defaults for "build the extras" follow from this.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(VC_IS_TOP_LEVEL ON)
else()
  set(VC_IS_TOP_LEVEL OFF)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# --- Options -----------------------------------------------------------------
option(VC_BUILD_EXAMPLES "Build example programs" ${VC_IS_TOP_LEVEL})
option(VC_BUILD_TESTS "Build tests" ${VC_IS_TOP_LEVEL})
# No GPU options: calib's GPU work runs on volumetric_kit_core's vulkan tier, on
# a VkDevice shared with recon and gfx (DECISIONS.md), not on a backend of its
# own. On by default only at the top level: a consumer that pulls us in via
# FetchContent with a newer compiler should see our new warnings, not a broken
# build.
option(VC_WARNINGS_AS_ERRORS "Treat first-party compiler warnings as errors"
       ${VC_IS_TOP_LEVEL})
# OFF by default: the rig tiers -- capture, depth and viz -- take all of recon,
# which needs the Vulkan SDK (DECISIONS.md, 2026-10-13), and viz fetches
# volumetric_kit_gfx.
option(VC_WITH_RIG
       "Build the rig tiers, capture, depth and viz (all of recon; Vulkan SDK)"
       OFF)
# calib_rig and its app tier, an application: by default only at the top level,
# as the examples are, so a project that adds calib builds neither. It needs the
# viz tier (VC_WITH_RIG above), GLFW and recon's Orbbec driver too
# (src/CMakeLists.txt).
option(
  VC_BUILD_APPS
  "Build calib_rig and its app tier (with VC_WITH_RIG; needs GLFW and recon's VR_WITH_ORBBEC)"
  ${VC_IS_TOP_LEVEL})

# Semicolon list of sanitizers (e.g. "address;undefined"); empty = off. Applied
# globally — see cmake/vc_sanitizers.cmake for why.
set(VC_SANITIZE
    ""
    CACHE STRING "Sanitizers to enable, e.g. address;undefined (empty = off)")

include(vc_warnings)
include(vc_sanitizers)

# --- Dependencies + library --------------------------------------------------
# The oldest volumetric_kit_core calib builds with (DECISIONS.md, 2026-10-13),
# checked in the core that arrives (third_party/).
set(VC_CORE_MIN_VERSION 0.25.0)
# OpenCV 5, a prerequisite of the compute tier and those built on it
# (DECISIONS.md, 2026-10-17): Homebrew's on macOS, tools/install-opencv.sh's on
# Linux (README, "Prerequisites"). Found here so its imported targets reach
# every tier.
find_package(OpenCV 5.0 CONFIG REQUIRED COMPONENTS core imgproc objdetect
                                                   geometry)
# The capture tier's writer thread; found here, as OpenCV is, so the imported
# target reaches the tests that link the tier.
if(VC_WITH_RIG)
  find_package(Threads REQUIRED)
endif()
add_subdirectory(third_party)
add_subdirectory(src)

if(VC_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# calib_rig, the live rig calibration application, built with its tier
# (VC_BUILD_APPS).
if(TARGET calib_app)
  add_subdirectory(apps)
endif()

# Guard: examples/ is optional and may be absent in a slimmed checkout.
if(VC_BUILD_EXAMPLES AND EXISTS
                         "${CMAKE_CURRENT_SOURCE_DIR}/examples/CMakeLists.txt")
  add_subdirectory(examples)
endif()
