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

# Semicolon list of sanitizers (e.g. "address;undefined"); empty = off. Applied
# globally — see cmake/vc_sanitizers.cmake for why.
set(VC_SANITIZE
    ""
    CACHE STRING "Sanitizers to enable, e.g. address;undefined (empty = off)")

include(vc_warnings)
include(vc_sanitizers)

# --- Public dependencies -----------------------------------------------------
# Resolved at the top level so their imported targets are visible to every
# subdirectory, not just the one find_package() ran in. Eigen is the
# linear-algebra spine of the calibration data model and solvers; its config
# defines the Eigen3::Eigen interface target.
#
# No version pin: Eigen's ConfigVersion file is SameMajorVersion-compatible, so
# a minimum like "3.3" is rejected against a 4.x/5.x system install. The public
# usage here is just <Eigen/Core> (stable across major versions), so accept
# whatever is installed.
#
# TODO: add a pinned vendored FetchContent fallback (mirroring how gfx vendors
# glm) so CI runners without a system Eigen stay self-contained. Today a system
# Eigen is required (macOS: `brew install eigen`; Ubuntu: `libeigen3-dev`).
find_package(Eigen3 REQUIRED CONFIG)

# --- Dependencies + library --------------------------------------------------
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
