# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Third-party dependencies.
#
# System via find_package (resolved at the top level so imported targets are
# visible across sibling subdirectories): Eigen3 (the calibration data model /
# solver math). Vendored via pinned FetchContent: nlohmann/json and stb for the
# capture tier's session files, and googletest when the test suite is built.
#
# TODO (as tiers land, mirroring gfx's hybrid system/vendored strategy): OpenCV
# via find_package for the detect tier (pattern detection + subpixel); Ceres via
# find_package for the solve tier (intrinsics / stereo / BA); Vulkan-Headers via
# FetchContent for the portable Vulkan-compute backend (VC_WITH_VULKAN, sharing
# the SPIR-V toolchain with volumetric_kit_gfx); and volumetric_kit_gfx via
# FetchContent for the viz / app tiers.

include(FetchContent)

if(VC_BUILD_TESTS)
  # googletest is only needed when building the test suite.
  set(INSTALL_GTEST
      OFF
      CACHE BOOL "" FORCE)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(googletest)
endif()

# --- nlohmann/json + stb (header-only; the capture tier's session files) ------
# A session is a session.json and one PNG per camera per capture. Both libraries
# are PRIVATE SYSTEM includes of calib_capture, never part of its installed
# interface, so they are exposed as include directories rather than targets (a
# static library's PRIVATE target would otherwise have to be exported too). The
# same pins as volumetric_kit_recon: json's release archive, hash-pinned, with
# its CMake not run (SOURCE_SUBDIR names a directory the archive does not have);
# stb has no tagged releases, so a commit.
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW) # extracted files get the extraction time
endif()
FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  URL_HASH
    SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
  SOURCE_SUBDIR do-not-add-json-cmake)
FetchContent_Declare(
  stb
  GIT_REPOSITORY https://github.com/nothings/stb.git
  GIT_TAG 2c980bb59875b0d32144a71867fbdebb2f77cd20
  GIT_SHALLOW FALSE)
FetchContent_MakeAvailable(nlohmann_json stb)

set(VC_JSON_INCLUDE_DIR
    "${nlohmann_json_SOURCE_DIR}/include"
    PARENT_SCOPE)
set(VC_STB_INCLUDE_DIR
    "${stb_SOURCE_DIR}"
    PARENT_SCOPE)
