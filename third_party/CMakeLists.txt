# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Third-party dependencies.
#
# System via find_package (resolved at the top level so imported targets are
# visible across sibling subdirectories): Eigen3 (the calibration data model /
# solver math). Vendored via pinned FetchContent: volumetric_kit_core (the
# family's shared foundation), nlohmann/json and stb for the capture tier's
# session files, and googletest when the test suite is built.
#
# TODO (as tiers land, mirroring gfx's hybrid system/vendored strategy): OpenCV
# via find_package for the detect tier (pattern detection + subpixel); Ceres via
# find_package for the solve tier (intrinsics / stereo / BA); and
# volumetric_kit_gfx via FetchContent for the viz / app tiers. GPU work comes
# from volumetric_kit_core's vulkan tier, not a Vulkan dependency of calib's.

include(FetchContent)

# --- volumetric_kit_core -----------------------------------------------------
# The family's shared foundation: calib's Status / Result / VKC_CHECK / log sink
# come from its base tier (DECISIONS.md, "Build on volumetric_kit_core"). Pinned
# by commit, never main, so every consumer builds the same core. An application
# that also fetches recon or gfx declares the core first; FetchContent keeps the
# first declaration of a name, so this pin then yields to the application's.
# Work on both checkouts at once with
# -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<core checkout>.
#
# The core's tests, -Werror and vulkan tier stay off in a subproject; its
# install rules stay on, so calib's install carries the core its exported
# targets link. calib needs only the base tier, so it builds with no Vulkan SDK.
# It leaves VKC_WITH_VULKAN unset rather than OFF, so an application that turns
# the tier on for recon or gfx keeps it.
#
# The pin is main after core PR #13. Since PR #1, which landed the base tier,
# the core has gained its vulkan tier; the base tier kept its API (the core's
# CHANGELOG.md).
#
# TODO: pin a release tag once the core tags one (v0.1.0).
FetchContent_Declare(
  volumetric_kit_core
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_core.git
  GIT_TAG a60398b7aeb2cad722a3f79cb71696bda10fc054)
FetchContent_MakeAvailable(volumetric_kit_core)

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
