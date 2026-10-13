# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Third-party dependencies.
#
# Vendored via pinned FetchContent: volumetric_kit_core (the family's shared
# foundation), volumetric_kit_recon's camera tier (the family's camera
# vocabulary) -- all of recon with VC_WITH_RIG -- nlohmann/json for the capture
# tier's files, and googletest when the test suite is built. recon takes GLM,
# and with VC_WITH_RIG the Vulkan SDK, from the system. OpenCV, with
# VC_WITH_OPENCV, is a prerequisite the root CMakeLists finds.
#
# TODO (as tiers land, mirroring gfx's hybrid system/vendored strategy): Ceres
# via find_package for the solve tier (intrinsics / stereo / BA); and
# volumetric_kit_gfx via FetchContent for the viz / app tiers. GPU work comes
# from volumetric_kit_core's vulkan tier, not a Vulkan dependency of calib's.

include(FetchContent)

# Only applications need the common JSON format. The calibration library keeps
# its dependency on core_base and recon_camera alone.
if(VC_BUILD_EXAMPLES)
  set(VKC_WITH_CONFIG ON)
endif()

# --- volumetric_kit_core -----------------------------------------------------
# The family's shared foundation: calib's Status / Result / VKC_CHECK / log sink
# come from its base tier (DECISIONS.md, "Build on volumetric_kit_core"). Pinned
# by commit, never main, so every consumer builds the same core. An application
# that also fetches recon or gfx declares the core first; FetchContent keeps the
# first declaration of a name, so this pin then yields to the application's, and
# vkc_require_core refuses that core if it is older than the oldest calib builds
# with. Work on both checkouts at once with
# -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<core checkout>.
#
# The core's tests and -Werror stay off in a subproject; its install rules stay
# on, so calib's install carries the core its exported targets link. The brain
# needs only the base tier, so it builds with no Vulkan SDK: calib turns the
# vulkan tier on only with VC_WITH_RIG, for recon's sensor tier, and otherwise
# leaves VKC_WITH_VULKAN unset rather than OFF, so an application that turns the
# tier on for recon or gfx keeps it. Only examples opt into core_config.
#
# The pin is core PR #32's head, 0.6.1, which merges first: its package can be
# read twice in one configure (DECISIONS.md, 2026-10-13).
#
# TODO: re-pin core main, or v0.6.1, once core PR #32 merges.
if(VC_WITH_RIG)
  set(VKC_WITH_VULKAN ON)
endif()
FetchContent_Declare(
  volumetric_kit_core
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_core.git
  GIT_TAG 278d449cc5ce543970d0dd65ecd3ab4f92397f73)
FetchContent_MakeAvailable(volumetric_kit_core)
# The set() above is lost when the application made the core available first.
if(VC_WITH_RIG)
  vkc_require_core(${VC_CORE_MIN_VERSION} VULKAN)
else()
  vkc_require_core(${VC_CORE_MIN_VERSION})
endif()
if(VC_BUILD_EXAMPLES AND NOT TARGET volumetric_kit::core_config)
  message(
    FATAL_ERROR "calib examples require volumetric_kit_core with core_config")
endif()

# --- volumetric_kit_recon: the camera tier, or all of it ---------------------
# calib's cameras are recon's CameraModel, with its rigid transforms
# (DECISIONS.md, "The camera model is recon's"). VR_CAMERA_ONLY builds
# recon_camera alone, over the core's base tier and GLM, so calib still builds
# with no Vulkan SDK. Pinned by commit, never main, and declared after the core,
# so recon's own core pin yields to calib's. recon's tests, examples and -Werror
# stay off in a subproject; its install follows calib's, as calib's exported
# targets link recon_camera. It leaves an application's VR_CAMERA_ONLY as set,
# so an application that turns it off keeps the rest of recon.
FetchContent_Declare(
  volumetric_kit_recon
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_recon.git
  GIT_TAG 2f8d6eaf2d7cec17f3815240c27f7450095d4a12)
# VC_WITH_RIG takes all of recon: the capture tier links its sensor and io
# tiers. An application's own VR_WITH_ORBBEC / VR_WITH_FFMPEG still apply.
if(VC_WITH_RIG)
  set(VR_CAMERA_ONLY OFF)
elseif(NOT DEFINED VR_CAMERA_ONLY)
  set(VR_CAMERA_ONLY ON)
endif()
set(VR_INSTALL ${VC_INSTALL})
FetchContent_MakeAvailable(volumetric_kit_recon)
# The set() above is lost when the application made recon available first.
if(VC_WITH_RIG AND NOT TARGET volumetric_kit::recon_io)
  message(
    FATAL_ERROR
      "VC_WITH_RIG needs all of volumetric_kit_recon, but it was made "
      "available camera-only: set VR_CAMERA_ONLY OFF before the build's first "
      "FetchContent_MakeAvailable(volumetric_kit_recon).")
endif()

# --- nlohmann/json (header-only; the capture tier's files) -------------------
# calib_capture writes its session and capture files with it, as a PRIVATE
# SYSTEM include, never part of its installed interface. Declared under recon's
# name and pin, after recon, so the one copy recon fetched is the one used.
if(VC_WITH_RIG)
  if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW) # extracted files get the extraction time
  endif()
  FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
    URL_HASH
      SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
    SOURCE_SUBDIR do-not-add-json-cmake)
  FetchContent_MakeAvailable(nlohmann_json)
  set(VC_JSON_INCLUDE_DIR
      "${nlohmann_json_SOURCE_DIR}/include"
      PARENT_SCOPE)
endif()

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
