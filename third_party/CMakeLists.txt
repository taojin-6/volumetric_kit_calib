# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Third-party dependencies.
#
# Vendored via pinned FetchContent: volumetric_kit_core (the family's shared
# foundation), volumetric_kit_recon's camera tier (the family's camera
# vocabulary), and googletest when the test suite is built. recon's camera tier
# takes GLM from the system. OpenCV, with VC_WITH_OPENCV, is a prerequisite the
# root CMakeLists finds.
#
# TODO (as tiers land, mirroring gfx's hybrid system/vendored strategy): Ceres
# via find_package for the solve tier (intrinsics / stereo / BA); the rest of
# volumetric_kit_recon for the capture tier (its sensor tier's drivers); and
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
# The core's tests, -Werror and vulkan tier stay off in a subproject; its
# install rules stay on, so calib's install carries the core its exported
# targets link. calib needs only the base tier, so it builds with no Vulkan SDK.
# It leaves VKC_WITH_VULKAN unset rather than OFF, so an application that turns
# the tier on for recon or gfx keeps it.
#
# The pin includes core_config (0.3.0). Only examples opt into this optional
# tier; library-only consumers still require only core_base >= 0.1.0.
FetchContent_Declare(
  volumetric_kit_core
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_core.git
  GIT_TAG 7857a6d4b9973b32461af09062d0bb894c86c751)
FetchContent_MakeAvailable(volumetric_kit_core)
vkc_require_core(${VC_CORE_MIN_VERSION})
if(VC_BUILD_EXAMPLES AND NOT TARGET volumetric_kit::core_config)
  message(
    FATAL_ERROR "calib examples require volumetric_kit_core with core_config")
endif()

# --- volumetric_kit_recon: the camera tier alone ----------------------------
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
if(NOT DEFINED VR_CAMERA_ONLY)
  set(VR_CAMERA_ONLY ON)
endif()
set(VR_INSTALL ${VC_INSTALL})
FetchContent_MakeAvailable(volumetric_kit_recon)

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
