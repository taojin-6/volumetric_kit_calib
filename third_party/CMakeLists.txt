# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Third-party dependencies.
#
# System via find_package (resolved at the top level so imported targets are
# visible across sibling subdirectories): Eigen3 (the calibration data model /
# solver math). Vendored via pinned FetchContent: volumetric_kit_core (the
# family's shared foundation), and googletest when the test suite is built.
#
# TODO (as tiers land, mirroring gfx's hybrid system/vendored strategy): OpenCV
# via find_package for the detect tier (pattern detection + subpixel); Ceres via
# find_package for the solve tier (intrinsics / stereo / BA);
# volumetric_kit_recon via FetchContent for the capture tier (its sensor tier's
# drivers); and volumetric_kit_gfx via FetchContent for the viz / app tiers. GPU
# work comes from volumetric_kit_core's vulkan tier, not a Vulkan dependency of
# calib's.

include(FetchContent)

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
# The pin is core 0.1.0: main after core PR #21, which gave vkc_require_core the
# PACKAGE mode calib's package config uses.
#
# TODO: pin the v0.1.0 tag once the core tags it.
FetchContent_Declare(
  volumetric_kit_core
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_core.git
  GIT_TAG ce76978a9b829313e911b7a47b4110e2a02ed69f)
FetchContent_MakeAvailable(volumetric_kit_core)
vkc_require_core(${VC_CORE_MIN_VERSION})

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
