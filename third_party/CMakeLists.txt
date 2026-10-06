# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Third-party dependencies.
#
# System via find_package (resolved at the top level so imported targets are
# visible across sibling subdirectories): Eigen3 (the calibration data model /
# solver math). Vendored via pinned FetchContent: volumetric_kit_core (the
# family's shared foundation), volumetric_kit_recon (the family's camera model
# and calibration file), and googletest when the test suite is built.
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
# The core's tests and -Werror stay off in a subproject; its install rules stay
# on, so calib's install carries the core its exported targets link. Its vulkan
# tier is on: recon, below, builds on it, so configuring calib needs the Vulkan
# headers and loader (and a GLSL compiler, for recon's other tiers).
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
set(VKC_WITH_VULKAN ON)
FetchContent_MakeAvailable(volumetric_kit_core)

# --- volumetric_kit_recon ----------------------------------------------------
# The family's camera vocabulary is recon's camera tier (recon_camera): the
# camera model calib solves for, rigid transforms, and the sensor array's
# calibration file calib writes and recon's captures read (DECISIONS.md, "The
# camera model is recon's"). Pinned by commit, never main, and declared after
# the core, so recon's own core pin yields to calib's.
#
# recon_camera links the core's base tier and GLM and no Vulkan, but recon's
# configure finds Vulkan and a GLSL compiler for its other tiers. Its tests,
# examples and -Werror stay off. calib installs recon beside itself, as its
# exported targets link recon_camera; a build that installs nothing (an
# application embedding calib, the -fno-exceptions CI leg) builds only the recon
# targets it links (EXCLUDE_FROM_ALL, CMake 3.28+), which keeps recon's GPU
# tiers out of builds that never use them.
#
# The pin is recon PR #180 (the camera tier builds without exceptions), over
# #175, which landed the tier. TODO: re-pin to main once #180 merges.
set(_vc_recon_exclude)
if(NOT VC_INSTALL AND CMAKE_VERSION VERSION_GREATER_EQUAL 3.28)
  set(_vc_recon_exclude EXCLUDE_FROM_ALL)
endif()
FetchContent_Declare(
  volumetric_kit_recon
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_recon.git
  GIT_TAG 5b25dae4e79b8b9917e545622e378285bb9a3c1a
  ${_vc_recon_exclude})
set(VR_INSTALL ${VC_INSTALL})
set(VR_WARNINGS_AS_ERRORS OFF)
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
