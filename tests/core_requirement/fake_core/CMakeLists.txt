# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# A stand-in volumetric_kit_core at VC_FAKE_CORE_VERSION, which calib's
# configure takes in place of the pinned one (configure.cmake): the base-tier
# target calib links, exported as the real one is and carrying that version as
# the real one does, and the real core's vkc_require_core from
# VC_FAKE_CORE_MODULES. Version "none" stands in for a core older than 0.1.0,
# which has neither.
cmake_minimum_required(VERSION 3.21)
project(volumetric_kit_core LANGUAGES NONE)

add_library(core_base INTERFACE)
add_library(volumetric_kit::core_base ALIAS core_base)
install(TARGETS core_base EXPORT volumetric_kit_coreTargets)
install(
  EXPORT volumetric_kit_coreTargets
  NAMESPACE volumetric_kit::
  DESTINATION lib/cmake/volumetric_kit_core)

if(NOT VC_FAKE_CORE_VERSION STREQUAL "none")
  set_target_properties(core_base PROPERTIES VKC_CORE_VERSION
                                             "${VC_FAKE_CORE_VERSION}")
  include("${VC_FAKE_CORE_MODULES}/vkc_require_core.cmake")
endif()
