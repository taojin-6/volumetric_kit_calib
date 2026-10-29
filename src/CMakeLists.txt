# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Orchestrator for the component tiers. Each subdirectory owns its own target
# and build settings; the strict dependency rule (a tier may depend only on
# tiers to its left) is expressed by what each links. calib is consumed by
# commit, from its build tree, and installs nothing (DECISIONS.md, 2026-10-22).
add_subdirectory(volumetric_kit/calib/core)
add_subdirectory(volumetric_kit/calib/compute)
add_subdirectory(volumetric_kit/calib/detect)
add_subdirectory(volumetric_kit/calib/solve)
add_subdirectory(volumetric_kit/calib/quality)
if(VC_WITH_RIG)
  add_subdirectory(volumetric_kit/calib/capture)
  add_subdirectory(volumetric_kit/calib/depth)
  add_subdirectory(volumetric_kit/calib/viz)
endif()
# The app tier, calib_rig's (VC_BUILD_APPS), also needs gfx's GLFW glue (an
# installed GLFW) and sensor's Orbbec driver (VS_WITH_ORBBEC).
if(VC_WITH_RIG AND VC_BUILD_APPS)
  set(_vc_app_needs volumetric_kit::gfx_glfw volumetric_kit::sensor_orbbec)
  set(_vc_build_app ON)
  foreach(_vc_need IN LISTS _vc_app_needs)
    if(NOT TARGET ${_vc_need})
      set(_vc_build_app OFF)
    endif()
  endforeach()
  if(_vc_build_app)
    add_subdirectory(volumetric_kit/calib/app)
  else()
    message(STATUS "calib_app and calib_rig: skipped (they need gfx_glfw, "
                   "from an installed GLFW, and sensor's VS_WITH_ORBBEC)")
  endif()
endif()
# TODO: add_subdirectory() for color as that tier lands (see DESIGN.md).

# --- calib: umbrella alias ---------------------------------------------------
# Consumers link `volumetric_kit::calib` and get whatever tiers are built.
add_library(calib INTERFACE)
add_library(volumetric_kit::calib ALIAS calib)
target_link_libraries(calib INTERFACE calib_core calib_compute calib_detect
                                      calib_solve calib_quality)
if(VC_WITH_RIG)
  target_link_libraries(calib INTERFACE calib_capture calib_depth calib_viz)
endif()
if(TARGET calib_app)
  target_link_libraries(calib INTERFACE calib_app)
endif()
