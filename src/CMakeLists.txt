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
  add_subdirectory(volumetric_kit/calib/source)
  add_subdirectory(volumetric_kit/calib/capture)
  add_subdirectory(volumetric_kit/calib/depth)
  add_subdirectory(volumetric_kit/calib/viz)
endif()
# The app tier, calib_rig's (VC_BUILD_APPS), also needs gfx's GLFW glue (an
# installed GLFW) and a sensor driver: Orbbec's (VS_WITH_ORBBEC) or DeckLink's
# (VS_WITH_DECKLINK).
if(VC_WITH_RIG AND VC_BUILD_APPS)
  if(TARGET volumetric_kit::gfx_glfw
     AND (TARGET volumetric_kit::sensor_orbbec
          OR TARGET volumetric_kit::sensor_decklink))
    add_subdirectory(volumetric_kit/calib/app)
  else()
    message(
      STATUS "calib_app and calib_rig: skipped (they need gfx_glfw, from an "
             "installed GLFW, and sensor's VS_WITH_ORBBEC or VS_WITH_DECKLINK)")
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
  target_link_libraries(calib INTERFACE calib_source calib_capture calib_depth
                                        calib_viz)
endif()
if(TARGET calib_app)
  target_link_libraries(calib INTERFACE calib_app)
endif()
