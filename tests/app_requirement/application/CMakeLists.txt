# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# An application that adds calib with its rig and viz tiers (../configure.cmake)
# after making the core available first, to share it, without its config tier,
# and sensor with its Orbbec driver, so calib_rig's prerequisites are all here.
# Unless the application asks for the apps (BUILD_APPS), calib's configure must
# leave calib_rig and its tier out; asked, it must stop, naming the core's
# config tier. The sources are the calling build's, so nothing is fetched.
cmake_minimum_required(VERSION 3.21)
project(vc_app_application LANGUAGES CXX)

include(FetchContent)
FetchContent_Declare(volumetric_kit_core SOURCE_DIR "${CORE_SOURCE}")
FetchContent_Declare(volumetric_kit_sensor SOURCE_DIR "${SENSOR_SOURCE}")
set(VKC_WITH_VULKAN ON)
set(VS_CAMERA_ONLY OFF)
set(VS_WITH_ORBBEC ON)
# The driver's colour decoders: VideoToolbox on Apple, with no FFmpeg; FFmpeg
# and CUDA elsewhere, which sensor requires together (its
# cmake/vs_ffmpeg.cmake).
if(NOT APPLE)
  set(VS_WITH_FFMPEG ON)
  set(VS_WITH_CUDA ON)
endif()
FetchContent_MakeAvailable(volumetric_kit_core volumetric_kit_sensor)

set(VC_WITH_RIG ON)
if(BUILD_APPS)
  set(VC_BUILD_APPS ON)
endif()
add_subdirectory("${CALIB_SOURCE}" calib)

# The case shows something only where calib_rig could be built.
foreach(_vc_need volumetric_kit::gfx_glfw volumetric_kit::sensor_orbbec)
  if(NOT TARGET ${_vc_need})
    message(FATAL_ERROR "the application has no ${_vc_need}")
  endif()
endforeach()
if(TARGET calib_app OR TARGET calib_rig)
  message(FATAL_ERROR "calib built calib_rig for an application that did not "
                      "ask for it")
endif()
message(STATUS "calib left calib_rig out")
