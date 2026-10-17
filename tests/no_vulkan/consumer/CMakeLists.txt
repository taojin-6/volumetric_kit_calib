# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# A stand-in for an application of calib's brain, built against an installed
# calib (../configure.cmake): find_package must find calib with the core and
# recon installed beside it, and OpenCV, and none of them may reach for Vulkan.
# With FIND_CORE, the application finds the core itself too, before calib and
# again after it.
cmake_minimum_required(VERSION 3.21)
project(vc_brain_consumer LANGUAGES CXX)

if(FIND_CORE)
  find_package(volumetric_kit_core CONFIG REQUIRED)
endif()
find_package(volumetric_kit_calib CONFIG REQUIRED)
if(FIND_CORE)
  find_package(volumetric_kit_core CONFIG REQUIRED)
endif()
foreach(_target volumetric_kit::core_vulkan volumetric_kit::recon_core)
  if(TARGET ${_target})
    message(FATAL_ERROR "calib's installed package brought ${_target}")
  endif()
endforeach()
foreach(_tier calib_compute calib_detect calib_solve calib_quality)
  if(NOT TARGET volumetric_kit::${_tier})
    message(FATAL_ERROR "calib's installed package lacks ${_tier}")
  endif()
endforeach()

add_executable(vc_brain_consumer consumer.cpp)
target_link_libraries(vc_brain_consumer PRIVATE volumetric_kit::calib)
