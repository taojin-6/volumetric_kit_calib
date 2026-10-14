# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# A stand-in for an application of calib's brain, built against an installed
# calib (../configure.cmake): find_package must find calib with the core and
# recon installed beside it, and OpenCV with compute and solve (EXPECT_COMPUTE),
# and none of them may reach for Vulkan.
cmake_minimum_required(VERSION 3.21)
project(vc_brain_consumer LANGUAGES CXX)

find_package(volumetric_kit_calib CONFIG REQUIRED)
foreach(_target volumetric_kit::core_vulkan volumetric_kit::recon_core)
  if(TARGET ${_target})
    message(FATAL_ERROR "calib's installed package brought ${_target}")
  endif()
endforeach()
foreach(_tier calib_compute calib_solve)
  if(EXPECT_COMPUTE AND NOT TARGET volumetric_kit::${_tier})
    message(FATAL_ERROR "calib's installed package lacks ${_tier}")
  elseif(NOT EXPECT_COMPUTE AND TARGET volumetric_kit::${_tier})
    message(FATAL_ERROR "calib's installed package brought ${_tier}")
  endif()
endforeach()

add_executable(vc_brain_consumer consumer.cpp)
target_link_libraries(vc_brain_consumer PRIVATE volumetric_kit::calib)
if(EXPECT_COMPUTE)
  target_compile_definitions(vc_brain_consumer PRIVATE VC_CONSUMER_COMPUTE)
endif()
