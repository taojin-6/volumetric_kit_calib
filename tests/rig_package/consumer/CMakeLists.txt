# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# A stand-in for the rig application, built against an installed calib with the
# capture tier (../configure.cmake): find_package must find calib, and the core
# and all of recon installed beside it, with what they re-find.
cmake_minimum_required(VERSION 3.21)
project(vc_rig_consumer LANGUAGES CXX)

find_package(volumetric_kit_calib CONFIG REQUIRED)
if(NOT TARGET volumetric_kit::calib_capture)
  message(FATAL_ERROR "calib's installed package lacks calib_capture")
endif()

add_executable(vc_rig_consumer consumer.cpp)
target_link_libraries(vc_rig_consumer PRIVATE volumetric_kit::calib_capture)
