# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# An application that makes the core and sensor available before calib with
# VC_WITH_RIG (../configure.cmake): the core with its vulkan tier as CORE_VULKAN
# says, and sensor camera-only. calib's configure must then stop, naming what is
# missing. The sources are the calling build's (CORE_SOURCE, SENSOR_SOURCE,
# CALIB_SOURCE), so nothing is fetched.
cmake_minimum_required(VERSION 3.21)
project(vc_rig_application LANGUAGES CXX)

include(FetchContent)
FetchContent_Declare(volumetric_kit_core SOURCE_DIR "${CORE_SOURCE}")
FetchContent_Declare(volumetric_kit_sensor SOURCE_DIR "${SENSOR_SOURCE}")
set(VKC_WITH_VULKAN ${CORE_VULKAN})
set(VS_CAMERA_ONLY ON)
FetchContent_MakeAvailable(volumetric_kit_core volumetric_kit_sensor)

set(VC_WITH_RIG ON)
add_subdirectory("${CALIB_SOURCE}" calib)
