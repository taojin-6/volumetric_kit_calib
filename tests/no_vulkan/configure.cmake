# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# calib's brain with no Vulkan: run by the no_vulkan.brain test
# (tests/CMakeLists.txt), in BINARY_DIR, with find_package(Vulkan) disabled
# throughout, so a configure that reaches for Vulkan -- sensor beyond its camera
# tier, or the core's vulkan tier -- fails rather than finds one.
#
# 1. With VS_CAMERA_ONLY OFF, as an application that needs the rest of sensor
#    sets it, calib's configure keeps it, and so reaches for Vulkan and fails.
# 2. calib configures from CALIB_SOURCE, with OpenCV found at OPENCV_DIR, and
#    builds the brain: calib_core, calib_compute, calib_detect, calib_solve and
#    calib_quality, over the core's base tier and sensor's camera tier.
#
# CORE_SOURCE, SENSOR_SOURCE and JSON_SOURCE are the sources the calling build
# fetched, so this needs no network; GENERATOR, CXX_COMPILER, MAKE_PROGRAM,
# GLM_DIR and OPENCV_DIR repeat the calling build's settings. The build is
# Release, so a multi-config generator builds that, and runs at the caller's
# CMAKE_BUILD_PARALLEL_LEVEL, or on every core where none is set (a bare
# --parallel would not bound make's jobs).
foreach(_var CALIB_SOURCE BINARY_DIR CORE_SOURCE SENSOR_SOURCE JSON_SOURCE
             GENERATOR)
  if(NOT DEFINED ${_var})
    message(FATAL_ERROR "configure.cmake: ${_var} is not set")
  endif()
endforeach()

# Runs `cmake <ARGN>`; fails, with its output, unless it succeeds.
function(_vc_cmake what)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" ${ARGN}
    RESULT_VARIABLE _result
    OUTPUT_VARIABLE _output
    ERROR_VARIABLE _output)
  if(NOT _result EQUAL 0)
    message(FATAL_ERROR "${what} failed:\n${_output}")
  endif()
endfunction()

# Runs `cmake <ARGN>`; fails unless it fails with output matching `expected`,
# each run of blanks and line breaks counting as one space.
function(_vc_cmake_refuses what expected)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" ${ARGN}
    RESULT_VARIABLE _result
    OUTPUT_VARIABLE _output
    ERROR_VARIABLE _output)
  string(REGEX REPLACE "[ \n]+" " " _flat "${_output}")
  if(_result EQUAL 0 OR NOT _flat MATCHES "${expected}")
    message(FATAL_ERROR "${what} did not fail with '${expected}':\n${_output}")
  endif()
endfunction()

set(_common
    -S
    "${CALIB_SOURCE}"
    -G
    "${GENERATOR}"
    "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
    "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}"
    "-Dglm_DIR=${GLM_DIR}"
    "-DOpenCV_DIR=${OPENCV_DIR}"
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_DISABLE_FIND_PACKAGE_Vulkan=ON
    "-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=${CORE_SOURCE}"
    "-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_SENSOR=${SENSOR_SOURCE}"
    "-DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=${JSON_SOURCE}"
    -DVC_BUILD_TESTS=OFF
    -DVC_BUILD_EXAMPLES=OFF)

file(REMOVE_RECURSE "${BINARY_DIR}")
_vc_cmake_refuses(
  "Configuring calib with VS_CAMERA_ONLY OFF"
  "CMAKE_DISABLE_FIND_PACKAGE_Vulkan" ${_common} -B
  "${BINARY_DIR}/sensor_whole" -DVS_CAMERA_ONLY=OFF)

_vc_cmake("Configuring calib with no Vulkan" ${_common} -B
          "${BINARY_DIR}/calib")
if(NOT DEFINED ENV{CMAKE_BUILD_PARALLEL_LEVEL})
  cmake_host_system_information(RESULT _cores QUERY NUMBER_OF_LOGICAL_CORES)
  set(ENV{CMAKE_BUILD_PARALLEL_LEVEL} ${_cores})
endif()
_vc_cmake("Building calib" --build "${BINARY_DIR}/calib" --config Release)
