# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# calib's brain with no Vulkan: run by the no_vulkan.brain test
# (tests/CMakeLists.txt), in BINARY_DIR, with find_package(Vulkan) disabled
# throughout, so a configure that reaches for Vulkan -- recon beyond its camera
# tier, or the core's vulkan tier -- fails rather than finds one.
#
# 1. calib configures from CALIB_SOURCE, builds and installs to
#    BINARY_DIR/prefix.
# 2. The consumer (consumer/) finds that installed package, and with it the core
#    and recon installed beside it, and builds and runs against calib_core.
#
# CORE_SOURCE, RECON_SOURCE and JSON_SOURCE are the sources the calling build
# fetched, so this needs no network; GENERATOR, CXX_COMPILER, MAKE_PROGRAM and
# GLM_DIR repeat the calling build's settings. Each build and the install are
# Release, so a multi-config generator builds what it installs.
foreach(_var CALIB_SOURCE BINARY_DIR CORE_SOURCE RECON_SOURCE JSON_SOURCE
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

set(_common
    -G
    "${GENERATOR}"
    "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
    "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}"
    "-Dglm_DIR=${GLM_DIR}"
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_DISABLE_FIND_PACKAGE_Vulkan=ON)
set(_prefix "${BINARY_DIR}/prefix")

file(REMOVE_RECURSE "${BINARY_DIR}")
_vc_cmake(
  "Configuring calib with no Vulkan"
  -S
  "${CALIB_SOURCE}"
  -B
  "${BINARY_DIR}/calib"
  ${_common}
  "-DCMAKE_INSTALL_PREFIX=${_prefix}"
  "-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=${CORE_SOURCE}"
  "-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=${RECON_SOURCE}"
  "-DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=${JSON_SOURCE}"
  -DVC_BUILD_TESTS=OFF
  -DVC_BUILD_EXAMPLES=OFF)
_vc_cmake("Building calib" --build "${BINARY_DIR}/calib" --config Release)
_vc_cmake("Installing calib" --install "${BINARY_DIR}/calib" --config Release)

_vc_cmake(
  "Configuring against the installed calib"
  -S
  "${CMAKE_CURRENT_LIST_DIR}/consumer"
  -B
  "${BINARY_DIR}/consumer"
  ${_common}
  "-DCMAKE_PREFIX_PATH=${_prefix}")
_vc_cmake("Building against the installed calib" --build
          "${BINARY_DIR}/consumer" --config Release)

file(GLOB_RECURSE _consumer "${BINARY_DIR}/consumer/vc_brain_consumer")
if(NOT _consumer)
  message(FATAL_ERROR "the consumer built no vc_brain_consumer")
endif()
list(GET _consumer 0 _consumer)
execute_process(
  COMMAND "${_consumer}"
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _output
  ERROR_VARIABLE _output)
if(NOT _result EQUAL 0)
  message(FATAL_ERROR "the consumer failed (${_result}):\n${_output}")
endif()
