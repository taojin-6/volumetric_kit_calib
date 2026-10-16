# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The capture tier from an installed package: installs the calling build,
# BUILD_DIR, in its CONFIG, to BINARY_DIR/prefix, checks it carries the capture
# tier and not the viz tier, then configures, builds and runs the consumer
# (consumer/) against that prefix alone. GENERATOR, CXX_COMPILER, MAKE_PROGRAM,
# GLM_DIR and FLAGS, its sanitizers' compile and link flags, repeat the calling
# build's settings (tests/CMakeLists.txt).
foreach(_var BUILD_DIR CONFIG BINARY_DIR GENERATOR)
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

set(_prefix "${BINARY_DIR}/prefix")
file(REMOVE_RECURSE "${BINARY_DIR}")
_vc_cmake(
  "Installing the build"
  --install
  "${BUILD_DIR}"
  --config
  "${CONFIG}"
  --prefix
  "${_prefix}")
foreach(_header session_writer.hpp export.hpp)
  if(NOT EXISTS "${_prefix}/include/volumetric_kit/calib/capture/${_header}")
    message(FATAL_ERROR "the install lacks capture/${_header}")
  endif()
endforeach()
# The viz tier is build-tree only, as gfx's ui tier is.
file(GLOB_RECURSE _viz "${_prefix}/include/volumetric_kit/calib/viz/*"
     "${_prefix}/lib*/*calib_viz*")
if(_viz)
  message(
    FATAL_ERROR "the install carries the build-tree-only viz tier: ${_viz}")
endif()

_vc_cmake(
  "Configuring against the installed calib"
  -S
  "${CMAKE_CURRENT_LIST_DIR}/consumer"
  -B
  "${BINARY_DIR}/consumer"
  -G
  "${GENERATOR}"
  "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
  "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}"
  "-Dglm_DIR=${GLM_DIR}"
  "-DCMAKE_BUILD_TYPE=${CONFIG}"
  "-DCMAKE_CXX_FLAGS=${FLAGS}"
  "-DCMAKE_EXE_LINKER_FLAGS=${FLAGS}"
  "-DCMAKE_PREFIX_PATH=${_prefix}")
_vc_cmake("Building against the installed calib" --build
          "${BINARY_DIR}/consumer" --config "${CONFIG}")

file(GLOB_RECURSE _consumer "${BINARY_DIR}/consumer/vc_rig_consumer")
if(NOT _consumer)
  message(FATAL_ERROR "the consumer built no vc_rig_consumer")
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
