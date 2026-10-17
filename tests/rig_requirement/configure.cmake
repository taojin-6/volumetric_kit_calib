# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Configures the application (application/) into BINARY_DIR, with CORE_VULKAN,
# and requires the configure to fail with output matching EXPECT_ERROR, blanks
# and line breaks counting as one space. CALIB_SOURCE, CORE_SOURCE, RECON_SOURCE
# and JSON_SOURCE are the sources the calling build used; GENERATOR,
# CXX_COMPILER, MAKE_PROGRAM, GLM_DIR and OPENCV_DIR repeat its settings
# (tests/CMakeLists.txt).
foreach(
  _var
  CALIB_SOURCE
  BINARY_DIR
  CORE_SOURCE
  RECON_SOURCE
  JSON_SOURCE
  GENERATOR
  CORE_VULKAN
  EXPECT_ERROR)
  if(NOT DEFINED ${_var})
    message(FATAL_ERROR "configure.cmake: ${_var} is not set")
  endif()
endforeach()

file(REMOVE_RECURSE "${BINARY_DIR}")
execute_process(
  COMMAND
    "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_LIST_DIR}/application" -B
    "${BINARY_DIR}" -G "${GENERATOR}" "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
    "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}" "-Dglm_DIR=${GLM_DIR}"
    "-DOpenCV_DIR=${OPENCV_DIR}" "-DCALIB_SOURCE=${CALIB_SOURCE}"
    "-DCORE_SOURCE=${CORE_SOURCE}" "-DRECON_SOURCE=${RECON_SOURCE}"
    "-DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=${JSON_SOURCE}"
    "-DCORE_VULKAN=${CORE_VULKAN}"
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _output
  ERROR_VARIABLE _output)
if(_result EQUAL 0)
  message(FATAL_ERROR "the application configured; expected it to fail with "
                      "'${EXPECT_ERROR}':\n${_output}")
endif()
# CMake wraps a message's lines: match it with each run of blanks as one.
string(REGEX REPLACE "[ \n]+" " " _flat "${_output}")
if(NOT _flat MATCHES "${EXPECT_ERROR}")
  message(FATAL_ERROR "the application's configure failed without "
                      "'${EXPECT_ERROR}':\n${_output}")
endif()
