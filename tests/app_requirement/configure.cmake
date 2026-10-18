# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Configures the application (application/) into BINARY_DIR, BUILD_APPS as
# given, and requires the configure to succeed, or, with EXPECT_ERROR, to fail
# with output matching it, blanks and line breaks counting as one space.
# CALIB_SOURCE, CORE_SOURCE and RECON_SOURCE are the sources the calling build
# used, and FETCHED its other fetched sources, "<NAME>=<dir>" entries joined by
# "|"; GENERATOR, CXX_COMPILER, MAKE_PROGRAM, GLM_DIR, GLFW_DIR, OPENCV_DIR and
# ORBBEC_DIR repeat its settings (tests/CMakeLists.txt).
foreach(
  _var
  CALIB_SOURCE
  BINARY_DIR
  CORE_SOURCE
  RECON_SOURCE
  FETCHED
  GENERATOR
  BUILD_APPS)
  if(NOT DEFINED ${_var})
    message(FATAL_ERROR "configure.cmake: ${_var} is not set")
  endif()
endforeach()

set(_sources)
string(REPLACE "|" ";" _fetched "${FETCHED}")
foreach(_entry IN LISTS _fetched)
  list(APPEND _sources "-DFETCHCONTENT_SOURCE_DIR_${_entry}")
endforeach()

file(REMOVE_RECURSE "${BINARY_DIR}")
execute_process(
  COMMAND
    "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_LIST_DIR}/application" -B
    "${BINARY_DIR}" -G "${GENERATOR}" "-DCMAKE_BUILD_TYPE=Release"
    "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
    "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}" "-Dglm_DIR=${GLM_DIR}"
    "-Dglfw3_DIR=${GLFW_DIR}" "-DOpenCV_DIR=${OPENCV_DIR}"
    "-DOrbbecSDK_DIR=${ORBBEC_DIR}" "-DCALIB_SOURCE=${CALIB_SOURCE}"
    "-DCORE_SOURCE=${CORE_SOURCE}" "-DRECON_SOURCE=${RECON_SOURCE}"
    "-DBUILD_APPS=${BUILD_APPS}" ${_sources}
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _output
  ERROR_VARIABLE _output)
if(NOT DEFINED EXPECT_ERROR)
  if(NOT _result EQUAL 0)
    message(FATAL_ERROR "the application's configure failed:\n${_output}")
  endif()
  return()
endif()
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
