# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Configures calib (CALIB_SOURCE) into BINARY_DIR against the stand-in core
# (fake_core/) at CORE_VERSION, with the real core's modules from CORE_MODULES,
# and checks the outcome: with EXPECT_ERROR, the configure must fail with output
# matching that regex, blanks and line breaks counting as one space; without it,
# it must succeed. RECON_SOURCE and JSON_SOURCE are the sources the calling
# build fetched; GENERATOR, CXX_COMPILER, MAKE_PROGRAM, GLM_DIR and OPENCV_DIR
# repeat its settings (tests/CMakeLists.txt).
foreach(
  _var
  CALIB_SOURCE
  BINARY_DIR
  CORE_VERSION
  CORE_MODULES
  RECON_SOURCE
  JSON_SOURCE
  GENERATOR)
  if(NOT DEFINED ${_var})
    message(FATAL_ERROR "configure.cmake: ${_var} is not set")
  endif()
endforeach()

file(REMOVE_RECURSE "${BINARY_DIR}")
execute_process(
  COMMAND
    "${CMAKE_COMMAND}" -S "${CALIB_SOURCE}" -B "${BINARY_DIR}" -G "${GENERATOR}"
    "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
    "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}" "-Dglm_DIR=${GLM_DIR}"
    "-DOpenCV_DIR=${OPENCV_DIR}"
    "-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=${CMAKE_CURRENT_LIST_DIR}/fake_core"
    "-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=${RECON_SOURCE}"
    "-DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=${JSON_SOURCE}"
    "-DVC_FAKE_CORE_VERSION=${CORE_VERSION}"
    "-DVC_FAKE_CORE_MODULES=${CORE_MODULES}" -DVC_BUILD_TESTS=OFF
    -DVC_BUILD_EXAMPLES=OFF
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _output
  ERROR_VARIABLE _output)

if(NOT DEFINED EXPECT_ERROR)
  if(NOT _result EQUAL 0)
    message(
      FATAL_ERROR
        "calib's configure against core ${CORE_VERSION} failed:\n${_output}")
  endif()
  return()
endif()
if(_result EQUAL 0)
  message(FATAL_ERROR "calib configured against core ${CORE_VERSION}; "
                      "expected it to fail with '${EXPECT_ERROR}'")
endif()
# CMake wraps a message's lines: match it with each run of blanks as one.
string(REGEX REPLACE "[ \n]+" " " _flat "${_output}")
if(NOT _flat MATCHES "${EXPECT_ERROR}")
  message(FATAL_ERROR "calib's configure against core ${CORE_VERSION} failed "
                      "without '${EXPECT_ERROR}':\n${_output}")
endif()
