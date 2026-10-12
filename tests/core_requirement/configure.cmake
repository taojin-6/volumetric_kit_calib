# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Configures calib (CALIB_SOURCE) into BINARY_DIR against the stand-in core
# (fake_core/) at CORE_VERSION, with the real core's modules from CORE_MODULES,
# and checks the outcome. With EXPECT_ERROR, the configure must fail with output
# matching that regex, blanks and line breaks counting as one space. With
# EXPECT_CONFIG_VERSION, it must succeed: recon's package config must re-find
# the core at that MAJOR.MINOR, and calib's must re-find recon, hinted at its
# own prefix, refuse a core of another MAJOR.MINOR, and then require
# MIN_VERSION, calib's floor, of it. RECON_SOURCE and JSON_SOURCE are the
# sources the calling build fetched; GENERATOR, CXX_COMPILER, MAKE_PROGRAM and
# GLM_DIR repeat its settings (tests/CMakeLists.txt).
foreach(
  _var
  CALIB_SOURCE
  BINARY_DIR
  CORE_VERSION
  CORE_MODULES
  MIN_VERSION
  RECON_SOURCE
  JSON_SOURCE
  GENERATOR)
  if(NOT DEFINED ${_var})
    message(FATAL_ERROR "configure.cmake: ${_var} is not set")
  endif()
endforeach()
if((DEFINED EXPECT_ERROR AND DEFINED EXPECT_CONFIG_VERSION)
   OR (NOT DEFINED EXPECT_ERROR AND NOT DEFINED EXPECT_CONFIG_VERSION))
  message(
    FATAL_ERROR
      "configure.cmake: set one of EXPECT_ERROR and EXPECT_CONFIG_VERSION")
endif()

file(REMOVE_RECURSE "${BINARY_DIR}")
execute_process(
  COMMAND
    "${CMAKE_COMMAND}" -S "${CALIB_SOURCE}" -B "${BINARY_DIR}" -G "${GENERATOR}"
    "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
    "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}" "-Dglm_DIR=${GLM_DIR}"
    "-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=${CMAKE_CURRENT_LIST_DIR}/fake_core"
    "-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=${RECON_SOURCE}"
    "-DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=${JSON_SOURCE}"
    "-DVC_FAKE_CORE_VERSION=${CORE_VERSION}"
    "-DVC_FAKE_CORE_MODULES=${CORE_MODULES}" -DVC_BUILD_TESTS=OFF
    -DVC_BUILD_EXAMPLES=OFF
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _output
  ERROR_VARIABLE _output)

if(DEFINED EXPECT_ERROR)
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
  return()
endif()

if(NOT _result EQUAL 0)
  message(
    FATAL_ERROR
      "calib's configure against core ${CORE_VERSION} failed:\n${_output}")
endif()
set(_recon_config
    "${BINARY_DIR}/_deps/volumetric_kit_recon-build/src/volumetric_kit_reconConfig.cmake"
)
file(READ "${_recon_config}" _recon_config_text)
set(_find_core
    "find_dependency(volumetric_kit_core ${EXPECT_CONFIG_VERSION} CONFIG")
string(FIND "${_recon_config_text}" "${_find_core}" _core_at)
if(_core_at EQUAL -1)
  message(
    FATAL_ERROR "${_recon_config} lacks '${_find_core}':\n${_recon_config_text}"
  )
endif()

set(_config "${BINARY_DIR}/src/volumetric_kit_calibConfig.cmake")
file(READ "${_config}" _config_text)
# Each must follow the one before it: the version check reads the core recon
# found, and the floor is checked once the minor is.
set(_after -1)
foreach(
  _expected
  [[find_dependency(volumetric_kit_recon CONFIG HINTS "${PACKAGE_PREFIX_DIR}")]]
  "vkc_core_version(_vc_core_found MAJOR_MINOR)"
  "if(NOT _vc_core_found STREQUAL \"${EXPECT_CONFIG_VERSION}\")"
  "vkc_require_core(${MIN_VERSION} PACKAGE volumetric_kit_calib)")
  string(FIND "${_config_text}" "${_expected}" _at)
  if(_at LESS_EQUAL _after)
    message(
      FATAL_ERROR
        "${_config} lacks '${_expected}' where expected:\n${_config_text}")
  endif()
  set(_after ${_at})
endforeach()
