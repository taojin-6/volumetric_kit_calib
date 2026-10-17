# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The calib commit a session records (app_software) is SOURCE_DIR's HEAD as it
# stands now, with "-dirty" when tracked files have uncommitted changes: what
# the build before these tests read. Reads both with git, apart from
# software.cmake, and runs TEST's AppCapture.RecordsTheSoftwareItWasBuiltFrom
# expecting that value (VC_EXPECT_CALIB_COMMIT). Prints "not a git checkout",
# which skips the test, where SOURCE_DIR is none (tests/CMakeLists.txt).
foreach(_var IN ITEMS SOURCE_DIR TEST)
  if(NOT DEFINED ${_var})
    message(FATAL_ERROR "app_software_commit.cmake: ${_var} is not set")
  endif()
endforeach()

find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${SOURCE_DIR}" rev-parse HEAD
    OUTPUT_VARIABLE _head
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _rc
    ERROR_QUIET)
endif()
if(NOT GIT_FOUND OR NOT _rc EQUAL 0)
  message("${SOURCE_DIR} is not a git checkout: nothing to compare")
  return()
endif()
execute_process(
  COMMAND "${GIT_EXECUTABLE}" -C "${SOURCE_DIR}" status --porcelain
          --untracked-files=no
  OUTPUT_VARIABLE _changes
  RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "git status failed in ${SOURCE_DIR}")
endif()
set(_expected "${_head}")
if(NOT _changes STREQUAL "")
  string(APPEND _expected "-dirty")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env "VC_EXPECT_CALIB_COMMIT=${_expected}"
          "${TEST}" --gtest_filter=AppCapture.RecordsTheSoftwareItWasBuiltFrom
  RESULT_VARIABLE _rc
  OUTPUT_VARIABLE _output
  ERROR_VARIABLE _output)
if(NOT _rc EQUAL 0 OR NOT _output MATCHES "\\[  PASSED  \\] 1 test")
  message(FATAL_ERROR "the session records another calib commit than "
                      "${_expected}:\n${_output}")
endif()
