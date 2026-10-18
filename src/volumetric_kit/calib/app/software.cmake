# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Writes OUT from IN (software.cpp.in): what a session records it was captured
# with. Each checkout (CALIB_DIR, CORE_DIR, RECON_DIR, GFX_DIR) gives its commit
# as it stands now, 40 hex digits with "-dirty" when tracked files have
# uncommitted changes, or nothing where the directory is not a git checkout's
# top; ORBBEC_SDK_VERSION is the configure's. calib_app runs it at every build
# (CMakeLists.txt). configure_file rewrites OUT only when its text changes, so
# an unchanged checkout recompiles nothing.
foreach(_var IN ITEMS IN OUT CALIB_DIR CORE_DIR RECON_DIR GFX_DIR)
  if(NOT DEFINED ${_var})
    message(FATAL_ERROR "software.cmake: ${_var} is not set")
  endif()
endforeach()

find_package(Git QUIET)

# Sets <out> to the commit of the checkout whose top is <dir>.
function(_vc_checkout_commit out dir)
  set(${out}
      ""
      PARENT_SCOPE)
  if(NOT GIT_FOUND OR NOT IS_DIRECTORY "${dir}")
    return()
  endif()
  # A directory inside some other checkout is not one itself.
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${dir}" rev-parse --show-toplevel
    OUTPUT_VARIABLE _top
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _rc
    ERROR_QUIET)
  file(REAL_PATH "${dir}" _dir)
  if(NOT _rc EQUAL 0 OR _top STREQUAL "")
    return()
  endif()
  file(REAL_PATH "${_top}" _top)
  if(NOT _top STREQUAL _dir)
    return()
  endif()
  # Excluding every tag names the commit itself, never a tag's distance.
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${dir}" describe --always --abbrev=40
            --dirty --exclude=*
    OUTPUT_VARIABLE _commit
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _rc
    ERROR_QUIET)
  if(_rc EQUAL 0)
    set(${out}
        "${_commit}"
        PARENT_SCOPE)
  endif()
endfunction()

_vc_checkout_commit(VC_APP_CALIB_COMMIT "${CALIB_DIR}")
_vc_checkout_commit(VC_APP_CORE_COMMIT "${CORE_DIR}")
_vc_checkout_commit(VC_APP_RECON_COMMIT "${RECON_DIR}")
_vc_checkout_commit(VC_APP_GFX_COMMIT "${GFX_DIR}")
set(VC_APP_ORBBEC_SDK_VERSION "${ORBBEC_SDK_VERSION}")
configure_file("${IN}" "${OUT}" @ONLY)
