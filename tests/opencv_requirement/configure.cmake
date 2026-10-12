# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Configures calib (CALIB_SOURCE) into BINARY_DIR with VC_WITH_OPENCV on and
# OpenCV hidden from find_package: the configure must fail with the message that
# names the fixes. GENERATOR, CXX_COMPILER and MAKE_PROGRAM repeat the calling
# build's settings (tests/CMakeLists.txt). It fails before anything is fetched.
foreach(_var CALIB_SOURCE BINARY_DIR GENERATOR)
  if(NOT DEFINED ${_var})
    message(FATAL_ERROR "configure.cmake: ${_var} is not set")
  endif()
endforeach()

file(REMOVE_RECURSE "${BINARY_DIR}")
execute_process(
  COMMAND
    "${CMAKE_COMMAND}" -S "${CALIB_SOURCE}" -B "${BINARY_DIR}" -G "${GENERATOR}"
    "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
    "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}" -DVC_WITH_OPENCV=ON
    -DCMAKE_DISABLE_FIND_PACKAGE_OpenCV=ON -DVC_BUILD_TESTS=OFF
    -DVC_BUILD_EXAMPLES=OFF
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _output
  ERROR_VARIABLE _output)

if(_result EQUAL 0)
  message(FATAL_ERROR "calib configured with VC_WITH_OPENCV and no OpenCV")
endif()
# CMake wraps a message's lines: match it with each run of blanks as one.
string(REGEX REPLACE "[ \n]+" " " _flat "${_output}")
set(_expected
    "VC_WITH_OPENCV is ON but OpenCV 5 .* was not found\\..*brew install opencv.*-DOpenCV_DIR=.*-DVC_WITH_OPENCV=OFF"
)
if(NOT _flat MATCHES "${_expected}")
  message(FATAL_ERROR "calib's configure without OpenCV failed without "
                      "'${_expected}':\n${_output}")
endif()
