# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

file(MAKE_DIRECTORY "${WORK_DIR}")
file(
  WRITE "${WORK_DIR}/point.json"
  [[
{"version":1,"operation":"project","calibration":{
"intrinsics":[100,200,10,20],"distortion":[0,0,0,0,0],"point":[1,2,4]}}
]])
execute_process(
  COMMAND "${EXECUTABLE}" --config point.json
  WORKING_DIRECTORY "${WORK_DIR}"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT output MATCHES "35\\.000, 120\\.000")
  message(
    FATAL_ERROR "Configured projection failed: ${result}: ${output}${error}")
endif()
# Each coefficient lands in its own place in recon's lens: OpenCV's order, k1,
# k2, p1, p2, k3 (cv::projectPoints gives this pixel).
file(
  WRITE "${WORK_DIR}/distorted.json"
  [[
{"version":1,"operation":"project","calibration":{"intrinsics":[100,200,10,20],
"distortion":[0.1,0.01,0.001,0.002,0.05],"point":[1,2,4]}}
]])
execute_process(
  COMMAND "${EXECUTABLE}" --config distorted.json
  WORKING_DIRECTORY "${WORK_DIR}"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT output MATCHES "35\\.956, 123\\.638")
  message(
    FATAL_ERROR "Distorted projection failed: ${result}: ${output}${error}")
endif()
# A valid lens whose projection overflows this far off-axis is a numerical
# error, not a pixel.
file(
  WRITE "${WORK_DIR}/far.json"
  [[
{"version":1,"operation":"project","calibration":{"intrinsics":[100,200,10,20],
"distortion":[-0.28,0.1,0.001,-0.0005,0.01],"point":[1e120,0,1]}}
]])
execute_process(
  COMMAND "${EXECUTABLE}" --config far.json
  WORKING_DIRECTORY "${WORK_DIR}"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)
if(result EQUAL 0
   OR NOT error MATCHES "Numerical"
   OR output MATCHES "pixel")
  message(
    FATAL_ERROR "Overflowed projection printed a pixel: ${output}${error}")
endif()
file(
  WRITE "${WORK_DIR}/invalid.json"
  [[
{"version":1,"operation":"project","calibration":{"point":[0,0,-1]}}
]])
execute_process(
  COMMAND "${EXECUTABLE}" --config invalid.json
  WORKING_DIRECTORY "${WORK_DIR}"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)
if(result EQUAL 0
   OR NOT error MATCHES "point"
   OR output MATCHES "demo")
  message(
    FATAL_ERROR "Invalid config reached the application: ${output}${error}")
endif()
