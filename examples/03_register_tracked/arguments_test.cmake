# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

file(MAKE_DIRECTORY "${WORK_DIR}")
# No arguments, or a window without its end, give the usage.
foreach(arguments "" "a.json;b.json;147.25;out.json;0")
  execute_process(
    COMMAND "${EXECUTABLE}" ${arguments}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error)
  if(NOT result EQUAL 2 OR NOT error MATCHES "usage: register_tracked")
    message(FATAL_ERROR "\"${arguments}\" gave: ${result}: ${output}${error}")
  endif()
endforeach()
# A calibration file it cannot read is named, and nothing is written.
execute_process(
  COMMAND "${EXECUTABLE}" "${WORK_DIR}/missing.json" "${WORK_DIR}/stills.json"
          147.25 "${WORK_DIR}/out.json"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)
if(NOT result EQUAL 1
   OR NOT error MATCHES "missing\\.json"
   OR EXISTS "${WORK_DIR}/out.json")
  message(
    FATAL_ERROR "A missing calibration gave: ${result}: ${output}${error}")
endif()
