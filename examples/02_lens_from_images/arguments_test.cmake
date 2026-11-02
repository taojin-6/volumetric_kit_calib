# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

file(MAKE_DIRECTORY "${WORK_DIR}")
execute_process(
  COMMAND "${EXECUTABLE}"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)
if(NOT result EQUAL 2 OR NOT error MATCHES "usage: lens_from_images")
  message(FATAL_ERROR "No arguments gave: ${result}: ${output}${error}")
endif()
# A picture it cannot read is named.
execute_process(
  COMMAND "${EXECUTABLE}" bm-1 14 9 20 "${WORK_DIR}" "${WORK_DIR}/missing.pgm"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)
if(NOT result EQUAL 1 OR NOT error MATCHES "missing\\.pgm")
  message(FATAL_ERROR "A missing picture gave: ${result}: ${output}${error}")
endif()
# A board too small for the detector is refused before any picture is read.
execute_process(
  COMMAND "${EXECUTABLE}" bm-1 3 3 20 "${WORK_DIR}" "${WORK_DIR}/missing.pgm"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)
if(NOT result EQUAL 1 OR error MATCHES "missing\\.pgm")
  message(FATAL_ERROR "A small board gave: ${result}: ${output}${error}")
endif()
