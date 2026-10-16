# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# calib_rig: the live rig calibration application, a short main over calib_app
# (src/volumetric_kit/calib/app/).
add_executable(calib_rig calib_rig/main.cpp)
target_link_libraries(calib_rig PRIVATE volumetric_kit::calib_app)
vc_target_warnings(calib_rig)

if(VC_BUILD_TESTS)
  # What calib_rig refuses before it opens a window or a camera
  # (calib_rig/config_test.cmake).
  add_test(
    NAME calib_rig.config
    COMMAND
      ${CMAKE_COMMAND} -DEXECUTABLE=$<TARGET_FILE:calib_rig>
      -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/config-test -P
      ${CMAKE_CURRENT_SOURCE_DIR}/calib_rig/config_test.cmake)
endif()
