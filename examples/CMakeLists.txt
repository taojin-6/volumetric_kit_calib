# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Examples are pure-CPU CLIs for now (the core tier needs no window), so unlike
# volumetric_kit_gfx there is no GLFW gate here. The viz/app examples that
# consume volumetric_kit_gfx will add that dependency when those tiers land.

# 01_project_point: construct a PinholeBrownConrady camera and round-trip a
# point through project/unproject — the smallest end-to-end use of calib_core.
add_executable(example_01_project_point 01_project_point/main.cpp)
target_link_libraries(
  example_01_project_point PRIVATE volumetric_kit::calib_core
                                   volumetric_kit::core_config)
vc_target_warnings(example_01_project_point)

if(VC_BUILD_TESTS)
  add_test(
    NAME project_point.config
    COMMAND
      ${CMAKE_COMMAND} -DEXECUTABLE=$<TARGET_FILE:example_01_project_point>
      -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/config-test -P
      ${CMAKE_CURRENT_SOURCE_DIR}/01_project_point/config_test.cmake)
endif()
