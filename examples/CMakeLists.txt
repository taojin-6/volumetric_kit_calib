# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Examples are pure-CPU CLIs for now (the core tier needs no window), so unlike
# volumetric_kit_gfx there is no GLFW gate here. The viz/app examples that
# consume volumetric_kit_gfx will add that dependency when those tiers land.

# 01_project_point: map the scenario's camera into sensor's CameraModel and
# project a point through its lens -- the smallest end-to-end use of calib's
# camera model.
add_executable(example_01_project_point 01_project_point/main.cpp)
target_link_libraries(
  example_01_project_point
  PRIVATE volumetric_kit::calib_core volumetric_kit::sensor_camera
          volumetric_kit::core_config)
vc_target_warnings(example_01_project_point)

# 02_lens_from_images: one camera's lens from saved pictures of a checkerboard,
# on the stable API (README.md). Only this example reads image files, so only it
# takes OpenCV's imgcodecs.
find_package(OpenCV 5.0 CONFIG REQUIRED COMPONENTS imgcodecs)
add_executable(example_02_lens_from_images 02_lens_from_images/main.cpp)
target_link_libraries(
  example_02_lens_from_images
  PRIVATE volumetric_kit::calib_detect volumetric_kit::calib_quality
          opencv_imgcodecs)
vc_target_warnings(example_02_lens_from_images)

if(VC_BUILD_TESTS)
  add_test(
    NAME project_point.config
    COMMAND
      ${CMAKE_COMMAND} -DEXECUTABLE=$<TARGET_FILE:example_01_project_point>
      -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/config-test -P
      ${CMAKE_CURRENT_SOURCE_DIR}/01_project_point/config_test.cmake)
  # `api`, on synthetic views, runs the calls it makes; this, its arguments.
  add_test(
    NAME lens_from_images.arguments
    COMMAND
      ${CMAKE_COMMAND} -DEXECUTABLE=$<TARGET_FILE:example_02_lens_from_images>
      -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/lens-test -P
      ${CMAKE_CURRENT_SOURCE_DIR}/02_lens_from_images/arguments_test.cmake)
endif()
