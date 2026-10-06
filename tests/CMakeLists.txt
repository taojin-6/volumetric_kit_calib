# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

include(GoogleTest)

# version smoke test.
add_executable(vc_version_test version_test.cpp)
target_link_libraries(vc_version_test PRIVATE volumetric_kit::calib_core
                                              GTest::gtest_main)
vc_target_warnings(vc_version_test)
gtest_discover_tests(vc_version_test)

# core: the camera model calib solves for and the calibration file it writes,
# recon's camera tier, from calib's side.
add_executable(vc_camera_test camera_test.cpp)
target_link_libraries(vc_camera_test PRIVATE volumetric_kit::calib_core
                                             GTest::gtest_main)
target_compile_definitions(
  vc_camera_test PRIVATE VC_TEST_SCRATCH_DIR="${CMAKE_CURRENT_BINARY_DIR}")
vc_target_warnings(vc_camera_test)
gtest_discover_tests(vc_camera_test)

# core: the data model (Target / Observation / SensorProfile / LensProfile).
add_executable(vc_data_model_test data_model_test.cpp)
target_link_libraries(vc_data_model_test PRIVATE volumetric_kit::calib_core
                                                 GTest::gtest_main)
vc_target_warnings(vc_data_model_test)
gtest_discover_tests(vc_data_model_test)
