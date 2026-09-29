# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

include(GoogleTest)

# version smoke test.
add_executable(vc_version_test version_test.cpp)
target_link_libraries(vc_version_test PRIVATE volumetric_kit::calib_core
                                              GTest::gtest_main)
vc_target_warnings(vc_version_test)
gtest_discover_tests(vc_version_test)

# core: exception-free error handling (Status / Result / VC_TRY / VC_ASSIGN).
add_executable(vc_result_test result_test.cpp)
target_link_libraries(vc_result_test PRIVATE volumetric_kit::calib_core
                                             GTest::gtest_main)
vc_target_warnings(vc_result_test)
gtest_discover_tests(vc_result_test)

# core: the pinhole + Brown-Conrady camera model.
add_executable(vc_camera_model_test camera_model_test.cpp)
target_link_libraries(vc_camera_model_test PRIVATE volumetric_kit::calib_core
                                                   GTest::gtest_main)
vc_target_warnings(vc_camera_model_test)
gtest_discover_tests(vc_camera_model_test)
