# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

include(GoogleTest)

# version smoke test.
add_executable(vc_version_test version_test.cpp)
target_link_libraries(vc_version_test PRIVATE volumetric_kit::calib_core
                                              GTest::gtest_main)
vc_target_warnings(vc_version_test)
gtest_discover_tests(vc_version_test)

# core: the pinhole + Brown-Conrady camera model.
add_executable(vc_camera_model_test camera_model_test.cpp)
target_link_libraries(vc_camera_model_test PRIVATE volumetric_kit::calib_core
                                                   GTest::gtest_main)
vc_target_warnings(vc_camera_model_test)
gtest_discover_tests(vc_camera_model_test)

# core: the data model (Target / Observation / SensorProfile / LensProfile).
add_executable(vc_data_model_test data_model_test.cpp)
target_link_libraries(vc_data_model_test PRIVATE volumetric_kit::calib_core
                                                 GTest::gtest_main)
vc_target_warnings(vc_data_model_test)
gtest_discover_tests(vc_data_model_test)

# capture: sessions on disk, their replay, and the recorder. Each test works in
# a scratch directory of its own under the system temp directory.
foreach(_vc_capture_test capture_session capture_replay capture_recorder)
  add_executable(vc_${_vc_capture_test}_test ${_vc_capture_test}_test.cpp)
  target_link_libraries(vc_${_vc_capture_test}_test
                        PRIVATE volumetric_kit::calib_capture GTest::gtest_main)
  vc_target_warnings(vc_${_vc_capture_test}_test)
  gtest_discover_tests(vc_${_vc_capture_test}_test)
endforeach()

# capture: a program with its own global stb_image still links calib_capture. It
# compiles stb's implementation, which is third-party, so warnings are off.
add_executable(vc_capture_stb_link_test capture_stb_link_test.cpp)
target_include_directories(vc_capture_stb_link_test SYSTEM
                           PRIVATE "${VC_STB_INCLUDE_DIR}")
target_link_libraries(vc_capture_stb_link_test
                      PRIVATE volumetric_kit::calib_capture GTest::gtest_main)
target_compile_options(vc_capture_stb_link_test PRIVATE -w)
gtest_discover_tests(vc_capture_stb_link_test)
