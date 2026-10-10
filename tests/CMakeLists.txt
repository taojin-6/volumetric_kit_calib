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

# The core calib requires (third_party/CMakeLists.txt): calib's configure
# against a stand-in core of each version (core_requirement/) refuses one older
# than 0.1.0 -- before 0.1.0 the core has no vkc_require_core -- and accepts
# 0.1.0 or newer, its package config re-finding the core at its MAJOR.MINOR and
# requiring VC_CORE_MIN_VERSION of it.
include(FetchContent)
FetchContent_GetProperties(volumetric_kit_core)
function(vc_add_core_requirement_test name core_version expect)
  add_test(
    NAME core_requirement.${name}
    COMMAND
      ${CMAKE_COMMAND} -DCALIB_SOURCE=${PROJECT_SOURCE_DIR}
      -DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/core_requirement/${name}
      -DCORE_VERSION=${core_version}
      -DCORE_MODULES=${volumetric_kit_core_SOURCE_DIR}/cmake
      -DMIN_VERSION=${VC_CORE_MIN_VERSION} ${expect}
      -DGENERATOR=${CMAKE_GENERATOR} -DCXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM} -DEIGEN3_DIR=${Eigen3_DIR} -P
      ${CMAKE_CURRENT_SOURCE_DIR}/core_requirement/configure.cmake)
endfunction()
vc_add_core_requirement_test(
  before_0_1_0 none "-DEXPECT_ERROR=Unknown CMake command \"vkc_require_core\"")
vc_add_core_requirement_test(
  0_0_9 0.0.9 "-DEXPECT_ERROR=needs volumetric_kit_core 0\\.1\\.0 or newer")
vc_add_core_requirement_test(0_1_0 0.1.0 -DEXPECT_CONFIG_VERSION=0.1)
vc_add_core_requirement_test(0_3_2 0.3.2 -DEXPECT_CONFIG_VERSION=0.3)
