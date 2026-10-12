# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

include(GoogleTest)

# version smoke test.
add_executable(vc_version_test version_test.cpp)
target_link_libraries(vc_version_test PRIVATE volumetric_kit::calib_core
                                              GTest::gtest_main)
vc_target_warnings(vc_version_test)
gtest_discover_tests(vc_version_test)

# core: the board, its check, and the rig's board.
add_executable(vc_target_test target_test.cpp)
target_link_libraries(vc_target_test PRIVATE volumetric_kit::calib_core
                                             GTest::gtest_main)
vc_target_warnings(vc_target_test)
gtest_discover_tests(vc_target_test)

# core: the plain records (Observation / ViewCheck / CaptureMeta / SensorProfile
# / LensProfile).
add_executable(vc_data_model_test data_model_test.cpp)
target_link_libraries(vc_data_model_test PRIVATE volumetric_kit::calib_core
                                                 GTest::gtest_main)
vc_target_warnings(vc_data_model_test)
gtest_discover_tests(vc_data_model_test)

if(VC_WITH_OPENCV)
  # The synthetic source every calib test draws on (support/): lenses, a rig, a
  # plan of placements and an exact-inverse renderer, over compute's bridge.
  add_library(vc_synthetic_rig STATIC EXCLUDE_FROM_ALL
              support/synthetic_rig.cpp)
  target_include_directories(vc_synthetic_rig
                             PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/support")
  target_link_libraries(vc_synthetic_rig PUBLIC volumetric_kit::calib_compute
                                                opencv_geometry opencv_imgproc)
  vc_target_warnings(vc_synthetic_rig)

  # compute: the OpenCV bridge, and the synthetic source itself.
  foreach(_vc_test cv_camera cv_pose board guarded synthetic_rig)
    add_executable(vc_${_vc_test}_test ${_vc_test}_test.cpp)
    target_link_libraries(vc_${_vc_test}_test PRIVATE vc_synthetic_rig
                                                      GTest::gtest_main)
    vc_target_warnings(vc_${_vc_test}_test)
    gtest_discover_tests(vc_${_vc_test}_test)
  endforeach()
endif()

# The core calib requires (third_party/CMakeLists.txt): calib's configure
# against a stand-in core of each version (core_requirement/) refuses one older
# than VC_CORE_MIN_VERSION, 0.2.0 -- before 0.1.0 the core has no
# vkc_require_core -- and accepts 0.2.0 or newer, its package config re-finding
# recon, whose package config re-finds the core, and holding that core to its
# MAJOR.MINOR and VC_CORE_MIN_VERSION.
#
# Those configures, and the one below, take the fetched sources this build used
# (FETCHCONTENT_SOURCE_DIR_<NAME>), so they need no network.
include(FetchContent)
FetchContent_GetProperties(volumetric_kit_core)
FetchContent_GetProperties(volumetric_kit_recon)
FetchContent_GetProperties(nlohmann_json)
set(_vc_fetched_sources
    -DRECON_SOURCE=${volumetric_kit_recon_SOURCE_DIR}
    -DJSON_SOURCE=${nlohmann_json_SOURCE_DIR}
    -DGENERATOR=${CMAKE_GENERATOR}
    -DCXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}
    -DGLM_DIR=${glm_DIR})
function(vc_add_core_requirement_test name core_version expect)
  add_test(
    NAME core_requirement.${name}
    COMMAND
      ${CMAKE_COMMAND} -DCALIB_SOURCE=${PROJECT_SOURCE_DIR}
      -DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/core_requirement/${name}
      -DCORE_VERSION=${core_version}
      -DCORE_MODULES=${volumetric_kit_core_SOURCE_DIR}/cmake
      -DMIN_VERSION=${VC_CORE_MIN_VERSION} ${expect} ${_vc_fetched_sources} -P
      ${CMAKE_CURRENT_SOURCE_DIR}/core_requirement/configure.cmake)
endfunction()
vc_add_core_requirement_test(
  before_0_1_0 none "-DEXPECT_ERROR=Unknown CMake command \"vkc_require_core\"")
vc_add_core_requirement_test(
  0_0_9 0.0.9 "-DEXPECT_ERROR=needs volumetric_kit_core 0\\.2\\.0 or newer")
vc_add_core_requirement_test(
  0_1_0 0.1.0 "-DEXPECT_ERROR=needs volumetric_kit_core 0\\.2\\.0 or newer")
vc_add_core_requirement_test(0_2_0 0.2.0 -DEXPECT_CONFIG_VERSION=0.2)
vc_add_core_requirement_test(0_3_2 0.3.2 -DEXPECT_CONFIG_VERSION=0.3)

# The brain needs no Vulkan (no_vulkan/): calib, with OpenCV as this build has
# it, configures, builds and installs with find_package(Vulkan) disabled, and a
# consumer builds against the installed package with it disabled too; the
# installed package refuses a core of another minor version.
add_test(
  NAME no_vulkan.brain
  COMMAND
    ${CMAKE_COMMAND} -DCALIB_SOURCE=${PROJECT_SOURCE_DIR}
    -DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/no_vulkan
    -DCORE_SOURCE=${volumetric_kit_core_SOURCE_DIR} ${_vc_fetched_sources}
    -DWITH_OPENCV=${VC_WITH_OPENCV} -DOPENCV_DIR=${OpenCV_DIR} -P
    ${CMAKE_CURRENT_SOURCE_DIR}/no_vulkan/configure.cmake)
