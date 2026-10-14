# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

include(GoogleTest)

# vc_<name>_test from <name>_test.cpp, linking the given targets. A case that
# hangs fails after 5 minutes rather than holding a CI job to its limit.
function(vc_add_test name)
  add_executable(vc_${name}_test ${name}_test.cpp)
  target_link_libraries(vc_${name}_test PRIVATE ${ARGN} GTest::gtest_main)
  vc_target_warnings(vc_${name}_test)
  gtest_discover_tests(
    vc_${name}_test
    PROPERTIES
    TIMEOUT 300)
endfunction()

# The tests' header-only helpers (support/).
add_library(vc_test_support INTERFACE)
target_include_directories(vc_test_support
                           INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/support")
target_link_libraries(vc_test_support INTERFACE volumetric_kit::calib_core
                                                GTest::gtest)

# version smoke test.
vc_add_test(version volumetric_kit::calib_core)
# core: the board, its check, and the rig's board.
vc_add_test(target vc_test_support)
# core: the plain records (Observation / Detection / ViewCheck / CaptureMeta /
# SensorProfile / LensProfile).
vc_add_test(data_model volumetric_kit::calib_core)

if(VC_WITH_OPENCV)
  # The synthetic source every calib test draws on (support/): lenses, a rig, a
  # plan of placements and an exact-inverse renderer, over compute's bridge.
  add_library(vc_synthetic_rig STATIC EXCLUDE_FROM_ALL
              support/synthetic_rig.cpp)
  target_link_libraries(
    vc_synthetic_rig
    PUBLIC vc_test_support volumetric_kit::calib_compute
    PRIVATE opencv_geometry opencv_imgproc)
  vc_target_warnings(vc_synthetic_rig)

  # compute: the OpenCV bridge, and the synthetic source itself.
  vc_add_test(guarded volumetric_kit::calib_compute)
  vc_add_test(cv_pose volumetric_kit::calib_compute opencv_geometry)
  vc_add_test(board volumetric_kit::calib_compute vc_test_support)
  vc_add_test(usable_for_pose volumetric_kit::calib_compute)
  vc_add_test(cv_camera vc_synthetic_rig opencv_geometry)
  vc_add_test(synthetic_rig vc_synthetic_rig opencv_geometry)
  # Keep allocation replacement isolated from the other test executables.
  vc_add_test(compute_oom volumetric_kit::calib_compute vc_test_support)
  target_sources(vc_compute_oom_test PRIVATE support/allocation_failure.cpp)

  # detect: the board in rendered camera images, JPEG-compressed too.
  find_package(OpenCV 5.0 CONFIG REQUIRED COMPONENTS imgcodecs)
  vc_add_test(detect_charuco volumetric_kit::calib_detect vc_synthetic_rig
              opencv_imgcodecs opencv_imgproc)
  vc_add_test(detect_oom volumetric_kit::calib_detect vc_test_support)
  target_sources(vc_detect_oom_test PRIVATE support/allocation_failure.cpp)

  # solve: views, the spanning tree, the first rig and the bundle adjustment, on
  # the synthetic rig; OpenCV's calibrateMultiview is the adjustment's oracle on
  # full views.
  find_package(OpenCV 5.0 CONFIG REQUIRED COMPONENTS calib)
  vc_add_test(view_pose volumetric_kit::calib_solve vc_synthetic_rig)
  vc_add_test(spanning_tree volumetric_kit::calib_solve)
  vc_add_test(initialise volumetric_kit::calib_solve vc_synthetic_rig)
  vc_add_test(bundle_adjust volumetric_kit::calib_solve vc_synthetic_rig
              opencv_calib opencv_geometry)
  vc_add_test(solve_oom volumetric_kit::calib_solve vc_synthetic_rig)
  target_sources(vc_solve_oom_test PRIVATE support/allocation_failure.cpp)
endif()

if(VC_WITH_RIG)
  # capture: sessions on disk, read back with recon's readers and nlohmann.
  vc_add_test(session_writer volumetric_kit::calib_capture
              volumetric_kit::recon_io Threads::Threads)
  target_include_directories(vc_session_writer_test SYSTEM
                             PRIVATE "${VC_JSON_INCLUDE_DIR}")
  vc_add_test(capture_oom volumetric_kit::calib_capture vc_test_support
              Threads::Threads)
  target_sources(vc_capture_oom_test PRIVATE support/allocation_failure.cpp)
endif()

# The core calib requires (third_party/CMakeLists.txt): calib's configure
# against a stand-in core of each version (core_requirement/) refuses one older
# than VC_CORE_MIN_VERSION, 0.6.1 -- before 0.1.0 the core has no
# vkc_require_core -- and accepts 0.6.1 or newer, its package config finding
# that core at its version and requiring VC_CORE_MIN_VERSION of it, then recon,
# whose package config re-finds the core at its MAJOR.MINOR.
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
  0_2_0 0.2.0 "-DEXPECT_ERROR=needs volumetric_kit_core 0\\.6\\.1 or newer")
vc_add_core_requirement_test(
  0_6_0 0.6.0 "-DEXPECT_ERROR=needs volumetric_kit_core 0\\.6\\.1 or newer")
vc_add_core_requirement_test(0_6_1 0.6.1 -DEXPECT_CONFIG_VERSION=0.6.1)
vc_add_core_requirement_test(0_7_2 0.7.2 -DEXPECT_CONFIG_VERSION=0.7.2)

if(VC_WITH_RIG)
  # VC_WITH_RIG needs the core's vulkan tier and all of recon: an application
  # that made either available without them first stops calib's configure, which
  # names what is missing (rig_requirement/).
  function(vc_add_rig_requirement_test name core_vulkan expect)
    add_test(
      NAME rig_requirement.${name}
      COMMAND
        ${CMAKE_COMMAND} -DCALIB_SOURCE=${PROJECT_SOURCE_DIR}
        -DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/rig_requirement/${name}
        -DCORE_SOURCE=${volumetric_kit_core_SOURCE_DIR} ${_vc_fetched_sources}
        -DCORE_VULKAN=${core_vulkan} "-DEXPECT_ERROR=${expect}" -P
        ${CMAKE_CURRENT_SOURCE_DIR}/rig_requirement/configure.cmake)
  endfunction()
  vc_add_rig_requirement_test(
    core_without_vulkan OFF
    "volumetric_kit_calib needs volumetric_kit_core's vulkan tier")
  vc_add_rig_requirement_test(
    camera_only_recon
    ON
    "VC_WITH_RIG needs all of volumetric_kit_recon, but it was made available camera-only"
  )

  # The capture tier from this build's install (rig_package/): a consumer finds
  # calib, the core and all of recon at the prefix, and builds and runs against
  # calib_capture, with this build's sanitizers, which its libraries need.
  list(JOIN _vc_sanitize_flags " " _vc_consumer_flags)
  add_test(
    NAME rig_package.consumer
    COMMAND
      ${CMAKE_COMMAND} -DBUILD_DIR=${PROJECT_BINARY_DIR} -DCONFIG=$<CONFIG>
      -DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/rig_package
      -DGENERATOR=${CMAKE_GENERATOR} -DCXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM} -DGLM_DIR=${glm_DIR}
      "-DFLAGS=${_vc_consumer_flags}" -P
      ${CMAKE_CURRENT_SOURCE_DIR}/rig_package/configure.cmake)
endif()

# VC_WITH_OPENCV without an OpenCV 5 to find stops the configure, naming the
# fixes (opencv_requirement/).
add_test(
  NAME opencv_requirement.missing
  COMMAND
    ${CMAKE_COMMAND} -DCALIB_SOURCE=${PROJECT_SOURCE_DIR}
    -DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/opencv_requirement
    -DGENERATOR=${CMAKE_GENERATOR} -DCXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM} -P
    ${CMAKE_CURRENT_SOURCE_DIR}/opencv_requirement/configure.cmake)

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
