# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

include(GoogleTest)

# vc_<name>_test from <name>_test.cpp, linking the given targets. A case that
# hangs fails after 5 minutes rather than holding a CI job to its limit. Under
# the sanitizers, LeakSanitizer reads lsan.supp, which spares OpenCV's own leak
# when an out-of-memory test fails an allocation inside it.
set(_vc_test_properties TIMEOUT 300)
if(VC_SANITIZE)
  list(APPEND _vc_test_properties ENVIRONMENT
       "LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_SOURCE_DIR}/lsan.supp")
endif()
function(vc_add_test name)
  add_executable(vc_${name}_test ${name}_test.cpp)
  target_link_libraries(vc_${name}_test PRIVATE ${ARGN} GTest::gtest_main)
  vc_target_warnings(vc_${name}_test)
  gtest_discover_tests(vc_${name}_test PROPERTIES ${_vc_test_properties})
endfunction()

# The tests' header-only helpers (support/); allocation_failure.cpp, which an
# out-of-memory test adds, walks the stack with backtrace and dladdr.
add_library(vc_test_support INTERFACE)
target_include_directories(vc_test_support
                           INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/support")
target_link_libraries(vc_test_support INTERFACE volumetric_kit::calib_core
                                                GTest::gtest ${CMAKE_DL_LIBS})
# The out-of-memory sweep itself.
vc_add_test(allocation_failure vc_test_support)
target_sources(vc_allocation_failure_test
               PRIVATE support/allocation_failure.cpp)

# version smoke test.
vc_add_test(version volumetric_kit::calib_core)
# core: the board, its check, and the rig's board.
vc_add_test(target vc_test_support)
# core: the plain records (Observation / Detection / ViewCheck / CaptureMeta /
# SensorProfile / LensProfile).
vc_add_test(data_model volumetric_kit::calib_core)

# The OpenCV modules only the tests use: imgcodecs, to compress rendered views,
# and calib, the bundle adjustment's oracle.
find_package(OpenCV 5.0 CONFIG REQUIRED COMPONENTS imgcodecs calib)

# The synthetic source every calib test draws on (support/): lenses, a rig, a
# plan of placements and an exact-inverse renderer, over compute's bridge.
add_library(vc_synthetic_rig STATIC EXCLUDE_FROM_ALL support/synthetic_rig.cpp)
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
vc_add_test(measures volumetric_kit::calib_compute)
vc_add_test(cv_camera vc_synthetic_rig opencv_geometry)
vc_add_test(synthetic_rig vc_synthetic_rig opencv_geometry)
# Keep allocation replacement isolated from the other test executables.
vc_add_test(compute_oom volumetric_kit::calib_compute vc_test_support)
target_sources(vc_compute_oom_test PRIVATE support/allocation_failure.cpp)

# detect: the board in rendered camera images, JPEG-compressed too; the origin
# tag warped into them.
vc_add_test(detect_charuco volumetric_kit::calib_detect vc_synthetic_rig
            opencv_imgcodecs opencv_imgproc)
vc_add_test(detect_tag volumetric_kit::calib_detect vc_synthetic_rig)
vc_add_test(detect_oom volumetric_kit::calib_detect vc_test_support)
target_sources(vc_detect_oom_test PRIVATE support/allocation_failure.cpp)

# solve: views, the spanning tree, the first rig and the bundle adjustment, on
# the synthetic rig; OpenCV's calibrateMultiview is the adjustment's oracle on
# full views.
vc_add_test(view_pose volumetric_kit::calib_solve vc_synthetic_rig)
vc_add_test(spanning_tree volumetric_kit::calib_solve)
vc_add_test(initialise volumetric_kit::calib_solve vc_synthetic_rig)
vc_add_test(bundle_adjust volumetric_kit::calib_solve vc_synthetic_rig
            opencv_calib opencv_geometry)
# solve: the world at the origin tag on the synthetic rig, and the outputs moved
# into it, read back with recon's calibration file.
vc_add_test(world_tag volumetric_kit::calib_solve vc_synthetic_rig)
vc_add_test(solve_oom volumetric_kit::calib_solve vc_synthetic_rig)
target_sources(vc_solve_oom_test PRIVATE support/allocation_failure.cpp)

# quality: the live checks against known poses and rendered views, the capture
# rule's states, and the session's checks on the synthetic rig.
vc_add_test(live_checks volumetric_kit::calib_quality
            volumetric_kit::calib_detect vc_synthetic_rig)
vc_add_test(capture_rule volumetric_kit::calib_quality)
vc_add_test(session_checks volumetric_kit::calib_quality vc_synthetic_rig)
vc_add_test(quality_oom volumetric_kit::calib_quality vc_synthetic_rig)
target_sources(vc_quality_oom_test PRIVATE support/allocation_failure.cpp)
# quality: the solved rig's metrics on the synthetic rig, the transfer against
# verify_rig.py's check in OpenCV; its results files read back with nlohmann and
# recon's reader.
vc_add_test(calibration_metrics volumetric_kit::calib_quality vc_synthetic_rig
            opencv_geometry)
# quality: the depth checks' geometry against planes and corners made here, and
# their verdicts, on rigs of 1, 3, 4 and 5 cameras.
vc_add_test(depth_checks volumetric_kit::calib_quality vc_synthetic_rig)
# quality: the boards on the floor of a tag's world, placed here.
vc_add_test(floor_boards volumetric_kit::calib_quality vc_synthetic_rig)
vc_add_test(results volumetric_kit::calib_quality vc_synthetic_rig)
target_include_directories(vc_results_test SYSTEM
                           PRIVATE "${VC_JSON_INCLUDE_DIR}")

if(VC_WITH_RIG)
  # capture: sessions on disk, read back with recon's readers and nlohmann.
  vc_add_test(session_writer volumetric_kit::calib_capture
              volumetric_kit::recon_io vc_test_support Threads::Threads)
  target_include_directories(vc_session_writer_test SYSTEM
                             PRIVATE "${VC_JSON_INCLUDE_DIR}")
  vc_add_test(capture_oom volumetric_kit::calib_capture vc_test_support
              Threads::Threads)
  target_sources(vc_capture_oom_test PRIVATE support/allocation_failure.cpp)

  # depth: the board-plane fit and the pair agreement on a device, through the
  # core's Vulkan test fixtures (they skip without one), against references in
  # double made here; the board region's rule on the device, pixel by pixel,
  # through a kernel of the test's own over the tier's GLSL
  # (shaders/board_member_cases.comp); and host allocation failures, the Vulkan
  # driver's spared.
  vc_add_test(depth_plane_fit volumetric_kit::calib_depth
              volumetric_kit::core_test_support vc_synthetic_rig)
  get_target_property(_vc_glsl_roots calib_depth VC_SHADER_INCLUDE_DIRS)
  vkc_embed_shaders(
    vc_depth_plane_fit_test
    SYMBOL_PREFIX
    vc_test_
    INCLUDE_DIRS
    ${_vc_glsl_roots}
    SHADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/board_member_cases.comp")
  vc_add_test(depth_agreement volumetric_kit::calib_depth
              volumetric_kit::core_test_support vc_synthetic_rig)
  # The agreement kernel's SPIR-V too, whose shared memory the test sums.
  vkc_embed_shaders(
    vc_depth_agreement_test
    SYMBOL_PREFIX
    vc_test_
    INCLUDE_DIRS
    ${_vc_glsl_roots}
    SHADERS
    "${PROJECT_SOURCE_DIR}/src/volumetric_kit/calib/depth/shaders/agreement.comp"
  )
  vc_add_test(depth_oom volumetric_kit::calib_depth
              volumetric_kit::core_test_support vc_test_support)
  target_sources(vc_depth_oom_test PRIVATE support/allocation_failure.cpp)

  # viz: the mosaic's layout, the marks, the panels and the board editor from
  # host data; the offscreen render of the mosaic and its marks on a device,
  # through the core's Vulkan test fixtures (they skip without one).
  vc_add_test(viz_mosaic volumetric_kit::calib_viz)
  vc_add_test(viz_overlay volumetric_kit::calib_viz)
  vc_add_test(viz_panels volumetric_kit::calib_viz)
  vc_add_test(board_editor volumetric_kit::calib_viz)
  vc_add_test(viz_render volumetric_kit::calib_viz
              volumetric_kit::core_test_support)
  # viz: the 3D view's frusta, axes, boards, display rotation and window split
  # from host data; each camera's prepared frame as gfx's draw, on real buffers.
  vc_add_test(viz_scene volumetric_kit::calib_viz vc_synthetic_rig)
  vc_add_test(viz_point_cloud volumetric_kit::calib_viz
              volumetric_kit::core_test_support)
  vc_add_test(viz_oom volumetric_kit::calib_viz vc_test_support)
  target_sources(vc_viz_oom_test PRIVATE support/allocation_failure.cpp)
endif()

if(TARGET calib_app)
  # app: the preset and the keys from host data; the live worker over fake
  # cameras whose synthetic board pictures sit in device images, and the mosaic
  # drawn from its snapshots, through the core's Vulkan test fixtures (they skip
  # without a device); the capture worker's sessions on disk, read back with
  # recon's readers and nlohmann.
  vc_add_test(app_preset volumetric_kit::calib_app)
  # GLFW's key codes, through gfx_glfw, whose GLFW is gfx's own find.
  vc_add_test(app_keys volumetric_kit::calib_app volumetric_kit::gfx_glfw)
  # The window's mode: each key's action in each view, Calibrate's gate, the 3D
  # view's placement and framing.
  vc_add_test(app_rig_mode volumetric_kit::calib_app)
  vc_add_test(app_live volumetric_kit::calib_app vc_synthetic_rig
              volumetric_kit::core_test_support)
  vc_add_test(app_live_view volumetric_kit::calib_app vc_synthetic_rig
              volumetric_kit::core_test_support)
  # app: the 3D view's worker over synthetic frames, and its points drawn on the
  # renderer's queue family through gfx's depth-cloud pipeline.
  vc_add_test(app_cloud volumetric_kit::calib_app
              volumetric_kit::core_test_support)
  # calib_capture before recon_io, which it links, so recon_io is linked once.
  # allocation_failure.cpp fails a detection and counts the depth buffers.
  vc_add_test(app_capture volumetric_kit::calib_app vc_synthetic_rig
              volumetric_kit::calib_capture volumetric_kit::recon_io)
  target_sources(vc_app_capture_test PRIVATE support/allocation_failure.cpp)
  target_include_directories(vc_app_capture_test SYSTEM
                             PRIVATE "${VC_JSON_INCLUDE_DIR}")
  # The calib commit a session records is the checkout's HEAD, "-dirty"
  # included, as git gives it when the tests run (app_software_commit.cmake).
  add_test(
    NAME app_capture.software_commit
    COMMAND
      ${CMAKE_COMMAND} -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
      -DTEST=$<TARGET_FILE:vc_app_capture_test> -P
      ${CMAKE_CURRENT_SOURCE_DIR}/app_software_commit.cmake)
  set_tests_properties(app_capture.software_commit
                       PROPERTIES SKIP_REGULAR_EXPRESSION "not a git checkout")
  # app: Calibrate on the synthetic rig's detections, its results directories
  # read back with recon's reader and nlohmann; its out-of-memory sweep.
  vc_add_test(app_calibrate volumetric_kit::calib_app vc_synthetic_rig)
  target_sources(vc_app_calibrate_test PRIVATE support/allocation_failure.cpp)
  target_include_directories(vc_app_calibrate_test SYSTEM
                             PRIVATE "${VC_JSON_INCLUDE_DIR}")
  # app: Calibrate's depth checks on a device, over raw depth rendered through
  # the synthetic depth lens, beside the 3D view's worker across two queue
  # families, through the core's Vulkan test fixtures (they skip without one).
  vc_add_test(app_calibrate_depth volumetric_kit::calib_app vc_synthetic_rig
              volumetric_kit::core_test_support)
  target_include_directories(vc_app_calibrate_depth_test SYSTEM
                             PRIVATE "${VC_JSON_INCLUDE_DIR}")
endif()

# The core calib requires (third_party/CMakeLists.txt): calib's configure
# against a stand-in core of each version (core_requirement/) refuses one older
# than VC_CORE_MIN_VERSION, 0.6.1 -- before 0.1.0 the core has no
# vkc_require_core -- and accepts 0.6.1 or newer.
#
# Those configures, and the ones below, take the fetched sources this build used
# (FETCHCONTENT_SOURCE_DIR_<NAME>), so they need no network, and the OpenCV it
# found. They check calib's build, not its code, so they carry the label
# `package`, which the default local run leaves out and CI runs on one leg, and
# a cost that starts them first.
set(_vc_package_properties LABELS package COST 60)
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
    -DGLM_DIR=${glm_DIR}
    -DOPENCV_DIR=${OpenCV_DIR})
function(vc_add_core_requirement_test name core_version)
  add_test(
    NAME core_requirement.${name}
    COMMAND
      ${CMAKE_COMMAND} -DCALIB_SOURCE=${PROJECT_SOURCE_DIR}
      -DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/core_requirement/${name}
      -DCORE_VERSION=${core_version}
      -DCORE_MODULES=${volumetric_kit_core_SOURCE_DIR}/cmake ${ARGN}
      ${_vc_fetched_sources} -P
      ${CMAKE_CURRENT_SOURCE_DIR}/core_requirement/configure.cmake)
  set_tests_properties(core_requirement.${name}
                       PROPERTIES ${_vc_package_properties})
endfunction()
vc_add_core_requirement_test(
  before_0_1_0 none "-DEXPECT_ERROR=Unknown CMake command \"vkc_require_core\"")
vc_add_core_requirement_test(
  0_2_0 0.2.0 "-DEXPECT_ERROR=needs volumetric_kit_core 0\\.6\\.1 or newer")
vc_add_core_requirement_test(
  0_6_0 0.6.0 "-DEXPECT_ERROR=needs volumetric_kit_core 0\\.6\\.1 or newer")
vc_add_core_requirement_test(0_6_1 0.6.1)
vc_add_core_requirement_test(0_7_2 0.7.2)

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
    set_tests_properties(rig_requirement.${name}
                         PROPERTIES ${_vc_package_properties})
  endfunction()
  vc_add_rig_requirement_test(
    core_without_vulkan OFF
    "volumetric_kit_calib needs volumetric_kit_core's vulkan tier")
  vc_add_rig_requirement_test(
    camera_only_recon
    ON
    "VC_WITH_RIG needs all of volumetric_kit_recon, but it was made available camera-only"
  )

  if(TARGET calib_app)
    # calib_rig is an application's own choice (VC_BUILD_APPS): an application
    # that adds calib with every one of calib_rig's prerequisites, having made
    # the core available without its config tier, configures with calib_rig left
    # out, and stops, naming the config tier, when it asks for it
    # (app_requirement/). It takes every source this build fetched.
    set(_vc_fetched)
    get_cmake_property(_vc_cache_vars CACHE_VARIABLES)
    foreach(_vc_var IN LISTS _vc_cache_vars)
      if(NOT _vc_var MATCHES "^FETCHCONTENT_SOURCE_DIR_(.+)$")
        continue()
      endif()
      set(_vc_name "${CMAKE_MATCH_1}")
      if(NOT _vc_name MATCHES "^VOLUMETRIC_KIT_(CORE|RECON)$")
        string(TOLOWER "${_vc_name}" _vc_lower)
        FetchContent_GetProperties(${_vc_lower})
        if(${_vc_lower}_SOURCE_DIR)
          list(APPEND _vc_fetched "${_vc_name}=${${_vc_lower}_SOURCE_DIR}")
        endif()
      endif()
    endforeach()
    list(JOIN _vc_fetched "|" _vc_fetched)
    function(vc_add_app_requirement_test name build_apps)
      add_test(
        NAME app_requirement.${name}
        COMMAND
          ${CMAKE_COMMAND} -DCALIB_SOURCE=${PROJECT_SOURCE_DIR}
          -DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/app_requirement/${name}
          -DCORE_SOURCE=${volumetric_kit_core_SOURCE_DIR}
          -DRECON_SOURCE=${volumetric_kit_recon_SOURCE_DIR}
          "-DFETCHED=${_vc_fetched}" -DGENERATOR=${CMAKE_GENERATOR}
          -DCXX_COMPILER=${CMAKE_CXX_COMPILER}
          -DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM} -DGLM_DIR=${glm_DIR}
          -DGLFW_DIR=${glfw3_DIR} -DOPENCV_DIR=${OpenCV_DIR}
          -DORBBEC_DIR=${OrbbecSDK_DIR} -DBUILD_APPS=${build_apps} ${ARGN} -P
          ${CMAKE_CURRENT_SOURCE_DIR}/app_requirement/configure.cmake)
      set_tests_properties(app_requirement.${name}
                           PROPERTIES ${_vc_package_properties})
    endfunction()
    vc_add_app_requirement_test(not_asked OFF)
    vc_add_app_requirement_test(
      asked
      ON
      "-DEXPECT_ERROR=volumetric_kit_calib needs volumetric_kit_core's config tier"
    )
  endif()
endif()

# The brain needs no Vulkan (no_vulkan/): with find_package(Vulkan) disabled,
# calib configures and builds it, and a configure that keeps all of recon fails.
add_test(
  NAME no_vulkan.brain
  COMMAND
    ${CMAKE_COMMAND} -DCALIB_SOURCE=${PROJECT_SOURCE_DIR}
    -DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/no_vulkan
    -DCORE_SOURCE=${volumetric_kit_core_SOURCE_DIR} ${_vc_fetched_sources} -P
    ${CMAKE_CURRENT_SOURCE_DIR}/no_vulkan/configure.cmake)
set_tests_properties(no_vulkan.brain PROPERTIES ${_vc_package_properties})
