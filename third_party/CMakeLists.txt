# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Third-party dependencies.
#
# Vendored via pinned FetchContent: volumetric_kit_core (the family's shared
# foundation), volumetric_kit_sensor's camera tier (the family's camera
# vocabulary) -- with VC_WITH_RIG, its sensor tiers too -- volumetric_kit_gfx
# for the viz tier, nlohmann/json for the quality and capture tiers' files, and
# googletest when the test suite is built. sensor takes GLM, and with
# VC_WITH_RIG the Vulkan SDK, from the system. OpenCV is a prerequisite the root
# CMakeLists finds. GPU work comes from volumetric_kit_core's vulkan tier, not a
# Vulkan dependency of calib's.

include(FetchContent)

# Only applications need the common JSON format: the examples, and calib_rig
# (VC_BUILD_APPS, with VC_WITH_RIG's viz tier). The calibration library keeps
# its dependency on core_base and sensor_camera alone.
if(VC_BUILD_EXAMPLES OR (VC_BUILD_APPS AND VC_WITH_RIG))
  set(VKC_WITH_CONFIG ON)
endif()

# --- volumetric_kit_core -----------------------------------------------------
# The family's shared foundation: calib's Status / Result / VKC_CHECK / log sink
# come from its base tier (DECISIONS.md, "Build on volumetric_kit_core"). Pinned
# by commit, never main, so every consumer builds the same core. An application
# that also fetches sensor or gfx declares the core first; FetchContent keeps
# the first declaration of a name, so this pin then yields to the application's,
# and vkc_require_core refuses that core if it is older than the oldest calib
# builds with. Work on both checkouts at once with
# -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<core checkout>.
#
# The core's tests and -Werror stay off in a subproject. The brain needs only
# the base tier, so it builds with no Vulkan SDK: calib turns the vulkan tier on
# only with VC_WITH_RIG, for sensor's tiers, and otherwise leaves
# VKC_WITH_VULKAN unset rather than OFF, so an application that turns the tier
# on for sensor or gfx keeps it. Only the examples and calib_rig opt into
# core_config.
#
# The pin is core 0.30.0 (2120e62, core #70's merge), as are the core's actions
# in .github/workflows.
if(VC_WITH_RIG)
  set(VKC_WITH_VULKAN ON)
endif()
# The depth and viz tiers' GPU tests derive from the core's Vulkan test
# fixtures.
if(VC_WITH_RIG AND VC_BUILD_TESTS)
  set(VKC_BUILD_TEST_SUPPORT ON)
endif()
FetchContent_Declare(
  volumetric_kit_core
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_core.git
  GIT_TAG 2120e6294e3c7ead8bb168d757f940b21d21267c)
FetchContent_MakeAvailable(volumetric_kit_core)
# The set() above is lost when the application made the core available first.
if(VC_WITH_RIG)
  vkc_require_core(${VC_CORE_MIN_VERSION} VULKAN)
else()
  vkc_require_core(${VC_CORE_MIN_VERSION})
endif()
if(VC_BUILD_EXAMPLES AND NOT TARGET volumetric_kit::core_config)
  message(
    FATAL_ERROR "calib examples require volumetric_kit_core with core_config")
endif()

# --- volumetric_kit_sensor: the camera tier, or the rig's tiers too ----------
# calib's cameras are sensor's CameraModel, with its rigid transforms
# (DECISIONS.md, "Built on sensor, not recon"); with VC_WITH_RIG, the capture,
# depth, viz and app tiers take its sensor interface, prep, array, image I/O and
# Orbbec driver, and its exported pinhole.glsl. VS_CAMERA_ONLY builds
# sensor_camera alone, over the core's base tier and GLM, so calib still builds
# with no Vulkan SDK. Pinned by commit, never main, and declared after the core,
# so sensor's own core pin yields to calib's; vkc_require_sibling then refuses a
# sensor older than VC_SENSOR_MIN_VERSION or without a tier calib links.
# sensor's tests and -Werror stay off in a subproject. calib leaves an
# application's VS_CAMERA_ONLY as set, and its VS_WITH_ORBBEC, VS_WITH_FFMPEG
# and VS_WITH_CUDA apply. Work on both checkouts at once with
# -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_SENSOR=<sensor checkout>.
#
# The pin is sensor 0.1.0 (436840c, its repository's first main), as is its
# Orbbec SDK action in .github/workflows/_build.yml.
FetchContent_Declare(
  volumetric_kit_sensor
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_sensor.git
  GIT_TAG 436840cb269553cdcb6cbd6232c5ee1a643efc02)
if(VC_WITH_RIG)
  set(VS_CAMERA_ONLY OFF)
elseif(NOT DEFINED VS_CAMERA_ONLY)
  set(VS_CAMERA_ONLY ON)
endif()
FetchContent_MakeAvailable(volumetric_kit_sensor)
# The set() above is lost when the application made sensor available first.
if(VC_WITH_RIG)
  vkc_require_sibling(
    volumetric_kit::sensor
    ${VC_SENSOR_MIN_VERSION}
    TARGETS
    volumetric_kit::sensor_camera
    volumetric_kit::sensor_frame
    volumetric_kit::sensor_interface
    volumetric_kit::sensor_prep
    volumetric_kit::sensor_array
    volumetric_kit::sensor_image_io)
else()
  vkc_require_sibling(volumetric_kit::sensor ${VC_SENSOR_MIN_VERSION} TARGETS
                      volumetric_kit::sensor_camera)
endif()

# --- volumetric_kit_gfx: the viz tier's renderer and UI ----------------------
# The rig mosaic (ImagePipeline's region writes), the marks over its tiles
# (ImageOverlay's regions), the panels (ImGui, draw_metrics_panel), the views
# (ImageView2D, OrbitCamera) and the 3D view (DepthCloudPipeline's points from
# depth buffers, PrimitivePipeline's lines). Pinned by commit, never main, and
# declared after the core, so gfx's own core pin yields to calib's. gfx's tests,
# examples and -Werror stay off in a subproject.
#
# The pin is gfx main at #168's merge (df3a0de), on core 0.20.0, where every
# window renders at 4x MSAA. calib takes #145's overlay_config for the
# swapchain's ImGui overlay, #142's categorical_rgba8, the cameras' default
# tints, #141's 0 x 0 framebuffer while iconified, #138's DepthCloudPipeline and
# the region writes, and the marks over a region and the fractional-scale
# scissor repair of #136 and #137.
if(VC_WITH_RIG)
  FetchContent_Declare(
    volumetric_kit_gfx
    GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_gfx.git
    GIT_TAG df3a0de4184f6b989faa8d8ffbe0a338e00642a5)
  FetchContent_MakeAvailable(volumetric_kit_gfx)
endif()

# --- nlohmann/json (header-only; the quality and capture tiers' files) -------
# calib_quality writes its report, and calib_capture its session and capture
# files, with it, as a PRIVATE SYSTEM include, never part of a tier's public
# interface. Declared under sensor's name and pin, after sensor, so the one copy
# sensor fetched is the one used.
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW) # extracted files get the extraction time
endif()
FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  URL_HASH
    SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
  SOURCE_SUBDIR do-not-add-json-cmake)
FetchContent_MakeAvailable(nlohmann_json)
set(VC_JSON_INCLUDE_DIR
    "${nlohmann_json_SOURCE_DIR}/include"
    PARENT_SCOPE)

if(VC_BUILD_TESTS)
  # googletest is only needed when building the test suite.
  set(INSTALL_GTEST
      OFF
      CACHE BOOL "" FORCE)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(googletest)
endif()
