# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# calib's brain with no Vulkan, and its installed package: run by the
# no_vulkan.brain test (tests/CMakeLists.txt), in BINARY_DIR, with
# find_package(Vulkan) disabled throughout, so a configure that reaches for
# Vulkan -- recon beyond its camera tier, or the core's vulkan tier -- fails
# rather than finds one.
#
# 1. With VR_CAMERA_ONLY OFF, as an application that needs the rest of recon sets
#    it, calib's configure keeps it, and so reaches for Vulkan and fails.
# 2. calib configures from CALIB_SOURCE, with VC_WITH_OPENCV as WITH_OPENCV says
#    (OpenCV found at OPENCV_DIR), builds and installs to BINARY_DIR/prefix:
#    compute's and solve's headers with it, none without, and never capture's.
# 3. The consumer (consumer/) finds that installed package, and with it the core
#    and recon installed beside it, and OpenCV with compute and solve, and builds
#    and runs against calib_core, and compute and solve when built. It finds them
#    by CMAKE_PREFIX_PATH; again finding the core itself too, before calib and
#    after it; and again by volumetric_kit_calib_DIR alone.
# 4. CMAKE_PREFIX_PATH, which takes precedence over calib's prefix, holds the
#    same recon and core, relabelled to the core's next minor version: the
#    consumer, given calib's directory, does not find calib, which names both
#    cores; nor when it finds that core itself first.
#
# CORE_SOURCE, RECON_SOURCE and JSON_SOURCE are the sources the calling build
# fetched, so this needs no network; GENERATOR, CXX_COMPILER, MAKE_PROGRAM and
# GLM_DIR repeat the calling build's settings. Each build and the install are
# Release, so a multi-config generator builds what it installs.
foreach(_var CALIB_SOURCE BINARY_DIR CORE_SOURCE RECON_SOURCE JSON_SOURCE
             GENERATOR)
  if(NOT DEFINED ${_var})
    message(FATAL_ERROR "configure.cmake: ${_var} is not set")
  endif()
endforeach()

# Runs `cmake <ARGN>`; fails, with its output, unless it succeeds.
function(_vc_cmake what)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" ${ARGN}
    RESULT_VARIABLE _result
    OUTPUT_VARIABLE _output
    ERROR_VARIABLE _output)
  if(NOT _result EQUAL 0)
    message(FATAL_ERROR "${what} failed:\n${_output}")
  endif()
endfunction()

# Runs `cmake <ARGN>`; fails unless it fails with output matching `expected`,
# each run of blanks and line breaks counting as one space.
function(_vc_cmake_refuses what expected)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" ${ARGN}
    RESULT_VARIABLE _result
    OUTPUT_VARIABLE _output
    ERROR_VARIABLE _output)
  string(REGEX REPLACE "[ \n]+" " " _flat "${_output}")
  if(_result EQUAL 0 OR NOT _flat MATCHES "${expected}")
    message(FATAL_ERROR "${what} did not fail with '${expected}':\n${_output}")
  endif()
endfunction()

# Replaces `from` with `to` in `file`; fails unless `from` is there.
function(_vc_relabel file from to)
  file(READ "${file}" _text)
  string(FIND "${_text}" "${from}" _at)
  if(_at EQUAL -1)
    message(FATAL_ERROR "${file} lacks '${from}':\n${_text}")
  endif()
  string(REPLACE "${from}" "${to}" _text "${_text}")
  file(WRITE "${file}" "${_text}")
endfunction()

set(_common
    -G
    "${GENERATOR}"
    "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
    "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}"
    "-Dglm_DIR=${GLM_DIR}"
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_DISABLE_FIND_PACKAGE_Vulkan=ON)
set(_prefix "${BINARY_DIR}/prefix")
if(WITH_OPENCV)
  set(_opencv -DVC_WITH_OPENCV=ON "-DOpenCV_DIR=${OPENCV_DIR}")
  set(_consumer_opencv -DEXPECT_COMPUTE=ON "-DOpenCV_DIR=${OPENCV_DIR}")
else()
  set(_opencv -DVC_WITH_OPENCV=OFF)
  set(_consumer_opencv -DEXPECT_COMPUTE=OFF)
endif()

file(REMOVE_RECURSE "${BINARY_DIR}")
_vc_cmake_refuses(
  "Configuring calib with VR_CAMERA_ONLY OFF"
  "CMAKE_DISABLE_FIND_PACKAGE_Vulkan"
  -S
  "${CALIB_SOURCE}"
  -B
  "${BINARY_DIR}/recon_whole"
  ${_common}
  "-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=${CORE_SOURCE}"
  "-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=${RECON_SOURCE}"
  "-DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=${JSON_SOURCE}"
  -DVR_CAMERA_ONLY=OFF
  -DVC_BUILD_TESTS=OFF
  -DVC_BUILD_EXAMPLES=OFF)

_vc_cmake(
  "Configuring calib with no Vulkan"
  -S
  "${CALIB_SOURCE}"
  -B
  "${BINARY_DIR}/calib"
  ${_common}
  "-DCMAKE_INSTALL_PREFIX=${_prefix}"
  "-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=${CORE_SOURCE}"
  "-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=${RECON_SOURCE}"
  "-DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=${JSON_SOURCE}"
  -DVC_BUILD_TESTS=OFF
  -DVC_BUILD_EXAMPLES=OFF
  ${_opencv})
_vc_cmake("Building calib" --build "${BINARY_DIR}/calib" --config Release)
_vc_cmake("Installing calib" --install "${BINARY_DIR}/calib" --config Release)
set(_calib_include "${_prefix}/include/volumetric_kit/calib")
foreach(_header compute/board.hpp solve/initialise.hpp)
  get_filename_component(_tier "${_header}" DIRECTORY)
  if(WITH_OPENCV AND NOT EXISTS "${_calib_include}/${_header}")
    message(FATAL_ERROR "calib with OpenCV installed no ${_header}")
  elseif(NOT WITH_OPENCV AND EXISTS "${_calib_include}/${_tier}")
    message(FATAL_ERROR "calib without OpenCV installed ${_tier}'s headers")
  endif()
endforeach()
if(EXISTS "${_calib_include}/capture")
  message(FATAL_ERROR "calib without the rig installed capture's headers")
endif()

_vc_cmake(
  "Configuring against the installed calib"
  -S
  "${CMAKE_CURRENT_LIST_DIR}/consumer"
  -B
  "${BINARY_DIR}/consumer"
  ${_common}
  "-DCMAKE_PREFIX_PATH=${_prefix}"
  ${_consumer_opencv})
_vc_cmake("Building against the installed calib" --build
          "${BINARY_DIR}/consumer" --config Release)

file(GLOB_RECURSE _consumer "${BINARY_DIR}/consumer/vc_brain_consumer")
if(NOT _consumer)
  message(FATAL_ERROR "the consumer built no vc_brain_consumer")
endif()
list(GET _consumer 0 _consumer)
execute_process(
  COMMAND "${_consumer}"
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _output
  ERROR_VARIABLE _output)
if(NOT _result EQUAL 0)
  message(FATAL_ERROR "the consumer failed (${_result}):\n${_output}")
endif()

_vc_cmake(
  "Configuring against the installed calib, finding the core too"
  -S
  "${CMAKE_CURRENT_LIST_DIR}/consumer"
  -B
  "${BINARY_DIR}/consumer_and_core"
  ${_common}
  "-DCMAKE_PREFIX_PATH=${_prefix}"
  -DFIND_CORE=ON
  ${_consumer_opencv})

set(_calib_dir "${_prefix}/lib/cmake/volumetric_kit_calib")
_vc_cmake(
  "Configuring against the installed calib by its directory alone"
  -S
  "${CMAKE_CURRENT_LIST_DIR}/consumer"
  -B
  "${BINARY_DIR}/consumer_by_dir"
  ${_common}
  "-Dvolumetric_kit_calib_DIR=${_calib_dir}"
  ${_consumer_opencv})

# The other prefix: calib's, less calib, with the core relabelled.
set(_other "${BINARY_DIR}/other_core")
file(COPY "${_prefix}/" DESTINATION "${_other}")
file(REMOVE_RECURSE "${_other}/lib/cmake/volumetric_kit_calib")
set(_core_dir "${_other}/lib/cmake/volumetric_kit_core")
file(READ "${_core_dir}/volumetric_kit_coreConfigVersion.cmake" _text)
if(NOT _text MATCHES
   "set\\(PACKAGE_VERSION \"(([0-9]+)\\.([0-9]+))(\\.[0-9]+)\"\\)")
  message(FATAL_ERROR "the installed core's version file names no version")
endif()
set(_version "${CMAKE_MATCH_1}${CMAKE_MATCH_4}")
set(_minor "${CMAKE_MATCH_1}")
set(_major "${CMAKE_MATCH_2}")
math(EXPR _next "${CMAKE_MATCH_3} + 1")
set(_next_minor "${_major}.${_next}")
_vc_relabel("${_core_dir}/volumetric_kit_coreConfigVersion.cmake"
            "\"${_version}\"" "\"${_next_minor}.0\"")
_vc_relabel(
  "${_core_dir}/volumetric_kit_coreTargets.cmake"
  "VKC_CORE_VERSION \"${_version}\"" "VKC_CORE_VERSION \"${_next_minor}.0\"")
_vc_relabel(
  "${_other}/lib/cmake/volumetric_kit_recon/volumetric_kit_reconConfig.cmake"
  "find_dependency(volumetric_kit_core ${_minor} CONFIG"
  "find_dependency(volumetric_kit_core ${_next_minor} CONFIG")
_vc_cmake_refuses(
  "Configuring against calib behind a recon with another core"
  "found volumetric_kit_core at .*/prefix/lib/cmake/volumetric_kit_core, but the volumetric_kit_recon at .*/other_core/lib/cmake/volumetric_kit_recon brings another, at .*/other_core/lib/cmake/volumetric_kit_core: put calib's prefix"
  -S
  "${CMAKE_CURRENT_LIST_DIR}/consumer"
  -B
  "${BINARY_DIR}/consumer_other_core"
  ${_common}
  "-DCMAKE_PREFIX_PATH=${_other}"
  "-Dvolumetric_kit_calib_DIR=${_calib_dir}")
_vc_cmake_refuses(
  "Configuring against calib after finding another core"
  "needs the volumetric_kit_core at .*/prefix/lib/cmake/volumetric_kit_core, but the application found another first, at .*/other_core/lib/cmake/volumetric_kit_core: find calib's"
  -S
  "${CMAKE_CURRENT_LIST_DIR}/consumer"
  -B
  "${BINARY_DIR}/consumer_other_core_first"
  ${_common}
  "-DCMAKE_PREFIX_PATH=${_other}"
  "-Dvolumetric_kit_calib_DIR=${_calib_dir}"
  -DFIND_CORE=ON)
