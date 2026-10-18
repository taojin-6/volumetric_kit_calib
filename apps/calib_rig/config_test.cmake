# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# What calib_rig refuses before it opens a window or a camera: each scenario
# below has one fault, which the core's config or calib_rig's check_scenario
# refuses, so no case reaches GLFW or the rig.

file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}")

# Runs calib_rig on `scenario` (a file in WORK_DIR, or none for --help) and
# fails unless it exits as `expect_ok` says, its stderr (or stdout) matching
# `pattern`.
function(expect_run name args expect_ok stream pattern)
  execute_process(
    COMMAND "${EXECUTABLE}" ${args}
    WORKING_DIRECTORY "${WORK_DIR}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error
    TIMEOUT 60)
  if(stream STREQUAL "stdout")
    set(text "${output}")
  else()
    set(text "${error}")
  endif()
  if(expect_ok)
    set(passed FALSE)
    if(result EQUAL 0)
      set(passed TRUE)
    endif()
  else()
    set(passed FALSE)
    if(NOT result EQUAL 0)
      set(passed TRUE)
    endif()
  endif()
  if(NOT passed OR NOT text MATCHES "${pattern}")
    message(FATAL_ERROR "${name}: exit ${result}, expected "
                        "${expect_ok}/${pattern}:\n${output}${error}")
  endif()
endfunction()

expect_run(help "--help" TRUE stdout "Usage: <executable> --config PATH")

# A scenario of calib_rig's whose sync file is `sync`, with `source` added to
# its source and `sections` after it; its session root `out`.
function(write_scenario name sync source out sections)
  file(
    WRITE "${WORK_DIR}/${name}.json"
    "{\"version\":1,\"operation\":\"calibrate\","
    "\"source\":{\"kind\":\"live\",\"sync_config\":\"${sync}\"${source}},"
    "\"output\":{\"path\":\"${out}\"}${sections}}\n")
endfunction()

# A sync file of one camera, so each case below fails for its own fault alone.
file(
  WRITE "${WORK_DIR}/rig.json"
  "{\"devices\":[{\"sn\":\"CL2A141000N\",\"syncConfig\":{"
  "\"syncMode\":\"OB_MULTI_DEVICE_SYNC_MODE_PRIMARY\",\"depthDelayUs\":0,"
  "\"colorDelayUs\":0,\"trigger2ImageDelayUs\":0,"
  "\"triggerOutEnable\":true,\"triggerOutDelayUs\":0,"
  "\"framesPerTrigger\":1}}]}\n")
file(MAKE_DIRECTORY "${WORK_DIR}/sessions")

# The session root must exist: nothing is created outside a session.
write_scenario(missing_root rig.json "" "no-such-dir" "")
expect_run(
  missing_root "--config;missing_root.json" FALSE stderr
  "NotFound: output\\.path: .*no-such-dir is not an existing directory")

# A depth size the Femto Mega has no mode for.
write_scenario(odd_depth rig.json ",\"depth_width\":800,\"depth_height\":600"
               "sessions" "")
expect_run(odd_depth "--config;odd_depth.json" FALSE stderr
           "source\\.depth_width/depth_height: 800x600 is not a depth mode")

# The sync file: missing, not one, or naming no camera.
write_scenario(missing_sync no-such-rig.json "" "sessions" "")
expect_run(missing_sync "--config;missing_sync.json" FALSE stderr
           "NotFound: source\\.sync_config: .*no-such-rig\\.json")
file(WRITE "${WORK_DIR}/bad_rig.json" "{\"devices\":[{\"sn\":1}]}\n")
write_scenario(bad_sync bad_rig.json "" "sessions" "")
expect_run(bad_sync "--config;bad_sync.json" FALSE stderr
           "InvalidArgument: source\\.sync_config: .*bad_rig\\.json")
file(WRITE "${WORK_DIR}/empty_rig.json" "{\"devices\":[]}\n")
write_scenario(empty_sync empty_rig.json "" "sessions" "")
expect_run(
  empty_sync "--config;empty_sync.json" FALSE stderr
  "source\\.sync_config: .*empty_rig\\.json: sync config: no \"devices\"")

# The core's refusals reach the user: H.265 colour, and a setting calib_rig does
# not use.
write_scenario(hevc rig.json ",\"color_codec\":\"hevc\"" "sessions" "")
expect_run(hevc "--config;hevc.json" FALSE stderr "source\\.color_codec")
write_scenario(overlay rig.json "" "sessions" ",\"window\":{\"overlay\":false}")
expect_run(overlay "--config;overlay.json" FALSE stderr
           "window\\.overlay: is not used by this application")

# Nothing was written under the session root.
file(GLOB written "${WORK_DIR}/sessions/*")
if(written)
  message(FATAL_ERROR "calib_rig wrote under the session root: ${written}")
endif()
