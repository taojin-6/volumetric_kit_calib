# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Shared compiler-warning flags, applied PRIVATE so they never leak to consumers
# (a consumer's build should not inherit our warning policy). Used as
# `vc_target_warnings(<target>)` on every first-party target.
function(vc_target_warnings target)
  # TODO: no supported build reaches this MSVC branch: volumetric_kit_core,
  # whose headers calib's include, supports only GCC and Clang (its
  # DECISIONS.md, "Platforms"). Drop it, and vc_sanitizers.cmake's, once calib
  # records the same platform decision.
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
    if(VC_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    if(VC_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()
