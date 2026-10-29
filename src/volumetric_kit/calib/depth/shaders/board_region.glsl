// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The board region's rule on the device: quality::in_board_region's, for a
// depth pixel. plane_moments.comp classifies the region's pixels by it, and
// tests/shaders/board_member_cases.comp holds it to the host's, pixel by pixel.

#ifndef VC_DEPTH_BOARD_REGION_GLSL
#define VC_DEPTH_BOARD_REGION_GLSL

#include "volumetric_kit/sensor/frame/shaders/pinhole.glsl"

// Whether depth pixel `pixel` belongs to the region: its ray, the pixel
// unprojected at depth 1 through the pinhole `k` (fx, fy, cx, cy), moved into
// the board's frame by `depth_to_board`, meets the board's plane, z = 0, in
// front of the camera, inside `rect` (min.x, min.y, max.x, max.y; edges
// included). Whatever its depth, so a pixel with none belongs as one with
// does. The tests are negated, so a NaN fails them.
bool vc_board_member(uvec2 pixel, vec4 k, mat4 depth_to_board, vec4 rect) {
  vec3 origin = depth_to_board[3].xyz;
  vec3 ray = vs_unproject_pinhole(vec2(pixel), 1.0, k);
  vec3 direction = mat3(depth_to_board) * ray;
  float s = -origin.z / direction.z;
  if (!(s > 0.0) || isinf(s)) {
    return false;  // behind the camera, along the plane, or not a number
  }
  vec2 at = origin.xy + s * direction.xy;
  return at.x >= rect.x && at.x <= rect.z && at.y >= rect.y && at.y <= rect.w;
}

#endif  // VC_DEPTH_BOARD_REGION_GLSL
