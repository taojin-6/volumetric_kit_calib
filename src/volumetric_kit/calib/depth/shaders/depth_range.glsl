// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Whether a depth sample was measured: the rule every depth check's kernel
// takes, plane_moments.comp's and agreement.comp's alike.

#ifndef VC_DEPTH_RANGE_GLSL
#define VC_DEPTH_RANGE_GLSL

// Whether depth `z` (metres; 0 for none) lies in the camera's range,
// `range` (min, max), ends included. Written so a NaN fails it.
bool vc_in_range(float z, vec2 range) {
  return z > 0.0 && z >= range.x && z <= range.y;
}

#endif  // VC_DEPTH_RANGE_GLSL
