// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Whether a depth sample was measured: the rule every depth check's kernel
// takes, plane_moments.comp's and agreement.comp's alike.
//
// TODO: The rule is the prepared-depth contract of sensor's DeviceFrame, which
// each of its readers writes inline. Include it from sensor once sensor_frame
// exports it beside pinhole.glsl (e.g. vs_depth_measured(z, min, max)), and
// delete this file.

#ifndef VC_DEPTH_RANGE_GLSL
#define VC_DEPTH_RANGE_GLSL

// Whether depth `z` (metres; 0 for none) lies in the camera's range,
// `range` (min, max), ends included. Written so a NaN fails it.
bool vc_in_range(float z, vec2 range) {
  return z > 0.0 && z >= range.x && z <= range.y;
}

#endif  // VC_DEPTH_RANGE_GLSL
