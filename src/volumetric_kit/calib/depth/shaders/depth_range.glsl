// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Whether a depth sample was measured: the rule every depth check's kernel
// takes, plane_moments.comp's and agreement.comp's alike.
//
// TODO: The rule is recon's prepared-depth contract, which recon writes
// inline in tsdf_integrate.glsl, hash_allocate_depth.comp and
// texture_common.glsl. Include it from recon once recon exports it beside
// pinhole.glsl (e.g. vr_depth_measured(z, min, max)), and delete this file.

#ifndef VC_DEPTH_RANGE_GLSL
#define VC_DEPTH_RANGE_GLSL

// Whether depth `z` (metres; 0 for none) lies in the camera's range,
// `range` (min, max), ends included. Written so a NaN fails it.
bool vc_in_range(float z, vec2 range) {
  return z > 0.0 && z >= range.x && z <= range.y;
}

#endif  // VC_DEPTH_RANGE_GLSL
