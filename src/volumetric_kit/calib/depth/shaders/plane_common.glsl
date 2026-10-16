// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The board-plane fit's blocks and its workgroup sum, shared by
// plane_moments.comp and plane_solve.comp. The blocks are std430, mirrored by
// plane_blocks.hpp on the host.

#ifndef VC_DEPTH_PLANE_COMMON_GLSL
#define VC_DEPTH_PLANE_COMMON_GLSL

#include "volumetric_kit/calib/depth/shaders/board_region.glsl"

// Both kernels' workgroup: a power of two, for the tree in vc_sum_workgroup.
#define VC_PLANE_LOCAL_SIZE 256

// One region: its depth image and its board frame, as the host writes it.
struct Region {
  mat4 depth_to_board;  // the depth camera's frame to the board's
  vec4 k;               // the depth's pinhole: fx, fy, cx, cy (pixels)
  vec4 rect;            // the inset rectangle on z = 0: min.xy, max.xy (m)
  uvec2 size;           // the depth image's width and height
  vec2 range;           // the depth measured: min and max (metres)
  uint first_partial;   // its first slot in the partials
  uint groups;          // its slots: plane_moments' workgroups over it
  uint pad0;
  uint pad1;
};

// One region's fit, as a round of plane_solve leaves it: the next round reads
// it, and the host reads it back after the last. When `valid` is 0, `point`,
// `normal`, `rms` and `gate` are 0.
struct Fit {
  vec3 point;          // the inliers' centroid (board frame, metres)
  float rms;           // their RMS distance from the plane (metres)
  vec3 normal;         // the plane's unit normal, its z not negative
  uint valid;          // whether enough inliers fitted a plane
  uint members;        // the region's pixels
  uint out_of_range;   // those with no depth, or depth out of range
  uint inliers;        // the points the round took
  float gate;          // the next round's half-width (metres)
};

// One workgroup's sums over its pixels of a region: the counts (members, out
// of range, inliers) and the inliers' moments about the round's reference
// point (x, y, z, xx, xy, xz, yy, yz, zz).
struct Partial {
  uint counts[3];
  float moments[9];
};

shared uint vc_shared_counts[3 * VC_PLANE_LOCAL_SIZE];
shared float vc_shared_moments[9 * VC_PLANE_LOCAL_SIZE];

// The workgroup's sums of every thread's counts and moments, in a fixed
// order whatever the scheduling: a tree in shared memory, each step adding
// the upper half of the threads' values to the lower half. Called by every
// thread, in uniform control flow; thread 0 is left holding the sums.
void vc_sum_workgroup(uint lid, inout uint counts[3], inout float moments[9]) {
  for (uint j = 0u; j < 3u; ++j) {
    vc_shared_counts[j * VC_PLANE_LOCAL_SIZE + lid] = counts[j];
  }
  for (uint j = 0u; j < 9u; ++j) {
    vc_shared_moments[j * VC_PLANE_LOCAL_SIZE + lid] = moments[j];
  }
  barrier();
  for (uint span = VC_PLANE_LOCAL_SIZE / 2u; span > 0u; span >>= 1u) {
    if (lid < span) {
      for (uint j = 0u; j < 3u; ++j) {
        uint at = j * VC_PLANE_LOCAL_SIZE + lid;
        vc_shared_counts[at] += vc_shared_counts[at + span];
      }
      for (uint j = 0u; j < 9u; ++j) {
        uint at = j * VC_PLANE_LOCAL_SIZE + lid;
        vc_shared_moments[at] += vc_shared_moments[at + span];
      }
    }
    barrier();
  }
  for (uint j = 0u; j < 3u; ++j) {
    counts[j] = vc_shared_counts[j * VC_PLANE_LOCAL_SIZE];
  }
  for (uint j = 0u; j < 9u; ++j) {
    moments[j] = vc_shared_moments[j * VC_PLANE_LOCAL_SIZE];
  }
}

// The point a round's moments are taken about, near its points so float sums
// keep their precision: round 0's is the inset rectangle's centre, on the
// solved board; round 1's, round 0's centroid.
vec3 vc_reference(uint round, Region region, Fit previous) {
  return round == 0u
             ? vec3(0.5 * (region.rect.xy + region.rect.zw), 0.0)
             : previous.point;
}

#endif  // VC_DEPTH_PLANE_COMMON_GLSL
