// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The depth checks' host geometry and verdicts, on device results made here:
// the board region and its membership rule against the inset outline, a 4K
// colour lens's corner rays met with an exact plane (a board printed 0.3 %
// large), a plane 5 mm off and 1 degree turned, the ratios' pairs and order
// on boards of any size, and the summary's pooling and verdicts over rigs of
// 1, 3, 4 and 5 cameras; and refusals.

#include "volumetric_kit/calib/quality/depth_checks.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <glm/glm.hpp>

#include "synthetic_rig.hpp"
#include "volumetric_kit/calib/compute/board.hpp"

namespace volumetric_kit::calib::quality {
namespace {

namespace camera = recon::camera;
using camera::Mat3d;
using camera::Mat4d;
using camera::Vec2d;
using camera::Vec3d;
using Code = core::Status::Code;

constexpr double kPi = 3.14159265358979323846;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

template <class T>
T value_of(core::Result<T> result) {
  if (!result.ok()) throw std::runtime_error(result.status().message());
  return *std::move(result);
}

// `status` is an InvalidArgument starting `prefix` and saying `says`.
void expect_refused(const core::Status& status, const std::string& prefix,
                    const std::string& says) {
  EXPECT_EQ(status.domain(), Code::InvalidArgument) << says;
  EXPECT_EQ(status.message().rfind(prefix + ": ", 0), 0u) << status.message();
  EXPECT_NE(status.message().find(says), std::string::npos) << status.message();
}

Mat3d rotation(const Vec3d& axis, double deg) {
  return camera::rotation_from_rodrigues(glm::normalize(axis) * deg * kPi /
                                         180.0);
}

Vec3d apply(const Mat4d& m, const Vec3d& p) {
  return Vec3d(m * glm::dvec4(p, 1.0));
}

// A Femto Mega's depth camera beside its colour camera: 32 mm across,
// turned 6 degrees down.
Mat4d femto_depth_to_color() {
  Mat4d m(rotation({1.0, 0.02, -0.01}, -6.0));
  m[3] = glm::dvec4(0.0321, -0.0021, 0.0039, 1.0);
  return m;
}

DepthCheckOptions inset_of(double inset_m) {
  DepthCheckOptions options;
  options.inset_m = inset_m;
  return options;
}

compute::Board board_of(int rows, int cols, double square_m) {
  Target target = legacy_femto_board();
  target.rows = rows;
  target.cols = cols;
  target.square_size_m = square_m;
  target.marker_size_m = 0.75 * square_m;
  target.legacy_pattern = false;
  return value_of(compute::Board::create(target));
}

// `cameras` colour-like cameras, each with its own intrinsics.
std::vector<solve::RigCamera> rig_of(std::size_t cameras) {
  std::vector<solve::RigCamera> rig;
  for (std::size_t c = 0; c < cameras; ++c) {
    camera::CameraModel model = test::color_like_lens();
    model.intrinsics.fx += 25.0 * static_cast<double>(c);
    model.intrinsics.cx -= 9.0 * static_cast<double>(c);
    rig.push_back({"CL" + std::to_string(c), model});
  }
  return rig;
}

// `board`'s corners `ids` as `lens` sees them at `board_to_color`.
Observation view_of(const solve::RigCamera& cam, const compute::Board& board,
                    const Mat4d& board_to_color, std::vector<int> ids = {}) {
  if (ids.empty()) {
    for (std::size_t i = 0; i < board.object_points().size(); ++i) {
      ids.push_back(static_cast<int>(i));
    }
  }
  std::vector<cv::Point3d> points;
  for (const int id : ids) {
    points.push_back(board.object_points()[static_cast<std::size_t>(id)]);
  }
  const std::vector<cv::Point2d> pixels =
      test::project(cam.model, board_to_color, points);
  Observation view;
  view.camera_id = cam.id;
  view.frame_id = 7;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    view.points.push_back({ids[i], Vec2d(pixels[i].x, pixels[i].y), 1.0});
  }
  return view;
}

// The plane through `point_c` with `normal_c` (colour frame) as the device
// gives it in `region`'s board frame.
DepthPlane plane_in(const BoardRegion& region, const Vec3d& point_c,
                    const Vec3d& normal_c) {
  const Mat4d color_to_board = camera::rigid_inverse(region.board_to_color);
  DepthPlane plane;
  plane.valid = true;
  plane.point = apply(color_to_board, point_c);
  plane.normal = Mat3d(color_to_board) * normal_c;
  plane.rms_m = 0.0012;
  plane.members = 9000;
  plane.out_of_range = 120;
  plane.inliers = 8500;
  return plane;
}

// --- The board region -------------------------------------------------------

TEST(BoardRegion, IsTheChessboardInsetInTheSolvedBoardsFrame) {
  const compute::Board board = test::femto_board();
  const Mat4d board_to_color =
      test::board_facing_camera(board, {0.1, -0.05, 1.6}, 20.0, 10.0);
  const BoardRegion region = value_of(board_region(
      board, board_to_color, femto_depth_to_color(), inset_of(0.02)));
  // 5 x 3 squares of 0.15 m, 2 cm in from each edge.
  EXPECT_NEAR(region.min.x, 0.02, 1e-15);
  EXPECT_NEAR(region.min.y, 0.02, 1e-15);
  EXPECT_NEAR(region.max.x, 0.73, 1e-15);
  EXPECT_NEAR(region.max.y, 0.43, 1e-15);
  EXPECT_EQ(region.board_to_color, board_to_color);
  // depth -> board, then board -> colour, is depth -> colour.
  const Mat4d round = board_to_color * region.depth_to_board;
  const Mat4d want = femto_depth_to_color();
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      EXPECT_NEAR(round[col][row], want[col][row], 1e-12);
    }
  }
  // The default inset is 2 cm.
  const BoardRegion by_default =
      value_of(board_region(board, board_to_color, femto_depth_to_color()));
  EXPECT_EQ(by_default.min, region.min);
  EXPECT_EQ(by_default.max, region.max);
  // No inset: the chessboard's outer edge.
  const BoardRegion edge = value_of(board_region(
      board, board_to_color, femto_depth_to_color(), inset_of(0.0)));
  EXPECT_NEAR(edge.max.x, 0.75, 1e-15);
  EXPECT_EQ(edge.min, Vec2d(0.0));
}

TEST(BoardRegion, RefusesATransformNotRigidAndAnInsetOutOfRange) {
  const compute::Board board = test::femto_board();
  const Mat4d board_to_color =
      test::board_facing_camera(board, {0.0, 0.0, 1.5}, 0.0, 0.0);
  const Mat4d depth_to_color = femto_depth_to_color();
  Mat4d scaled = board_to_color;
  scaled[0] *= 1.01;
  expect_refused(board_region(board, scaled, depth_to_color).status(),
                 "board_region", "board_to_color");
  Mat4d nan = depth_to_color;
  nan[3][0] = kNaN;
  expect_refused(board_region(board, board_to_color, nan).status(),
                 "board_region", "depth_to_color");
  // The board is 0.45 m on its shorter side.
  for (const double inset :
       {-0.001, 0.225, 0.3, kNaN, std::numeric_limits<double>::infinity()}) {
    expect_refused(
        board_region(board, board_to_color, depth_to_color, inset_of(inset))
            .status(),
        "board_region", "options.inset_m");
  }
  EXPECT_TRUE(
      board_region(board, board_to_color, depth_to_color, inset_of(0.2249))
          .ok());
  // A portrait board: 0.45 m across x, 0.75 m down y.
  const compute::Board portrait = board_of(4, 2, 0.15);
  expect_refused(
      board_region(portrait, board_to_color, depth_to_color, inset_of(0.225))
          .status(),
      "board_region", "options.inset_m");
  EXPECT_TRUE(
      board_region(portrait, board_to_color, depth_to_color, inset_of(0.2249))
          .ok());
  const BoardRegion tall =
      value_of(board_region(portrait, board_to_color, depth_to_color));
  EXPECT_NEAR(tall.max.x, 0.43, 1e-15);
  EXPECT_NEAR(tall.max.y, 0.73, 1e-15);
  // The options the summary refuses, refused alike.
  DepthCheckOptions unbounded;
  unbounded.max_angle_deg = 0.0;
  expect_refused(
      board_region(board, board_to_color, depth_to_color, unbounded).status(),
      "board_region", "options.max_angle_deg");
}

// Points 1 um inside the inset rectangle's edges, and 1 cm outside them,
// for both board orientations (landscape, portrait) and tilted either way;
// a depth pixel's ray belongs alike at any scale, a pinhole pixel's ray at
// depth 1 included, so a pixel with no depth is classified as one with.
TEST(BoardRegion, MembershipIsTheInsetOutlineByRayAlone) {
  const compute::Board board = test::femto_board();
  const camera::PinholeIntrinsics k{504.2, 504.3, 321.1, 330.6};
  for (const double roll : {0.0, 90.0}) {
    for (const double tilt : {-30.0, 0.0, 35.0}) {
      SCOPED_TRACE("roll " + std::to_string(roll) + ", tilt " +
                   std::to_string(tilt));
      const Mat4d board_to_color =
          test::board_facing_camera(board, {0.05, 0.08, 1.7}, tilt, roll);
      const BoardRegion region =
          value_of(board_region(board, board_to_color, femto_depth_to_color()));
      const Mat4d board_to_depth = camera::rigid_inverse(region.depth_to_board);
      const auto belongs = [&](const Vec2d& b) {
        const Vec3d d = apply(board_to_depth, Vec3d(b, 0.0));
        // As a pixel of the prepared depth's pinhole, at depth 1.
        const Vec2d px(k.fx * d.x / d.z + k.cx, k.fy * d.y / d.z + k.cy);
        const Vec3d ray((px.x - k.cx) / k.fx, (px.y - k.cy) / k.fy, 1.0);
        const bool in = in_board_region(region, d);
        EXPECT_EQ(in_board_region(region, 0.37 * d), in);
        EXPECT_EQ(in_board_region(region, 4.2 * d), in);
        EXPECT_EQ(in_board_region(region, ray), in);
        return in;
      };
      const Vec2d lo = region.min;
      const Vec2d hi = region.max;
      constexpr double kIn = 1e-6;
      constexpr double kOut = 0.01;
      for (int i = 0; i <= 8; ++i) {
        const double fx = lo.x + (hi.x - lo.x) * i / 8.0;
        const double fy = lo.y + (hi.y - lo.y) * i / 8.0;
        const double x = std::min(std::max(fx, lo.x + kIn), hi.x - kIn);
        const double y = std::min(std::max(fy, lo.y + kIn), hi.y - kIn);
        EXPECT_TRUE(belongs({x, lo.y + kIn})) << i;
        EXPECT_TRUE(belongs({x, hi.y - kIn})) << i;
        EXPECT_TRUE(belongs({lo.x + kIn, y})) << i;
        EXPECT_TRUE(belongs({hi.x - kIn, y})) << i;
        EXPECT_FALSE(belongs({x, lo.y - kOut})) << i;
        EXPECT_FALSE(belongs({x, hi.y + kOut})) << i;
        EXPECT_FALSE(belongs({lo.x - kOut, y})) << i;
        EXPECT_FALSE(belongs({hi.x + kOut, y})) << i;
      }
      EXPECT_TRUE(belongs(0.5 * (lo + hi)));
      // Pointing away from the board, along its plane, or not a number.
      const Vec3d centre = apply(board_to_depth, Vec3d(0.5 * (lo + hi), 0.0));
      EXPECT_FALSE(in_board_region(region, -centre));
      EXPECT_FALSE(in_board_region(
          region,
          glm::transpose(Mat3d(region.depth_to_board)) * Vec3d(1.0, 0.0, 0.0)));
      EXPECT_FALSE(in_board_region(region, Vec3d(kNaN, 0.0, 1.0)));
      EXPECT_FALSE(in_board_region(region, Vec3d(0.0)));
    }
  }
}

// The rectangle's edges belong: a depth camera 1 m in front of the board,
// square to it, so each ray meets the plane exactly where it points.
TEST(BoardRegion, MembershipIncludesTheEdges) {
  BoardRegion region;
  region.depth_to_board[3] = glm::dvec4(0.0, 0.0, -1.0, 1.0);
  region.min = Vec2d(0.25, 0.125);
  region.max = Vec2d(0.5, 0.375);
  for (const double x : {0.25, 0.375, 0.5}) {
    for (const double y : {0.125, 0.25, 0.375}) {
      EXPECT_TRUE(in_board_region(region, Vec3d(x, y, 1.0))) << x << ", " << y;
    }
  }
  EXPECT_FALSE(
      in_board_region(region, Vec3d(std::nextafter(0.25, 0.0), 0.25, 1.0)));
  EXPECT_FALSE(
      in_board_region(region, Vec3d(0.375, std::nextafter(0.375, 1.0), 1.0)));
}

// --- One view's board against its depth -------------------------------------

// A board printed 0.3 % large, seen through a 4K colour lens: its corners'
// rays met with the exact plane give 1.003 for each of the 10 adjacent
// pairs, however far off the solved board (the region's frame) is.
TEST(BoardDepthCheck, FindsABoardPrintedLargeOnTheExactPlane) {
  const compute::Board nominal = test::femto_board();
  const compute::Board printed = board_of(2, 4, 0.15 * 1.003);
  const std::vector<solve::RigCamera> rig = rig_of(3);
  const Mat4d truth =
      test::board_facing_camera(printed, {0.15, -0.1, 1.8}, 30.0, 10.0);
  const Observation view = view_of(rig[2], printed, truth);
  const Vec3d point_c = compute::centre_in(printed, truth);
  const Vec3d normal_c = Mat3d(truth) * Vec3d(0.0, 0.0, 1.0);
  // The solve, with the nominal square, puts the board 0.3 % nearer; and a
  // pose 5 mm and 1 degree further off.
  Mat4d solved = truth;
  solved[3] = glm::dvec4(Vec3d(truth[3]) / 1.003, 1.0);
  Mat4d off = solved;
  off = Mat4d(rotation({0.3, 1.0, 0.2}, 1.0)) * off;
  off[3] += glm::dvec4(0.004, -0.003, 0.002, 0.0);
  std::vector<double> first;
  for (const Mat4d& pose : {solved, off}) {
    const BoardRegion region =
        value_of(board_region(nominal, pose, femto_depth_to_color()));
    const BoardDepthCheck check = value_of(board_depth_check(
        rig, nominal, view, region, plane_in(region, point_c, normal_c)));
    EXPECT_EQ(check.camera, 2u);
    EXPECT_EQ(check.capture, 7);
    ASSERT_EQ(check.ratios.size(), 10u);
    double sum = 0.0;
    for (const double r : check.ratios) {
      EXPECT_NEAR(r, 1.003, 1e-6);
      sum += r;
    }
    EXPECT_NEAR(sum / 10.0, 1.003, 0.0005);  // the plan's bound
    if (first.empty()) {
      first = check.ratios;
    } else {
      for (std::size_t i = 0; i < first.size(); ++i) {
        EXPECT_NEAR(check.ratios[i], first[i], 1e-9);
      }
    }
  }
}

// A plane 5 mm into the board and turned 1 degree about an axis off the
// centre, its point given away from the centre, either sign of normal: the
// offset at the centre and the angle, exactly (the plan asks 0.5 mm and
// 0.05 degrees); nearer the camera is negative.
TEST(BoardDepthCheck, FindsAPlaneFiveMillimetresOffAndOneDegreeTurned) {
  const compute::Board board = test::femto_board();
  const std::vector<solve::RigCamera> rig = rig_of(1);
  const Mat4d pose =
      test::board_facing_camera(board, {-0.1, 0.05, 2.1}, -25.0, 5.0);
  const BoardRegion region =
      value_of(board_region(board, pose, femto_depth_to_color()));
  const Observation view = view_of(rig[0], board, pose);
  const cv::Point3d c = board.centre();
  const Vec3d centre(c.x, c.y, c.z);
  for (const double offset_mm : {5.0, -5.0}) {
    for (const double sign : {1.0, -1.0}) {
      const Mat3d turn = rotation({0.6, -0.8, 0.0}, 1.0);
      DepthPlane plane;
      plane.valid = true;
      plane.normal = sign * 3.0 * (turn * Vec3d(0.0, 0.0, 1.0));
      plane.point = centre + Vec3d(0.0, 0.0, offset_mm / 1000.0) +
                    turn * Vec3d(0.21, -0.13, 0.0);
      const BoardDepthCheck check =
          value_of(board_depth_check(rig, board, view, region, plane));
      EXPECT_NEAR(check.offset_mm, offset_mm, 1e-9);
      EXPECT_NEAR(check.angle_deg, 1.0, 1e-9);
      EXPECT_NEAR(check.offset_mm, offset_mm, 0.5);
      EXPECT_NEAR(check.angle_deg, 1.0, 0.05);
      EXPECT_EQ(check.ratios.size(), 10u);
    }
  }
  // A normal of any finite length but 0, whose length would overflow or
  // underflow: the same.
  for (const double scale : {1e-170, 1e200}) {
    SCOPED_TRACE(scale);
    const Mat3d turn = rotation({0.6, -0.8, 0.0}, 1.0);
    DepthPlane plane;
    plane.valid = true;
    plane.normal = scale * (turn * Vec3d(0.0, 0.0, 1.0));
    plane.point = centre + Vec3d(0.0, 0.0, 0.005);
    const BoardDepthCheck check =
        value_of(board_depth_check(rig, board, view, region, plane));
    EXPECT_NEAR(check.offset_mm, 5.0, 1e-9);
    EXPECT_NEAR(check.angle_deg, 1.0, 1e-9);
    EXPECT_EQ(check.ratios.size(), 10u);
  }
  // Perpendicular to the board: no offset, 90 degrees; its point at the
  // centre or off it.
  DepthPlane across;
  across.valid = true;
  across.normal = Vec3d(1.0, 0.0, 0.0);
  for (const Vec3d& at : {centre, centre + Vec3d(0.1, 0.0, 0.0)}) {
    across.point = at;
    const BoardDepthCheck check =
        value_of(board_depth_check(rig, board, view, region, across));
    EXPECT_TRUE(std::isnan(check.offset_mm)) << at.x;
    EXPECT_NEAR(check.angle_deg, 90.0, 1e-12);
  }
}

// Each two adjacent corners both detected, in row-major order of the first,
// right before below, never across a row's end; on a 6 x 4 board, against
// meetings computed here from the corners' true positions on a plane that is
// not the board's, so every ratio differs.
TEST(BoardDepthCheck, MeasuresEachAdjacentPairInOrderOnAnyBoard) {
  const compute::Board board = board_of(4, 6, 0.08);
  const std::vector<solve::RigCamera> rig = rig_of(4);
  const Mat4d pose =
      test::board_facing_camera(board, {0.0, 0.1, 1.4}, 20.0, -15.0);
  const BoardRegion region =
      value_of(board_region(board, pose, femto_depth_to_color()));
  const Vec3d point_c = compute::centre_in(board, pose);
  const Vec3d normal_c =
      rotation({1.0, 1.0, 0.0}, 12.0) * (Mat3d(pose) * Vec3d(0.0, 0.0, 1.0));
  const DepthPlane plane = plane_in(region, point_c, normal_c);
  const auto meeting = [&](int id) {
    const cv::Point3d p = board.object_points()[static_cast<std::size_t>(id)];
    const Vec3d x = apply(pose, Vec3d(p.x, p.y, p.z));
    return x * (glm::dot(normal_c, point_c) / glm::dot(normal_c, x));
  };
  const auto expected = [&](const std::vector<std::pair<int, int>>& pairs) {
    std::vector<double> ratios;
    for (const auto& [a, b] : pairs) {
      ratios.push_back(glm::length(meeting(b) - meeting(a)) / 0.08);
    }
    return ratios;
  };
  const auto ratios_of = [&](const std::vector<int>& ids) {
    return value_of(board_depth_check(rig, board,
                                      view_of(rig[3], board, pose, ids), region,
                                      plane))
        .ratios;
  };
  const auto expect_ratios = [](const std::vector<double>& got,
                                const std::vector<double>& want) {
    ASSERT_EQ(got.size(), want.size());
    for (std::size_t i = 0; i < want.size(); ++i) {
      EXPECT_NEAR(got[i], want[i], 1e-7) << i;
    }
  };
  // Every corner: 4 rows of 5, then 6 columns of 3.
  std::vector<std::pair<int, int>> every;
  for (int a = 0; a < 24; ++a) {
    if (a % 6 != 5) every.emplace_back(a, a + 1);
    if (a + 6 < 24) every.emplace_back(a, a + 6);
  }
  ASSERT_EQ(every.size(), 38u);
  expect_ratios(ratios_of({}), expected(every));
  // Some missing, listed out of order; 5-6 and 11-12 cross a row's end.
  expect_ratios(
      ratios_of({23, 5, 13, 4, 12, 17, 6, 11}),
      expected({{4, 5}, {5, 11}, {6, 12}, {11, 17}, {12, 13}, {17, 23}}));
  // A row's last corner and the next row's first are not adjacent.
  EXPECT_TRUE(ratios_of({5, 6}).empty());
  EXPECT_TRUE(ratios_of({0}).empty());
}

// A plane behind the colour camera, or through its centre, meets no corner's
// ray in front of it; an invalid plane is measured not at all, whatever it
// holds, but kept.
TEST(BoardDepthCheck, MeasuresNothingThatMeetsNoPlane) {
  const compute::Board board = test::femto_board();
  const std::vector<solve::RigCamera> rig = rig_of(2);
  // Square to the camera, so the board frame is the colour frame moved, and
  // the colour camera's centre comes back to exactly 0.
  const Mat4d pose =
      test::board_facing_camera(board, {0.0, 0.0, 1.5}, 0.0, 0.0);
  ASSERT_EQ(Mat3d(pose), Mat3d(1.0));
  const BoardRegion region =
      value_of(board_region(board, pose, femto_depth_to_color()));
  const Observation view = view_of(rig[1], board, pose);
  for (const double z : {0.0, -1.5}) {
    DepthPlane plane = plane_in(region, Vec3d(0.0), Vec3d(0.0, 0.3, 1.0));
    plane.point = Vec3d(0.0, 0.0, z) - Vec3d(pose[3]);
    const BoardDepthCheck none =
        value_of(board_depth_check(rig, board, view, region, plane));
    EXPECT_TRUE(none.ratios.empty()) << z;
    EXPECT_FALSE(std::isnan(none.angle_deg));
  }

  DepthPlane invalid =
      plane_in(region, Vec3d(0.0, 0.0, 1.5), Vec3d(0.0, 0.0, 1.0));
  invalid.valid = false;
  invalid.members = 40;
  invalid.out_of_range = 31;
  invalid.inliers = 9;
  const BoardDepthCheck skipped =
      value_of(board_depth_check(rig, board, view, region, invalid));
  EXPECT_TRUE(skipped.ratios.empty());
  EXPECT_TRUE(std::isnan(skipped.offset_mm));
  EXPECT_TRUE(std::isnan(skipped.angle_deg));
  EXPECT_FALSE(skipped.plane.valid);
  EXPECT_EQ(skipped.plane.members, 40u);
  EXPECT_EQ(skipped.plane.out_of_range, 31u);
  EXPECT_EQ(skipped.camera, 1u);

  // No corners: the plane against the board still.
  const Mat4d tilted =
      test::board_facing_camera(board, {0.0, 0.0, 1.5}, 10.0, 0.0);
  const BoardRegion region_t =
      value_of(board_region(board, tilted, femto_depth_to_color()));
  Observation empty = view;
  empty.points.clear();
  const BoardDepthCheck blank = value_of(board_depth_check(
      rig, board, empty, region_t,
      plane_in(region_t, Vec3d(0.0, 0.0, 1.5), Vec3d(0.0, 0.0, 1.0))));
  EXPECT_TRUE(blank.ratios.empty());
  EXPECT_NEAR(blank.angle_deg, 10.0, 1e-9);
}

TEST(BoardDepthCheck, RefusesWhatDoesNotFit) {
  const compute::Board board = test::femto_board();
  const std::vector<solve::RigCamera> rig = rig_of(2);
  const Mat4d pose =
      test::board_facing_camera(board, {0.0, 0.0, 1.5}, 0.0, 0.0);
  const BoardRegion region =
      value_of(board_region(board, pose, femto_depth_to_color()));
  const Observation view = view_of(rig[0], board, pose);
  const DepthPlane plane =
      plane_in(region, Vec3d(0.0, 0.0, 1.5), Vec3d(0.0, 0.0, 1.0));
  const auto refused = [&](const std::vector<solve::RigCamera>& cameras,
                           const Observation& v, const BoardRegion& r,
                           const DepthPlane& p, const std::string& says) {
    core::Result<BoardDepthCheck> check = BoardDepthCheck{};
    EXPECT_NO_THROW(check = board_depth_check(cameras, board, v, r, p));
    expect_refused(check.status(), "board_depth_check", says);
  };
  refused({}, view, region, plane, "no cameras");
  Observation stranger = view;
  stranger.camera_id = "nobody";
  refused(rig, stranger, region, plane, "nobody");
  std::vector<solve::RigCamera> bad_lens = rig;
  bad_lens[1].model.intrinsics.fx = 0.0;
  refused(bad_lens, view, region, plane, "camera CL1");
  for (const int id : {-1, 8}) {
    Observation v = view;
    v.points[3].id = id;
    refused(rig, v, region, plane, "corner " + std::to_string(id));
  }
  Observation twice = view;
  twice.points[3].id = twice.points[2].id;
  refused(rig, twice, region, plane, "detected twice");
  BoardRegion r = region;
  r.board_to_color[0] *= 2.0;
  refused(rig, view, r, plane, "board_to_color");
  r = region;
  r.depth_to_board[3][2] = kNaN;
  refused(rig, view, r, plane, "depth_to_board");
  for (const auto& [lo, hi] :
       {std::pair{Vec2d(0.5, 0.02), Vec2d(0.4, 0.43)},
        std::pair{Vec2d(0.02, 0.02), Vec2d(0.73, 0.02)},
        std::pair{Vec2d(kNaN, 0.02), Vec2d(0.73, 0.43)},
        std::pair{Vec2d(-std::numeric_limits<double>::infinity(), 0.0),
                  Vec2d(0.73, 0.43)}}) {
    r = region;
    r.min = lo;
    r.max = hi;
    refused(rig, view, r, plane, "rectangle");
  }
  DepthPlane p = plane;
  p.inliers = p.members + 1;
  refused(rig, view, region, p, "exceed its members");
  p = plane;
  p.out_of_range = p.members - p.inliers + 1;
  refused(rig, view, region, p, "exceed its members");
  p = plane;
  p.point.y = kNaN;
  refused(rig, view, region, p, "finite point");
  p = plane;
  p.normal = Vec3d(0.0);
  refused(rig, view, region, p, "nonzero normal");
  p = plane;
  p.normal.x = std::numeric_limits<double>::infinity();
  refused(rig, view, region, p, "nonzero normal");
  // Not valid: what the point and normal hold does not matter.
  p.valid = false;
  EXPECT_TRUE(board_depth_check(rig, board, view, region, p).ok());
}

// --- The summary ------------------------------------------------------------

BoardDepthCheck check_of(std::size_t camera, std::int64_t capture,
                         std::vector<double> ratios, double offset_mm,
                         double angle_deg, bool valid = true) {
  BoardDepthCheck check;
  check.camera = camera;
  check.capture = capture;
  check.plane.valid = valid;
  check.ratios = std::move(ratios);
  check.offset_mm = offset_mm;
  check.angle_deg = angle_deg;
  return check;
}

PairAgreement pair_of(std::size_t from, std::size_t to,
                      std::optional<std::int64_t> capture, std::size_t overlap,
                      std::size_t in_window, double signed_median_mm,
                      double median_abs_mm) {
  PairAgreement pair;
  pair.from = from;
  pair.to = to;
  pair.capture = capture;
  pair.overlap = overlap;
  pair.in_window = in_window;
  pair.signed_median_mm = signed_median_mm;
  pair.median_abs_mm = median_abs_mm;
  pair.p90_abs_mm = 3.0 * median_abs_mm;
  return pair;
}

// The mean and sample standard deviation, computed here.
void expect_stats(const RatioStats& got, const std::vector<double>& values) {
  ASSERT_EQ(got.count, values.size());
  double sum = 0.0;
  for (const double v : values) sum += v;
  const double mean = sum / static_cast<double>(values.size());
  double squares = 0.0;
  for (const double v : values) squares += (v - mean) * (v - mean);
  EXPECT_NEAR(got.mean, mean, 1e-15);
  EXPECT_NEAR(got.sd,
              std::sqrt(squares / static_cast<double>(values.size() - 1)),
              1e-15);
}

// Each camera's ratios pooled over its captures, and every camera's pooled
// for the board; a view with no valid plane counts for nothing, even holding
// ratios; the views listed by capture, then camera.
TEST(SummarizeDepthChecks, PoolsTheRatiosOverCapturesAndCameras) {
  const std::vector<BoardDepthCheck> views = {
      check_of(1, 4, {1.004, 1.002, 1.003}, 1.0, 0.2),
      check_of(0, 4, {1.001, 1.0035}, -2.0, 0.4),
      check_of(0, 2, {1.002, 1.0025, 1.0045, 1.0001}, 3.0, 0.1),
      check_of(2, 2, {9.0, 9.0}, 0.0, 0.0, false),
      check_of(1, 2, {1.0031}, 4.0, 0.3),
  };
  const DepthSummary s = value_of(summarize_depth_checks(3, views, {}));
  ASSERT_EQ(s.views.size(), 5u);
  const std::vector<std::pair<std::int64_t, std::size_t>> order = {
      {2, 0}, {2, 1}, {2, 2}, {4, 0}, {4, 1}};
  for (std::size_t v = 0; v < order.size(); ++v) {
    EXPECT_EQ(s.views[v].capture, order[v].first) << v;
    EXPECT_EQ(s.views[v].camera, order[v].second) << v;
  }
  ASSERT_EQ(s.cameras.size(), 3u);
  expect_stats(s.cameras[0].ratio,
               {1.002, 1.0025, 1.0045, 1.0001, 1.001, 1.0035});
  expect_stats(s.cameras[1].ratio, {1.0031, 1.004, 1.002, 1.003});
  EXPECT_EQ(s.cameras[0].views, 2u);
  EXPECT_EQ(s.cameras[1].views, 2u);
  EXPECT_EQ(s.cameras[2].views, 0u);
  EXPECT_EQ(s.cameras[2].ratio.count, 0u);
  EXPECT_TRUE(std::isnan(s.cameras[2].ratio.mean));
  EXPECT_TRUE(std::isnan(s.cameras[2].offset_mm));
  expect_stats(s.ratio, {1.002, 1.0025, 1.0045, 1.0001, 1.0031, 1.001, 1.0035,
                         1.004, 1.002, 1.003});
  // Medians of an even count: the mean of the middle two.
  EXPECT_DOUBLE_EQ(s.cameras[0].offset_mm, 0.5);
  EXPECT_DOUBLE_EQ(s.cameras[0].angle_deg, 0.25);
  EXPECT_EQ(s.options.inset_m, kDepthInsetM);
  EXPECT_EQ(s.options.min_overlap, kMinOverlap);
  // One ratio: a mean, no spread.
  const DepthSummary one =
      value_of(summarize_depth_checks(1, {check_of(0, 1, {1.002}, 0, 0)}, {}));
  EXPECT_DOUBLE_EQ(one.ratio.mean, 1.002);
  EXPECT_TRUE(std::isnan(one.ratio.sd));
}

TEST(SummarizeDepthChecks, JudgesEachCamerasScaleAndPlane) {
  DepthCheckOptions options;
  options.max_scale_error = 0.002;
  options.max_offset_mm = 10.0;
  options.max_angle_deg = 1.0;
  const std::vector<BoardDepthCheck> views = {
      // 0: scale 1.0015, within; median offset -9, angle 0.9: ok.
      check_of(0, 1, {1.001, 1.002}, -9.0, 0.9),
      check_of(0, 2, {1.0015}, -12.0, 0.2),
      check_of(0, 3, {}, 3.0, 1.5),
      // 1: scale 1.0025, beyond; median offset 11: off.
      check_of(1, 1, {1.0025}, 11.0, 0.1),
      // 2: scale 0.9975, beyond below; median angle 1.2: off.
      check_of(2, 1, {0.9975}, 0.0, 1.2),
      // 3: a valid plane but no ratios (its rays met nothing).
      check_of(3, 1, {}, 2.0, 0.5),
      // 4: an offset not a number (the plane across the board).
      check_of(4, 1, {1.0}, kNaN, 90.0),
      // 5: no valid view.
      check_of(5, 1, {1.0}, 0.0, 0.0, false),
  };
  const DepthSummary s =
      value_of(summarize_depth_checks(6, views, {}, options));
  const std::vector<std::pair<DepthVerdict, DepthVerdict>> want = {
      {DepthVerdict::Ok, DepthVerdict::Ok},
      {DepthVerdict::Off, DepthVerdict::Off},
      {DepthVerdict::Off, DepthVerdict::Off},
      {DepthVerdict::Insufficient, DepthVerdict::Ok},
      {DepthVerdict::Ok, DepthVerdict::Off},
      {DepthVerdict::Insufficient, DepthVerdict::Insufficient},
  };
  for (std::size_t c = 0; c < want.size(); ++c) {
    EXPECT_EQ(s.cameras[c].scale, want[c].first) << c;
    EXPECT_EQ(s.cameras[c].plane, want[c].second) << c;
  }
  EXPECT_DOUBLE_EQ(s.cameras[0].offset_mm, -9.0);
  EXPECT_DOUBLE_EQ(s.cameras[0].angle_deg, 0.9);
  EXPECT_TRUE(std::isnan(s.cameras[4].offset_mm));
  // The board's: every valid view's ratio, 1.00075 on average.
  EXPECT_EQ(s.scale, DepthVerdict::Ok);
  options.max_scale_error = 0.0004;
  EXPECT_EQ(value_of(summarize_depth_checks(6, views, {}, options)).scale,
            DepthVerdict::Off);
  EXPECT_EQ(value_of(summarize_depth_checks(6, {}, {})).scale,
            DepthVerdict::Insufficient);

  // Exactly at each bound: ok (values a double holds exactly).
  options.max_scale_error = 0.25;
  const DepthSummary edge = value_of(summarize_depth_checks(
      1,
      {check_of(0, 1, {1.25}, -12.0, 1.5), check_of(0, 2, {}, -10.0, 1.0),
       check_of(0, 3, {}, 3.0, 0.2)},
      {}, options));
  EXPECT_EQ(edge.cameras[0].offset_mm, -10.0);
  EXPECT_EQ(edge.cameras[0].angle_deg, 1.0);
  EXPECT_EQ(edge.cameras[0].ratio.mean, 1.25);
  EXPECT_EQ(edge.cameras[0].scale, DepthVerdict::Ok);
  EXPECT_EQ(edge.cameras[0].plane, DepthVerdict::Ok);
  EXPECT_EQ(edge.scale, DepthVerdict::Ok);

  // A view across the board has no offset, which its camera's median leaves
  // out; its angle counts.
  const DepthSummary across = value_of(summarize_depth_checks(
      1,
      {check_of(0, 1, {}, 1.0, 0.1), check_of(0, 2, {}, kNaN, 90.0),
       check_of(0, 3, {}, 2.0, 0.2)},
      {}, options));
  EXPECT_EQ(across.cameras[0].views, 3u);
  EXPECT_DOUBLE_EQ(across.cameras[0].offset_mm, 1.5);
  EXPECT_DOUBLE_EQ(across.cameras[0].angle_deg, 0.2);
  EXPECT_EQ(across.cameras[0].plane, DepthVerdict::Ok);
}

// Every ordered pair, by from then to, for rigs of 1, 3, 4 and 5 cameras:
// pooled always, and each capture that has a pair measured on its own; a
// pair not given has no samples.
TEST(SummarizeDepthChecks, ListsEveryOrderedPairOfAnyRig) {
  for (const std::size_t k : {1u, 3u, 4u, 5u}) {
    SCOPED_TRACE(k);
    std::vector<PairAgreement> pairs;
    if (k > 1) {
      pairs.push_back(pair_of(k - 1, 0, std::nullopt, 5000, 4900, 0.5, 1.5));
      pairs.push_back(pair_of(0, 1, 12, 3000, 2900, -0.2, 1.1));
      pairs.push_back(pair_of(1, 0, 3, 3000, 2900, 0.2, 1.1));
    }
    const DepthSummary s = value_of(summarize_depth_checks(k, {}, pairs));
    ASSERT_EQ(s.pairs.size(), k * (k - 1));
    std::size_t i = 0;
    for (std::size_t from = 0; from < k; ++from) {
      for (std::size_t to = 0; to < k; ++to) {
        if (from == to) continue;
        const PairCheck& p = s.pairs[i++];
        EXPECT_EQ(p.agreement.from, from);
        EXPECT_EQ(p.agreement.to, to);
        EXPECT_FALSE(p.agreement.capture.has_value());
        if (from == k - 1 && to == 0) {
          EXPECT_EQ(p.agreement.overlap, 5000u);
          EXPECT_EQ(p.agreement.p90_abs_mm, 4.5);
          EXPECT_EQ(p.verdict, DepthVerdict::Ok);
        } else {
          EXPECT_EQ(p.agreement.overlap, 0u);
          EXPECT_TRUE(std::isnan(p.agreement.signed_median_mm));
          EXPECT_EQ(p.verdict, DepthVerdict::Insufficient);
        }
      }
    }
    EXPECT_EQ(s.cameras.size(), k);
    if (k == 1) {
      EXPECT_TRUE(s.captures.empty());
      continue;
    }
    ASSERT_EQ(s.captures.size(), 2u);
    EXPECT_EQ(s.captures[0].capture, 3);
    EXPECT_EQ(s.captures[1].capture, 12);
    for (const CapturePairs& c : s.captures) {
      ASSERT_EQ(c.pairs.size(), k * (k - 1));
      for (const PairCheck& p : c.pairs) {
        EXPECT_EQ(p.agreement.capture, c.capture);
        const bool measured =
            c.capture == 3 ? p.agreement.from == 1 && p.agreement.to == 0
                           : p.agreement.from == 0 && p.agreement.to == 1;
        EXPECT_EQ(p.agreement.overlap, measured ? 3000u : 0u);
        EXPECT_EQ(p.verdict,
                  measured ? DepthVerdict::Ok : DepthVerdict::Insufficient);
      }
    }
  }
}

TEST(SummarizeDepthChecks, JudgesEachPairOnItsMediansAndOverlap) {
  DepthCheckOptions options;
  options.min_overlap = 1000;
  options.max_signed_median_mm = 5.0;
  options.max_median_abs_mm = 10.0;
  const std::vector<std::pair<PairAgreement, DepthVerdict>> cases = {
      {pair_of(0, 1, {}, 999, 999, 0.0, 1.0), DepthVerdict::Insufficient},
      {pair_of(0, 1, {}, 1000, 1000, 0.0, 1.0), DepthVerdict::Ok},
      {pair_of(0, 1, {}, 1000, 1000, -5.0, 10.0), DepthVerdict::Ok},
      {pair_of(0, 1, {}, 1000, 1000, 5.01, 6.0), DepthVerdict::Off},
      {pair_of(0, 1, {}, 1000, 1000, -5.01, 6.0), DepthVerdict::Off},
      {pair_of(0, 1, {}, 1000, 1000, 0.0, 10.01), DepthVerdict::Off},
      // Half in the window: judged on the medians; fewer: off.
      {pair_of(0, 1, {}, 2000, 1000, 0.0, 1.0), DepthVerdict::Ok},
      {pair_of(0, 1, {}, 2001, 1000, 0.0, 1.0), DepthVerdict::Off},
      {pair_of(0, 1, {}, 5000, 0, kNaN, kNaN), DepthVerdict::Off},
      {pair_of(0, 1, {}, 5000, 5000, kNaN, 1.0), DepthVerdict::Off},
  };
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const DepthSummary s =
        value_of(summarize_depth_checks(2, {}, {cases[i].first}, options));
    EXPECT_EQ(s.pairs[0].verdict, cases[i].second) << i;
  }
  options.min_overlap = 2;
  EXPECT_EQ(value_of(summarize_depth_checks(
                         2, {}, {pair_of(0, 1, {}, 2, 2, 0.0, 1.0)}, options))
                .pairs[0]
                .verdict,
            DepthVerdict::Ok);
}

TEST(SummarizeDepthChecks, RefusesWhatDoesNotFit) {
  const auto refused =
      [](std::size_t k, const std::vector<BoardDepthCheck>& views,
         const std::vector<PairAgreement>& pairs,
         const DepthCheckOptions& options, const std::string& says) {
        core::Result<DepthSummary> s = DepthSummary{};
        EXPECT_NO_THROW(s = summarize_depth_checks(k, views, pairs, options));
        expect_refused(s.status(), "summarize_depth_checks", says);
      };
  const DepthCheckOptions fine;
  refused(0, {}, {}, fine, "no cameras");
  refused(2, {check_of(2, 1, {}, 0, 0)}, {}, fine, "camera 2");
  refused(2, {check_of(1, 1, {}, 0, 0), check_of(1, 1, {}, 0, 0, false)}, {},
          fine, "two views of capture 1");
  refused(3, {}, {pair_of(1, 1, {}, 0, 0, 0, 0)}, fine,
          "from camera 1 to camera 1");
  refused(3, {}, {pair_of(0, 3, 5, 0, 0, 0, 0)}, fine,
          "from camera 0 to camera 3 in capture 5");
  refused(3, {}, {pair_of(0, 1, {}, 10, 11, 0, 0)}, fine,
          "more samples in the window");
  refused(3, {}, {pair_of(0, 1, 4, 10, 1, 0, 0), pair_of(0, 1, 4, 20, 2, 0, 0)},
          fine, "appears twice");
  // The same pair pooled and in a capture is two pairs.
  EXPECT_TRUE(summarize_depth_checks(3, {},
                                     {pair_of(0, 1, {}, 10, 1, 0, 0),
                                      pair_of(0, 1, 4, 20, 2, 0, 0)})
                  .ok());
  const auto option = [&](auto set, const std::string& name) {
    DepthCheckOptions o;
    set(o);
    refused(1, {}, {}, o, "options." + name);
  };
  for (const double bad :
       {0.0, -1.0, kNaN, std::numeric_limits<double>::infinity()}) {
    option([&](DepthCheckOptions& o) { o.max_scale_error = bad; },
           "max_scale_error");
    option([&](DepthCheckOptions& o) { o.max_offset_mm = bad; },
           "max_offset_mm");
    option([&](DepthCheckOptions& o) { o.max_angle_deg = bad; },
           "max_angle_deg");
    option([&](DepthCheckOptions& o) { o.max_signed_median_mm = bad; },
           "max_signed_median_mm");
    option([&](DepthCheckOptions& o) { o.max_median_abs_mm = bad; },
           "max_median_abs_mm");
  }
  option([](DepthCheckOptions& o) { o.inset_m = -0.01; }, "inset_m");
  option([](DepthCheckOptions& o) { o.inset_m = kNaN; }, "inset_m");
  option(
      [](DepthCheckOptions& o) {
        o.inset_m = std::numeric_limits<double>::infinity();
      },
      "inset_m");
  option([](DepthCheckOptions& o) { o.min_overlap = 0; }, "min_overlap");
  DepthCheckOptions zero_inset;
  zero_inset.inset_m = 0.0;
  EXPECT_TRUE(summarize_depth_checks(1, {}, {}, zero_inset).ok());
}

TEST(DepthVerdict, NamesEachVerdict) {
  EXPECT_STREQ(to_string(DepthVerdict::Ok), "ok");
  EXPECT_STREQ(to_string(DepthVerdict::Off), "off");
  EXPECT_STREQ(to_string(DepthVerdict::Insufficient), "insufficient");
  EXPECT_STREQ(to_string(static_cast<DepthVerdict>(9)), "unknown");
}

}  // namespace
}  // namespace volumetric_kit::calib::quality
