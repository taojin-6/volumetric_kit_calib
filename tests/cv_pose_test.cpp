// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// sensor's rigid transforms in OpenCV's types: glm's column-major Mat4d against
// cv::Matx's row-major layout, and rvec/tvec through sensor's geometry.

#include "volumetric_kit/calib/compute/cv_pose.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/geometry.hpp>

namespace volumetric_kit::calib::compute {
namespace {

namespace camera = sensor::camera;

constexpr double kPi = 3.14159265358979323846;

// A rotation about an axis off every coordinate axis, so its matrix is not
// symmetric, and a translation with three different components.
CvPose asymmetric_pose() {
  return {cv::normalize(cv::Vec3d(0.3, -0.5, 0.8)) * 1.1,
          cv::Vec3d(0.25, -1.5, 3.75)};
}

TEST(CvPose, MatrixLayoutIsTransposedElementForElement) {
  camera::Mat4d m(0.0);
  for (int column = 0; column < 4; ++column) {
    for (int row = 0; row < 4; ++row) m[column][row] = 10.0 * row + column;
  }
  const cv::Matx44d cv_m = cv_matrix(m);
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      EXPECT_EQ(cv_m(row, column), 10.0 * row + column) << row << column;
    }
  }
  EXPECT_EQ(matrix_from_cv(cv_m), m);
}

// OpenCV's extrinsic [R | t] from cv::Rodrigues sits in cv_matrix's rows and
// sensor's columns: the translation in OpenCV's last column, sensor's m[3].
TEST(CvPose, MatrixOfAnAsymmetricTransformIsOpenCVs) {
  const CvPose pose = asymmetric_pose();
  cv::Matx33d rotation;
  cv::Rodrigues(pose.rvec, rotation);

  const camera::Mat4d m = matrix_from_cv_pose(pose);
  const cv::Matx44d cv_m = cv_matrix(m);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      EXPECT_NEAR(cv_m(row, column), rotation(row, column), 1e-15);
      EXPECT_NEAR(m[column][row], rotation(row, column), 1e-15);
    }
    EXPECT_EQ(cv_m(row, 3), pose.tvec[row]);
    EXPECT_EQ(m[3][row], pose.tvec[row]);
    EXPECT_EQ(cv_m(3, row), 0.0);
  }
  EXPECT_EQ(cv_m(3, 3), 1.0);
  ASSERT_NE(rotation(0, 1), rotation(1, 0));  // the check above can tell

  // A point moves the same way in both.
  const cv::Vec3d x(0.7, -0.2, 1.9);
  const cv::Vec3d expected = rotation * x + pose.tvec;
  const camera::Vec3d moved(m * glm::dvec4(x[0], x[1], x[2], 1.0));
  for (int i = 0; i < 3; ++i) EXPECT_NEAR(moved[i], expected[i], 1e-15);
}

TEST(CvPose, RoundTripsThroughTheMatrix) {
  const std::vector<CvPose> poses = {
      asymmetric_pose(),
      {cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(1.0, 2.0, 3.0)},
      {cv::Vec3d(0.0, 2.5, 0.0), cv::Vec3d(-0.1, 0.0, 0.2)},
      {cv::normalize(cv::Vec3d(1.0, 1.0, -2.0)) * (kPi - 1e-3),
       cv::Vec3d(0.0, 0.0, 0.0)}};
  for (const CvPose& pose : poses) {
    core::Result<CvPose> back = core::Status::invalid_argument("unset");
    EXPECT_NO_THROW(back = cv_pose(matrix_from_cv_pose(pose)));
    ASSERT_TRUE(back.ok()) << back.status().message();
    EXPECT_LE(cv::norm(back->rvec - pose.rvec), 1e-12) << back->rvec;
    EXPECT_LE(cv::norm(back->tvec - pose.tvec), 1e-15) << back->tvec;
  }
}

// Through sensor's rodrigues_from_matrix, rotations round-trip where
// cv::Rodrigues returns exactly zero (below about 1e-5 rad) and toward a half
// turn.
TEST(CvPose, RoundTripsTinyRotationsAndNearlyHalfTurns) {
  const cv::Vec3d axis = cv::normalize(cv::Vec3d(0.3, -0.5, 0.8));
  for (const double angle : {1e-9, 1e-6, 1e-5, kPi - 1e-6}) {
    const CvPose pose{axis * angle, cv::Vec3d(0.1, 0.2, 0.3)};
    const core::Result<CvPose> back = cv_pose(matrix_from_cv_pose(pose));
    ASSERT_TRUE(back.ok()) << back.status().message();
    // Relative to the angle: 1e-9 of it.
    EXPECT_LE(cv::norm(back->rvec - pose.rvec), 1e-9 * angle)
        << angle << " rad: " << back->rvec;
  }
  cv::Vec3d by_opencv;
  cv::Rodrigues(cv::Matx33d(cv_matrix(matrix_from_cv_pose({axis * 1e-6, {}}))
                                .get_minor<3, 3>(0, 0)),
                by_opencv);
  EXPECT_EQ(by_opencv, cv::Vec3d(0.0, 0.0, 0.0));  // what cv_pose avoids
}

TEST(CvPose, RefusesATransformThatIsNotRigid) {
  const camera::Mat4d rigid = matrix_from_cv_pose(asymmetric_pose());
  camera::Mat4d scaled = rigid;
  scaled[0] *= 1.01;
  camera::Mat4d reflected = rigid;
  reflected[2] = -reflected[2];
  camera::Mat4d projective = rigid;
  projective[0][3] = 0.5;
  camera::Mat4d not_finite = rigid;
  not_finite[3][1] = std::numeric_limits<double>::quiet_NaN();
  for (const camera::Mat4d& m : {scaled, reflected, projective, not_finite}) {
    core::Result<CvPose> pose = CvPose{};
    EXPECT_NO_THROW(pose = cv_pose(m));
    EXPECT_EQ(pose.status().domain(), core::Status::Code::InvalidArgument);
    EXPECT_EQ(pose.status().message().rfind("cv_pose: ", 0), 0u)
        << pose.status().message();
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::compute
