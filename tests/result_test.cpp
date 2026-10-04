// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// calib's Status and Result are volumetric_kit_core's, named in calib's
// namespace; the core's own suite tests their behavior. This checks the names
// are the core's types -- so a calib error passes to recon or gfx unchanged --
// and that the core's macros work on calib's calls.

#include "volumetric_kit/calib/core/result.hpp"

#include <type_traits>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "volumetric_kit/calib/core/camera_model.hpp"

namespace volumetric_kit::calib {
namespace {

static_assert(std::is_same_v<Status, core::Status>,
              "calib::Status must be the core's type, not a copy");
static_assert(std::is_same_v<Result<int>, core::Result<int>>,
              "calib::Result must be the core's template, not a copy");

// Takes the core's type, as recon or gfx would.
core::Status relay(const core::Status& status) { return status; }

TEST(Result, CalibStatusIsTheCoreStatus) {
  const Status s = Status::numerical("singular normal equations");
  const core::Status relayed = relay(s);
  EXPECT_EQ(relayed.domain(), core::Status::Code::Numerical);
  EXPECT_EQ(relayed.message(), "singular normal equations");
  EXPECT_EQ(to_string(relayed.domain()), "Numerical");
}

Result<Eigen::Vector2d> project_with(double fx, const Eigen::Vector3d& p) {
  VKC_ASSIGN(const PinholeBrownConrady cam,
             PinholeBrownConrady::create({1280, 720}, {fx, fx, 640, 360}));
  return cam.project(p);
}

TEST(Result, CoreMacrosPropagateCalibFailures) {
  const Result<Eigen::Vector2d> ok = project_with(1000, {0.0, 0.0, 1.0});
  ASSERT_TRUE(ok.ok());
  EXPECT_NEAR(ok->x(), 640.0, 1e-12);

  const Result<Eigen::Vector2d> bad = project_with(-1, {0.0, 0.0, 1.0});
  EXPECT_EQ(bad.status().domain(), Status::Code::InvalidArgument);
}

}  // namespace
}  // namespace volumetric_kit::calib
