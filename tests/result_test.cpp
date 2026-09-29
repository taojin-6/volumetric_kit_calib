// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/core/result.hpp"

#include <string>

#include <gtest/gtest.h>

namespace volumetric_kit::calib {
namespace {

TEST(Status, DefaultIsOk) {
  Status s;
  EXPECT_TRUE(s.ok());
  EXPECT_TRUE(static_cast<bool>(s));
  EXPECT_EQ(s.domain(), Status::Code::Ok);
}

TEST(Status, FactoriesSetDomainAndMessage) {
  Status s = Status::invalid_argument("bad fx");
  EXPECT_FALSE(s.ok());
  EXPECT_EQ(s.domain(), Status::Code::InvalidArgument);
  EXPECT_EQ(s.message(), "bad fx");

  EXPECT_EQ(Status::numerical("singular").domain(), Status::Code::Numerical);
  EXPECT_EQ(Status::not_found("file").domain(), Status::Code::NotFound);
}

TEST(Status, ToStringNamesDomains) {
  EXPECT_EQ(to_string(Status::Code::Ok), "Ok");
  EXPECT_EQ(to_string(Status::Code::InvalidArgument), "InvalidArgument");
  EXPECT_EQ(to_string(Status::Code::Numerical), "Numerical");
}

TEST(Result, HoldsValueOnSuccess) {
  Result<int> r = 42;
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value(), 42);
  EXPECT_EQ(*r, 42);
}

TEST(Result, HoldsStatusOnFailure) {
  Result<int> r = Status::numerical("did not converge");
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.status().domain(), Status::Code::Numerical);
}

// --- VC_TRY / VC_ASSIGN exercise the propagation macros ---------------------

Status fail_if_negative(int x) {
  if (x < 0) return Status::invalid_argument("negative");
  return {};
}

Status run_try(int x) {
  VC_TRY(fail_if_negative(x));
  return {};
}

Result<int> doubled_if_nonneg(int x) {
  if (x < 0) return Status::invalid_argument("negative");
  return x * 2;
}

Result<int> run_assign(int x) {
  VC_ASSIGN(int doubled, doubled_if_nonneg(x));
  return doubled + 1;
}

TEST(Macros, VcTryPropagates) {
  EXPECT_TRUE(run_try(3).ok());
  EXPECT_FALSE(run_try(-1).ok());
}

TEST(Macros, VcAssignUnwrapsOrPropagates) {
  Result<int> ok = run_assign(10);
  ASSERT_TRUE(ok.ok());
  EXPECT_EQ(ok.value(), 21);

  EXPECT_FALSE(run_assign(-1).ok());
}

}  // namespace
}  // namespace volumetric_kit::calib
