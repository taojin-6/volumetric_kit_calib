// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/version.hpp"

#include <string>

#include <gtest/gtest.h>

namespace volumetric_kit::calib {
namespace {

TEST(Version, MatchesCompiledMacros) {
  EXPECT_EQ(version_major(), VC_VERSION_MAJOR);
  EXPECT_EQ(version_minor(), VC_VERSION_MINOR);
  EXPECT_EQ(version_patch(), VC_VERSION_PATCH);
}

TEST(Version, StringIsNonEmpty) {
  EXPECT_FALSE(std::string(version_string()).empty());
  EXPECT_EQ(std::string(version_string()), std::string(VC_VERSION_STRING));
}

}  // namespace
}  // namespace volumetric_kit::calib
