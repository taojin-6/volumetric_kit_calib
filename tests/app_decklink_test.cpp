// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The opener's DeckLink arm, where sensor builds its driver: a scenario's
// pair as the driver's options -- each side's id and input by the pair's
// names, the cameras' mode, and no calibration -- and the pairs it refuses.
// Host-only: no card is opened.

#include "volumetric_kit/calib/app/decklink_arm.hpp"

#include <string>

#include <gtest/gtest.h>

namespace volumetric_kit::calib::app {
namespace {

using Code = core::Status::Code;

// The lab's pair as core's config resolves it: bm-1 on SDI 2 left, bm-2 on
// SDI 1 right, at 2160p25.
core::config::Source pair_source() {
  core::config::Source source;
  source.driver = "decklink";
  source.cameras = {{"bm-1", "SDI 2"}, {"bm-2", "SDI 1"}};
  source.pairs = {{"bm-1", "bm-2"}};
  source.color_width = 3840;
  source.color_height = 2160;
  source.fps = 25;
  return source;
}

void refused(const core::Status& status, const std::string& text) {
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.domain(), Code::InvalidArgument) << status.message();
  EXPECT_NE(status.message().find(text), std::string::npos)
      << status.message() << " lacks " << text;
}

TEST(AppDecklink, OpensThePairByItsNamesWithNoCalibration) {
  const core::Result<sensor::DecklinkStereo::Options> options =
      decklink_options(pair_source());
  ASSERT_TRUE(options.ok()) << options.status().message();
  EXPECT_EQ(options->left.id, "bm-1");
  EXPECT_EQ(options->left.input, "SDI 2");
  EXPECT_EQ(options->right.id, "bm-2");
  EXPECT_EQ(options->right.input, "SDI 1");
  EXPECT_EQ(options->width, 3840u);
  EXPECT_EQ(options->height, 2160u);
  EXPECT_EQ(options->fps, 25u);
  // calib makes the lenses: the pair opens uncalibrated.
  EXPECT_FALSE(options->calibration.has_value());
  const sensor::DecklinkStereo::Options defaults;
  EXPECT_EQ(options->max_skew_ns, defaults.max_skew_ns);
  EXPECT_EQ(options->lock_timeout, defaults.lock_timeout);

  // The pair, not the cameras' order, says which side is which.
  core::config::Source swapped = pair_source();
  swapped.pairs = {{"bm-2", "bm-1"}};
  const core::Result<sensor::DecklinkStereo::Options> other =
      decklink_options(swapped);
  ASSERT_TRUE(other.ok()) << other.status().message();
  EXPECT_EQ(other->left.id, "bm-2");
  EXPECT_EQ(other->left.input, "SDI 1");
  EXPECT_EQ(other->right.id, "bm-1");
  EXPECT_EQ(other->right.input, "SDI 2");
}

TEST(AppDecklink, RefusesAPairItCannotName) {
  core::config::Source source = pair_source();
  source.pairs.clear();
  refused(decklink_options(source).status(),
          "source.pairs: 0 pairs; calib_rig opens exactly one");
  source = pair_source();
  source.pairs.push_back({"bm-2", "bm-1"});
  refused(decklink_options(source).status(), "source.pairs: 2 pairs");
  source = pair_source();
  source.pairs = {{"bm-3", "bm-2"}};
  refused(decklink_options(source).status(),
          "source.pairs[0].left: bm-3 is not a camera of source.cameras");
  source.pairs = {{"bm-1", "bm-4"}};
  refused(decklink_options(source).status(),
          "source.pairs[0].right: bm-4 is not a camera of source.cameras");
}

}  // namespace
}  // namespace volumetric_kit::calib::app
