// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A program that compiles stb_image itself, with external linkage -- as gfx's
// glTF loader does -- must still link calib_capture, whose copy of stb has
// internal linkage. This file is that program: if calib's stb symbols were
// global, it would not link.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include "stb_image.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "capture_test_util.hpp"
#include "volumetric_kit/calib/capture/session.hpp"

namespace volumetric_kit::calib::capture {
namespace {

using testing::make_camera;
using testing::pattern_image;
using testing::ScratchDir;

TEST(CaptureStbLink, CoexistsWithTheProgramsOwnStbImage) {
  const ScratchDir dir;
  const SessionCamera camera = make_camera("CL2A141000N", 16, 8);
  auto writer = SessionWriter::create(dir.str(), {camera}, 0);
  ASSERT_TRUE(writer.ok()) << writer.status().message();
  const LumaImage image = pattern_image(camera.lens.image_size(), 5);
  ASSERT_TRUE(writer->add(1, 1, {image}).ok());

  // The program's own stb reads calib's file, and calib's reads it too.
  const std::string path = session_image_path(dir.str(), camera.serial, 0);
  int w = 0;
  int h = 0;
  int n = 0;
  unsigned char* data = stbi_load(path.c_str(), &w, &h, &n, 1);
  ASSERT_NE(data, nullptr);
  const std::vector<unsigned char> theirs(data, data + w * h);
  stbi_image_free(data);
  EXPECT_EQ(theirs, image.pixels);
  auto ours = read_session_image(dir.str(), camera.serial, 0);
  ASSERT_TRUE(ours.ok());
  EXPECT_EQ(ours->pixels, image.pixels);
}

}  // namespace
}  // namespace volumetric_kit::calib::capture
