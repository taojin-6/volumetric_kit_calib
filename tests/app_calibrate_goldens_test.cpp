// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The Femto goldens: Calibrate on the synthetic Femto rig -- four Femto-like
// 4K colour cameras, 40 placements of the rig's board, the origin tag on the
// floor -- gives the array_calibration.json and residuals.csv kept in
// goldens/femto_calibrate/. Numbers agree within 1e-9 m, 1e-9 rad and
// 1e-6 px (a lens's coefficients and a corner's weight within 1e-9), as
// machines and OpenCV builds differ in their last bits; keys, ids and words
// byte for byte. The scene's noise is the same with every standard library
// (ObserveOptions::portable). VC_WRITE_GOLDENS=1 writes them afresh
// (AGENTS.md, "Build and validation").

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "kept_scene.hpp"
#include "scratch_dir.hpp"
#include "synthetic_scene.hpp"
#include "volumetric_kit/calib/app/calibrate_worker.hpp"
#include "volumetric_kit/calib/quality/results.hpp"
#include "volumetric_kit/core/base/file_io.hpp"

namespace volumetric_kit::calib::app {
namespace {

namespace fs = std::filesystem;
using json = nlohmann::ordered_json;  // keys in the file's order

constexpr double kPixels = 1e-6;  // px
// Metres, radians, and the unitless: a lens's coefficients, a weight.
constexpr double kFine = 1e-9;

const fs::path kGoldens = VC_GOLDENS_DIR;

// The tolerance on a number under `key`: pixels for a lens's intrinsics and
// a corner's place or residual, else kFine.
double tolerance_of(const std::string& key) {
  static const std::set<std::string> kInPixels = {"fx", "fy", "cx", "cy",
                                                  "u",  "v",  "du", "dv"};
  return kInPixels.count(key) != 0 ? kPixels : kFine;
}

// Whether two numbers agree within `tolerance`; `nan` agrees with `nan`.
bool near(double got, double want, double tolerance) {
  if (std::isnan(got) || std::isnan(want)) {
    return std::isnan(got) && std::isnan(want);
  }
  return std::abs(got - want) <= tolerance;
}

// `got` is `want`: the same keys, in the same order, the same words and
// lengths, and numbers within the tolerance of the key they sit under.
void expect_same(const json& got, const json& want, const std::string& path,
                 const std::string& key) {
  if (want.is_number() && got.is_number()) {
    EXPECT_PRED3(near, got.get<double>(), want.get<double>(), tolerance_of(key))
        << path;
    return;
  }
  ASSERT_EQ(got.type_name(), want.type_name()) << path;
  if (want.is_object()) {
    std::vector<std::string> got_keys;
    std::vector<std::string> want_keys;
    for (const auto& item : got.items()) got_keys.push_back(item.key());
    for (const auto& item : want.items()) want_keys.push_back(item.key());
    ASSERT_EQ(got_keys, want_keys) << path;
    for (const auto& item : want.items()) {
      expect_same(got.at(item.key()), item.value(), path + "." + item.key(),
                  item.key());
    }
  } else if (want.is_array()) {
    ASSERT_EQ(got.size(), want.size()) << path;
    for (std::size_t i = 0; i < want.size(); ++i) {
      expect_same(got[i], want[i], path + "[" + std::to_string(i) + "]", key);
    }
  } else {
    EXPECT_EQ(got, want) << path;
  }
}

// A CSV text's lines, each split at its commas.
std::vector<std::vector<std::string>> rows_of(const std::string& text) {
  std::vector<std::vector<std::string>> rows;
  std::istringstream lines(text);
  for (std::string line; std::getline(lines, line);) {
    std::vector<std::string>& fields = rows.emplace_back();
    std::istringstream cells(line);
    for (std::string field; std::getline(cells, field, ',');) {
      fields.push_back(field);
    }
  }
  return rows;
}

// residuals.csv: the same header and corners, each corner's capture, camera,
// corner and verdict byte for byte, its numbers within their tolerances. The
// first few cells that differ are named, then how many did.
void expect_same_residuals(const std::string& got, const std::string& want) {
  const std::vector<std::vector<std::string>> got_rows = rows_of(got);
  const std::vector<std::vector<std::string>> want_rows = rows_of(want);
  ASSERT_FALSE(want_rows.empty());
  const std::vector<std::string>& header = want_rows.front();
  ASSERT_EQ(got_rows.size(), want_rows.size());
  EXPECT_EQ(got_rows.front(), header);
  const std::set<std::string> numbers = {"u", "v", "du", "dv", "weight"};
  std::size_t differ = 0;
  for (std::size_t r = 1; r < want_rows.size(); ++r) {
    const std::vector<std::string>& g = got_rows[r];
    const std::vector<std::string>& w = want_rows[r];
    ASSERT_EQ(g.size(), header.size()) << "row " << r;
    ASSERT_EQ(w.size(), header.size()) << "row " << r;
    for (std::size_t c = 0; c < header.size(); ++c) {
      const bool same = numbers.count(header[c]) == 0
                            ? g[c] == w[c]
                            : near(std::strtod(g[c].c_str(), nullptr),
                                   std::strtod(w[c].c_str(), nullptr),
                                   tolerance_of(header[c]));
      if (!same && ++differ <= 5) {
        ADD_FAILURE() << "residuals.csv row " << r << " " << header[c] << ": "
                      << g[c] << ", the golden " << w[c];
      }
    }
  }
  EXPECT_EQ(differ, 0u) << "cells of residuals.csv differ";
}

// The synthetic Femto rig's captures as Calibrate takes them, the tag seen.
KeptCaptures femto_captures() {
  test::Scene scene = test::make_kept_scene(4, 40);
  scene.views = test::observe(scene.rig, scene.board, scene.plan,
                              test::ObserveOptions{0.3, 1, true});
  return test::with_origin_tag(test::kept_captures(scene, 0, "unwritten"),
                               scene);
}

TEST(FemtoGoldens, CalibrateGivesTheGoldenFiles) {
  const core::Result<CalibrateResult> made = calibrate(femto_captures());
  ASSERT_TRUE(made.ok()) << made.status().message();
  const quality::ResultFiles& files = made->files;
  ASSERT_TRUE(made->world.origin.tag) << made->world.origin.reason;
  const fs::path calibration = kGoldens / quality::kArrayCalibrationFile;
  const fs::path residuals = kGoldens / quality::kResidualsFile;
  if (std::getenv("VC_WRITE_GOLDENS") != nullptr) {
    ASSERT_TRUE(core::write_file(calibration.string(), files.array_calibration,
                                 core::WriteMode::Replace)
                    .ok());
    ASSERT_TRUE(core::write_file(residuals.string(), files.residuals,
                                 core::WriteMode::Replace)
                    .ok());
  }
  expect_same(json::parse(files.array_calibration),
              json::parse(test::read(calibration)), "array_calibration", "");
  expect_same_residuals(files.residuals, test::read(residuals));
}

}  // namespace
}  // namespace volumetric_kit::calib::app
