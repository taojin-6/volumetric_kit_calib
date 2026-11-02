// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// 02_lens_from_images: one camera's lens from saved pictures of a
// checkerboard, offline: find the board in each picture, fit the lens, judge
// it and write the lens-only calibration file. The first pictures of a lens
// session can be judged again here without going back to the lab.
//
//   lens_from_images <camera id> <squares across> <squares down> <square mm>
//                    <out dir> <img.pgm>...

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/detect/board.hpp"
#include "volumetric_kit/calib/quality/lens_gate.hpp"
#include "volumetric_kit/calib/solve/lens.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace calib = volumetric_kit::calib;
namespace core = volumetric_kit::core;
namespace camera = volumetric_kit::sensor::camera;

namespace {

core::Status run(int argc, char** argv) {
  calib::Target target = calib::legacy_checkerboard();
  target.cols = std::atoi(argv[2]) - 1;
  target.rows = std::atoi(argv[3]) - 1;
  target.square_size_m = std::atof(argv[4]) / 1000.0;
  VKC_ASSIGN(const calib::compute::Board board,
             calib::compute::Board::create(target));

  std::vector<calib::Observation> views;
  camera::ImageSize size{};
  for (int i = 6; i < argc; ++i) {
    const cv::Mat grey = cv::imread(argv[i], cv::IMREAD_GRAYSCALE);
    if (grey.empty()) {
      return core::Status::not_found(std::string("cannot read ") + argv[i]);
    }
    size = {static_cast<std::uint32_t>(grey.cols),
            static_cast<std::uint32_t>(grey.rows)};
    VKC_ASSIGN(calib::Detection found,
               calib::detect::detect_board(
                   {size.width, size.height, grey.step, grey.data}, board));
    found.observation.camera_id = argv[1];
    found.observation.frame_id = i - 6;
    views.push_back(std::move(found.observation));
  }

  VKC_ASSIGN(calib::solve::LensFit fit,
             calib::solve::solve_lens(argv[1], size, board, views));
  VKC_ASSIGN(const calib::quality::LensVerdict verdict,
             calib::quality::judge_lens(std::move(fit)));
  std::printf("%s: hold-out %.2f px, %s\n", argv[1], verdict.fit.holdout_rms_px,
              verdict.pass ? "pass" : verdict.failed.front().c_str());
  VKC_ASSIGN(const calib::quality::ResultFiles files,
             calib::quality::format_lens_results({verdict}, {}, nullptr));
  return core::write_file(std::string(argv[5]) + "/array_calibration.json",
                          files.array_calibration, core::WriteMode::CreateNew);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 7) {
    std::fprintf(stderr,
                 "usage: lens_from_images <camera id> <squares across> "
                 "<squares down> <square mm> <out dir> <img.pgm>...\n");
    return 2;
  }
  const core::Status status = run(argc, argv);
  if (!status) {
    std::fprintf(stderr, "error: %s\n", status.message().c_str());
    return 1;
  }
  return 0;
}
