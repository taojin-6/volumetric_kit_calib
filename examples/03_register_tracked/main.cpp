// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// 03_register_tracked: a tracked camera's world in the rig's, from its stills
// of the origin tag: find the tag in each still, place the tracking world at
// the tag with the edge the rig measured, and write the registration a
// tracked source reads. A window, in seconds from the first still, picks one
// hold of a take (a start hold and an end hold).
//
//   register_tracked <array_calibration.json> <stills.json> <edge mm>
//                    <out.json> [<from s> <to s>]

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/detect/tag.hpp"
#include "volumetric_kit/calib/solve/tracked_world.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"
#include "volumetric_kit/sensor/camera/tracked_stills.hpp"

namespace calib = volumetric_kit::calib;
namespace core = volumetric_kit::core;
namespace camera = volumetric_kit::sensor::camera;

namespace {

core::Status run(int argc, char** argv) {
  VKC_ASSIGN(const camera::ArrayCalibration array,
             camera::read_array_calibration(argv[1]));
  VKC_ASSIGN(const camera::TrackedStills stills,
             camera::read_tracked_stills(argv[2]));
  const double from_s = argc == 7 ? std::atof(argv[5]) : 0.0;
  const double to_s = argc == 7 ? std::atof(argv[6]) : 1e300;
  std::vector<calib::solve::TrackedTagView> views;
  std::size_t held = 0;
  for (const camera::TrackedStill& s : stills.stills) {
    const double t =
        1e-9 * double(s.timestamp_ns - stills.stills.front().timestamp_ns);
    if (t < from_s || t > to_s) continue;
    ++held;
    const std::string file =
        std::filesystem::path(argv[2]).replace_filename(s.image).string();
    const cv::Mat grey = cv::imread(file, cv::IMREAD_GRAYSCALE);
    if (grey.empty()) return core::Status::not_found("cannot read " + file);
    const camera::ImageSize size{std::uint32_t(grey.cols),
                                 std::uint32_t(grey.rows)};
    VKC_ASSIGN(const calib::detect::MarkerSearch search,
               calib::detect::detect_tag(
                   {size.width, size.height, grey.step, grey.data},
                   calib::kOriginTag));
    if (search.found) {
      views.push_back(
          {s.image, size, s.model, s.cam_to_tracking, *search.found});
    }
  }
  VKC_ASSIGN(const calib::solve::TrackedWorld found,
             calib::solve::tracked_world_from_tag(views, calib::kOriginTag,
                                                  std::atof(argv[3]) / 1000.0));
  // A miss is refused here, with its reason.
  VKC_ASSIGN(const camera::TrackedRegistration reg,
             calib::solve::register_tracked(stills, array.world, found));
  std::printf("%u of %zu stills, %.2f px\n", reg.views, held, reg.rms_px);
  return camera::write_tracked_registration(argv[4], reg,
                                            core::WriteMode::CreateNew);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 5 && argc != 7) {
    std::fprintf(stderr,
                 "usage: register_tracked <array> <stills> <edge mm> <out> "
                 "[<from s> <to s>]\n");
    return 2;
  }
  const core::Status status = run(argc, argv);
  if (!status) {
    std::fprintf(stderr, "error: %s\n", status.message().c_str());
    return 1;
  }
  return 0;
}
