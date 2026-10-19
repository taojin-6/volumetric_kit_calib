// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/quality/floor_boards.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"

namespace volumetric_kit::calib::quality {

core::Result<std::vector<FloorBoard>> floor_boards(
    const compute::Board& board, const std::vector<solve::SolvedBoard>& boards,
    const std::vector<recon::camera::Mat4d>& boards_to_world,
    const FloorBoardOptions& options) {
  constexpr const char* kWhat = "floor_boards";
  return compute::guarded(
      kWhat, [&]() -> core::Result<std::vector<FloorBoard>> {
        for (const auto& [name, value] :
             {std::pair{"max_angle_deg", options.max_angle_deg},
              std::pair{"max_height_m", options.max_height_m}}) {
          if (!(value > 0.0) || !std::isfinite(value)) {
            return core::Status::invalid_argument(
                std::string(kWhat) + ": options." + name +
                " must be positive and finite");
          }
        }
        if (boards_to_world.size() != boards.size()) {
          return core::Status::invalid_argument(
              std::string(kWhat) + ": " +
              std::to_string(boards_to_world.size()) + " poses for " +
              std::to_string(boards.size()) + " boards");
        }
        std::vector<FloorBoard> floor;
        for (std::size_t b = 0; b < boards.size(); ++b) {
          const recon::camera::Mat4d& pose = boards_to_world[b];
          VKC_TRY(recon::camera::check_rigid(pose).with_context(
              std::string(kWhat) + ": the board of capture " +
              std::to_string(boards[b].capture)));
          const double angle =
              compute::angle_deg(compute::face_normal(pose), {0.0, 0.0, 1.0});
          const double height = compute::centre_in(board, pose).z;
          if (angle <= options.max_angle_deg &&
              std::abs(height) <= options.max_height_m) {
            floor.push_back({boards[b].capture, angle, height});
          }
        }
        return floor;
      });
}

}  // namespace volumetric_kit::calib::quality
