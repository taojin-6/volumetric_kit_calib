// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/viz/scene_view.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include <glm/mat3x3.hpp>
#include <opencv2/core.hpp>

#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::viz {

namespace {

namespace pipelines = gfx::pipelines;
using recon::camera::Mat3d;
using recon::camera::Mat4d;
using recon::camera::Vec3d;
using Lines = std::vector<pipelines::ColorVertex>;

// How far the view aims ahead of the cameras, and how far back and how high
// its eye starts.
constexpr double kAimM = 2.0;
constexpr double kMinDistanceM = 3.0;
constexpr double kReachScale = 1.5;
constexpr float kElevation = 0.35f;  // radians, about 20 degrees

// A depth frustum's colour: its camera's tint at this fraction.
constexpr double kDepthShade = 0.6;

Vec3d apply(const Mat4d& transform, const Vec3d& point) {
  return Vec3d(transform * glm::dvec4(point, 1.0));
}

void line(Lines& out, const Vec3d& a, const Vec3d& b, std::uint32_t rgba) {
  out.push_back({glm::vec3(a), rgba});
  out.push_back({glm::vec3(b), rgba});
}

std::uint32_t shaded(std::uint32_t rgba, double fraction) {
  std::uint32_t out = rgba & 0xFF000000u;
  for (int shift = 0; shift < 24; shift += 8) {
    const auto byte = static_cast<double>((rgba >> shift) & 0xFFu);
    out |= static_cast<std::uint32_t>(std::lround(byte * fraction)) << shift;
  }
  return out;
}

// `v` with its part along the unit `up` removed, at unit length; empty when
// what is left is too short to have a direction.
std::optional<Vec3d> levelled(const Vec3d& v, const Vec3d& up) {
  const Vec3d flat = v - glm::dot(v, up) * up;
  const double length = glm::length(flat);
  if (!(length > 1e-6)) return std::nullopt;
  return flat / length;
}

core::Status append(Lines& out, core::Result<Lines> lines,
                    const std::string& what) {
  if (!lines.ok()) return lines.status().with_context(what);
  out.insert(out.end(), lines.value().begin(), lines.value().end());
  return {};
}

}  // namespace

core::Result<Lines> frustum_lines(const recon::camera::CameraModel& camera,
                                  const Mat4d& to_world, double depth_m,
                                  std::uint32_t rgba) {
  return compute::guarded("frustum_lines", [&]() -> core::Result<Lines> {
    if (!(depth_m > 0.0) || !std::isfinite(depth_m)) {
      return core::Status::invalid_argument(
          "frustum_lines: the depth must be positive and finite");
    }
    VKC_TRY(recon::camera::check_rigid(to_world).with_context("frustum_lines"));
    VKC_ASSIGN(const compute::CvCamera cv, compute::cv_camera(camera));
    // The image's outer edges: half a pixel past the corner pixels' centres.
    const double right = static_cast<double>(camera.size.width) - 0.5;
    const double bottom = static_cast<double>(camera.size.height) - 0.5;
    VKC_ASSIGN(
        const std::vector<cv::Point2d> rays,
        compute::undistort_points(
            cv,
            {{-0.5, -0.5}, {right, -0.5}, {right, bottom}, {-0.5, bottom}}));
    const Vec3d centre = apply(to_world, Vec3d(0.0));
    std::array<Vec3d, 4> corners{};
    for (std::size_t i = 0; i < corners.size(); ++i) {
      if (!std::isfinite(rays[i].x) || !std::isfinite(rays[i].y)) {
        return core::Status::invalid_argument(
            "frustum_lines: the lens takes no ray back from an image corner");
      }
      corners[i] = apply(
          to_world, Vec3d(rays[i].x * depth_m, rays[i].y * depth_m, depth_m));
    }
    Lines out;
    out.reserve(16);
    for (const Vec3d& corner : corners) line(out, centre, corner, rgba);
    for (std::size_t i = 0; i < corners.size(); ++i) {
      line(out, corners[i], corners[(i + 1) % corners.size()], rgba);
    }
    return out;
  });
}

core::Result<Lines> axes_lines(double length_m) {
  return compute::guarded("axes_lines", [&]() -> core::Result<Lines> {
    if (!(length_m > 0.0) || !std::isfinite(length_m)) {
      return core::Status::invalid_argument(
          "axes_lines: the length must be positive and finite");
    }
    Lines out;
    out.reserve(6);
    const Vec3d origin(0.0);
    line(out, origin, Vec3d(length_m, 0.0, 0.0),
         pipelines::pack_rgba8(255, 64, 64));
    line(out, origin, Vec3d(0.0, length_m, 0.0),
         pipelines::pack_rgba8(64, 220, 64));
    line(out, origin, Vec3d(0.0, 0.0, length_m),
         pipelines::pack_rgba8(80, 140, 255));
    return out;
  });
}

core::Result<Lines> board_lines(const compute::Board& board,
                                const Mat4d& board_to_world,
                                std::uint32_t rgba) {
  return compute::guarded("board_lines", [&]() -> core::Result<Lines> {
    VKC_TRY(
        recon::camera::check_rigid(board_to_world).with_context("board_lines"));
    // The chessboard spans twice its centre, from the frame's origin.
    const cv::Point3d middle = board.centre();
    const double w = 2.0 * middle.x;
    const double h = 2.0 * middle.y;
    const std::array<Vec3d, 4> corners{
        apply(board_to_world, Vec3d(0.0, 0.0, 0.0)),
        apply(board_to_world, Vec3d(w, 0.0, 0.0)),
        apply(board_to_world, Vec3d(w, h, 0.0)),
        apply(board_to_world, Vec3d(0.0, h, 0.0))};
    Lines out;
    out.reserve(8);
    for (std::size_t i = 0; i < corners.size(); ++i) {
      line(out, corners[i], corners[(i + 1) % corners.size()], rgba);
    }
    return out;
  });
}

core::Result<Lines> tag_lines(double edge_m, std::uint32_t rgba) {
  return compute::guarded("tag_lines", [&]() -> core::Result<Lines> {
    if (!(edge_m > 0.0) || !std::isfinite(edge_m)) {
      return core::Status::invalid_argument(
          "tag_lines: the edge must be positive and finite");
    }
    const double h = 0.5 * edge_m;
    const std::array<Vec3d, 4> corners{Vec3d(-h, h, 0.0), Vec3d(h, h, 0.0),
                                       Vec3d(h, -h, 0.0), Vec3d(-h, -h, 0.0)};
    Lines out;
    out.reserve(8);
    for (std::size_t i = 0; i < corners.size(); ++i) {
      line(out, corners[i], corners[(i + 1) % corners.size()], rgba);
    }
    return out;
  });
}

core::Result<Lines> scene_lines(const std::vector<SceneCamera>& cameras,
                                const compute::Board* board,
                                const std::vector<Mat4d>& boards_to_world,
                                const SceneOptions& options) {
  return compute::guarded("scene_lines", [&]() -> core::Result<Lines> {
    if (!boards_to_world.empty() && board == nullptr) {
      return core::Status::invalid_argument(
          "scene_lines: solved boards without the board they are of");
    }
    Lines out;
    if (options.axes_m != 0.0) {
      VKC_TRY(append(out, axes_lines(options.axes_m), "scene_lines: axes"));
    }
    for (std::size_t i = 0; i < cameras.size(); ++i) {
      const SceneCamera& camera = cameras[i];
      if (!camera.shown || !camera.color_to_world) continue;
      const std::string what = "scene_lines: camera " + std::to_string(i);
      VKC_TRY(append(out,
                     frustum_lines(camera.color, *camera.color_to_world,
                                   options.frustum_m, camera.tint),
                     what));
      if (options.depth_frusta && camera.depth) {
        VKC_TRY(append(
            out,
            frustum_lines(*camera.depth,
                          *camera.color_to_world * camera.depth_to_color,
                          options.frustum_m, shaded(camera.tint, kDepthShade)),
            what + "'s depth"));
      }
    }
    for (std::size_t k = 0; k < boards_to_world.size(); ++k) {
      VKC_TRY(append(
          out, board_lines(*board, boards_to_world[k], options.board_rgba),
          "scene_lines: board " + std::to_string(k)));
    }
    if (options.tag_edge_m != 0.0) {
      VKC_TRY(append(out, tag_lines(options.tag_edge_m, options.tag_rgba),
                     "scene_lines: the origin tag"));
    }
    return out;
  });
}

core::Result<std::vector<pipelines::DepthCloudDraw>> cloud_draws(
    const std::vector<std::optional<recon::sensor::DeviceFrame>>& frames,
    const std::vector<SceneCamera>& cameras, const CloudStyle& style) {
  return compute::guarded(
      "cloud_draws",
      [&]() -> core::Result<std::vector<pipelines::DepthCloudDraw>> {
        if (frames.size() != cameras.size()) {
          return core::Status::invalid_argument(
              "cloud_draws: " + std::to_string(frames.size()) + " frames for " +
              std::to_string(cameras.size()) + " cameras");
        }
        std::vector<pipelines::DepthCloudDraw> draws;
        draws.reserve(cameras.size());
        for (std::size_t i = 0; i < cameras.size(); ++i) {
          const SceneCamera& camera = cameras[i];
          if (!camera.shown || !camera.color_to_world || !frames[i]) continue;
          CloudStyle own = style;
          own.tint = camera.tint;
          core::Result<pipelines::DepthCloudDraw> draw = cloud_draw(
              *frames[i], *camera.color_to_world, camera.depth_to_color, own);
          if (!draw.ok()) {
            return draw.status().with_context("camera " + std::to_string(i));
          }
          draws.push_back(draw.value());
        }
        return draws;
      });
}

core::Result<Mat3d> display_rotation(const std::vector<Mat4d>& color_to_world,
                                     const std::optional<Vec3d>& up_given) {
  return compute::guarded("display_rotation", [&]() -> core::Result<Mat3d> {
    if (color_to_world.empty()) {
      return core::Status::invalid_argument(
          "display_rotation: no camera is placed");
    }
    Vec3d ups(0.0);
    Vec3d forwards(0.0);
    for (const Mat4d& pose : color_to_world) {
      VKC_TRY(
          recon::camera::check_rigid(pose).with_context("display_rotation"));
      const Mat3d turn(pose);
      ups -= turn[1];  // the camera's -y, its image's up, in the world
      forwards += turn[2];
    }
    if (up_given) {
      const double given = glm::length(*up_given);
      if (!(given > 0.0) || !std::isfinite(given)) {
        return core::Status::invalid_argument(
            "display_rotation: the world's up must be finite and have a "
            "length");
      }
      ups = *up_given / given;
    }
    const double length = glm::length(ups);
    if (!(length > 1e-6)) {
      return core::Status::invalid_argument(
          "display_rotation: the cameras' ups cancel");
    }
    const Vec3d up = ups / length;
    std::optional<Vec3d> forward = levelled(forwards, up);
    for (const Vec3d& fallback : {Mat3d(color_to_world.front())[2],
                                  Vec3d(0.0, 0.0, 1.0), Vec3d(1.0, 0.0, 0.0)}) {
      if (forward) break;
      forward = levelled(fallback, up);
    }
    // The view's axes in the world: +Y up, -Z forward, +X their cross.
    const Vec3d z = -*forward;
    const Vec3d x = glm::cross(up, z);
    return glm::transpose(Mat3d(x, up, z));
  });
}

WindowSplit window_split(ScenePlacement placement, VkExtent2D extent) noexcept {
  WindowSplit split;
  switch (placement) {
    case ScenePlacement::Hidden:
      split.mosaic = {{0, 0}, extent};
      break;
    case ScenePlacement::Beside: {
      const std::uint32_t left = extent.width - extent.width / 2;
      split.mosaic = {{0, 0}, {left, extent.height}};
      split.scene = {{static_cast<std::int32_t>(left), 0},
                     {extent.width - left, extent.height}};
      break;
    }
    case ScenePlacement::FullWindow:
      split.scene = {{0, 0}, extent};
      break;
  }
  return split;
}

core::Status SceneView::frame(const std::vector<Mat4d>& color_to_world,
                              const std::optional<Vec3d>& up) {
  return compute::guarded("SceneView::frame", [&]() -> core::Status {
    VKC_ASSIGN(const Mat3d turn, display_rotation(color_to_world, up));
    Vec3d centre(0.0);
    Vec3d forward(0.0);
    for (const Mat4d& pose : color_to_world) {
      centre += turn * Vec3d(pose[3]);
      forward += turn * Mat3d(pose)[2];
    }
    const auto n = static_cast<double>(color_to_world.size());
    const Vec3d target = centre / n + kAimM * (forward / n);
    double reach = 0.0;
    for (const Mat4d& pose : color_to_world) {
      reach = std::max(reach, glm::length(turn * Vec3d(pose[3]) - target));
    }
    rotation_ = turn;
    orbit_.set_target(glm::vec3(target));
    orbit_.set_distance(
        static_cast<float>(std::max(kMinDistanceM, kReachScale * reach)));
    orbit_.set_azimuth(0.0f);
    orbit_.set_elevation(kElevation);
    return {};
  });
}

glm::mat4 SceneView::view_proj(VkRect2D viewport) const {
  const float width = static_cast<float>(viewport.extent.width);
  const float height = static_cast<float>(viewport.extent.height);
  const float aspect = width > 0.0f && height > 0.0f ? width / height : 1.0f;
  const gfx::camera::Camera camera =
      orbit_.to_camera(kFovY, aspect, kNear, kFar);
  return camera.view_proj() * glm::mat4(glm::mat3(rotation_));
}

}  // namespace volumetric_kit::calib::viz
