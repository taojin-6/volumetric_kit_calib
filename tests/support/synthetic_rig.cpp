// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "synthetic_rig.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>

#include "volumetric_kit/calib/compute/cv_camera.hpp"

namespace volumetric_kit::calib::testing {
namespace {

namespace camera = recon::camera;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

compute::CvCamera cv_lens(const camera::CameraModel& model) {
  return compute::cv_camera(model).value();
}

// The rigid transform whose columns are the axes x, y, z and the origin o.
camera::Mat4d frame(const camera::Vec3d& x, const camera::Vec3d& y,
                    const camera::Vec3d& z, const camera::Vec3d& o) {
  camera::Mat4d m(1.0);
  m[0] = glm::dvec4(x, 0.0);
  m[1] = glm::dvec4(y, 0.0);
  m[2] = glm::dvec4(z, 0.0);
  m[3] = glm::dvec4(o, 1.0);
  return m;
}

camera::Vec3d apply(const camera::Mat4d& m, const camera::Vec3d& p) {
  return camera::Vec3d(m * glm::dvec4(p, 1.0));
}

// World to the camera at `centre` looking at `target`, x level with the floor
// before `roll_deg` turns it about the view direction.
camera::Mat4d look_at(const camera::Vec3d& centre, const camera::Vec3d& target,
                      double roll_deg) {
  const camera::Vec3d z = glm::normalize(target - centre);
  const camera::Vec3d x0 =
      glm::normalize(glm::cross(z, camera::Vec3d(0.0, 0.0, 1.0)));
  const camera::Vec3d y0 = glm::cross(z, x0);
  const double a = roll_deg * kDeg;
  const camera::Vec3d x = std::cos(a) * x0 + std::sin(a) * y0;
  return camera::rigid_inverse(frame(x, glm::cross(z, x), z, centre));
}

// Board to world: the printed face's normal `normal`, the board's x along
// `yaw` as far as the normal allows, centred at `centre`. The board's z is
// -normal (OpenCV's board frame).
camera::Mat4d board_pose(const camera::Vec3d& centre,
                         const camera::Vec3d& normal, double yaw,
                         const cv::Point2d& extent) {
  const camera::Vec3d z = -normal;
  const camera::Vec3d d(std::cos(yaw), std::sin(yaw), 0.0);
  const camera::Vec3d x = glm::normalize(d - glm::dot(d, normal) * normal);
  const camera::Vec3d y = glm::cross(z, x);
  return frame(x, y, z, centre - 0.5 * extent.x * x - 0.5 * extent.y * y);
}

// The chessboard's outer size (metres), from OpenCV's board.
cv::Point2d board_extent(const compute::Board& board) {
  const cv::Point3f corner = board.charuco().getRightBottomCorner();
  return {corner.x, corner.y};
}

// Points along the rectangle [-margin, extent + margin], `per_side` per side.
std::vector<cv::Point3d> outline(const cv::Point2d& extent, double margin,
                                 int per_side) {
  std::vector<cv::Point3d> points;
  const double w = extent.x + 2.0 * margin;
  const double h = extent.y + 2.0 * margin;
  for (int i = 0; i < per_side; ++i) {
    const double s = static_cast<double>(i) / per_side;
    points.emplace_back(-margin + s * w, -margin, 0.0);
    points.emplace_back(-margin + w, -margin + s * h, 0.0);
    points.emplace_back(-margin + w - s * w, -margin + h, 0.0);
    points.emplace_back(-margin, -margin + h - s * h, 0.0);
  }
  return points;
}

// Whether `cam` sees the whole board at `board_to_world`.
bool sees(const SyntheticCamera& cam, const camera::Mat4d& board_to_world,
          const cv::Point2d& extent, const PlanOptions& options) {
  const camera::Mat4d board_to_camera = cam.world_to_camera * board_to_world;
  const camera::Vec3d face = -camera::Vec3d(board_to_camera[2]);
  const camera::Vec3d centre =
      apply(board_to_camera, {0.5 * extent.x, 0.5 * extent.y, 0.0});
  if (glm::dot(face, glm::normalize(-centre)) <
      std::cos(options.max_tilt_deg * kDeg)) {
    return false;
  }
  const std::vector<cv::Point3d> edge = outline(extent, 0.0, 16);
  for (const cv::Point3d& p : edge) {
    if (apply(board_to_camera, {p.x, p.y, p.z}).z < options.min_depth_m) {
      return false;
    }
  }
  const double w = cam.model.size.width;
  const double h = cam.model.size.height;
  const double m = options.margin_px;
  for (const cv::Point2d& q : project(cam.model, board_to_camera, edge)) {
    if (!(q.x >= m && q.y >= m && q.x <= w - 1.0 - m && q.y <= h - 1.0 - m)) {
      return false;
    }
  }
  return true;
}

}  // namespace

camera::CameraModel color_like_lens() {
  return {{3840, 2160},
          {2240.3, 2239.1, 1913.4, 1039.2},
          {0.073, -0.10, -0.0005, -0.00005, 0.041, 0.0021, -0.0085, 0.0042}};
}

camera::CameraModel depth_like_lens() {
  return {{640, 576},
          {504.2, 504.3, 321.1, 330.6},
          {29.29, 9.9, 1e-5, -2e-5, 0.3, 29.6, 15.0, 3.77}};
}

std::vector<SyntheticCamera> synthetic_rig() {
  // A camera on the ring at `azimuth_deg`, `radius_m` out and `height_m` up,
  // looking at `target`, with its own intrinsics and the colour-like lens's
  // radial terms scaled by `lens_scale`.
  const auto ring_camera =
      [](const char* id, double azimuth_deg, double radius_m, double height_m,
         double roll_deg, const camera::Vec3d& target,
         const camera::PinholeIntrinsics& intrinsics, double lens_scale) {
        camera::CameraModel model = color_like_lens();
        model.intrinsics = intrinsics;
        camera::RationalDistortion& d = model.distortion;
        for (double* k : {&d.k1, &d.k2, &d.k3, &d.k4, &d.k5, &d.k6}) {
          *k *= lens_scale;
        }
        const camera::Vec3d centre(radius_m * std::cos(azimuth_deg * kDeg),
                                   radius_m * std::sin(azimuth_deg * kDeg),
                                   height_m);
        return SyntheticCamera{id, model, look_at(centre, target, roll_deg)};
      };
  return {ring_camera("SYN0", 0.0, 1.85, 2.50, 1.0, {0.05, -0.05, 0.85},
                      {2240.3, 2239.1, 1913.4, 1039.2}, 1.00),
          ring_camera("SYN1", 78.0, 1.95, 2.55, -1.5, {-0.10, 0.05, 0.80},
                      {2238.7, 2237.9, 1917.8, 1034.6}, 0.97),
          ring_camera("SYN2", 183.0, 2.05, 2.45, 0.5, {0.00, 0.10, 0.90},
                      {2242.0, 2241.2, 1909.1, 1043.0}, 1.03),
          ring_camera("SYN3", 272.0, 1.80, 2.52, -0.8, {0.08, 0.00, 0.85},
                      {2239.5, 2238.6, 1915.6, 1037.4}, 0.98)};
}

std::vector<cv::Point2d> project(const camera::CameraModel& model,
                                 const camera::Mat4d& to_camera,
                                 const std::vector<cv::Point3d>& points) {
  std::vector<cv::Point2d> pixels;
  if (points.empty()) return pixels;
  std::vector<cv::Point3d> in_camera;
  in_camera.reserve(points.size());
  for (const cv::Point3d& p : points) {
    const camera::Vec3d q = apply(to_camera, {p.x, p.y, p.z});
    in_camera.emplace_back(q.x, q.y, q.z);
  }
  const compute::CvCamera cam = cv_lens(model);
  cv::projectPoints(in_camera, cv::Vec3d(0.0, 0.0, 0.0),
                    cv::Vec3d(0.0, 0.0, 0.0), cam.camera_matrix, cam.distortion,
                    pixels);
  return pixels;
}

std::vector<Placement> synthetic_plan(const std::vector<SyntheticCamera>& rig,
                                      const compute::Board& board,
                                      const PlanOptions& options) {
  std::mt19937 rng(options.seed);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  const cv::Point2d extent = board_extent(board);
  std::vector<Placement> plan;
  for (std::size_t attempt = 0;
       plan.size() < options.count && attempt < 1000 * options.count;
       ++attempt) {
    const double yaw = 2.0 * kPi * uniform(rng);
    const double kind = uniform(rng);
    const bool floor = kind < 0.4;
    double height = floor ? 0.0 : kind < 0.6 ? 0.5 : kind < 0.8 ? 1.0 : 1.5;
    if (!floor) height += 0.05 * (2.0 * uniform(rng) - 1.0);
    const double radius = (floor ? 1.0 : 0.7) * std::sqrt(uniform(rng));
    const double theta = 2.0 * kPi * uniform(rng);
    const camera::Vec3d centre(radius * std::cos(theta),
                               radius * std::sin(theta), height);
    camera::Vec3d normal(0.0, 0.0, 1.0);
    if (!floor && uniform(rng) < 0.5) {
      // Tilted 5-15 degrees toward one camera.
      const std::size_t toward = std::min<std::size_t>(
          rig.size() - 1, static_cast<std::size_t>(uniform(rng) * rig.size()));
      camera::Vec3d to =
          apply(camera::rigid_inverse(rig[toward].world_to_camera),
                camera::Vec3d(0.0)) -
          centre;
      to.z = 0.0;
      const double tilt = (5.0 + 10.0 * uniform(rng)) * kDeg;
      normal = std::cos(tilt) * normal + std::sin(tilt) * glm::normalize(to);
    }
    Placement placement;
    placement.board_to_world = board_pose(centre, normal, yaw, extent);
    for (std::size_t c = 0; c < rig.size(); ++c) {
      if (sees(rig[c], placement.board_to_world, extent, options)) {
        placement.cameras.push_back(c);
      }
    }
    if (placement.cameras.size() >= options.min_cameras) {
      plan.push_back(std::move(placement));
    }
  }
  return plan;
}

std::vector<cv::Point2d> undistort_pixels(
    const camera::CameraModel& model, const std::vector<cv::Point2d>& pixels) {
  std::vector<cv::Point2d> normalized;
  if (pixels.empty()) return normalized;
  const compute::CvCamera cam = cv_lens(model);
  cv::undistortPoints(
      pixels, normalized, cam.camera_matrix, cam.distortion, cv::noArray(),
      cv::noArray(),
      cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100,
                       1e-12));
  return normalized;
}

cv::Mat render_board(const camera::CameraModel& model,
                     const camera::Mat4d& board_to_camera,
                     const compute::Board& board,
                     const RenderOptions& options) {
  const compute::CvCamera cam = cv_lens(model);
  cv::Mat image(cam.size, CV_32F, cv::Scalar(options.background));

  // The texture: the board, a white margin around it, at the levels asked.
  const cv::Point2d extent = board_extent(board);
  const double s = options.texture_px_per_m;
  const double margin = options.margin_m;
  cv::Mat drawn;
  board.charuco().generateImage(
      cv::Size(static_cast<int>(std::lround(extent.x * s)),
               static_cast<int>(std::lround(extent.y * s))),
      drawn, 0, 1);
  const int margin_px = static_cast<int>(std::lround(margin * s));
  cv::Mat padded;
  cv::copyMakeBorder(drawn, padded, margin_px, margin_px, margin_px, margin_px,
                     cv::BORDER_CONSTANT, cv::Scalar(255));
  cv::Mat texture;
  padded.convertTo(texture, CV_32F, (options.white - options.black) / 255.0,
                   options.black);

  // The pixels the board and its margin can reach.
  double min_x = 1e300, min_y = 1e300, max_x = -1e300, max_y = -1e300;
  for (const cv::Point2d& q :
       project(model, board_to_camera, outline(extent, margin, 40))) {
    min_x = std::min(min_x, q.x);
    max_x = std::max(max_x, q.x);
    min_y = std::min(min_y, q.y);
    max_y = std::max(max_y, q.y);
  }
  const int x0 = std::max(0, static_cast<int>(std::floor(min_x)) - 4);
  const int y0 = std::max(0, static_cast<int>(std::floor(min_y)) - 4);
  const int x1 =
      std::min(cam.size.width - 1, static_cast<int>(std::ceil(max_x)) + 4);
  const int y1 =
      std::min(cam.size.height - 1, static_cast<int>(std::ceil(max_y)) + 4);
  cv::Mat out;
  if (x1 <= x0 || y1 <= y0) {
    image.convertTo(out, CV_8U);
    return out;
  }

  // Each sample's ray meets the board's plane; the texture is read there.
  const int S = options.supersampling;
  const int w = x1 - x0 + 1;
  const int h = y1 - y0 + 1;
  const int W = w * S;
  const int H = h * S;
  std::vector<cv::Point2d> samples(static_cast<std::size_t>(W) * H);
  for (int j = 0; j < H; ++j) {
    for (int i = 0; i < W; ++i) {
      samples[static_cast<std::size_t>(j) * W + i] = {x0 + (i + 0.5) / S - 0.5,
                                                      y0 + (j + 0.5) / S - 0.5};
    }
  }
  const std::vector<cv::Point2d> rays = undistort_pixels(model, samples);
  const camera::Mat3d rotation(board_to_camera);
  const camera::Vec3d normal = rotation[2];
  const camera::Vec3d origin(board_to_camera[3]);
  const camera::Mat3d to_board = glm::transpose(rotation);
  const double offset = glm::dot(normal, origin);
  cv::Mat map_x(H, W, CV_32F), map_y(H, W, CV_32F);
  for (int j = 0; j < H; ++j) {
    for (int i = 0; i < W; ++i) {
      const cv::Point2d& r = rays[static_cast<std::size_t>(j) * W + i];
      const camera::Vec3d ray(r.x, r.y, 1.0);
      const double along = glm::dot(normal, ray);
      const double depth = std::abs(along) > 1e-12 ? offset / along : -1.0;
      float u = -1e6f, v = -1e6f;  // off the texture: left transparent
      if (depth > 0.0) {
        const camera::Vec3d q = to_board * (depth * ray - origin);
        u = static_cast<float>(q.x * s + margin_px - 0.5);
        v = static_cast<float>(q.y * s + margin_px - 0.5);
      }
      map_x.at<float>(j, i) = u;
      map_y.at<float>(j, i) = v;
    }
  }
  const cv::Rect roi(x0, y0, w, h);
  cv::Mat fine;
  cv::resize(image(roi), fine, cv::Size(W, H), 0, 0, cv::INTER_NEAREST);
  cv::remap(texture, fine, map_x, map_y, cv::INTER_LINEAR,
            cv::BORDER_TRANSPARENT);
  cv::Mat coarse;
  cv::resize(fine, coarse, cv::Size(w, h), 0, 0, cv::INTER_AREA);
  coarse.copyTo(image(roi));
  image.convertTo(out, CV_8U);
  return out;
}

}  // namespace volumetric_kit::calib::testing
