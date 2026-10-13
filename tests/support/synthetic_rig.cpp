// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "synthetic_rig.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <utility>

#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>

#include "volumetric_kit/calib/compute/cv_camera.hpp"

namespace volumetric_kit::calib::test {
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

// An 8-bit texture's integral over any box, exact for its texels as squares
// of constant value: texel (i, j) covers [i, i + 1) x [j, j + 1).
class SummedTexture {
 public:
  // What a box holds of the texture: the integral of its values, and the
  // area of the box that lies on it.
  struct Box {
    double sum;
    double area;
  };

  explicit SummedTexture(const cv::Mat& texture)
      : width_(texture.cols), height_(texture.rows) {
    cv::integral(texture, sums_, CV_64F);
  }

  // The texture over [u0, u1] x [v0, v1] (u0 <= u1, v0 <= v1).
  Box box(double u0, double u1, double v0, double v1) const {
    u0 = std::clamp(u0, 0.0, static_cast<double>(width_));
    u1 = std::clamp(u1, 0.0, static_cast<double>(width_));
    v0 = std::clamp(v0, 0.0, static_cast<double>(height_));
    v1 = std::clamp(v1, 0.0, static_cast<double>(height_));
    return {integral(u1, v1) - integral(u0, v1) - integral(u1, v0) +
                integral(u0, v0),
            (u1 - u0) * (v1 - v0)};
  }

 private:
  // The integral over [0, u] x [0, v], inside the texture: bilinear between
  // the table's entries, which is exact within a texel.
  double integral(double u, double v) const {
    const int i = std::min(static_cast<int>(u), width_ - 1);
    const int j = std::min(static_cast<int>(v), height_ - 1);
    const double a = u - i;
    const double b = v - j;
    const double* top = sums_.ptr<double>(j);
    const double* bottom = sums_.ptr<double>(j + 1);
    return (1.0 - b) * ((1.0 - a) * top[i] + a * top[i + 1]) +
           b * ((1.0 - a) * bottom[i] + a * bottom[i + 1]);
  }

  int width_;
  int height_;
  cv::Mat sums_;  // CV_64F, (height + 1) x (width + 1)
};

// Whether `cam` sees the whole board, whose outer edge is `edge`, at
// `board_to_world`.
bool sees(const SyntheticCamera& cam, const camera::Mat4d& board_to_world,
          const cv::Point2d& extent, const std::vector<cv::Point3d>& edge,
          const PlanOptions& options) {
  const camera::Mat4d board_to_camera = cam.world_to_camera * board_to_world;
  const camera::Vec3d face = -camera::Vec3d(board_to_camera[2]);
  const camera::Vec3d centre =
      apply(board_to_camera, {0.5 * extent.x, 0.5 * extent.y, 0.0});
  if (glm::dot(face, glm::normalize(-centre)) <
      std::cos(options.max_tilt_deg * kDeg)) {
    return false;
  }
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

camera::CameraModel strongly_distorted_lens() {
  return {{1280, 720},
          {800.0, 800.0, 639.5, 359.5},
          {0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
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

std::vector<cv::Point3d> board_outline(const compute::Board& board,
                                       double margin_m, int per_side) {
  const cv::Point2d extent = board_extent(board);
  const double w = extent.x + 2.0 * margin_m;
  const double h = extent.y + 2.0 * margin_m;
  std::vector<cv::Point3d> points;
  for (int i = 0; i < per_side; ++i) {
    const double s = static_cast<double>(i) / per_side;
    points.emplace_back(-margin_m + s * w, -margin_m, 0.0);
    points.emplace_back(-margin_m + w, -margin_m + s * h, 0.0);
    points.emplace_back(-margin_m + w - s * w, -margin_m + h, 0.0);
    points.emplace_back(-margin_m, -margin_m + h - s * h, 0.0);
  }
  return points;
}

std::vector<Placement> synthetic_plan(const std::vector<SyntheticCamera>& rig,
                                      const compute::Board& board,
                                      const PlanOptions& options) {
  std::mt19937 rng(options.seed);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  const cv::Point2d extent = board_extent(board);
  const std::vector<cv::Point3d> edge = board_outline(board, 0.0, 16);
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
    if (!floor && !rig.empty() && uniform(rng) < 0.5) {
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
      if (sees(rig[c], placement.board_to_world, extent, edge, options)) {
        placement.cameras.push_back(c);
      }
    }
    if (placement.cameras.size() >= options.min_cameras) {
      plan.push_back(std::move(placement));
    }
  }
  return plan;
}

std::vector<Observation> observe(const std::vector<SyntheticCamera>& rig,
                                 const compute::Board& board,
                                 const std::vector<Placement>& plan,
                                 const ObserveOptions& options) {
  std::mt19937 rng(options.seed);
  std::normal_distribution<double> noise(0.0, 1.0);
  const std::vector<cv::Point3d>& corners = board.object_points();
  const double weight = options.noise_px > 0.0
                            ? 1.0 / (options.noise_px * options.noise_px)
                            : 1.0;
  std::vector<Observation> views;
  for (std::size_t p = 0; p < plan.size(); ++p) {
    for (const std::size_t c : plan[p].cameras) {
      const SyntheticCamera& cam = rig[c];
      const std::vector<cv::Point2d> pixels = project(
          cam.model, cam.world_to_camera * plan[p].board_to_world, corners);
      Observation view;
      view.camera_id = cam.id;
      view.frame_id = static_cast<std::int64_t>(p);
      for (std::size_t i = 0; i < pixels.size(); ++i) {
        const double dx = options.noise_px * noise(rng);
        const double dy = options.noise_px * noise(rng);
        view.points.push_back({static_cast<int>(i),
                               {pixels[i].x + dx, pixels[i].y + dy},
                               weight});
      }
      views.push_back(std::move(view));
    }
  }
  return views;
}

cv::Mat render_board(const camera::CameraModel& model,
                     const camera::Mat4d& board_to_camera,
                     const compute::Board& board,
                     const RenderOptions& options) {
  const compute::CvCamera cam = cv_lens(model);
  cv::Mat image(cam.size, CV_32F, cv::Scalar(options.background));

  // The texture: the board and a white margin around it, 0-255, summed so a
  // box's mean costs four lookups.
  const cv::Point2d extent = board_extent(board);
  const double s = options.texture_px_per_m;
  const int margin_px = static_cast<int>(std::lround(options.margin_m * s));
  cv::Mat drawn;
  board.charuco().generateImage(
      cv::Size(static_cast<int>(std::lround(extent.x * s)),
               static_cast<int>(std::lround(extent.y * s))),
      drawn, 0, 1);
  cv::Mat padded;
  cv::copyMakeBorder(drawn, padded, margin_px, margin_px, margin_px, margin_px,
                     cv::BORDER_CONSTANT, cv::Scalar(255));
  const SummedTexture texture(padded);

  // The pixels the board and its margin can reach.
  double min_x = 1e300, min_y = 1e300, max_x = -1e300, max_y = -1e300;
  for (const cv::Point2d& q :
       project(model, board_to_camera,
               board_outline(board, options.margin_m, 40))) {
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

  // S x S samples per pixel; each one's ray meets the board's plane at
  // (u, v), in texels, or misses it (NaN).
  const int S = options.supersampling;
  const int w = x1 - x0 + 1;
  const int h = y1 - y0 + 1;
  const int W = w * S;
  const int H = h * S;
  const auto at = [W](int i, int j) {
    return static_cast<std::size_t>(j) * W + i;
  };
  std::vector<cv::Point2d> samples(static_cast<std::size_t>(W) * H);
  for (int j = 0; j < H; ++j) {
    for (int i = 0; i < W; ++i) {
      samples[at(i, j)] = {x0 + (i + 0.5) / S - 0.5, y0 + (j + 0.5) / S - 0.5};
    }
  }
  const std::vector<cv::Point2d> rays =
      compute::undistort_points(cam, samples).value();
  const camera::Mat3d rotation(board_to_camera);
  const camera::Vec3d normal = rotation[2];
  const camera::Vec3d origin(board_to_camera[3]);
  const camera::Mat3d to_board = glm::transpose(rotation);
  const double offset = glm::dot(normal, origin);
  constexpr double kMiss = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> u(samples.size(), kMiss), v(samples.size(), kMiss);
  for (std::size_t k = 0; k < rays.size(); ++k) {
    const camera::Vec3d ray(rays[k].x, rays[k].y, 1.0);
    const double along = glm::dot(normal, ray);
    const double depth = std::abs(along) > 1e-12 ? offset / along : -1.0;
    if (depth > 0.0) {
      const camera::Vec3d q = to_board * (depth * ray - origin);
      u[k] = q.x * s + margin_px;
      v[k] = q.y * s + margin_px;
    }
  }

  // Each sample is the board's mean over the bounding box of its footprint:
  // the 1/S-pixel square around it, as the steps to its neighbouring samples
  // map it onto the board.
  const auto half_step = [&](const std::vector<double>& c, int i, int j, int di,
                             int dj) {
    const auto value = [&](int ii, int jj) {
      return ii < 0 || jj < 0 || ii >= W || jj >= H ? kMiss : c[at(ii, jj)];
    };
    const double before = value(i - di, j - dj);
    const double after = value(i + di, j + dj);
    const double here = c[at(i, j)];
    if (std::isfinite(before) && std::isfinite(after)) {
      return 0.25 * std::abs(after - before);
    }
    if (std::isfinite(after)) return 0.5 * std::abs(after - here);
    if (std::isfinite(before)) return 0.5 * std::abs(here - before);
    return 0.0;
  };
  const double scale = (options.white - options.black) / 255.0;
  cv::Mat fine(H, W, CV_32F);
  for (int j = 0; j < H; ++j) {
    for (int i = 0; i < W; ++i) {
      const std::size_t k = at(i, j);
      float& level = fine.at<float>(j, i);
      if (!std::isfinite(u[k])) {
        level = options.background;
        continue;
      }
      // At least a tenth of a texel across, which keeps the box's sums
      // well above the table's rounding.
      const double hu =
          std::max(0.05, half_step(u, i, j, 1, 0) + half_step(u, i, j, 0, 1));
      const double hv =
          std::max(0.05, half_step(v, i, j, 1, 0) + half_step(v, i, j, 0, 1));
      const SummedTexture::Box box =
          texture.box(u[k] - hu, u[k] + hu, v[k] - hv, v[k] + hv);
      const double area = 4.0 * hu * hv;
      level = static_cast<float>((options.black * box.area + scale * box.sum +
                                  options.background * (area - box.area)) /
                                 area);
    }
  }
  const cv::Rect roi(x0, y0, w, h);
  cv::Mat coarse;
  cv::resize(fine, coarse, cv::Size(w, h), 0, 0, cv::INTER_AREA);
  coarse.copyTo(image(roi));
  image.convertTo(out, CV_8U);
  return out;
}

compute::Board femto_board() {
  return compute::Board::create(legacy_femto_board()).value();
}

camera::Mat4d board_facing_camera(const compute::Board& board,
                                  const camera::Vec3d& centre, double tilt_deg,
                                  double roll_deg) {
  const camera::Mat3d rotation =
      camera::rotation_from_rodrigues({0.0, 0.0, roll_deg * kPi / 180.0}) *
      camera::rotation_from_rodrigues({tilt_deg * kPi / 180.0, 0.0, 0.0});
  const cv::Point3f corner = board.charuco().getRightBottomCorner();
  const camera::Vec3d middle(0.5 * corner.x, 0.5 * corner.y, 0.0);
  camera::Mat4d m(rotation);
  m[3] = glm::dvec4(centre - rotation * middle, 1.0);
  return m;
}

cv::Mat camera_image(const cv::Mat& render, const CameraOptions& options) {
  cv::Mat blurred;
  if (options.blur_px > 0.0) {
    cv::GaussianBlur(render, blurred, cv::Size(), options.blur_px);
  } else {
    blurred = render.clone();
  }
  if (options.noise_dn <= 0.0) return blurred;
  cv::Mat noise(render.size(), CV_32F);
  cv::RNG(options.seed).fill(noise, cv::RNG::NORMAL, 0.0, options.noise_dn);
  cv::Mat level;
  blurred.convertTo(level, CV_32F);
  cv::Mat out;
  cv::Mat(level + noise).convertTo(out, CV_8U);
  return out;
}

}  // namespace volumetric_kit::calib::test
