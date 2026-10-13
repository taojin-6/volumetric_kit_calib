// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The synthetic source every calib test draws on: its lenses and rig, its plan
// of placements, what the cameras detect of it, and its renderer, whose
// inverse map and orientation the detector's tests will rely on.

#include "synthetic_rig.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/geometry.hpp>

#include "volumetric_kit/calib/compute/cv_camera.hpp"

namespace volumetric_kit::calib::test {
namespace {

namespace camera = recon::camera;

constexpr double kPi = 3.14159265358979323846;

compute::Board femto_board() {
  return compute::Board::create(legacy_femto_board()).value();
}

TEST(SyntheticRig, LensesAreValid) {
  for (const camera::CameraModel& model :
       {color_like_lens(), depth_like_lens()}) {
    const core::Status status = camera::check_camera_model(model);
    EXPECT_TRUE(status.ok()) << status.message();
  }
}

// Pixels through the renderer's inverse map and back through the lens land
// where they started, across the whole image.
TEST(SyntheticRig, InverseMapRoundTripsBelowANanopixel) {
  for (const camera::CameraModel& model :
       {color_like_lens(), depth_like_lens()}) {
    const double w = model.size.width;
    const double h = model.size.height;
    const double step = w / 128.0;
    std::vector<cv::Point2d> pixels;
    for (double v = -0.5; v <= h - 0.5; v += step) {
      for (double u = -0.5; u <= w - 0.5; u += step) pixels.emplace_back(u, v);
      pixels.emplace_back(w - 0.5, v);
    }
    const std::vector<cv::Point2d> rays = undistort_pixels(model, pixels);
    std::vector<cv::Point3d> points;
    for (const cv::Point2d& r : rays) points.emplace_back(r.x, r.y, 1.0);
    const std::vector<cv::Point2d> back =
        project(model, camera::Mat4d(1.0), points);
    double worst = 0.0;
    for (std::size_t i = 0; i < pixels.size(); ++i) {
      worst = std::max(worst, cv::norm(back[i] - pixels[i]));
    }
    EXPECT_LE(worst, 1e-9) << model.size.width << "x" << model.size.height;
  }
}

TEST(SyntheticRig, ProjectsThroughTheLens) {
  const camera::CameraModel model = color_like_lens();
  const compute::CvCamera cam = compute::cv_camera(model).value();
  camera::Mat4d to_camera(1.0);
  to_camera[3] = glm::dvec4(0.1, -0.2, 2.0, 1.0);
  const std::vector<cv::Point3d> points = {{0.0, 0.0, 0.0}, {0.4, 0.3, 0.5}};
  std::vector<cv::Point2d> expected;
  cv::projectPoints(std::vector<cv::Point3d>{{0.1, -0.2, 2.0}, {0.5, 0.1, 2.5}},
                    cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0),
                    cam.camera_matrix, cam.distortion, expected);
  const std::vector<cv::Point2d> got = project(model, to_camera, points);
  ASSERT_EQ(got.size(), 2u);
  for (std::size_t i = 0; i < got.size(); ++i) {
    EXPECT_LE(cv::norm(got[i] - expected[i]), 1e-12) << i;
  }
}

// Four valid cameras, 2.4-2.6 m up around the floor's middle, each seeing it.
TEST(SyntheticRig, RigSurroundsTheFloor) {
  const std::vector<SyntheticCamera> rig = synthetic_rig();
  ASSERT_EQ(rig.size(), 4u);
  std::set<std::string> ids;
  for (const SyntheticCamera& cam : rig) {
    ids.insert(cam.id);
    EXPECT_TRUE(camera::check_camera_model(cam.model).ok()) << cam.id;
    EXPECT_TRUE(camera::check_rigid(cam.world_to_camera).ok()) << cam.id;
    const camera::Vec3d centre(camera::rigid_inverse(cam.world_to_camera)[3]);
    EXPECT_GT(centre.z, 2.4) << cam.id;
    EXPECT_LT(centre.z, 2.6) << cam.id;
    const cv::Point2d middle =
        project(cam.model, cam.world_to_camera, {{0.0, 0.0, 0.0}})[0];
    EXPECT_GT(middle.x, 0.0) << cam.id;
    EXPECT_GT(middle.y, 0.0) << cam.id;
    EXPECT_LT(middle.x, cam.model.size.width - 1.0) << cam.id;
    EXPECT_LT(middle.y, cam.model.size.height - 1.0) << cam.id;
  }
  EXPECT_EQ(ids.size(), rig.size());
}

// Whether `cam` sees the board at `board_to_world` by `options`' rules, at
// the outline points PlanOptions names.
bool sees(const SyntheticCamera& cam, const camera::Mat4d& board_to_world,
          const compute::Board& board, const PlanOptions& options) {
  const camera::Mat4d board_to_camera = cam.world_to_camera * board_to_world;
  const std::vector<cv::Point3d> outline = board_outline(board, 0.0, 16);
  for (const cv::Point3d& p : outline) {
    const camera::Vec3d q(board_to_camera * glm::dvec4(p.x, p.y, p.z, 1.0));
    if (q.z < options.min_depth_m) return false;
  }
  const double w = cam.model.size.width - 1.0;
  const double h = cam.model.size.height - 1.0;
  const double m = options.margin_px;
  for (const cv::Point2d& q : project(cam.model, board_to_camera, outline)) {
    if (q.x < m || q.y < m || q.x > w - m || q.y > h - m) return false;
  }
  const cv::Point3f corner = board.charuco().getRightBottomCorner();
  const camera::Vec3d centre(board_to_camera *
                             glm::dvec4(0.5 * corner.x, 0.5 * corner.y, 0, 1));
  const camera::Vec3d face = -camera::Vec3d(board_to_camera[2]);
  const double tilt =
      std::acos(std::min(1.0, glm::dot(face, glm::normalize(-centre)))) *
      180.0 / kPi;
  return tilt <= options.max_tilt_deg;
}

// Each placement lists exactly the cameras that see it, at least
// `min_cameras` of them.
void expect_plan_keeps_its_rules(const std::vector<SyntheticCamera>& rig,
                                 const compute::Board& board,
                                 const PlanOptions& options) {
  const std::vector<Placement> plan = synthetic_plan(rig, board, options);
  ASSERT_EQ(plan.size(), options.count);
  for (std::size_t i = 0; i < plan.size(); ++i) {
    EXPECT_TRUE(camera::check_rigid(plan[i].board_to_world).ok()) << i;
    EXPECT_GE(plan[i].cameras.size(), options.min_cameras) << i;
    std::vector<std::size_t> seeing;
    for (std::size_t c = 0; c < rig.size(); ++c) {
      if (sees(rig[c], plan[i].board_to_world, board, options)) {
        seeing.push_back(c);
      }
    }
    EXPECT_EQ(plan[i].cameras, seeing) << i;
  }
}

TEST(SyntheticRig, PlanKeepsBoardsTheCamerasSee) {
  const std::vector<SyntheticCamera> rig = synthetic_rig();
  const compute::Board board = femto_board();
  PlanOptions defaults;
  defaults.count = 40;
  expect_plan_keeps_its_rules(rig, board, defaults);
  // Bounds tight enough that each turns boards away.
  PlanOptions tight;
  tight.count = 30;
  tight.min_cameras = 1;
  tight.max_tilt_deg = 45.0;
  tight.margin_px = 300.0;
  tight.min_depth_m = 1.5;
  expect_plan_keeps_its_rules(rig, board, tight);
}

// Floor and stand placements at the plan's heights, half the stands' tilted
// 5-15 degrees toward a camera.
TEST(SyntheticRig, PlanPlacesBoardsAsAPersonDoes) {
  const std::vector<SyntheticCamera> rig = synthetic_rig();
  const compute::Board board = femto_board();
  PlanOptions options;
  options.count = 60;
  const std::vector<Placement> plan = synthetic_plan(rig, board, options);
  ASSERT_EQ(plan.size(), options.count);
  const cv::Point3f corner = board.charuco().getRightBottomCorner();
  std::size_t floor = 0, level = 0, tilted = 0;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    const camera::Mat4d& pose = plan[i].board_to_world;
    const camera::Vec3d centre(
        pose * glm::dvec4(0.5 * corner.x, 0.5 * corner.y, 0.0, 1.0));
    const camera::Vec3d normal = -camera::Vec3d(pose[2]);
    const double tilt = std::acos(std::min(1.0, normal.z)) * 180.0 / kPi;
    if (std::abs(centre.z) < 1e-12) {
      ++floor;
      EXPECT_LT(tilt, 1e-6) << i;
      continue;
    }
    const double stand = std::round(2.0 * centre.z) / 2.0;
    EXPECT_TRUE(stand == 0.5 || stand == 1.0 || stand == 1.5) << i;
    EXPECT_LE(std::abs(centre.z - stand), 0.05 + 1e-12) << i;
    if (tilt < 1e-6) {
      ++level;
      continue;
    }
    ++tilted;
    EXPECT_GE(tilt, 5.0 - 1e-9) << i;
    EXPECT_LE(tilt, 15.0 + 1e-9) << i;
    // Its face leans toward a camera, seen from above.
    const glm::dvec2 lean = glm::normalize(glm::dvec2(normal));
    double best = -1.0;
    for (const SyntheticCamera& cam : rig) {
      const camera::Vec3d at(camera::rigid_inverse(cam.world_to_camera)[3]);
      best = std::max(best, glm::dot(lean, glm::normalize(glm::dvec2(at) -
                                                          glm::dvec2(centre))));
    }
    EXPECT_GT(best, 1.0 - 1e-12) << i;
  }
  EXPECT_GT(floor, 0u);
  EXPECT_GT(level, 0u);
  EXPECT_GT(tilted, 0u);
}

// With no cameras, nothing is tilted, and only a plan that needs none keeps
// a board.
TEST(SyntheticRig, PlanOfNoCameras) {
  const compute::Board board = femto_board();
  PlanOptions options;
  options.count = 5;
  options.min_cameras = 0;
  const std::vector<Placement> plan = synthetic_plan({}, board, options);
  ASSERT_EQ(plan.size(), options.count);
  for (const Placement& placement : plan) {
    EXPECT_TRUE(placement.cameras.empty());
    EXPECT_EQ(placement.board_to_world[2], glm::dvec4(0.0, 0.0, -1.0, 0.0));
  }
  options.min_cameras = 1;
  EXPECT_TRUE(synthetic_plan({}, board, options).empty());
}

TEST(SyntheticRig, PlanIsTheSameForTheSameSeed) {
  const std::vector<SyntheticCamera> rig = synthetic_rig();
  const compute::Board board = femto_board();
  PlanOptions options;
  options.count = 10;
  const std::vector<Placement> a = synthetic_plan(rig, board, options);
  const std::vector<Placement> b = synthetic_plan(rig, board, options);
  options.seed = 2;
  const std::vector<Placement> c = synthetic_plan(rig, board, options);
  ASSERT_EQ(a.size(), b.size());
  ASSERT_FALSE(c.empty());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].board_to_world, b[i].board_to_world) << i;
    EXPECT_EQ(a[i].cameras, b[i].cameras) << i;
  }
  EXPECT_NE(a[0].board_to_world, c[0].board_to_world);
}

// Each placement in each camera that sees it, every corner where it
// projects, plus the noise asked for.
TEST(SyntheticRig, ObservesEachPlacementInEachCameraThatSeesIt) {
  const std::vector<SyntheticCamera> rig = synthetic_rig();
  const compute::Board board = femto_board();
  PlanOptions plan_options;
  plan_options.count = 30;
  const std::vector<Placement> plan = synthetic_plan(rig, board, plan_options);
  ObserveOptions exact;
  exact.noise_px = 0.0;
  const std::vector<Observation> views = observe(rig, board, plan, exact);
  const std::vector<Observation> noisy = observe(rig, board, plan);
  ASSERT_EQ(noisy.size(), views.size());
  std::size_t v = 0;
  double sum = 0.0;
  std::size_t axes = 0;
  for (std::size_t p = 0; p < plan.size(); ++p) {
    for (const std::size_t c : plan[p].cameras) {
      ASSERT_LT(v, views.size());
      const Observation& view = views[v];
      EXPECT_EQ(view.camera_id, rig[c].id);
      EXPECT_EQ(view.frame_id, static_cast<std::int64_t>(p));
      const std::vector<cv::Point2d> pixels =
          project(rig[c].model, rig[c].world_to_camera * plan[p].board_to_world,
                  board.object_points());
      ASSERT_EQ(view.points.size(), 8u);
      for (std::size_t i = 0; i < 8; ++i) {
        EXPECT_EQ(view.points[i].id, static_cast<int>(i));
        EXPECT_EQ(view.points[i].pixel,
                  camera::Vec2d(pixels[i].x, pixels[i].y));
        EXPECT_EQ(view.points[i].weight, 1.0);
        const DetectedPoint& seen = noisy[v].points[i];
        EXPECT_DOUBLE_EQ(seen.weight, 1.0 / (0.3 * 0.3));
        const camera::Vec2d off = seen.pixel - view.points[i].pixel;
        sum += off.x * off.x + off.y * off.y;
        axes += 2;
      }
      ++v;
    }
  }
  EXPECT_EQ(v, views.size());
  EXPECT_NEAR(std::sqrt(sum / axes), 0.3, 0.03);

  const std::vector<Observation> again = observe(rig, board, plan);
  ObserveOptions other;
  other.seed = 2;
  const std::vector<Observation> reseeded = observe(rig, board, plan, other);
  EXPECT_EQ(again[0].points[0].pixel, noisy[0].points[0].pixel);
  EXPECT_NE(reseeded[0].points[0].pixel, noisy[0].points[0].pixel);
}

// A camera on the printed side, looking along the board's +z with its axes on
// the board's, sees the board as OpenCV draws it: not mirrored, the
// top-left square black, at 1000 px per metre.
TEST(SyntheticRig, RendersThePrintedFaceUnmirrored) {
  const compute::Board board = femto_board();
  const camera::CameraModel pinhole{
      {800, 500}, {1000.0, 1000.0, 399.5, 249.5}, {}};
  camera::Mat4d board_to_camera(1.0);
  board_to_camera[3] = glm::dvec4(-0.375, -0.225, 1.0, 1.0);
  const cv::Mat image = render_board(pinhole, board_to_camera, board);
  ASSERT_EQ(image.type(), CV_8UC1);
  ASSERT_EQ(image.size(), cv::Size(800, 500));

  // The board fills pixels [25, 775) x [25, 475).
  cv::Mat drawn, expected;
  board.charuco().generateImage(cv::Size(750, 450), drawn, 0, 1);
  const RenderOptions levels;
  drawn.convertTo(expected, CV_8U, (levels.white - levels.black) / 255.0,
                  levels.black);
  const cv::Mat seen = image(cv::Rect(25, 25, 750, 450));
  cv::Mat mirrored;
  cv::flip(expected, mirrored, 1);
  const double error = cv::norm(seen, expected, cv::NORM_L1) / seen.total();
  const double mirrored_error =
      cv::norm(seen, mirrored, cv::NORM_L1) / seen.total();
  // Only the marker cells' edges, off the pixel grid, differ.
  EXPECT_LT(error, 1.5);
  EXPECT_GT(mirrored_error, 5.0 * error);
  // At (row, column): square (0, 0), off the board and its margin, and
  // square (1, 0) around its marker.
  EXPECT_EQ(image.at<unsigned char>(100, 100), levels.black);
  EXPECT_EQ(image.at<unsigned char>(2, 2), levels.background);
  EXPECT_EQ(image.at<unsigned char>(30, 180), levels.white);
}

// Where the render puts a straight edge that projects from p0 to p1, with
// the dark side toward `inside`, less where it projects (pixels), on each
// image row across its middle 40 % (columns, for an edge nearer horizontal).
// On a row, the dark coverage of a run of pixels from the light side to the
// dark is how far into the run the edge crosses the row's centre line.
std::vector<double> edge_errors(cv::Mat image, cv::Point2d p0, cv::Point2d p1,
                                cv::Point2d inside,
                                const RenderOptions& levels) {
  if (std::abs(p1.y - p0.y) < std::abs(p1.x - p0.x)) {
    cv::transpose(image, image);
    for (cv::Point2d* p : {&p0, &p1, &inside}) *p = {p->y, p->x};
  }
  const auto edge_at = [&](double y) {
    return p0.x + (y - p0.y) * (p1.x - p0.x) / (p1.y - p0.y);
  };
  const bool dark_right = inside.x > edge_at(inside.y);
  const double low = std::min(p0.y, p1.y);
  const double span = std::abs(p1.y - p0.y);
  constexpr int kRun = 6;  // pixels each side, short of the next feature
  std::vector<double> errors;
  for (int y = static_cast<int>(std::ceil(low + 0.3 * span));
       y <= static_cast<int>(std::floor(low + 0.7 * span)); ++y) {
    const double x = edge_at(y);
    const int first = static_cast<int>(std::lround(x)) - kRun;
    const int last = first + 2 * kRun;
    double dark = 0.0;
    for (int i = first; i <= last; ++i) {
      dark += (levels.white - image.at<unsigned char>(y, i)) /
              (levels.white - levels.black);
    }
    errors.push_back((dark_right ? last + 0.5 - dark : first - 0.5 + dark) - x);
  }
  return errors;
}

// Through a pinhole, the board's edges stay straight lines between their
// projected corners: the render puts each of square (0, 0)'s four there,
// off the pixel grid, turned and tilted.
TEST(SyntheticRig, PlacesSquaresWhereTheyProject) {
  const compute::Board board = femto_board();
  const camera::CameraModel pinhole{
      {800, 500}, {1000.0, 1000.0, 399.5, 249.5}, {}};
  const double s = board.target().square_size_m;
  const std::vector<cv::Point3d> square = {
      {0.0, 0.0, 0.0}, {s, 0.0, 0.0}, {s, s, 0.0}, {0.0, s, 0.0}};
  const cv::Point3d middle(0.5 * s, 0.5 * s, 0.0);
  struct View {
    const char* name;
    camera::Vec3d rvec;  // the board's rotation into the camera
  };
  const View views[] = {{"front-on", {0.0, 0.0, 0.0}},
                        {"turned 30 deg", {0.0, 0.0, 30.0}},
                        {"tilted 50 deg", {50.0, 0.0, 0.0}},
                        {"tilted 40 deg and turned", {10.0, -40.0, 20.0}}};
  const RenderOptions levels;
  for (const View& view : views) {
    camera::Mat4d board_to_camera(
        camera::rotation_from_rodrigues(view.rvec * (kPi / 180.0)));
    // The square's middle 1 m away, off the pixel grid by (0.31, 0.17) px.
    const camera::Vec3d at(0.31e-3, 0.17e-3, 1.0);
    board_to_camera[3] =
        glm::dvec4(at - camera::Mat3d(board_to_camera) *
                            camera::Vec3d(middle.x, middle.y, middle.z),
                   1.0);
    const cv::Mat image = render_board(pinhole, board_to_camera, board);
    const std::vector<cv::Point2d> corners =
        project(pinhole, board_to_camera, square);
    const cv::Point2d inside =
        project(pinhole, board_to_camera, {middle}).front();
    for (std::size_t e = 0; e < 4; ++e) {
      const std::vector<double> errors =
          edge_errors(image, corners[e], corners[(e + 1) % 4], inside, levels);
      ASSERT_GE(errors.size(), 20u) << view.name << ", edge " << e;
      double mean = 0.0;
      double worst = 0.0;
      for (const double error : errors) {
        mean += error / errors.size();
        worst = std::max(worst, std::abs(error));
      }
      EXPECT_LT(std::abs(mean), 0.005) << view.name << ", edge " << e;
      EXPECT_LT(worst, 0.02) << view.name << ", edge " << e;
    }
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::test
