// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/solve/lens.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>

#include <opencv2/calib.hpp>
#include <opencv2/geometry.hpp>

#include "common.hpp"
#include "reprojection.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/compute/cv_pose.hpp"
#include "volumetric_kit/calib/compute/guarded.hpp"
#include "volumetric_kit/calib/compute/measures.hpp"
#include "volumetric_kit/calib/compute/usable_for_pose.hpp"

namespace volumetric_kit::calib::solve {
namespace {

namespace camera = sensor::camera;
using detail::Derivatives;
using detail::Pose;
using detail::Projection;
using Use = LensView::Use;

constexpr int kRegionsPerSide = 3;
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// A camera's views in capture order, and what the solve does with each: a
// view is in the fit while it is `Fitted`.
struct Plan {
  std::vector<const Observation*> views;
  std::vector<LensView> uses;
};

// A view fitted or held out: its corners, those kept and its pose.
struct ViewState {
  std::size_t planned = 0;  // its place in the Plan
  std::vector<cv::Point3d> object;
  std::vector<cv::Point2d> pixels;
  std::vector<char> kept;
  Pose pose = Pose::all(0.0);
};

// A fit: the lens, and a pose for each view in it, in the Plan's order.
struct Solved {
  double rms = 0.0;
  compute::CvCamera lens;
  camera::PinholeIntrinsics sigma;
  std::vector<Pose> poses;
  bool from_start = false;
};

core::Status check_options(const LensOptions& o, std::string_view prefix) {
  if (o.min_views < 1) {
    return detail::refuse_option(prefix, "min_views", "at least 1");
  }
  if (o.min_corners < 4) {
    return detail::refuse_option(prefix, "min_corners", "at least 4");
  }
  if (o.holdout_every < 2) {
    return detail::refuse_option(prefix, "holdout_every", "at least 2");
  }
  if (o.outlier_rounds < 0) {
    return detail::refuse_option(prefix, "outlier_rounds", "at least 0");
  }
  return {};
}

cv::Size image_of(camera::ImageSize size) {
  return {static_cast<int>(size.width), static_cast<int>(size.height)};
}

LensReach view_reach(camera::ImageSize size, const Observation& view) {
  LensReach reach{};
  for (const DetectedPoint& point : view.points) {
    const LensReach here = pixel_reach(size, point.pixel);
    for (std::size_t d = 0; d < reach.size(); ++d) {
      reach[d] = std::max(reach[d], here[d]);
    }
  }
  return reach;
}

// The cell of the 3 x 3 grid the view's corners centre in.
std::size_t region_of(camera::ImageSize size, const Observation& view) {
  camera::Vec2d centre(0.0);
  for (const DetectedPoint& point : view.points) centre += point.pixel;
  centre /= static_cast<double>(view.points.size());
  const std::optional<int> cell = compute::grid_cell(
      centre.x, centre.y, static_cast<int>(size.width),
      static_cast<int>(size.height), kRegionsPerSide, kRegionsPerSide);
  return static_cast<std::size_t>(cell.value_or(0));
}

// Within each region, the middle view of every `holdout_every` in capture
// order is held out, unless it is the last fitted view reaching a corner or
// a side. So no capture rhythm puts every held-out view in one place or takes
// a corner's reach from the fit.
void hold_out(Plan& plan, camera::ImageSize size, const LensOptions& options) {
  const std::size_t n = plan.views.size();
  std::vector<LensReach> reach(n);
  std::array<int, std::tuple_size<LensReach>::value> reaching{};
  for (std::size_t v = 0; v < n; ++v) {
    if (plan.uses[v].use != Use::Fitted) continue;
    reach[v] = view_reach(size, *plan.views[v]);
    for (std::size_t d = 0; d < reaching.size(); ++d) {
      if (reach[v][d] >= kLensReach) ++reaching[d];
    }
  }
  std::array<int, kRegionsPerSide * kRegionsPerSide> seen{};
  for (std::size_t v = 0; v < n; ++v) {
    if (plan.uses[v].use != Use::Fitted) continue;
    const int k = seen[region_of(size, *plan.views[v])]++;
    if (k % options.holdout_every != options.holdout_every / 2) continue;
    bool last = false;
    for (std::size_t d = 0; d < reaching.size(); ++d) {
      last = last || (reach[v][d] >= kLensReach && reaching[d] == 1);
    }
    if (last) continue;
    plan.uses[v].use = Use::HeldOut;
    for (std::size_t d = 0; d < reaching.size(); ++d) {
      if (reach[v][d] >= kLensReach) --reaching[d];
    }
  }
}

core::Result<Plan> plan_views(std::string_view camera_id,
                              camera::ImageSize size,
                              const compute::Board& board,
                              const std::vector<Observation>& views,
                              const LensOptions& options,
                              const std::string& prefix) {
  VKC_TRY(check_options(options, prefix + "options."));
  if (camera_id.empty()) {
    return core::Status::invalid_argument(prefix + "the camera has no id");
  }
  constexpr auto kLargest =
      static_cast<std::uint32_t>(std::numeric_limits<int>::max());
  if (size.width == 0 || size.height == 0 || size.width > kLargest ||
      size.height > kLargest) {
    return core::Status::invalid_argument(prefix + "an image of " +
                                          std::to_string(size.width) + " x " +
                                          std::to_string(size.height));
  }
  VKC_TRY(detail::check_one_view_each(views, prefix));
  Plan plan;
  for (const Observation& view : views) {
    if (view.camera_id == camera_id) plan.views.push_back(&view);
  }
  std::sort(plan.views.begin(), plan.views.end(),
            [](const Observation* a, const Observation* b) {
              return a->frame_id < b->frame_id;
            });
  plan.uses.reserve(plan.views.size());
  for (const Observation* view : plan.views) {
    for (const DetectedPoint& point : view->points) {
      if (!std::isfinite(point.pixel.x) || !std::isfinite(point.pixel.y)) {
        return core::Status::invalid_argument(prefix + "capture " +
                                              std::to_string(view->frame_id) +
                                              " has a corner not finite");
      }
    }
    LensView& use = plan.uses.emplace_back();
    use.frame_id = view->frame_id;
    use.corners = static_cast<int>(view->points.size());
    VKC_ASSIGN(const compute::ViewUse usable,
               compute::usable_for_pose(board, *view, image_of(size)));
    use.use =
        use.corners >= options.min_corners && usable == compute::ViewUse::Usable
            ? Use::Fitted
            : Use::TooFew;
  }
  hold_out(plan, size, options);
  return plan;
}

// Each view fitted or held out, in the Plan's order.
std::vector<ViewState> read_views(const compute::Board& board,
                                  const Plan& plan) {
  std::vector<ViewState> states;
  for (std::size_t v = 0; v < plan.views.size(); ++v) {
    if (plan.uses[v].use == Use::TooFew) continue;
    ViewState& state = states.emplace_back();
    state.planned = v;
    for (const DetectedPoint& point : plan.views[v]->points) {
      state.object.push_back(
          board.object_points()[static_cast<std::size_t>(point.id)]);
      state.pixels.emplace_back(point.pixel.x, point.pixel.y);
    }
    state.kept.assign(state.object.size(), 1);
  }
  return states;
}

// calibrateCamera from `start`, or from OpenCV's own without one. Empty, with
// its reason in `why`, when OpenCV refuses or the fit is not finite: another
// start may yet fit. Running out of memory goes on up.
std::optional<Solved> fit_from(const Plan& plan,
                               const std::vector<ViewState>& states,
                               const cv::Size& image,
                               const compute::CvCamera* start,
                               std::string& why) {
  // OpenCV's calibration takes single precision alone: 2e-4 px at 4K.
  std::vector<std::vector<cv::Point3f>> object;
  std::vector<std::vector<cv::Point2f>> pixels;
  for (const ViewState& state : states) {
    if (plan.uses[state.planned].use != Use::Fitted) continue;
    std::vector<cv::Point3f>& o = object.emplace_back();
    std::vector<cv::Point2f>& p = pixels.emplace_back();
    for (std::size_t i = 0; i < state.object.size(); ++i) {
      if (state.kept[i] == 0) continue;
      o.emplace_back(state.object[i]);
      p.emplace_back(state.pixels[i]);
    }
  }
  cv::Mat matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distortion = cv::Mat::zeros(8, 1, CV_64F);
  int flags = cv::CALIB_RATIONAL_MODEL;
  if (start != nullptr) {
    matrix = cv::Mat(start->camera_matrix).clone();
    distortion = cv::Mat(start->distortion).clone();
    flags |= cv::CALIB_USE_INTRINSIC_GUESS;
  }
  // A fisheye's fit creeps for hundreds of iterations before it settles.
  const cv::TermCriteria criteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, 1e-14);
  // OpenCV sizes these in its own, uninstrumented, code: a fresh vector each
  // time, as AddressSanitizer's container checks need.
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  cv::Mat sigma;
  cv::Mat pose_sigma;
  cv::Mat view_errors;
  Solved solved;
  try {
    solved.rms = cv::calibrateCamera(object, pixels, image, matrix, distortion,
                                     rvecs, tvecs, sigma, pose_sigma,
                                     view_errors, flags, criteria);
  } catch (const cv::Exception& e) {
    if (e.code == cv::Error::StsNoMem) throw;
    why = e.what();
    return std::nullopt;
  }
  if (!std::isfinite(solved.rms) || !cv::checkRange(matrix) ||
      !cv::checkRange(distortion) || !(matrix.at<double>(0, 0) > 0.0) ||
      !(matrix.at<double>(1, 1) > 0.0)) {
    why = "a fit not finite";
    return std::nullopt;
  }
  solved.lens.size = image;
  solved.lens.camera_matrix = matrix;
  for (int i = 0; i < 8; ++i) {
    solved.lens.distortion[i] = distortion.at<double>(i);
  }
  solved.sigma = {sigma.at<double>(0), sigma.at<double>(1), sigma.at<double>(2),
                  sigma.at<double>(3)};
  solved.poses.reserve(rvecs.size());
  for (std::size_t v = 0; v < rvecs.size(); ++v) {
    const cv::Mat& r = rvecs[v];
    const cv::Mat& t = tvecs[v];
    solved.poses.emplace_back(r.at<double>(0), r.at<double>(1), r.at<double>(2),
                              t.at<double>(0), t.at<double>(1),
                              t.at<double>(2));
  }
  solved.from_start = start != nullptr;
  return solved;
}

// The lower-RMS fit of the starts, all on the same corners, so their RMS
// compares like with like.
core::Result<Solved> best_fit(const Plan& plan,
                              const std::vector<ViewState>& states,
                              const cv::Size& image,
                              const compute::CvCamera* start) {
  std::string why;
  std::optional<Solved> best = fit_from(plan, states, image, nullptr, why);
  if (start != nullptr) {
    std::optional<Solved> other = fit_from(plan, states, image, start, why);
    if (other && (!best || other->rms < best->rms)) best = std::move(other);
  }
  if (!best) {
    return core::Status::numerical("solve_lens: no start gave a fit: " + why);
  }
  return *std::move(best);
}

// Each of the view's corners through the lens at `pose`.
void project_view(const compute::CvCamera& lens, const ViewState& state,
                  const Pose& pose, Projection& at) {
  detail::project(lens, pose, Pose::all(0.0), state.object, &state.pixels,
                  Derivatives::None, at);
}

// Drops the outlier corners and views of `solved`; true if any. The fit has no
// robust loss, so a bad corner pulls its view's pose and the view's other
// corners with it: a view with such corners loses those, and is judged whole
// once they are gone. A view whose corners kept no longer do goes whole.
core::Result<bool> drop_outliers(std::vector<ViewState>& states,
                                 const Solved& solved,
                                 const compute::Board& board,
                                 const cv::Size& image,
                                 const LensOptions& options, Plan& plan) {
  std::vector<std::size_t> counted;
  std::vector<std::vector<double>> norms;
  std::vector<double> components;
  Projection at;
  for (std::size_t s = 0; s < states.size(); ++s) {
    const ViewState& state = states[s];
    if (plan.uses[state.planned].use != Use::Fitted) continue;
    project_view(solved.lens, state, solved.poses[counted.size()], at);
    counted.push_back(s);
    std::vector<double>& view_norms = norms.emplace_back();
    view_norms.assign(state.object.size(), kNaN);
    for (std::size_t i = 0; i < view_norms.size(); ++i) {
      if (state.kept[i] == 0) continue;
      const cv::Vec2d& e = at.residuals[i];
      components.push_back(e[0]);
      components.push_back(e[1]);
      view_norms[i] = std::sqrt(e.dot(e));
    }
  }
  const detail::OutlierBounds bounds =
      detail::outlier_bounds(norms, std::move(components), {});

  bool any = false;
  for (std::size_t f = 0; f < counted.size(); ++f) {
    ViewState& state = states[counted[f]];
    LensView& use = plan.uses[state.planned];
    use.rms_px = bounds.rms[f];
    bool touched = false;
    for (std::size_t i = 0; i < norms[f].size(); ++i) {
      if (!(norms[f][i] > bounds.corner_px)) continue;
      state.kept[i] = 0;
      touched = true;
    }
    if (!touched) {
      if (bounds.rms[f] > bounds.view_px) {
        use.use = Use::Dropped;
        any = true;
      }
      continue;
    }
    any = true;
    Observation kept;
    const Observation& all = *plan.views[state.planned];
    for (std::size_t i = 0; i < state.kept.size(); ++i) {
      if (state.kept[i] != 0) kept.points.push_back(all.points[i]);
    }
    use.corners = static_cast<int>(kept.points.size());
    VKC_ASSIGN(const compute::ViewUse usable,
               compute::usable_for_pose(board, kept, image));
    if (use.corners < options.min_corners ||
        usable != compute::ViewUse::Usable) {
      use.use = Use::Dropped;
    }
  }
  return any;
}

// A held-out view's pose through the fitted lens: IPPE's two on its rays, each
// refined, the better kept; empty with none.
core::Result<std::optional<Pose>> pose_through(const compute::CvCamera& lens,
                                               const compute::Board& board,
                                               const Observation& view,
                                               const ViewState& state) {
  core::Result<ViewPose> found = view_pose(lens, board, view);
  if (!found.ok()) {
    if (found.status().domain() != core::Status::Code::Numerical) {
      return found.status();
    }
    return std::optional<Pose>{};
  }
  std::vector<PoseCandidate> candidates{found->best};
  if (found->alternative) candidates.push_back(*found->alternative);
  std::optional<Pose> best;
  double least = kInfinity;
  Projection at;
  for (const PoseCandidate& candidate : candidates) {
    VKC_ASSIGN(compute::CvPose pose,
               compute::cv_pose(candidate.board_to_camera));
    cv::solvePnPRefineLM(state.object, state.pixels, lens.camera_matrix,
                         lens.distortion, pose.rvec, pose.tvec);
    const Pose refined(pose.rvec[0], pose.rvec[1], pose.rvec[2], pose.tvec[0],
                       pose.tvec[1], pose.tvec[2]);
    project_view(lens, state, refined, at);
    double sum = 0.0;
    for (const cv::Vec2d& e : at.residuals) sum += e.dot(e);
    if (sum < least) {
      least = sum;
      best = refined;
    }
  }
  return best;
}

// The view's corners through the lens at its pose, into `fit`'s residuals, its
// corners kept and their RMS into `use`: the squared residuals' sum.
double score(const compute::CvCamera& lens, const ViewState& state,
             const Observation& view, LensFit& fit, LensView& use) {
  Projection at;
  project_view(lens, state, state.pose, at);
  double sum = 0.0;
  int count = 0;
  for (std::size_t i = 0; i < state.object.size(); ++i) {
    const cv::Vec2d& e = at.residuals[i];
    CornerResidual& r = fit.residuals.emplace_back();
    r.capture = view.frame_id;
    r.corner = view.points[i].id;
    r.pixel = view.points[i].pixel;
    r.residual =
        at.depths[i] > 0.0 ? camera::Vec2d(e[0], e[1]) : camera::Vec2d(kNaN);
    r.weight = view.points[i].weight;
    r.dropped = state.kept[i] == 0;
    if (r.dropped) continue;
    sum += glm::dot(r.residual, r.residual);
    ++count;
  }
  use.corners = count;
  use.rms_px = std::sqrt(sum / count);
  return sum;
}

}  // namespace

LensReach pixel_reach(camera::ImageSize size,
                      const camera::Vec2d& pixel) noexcept {
  const double a = 0.5 * size.width;   // from the image's centre to a side
  const double b = 0.5 * size.height;  // and to the top
  const double x = pixel.x - (a - 0.5);
  const double y = pixel.y - (b - 0.5);
  const auto along = [&](double sx, double sy) {
    return std::max(0.0, (sx * a * x + sy * b * y) / (a * a + b * b));
  };
  return {along(-1.0, -1.0), along(1.0, -1.0),      along(-1.0, 1.0),
          along(1.0, 1.0),   std::max(0.0, -x / a), std::max(0.0, x / a)};
}

const char* to_string(LensView::Use use) noexcept {
  switch (use) {
    case Use::Fitted:
      return "fitted";
    case Use::HeldOut:
      return "held_out";
    case Use::Dropped:
      return "dropped";
    case Use::TooFew:
      return "too_few";
  }
  return "unknown";
}

core::Result<std::vector<LensView>> plan_lens_views(
    std::string_view camera_id, camera::ImageSize size,
    const compute::Board& board, const std::vector<Observation>& views,
    const LensOptions& options) {
  return compute::guarded(
      "plan_lens_views", [&]() -> core::Result<std::vector<LensView>> {
        VKC_ASSIGN(Plan plan, plan_views(camera_id, size, board, views, options,
                                         "plan_lens_views: "));
        return std::move(plan.uses);
      });
}

core::Result<LensFit> solve_lens(
    std::string_view camera_id, camera::ImageSize size,
    const compute::Board& board, const std::vector<Observation>& views,
    const std::optional<camera::CameraModel>& start,
    const LensOptions& options) {
  return compute::guarded("solve_lens", [&]() -> core::Result<LensFit> {
    const std::string id(camera_id);
    VKC_ASSIGN(Plan plan, plan_views(camera_id, size, board, views, options,
                                     "solve_lens: "));
    const cv::Size image = image_of(size);
    std::optional<compute::CvCamera> given;
    if (start) {
      if (start->size.width != size.width ||
          start->size.height != size.height) {
        return core::Status::invalid_argument(
            "solve_lens: the start is a lens at " +
            std::to_string(start->size.width) + " x " +
            std::to_string(start->size.height) + ", the views " +
            std::to_string(size.width) + " x " + std::to_string(size.height));
      }
      core::Result<compute::CvCamera> lens = compute::cv_camera(*start);
      if (!lens.ok()) return lens.status().with_context("solve_lens: start");
      given = *std::move(lens);
    }

    // The views to fit: enough at first, and after each round.
    const auto check_left = [&]() -> core::Status {
      const auto left = std::count_if(
          plan.uses.begin(), plan.uses.end(),
          [](const LensView& use) { return use.use == Use::Fitted; });
      if (left >= options.min_views) return {};
      return core::Status::invalid_argument(
          "solve_lens: camera " + id + " has " + std::to_string(left) +
          " views to fit, at least " + std::to_string(options.min_views) +
          " needed");
    };
    VKC_TRY(check_left());

    std::vector<ViewState> states = read_views(board, plan);
    const compute::CvCamera* from = given ? &*given : nullptr;
    Solved best;
    for (int round = 0;; ++round) {
      VKC_ASSIGN(best, best_fit(plan, states, image, from));
      if (round == options.outlier_rounds) break;
      VKC_ASSIGN(const bool dropped,
                 drop_outliers(states, best, board, image, options, plan));
      if (!dropped) break;
      VKC_TRY(check_left());
    }

    const cv::Matx33d& k = best.lens.camera_matrix;
    const cv::Vec<double, 8>& d = best.lens.distortion;
    LensFit fit;
    fit.camera.id = id;
    fit.camera.model = {size,
                        {k(0, 0), k(1, 1), k(0, 2), k(1, 2)},
                        {d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]}};
    fit.camera.source = camera::IntrinsicsSource::Calibrated;
    fit.sigma = best.sigma;
    fit.rms_px = best.rms;
    fit.from_start = best.from_start;

    // Every view scored in capture order: those fitted at their fit's poses,
    // those held out each posed on its own.
    double held_sum = 0.0;
    int held_corners = 0;
    int held_views = 0;
    std::size_t next_pose = 0;
    for (ViewState& state : states) {
      const Observation& view = *plan.views[state.planned];
      LensView& use = plan.uses[state.planned];
      if (use.use == Use::Fitted) {
        state.pose = best.poses[next_pose++];
        score(best.lens, state, view, fit, use);
      } else if (use.use == Use::HeldOut) {
        ++held_views;
        VKC_ASSIGN(const std::optional<Pose> pose,
                   pose_through(best.lens, board, view, state));
        if (pose) {
          state.pose = *pose;
          held_sum += score(best.lens, state, view, fit, use);
          held_corners += use.corners;
        } else {
          use.rms_px = kInfinity;
          held_sum = kInfinity;
        }
      }
    }
    if (held_views == 0) {
      fit.holdout_rms_px = kNaN;
    } else {
      fit.holdout_rms_px = std::sqrt(held_sum / std::max(held_corners, 1));
    }
    fit.views = std::move(plan.uses);
    return fit;
  });
}

}  // namespace volumetric_kit::calib::solve
