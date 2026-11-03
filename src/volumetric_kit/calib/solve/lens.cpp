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
#include "volumetric_kit/calib/solve/lens_poses.hpp"

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

// A lens to start a fit from, and whether it came of the caller's: the second
// pass starts from what the first one reached.
struct Start {
  compute::CvCamera lens;
  bool given = false;
};

// A fit: the lens, and a pose for each view in it, in the Plan's order.
struct Solved {
  double rms = 0.0;
  compute::CvCamera lens;
  camera::PinholeIntrinsics sigma;
  std::vector<Pose> poses;
  bool from_start = false;
};

// The fitted views at a fit.
struct Residuals {
  std::vector<std::size_t> fitted;         // their places in the states
  std::vector<std::vector<double>> norms;  // each corner's, NaN if not kept
  std::vector<double> components;          // the kept corners', x and y
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
  if (o.max_iterations < 1) {
    return detail::refuse_option(prefix, "max_iterations", "at least 1");
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
                               const cv::Size& image, const Start* start,
                               int max_iterations, std::string& why) {
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
    matrix = cv::Mat(start->lens.camera_matrix).clone();
    distortion = cv::Mat(start->lens.distortion).clone();
    flags |= cv::CALIB_USE_INTRINSIC_GUESS;
  }
  const cv::TermCriteria criteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, max_iterations, 1e-14);
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
  solved.from_start = start != nullptr && start->given;
  return solved;
}

// The lower-RMS fit of the starts, all on the same corners, so their RMS
// compares like with like.
core::Result<Solved> best_fit(const Plan& plan,
                              const std::vector<ViewState>& states,
                              const cv::Size& image, const Start* start,
                              int max_iterations) {
  std::string why;
  std::optional<Solved> best =
      fit_from(plan, states, image, nullptr, max_iterations, why);
  if (start != nullptr) {
    std::optional<Solved> other =
        fit_from(plan, states, image, start, max_iterations, why);
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

Residuals fitted_residuals(const Plan& plan,
                           const std::vector<ViewState>& states,
                           const Solved& solved) {
  Residuals out;
  Projection at;
  for (std::size_t s = 0; s < states.size(); ++s) {
    const ViewState& state = states[s];
    if (plan.uses[state.planned].use != Use::Fitted) continue;
    project_view(solved.lens, state, solved.poses[out.fitted.size()], at);
    out.fitted.push_back(s);
    std::vector<double>& view_norms = out.norms.emplace_back();
    view_norms.assign(state.object.size(), kNaN);
    for (std::size_t i = 0; i < view_norms.size(); ++i) {
      if (state.kept[i] == 0) continue;
      const cv::Vec2d& e = at.residuals[i];
      out.components.push_back(e[0]);
      out.components.push_back(e[1]);
      view_norms[i] = std::sqrt(e.dot(e));
    }
  }
  return out;
}

// The view's corners kept.
Observation kept_corners(const Observation& view, const ViewState& state) {
  Observation kept;
  for (std::size_t i = 0; i < state.kept.size(); ++i) {
    if (state.kept[i] != 0) kept.points.push_back(view.points[i]);
  }
  return kept;
}

// Whether `kept` still has corners enough to count and to fix a pose.
core::Result<bool> enough_corners(const Observation& kept,
                                  const compute::Board& board,
                                  const cv::Size& image,
                                  const LensOptions& options) {
  VKC_ASSIGN(const compute::ViewUse usable,
             compute::usable_for_pose(board, kept, image));
  return static_cast<int>(kept.points.size()) >= options.min_corners &&
         usable == compute::ViewUse::Usable;
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
  Residuals residuals = fitted_residuals(plan, states, solved);
  const detail::OutlierBounds bounds = detail::outlier_bounds(
      residuals.norms, std::move(residuals.components), {});

  bool any = false;
  for (std::size_t f = 0; f < residuals.fitted.size(); ++f) {
    ViewState& state = states[residuals.fitted[f]];
    LensView& use = plan.uses[state.planned];
    use.rms_px = bounds.rms[f];
    bool touched = false;
    for (std::size_t i = 0; i < residuals.norms[f].size(); ++i) {
      if (!(residuals.norms[f][i] > bounds.corner_px)) continue;
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
    const Observation kept = kept_corners(*plan.views[state.planned], state);
    use.corners = static_cast<int>(kept.points.size());
    VKC_ASSIGN(const bool enough, enough_corners(kept, board, image, options));
    if (!enough) use.use = Use::Dropped;
  }
  return any;
}

// A held-out view's pose through the fitted lens, on its corners kept: IPPE's
// two on their rays, each refined, the better kept; empty with none.
core::Result<std::optional<Pose>> pose_through(const compute::CvCamera& lens,
                                               const compute::Board& board,
                                               const Observation& view,
                                               const ViewState& state) {
  const Observation kept = kept_corners(view, state);
  core::Result<ViewPose> found = view_pose(lens, board, kept);
  if (!found.ok()) {
    if (found.status().domain() != core::Status::Code::Numerical) {
      return found.status();
    }
    return std::optional<Pose>{};
  }
  std::vector<cv::Point3d> object;
  std::vector<cv::Point2d> pixels;
  for (std::size_t i = 0; i < state.kept.size(); ++i) {
    if (state.kept[i] == 0) continue;
    object.push_back(state.object[i]);
    pixels.push_back(state.pixels[i]);
  }
  std::vector<PoseCandidate> candidates{found->best};
  if (found->alternative) candidates.push_back(*found->alternative);
  std::optional<Pose> best;
  double least = kInfinity;
  Projection at;
  for (const PoseCandidate& candidate : candidates) {
    VKC_ASSIGN(compute::CvPose pose,
               compute::cv_pose(candidate.board_to_camera));
    cv::solvePnPRefineLM(object, pixels, lens.camera_matrix, lens.distortion,
                         pose.rvec, pose.tvec);
    const Pose refined(pose.rvec[0], pose.rvec[1], pose.rvec[2], pose.tvec[0],
                       pose.tvec[1], pose.tvec[2]);
    detail::project(lens, refined, Pose::all(0.0), object, &pixels,
                    Derivatives::None, at);
    double sum = 0.0;
    for (const cv::Vec2d& e : at.residuals) sum += e.dot(e);
    if (sum < least) {
      least = sum;
      best = refined;
    }
  }
  return best;
}

// A held-out view as the fit's are treated: posed, its corners past `bound`
// marked, and posed again on the rest, up to `outlier_rounds` times, so a bad
// corner's pull on the pose costs its neighbours nothing. Empty when no pose
// is left.
core::Result<std::optional<Pose>> judge_held_out(
    const compute::CvCamera& lens, const compute::Board& board,
    const Observation& view, const cv::Size& image, double bound,
    const LensOptions& options, ViewState& state) {
  Projection at;
  for (int round = 0;; ++round) {
    VKC_ASSIGN(const std::optional<Pose> pose,
               pose_through(lens, board, view, state));
    if (!pose || round == options.outlier_rounds) return pose;
    project_view(lens, state, *pose, at);
    bool moved = false;
    for (std::size_t i = 0; i < state.kept.size(); ++i) {
      const cv::Vec2d& e = at.residuals[i];
      const char keep =
          at.depths[i] > 0.0 && std::sqrt(e.dot(e)) <= bound ? 1 : 0;
      moved = moved || keep != state.kept[i];
      state.kept[i] = keep;
    }
    if (!moved) return pose;
    VKC_ASSIGN(const bool enough, enough_corners(kept_corners(view, state),
                                                 board, image, options));
    if (!enough) return std::optional<Pose>{};
  }
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
    std::optional<Start> given;
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
      given = Start{*std::move(lens), true};
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
    const std::vector<ViewState> all_states = states;
    const std::vector<LensView> planned = plan.uses;
    // The rounds: fit, drop and fit again. A first fit can be wrong, and what
    // it drops then was never bad, so when they dropped anything they run once
    // more with everything back, from the lens they reached. One loop, so the
    // second pass makes the first's calls.
    std::optional<Start> from = given;
    Solved best;
    for (int pass = 0; pass < 2; ++pass) {
      bool dropped = false;
      for (int round = 0;; ++round) {
        VKC_ASSIGN(best, best_fit(plan, states, image, from ? &*from : nullptr,
                                  options.max_iterations));
        if (round == options.outlier_rounds) break;
        VKC_ASSIGN(const bool any,
                   drop_outliers(states, best, board, image, options, plan));
        if (!any) break;
        dropped = true;
        VKC_TRY(check_left());
      }
      if (!dropped || pass == 1) break;
      from = Start{best.lens, best.from_start};
      states = all_states;
      plan.uses = planned;
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
    // those held out each posed on its own, by the corner bound of this fit.
    Residuals residuals = fitted_residuals(plan, states, best);
    const double bound =
        detail::outlier_bounds(residuals.norms, std::move(residuals.components),
                               {})
            .corner_px;
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
                   judge_held_out(best.lens, board, view, image, bound, options,
                                  state));
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

core::Result<PosedLensViews> pose_lens_views(
    std::string_view camera_id, const camera::CameraModel& lens,
    const compute::Board& board, const std::vector<Observation>& views,
    const LensOptions& options) {
  return compute::guarded(
      "pose_lens_views", [&]() -> core::Result<PosedLensViews> {
        VKC_ASSIGN(Plan plan, plan_views(camera_id, lens.size, board, views,
                                         options, "pose_lens_views: "));
        core::Result<compute::CvCamera> held = compute::cv_camera(lens);
        if (!held.ok()) {
          return held.status().with_context("pose_lens_views: lens");
        }
        for (LensView& use : plan.uses) {
          if (use.use != Use::TooFew) use.use = Use::HeldOut;
        }
        const cv::Size image = image_of(lens.size);
        std::vector<ViewState> states = read_views(board, plan);

        // The bound, as solve_lens's over the fitted views: the adjustment's
        // rule over each view's residuals at its pose on all its corners.
        std::vector<double> components;
        Projection at;
        for (const ViewState& state : states) {
          VKC_ASSIGN(
              const std::optional<Pose> pose,
              pose_through(*held, board, *plan.views[state.planned], state));
          if (!pose) continue;
          project_view(*held, state, *pose, at);
          for (const cv::Vec2d& e : at.residuals) {
            components.push_back(e[0]);
            components.push_back(e[1]);
          }
        }
        PosedLensViews out;
        if (!components.empty()) {
          const BundleAdjustOptions rule;
          out.bound_px = detail::outlier_bound(
              std::move(components), rule.outlier_px, rule.outlier_mads);
        }

        // Each view as a held-out one; score() adds its corners to `scored`.
        LensFit scored;
        for (ViewState& state : states) {
          const Observation& view = *plan.views[state.planned];
          LensView& use = plan.uses[state.planned];
          VKC_ASSIGN(const std::optional<Pose> pose,
                     judge_held_out(*held, board, view, image, out.bound_px,
                                    options, state));
          if (pose) {
            state.pose = *pose;
            score(*held, state, view, scored, use);
            continue;
          }
          use.use = Use::Dropped;
          use.rms_px = kInfinity;
          for (const DetectedPoint& point : view.points) {
            CornerResidual& r = scored.residuals.emplace_back();
            r.capture = view.frame_id;
            r.corner = point.id;
            r.pixel = point.pixel;
            r.residual = camera::Vec2d(kNaN);
            r.weight = point.weight;
            r.dropped = true;
          }
        }
        out.views = std::move(plan.uses);
        out.residuals = std::move(scored.residuals);
        return out;
      });
}

}  // namespace volumetric_kit::calib::solve
