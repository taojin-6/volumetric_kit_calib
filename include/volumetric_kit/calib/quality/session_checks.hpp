// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file session_checks.hpp
/// @brief What the captures kept so far add up to, before any solve: views
///        and image coverage per camera, which cameras share captures, the
///        tree the solve would place them by, whether Calibrate may run, and
///        the warnings that call for other placements.

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/compute/cv_camera.hpp"
#include "volumetric_kit/calib/core/detection.hpp"
#include "volumetric_kit/calib/quality/export.hpp"
#include "volumetric_kit/calib/quality/live_checks.hpp"
#include "volumetric_kit/calib/solve/initialise.hpp"
#include "volumetric_kit/calib/solve/spanning_tree.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"

namespace volumetric_kit::calib::quality {

/// The coverage grid's columns over each camera's image.
inline constexpr int kCoverageCols = 8;
/// The coverage grid's rows.
inline constexpr int kCoverageRows = 6;

/// @brief The thresholds of the session's checks.
///
/// @code
/// SessionOptions options;
/// options.ready_from = 5;  // Calibrate once every link shares 5 captures
/// @endcode
struct SessionOptions {
  ViewCheckOptions view;  ///< How each captured view is checked.
  /// A capture repeats an earlier one when every camera that sees its board
  /// saw the earlier one's within this (metres, the board's centre) ...
  double duplicate_m = 0.02;
  double duplicate_deg = 2.0;  ///< ... and this (degrees).
  /// A camera is warned floor-only when this share of its views' board
  /// normals or more lie within `floor_deg` of one of them ...
  double floor_share = 0.8;
  double floor_deg = 10.0;  ///< ... (degrees).
  /// A camera is warned mostly frontal when more than this share of its
  /// views are frontal (@ref CheckedView::frontal).
  double frontal_share = 0.5;
  int fair_from = 5;     ///< A tree link sharing fewer captures is weak.
  int strong_from = 10;  ///< A tree link sharing this many is strong.
  /// Calibrate may run once every tree link shares this many captures.
  int ready_from = 3;
};

/// @brief How strong a link of the tree preview is.
enum class LinkStrength {
  Weak,    ///< Fewer than `fair_from` shared captures (red).
  Fair,    ///< From `fair_from` to below `strong_from` (amber).
  Strong,  ///< `strong_from` or more (green).
};

/// @brief How the tree preview places a camera.
///
/// @code
/// if (cam.link && cam.link->strength == LinkStrength::Weak) {
///   warn(ids[i] + " shares too few captures with " + ids[cam.link->parent]);
/// }
/// @endcode
struct PreviewLink {
  std::size_t parent = 0;  ///< The camera it is placed from.
  int shared = 0;          ///< The captures the two share.
  LinkStrength strength = LinkStrength::Weak;  ///< How strong the link is.
};

/// @brief One camera's part of the session.
///
/// @code
/// const CameraSummary& cam = summary.cameras[i];
/// show_heat_map(cam.coverage);  // kCoverageCols x kCoverageRows, row-major
/// if (cam.mostly_frontal) warn("tilt the board more for " + ids[i]);
/// @endcode
struct CameraSummary {
  int usable_views = 0;   ///< Kept captures with a usable view in it.
  int frontal_views = 0;  ///< Of those, the frontal ones.
  /// Corners of its usable views in each cell of a `kCoverageCols` x
  /// `kCoverageRows` grid over its image, row-major from the top left.
  std::array<int, kCoverageCols * kCoverageRows> coverage{};
  double coverage_filled = 0.0;  ///< The share of cells with a corner.
  bool linked = false;           ///< In the reference's tree; the reference is.
  /// Its link in the tree preview: empty for the reference and an unlinked
  /// camera.
  std::optional<PreviewLink> link;
  bool floor_only = false;      ///< Its views' boards nearly all face alike.
  bool mostly_frontal = false;  ///< Most of its views are frontal.
};

/// @brief A capture that repeats an earlier one (@ref
///        SessionOptions::duplicate_m).
struct Duplicate {
  int capture = 0;  ///< The capture's index.
  int earlier = 0;  ///< The first earlier capture it repeats.
};

/// @brief What the captures kept add up to.
///
/// @code
/// VKC_ASSIGN(const SessionSummary summary, checks.summary());
/// calibrate_button.enabled = summary.ready;
/// @endcode
struct SessionSummary {
  int kept = 0;     ///< Captures kept.
  int dropped = 0;  ///< Captures dropped.
  /// One per camera, in the rig's order.
  std::vector<CameraSummary> cameras;
  /// The kept captures each pair of cameras shares with a usable view in
  /// both (@ref solve::covisibility).
  solve::Covisibility covisibility;
  std::vector<Duplicate> duplicates;  ///< In the order of the captures.
  /// Calibrate may run: the rig has two cameras or more, each linked to the
  /// reference through links of `ready_from` shared captures or more.
  bool ready = false;
};

/// @brief The session's checks: the captures kept and dropped, each
///        camera's views checked with @ref check_view.
///
/// The tree preview is @ref solve::spanning_tree over the cameras the
/// reference reaches through shared captures; the others are unlinked. A
/// value; not thread-safe.
///
/// @code
/// VKC_ASSIGN(SessionChecks checks,
///            SessionChecks::create(cameras, board, primary));
/// VKC_TRY(checks.add(done.index, full_detections));  // once it is written
/// VKC_TRY(checks.drop(dropped.index));
/// VKC_ASSIGN(const SessionSummary summary, checks.summary());
/// @endcode
class VC_QUALITY_API SessionChecks {
 public:
  /// @brief The checks of an empty session.
  /// @param cameras    The rig's colour cameras, in the order every capture
  ///                   gives them: at least one, ids unique and not empty.
  /// @param board      The board.
  /// @param reference  The camera the world is, by index in @p cameras.
  /// @param options    The thresholds.
  /// @return The checks; `InvalidArgument` naming what the arguments get
  ///         wrong, a camera model @ref compute::cv_camera refuses included;
  ///         or `OutOfMemory`.
  static core::Result<SessionChecks> create(
      const std::vector<solve::RigCamera>& cameras, const compute::Board& board,
      std::size_t reference, const SessionOptions& options = {});

  /// @brief Keep a capture: each camera's view checked, as the capture
  ///        stores it.
  /// @param index       The capture's index; not one added before.
  /// @param detections  Each camera's full detection, in the rig's order.
  /// @return OK; `InvalidArgument` for an index added before or negative,
  ///         one detection too many or too few, or a detection
  ///         @ref check_view refuses (naming the camera); or its `Backend`
  ///         error or `OutOfMemory`. The session is then unchanged.
  core::Status add(int index, const std::vector<Detection>& detections);

  /// @brief Drop a capture: it stays listed, but counts no more.
  /// @param index  The capture's index.
  /// @return OK; `NotFound` for an index not added; `InvalidArgument` for a
  ///         capture dropped already; or `OutOfMemory`.
  core::Status drop(int index);

  /// @brief What the captures kept add up to.
  /// @return The summary; or `OutOfMemory`.
  core::Result<SessionSummary> summary() const;

 private:
  // One camera's view of a capture, as the checks need it.
  struct View {
    bool usable = false;
    bool frontal = false;
    recon::camera::Mat4d board_to_camera{1.0};
    std::vector<recon::camera::Vec2d> corners;  // usable views only
  };
  struct Capture {
    int index = 0;
    bool dropped = false;
    std::vector<View> views;  // one per camera
  };

  SessionChecks(std::vector<std::string> ids,
                std::vector<compute::CvCamera> cameras, compute::Board board,
                std::size_t reference, const SessionOptions& options);

  std::vector<std::string> ids_;
  std::vector<compute::CvCamera> cameras_;
  compute::Board board_;
  std::size_t reference_ = 0;
  SessionOptions options_;
  std::vector<Capture> captures_;
};

}  // namespace volumetric_kit::calib::quality
