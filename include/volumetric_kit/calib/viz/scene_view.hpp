// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file scene_view.hpp
/// @brief The 3D view of the rig: each camera's points from its prepared
///        depth, its frusta, the world's axes, the solved boards and the
///        origin tag, through an orbiting view turned so the rig stands
///        upright.
///
/// The rig's world is the origin tag's, +z up from the floor, or its
/// reference camera's colour frame: +x right, +y down, +z forward. gfx's
/// `OrbitCamera` keeps +Y up, so the view turns the world by a display
/// rotation (@ref display_rotation) that takes the world's up to +Y: the
/// tag's +z, or the cameras' mean up, their -y. Points are gfx's draws from
/// recon's prepared frames in place (@ref cloud_draws); lines are a few KB of
/// `ColorVertex` made on the host from the cameras' models and poses
/// (@ref scene_lines), uploaded only when they change.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <glm/mat4x4.hpp>

#include "volumetric_kit/calib/compute/board.hpp"
#include "volumetric_kit/calib/viz/export.hpp"
#include "volumetric_kit/calib/viz/point_cloud.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/camera/orbit_camera.hpp"
#include "volumetric_kit/gfx/pipelines/depth_cloud_pipeline.hpp"
#include "volumetric_kit/gfx/pipelines/primitive_pipeline.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"

namespace volumetric_kit::calib::viz {

/// @brief One camera of the scene: its factory models, where it is, and how
///        the view shows it.
///
/// @code
/// SceneCamera camera;
/// camera.color = *info.color;
/// camera.depth = info.depth;
/// camera.depth_to_color = info.depth_to_color;
/// camera.color_to_world = recon::camera::Mat4d(1.0);  // the reference
/// @endcode
struct SceneCamera {
  /// The colour camera's model, lens included.
  recon::camera::CameraModel color;
  /// The depth camera's model, lens included; empty for none.
  std::optional<recon::camera::CameraModel> depth;
  /// The depth camera's frame to the colour camera's: the factory extrinsic.
  recon::camera::Mat4d depth_to_color{1.0};
  /// The colour camera's pose in the rig's world; empty while the camera is
  /// not placed (before the rig is solved, every camera but the reference),
  /// which the view then leaves out.
  std::optional<recon::camera::Mat4d> color_to_world;
  /// Its tint, an sRGB-encoded `pack_rgba8` word: its frustum's colour, and
  /// its points' in `Solid` and `TintedLuma`.
  std::uint32_t tint = gfx::pipelines::pack_rgba8(255, 255, 255);
  bool shown = true;  ///< Whether the view shows its points and frusta.
};

/// @brief What @ref scene_lines draws besides the cameras' colour frusta.
///
/// @code
/// SceneOptions options;
/// options.depth_frusta = true;
/// @endcode
struct SceneOptions {
  /// How far each frustum reaches along its camera's +z (metres); positive.
  double frustum_m = 0.3;
  /// Each camera's depth frustum beside its colour one, darker.
  bool depth_frusta = false;
  /// The world axes' length (metres): x red, y green, z blue from the origin;
  /// 0 for none.
  double axes_m = 0.5;
  /// The solved boards' outline colour.
  std::uint32_t board_rgba = gfx::pipelines::pack_rgba8(255, 200, 0);
  /// The origin tag's edge (metres), whose outline @ref tag_lines draws at
  /// the origin of a tag's world; 0 for none.
  double tag_edge_m = 0.0;
  /// The origin tag's outline colour.
  std::uint32_t tag_rgba = gfx::pipelines::pack_rgba8(255, 0, 255);
};

/// @brief A camera's frustum: lines from its centre to each corner of its
///        image, and around the corners, @p depth_m ahead.
///
/// The corners are the image's outer edges, half a pixel outside the corner
/// pixels' centres, taken back through the lens by `compute::undistort_points`,
/// so a distorted lens's frustum is the view it really has.
/// @param camera     The camera's model, lens included.
/// @param to_world   Its frame to the world's.
/// @param depth_m    How far ahead the corners are (metres); positive.
/// @param rgba       The lines' colour.
/// @return 8 lines, 16 vertices in the world: centre to each corner, top-left,
///         top-right, bottom-right, bottom-left, then the corners in that
///         order; or `InvalidArgument` for a model recon refuses, a pose that
///         is not rigid, or a depth that is not positive and finite; or what
///         `compute::undistort_points` returns.
///
/// @code
/// VKC_ASSIGN(const auto lines,
///            frustum_lines(*info.color, color_to_world, 0.3, tint));
/// @endcode
VC_VIZ_API core::Result<std::vector<gfx::pipelines::ColorVertex>> frustum_lines(
    const recon::camera::CameraModel& camera,
    const recon::camera::Mat4d& to_world, double depth_m, std::uint32_t rgba);

/// @brief The world's axes from its origin: x red, y green, z blue.
/// @param length_m  Each axis's length (metres); positive.
/// @return 3 lines, 6 vertices; `InvalidArgument` for a length that is not
///         positive and finite; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const auto axes, axes_lines(0.5));
/// @endcode
VC_VIZ_API core::Result<std::vector<gfx::pipelines::ColorVertex>> axes_lines(
    double length_m);

/// @brief A solved board's outline: the chessboard's outer edge, on its
///        plane.
/// @param board           The board.
/// @param board_to_world  T_wb: where the solve put it.
/// @param rgba            The lines' colour.
/// @return 4 lines, 8 vertices, from the board frame's origin along its +x,
///         then around; `InvalidArgument` for a pose that is not rigid; or
///         `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const auto quad, board_lines(board, solution.boards[k], rgba));
/// @endcode
VC_VIZ_API core::Result<std::vector<gfx::pipelines::ColorVertex>> board_lines(
    const compute::Board& board, const recon::camera::Mat4d& board_to_world,
    std::uint32_t rgba);

/// @brief The origin tag's outline at the origin of its world: the square of
///        its black border on z = 0, its corners where `solve/world_tag.hpp`
///        puts them, (-s/2, s/2), (s/2, s/2), (s/2, -s/2) and (-s/2, -s/2).
/// @param edge_m  The square's edge, s (metres); positive.
/// @param rgba    The lines' colour.
/// @return 4 lines, 8 vertices: corner k to corner k + 1, the last back to
///         the first; `InvalidArgument` for an edge that is not positive and
///         finite; or `OutOfMemory`.
///
/// @code
/// VKC_ASSIGN(const auto outline, tag_lines(tag_world.edge_m, rgba));
/// @endcode
VC_VIZ_API core::Result<std::vector<gfx::pipelines::ColorVertex>> tag_lines(
    double edge_m, std::uint32_t rgba);

/// @brief Every line of the scene: the axes, each shown and placed camera's
///        frusta in its tint, each solved board's outline, and the origin
///        tag's (`SceneOptions::tag_edge_m`).
/// @param cameras         The rig's cameras.
/// @param board           The board the solve placed; null when
///                        @p boards_to_world is empty.
/// @param boards_to_world Each solved board's T_wb; empty before a solve.
/// @param options         The frusta's depth, the depth frusta, the axes,
///                        the boards' colour and the tag.
/// @return The lines' vertices, in pairs; or what @ref frustum_lines,
///         @ref axes_lines, @ref board_lines and @ref tag_lines refuse,
///         naming the camera, board or tag; `InvalidArgument` for boards
///         without a board.
///
/// @code
/// VKC_ASSIGN(const auto lines, scene_lines(cameras, nullptr, {}, {}));
/// @endcode
VC_VIZ_API core::Result<std::vector<gfx::pipelines::ColorVertex>> scene_lines(
    const std::vector<SceneCamera>& cameras, const compute::Board* board,
    const std::vector<recon::camera::Mat4d>& boards_to_world,
    const SceneOptions& options);

/// @brief The draws of each shown and placed camera's points, at its pose
///        now (@ref cloud_draw), in its tint.
/// @param frames   Each camera's prepared frame, in the rig's order; empty
///                 where it has none.
/// @param cameras  The rig's cameras, as many as @p frames.
/// @param style    How and where they are drawn; each camera's tint replaces
///                 its tint.
/// @return One draw per camera drawn, in the rig's order; `InvalidArgument`
///         for as many cameras as frames missing or extra; or what
///         @ref cloud_draw refuses, naming the camera.
///
/// @code
/// VKC_ASSIGN(const auto draws, cloud_draws(snapshot->frames, cameras, style));
/// VKC_TRY(clouds->submit(cmd, {extent, frame.number, draws.data(),
///                              static_cast<std::uint32_t>(draws.size())}));
/// @endcode
VC_VIZ_API core::Result<std::vector<gfx::pipelines::DepthCloudDraw>>
cloud_draws(
    const std::vector<std::optional<recon::sensor::DeviceFrame>>& frames,
    const std::vector<SceneCamera>& cameras, const CloudStyle& style);

/// @brief The turn from the rig's world into the view's: the world's up to
///        +Y, and the placed cameras' mean forward (+z), levelled, to -Z,
///        where gfx's `OrbitCamera` looks from its default place.
///
/// The world's up is @p up, given -- a tag's world's +z, so its floor lies
/// level -- or the placed cameras' mean up, their -y. When the forwards
/// cancel -- an outside-in ring looking at its middle -- the first camera's
/// levelled forward is taken, and failing that any level direction.
/// @param color_to_world  Each placed camera's pose; at least one.
/// @param up              The world's up, any length; empty for the
///                        cameras' mean up.
/// @return The rotation, world to view; `InvalidArgument` for no camera, a
///         pose that is not rigid, ups that cancel, or an @p up that is not
///         finite or has no length.
///
/// @code
/// // The reference camera alone: a half turn about x.
/// VKC_ASSIGN(const recon::camera::Mat3d turn,
///            display_rotation({recon::camera::Mat4d(1.0)}));
/// // A tag's world: +z to +Y.
/// VKC_ASSIGN(const recon::camera::Mat3d level,
///            display_rotation(poses, recon::camera::Vec3d(0.0, 0.0, 1.0)));
/// @endcode
VC_VIZ_API core::Result<recon::camera::Mat3d> display_rotation(
    const std::vector<recon::camera::Mat4d>& color_to_world,
    const std::optional<recon::camera::Vec3d>& up = std::nullopt);

/// @brief Where the 3D view sits in the window.
enum class ScenePlacement : std::uint8_t {
  Hidden,      ///< Not shown: the rig mosaic has the window.
  Beside,      ///< In the window's right half, the mosaic in its left.
  FullWindow,  ///< The whole window, over the mosaic.
};

/// @brief The rectangles of a window the mosaic and the 3D view take.
///
/// @code
/// const WindowSplit split = window_split(ScenePlacement::Beside, extent);
/// @endcode
struct WindowSplit {
  VkRect2D mosaic{};  ///< The mosaic's; empty when it is not shown.
  VkRect2D scene{};   ///< The 3D view's; empty when it is not shown.
};

/// @brief Split a window between the mosaic and the 3D view.
/// @param placement  Where the 3D view sits.
/// @param extent     The window's framebuffer size, in pixels.
/// @return The two rectangles: the left half (the odd pixel's) and the right
///         half `Beside`, or one of them the whole window.
///
/// @code
/// const WindowSplit split = window_split(placement, frame.target->extent());
/// @endcode
VC_VIZ_API WindowSplit window_split(ScenePlacement placement,
                                    VkExtent2D extent) noexcept;

/// @brief The 3D view's camera: gfx's `OrbitCamera` in the turned world, and
///        its perspective.
///
/// @code
/// SceneView view;
/// VKC_TRY(view.frame(placed_poses));      // aim at the rig
/// view.orbit().orbit(0.01f * dx, 0.01f * dy);  // a drag
/// const glm::mat4 view_proj = view.view_proj(split.scene);
/// @endcode
class VC_VIZ_API SceneView {
 public:
  /// The vertical field of view, in radians.
  static constexpr float kFovY = 0.87f;  // 50 degrees
  static constexpr float kNear = 0.05f;  ///< The near plane (metres).
  static constexpr float kFar = 100.0f;  ///< The far plane (metres).

  /// @brief Turn the world upright (@ref display_rotation) and aim at the
  ///        cameras: at their mean centre, 2 m along their mean forward, from
  ///        at least 3 m away and slightly above.
  /// @param color_to_world  Each placed camera's pose; at least one.
  /// @param up              The world's up (@ref display_rotation); empty
  ///                        for the cameras' mean up.
  /// @return OK; or what @ref display_rotation refuses, the view unchanged.
  core::Status frame(
      const std::vector<recon::camera::Mat4d>& color_to_world,
      const std::optional<recon::camera::Vec3d>& up = std::nullopt);

  /// @return The orbiting camera, in the turned world, for pointer input.
  gfx::camera::OrbitCamera& orbit() noexcept { return orbit_; }
  /// @copydoc orbit
  const gfx::camera::OrbitCamera& orbit() const noexcept { return orbit_; }
  /// @return The turn from the rig's world into the view's.
  const recon::camera::Mat3d& rotation() const noexcept { return rotation_; }

  /// @brief The rig's world to Vulkan clip space, for a viewport.
  /// @param viewport  The rectangle drawn into, for its aspect; an empty one
  ///                  is taken as square.
  /// @return `projection * view * rotation`, for gfx's draws.
  glm::mat4 view_proj(VkRect2D viewport) const;

 private:
  gfx::camera::OrbitCamera orbit_;
  recon::camera::Mat3d rotation_{1.0};
};

}  // namespace volumetric_kit::calib::viz
