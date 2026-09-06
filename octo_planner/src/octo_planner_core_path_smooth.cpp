#include "octo_planner/octo_planner_core.h"
#include <cmath>
#include <algorithm>
#include <vector>

namespace octo_planner
{

std::vector<GridIndex> OctoPlannerCore::shortcutPath(const std::vector<GridIndex> & raw_path) const
{
  if (raw_path.size() <= 2) {
    return raw_path;
  }

  // 1. Forward greedy shortcut pass
  std::vector<GridIndex> fwd_path;
  fwd_path.reserve(raw_path.size());
  fwd_path.push_back(raw_path.front());

  size_t curr = 0;
  while (curr < raw_path.size() - 1) {
    size_t furthest = curr + 1;
    for (size_t next = raw_path.size() - 1; next > curr + 1; --next) {
      if (isLineTraversable(raw_path[curr], raw_path[next])) {
        furthest = next;
        break;
      }
    }
    fwd_path.push_back(raw_path[furthest]);
    curr = furthest;
  }

  // 2. Backward greedy shortcut pass
  std::vector<GridIndex> bwd_path;
  bwd_path.reserve(raw_path.size());
  bwd_path.push_back(raw_path.back());

  int bwd_curr = static_cast<int>(raw_path.size()) - 1;
  while (bwd_curr > 0) {
    int furthest = bwd_curr - 1;
    for (int next = 0; next < bwd_curr - 1; ++next) {
      if (isLineTraversable(raw_path[bwd_curr], raw_path[next])) {
        furthest = next;
        break;
      }
    }
    bwd_path.push_back(raw_path[furthest]);
    bwd_curr = furthest;
  }
  std::reverse(bwd_path.begin(), bwd_path.end());

  // 3. Compute total metric length and pick the shorter one
  const auto calcLength = [this](const std::vector<GridIndex> & p) {
    double len = 0.0;
    for (size_t i = 0; i + 1 < p.size(); ++i) {
      const auto p1 = gridToWorld(p[i]);
      const auto p2 = gridToWorld(p[i + 1]);
      len += std::sqrt(
        (p2.x() - p1.x()) * (p2.x() - p1.x()) +
        (p2.y() - p1.y()) * (p2.y() - p1.y()) +
        (p2.z() - p1.z()) * (p2.z() - p1.z()));
    }
    return len;
  };

  return (calcLength(bwd_path) < calcLength(fwd_path)) ? bwd_path : fwd_path;
}

std::vector<geometry_msgs::PoseStamped> OctoPlannerCore::generateSmoothPath(
  const std::vector<GridIndex> & cells,
  const geometry_msgs::PoseStamped & start_pose,
  const geometry_msgs::PoseStamped & goal_pose,
  bool has_goal_pose) const
{
  std::vector<geometry_msgs::PoseStamped> plan;
  if (cells.empty()) return plan;

  ros::Time plan_time = ros::Time::now();
  const std::string frame_id = start_pose.header.frame_id.empty() ? "map" : start_pose.header.frame_id;

  if (cells.size() == 1 || !enable_path_smoothing_) {
    for (size_t i = 0; i < cells.size(); ++i) {
      const auto p = gridToWorld(cells[i]);
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = frame_id;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation.w = 1.0;
      if (i == 0) pose.pose.orientation = start_pose.pose.orientation;
      else if (i + 1 == cells.size() && has_goal_pose) pose.pose.orientation = goal_pose.pose.orientation;
      plan.push_back(pose);
    }
    return plan;
  }

  // Convert cells to metric 3D waypoints
  std::vector<octomap::point3d> waypoints;
  waypoints.reserve(cells.size());
  for (const auto & c : cells) {
    waypoints.push_back(gridToWorld(c));
  }

  // Generate piecewise curve segments with Bézier corner fillets
  std::vector<octomap::point3d> raw_curve_points;
  const size_t K = waypoints.size();

  if (K == 2) {
    const auto & p0 = waypoints[0];
    const auto & p1 = waypoints[1];
    const double seg_len = std::sqrt(
      (p1.x() - p0.x()) * (p1.x() - p0.x()) +
      (p1.y() - p0.y()) * (p1.y() - p0.y()) +
      (p1.z() - p0.z()) * (p1.z() - p0.z()));
    const double step_res = std::max(0.01, path_interpolation_resolution_);
    const int num_samples = std::max(1, static_cast<int>(std::ceil(seg_len / step_res)));
    for (int s = 0; s <= num_samples; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(num_samples);
      raw_curve_points.push_back(octomap::point3d(
        p0.x() + t * (p1.x() - p0.x()),
        p0.y() + t * (p1.y() - p0.y()),
        p0.z() + t * (p1.z() - p0.z())));
    }
  } else {
    // 1. Calculate segment directions and lengths
    std::vector<octomap::point3d> dir(K - 1);
    std::vector<double> seg_len(K - 1);
    for (size_t i = 0; i < K - 1; ++i) {
      const double dx = waypoints[i + 1].x() - waypoints[i].x();
      const double dy = waypoints[i + 1].y() - waypoints[i].y();
      const double dz = waypoints[i + 1].z() - waypoints[i].z();
      seg_len[i] = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (seg_len[i] > 1e-6) {
        dir[i] = octomap::point3d(dx / seg_len[i], dy / seg_len[i], dz / seg_len[i]);
      } else {
        dir[i] = octomap::point3d(1.0, 0.0, 0.0);
      }
    }

    // 2. Determine fillet control points at internal corners
    std::vector<octomap::point3d> fillet_start(K);
    std::vector<octomap::point3d> fillet_end(K);
    std::vector<bool> has_fillet(K, false);

    for (size_t i = 1; i < K - 1; ++i) {
      const double dot = dir[i - 1].x() * dir[i].x() + dir[i - 1].y() * dir[i].y() + dir[i - 1].z() * dir[i].z();
      if (dot > 0.999) {
        has_fillet[i] = false;
        continue;
      }
      double d = std::min({corner_fillet_radius_, 0.45 * seg_len[i - 1], 0.45 * seg_len[i]});
      if (d < 0.05) {
        has_fillet[i] = false;
        continue;
      }

      octomap::point3d p_start(
        waypoints[i].x() - d * dir[i - 1].x(),
        waypoints[i].y() - d * dir[i - 1].y(),
        waypoints[i].z() - d * dir[i - 1].z());
      octomap::point3d p_end(
        waypoints[i].x() + d * dir[i].x(),
        waypoints[i].y() + d * dir[i].y(),
        waypoints[i].z() + d * dir[i].z());

      octomap::point3d mid(
        0.25 * p_start.x() + 0.5 * waypoints[i].x() + 0.25 * p_end.x(),
        0.25 * p_start.y() + 0.5 * waypoints[i].y() + 0.25 * p_end.y(),
        0.25 * p_start.z() + 0.5 * waypoints[i].z() + 0.25 * p_end.z());
      const GridIndex mid_g = worldToGrid(mid.x(), mid.y(), mid.z());
      if (isInsideMetricBounds(mid_g) && !isOccupiedCell(mid_g) &&
          preblocked_cells_.find(mid_g) == preblocked_cells_.end()) {
        fillet_start[i] = p_start;
        fillet_end[i] = p_end;
        has_fillet[i] = true;
      } else {
        has_fillet[i] = false;
      }
    }

    // 3. Assemble trajectory by sampling straight segments and Bézier arcs
    const double step_res = std::max(0.01, path_interpolation_resolution_);
    raw_curve_points.push_back(waypoints[0]);

    for (size_t i = 0; i < K - 1; ++i) {
      const octomap::point3d seg_start = (i == 0 || !has_fillet[i]) ? waypoints[i] : fillet_end[i];
      const octomap::point3d seg_end = (!has_fillet[i + 1]) ? waypoints[i + 1] : fillet_start[i + 1];

      const double l_dx = seg_end.x() - seg_start.x();
      const double l_dy = seg_end.y() - seg_start.y();
      const double l_dz = seg_end.z() - seg_start.z();
      const double l_dist = std::sqrt(l_dx * l_dx + l_dy * l_dy + l_dz * l_dz);
      const int l_samples = std::max(1, static_cast<int>(std::ceil(l_dist / step_res)));

      for (int s = 1; s <= l_samples; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(l_samples);
        raw_curve_points.push_back(octomap::point3d(
          seg_start.x() + t * l_dx,
          seg_start.y() + t * l_dy,
          seg_start.z() + t * l_dz));
      }

      if (i + 1 < K - 1 && has_fillet[i + 1]) {
        const auto & p_start = fillet_start[i + 1];
        const auto & p_ctrl = waypoints[i + 1];
        const auto & p_end = fillet_end[i + 1];
        const double arc_approx_len = corner_fillet_radius_ * 1.5;
        const int arc_samples = std::max(3, static_cast<int>(std::ceil(arc_approx_len / step_res)));

        for (int s = 1; s <= arc_samples; ++s) {
          const double u = static_cast<double>(s) / static_cast<double>(arc_samples);
          const double u_inv = 1.0 - u;
          const double bx = u_inv * u_inv * p_start.x() + 2.0 * u_inv * u * p_ctrl.x() + u * u * p_end.x();
          const double by = u_inv * u_inv * p_start.y() + 2.0 * u_inv * u * p_ctrl.y() + u * u * p_end.y();
          const double bz = u_inv * u_inv * p_start.z() + 2.0 * u_inv * u * p_ctrl.z() + u * u * p_end.z();
          raw_curve_points.push_back(octomap::point3d(bx, by, bz));
        }
      }
    }
  }

  if (raw_curve_points.empty()) return plan;

  // 4. Equidistant arc-length resampling
  std::vector<octomap::point3d> resampled_points;
  resampled_points.push_back(raw_curve_points.front());
  const double desired_ds = std::max(0.01, path_interpolation_resolution_);
  double accumulated_dist = 0.0;

  for (size_t i = 0; i + 1 < raw_curve_points.size(); ++i) {
    const auto & pA = raw_curve_points[i];
    const auto & pB = raw_curve_points[i + 1];
    const double dAB = std::sqrt(
      (pB.x() - pA.x()) * (pB.x() - pA.x()) +
      (pB.y() - pA.y()) * (pB.y() - pA.y()) +
      (pB.z() - pA.z()) * (pB.z() - pA.z()));
    if (dAB < 1e-6) continue;

    double curr_dist = 0.0;
    while (accumulated_dist + (dAB - curr_dist) >= desired_ds) {
      const double remain = desired_ds - accumulated_dist;
      curr_dist += remain;
      const double t = curr_dist / dAB;
      resampled_points.push_back(octomap::point3d(
        pA.x() + t * (pB.x() - pA.x()),
        pA.y() + t * (pB.y() - pA.y()),
        pA.z() + t * (pB.z() - pA.z())));
      accumulated_dist = 0.0;
    }
    accumulated_dist += (dAB - curr_dist);
  }

  if (std::sqrt(
        (resampled_points.back().x() - raw_curve_points.back().x()) * (resampled_points.back().x() - raw_curve_points.back().x()) +
        (resampled_points.back().y() - raw_curve_points.back().y()) * (resampled_points.back().y() - raw_curve_points.back().y()) +
        (resampled_points.back().z() - raw_curve_points.back().z()) * (resampled_points.back().z() - raw_curve_points.back().z())) > 1e-3) {
    resampled_points.push_back(raw_curve_points.back());
  }

  const size_t N_pts = resampled_points.size();
  if (N_pts == 0) return plan;

  // 5. Compute continuous smooth yaw profile
  std::vector<double> yaws(N_pts, 0.0);
  if (enable_continuous_yaw_) {
    for (size_t i = 0; i < N_pts; ++i) {
      if (i < N_pts - 1) {
        const double dx = resampled_points[i + 1].x() - resampled_points[i].x();
        const double dy = resampled_points[i + 1].y() - resampled_points[i].y();
        if (std::hypot(dx, dy) > 1e-4) {
          yaws[i] = std::atan2(dy, dx);
        } else {
          yaws[i] = (i > 0) ? yaws[i - 1] : 0.0;
        }
      } else {
        yaws[i] = (i > 0) ? yaws[i - 1] : 0.0;
      }
    }

    double goal_yaw = 0.0;
    if (has_goal_pose) {
      const auto & q = goal_pose.pose.orientation;
      goal_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      yaws[N_pts - 1] = goal_yaw;
    }

    for (size_t i = 1; i < N_pts; ++i) {
      double diff = yaws[i] - yaws[i - 1];
      while (diff > M_PI) { yaws[i] -= 2.0 * M_PI; diff = yaws[i] - yaws[i - 1]; }
      while (diff < -M_PI) { yaws[i] += 2.0 * M_PI; diff = yaws[i] - yaws[i - 1]; }
    }

    if (has_goal_pose && N_pts > 1) {
      const size_t blend_count = std::min(static_cast<size_t>(10), N_pts);
      const size_t start_blend = N_pts - blend_count;
      double diff_goal = goal_yaw - yaws[start_blend];
      while (diff_goal > M_PI) diff_goal -= 2.0 * M_PI;
      while (diff_goal < -M_PI) diff_goal += 2.0 * M_PI;

      for (size_t i = start_blend; i < N_pts; ++i) {
        const double frac = static_cast<double>(i - start_blend) / static_cast<double>(blend_count - 1);
        yaws[i] = yaws[start_blend] + frac * diff_goal;
      }
    }

    const int win = std::max(1, yaw_smoothing_window_);
    if (win > 1 && N_pts > static_cast<size_t>(win)) {
      std::vector<double> smooth_yaws = yaws;
      const int half_w = win / 2;
      for (size_t i = 1; i + 1 < N_pts; ++i) {
        double sum = 0.0;
        int count = 0;
        for (int w = -half_w; w <= half_w; ++w) {
          int idx = static_cast<int>(i) + w;
          if (idx >= 0 && idx < static_cast<int>(N_pts)) {
            sum += yaws[idx];
            ++count;
          }
        }
        smooth_yaws[i] = sum / count;
      }
      yaws = smooth_yaws;
    }
  }

  // 6. Build PoseStamped vector
  plan.reserve(N_pts);
  for (size_t i = 0; i < N_pts; ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = resampled_points[i].x();
    pose.pose.position.y = resampled_points[i].y();
    pose.pose.position.z = resampled_points[i].z();

    if (enable_continuous_yaw_) {
      const double half_yaw = yaws[i] * 0.5;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(half_yaw);
      pose.pose.orientation.w = std::cos(half_yaw);
    } else {
      pose.pose.orientation.w = 1.0;
    }

    if (i == 0 && !start_pose.header.frame_id.empty() && start_pose.pose.orientation.w != 0.0) {
      pose.pose.orientation = start_pose.pose.orientation;
    }
    if (i + 1 == N_pts && has_goal_pose) {
      pose.pose.orientation = goal_pose.pose.orientation;
    }

    plan.push_back(pose);
  }

  return plan;
}

} // namespace octo_planner
