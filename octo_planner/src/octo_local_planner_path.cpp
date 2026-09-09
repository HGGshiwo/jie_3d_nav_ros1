#include "octo_planner/octo_local_planner.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <algorithm>
#include <limits>

namespace octo_planner
{

std::vector<geometry_msgs::PoseStamped> OctoLocalPlanner::extractLocalBand(const RobotPose2D & robot_pose, double max_distance)
{
  int seg_idx = findInitialTargetIndex3D();
  std::vector<geometry_msgs::PoseStamped> local_band;

  geometry_msgs::PoseStamped cur_pose;
  cur_pose.header.frame_id = map_frame_;
  cur_pose.header.stamp = ros::Time::now();
  cur_pose.pose.position.x = robot_pose.x;
  cur_pose.pose.position.y = robot_pose.y;
  cur_pose.pose.position.z = robot_pose.z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, robot_pose.yaw);
  cur_pose.pose.orientation = tf2::toMsg(q);
  local_band.push_back(cur_pose);

  double accumulated_dist = 0.0;
  int idx = seg_idx + 1;
  // Reasonable upper bound to prevent runaway in edge cases (e.g. 40 points per meter)
  const size_t max_points = std::max(static_cast<size_t>(100), static_cast<size_t>(max_distance * 40.0));

  while (idx < static_cast<int>(global_plan_.size()) && accumulated_dist < max_distance && local_band.size() < max_points)
  {
    local_band.push_back(global_plan_[idx]);
    if (local_band.size() >= 2)
    {
      const auto & p_prev = local_band[local_band.size() - 2].pose.position;
      const auto & p_curr = local_band.back().pose.position;
      accumulated_dist += std::hypot(p_curr.x - p_prev.x, p_curr.y - p_prev.y);
    }
    idx++;
  }

  if (local_band.size() == 1 && !global_plan_.empty())
  {
    local_band.push_back(global_plan_.back());
  }

  return local_band;
}

std::vector<geometry_msgs::PoseStamped> OctoLocalPlanner::checkAndReplanAStarDetour(const std::vector<geometry_msgs::PoseStamped> & local_band)
{
  if (local_band.size() < 2 || !map_ready_)
  {
    if (!map_ready_)
    {
      ROS_WARN_THROTTLE(2.0, "OctoLocalPlanner: checkAndReplanAStarDetour: OcTree map is not ready! Returning global band.");
    }
    return local_band;
  }

  const auto & p_start = local_band.front().pose.position;
  geometry_msgs::Point p_goal = local_band.back().pose.position;
  geometry_msgs::PoseStamped goal_pose_stamped = local_band.back();

  // 1. Calculate lateral cross-track distance from robot to nearest point on global plan
  double d_lateral = 0.0;
  if (!global_plan_.empty() && target_index_ >= 0 && target_index_ < static_cast<int>(global_plan_.size()))
  {
    const auto & p_global = global_plan_[target_index_].pose.position;
    d_lateral = std::hypot(p_start.x - p_global.x, p_start.y - p_global.y);
  }

  // 2. Check if the default corridor trajectory has any obstacle collision
  bool is_corridor_blocked = false;
  for (size_t i = 0; i + 1 < local_band.size(); ++i)
  {
    const auto & p1 = local_band[i].pose.position;
    const auto & p2 = local_band[i + 1].pose.position;
    octomap::point3d pt1(static_cast<float>(p1.x), static_cast<float>(p1.y), static_cast<float>(p1.z));
    octomap::point3d pt2(static_cast<float>(p2.x), static_cast<float>(p2.y), static_cast<float>(p2.z));
    if (!planner_.isLineTraversable(pt1, pt2))
    {
      is_corridor_blocked = true;
      break;
    }
  }

  // 3. Continuous Guidance:
  // If the path ahead is clear AND the robot is already closely tracking the global path,
  // we follow the global corridor with zero overhead.
  if (!is_corridor_blocked && d_lateral < 0.15)
  {
    last_valid_detour_.clear();
    return local_band;
  }

  // 4. Anchor Point Adaptation for Detour Planning:
  // If the nominal forward anchor is itself occupied or lacking traversable ground support,
  // adaptively search forward along the global path to find the re-emergence point beyond the obstacle.
  octomap::point3d goal_pt(static_cast<float>(p_goal.x), static_cast<float>(p_goal.y), static_cast<float>(p_goal.z));
  if (!planner_.isLineTraversable(goal_pt, goal_pt))
  {
    bool found_clean_anchor = false;
    int cur_idx = (target_index_ >= 0 && target_index_ < static_cast<int>(global_plan_.size())) ? target_index_ : 0;
    int nominal_idx = cur_idx;
    for (int i = cur_idx; i < static_cast<int>(global_plan_.size()); ++i)
    {
      const auto & gp = global_plan_[i].pose.position;
      if (std::hypot(gp.x - p_goal.x, gp.y - p_goal.y) < 0.10)
      {
        nominal_idx = i;
        break;
      }
    }

    // A. Forward search along global plan (up to ~1.75m / 35 poses further) to emerge past the obstacle
    const int max_fwd_idx = std::min(static_cast<int>(global_plan_.size()) - 1, nominal_idx + 35);
    for (int i = nominal_idx + 1; i <= max_fwd_idx; ++i)
    {
      const auto & cand = global_plan_[i].pose.position;
      octomap::point3d cand_pt(static_cast<float>(cand.x), static_cast<float>(cand.y), static_cast<float>(cand.z));
      if (planner_.isLineTraversable(cand_pt, cand_pt))
      {
        p_goal = cand;
        goal_pose_stamped = global_plan_[i];
        found_clean_anchor = true;
        break;
      }
    }

    // B. Backward fallback: If obstacle extends too far, pick the furthest traversable pose in front
    if (!found_clean_anchor)
    {
      for (int i = static_cast<int>(local_band.size()) - 2; i >= 1; --i)
      {
        const auto & cand = local_band[i].pose.position;
        octomap::point3d cand_pt(static_cast<float>(cand.x), static_cast<float>(cand.y), static_cast<float>(cand.z));
        if (planner_.isLineTraversable(cand_pt, cand_pt))
        {
          p_goal = cand;
          goal_pose_stamped = local_band[i];
          found_clean_anchor = true;
          break;
        }
      }
    }
  }

  // 5. If an obstacle blocks the corridor OR the robot is laterally offset (e.g. detouring beside an obstacle),
  // continuously plan with 3D A* from current robot pose directly to the forward anchor point on the global path!
  std::vector<GridIndex> path_cells;
  std::string error_msg;
  if (planner_.plan(p_start, p_goal, path_cells, error_msg))
  {
    auto replanned = planner_.generateSmoothPath(path_cells, local_band.front(), goal_pose_stamped, true);
    if (replanned.size() >= 2)
    {
      last_valid_detour_ = replanned;
      ROS_INFO_THROTTLE(1.0, "OctoLocalPlanner: Continuous 3D A* path generated: %zu nodes (d_lat=%.2fm, horizon=%.2fm).",
                        replanned.size(), d_lateral, local_planner_horizon_);
      return replanned;
    }
    error_msg = "generateSmoothPath returned < 2 points";
  }

  // 5. Fail-safe continuity: If 3D A* fails temporarily (e.g. single-frame sensor shadow),
  // seamlessly roll along the previous valid detour path instead of snapping into obstacles.
  if (!last_valid_detour_.empty())
  {
    size_t best_idx = 0;
    double min_sq_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < last_valid_detour_.size(); ++i)
    {
      const auto & p = last_valid_detour_[i].pose.position;
      double d2 = (p.x - p_start.x) * (p.x - p_start.x) + (p.y - p_start.y) * (p.y - p_start.y);
      if (d2 < min_sq_dist)
      {
        min_sq_dist = d2;
        best_idx = i;
      }
    }

    std::vector<geometry_msgs::PoseStamped> rolled_detour;
    rolled_detour.push_back(local_band.front());
    for (size_t i = best_idx + 1; i < last_valid_detour_.size(); ++i)
    {
      rolled_detour.push_back(last_valid_detour_[i]);
    }

    if (rolled_detour.size() >= 2)
    {
      ROS_WARN_THROTTLE(1.0, "OctoLocalPlanner: A* plan returned false (%s), rolling along last valid detour (%zu poses left).",
                        error_msg.c_str(), rolled_detour.size());
      last_valid_detour_ = rolled_detour;
      return rolled_detour;
    }
  }

  ROS_WARN_THROTTLE(1.0,
    "OctoLocalPlanner: 3D A* detour failed [%s] (blocked=%d, d_lat=%.2fm)! Halting and returning empty to prevent collision.",
    error_msg.c_str(), is_corridor_blocked ? 1 : 0, d_lateral);

  last_valid_detour_.clear();
  return {};
}

std::vector<geometry_msgs::PoseStamped> OctoLocalPlanner::clipTrajectoryByDistance(const std::vector<geometry_msgs::PoseStamped> & path, double max_distance)
{
  if (path.size() <= 2)
  {
    return path;
  }

  std::vector<geometry_msgs::PoseStamped> clipped;
  clipped.reserve(path.size());
  clipped.push_back(path.front());

  double acc_d = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
  {
    clipped.push_back(path[i]);
    const auto & p_prev = clipped[clipped.size() - 2].pose.position;
    const auto & p_curr = clipped.back().pose.position;
    acc_d += std::hypot(p_curr.x - p_prev.x, p_curr.y - p_prev.y);
    if (acc_d >= max_distance && clipped.size() >= 3)
    {
      break;
    }
  }

  return clipped;
}

void OctoLocalPlanner::publishLocalAStarPlan(const std::vector<geometry_msgs::PoseStamped> & path)
{
  if (local_astar_plan_pub_.getNumSubscribers() > 0)
  {
    nav_msgs::Path astar_path_msg;
    astar_path_msg.header.frame_id = map_frame_;
    astar_path_msg.header.stamp = ros::Time::now();
    astar_path_msg.poses = path;
    local_astar_plan_pub_.publish(astar_path_msg);
  }
}

} // namespace octo_planner
