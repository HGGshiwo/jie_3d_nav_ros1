#include "octo_planner/octo_local_planner.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <algorithm>

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
    return local_band;
  }

  bool is_segment_blocked = false;
  for (size_t i = 0; i + 1 < local_band.size(); ++i)
  {
    const auto & p1 = local_band[i].pose.position;
    const auto & p2 = local_band[i + 1].pose.position;
    octomap::point3d pt1(static_cast<float>(p1.x), static_cast<float>(p1.y), static_cast<float>(p1.z));
    octomap::point3d pt2(static_cast<float>(p2.x), static_cast<float>(p2.y), static_cast<float>(p2.z));
    if (!planner_.isLineTraversable(pt1, pt2))
    {
      is_segment_blocked = true;
      break;
    }
  }

  if (is_segment_blocked)
  {
    const auto & p_start = local_band.front().pose.position;
    const auto & p_goal  = local_band.back().pose.position;
    std::vector<GridIndex> path_cells;
    std::string error_msg;
    if (planner_.plan(p_start, p_goal, path_cells, error_msg))
    {
      auto replanned = planner_.generateSmoothPath(path_cells, local_band.front(), local_band.back(), true);
      if (replanned.size() >= 2)
      {
        ROS_INFO_THROTTLE(1.0, "OctoLocalPlanner: Obstacle detected on local segment. Local 3D A* detour generated with %zu nodes (horizon=%.2fm).",
                          replanned.size(), local_planner_horizon_);
        return replanned;
      }
    }
  }

  return local_band;
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
