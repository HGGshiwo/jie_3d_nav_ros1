#include "octo_planner/octo_local_planner.h"
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>
#include <limits>
#include <cmath>
#include <algorithm>

namespace octo_planner
{

int OctoLocalPlanner::findInitialTargetIndex3D()
{
  if (global_plan_.empty()) return 0;
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return target_index_;

  const std::size_t plan_size = global_plan_.size();
  if (plan_size <= 1)
  {
    target_index_ = 0;
    return 0;
  }

  int best_seg_idx = target_index_;
  if (best_seg_idx < 0 || best_seg_idx >= static_cast<int>(plan_size) - 1)
  {
    best_seg_idx = 0;
  }

  // Tight search window around previous target_index_ to guarantee forward progression
  const int search_start = std::max(0, best_seg_idx - 2);
  const int search_end = std::min(static_cast<int>(plan_size) - 1, best_seg_idx + 40);

  double min_proj_sq_dist = std::numeric_limits<double>::max();

  for (int i = search_start; i < search_end; ++i)
  {
    const auto & p1 = global_plan_[i].pose.position;
    const auto & p2 = global_plan_[i + 1].pose.position;
    const double dx = p2.x - p1.x, dy = p2.y - p1.y, dz = p2.z - p1.z;
    const double seg_sq_len = dx * dx + dy * dy + dz * dz;

    double t = 0.0;
    if (seg_sq_len > 1.0e-6)
    {
      t = ((robot_pose.x - p1.x) * dx + (robot_pose.y - p1.y) * dy + (robot_pose.z - p1.z) * dz) / seg_sq_len;
      t = std::max(0.0, std::min(1.0, t));
    }
    const double px = p1.x + t * dx;
    const double py = p1.y + t * dy;
    const double pz = p1.z + t * dz;
    const double sq_dist = (robot_pose.x - px) * (robot_pose.x - px) +
                          (robot_pose.y - py) * (robot_pose.y - py) +
                          (robot_pose.z - pz) * (robot_pose.z - pz);

    if (sq_dist < min_proj_sq_dist)
    {
      min_proj_sq_dist = sq_dist;
      best_seg_idx = i;
    }
  }

  target_index_ = best_seg_idx;
  return target_index_;
}

bool OctoLocalPlanner::lookupRobotPose2D(RobotPose2D & robot_pose)
{
  if (!costmap_ros_) return false;
  std::string base_frame = costmap_ros_->getBaseFrameID();
  try
  {
    const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame, ros::Time(0), ros::Duration(0.05));
    robot_pose.x   = tf.transform.translation.x;
    robot_pose.y   = tf.transform.translation.y;
    robot_pose.z   = tf.transform.translation.z;
    robot_pose.yaw = tf2::getYaw(tf.transform.rotation);
    applyRobotCenterOffset(base_frame, robot_pose);
    return true;
  }
  catch (const tf2::TransformException & ex)
  {
    ROS_WARN_THROTTLE(2.0, "OctoLocalPlanner: Transform %s -> %s failed: %s", map_frame_.c_str(), base_frame.c_str(), ex.what());
    return false;
  }
  catch (...)
  {
    return false;
  }
}

bool OctoLocalPlanner::computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error)
{
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return false;
  geometry_msgs::PoseStamped final_pose = final_pose_in;
  if (final_pose.header.frame_id.empty()) final_pose.header.frame_id = map_frame_;
  final_pose.header.stamp = ros::Time(0);
  try
  {
    if (final_pose.header.frame_id != map_frame_)
      tf_buffer_->transform(final_pose, final_pose, map_frame_, ros::Duration(0.05));
  }
  catch (...)
  {
    return false;
  }
  yaw_error = normalizeAngle(tf2::getYaw(final_pose.pose.orientation) - robot_pose.yaw);
  return true;
}

bool OctoLocalPlanner::transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out)
{
  if (!costmap_ros_) return false;
  std::string base_frame = costmap_ros_->getBaseFrameID();
  geometry_msgs::PoseStamped stamped = pose_in;
  if (stamped.header.frame_id.empty()) stamped.header.frame_id = map_frame_;
  stamped.header.stamp = ros::Time(0);
  try
  {
    tf_buffer_->transform(stamped, pose_out, base_frame, ros::Duration(0.05));
    applyRobotCenterOffsetToRelativePose(base_frame, pose_out);
    return true;
  }
  catch (const tf2::TransformException & ex)
  {
    ROS_WARN_THROTTLE(2.0, "OctoLocalPlanner: transformToBase %s -> %s failed: %s", stamped.header.frame_id.c_str(), base_frame.c_str(), ex.what());
    return false;
  }
  catch (...)
  {
    return false;
  }
}

bool OctoLocalPlanner::isGoalReached()
{
  if (goal_reached_)
  {
    ROS_INFO_THROTTLE(1.0, "[OctoLocalPlanner] isGoalReached() is TRUE! Navigation complete.");
    return true;
  }

  // Robust verification: check if robot is already physically within goal tolerances
  if (!global_plan_.empty())
  {
    RobotPose2D robot_pose;
    if (lookupRobotPose2D(robot_pose))
    {
      const auto & goal_pos = global_plan_.back().pose.position;
      const double dist_to_goal = std::hypot(goal_pos.x - robot_pose.x, goal_pos.y - robot_pose.y);
      double final_yaw_error = 0.0;
      bool yaw_ok = true;
      if (align_final_yaw_)
      {
        if (computeFinalYawErrorXY(global_plan_.back(), final_yaw_error))
        {
          yaw_ok = std::abs(final_yaw_error) < goal_yaw_tol_;
        }
      }
      if (dist_to_goal < goal_pos_tol_ && yaw_ok)
      {
        resetPlanState();
        velocity_smoother_.reset();
        ROS_INFO("[OctoLocalPlanner] Goal reached directly verified! dist=%.3fm, yaw_err=%.3frad.",
                 dist_to_goal, final_yaw_error);
        return true;
      }
    }
  }

  return goal_reached_;
}

void OctoLocalPlanner::publishStatus(const std::string& status)
{
  last_status_ = status;
  std_msgs::String msg;
  msg.data = status;
  status_pub_.publish(msg);
}

bool OctoLocalPlanner::checkEmergencyStop(const RobotPose2D & robot_pose, geometry_msgs::Twist & cmd_vel)
{
  if (!enable_emergency_stop_check_) return false;

  auto octree = planner_.getOctree();
  if (!octree) return false;

  double min_fwd = robot_radius_ + 0.05;
  double max_fwd = robot_radius_ + 0.35;
  int occupied_count = 0;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker free_marker, occupied_marker;
  free_marker.header.frame_id = map_frame_;
  free_marker.header.stamp = ros::Time::now();
  free_marker.ns = "emergency_stop_free";
  free_marker.id = 0;
  free_marker.type = visualization_msgs::Marker::CUBE_LIST;
  free_marker.action = visualization_msgs::Marker::ADD;
  free_marker.scale.x = 0.06; free_marker.scale.y = 0.06; free_marker.scale.z = 0.06;
  free_marker.color.g = 1.0f; free_marker.color.a = 0.6f;
  free_marker.pose.orientation.w = 1.0;

  occupied_marker.header = free_marker.header;
  occupied_marker.ns = "emergency_stop_occupied";
  occupied_marker.id = 1;
  occupied_marker.type = visualization_msgs::Marker::CUBE_LIST;
  occupied_marker.action = visualization_msgs::Marker::ADD;
  occupied_marker.scale.x = 0.10; occupied_marker.scale.y = 0.10; occupied_marker.scale.z = 0.10;
  occupied_marker.color.r = 1.0f; occupied_marker.color.a = 0.95f;
  occupied_marker.pose.orientation.w = 1.0;

  geometry_msgs::Point first_hit_pt;

  const double probe_z = robot_pose.z + robot_body_height_;

  for (double fwd = min_fwd; fwd <= max_fwd; fwd += 0.08)
  {
    for (double lat = -robot_radius_ * 0.6; lat <= robot_radius_ * 0.6; lat += 0.08)
    {
      double mx = robot_pose.x + std::cos(robot_pose.yaw) * fwd - std::sin(robot_pose.yaw) * lat;
      double my = robot_pose.y + std::sin(robot_pose.yaw) * fwd + std::cos(robot_pose.yaw) * lat;
      octomap::point3d check_p(static_cast<float>(mx), static_cast<float>(my), static_cast<float>(probe_z));

      geometry_msgs::Point pt;
      pt.x = mx; pt.y = my; pt.z = probe_z;

      const octomap::OcTreeNode* node = octree->search(check_p);
      if (node && octree->isNodeOccupied(node))
      {
        occupied_count++;
        occupied_marker.points.push_back(pt);
        if (occupied_count == 1)
        {
          first_hit_pt = pt;
        }
      }
      else
      {
        free_marker.points.push_back(pt);
      }
    }
  }

  if (!free_marker.points.empty())
  {
    marker_array.markers.push_back(free_marker);
  }
  else
  {
    free_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.push_back(free_marker);
  }

  if (!occupied_marker.points.empty())
  {
    marker_array.markers.push_back(occupied_marker);
  }
  else
  {
    occupied_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.push_back(occupied_marker);
  }
  emergency_stop_pub_.publish(marker_array);

  if (occupied_count >= emergency_stop_min_occupied_voxels_)
  {
    ROS_WARN_THROTTLE(1.0, "OctoLocalPlanner: Obstacle detected in front (%d occupied voxels >= %d)! First hit at (x=%.2f, y=%.2f, z=%.2f), robot_z=%.2f, probe_z=%.2f. Stopping.",
                      occupied_count, emergency_stop_min_occupied_voxels_, first_hit_pt.x, first_hit_pt.y, first_hit_pt.z, robot_pose.z, probe_z);
    velocity_smoother_.reset();
    cmd_vel = geometry_msgs::Twist();
    return true;
  }
  return false;
}

void OctoLocalPlanner::onOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
{
  if (worker_running_.load()) return;

  worker_running_ = true;
  std::thread([this, msg]() {
    processOctomapAsync(msg);
  }).detach();
}

void OctoLocalPlanner::processOctomapAsync(const octomap_msgs::Octomap::ConstPtr & msg)
{
  std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
  if (octree)
  {
    bg_planner_.setOctree(octree);
    bg_planner_.rebuildPreblockedCells();
    bg_planner_.rebuildPreblockedCostmap();

    // Fast pointer/layer swap with main planner under lock (<0.01 ms)
    {
      std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
      active_octree_ = octree;
      planner_.swapLayersAndMap(bg_planner_);
      map_ready_ = true;
      map_changed_ = true;
    }
  }
  else
  {
    ROS_WARN_THROTTLE(2.0, "OctoLocalPlanner: OcTree conversion failed.");
  }
  worker_running_ = false;
}

} // namespace octo_planner
