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
  if (!lookupRobotPose2D(robot_pose)) return 0;

  int start_idx = 0;
  int end_idx = static_cast<int>(global_plan_.size());
  if (target_index_ > 0 && target_index_ < end_idx)
  {
    start_idx = std::max(0, target_index_ - 30);
    end_idx = std::min(static_cast<int>(global_plan_.size()), target_index_ + 100);
  }

  int nearest = start_idx;
  double nearest_sq = std::numeric_limits<double>::max();
  for (int i = start_idx; i < end_idx; ++i)
  {
    const auto & p = global_plan_[i].pose.position;
    const double sq = (p.x - robot_pose.x)*(p.x - robot_pose.x) +
                      (p.y - robot_pose.y)*(p.y - robot_pose.y) +
                      (p.z - robot_pose.z)*(p.z - robot_pose.z);
    if (sq < nearest_sq)
    {
      nearest_sq = sq;
      nearest = i;
    }
  }
  return nearest;
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
    std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
    active_octree_ = octree;
    planner_.setOctree(active_octree_);
    planner_.rebuildPreblockedCells();
    map_ready_ = true;
    map_changed_ = true;
  }
  else
  {
    ROS_WARN_THROTTLE(2.0, "OctoLocalPlanner: OcTree conversion failed.");
  }
  worker_running_ = false;
}

} // namespace octo_planner
