#ifndef OCTO_PLANNER_OCTO_LOCAL_PLANNER_H_
#define OCTO_PLANNER_OCTO_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>

#include "octo_planner/octo_planner_core.h"
#include "octo_planner/octo_elastic_band.h"
#include "octo_planner/octo_local_visualizer.h"
#include "octo_planner/d1_velocity_smoother.h"

namespace octo_planner
{

class OctoLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  OctoLocalPlanner();
  virtual ~OctoLocalPlanner();

  // nav_core::BaseLocalPlanner interface methods
  virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  virtual bool isGoalReached() override;

private:
  struct RobotPose2D { double x, y, z, yaw; };
  struct TrackingTarget { double base_x, base_y; };

  // Local Octomap callback
  void onOctomap(const octomap_msgs::Octomap::ConstPtr & msg);

  // Background Octomap deserialization worker
  void processOctomapAsync(const octomap_msgs::Octomap::ConstPtr & msg);

  // Helper tracking & TF functions
  int findInitialTargetIndex3D();
  bool lookupRobotPose2D(RobotPose2D & robot_pose);
  bool computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error);
  bool transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out);
  
  bool shouldApplyRobotCenterOffset(const std::string & frame) const { return frame == robot_center_offset_frame_; }
  void applyRobotCenterOffset(const std::string & frame, RobotPose2D & rp) const {
    if (!shouldApplyRobotCenterOffset(frame)) return;
    const double cy = std::cos(rp.yaw), sy = std::sin(rp.yaw);
    rp.x += cy * robot_center_offset_x_ - sy * robot_center_offset_y_;
    rp.y += sy * robot_center_offset_x_ + cy * robot_center_offset_y_;
    rp.z += robot_center_offset_z_;
  }
  void applyRobotCenterOffsetToRelativePose(const std::string & frame, geometry_msgs::PoseStamped & pose) const {
    if (!shouldApplyRobotCenterOffset(frame)) return;
    pose.pose.position.x -= robot_center_offset_x_;
    pose.pose.position.y -= robot_center_offset_y_;
    pose.pose.position.z -= robot_center_offset_z_;
  }

  static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
  static double applyDeadband(double v, double db) { return std::abs(v) < db ? 0.0 : v; }
  static double normalizeAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

  void publishStatus(const std::string& status);

  // TF & Costmap Pointers
  tf2_ros::Buffer* tf_buffer_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  bool initialized_;
  bool map_ready_;
  bool map_changed_;
  ros::Time last_local_rebuild_time_;
  ros::Time last_control_time_;

  // ROS communications
  ros::NodeHandle nh_;
  ros::Subscriber octomap_sub_;
  ros::Publisher status_pub_;
  ros::Publisher emergency_stop_pub_;

  // Planner Components
  OctoPlannerCore planner_;
  OctoElasticBand elastic_band_;
  OctoLocalVisualizer visualizer_;
  D1VelocitySmoother velocity_smoother_;

  // Async octomap worker thread & atomic ptr
  std::shared_ptr<octomap::OcTree> active_octree_;
  std::atomic<bool> worker_running_;

  // Parameters
  std::string map_frame_;
  std::string robot_center_offset_frame_;
  double robot_center_offset_x_, robot_center_offset_y_, robot_center_offset_z_;
  double lookahead_distance_, tracking_xy_tol_;
  double goal_pos_tol_, goal_yaw_tol_;
  double linear_gain_, lateral_gain_, heading_gain_, cross_track_angular_gain_, final_yaw_gain_;
  bool   enable_lateral_motion_;
  double max_linear_speed_, max_lateral_speed_, max_angular_speed_;
  bool   align_final_yaw_;
  double linear_deadband_, lateral_deadband_, angular_deadband_;
  bool   enable_emergency_stop_check_;
  int    emergency_stop_min_occupied_voxels_;

  double robot_radius_;
  bool require_ground_support_;
  bool strict_direct_ground_support_;
  int ground_support_xy_radius_cells_;
  int ground_support_depth_cells_;
  int max_step_height_cells_;
  int robot_clearance_height_cells_;
  int snap_search_radius_cells_;

  // Planner state
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> optimized_local_plan_;
  std::vector<geometry_msgs::PoseStamped> prev_optimized_local_plan_;
  int    target_index_;
  bool   pose_adjusting_;
  bool   goal_reached_;
  std::string last_status_;
  geometry_msgs::Twist last_cmd_vel_;
  std::recursive_mutex planner_mutex_;
};

} // namespace octo_planner

#endif // OCTO_PLANNER_OCTO_LOCAL_PLANNER_H_
