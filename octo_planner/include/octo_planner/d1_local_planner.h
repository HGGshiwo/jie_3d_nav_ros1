#ifndef OCTO_PLANNER_D1_LOCAL_PLANNER_H_
#define OCTO_PLANNER_D1_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>

#include "octo_planner/d1_control_types.h"
#include "octo_planner/d1_velocity_smoother.h"
#include "octo_planner/d1_debug_visualizer.h"

namespace octo_planner
{

class D1LocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  D1LocalPlanner();
  virtual ~D1LocalPlanner();

  // nav_core::BaseLocalPlanner interface methods
  virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  virtual bool isGoalReached() override;

private:
  // Helper tracking functions
  bool isFinalTrackingPointReached(const TrackingTarget & target) const;
  int findInitialTargetIndex3D();
  bool selectTrackingTarget(TrackingTarget & target);
  bool lookupRobotPose2D(RobotPose2D & robot_pose);
  double xyDistanceToPlanPoint(const RobotPose2D & rp, int idx) const;
  
  bool computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error);
  bool transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out);
  
  std::vector<std::string> getBaseFrameCandidates() const;
  bool shouldApplyRobotCenterOffset(const std::string & frame) const;
  void applyRobotCenterOffset(const std::string & frame, RobotPose2D & rp) const;
  void applyRobotCenterOffsetToRelativePose(const std::string & frame, geometry_msgs::PoseStamped & pose) const;

  void renderTrackingDebugView(const ros::TimerEvent &);

  // TF & Costmap Pointers
  tf2_ros::Buffer* tf_buffer_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  bool initialized_;

  // ROS communications & timers
  ros::NodeHandle nh_;
  ros::Timer debug_view_timer_;

  // Parameters
  std::string map_frame_, base_frame_, base_frame_candidates_str_;
  std::string robot_center_offset_frame_;
  double robot_center_offset_x_, robot_center_offset_y_, robot_center_offset_z_;
  double lookahead_distance_, tracking_xy_tol_;
  double goal_pos_tol_, goal_yaw_tol_;
  double linear_gain_, lateral_gain_, heading_gain_, cross_track_angular_gain_, final_yaw_gain_;
  bool   enable_lateral_motion_;
  bool   align_final_yaw_;
  double debug_view_frequency_;

  // Planner components & state
  D1VelocitySmoother velocity_smoother_;
  D1DebugVisualizer visualizer_;
  ros::Time last_control_time_;

  std::vector<geometry_msgs::PoseStamped> global_plan_;
  int    target_index_;
  bool   pose_adjusting_;
  bool   goal_reached_;
  std::string active_base_frame_;
  geometry_msgs::Twist last_cmd_vel_;
};

} // namespace octo_planner

#endif // OCTO_PLANNER_D1_LOCAL_PLANNER_H_
