#ifndef OCTO_PLANNER_D1_LOCAL_PLANNER_H_
#define OCTO_PLANNER_D1_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

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
  struct RobotPose2D { double x, y, z, yaw; };
  struct TrackingTarget { double base_x, base_y; };

  // Helper functions
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

  // Visualization & Debug
  void publishTrackingPointMarker();
  void clearTrackingPointMarker();
  void renderTrackingDebugView(const ros::TimerEvent &);
  void renderTrackingDebugViewImpl();
  
  cv::Point projectPlanPoint(
    const RobotPose2D & rp,
    const geometry_msgs::PoseStamped & pose,
    const cv::Point & center, double ppm) const;

  bool drawFinalGoalYaw(
    cv::Mat & image, const RobotPose2D & rp,
    const cv::Point & center, double ppm, double & yaw_error) const;

  // Static math & string helpers
  static std::vector<std::string> splitCsv(const std::string & text);
  static std::string trim(const std::string & text);
  static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
  static double applyDeadband(double v, double db) { return std::abs(v) < db ? 0.0 : v; }
  static double normalizeAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

  // TF & Costmap Pointers (managed by move_base)
  tf2_ros::Buffer* tf_buffer_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  bool initialized_;

  // ROS communications & timers
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  ros::Timer debug_view_timer_;

  // Parameters
  std::string map_frame_, base_frame_, base_frame_candidates_str_;
  std::string robot_center_offset_frame_;
  double robot_center_offset_x_, robot_center_offset_y_, robot_center_offset_z_;
  double lookahead_distance_, tracking_xy_tol_, tracking_marker_scale_;
  bool   enable_debug_view_;
  int    debug_view_size_px_;
  double debug_ppm_, debug_view_frequency_;
  double goal_pos_tol_, goal_yaw_tol_;
  double linear_gain_, lateral_gain_, heading_gain_, cross_track_angular_gain_, final_yaw_gain_;
  bool   enable_lateral_motion_;
  double max_linear_speed_, max_lateral_speed_, max_angular_speed_;
  bool   align_final_yaw_;
  double linear_deadband_, lateral_deadband_, angular_deadband_;

  // Planner state
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  int    target_index_;
  bool   pose_adjusting_;
  bool   goal_reached_;
  bool   debug_view_disabled_;
  std::string active_base_frame_;
  geometry_msgs::Twist last_cmd_vel_;
};

} // namespace octo_planner

#endif // OCTO_PLANNER_D1_LOCAL_PLANNER_H_
