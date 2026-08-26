#ifndef OCTO_PLANNER_OCTO_LOCAL_PLANNER_H_
#define OCTO_PLANNER_OCTO_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "octo_planner/octo_planner_core.h"

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

  // 3D Elastic Band Optimization
  void optimizeElasticBand(std::vector<geometry_msgs::PoseStamped>& band);
  bool getDistanceAndGradient(const octomap::point3d& p, const std::vector<octomap::point3d>& obstacles, double& distance, octomap::point3d& gradient);

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
  void publishLocalBandMarkers(const std::vector<geometry_msgs::PoseStamped>& band);
  void clearMarkers();
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
  static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
  static double applyDeadband(double v, double db) { return std::abs(v) < db ? 0.0 : v; }
  static double normalizeAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

  // TF & Costmap Pointers
  tf2_ros::Buffer* tf_buffer_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  bool initialized_;
  bool map_ready_;

  // ROS communications & timers
  ros::NodeHandle nh_;
  ros::Subscriber octomap_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher band_marker_pub_;
  ros::Publisher status_pub_;
  ros::Timer debug_view_timer_;

  // OctoPlannerCore instance for local traversability & ground checks
  OctoPlannerCore planner_;

  // Parameters
  std::string map_frame_;
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

  // 3D Planner Parameters
  double robot_radius_;
  bool require_ground_support_;
  bool strict_direct_ground_support_;
  int ground_support_xy_radius_cells_;
  int ground_support_depth_cells_;
  int max_step_height_cells_;
  int robot_clearance_height_cells_;
  int snap_search_radius_cells_;

  // Elastic Band Parameters
  int eb_iterations_;
  double eb_w_smooth_;
  double eb_w_obstacle_;
  double eb_w_ground_;
  double eb_safe_distance_;
  double eb_learning_rate_;

  // Helper status function
  void publishStatus(const std::string& status);

  // Planner state
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> optimized_local_plan_;
  int    target_index_;
  bool   pose_adjusting_;
  bool   goal_reached_;
  bool   debug_view_disabled_;
  std::string last_status_;
  geometry_msgs::Twist last_cmd_vel_;
  std::recursive_mutex planner_mutex_;
};

} // namespace octo_planner

#endif // OCTO_PLANNER_OCTO_LOCAL_PLANNER_H_
