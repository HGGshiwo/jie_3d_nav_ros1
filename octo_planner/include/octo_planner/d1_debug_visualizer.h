#ifndef OCTO_PLANNER_D1_DEBUG_VISUALIZER_H_
#define OCTO_PLANNER_D1_DEBUG_VISUALIZER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "octo_planner/d1_control_types.h"

namespace octo_planner
{

struct DebugVisualizerParams
{
  std::string map_frame{"map"};
  std::string tracking_marker_topic{"tracking_point_marker"};
  double tracking_marker_scale{0.28};
  bool enable_debug_view{true};
  int debug_view_size_px{640};
  double debug_ppm{80.0};
  double debug_view_frequency{10.0};
};

class D1DebugVisualizer
{
public:
  D1DebugVisualizer();

  void initialize(ros::NodeHandle & nh, const DebugVisualizerParams & params, const std::string & window_name);

  void publishTrackingPointMarker(const std::vector<geometry_msgs::PoseStamped> & plan, int target_index);
  void clearTrackingPointMarker();

  void renderDebugView(
    const RobotPose2D & robot_pose,
    const std::vector<geometry_msgs::PoseStamped> & plan,
    int target_index,
    const geometry_msgs::Twist & current_cmd_vel,
    double goal_pos_tol = 0.0,
    double goal_yaw_tol = 0.0,
    bool pos_ok = false,
    bool yaw_ok = false,
    bool pose_adjusting = false);

private:
  cv::Point projectPlanPoint(
    const RobotPose2D & rp,
    const geometry_msgs::PoseStamped & pose,
    const cv::Point & center, double ppm) const;

  bool drawFinalGoalYaw(
    cv::Mat & image,
    const RobotPose2D & rp,
    const std::vector<geometry_msgs::PoseStamped> & plan,
    const cv::Point & center, double ppm, double & yaw_error) const;

  DebugVisualizerParams params_;
  ros::Publisher marker_pub_;
  std::string window_name_;
  bool debug_view_disabled_{false};
};

} // namespace octo_planner

#endif // OCTO_PLANNER_D1_DEBUG_VISUALIZER_H_
