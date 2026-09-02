// d1_controller.cpp  —  ROS 1 port of the original ROS 2 implementation
#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "octo_planner/d1_control_types.h"
#include "octo_planner/d1_velocity_smoother.h"
#include "octo_planner/d1_debug_visualizer.h"

using namespace octo_planner;

class D1ControllerNode
{
public:
  D1ControllerNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  : nh_(nh), pnh_(pnh),
    tf_listener_(tf_buffer_),
    target_index_(0),
    pose_adjusting_(false),
    goal_reached_(true)
  {
    // ── parameters ──────────────────────────────────────────────────────
    pnh_.param<std::string>("path_topic",                   path_topic_,                   "/planned_path");
    pnh_.param<std::string>("start_navigation_topic",       start_navigation_topic_,       "/start_navigation");
    pnh_.param<std::string>("stop_navigation_topic",        stop_navigation_topic_,        "/stop_navigation");
    pnh_.param<std::string>("cmd_vel_topic",                cmd_vel_topic_,                "/cmd_vel");
    pnh_.param<std::string>("manual_cmd_vel_topic",         manual_cmd_vel_topic_,         "/web_cmd_vel");
    pnh_.param<std::string>("map_frame",                    map_frame_,                    "map");
    pnh_.param<std::string>("base_frame",                   base_frame_,                   "base_footprint");
    pnh_.param<std::string>("base_frame_candidates",        base_frame_candidates_str_,    "odin1_base_link,base_link,base_footprint");
    pnh_.param<std::string>("robot_center_offset_frame",    robot_center_offset_frame_,    "odin1_base_link");
    pnh_.param<double>     ("robot_center_offset_x",        robot_center_offset_x_,        -0.18);
    pnh_.param<double>     ("robot_center_offset_y",        robot_center_offset_y_,         0.0);
    pnh_.param<double>     ("robot_center_offset_z",        robot_center_offset_z_,         0.0);
    pnh_.param<bool>       ("require_start_command",        require_start_command_,         true);
    pnh_.param<double>     ("control_frequency",            control_frequency_,             20.0);
    pnh_.param<double>     ("lookahead_distance",           lookahead_distance_,            0.45);
    pnh_.param<double>     ("tracking_point_reached_xy_tolerance", tracking_xy_tol_,        0.20);
    pnh_.param<double>     ("goal_position_tolerance",      goal_pos_tol_,                  0.05);
    pnh_.param<double>     ("goal_yaw_tolerance",           goal_yaw_tol_,                  0.10);
    pnh_.param<double>     ("linear_gain",                  linear_gain_,                   1.2);
    pnh_.param<double>     ("lateral_gain",                 lateral_gain_,                  0.4);
    pnh_.param<double>     ("heading_gain",                 heading_gain_,                  1.2);
    pnh_.param<double>     ("final_yaw_gain",               final_yaw_gain_,                0.5);
    pnh_.param<bool>       ("enable_lateral_motion",        enable_lateral_motion_,         true);
    pnh_.param<bool>       ("align_final_yaw",              align_final_yaw_,               true);

    VelocitySmootherParams smoother_params;
    pnh_.param<double>("max_linear_speed",          smoother_params.max_linear_speed,          0.60);
    pnh_.param<double>("max_lateral_speed",         smoother_params.max_lateral_speed,         0.30);
    pnh_.param<double>("max_angular_speed",         smoother_params.max_angular_speed,         1.00);
    pnh_.param<double>("max_linear_acc",           smoother_params.max_linear_acc,           0.80);
    pnh_.param<double>("max_lateral_acc",          smoother_params.max_lateral_acc,          0.40);
    pnh_.param<double>("max_angular_acc",          smoother_params.max_angular_acc,          1.20);
    pnh_.param<bool>  ("enable_lateral_decoupling", smoother_params.enable_lateral_decoupling, true);
    pnh_.param<double>("linear_deadband",           smoother_params.linear_deadband,           0.05);
    pnh_.param<double>("lateral_deadband",          smoother_params.lateral_deadband,          0.05);
    pnh_.param<double>("angular_deadband",          smoother_params.angular_deadband,          0.05);
    velocity_smoother_.setParams(smoother_params);

    DebugVisualizerParams vis_params;
    vis_params.map_frame = map_frame_;
    pnh_.param<std::string>("tracking_point_marker_topic",  vis_params.tracking_marker_topic,   "/tracking_point_marker");
    pnh_.param<double>     ("tracking_point_marker_scale",  vis_params.tracking_marker_scale,   0.28);
    pnh_.param<bool>       ("enable_tracking_debug_view",   vis_params.enable_debug_view,       true);
    pnh_.param<int>        ("tracking_debug_view_size_px",  vis_params.debug_view_size_px,      640);
    pnh_.param<double>     ("tracking_debug_view_pixels_per_meter", vis_params.debug_ppm,       80.0);
    pnh_.param<double>     ("tracking_debug_view_frequency",debug_view_frequency_,           10.0);
    vis_params.debug_view_frequency = debug_view_frequency_;

    visualizer_.initialize(pnh_, vis_params, "d1_controller_xy_tracking");

    // ── subscribers / publishers ─────────────────────────────────────────
    path_sub_     = nh_.subscribe(path_topic_,             1,  &D1ControllerNode::onPath,            this);
    start_nav_sub_= nh_.subscribe(start_navigation_topic_, 10, &D1ControllerNode::onStartNavigation, this);
    stop_nav_sub_ = nh_.subscribe(stop_navigation_topic_,  10, &D1ControllerNode::onStopNavigation,  this);
    manual_sub_   = nh_.subscribe(manual_cmd_vel_topic_,   10, &D1ControllerNode::onManualCmdVel,    this);

    cmd_pub_     = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);

    // ── timers ───────────────────────────────────────────────────────────
    const double ctrl_period  = 1.0 / std::max(1.0, control_frequency_);
    const double debug_period = 1.0 / std::max(1.0, debug_view_frequency_);
    control_timer_    = nh_.createTimer(ros::Duration(ctrl_period),  &D1ControllerNode::onControlTimer,    this);
    debug_view_timer_ = nh_.createTimer(ros::Duration(debug_period), &D1ControllerNode::renderTrackingDebugView, this);

    ROS_INFO(
      "d1_controller started. path=%s start_navigation=%s stop_navigation=%s "
      "cmd_vel=%s manual_cmd_vel=%s map_frame=%s base_frame=%s require_start_command=%s",
      path_topic_.c_str(), start_navigation_topic_.c_str(), stop_navigation_topic_.c_str(),
      cmd_vel_topic_.c_str(), manual_cmd_vel_topic_.c_str(), map_frame_.c_str(), base_frame_.c_str(),
      require_start_command_ ? "true" : "false");
  }

private:
  // ── subscriber callbacks ─────────────────────────────────────────────
  void onPath(const nav_msgs::Path::ConstPtr & msg)
  {
    if (msg->poses.empty()) {
      pending_plan_.clear(); clearActivePlan();
      ROS_WARN("Received empty planned_path.");
      return;
    }
    if (require_start_command_) {
      pending_plan_ = msg->poses; clearActivePlan();
      publishCmd(geometry_msgs::Twist());
      ROS_INFO("Received planned_path with %zu poses. Waiting for start_navigation confirmation.",
        pending_plan_.size());
      return;
    }
    activatePlan(msg->poses);
  }

  void onStartNavigation(const std_msgs::Bool::ConstPtr & msg)
  {
    if (!msg->data) { stopNavigation("Navigation start denied/cancelled. Holding position."); return; }
    if (pending_plan_.empty()) {
      ROS_WARN("Start navigation requested, but no pending planned_path available."); return;
    }
    try {
      activatePlan(pending_plan_); pending_plan_.clear();
    } catch (const std::exception & ex) {
      stopNavigation("Start navigation failed. Holding position.");
      ROS_ERROR("Start navigation exception: %s", ex.what());
    }
  }

  void onStopNavigation(const std_msgs::Bool::ConstPtr & msg)
  {
    if (!msg->data) return;
    stopNavigation("Stop navigation requested. Path tracking aborted and zero velocity sent.");
  }

  void onManualCmdVel(const geometry_msgs::Twist::ConstPtr & msg)
  {
    if (isNavigationActive() && isNonZeroTwist(*msg)) {
      pending_plan_.clear(); clearActivePlan();
      ROS_INFO("Manual web velocity received while navigating. Path tracking aborted.");
    } else if (isNavigationActive()) {
      return;
    }
    publishCmd(*msg);
  }

  // ── plan management ───────────────────────────────────────────────────
  void activatePlan(const std::vector<geometry_msgs::PoseStamped> & plan)
  {
    velocity_smoother_.reset();
    last_control_time_ = ros::Time(0);
    global_plan_   = plan;
    target_index_  = 0;
    pose_adjusting_= false;
    goal_reached_  = global_plan_.empty();
    visualizer_.publishTrackingPointMarker(global_plan_, target_index_);
    ROS_INFO("Navigation execution started with %zu poses.", global_plan_.size());
  }

  void clearActivePlan()
  {
    velocity_smoother_.reset();
    global_plan_.clear(); target_index_ = 0;
    pose_adjusting_ = false; goal_reached_ = true;
    visualizer_.clearTrackingPointMarker();
  }

  void stopNavigation(const char * log_message)
  {
    pending_plan_.clear(); clearActivePlan();
    publishZeroBurst();
    ROS_INFO("%s", log_message);
  }

  bool isNavigationActive() const { return !global_plan_.empty() && !goal_reached_; }

  static bool isNonZeroTwist(const geometry_msgs::Twist & t)
  {
    constexpr double eps = 1.0e-6;
    return std::abs(t.linear.x)  > eps || std::abs(t.linear.y)  > eps ||
           std::abs(t.linear.z)  > eps || std::abs(t.angular.x) > eps ||
           std::abs(t.angular.y) > eps || std::abs(t.angular.z) > eps;
  }

  void publishZeroBurst()
  {
    velocity_smoother_.reset();
    const geometry_msgs::Twist zero;
    for (int i = 0; i < 5; ++i) publishCmd(zero);
  }

  // ── control timer ─────────────────────────────────────────────────────
  void onControlTimer(const ros::TimerEvent &)
  {
    try { onControlTimerImpl(); }
    catch (const std::exception & ex) {
      stopNavigation("Control loop exception. Path tracking aborted.");
      ROS_ERROR("Control loop exception: %s", ex.what());
    }
  }

  void onControlTimerImpl()
  {
    if (global_plan_.empty()) return;

    ros::Time now = ros::Time::now();
    double dt = 1.0 / std::max(1.0, control_frequency_);
    if (!last_control_time_.isZero())
    {
      dt = (now - last_control_time_).toSec();
      if (dt <= 1.0e-4 || dt > 0.5) dt = 1.0 / std::max(1.0, control_frequency_);
    }
    last_control_time_ = now;

    geometry_msgs::Twist raw_cmd;

    if (pose_adjusting_) {
      geometry_msgs::PoseStamped final_pose_base;
      if (!transformToBase(global_plan_.back(), final_pose_base)) return;
      trackFinalPose(final_pose_base, global_plan_.back(), dt);
      return;
    }

    TrackingTarget target;
    if (!selectTrackingTarget(target)) return;

    if (isFinalTrackingPointReached(target)) {
      pose_adjusting_ = true;
      ROS_INFO("Final tracking point reached. Switching to final yaw adjustment.");
      geometry_msgs::PoseStamped final_pose_base;
      if (!transformToBase(global_plan_.back(), final_pose_base)) return;
      trackFinalPose(final_pose_base, global_plan_.back(), dt);
      return;
    }

    // 1. Decoupled Pure Pursuit: smooth heading error calculation
    const double heading_error = std::atan2(target.base_y, std::max(0.05, target.base_x));

    // 2. Smooth cruise forward velocity (cruise speed + cornering adapt + goal ramp down)
    RobotPose2D robot_pose;
    double dist_to_goal = 1.0;
    if (lookupRobotPose2D(robot_pose) && !global_plan_.empty()) {
      const auto & g = global_plan_.back().pose.position;
      dist_to_goal = std::hypot(g.x - robot_pose.x, g.y - robot_pose.y);
    }

    const double cruise_speed = velocity_smoother_.getParams().max_linear_speed;
    const double corner_scale = std::max(0.3, std::cos(heading_error));
    const double goal_scale   = dist_to_goal < 0.6 ? std::max(0.0, dist_to_goal / 0.6) : 1.0;

    raw_cmd.linear.x  = cruise_speed * corner_scale * goal_scale;
    raw_cmd.linear.y  = enable_lateral_motion_ ? target.base_y * lateral_gain_ : 0.0;
    raw_cmd.angular.z = heading_error * heading_gain_;

    const geometry_msgs::Twist cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);

    ROS_INFO_THROTTLE(1.0,
      "Track target in %s: x=%.3f y=%.3f heading_err=%.3f cmd=(%.3f, %.3f, %.3f)",
      base_frame_.c_str(), target.base_x, target.base_y, heading_error,
      cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    publishCmd(cmd_vel);
  }

  // ── tracking helpers ──────────────────────────────────────────────────
  bool isFinalTrackingPointReached(const TrackingTarget & target) const
  {
    if (global_plan_.empty() || target_index_ < static_cast<int>(global_plan_.size()) - 3)
      return false;
    RobotPose2D robot_pose;
    if (!const_cast<D1ControllerNode*>(this)->lookupRobotPose2D(robot_pose)) return false;
    const auto & goal_pos = global_plan_.back().pose.position;
    return std::hypot(goal_pos.x - robot_pose.x, goal_pos.y - robot_pose.y) < tracking_xy_tol_;
  }

  bool selectTrackingTarget(TrackingTarget & target)
  {
    if (global_plan_.empty()) return false;
    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) return false;

    const int prev_idx = target_index_;
    if (!D1ControlUtils::interpolateLookaheadTarget(
          global_plan_, robot_pose, target_index_, lookahead_distance_, target))
    {
      return false;
    }

    if (target_index_ != prev_idx) {
      visualizer_.publishTrackingPointMarker(global_plan_, target_index_);
    }
    return true;
  }

  bool lookupRobotPose2D(RobotPose2D & robot_pose)
  {
    std::string last_error;
    for (const auto & base_frame : getBaseFrameCandidates()) {
      try {
        const auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame, ros::Time(0), ros::Duration(0.05));
        if (active_base_frame_ != base_frame) {
          active_base_frame_ = base_frame;
          ROS_INFO("Using robot base frame for tracking: %s", base_frame.c_str());
        }
        robot_pose.x   = tf.transform.translation.x;
        robot_pose.y   = tf.transform.translation.y;
        robot_pose.z   = tf.transform.translation.z;
        robot_pose.yaw = tf2::getYaw(tf.transform.rotation);
        applyRobotCenterOffset(base_frame, robot_pose);
        return true;
      } catch (const tf2::TransformException & ex) {
        last_error = ex.what();
      }
    }
    ROS_WARN_THROTTLE(2.0, "Lookup robot pose from %s failed for all base_frame candidates. Last: %s",
      map_frame_.c_str(), last_error.c_str());
    return false;
  }

  // ── final pose tracking ───────────────────────────────────────────────
  void trackFinalPose(
    const geometry_msgs::PoseStamped & final_pose_base,
    const geometry_msgs::PoseStamped & final_pose_map,
    double dt)
  {
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = final_pose_base.pose.position.x * linear_gain_;
    raw_cmd.linear.y = enable_lateral_motion_ ? final_pose_base.pose.position.y * lateral_gain_ : 0.0;

    double final_yaw_error = 0.0;
    if (align_final_yaw_) {
      if (!computeFinalYawErrorXY(final_pose_map, final_yaw_error)) return;
      raw_cmd.angular.z = final_yaw_error * final_yaw_gain_;
    }

    const bool pos_ok = std::hypot(final_pose_base.pose.position.x, final_pose_base.pose.position.y) < goal_pos_tol_;
    const bool yaw_ok = !align_final_yaw_ || std::abs(final_yaw_error) < goal_yaw_tol_;
    if (pos_ok && yaw_ok) { finishNavigationAtGoal(); return; }

    const geometry_msgs::Twist cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
    publishCmd(cmd_vel);
  }

  void finishNavigationAtGoal()
  {
    pending_plan_.clear(); clearActivePlan(); publishZeroBurst();
    ROS_INFO("Goal reached within position and yaw tolerances. Navigation finished.");
  }

  bool computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error)
  {
    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) return false;
    geometry_msgs::PoseStamped final_pose = final_pose_in;
    if (final_pose.header.frame_id.empty()) final_pose.header.frame_id = map_frame_;
    final_pose.header.stamp = ros::Time(0);
    try {
      if (final_pose.header.frame_id != map_frame_)
        tf_buffer_.transform(final_pose, final_pose, map_frame_, ros::Duration(0.05));
    } catch (const tf2::TransformException & ex) {
      ROS_WARN_THROTTLE(2.0, "Transform final pose yaw %s -> %s failed: %s",
        final_pose.header.frame_id.c_str(), map_frame_.c_str(), ex.what());
      return false;
    }
    yaw_error = D1ControlUtils::normalizeAngle(tf2::getYaw(final_pose.pose.orientation) - robot_pose.yaw);
    return true;
  }

  // ── transform helpers ─────────────────────────────────────────────────
  bool transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out)
  {
    RobotPose2D unused;
    if (active_base_frame_.empty() && !lookupRobotPose2D(unused)) return false;
    const std::string base_frame = active_base_frame_.empty() ? base_frame_ : active_base_frame_;
    geometry_msgs::PoseStamped stamped = pose_in;
    if (stamped.header.frame_id.empty()) stamped.header.frame_id = map_frame_;
    stamped.header.stamp = ros::Time(0);
    try {
      tf_buffer_.transform(stamped, pose_out, base_frame, ros::Duration(0.05));
      applyRobotCenterOffsetToRelativePose(base_frame, pose_out);
      return true;
    } catch (const tf2::TransformException & ex) {
      ROS_WARN_THROTTLE(2.0, "Transform %s -> %s failed: %s",
        stamped.header.frame_id.c_str(), base_frame.c_str(), ex.what());
      return false;
    }
  }

  std::vector<std::string> getBaseFrameCandidates() const
  {
    std::vector<std::string> candidates;
    const auto add = [&](const std::string & f) {
      if (!f.empty() && std::find(candidates.begin(), candidates.end(), f) == candidates.end())
        candidates.push_back(f);
    };
    add(base_frame_);
    for (const auto & f : D1ControlUtils::splitCsv(base_frame_candidates_str_)) add(f);
    return candidates;
  }

  bool shouldApplyRobotCenterOffset(const std::string & frame) const
  { return frame == robot_center_offset_frame_; }

  void applyRobotCenterOffset(const std::string & frame, RobotPose2D & rp) const
  {
    if (!shouldApplyRobotCenterOffset(frame)) return;
    const double cy = std::cos(rp.yaw), sy = std::sin(rp.yaw);
    rp.x += cy * robot_center_offset_x_ - sy * robot_center_offset_y_;
    rp.y += sy * robot_center_offset_x_ + cy * robot_center_offset_y_;
    rp.z += robot_center_offset_z_;
  }

  void applyRobotCenterOffsetToRelativePose(const std::string & frame, geometry_msgs::PoseStamped & pose) const
  {
    if (!shouldApplyRobotCenterOffset(frame)) return;
    pose.pose.position.x -= robot_center_offset_x_;
    pose.pose.position.y -= robot_center_offset_y_;
    pose.pose.position.z -= robot_center_offset_z_;
  }

  // ── OpenCV debug view ─────────────────────────────────────────────────
  void renderTrackingDebugView(const ros::TimerEvent &)
  {
    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) return;
    visualizer_.renderDebugView(robot_pose, global_plan_, target_index_, last_cmd_vel_);
  }

  // ── publish ───────────────────────────────────────────────────────────
  void publishCmd(const geometry_msgs::Twist & cmd_vel)
  {
    last_cmd_vel_ = cmd_vel;
    cmd_pub_.publish(cmd_vel);
  }

  // ── members ───────────────────────────────────────────────────────────
  ros::NodeHandle & nh_;
  ros::NodeHandle & pnh_;
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber path_sub_, start_nav_sub_, stop_nav_sub_, manual_sub_;
  ros::Publisher  cmd_pub_;
  ros::Timer      control_timer_, debug_view_timer_;

  // parameters
  std::string path_topic_, start_navigation_topic_, stop_navigation_topic_;
  std::string cmd_vel_topic_, manual_cmd_vel_topic_;
  std::string map_frame_, base_frame_, base_frame_candidates_str_;
  std::string robot_center_offset_frame_;
  double robot_center_offset_x_, robot_center_offset_y_, robot_center_offset_z_;
  bool   require_start_command_;
  double control_frequency_, lookahead_distance_, tracking_xy_tol_;
  double debug_view_frequency_;
  double goal_pos_tol_, goal_yaw_tol_;
  double linear_gain_, lateral_gain_, heading_gain_, final_yaw_gain_;
  bool   enable_lateral_motion_;
  bool   align_final_yaw_;

  // components & state
  D1VelocitySmoother velocity_smoother_;
  D1DebugVisualizer visualizer_;
  ros::Time last_control_time_;

  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> pending_plan_;
  int    target_index_;
  bool   pose_adjusting_;
  bool   goal_reached_;
  std::string active_base_frame_;
  geometry_msgs::Twist last_cmd_vel_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "d1_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  D1ControllerNode node(nh, pnh);
  ros::spin();
  return 0;
}