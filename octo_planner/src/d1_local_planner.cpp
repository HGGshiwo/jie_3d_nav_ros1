#include "octo_planner/d1_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <limits>

PLUGINLIB_EXPORT_CLASS(octo_planner::D1LocalPlanner, nav_core::BaseLocalPlanner)

namespace octo_planner
{

D1LocalPlanner::D1LocalPlanner()
: tf_buffer_(nullptr),
  costmap_ros_(nullptr),
  initialized_(false),
  target_index_(0),
  pose_adjusting_(false),
  goal_reached_(true)
{
}

D1LocalPlanner::~D1LocalPlanner()
{
}

void D1LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("D1LocalPlanner has already been initialized, doing nothing.");
    return;
  }

  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;

  ros::NodeHandle private_nh("~/" + name);

  // Read parameters
  private_nh.param<std::string>("map_frame",                    map_frame_,                    "map");
  private_nh.param<std::string>("base_frame",                   base_frame_,                   "base_footprint");
  private_nh.param<std::string>("base_frame_candidates",        base_frame_candidates_str_,    "odin1_base_link,base_link,base_footprint");
  private_nh.param<std::string>("robot_center_offset_frame",    robot_center_offset_frame_,    "odin1_base_link");
  private_nh.param<double>     ("robot_center_offset_x",        robot_center_offset_x_,        -0.18);
  private_nh.param<double>     ("robot_center_offset_y",        robot_center_offset_y_,         0.0);
  private_nh.param<double>     ("robot_center_offset_z",        robot_center_offset_z_,         0.0);
  private_nh.param<double>     ("lookahead_distance",           lookahead_distance_,            0.45);
  private_nh.param<double>     ("tracking_point_reached_xy_tolerance", tracking_xy_tol_,        0.20);
  private_nh.param<double>     ("goal_position_tolerance",      goal_pos_tol_,                  0.05);
  private_nh.param<double>     ("goal_yaw_tolerance",           goal_yaw_tol_,                  0.10);
  private_nh.param<double>     ("linear_gain",                  linear_gain_,                   1.2);
  private_nh.param<double>     ("lateral_gain",                 lateral_gain_,                  0.4);
  private_nh.param<double>     ("heading_gain",                 heading_gain_,                  1.2);
  private_nh.param<double>     ("final_yaw_gain",               final_yaw_gain_,                0.5);
  private_nh.param<bool>       ("enable_lateral_motion",        enable_lateral_motion_,         true);
  private_nh.param<bool>       ("align_final_yaw",              align_final_yaw_,               true);

  VelocitySmootherParams smoother_params;
  private_nh.param<double>("max_linear_speed",          smoother_params.max_linear_speed,          0.60);
  private_nh.param<double>("max_lateral_speed",         smoother_params.max_lateral_speed,         0.30);
  private_nh.param<double>("max_angular_speed",         smoother_params.max_angular_speed,         1.00);
  private_nh.param<double>("max_linear_acc",           smoother_params.max_linear_acc,           0.80);
  private_nh.param<double>("max_lateral_acc",          smoother_params.max_lateral_acc,          0.40);
  private_nh.param<double>("max_angular_acc",          smoother_params.max_angular_acc,          1.20);
  private_nh.param<bool>  ("enable_lateral_decoupling", smoother_params.enable_lateral_decoupling, true);
  private_nh.param<double>("linear_deadband",           smoother_params.linear_deadband,           0.05);
  private_nh.param<double>("lateral_deadband",          smoother_params.lateral_deadband,          0.05);
  private_nh.param<double>("angular_deadband",          smoother_params.angular_deadband,          0.05);
  velocity_smoother_.setParams(smoother_params);

  DebugVisualizerParams vis_params;
  vis_params.map_frame = map_frame_;
  private_nh.param<std::string>("tracking_point_marker_topic",   vis_params.tracking_marker_topic,  "tracking_point_marker");
  private_nh.param<double>     ("tracking_point_marker_scale",   vis_params.tracking_marker_scale,  0.28);
  private_nh.param<bool>       ("enable_tracking_debug_view",    vis_params.enable_debug_view,      true);
  private_nh.param<int>        ("tracking_debug_view_size_px",   vis_params.debug_view_size_px,     640);
  private_nh.param<double>     ("tracking_debug_view_pixels_per_meter", vis_params.debug_ppm,      80.0);
  private_nh.param<double>     ("tracking_debug_view_frequency", debug_view_frequency_,          10.0);
  vis_params.debug_view_frequency = debug_view_frequency_;

  visualizer_.initialize(private_nh, vis_params, "d1_local_planner_debug");

  if (vis_params.enable_debug_view)
  {
    const double debug_period = 1.0 / std::max(1.0, debug_view_frequency_);
    debug_view_timer_ = nh_.createTimer(ros::Duration(debug_period), &D1LocalPlanner::renderTrackingDebugView, this);
  }

  initialized_ = true;
  ROS_INFO("D1LocalPlanner initialized successfully under name: %s", name.c_str());
}

bool D1LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("D1LocalPlanner is not initialized! Call initialize first.");
    return false;
  }

  velocity_smoother_.reset();
  last_control_time_ = ros::Time(0);

  if (plan.empty())
  {
    global_plan_.clear();
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = true;
    visualizer_.clearTrackingPointMarker();
    return true;
  }

  global_plan_ = plan;
  target_index_ = 0;
  pose_adjusting_ = false;
  goal_reached_ = false;
  visualizer_.publishTrackingPointMarker(global_plan_, target_index_);
  ROS_INFO("D1LocalPlanner: Plan set with %zu points.", global_plan_.size());
  return true;
}

bool D1LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("D1LocalPlanner is not initialized!");
    return false;
  }

  if (global_plan_.empty())
  {
    ROS_WARN_THROTTLE(2.0, "D1LocalPlanner: Global plan is empty.");
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  if (goal_reached_)
  {
    cmd_vel = geometry_msgs::Twist();
    return true;
  }

  ros::Time now = ros::Time::now();
  double dt = 0.05;
  if (!last_control_time_.isZero())
  {
    dt = (now - last_control_time_).toSec();
    if (dt <= 1.0e-4 || dt > 0.5) dt = 0.05;
  }
  last_control_time_ = now;

  geometry_msgs::Twist raw_cmd;

  if (pose_adjusting_)
  {
    geometry_msgs::PoseStamped final_pose_base;
    if (!transformToBase(global_plan_.back(), final_pose_base))
    {
      cmd_vel = geometry_msgs::Twist();
      return false;
    }

    raw_cmd.linear.x = final_pose_base.pose.position.x * linear_gain_;
    raw_cmd.linear.y = enable_lateral_motion_ ? final_pose_base.pose.position.y * lateral_gain_ : 0.0;

    double final_yaw_error = 0.0;
    if (align_final_yaw_)
    {
      if (!computeFinalYawErrorXY(global_plan_.back(), final_yaw_error))
      {
        cmd_vel = geometry_msgs::Twist();
        return false;
      }
      raw_cmd.angular.z = final_yaw_error * final_yaw_gain_;
    }

    const bool pos_ok = std::hypot(final_pose_base.pose.position.x, final_pose_base.pose.position.y) < goal_pos_tol_;
    const bool yaw_ok = !align_final_yaw_ || std::abs(final_yaw_error) < goal_yaw_tol_;
    if (pos_ok && yaw_ok)
    {
      goal_reached_ = true;
      visualizer_.clearTrackingPointMarker();
      velocity_smoother_.reset();
      cmd_vel = geometry_msgs::Twist();
      ROS_INFO("D1LocalPlanner: Goal reached.");
      return true;
    }

    cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
    last_cmd_vel_ = cmd_vel;
    return true;
  }

  TrackingTarget target;
  if (!selectTrackingTarget(target))
  {
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  if (isFinalTrackingPointReached(target))
  {
    pose_adjusting_ = true;
    ROS_INFO("D1LocalPlanner: Final tracking point reached. Switching to final yaw adjustment.");
    
    geometry_msgs::PoseStamped final_pose_base;
    if (!transformToBase(global_plan_.back(), final_pose_base))
    {
      cmd_vel = geometry_msgs::Twist();
      return false;
    }

    raw_cmd.linear.x = final_pose_base.pose.position.x * linear_gain_;
    raw_cmd.linear.y = enable_lateral_motion_ ? final_pose_base.pose.position.y * lateral_gain_ : 0.0;

    double final_yaw_error = 0.0;
    if (align_final_yaw_)
    {
      if (!computeFinalYawErrorXY(global_plan_.back(), final_yaw_error))
      {
        cmd_vel = geometry_msgs::Twist();
        return false;
      }
      raw_cmd.angular.z = final_yaw_error * final_yaw_gain_;
    }

    cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
    last_cmd_vel_ = cmd_vel;
    return true;
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

  cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
  last_cmd_vel_ = cmd_vel;

  ROS_INFO_THROTTLE(1.0,
    "D1LocalPlanner: Track target: x=%.3f y=%.3f heading_err=%.3f cmd=(%.3f, %.3f, %.3f)",
    target.base_x, target.base_y, heading_error,
    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  return true;
}

bool D1LocalPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("D1LocalPlanner is not initialized!");
    return false;
  }
  return goal_reached_;
}

bool D1LocalPlanner::isFinalTrackingPointReached(const TrackingTarget & target) const
{
  if (global_plan_.empty() || target_index_ < static_cast<int>(global_plan_.size()) - 3)
    return false;
  RobotPose2D robot_pose;
  if (!const_cast<D1LocalPlanner*>(this)->lookupRobotPose2D(robot_pose)) return false;
  const auto & goal_pos = global_plan_.back().pose.position;
  return std::hypot(goal_pos.x - robot_pose.x, goal_pos.y - robot_pose.y) < tracking_xy_tol_;
}

bool D1LocalPlanner::selectTrackingTarget(TrackingTarget & target)
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

bool D1LocalPlanner::lookupRobotPose2D(RobotPose2D & robot_pose)
{
  std::string last_error;
  for (const auto & base_frame : getBaseFrameCandidates()) {
    try {
      const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame, ros::Time(0), ros::Duration(0.05));
      if (active_base_frame_ != base_frame) {
        active_base_frame_ = base_frame;
        ROS_INFO("D1LocalPlanner: Using robot base frame for tracking: %s", base_frame.c_str());
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
  ROS_WARN_THROTTLE(2.0, "D1LocalPlanner: Lookup robot pose from %s failed for all base_frame candidates. Last: %s",
    map_frame_.c_str(), last_error.c_str());
  return false;
}

bool D1LocalPlanner::computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error)
{
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return false;
  geometry_msgs::PoseStamped final_pose = final_pose_in;
  if (final_pose.header.frame_id.empty()) final_pose.header.frame_id = map_frame_;
  final_pose.header.stamp = ros::Time(0);
  try {
    if (final_pose.header.frame_id != map_frame_)
      tf_buffer_->transform(final_pose, final_pose, map_frame_, ros::Duration(0.05));
  } catch (const tf2::TransformException & ex) {
    ROS_WARN_THROTTLE(2.0, "D1LocalPlanner: Transform final pose yaw %s -> %s failed: %s",
      final_pose.header.frame_id.c_str(), map_frame_.c_str(), ex.what());
    return false;
  }
  yaw_error = D1ControlUtils::normalizeAngle(tf2::getYaw(final_pose.pose.orientation) - robot_pose.yaw);
  return true;
}

bool D1LocalPlanner::transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out)
{
  RobotPose2D unused;
  if (active_base_frame_.empty() && !lookupRobotPose2D(unused)) return false;
  const std::string base_frame = active_base_frame_.empty() ? base_frame_ : active_base_frame_;
  geometry_msgs::PoseStamped stamped = pose_in;
  if (stamped.header.frame_id.empty()) stamped.header.frame_id = map_frame_;
  stamped.header.stamp = ros::Time(0);
  try {
    tf_buffer_->transform(stamped, pose_out, base_frame, ros::Duration(0.05));
    applyRobotCenterOffsetToRelativePose(base_frame, pose_out);
    return true;
  } catch (const tf2::TransformException & ex) {
    ROS_WARN_THROTTLE(2.0, "D1LocalPlanner: Transform %s -> %s failed: %s",
      stamped.header.frame_id.c_str(), base_frame.c_str(), ex.what());
    return false;
  }
}

std::vector<std::string> D1LocalPlanner::getBaseFrameCandidates() const
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

bool D1LocalPlanner::shouldApplyRobotCenterOffset(const std::string & frame) const
{ return frame == robot_center_offset_frame_; }

void D1LocalPlanner::applyRobotCenterOffset(const std::string & frame, RobotPose2D & rp) const
{
  if (!shouldApplyRobotCenterOffset(frame)) return;
  const double cy = std::cos(rp.yaw), sy = std::sin(rp.yaw);
  rp.x += cy * robot_center_offset_x_ - sy * robot_center_offset_y_;
  rp.y += sy * robot_center_offset_x_ + cy * robot_center_offset_y_;
  rp.z += robot_center_offset_z_;
}

void D1LocalPlanner::applyRobotCenterOffsetToRelativePose(const std::string & frame, geometry_msgs::PoseStamped & pose) const
{
  if (!shouldApplyRobotCenterOffset(frame)) return;
  pose.pose.position.x -= robot_center_offset_x_;
  pose.pose.position.y -= robot_center_offset_y_;
  pose.pose.position.z -= robot_center_offset_z_;
}

void D1LocalPlanner::renderTrackingDebugView(const ros::TimerEvent &)
{
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return;
  bool pos_ok = false, yaw_ok = false;
  if (!global_plan_.empty()) {
    geometry_msgs::PoseStamped final_pose_base;
    if (transformToBase(global_plan_.back(), final_pose_base)) {
      pos_ok = std::hypot(final_pose_base.pose.position.x, final_pose_base.pose.position.y) < goal_pos_tol_;
    }
    double yaw_err = 0.0;
    if (computeFinalYawErrorXY(global_plan_.back(), yaw_err)) {
      yaw_ok = !align_final_yaw_ || std::abs(yaw_err) < goal_yaw_tol_;
    }
  }
  visualizer_.renderDebugView(
    robot_pose, global_plan_, target_index_, last_cmd_vel_,
    goal_pos_tol_, goal_yaw_tol_, pos_ok, yaw_ok, pose_adjusting_);
}

} // namespace octo_planner
