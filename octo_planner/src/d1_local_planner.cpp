#include "octo_planner/d1_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>

// Register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(octo_planner::D1LocalPlanner, nav_core::BaseLocalPlanner)

namespace octo_planner
{

D1LocalPlanner::D1LocalPlanner()
: tf_buffer_(nullptr),
  costmap_ros_(nullptr),
  initialized_(false),
  target_index_(0),
  pose_adjusting_(false),
  goal_reached_(true),
  debug_view_disabled_(false)
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
  private_nh.param<double>     ("lookahead_distance",           lookahead_distance_,            0.20);
  private_nh.param<double>     ("tracking_point_reached_xy_tolerance", tracking_xy_tol_,        0.20);
  private_nh.param<double>     ("tracking_point_marker_scale",  tracking_marker_scale_,         0.28);
  private_nh.param<bool>       ("enable_tracking_debug_view",   enable_debug_view_,             true);
  private_nh.param<int>        ("tracking_debug_view_size_px",  debug_view_size_px_,            640);
  private_nh.param<double>     ("tracking_debug_view_pixels_per_meter", debug_ppm_,             80.0);
  private_nh.param<double>     ("tracking_debug_view_frequency",debug_view_frequency_,          10.0);
  private_nh.param<double>     ("goal_position_tolerance",      goal_pos_tol_,                  0.05);
  private_nh.param<double>     ("goal_yaw_tolerance",           goal_yaw_tol_,                  0.10);
  private_nh.param<double>     ("linear_gain",                  linear_gain_,                   1.5);
  private_nh.param<double>     ("lateral_gain",                 lateral_gain_,                  1.5);
  private_nh.param<double>     ("heading_gain",                 heading_gain_,                  2.5);
  private_nh.param<double>     ("cross_track_angular_gain",     cross_track_angular_gain_,      1.0);
  private_nh.param<double>     ("final_yaw_gain",               final_yaw_gain_,                0.5);
  private_nh.param<bool>       ("enable_lateral_motion",        enable_lateral_motion_,         true);
  private_nh.param<double>     ("max_linear_speed",             max_linear_speed_,              0.60);
  private_nh.param<double>     ("max_lateral_speed",            max_lateral_speed_,             0.60);
  private_nh.param<double>     ("max_angular_speed",            max_angular_speed_,             1.50);
  private_nh.param<bool>       ("align_final_yaw",              align_final_yaw_,               true);
  private_nh.param<double>     ("linear_deadband",              linear_deadband_,               0.05);
  private_nh.param<double>     ("lateral_deadband",             lateral_deadband_,              0.05);
  private_nh.param<double>     ("angular_deadband",             angular_deadband_,              0.05);

  std::string tracking_marker_topic;
  private_nh.param<std::string>("tracking_point_marker_topic",  tracking_marker_topic,        "tracking_point_marker");
  marker_pub_ = private_nh.advertise<visualization_msgs::Marker>(tracking_marker_topic, 1, /*latch=*/true);

  if (enable_debug_view_)
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

  if (plan.empty())
  {
    global_plan_.clear();
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = true;
    clearTrackingPointMarker();
    return true;
  }

  global_plan_ = plan;
  target_index_ = findInitialTargetIndex3D();
  pose_adjusting_ = false;
  goal_reached_ = false;
  publishTrackingPointMarker();
  ROS_INFO("D1LocalPlanner: Plan set with %zu points. Initial target index: %d", global_plan_.size(), target_index_);
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

  if (pose_adjusting_)
  {
    geometry_msgs::PoseStamped final_pose_base;
    if (!transformToBase(global_plan_.back(), final_pose_base))
    {
      cmd_vel = geometry_msgs::Twist();
      return false;
    }

    cmd_vel.linear.x = clamp(final_pose_base.pose.position.x * linear_gain_,  -max_linear_speed_,  max_linear_speed_);
    cmd_vel.linear.y = enable_lateral_motion_
                       ? clamp(final_pose_base.pose.position.y * lateral_gain_, -max_lateral_speed_, max_lateral_speed_)
                       : 0.0;
    cmd_vel.linear.x = applyDeadband(cmd_vel.linear.x, linear_deadband_);
    cmd_vel.linear.y = applyDeadband(cmd_vel.linear.y, lateral_deadband_);

    double final_yaw_error = 0.0;
    if (align_final_yaw_)
    {
      if (!computeFinalYawErrorXY(global_plan_.back(), final_yaw_error))
      {
        cmd_vel = geometry_msgs::Twist();
        return false;
      }
      cmd_vel.angular.z = clamp(final_yaw_error * final_yaw_gain_, -max_angular_speed_, max_angular_speed_);
      cmd_vel.angular.z = applyDeadband(cmd_vel.angular.z, angular_deadband_);
    }

    const bool pos_ok = std::hypot(final_pose_base.pose.position.x, final_pose_base.pose.position.y) < goal_pos_tol_;
    const bool yaw_ok = !align_final_yaw_ || std::abs(final_yaw_error) < goal_yaw_tol_;
    if (pos_ok && yaw_ok)
    {
      goal_reached_ = true;
      clearTrackingPointMarker();
      cmd_vel = geometry_msgs::Twist();
      ROS_INFO("D1LocalPlanner: Goal reached.");
    }
    
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

    cmd_vel.linear.x = clamp(final_pose_base.pose.position.x * linear_gain_,  -max_linear_speed_,  max_linear_speed_);
    cmd_vel.linear.y = enable_lateral_motion_
                       ? clamp(final_pose_base.pose.position.y * lateral_gain_, -max_lateral_speed_, max_lateral_speed_)
                       : 0.0;
    cmd_vel.linear.x = applyDeadband(cmd_vel.linear.x, linear_deadband_);
    cmd_vel.linear.y = applyDeadband(cmd_vel.linear.y, lateral_deadband_);

    double final_yaw_error = 0.0;
    if (align_final_yaw_)
    {
      if (!computeFinalYawErrorXY(global_plan_.back(), final_yaw_error))
      {
        cmd_vel = geometry_msgs::Twist();
        return false;
      }
      cmd_vel.angular.z = clamp(final_yaw_error * final_yaw_gain_, -max_angular_speed_, max_angular_speed_);
      cmd_vel.angular.z = applyDeadband(cmd_vel.angular.z, angular_deadband_);
    }

    const bool pos_ok = std::hypot(final_pose_base.pose.position.x, final_pose_base.pose.position.y) < goal_pos_tol_;
    const bool yaw_ok = !align_final_yaw_ || std::abs(final_yaw_error) < goal_yaw_tol_;
    if (pos_ok && yaw_ok)
    {
      goal_reached_ = true;
      clearTrackingPointMarker();
      cmd_vel = geometry_msgs::Twist();
      ROS_INFO("D1LocalPlanner: Goal reached.");
    }
    
    last_cmd_vel_ = cmd_vel;
    return true;
  }

  const double heading_error = std::atan2(target.base_y, std::max(1.0e-6, target.base_x));
  cmd_vel.linear.x  = clamp(target.base_x * linear_gain_,  -max_linear_speed_,  max_linear_speed_);
  cmd_vel.linear.y  = enable_lateral_motion_
                      ? clamp(target.base_y * lateral_gain_, -max_lateral_speed_, max_lateral_speed_)
                      : 0.0;
  cmd_vel.angular.z = clamp(heading_error * heading_gain_ + target.base_y * cross_track_angular_gain_,
                            -max_angular_speed_, max_angular_speed_);
  cmd_vel.linear.x  = applyDeadband(cmd_vel.linear.x,  linear_deadband_);
  cmd_vel.linear.y  = applyDeadband(cmd_vel.linear.y,  lateral_deadband_);
  cmd_vel.angular.z = applyDeadband(cmd_vel.angular.z, angular_deadband_);

  ROS_INFO_THROTTLE(1.0,
    "D1LocalPlanner: Track target: x=%.3f y=%.3f heading_err=%.3f cmd=(%.3f, %.3f, %.3f)",
    target.base_x, target.base_y, heading_error,
    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  last_cmd_vel_ = cmd_vel;
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
  if (global_plan_.empty() || target_index_ != static_cast<int>(global_plan_.size()) - 1)
    return false;
  return std::hypot(target.base_x, target.base_y) < tracking_xy_tol_;
}

int D1LocalPlanner::findInitialTargetIndex3D()
{
  if (global_plan_.empty()) return 0;
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) {
    ROS_WARN("D1LocalPlanner: Failed to get robot pose. Start tracking from path index 0."); return 0;
  }
  int nearest = 0; double nearest_sq = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < global_plan_.size(); ++i) {
    const auto & p = global_plan_[i].pose.position;
    const double dx = p.x - robot_pose.x, dy = p.y - robot_pose.y, dz = p.z - robot_pose.z;
    const double sq = dx*dx + dy*dy + dz*dz;
    if (sq < nearest_sq) { nearest_sq = sq; nearest = static_cast<int>(i); }
  }
  return nearest;
}

bool D1LocalPlanner::selectTrackingTarget(TrackingTarget & target)
{
  if (global_plan_.empty()) return false;
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return false;

  const double reached_tol = tracking_xy_tol_;
  if (xyDistanceToPlanPoint(robot_pose, target_index_) < reached_tol
      && target_index_ < static_cast<int>(global_plan_.size()) - 1)
  {
    int next = target_index_;
    for (int i = target_index_ + 1; i < static_cast<int>(global_plan_.size()); ++i) {
      if (xyDistanceToPlanPoint(robot_pose, i) > reached_tol) { next = i; break; }
      if (i == static_cast<int>(global_plan_.size()) - 1) next = i;
    }
    if (next != target_index_) { target_index_ = next; publishTrackingPointMarker(); }
  }

  const auto & tp = global_plan_[static_cast<std::size_t>(target_index_)].pose.position;
  const double dx_map = tp.x - robot_pose.x, dy_map = tp.y - robot_pose.y;
  const double cy = std::cos(robot_pose.yaw), sy = std::sin(robot_pose.yaw);
  target.base_x =  cy * dx_map + sy * dy_map;
  target.base_y = -sy * dx_map + cy * dy_map;
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

double D1LocalPlanner::xyDistanceToPlanPoint(const RobotPose2D & rp, int idx) const
{
  const auto & p = global_plan_[static_cast<std::size_t>(idx)].pose.position;
  return std::hypot(p.x - rp.x, p.y - rp.y);
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
  yaw_error = normalizeAngle(tf2::getYaw(final_pose.pose.orientation) - robot_pose.yaw);
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
  for (const auto & f : splitCsv(base_frame_candidates_str_)) add(f);
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

void D1LocalPlanner::publishTrackingPointMarker()
{
  if (global_plan_.empty()) { clearTrackingPointMarker(); return; }
  visualization_msgs::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp    = ros::Time::now();
  marker.ns     = "d1_tracking_point";
  marker.id     = 0;
  marker.type   = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose   = global_plan_[static_cast<std::size_t>(target_index_)].pose;
  marker.scale.x = marker.scale.y = marker.scale.z = tracking_marker_scale_;
  marker.color.r = 0.1f; marker.color.g = 0.65f; marker.color.b = 1.0f; marker.color.a = 0.95f;
  marker_pub_.publish(marker);
}

void D1LocalPlanner::clearTrackingPointMarker()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp    = ros::Time::now();
  marker.ns     = "d1_tracking_point";
  marker.id     = 0;
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub_.publish(marker);
}

void D1LocalPlanner::renderTrackingDebugView(const ros::TimerEvent &)
{
  try { renderTrackingDebugViewImpl(); }
  catch (const std::exception & ex) {
    ROS_WARN_THROTTLE(2.0, "D1LocalPlanner: OpenCV tracking debug view exception: %s", ex.what());
  }
}

void D1LocalPlanner::renderTrackingDebugViewImpl()
{
  if (!enable_debug_view_ || debug_view_disabled_ || global_plan_.empty()) return;
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return;

  const int sz = std::max(240, debug_view_size_px_);
  const double ppm = std::max(10.0, debug_ppm_);
  const cv::Point center(sz / 2, sz / 2);
  cv::Mat image(sz, sz, CV_8UC3, cv::Scalar(18, 24, 28));

  cv::line(image, {center.x, 0}, {center.x, sz}, cv::Scalar(48, 64, 70), 1);
  cv::line(image, {0, center.y}, {sz, center.y}, cv::Scalar(48, 64, 70), 1);
  cv::arrowedLine(image, center, {center.x, center.y - 58}, cv::Scalar(230, 230, 230), 2, cv::LINE_AA, 0, 0.25);
  cv::circle(image, center, 8, cv::Scalar(230, 230, 230), -1, cv::LINE_AA);
  cv::putText(image, "robot +X", {center.x + 10, center.y - 62},
    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);

  std::vector<cv::Point> proj;
  proj.reserve(global_plan_.size());
  for (const auto & pose : global_plan_)
    proj.push_back(projectPlanPoint(robot_pose, pose, center, ppm));

  for (std::size_t i = 1; i < proj.size(); ++i)
    cv::line(image, proj[i - 1], proj[i], cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
  for (const auto & pt : proj)
    cv::circle(image, pt, 3, cv::Scalar(90, 210, 90), -1, cv::LINE_AA);

  if (target_index_ >= 0 && target_index_ < static_cast<int>(proj.size())) {
    cv::circle(image, proj[target_index_], 12, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    cv::circle(image, proj[target_index_],  4, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
  }

  double final_yaw_error = 0.0;
  if (drawFinalGoalYaw(image, robot_pose, center, ppm, final_yaw_error)) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "goal yaw err: %.1f deg", final_yaw_error * 180.0 / M_PI);
    cv::putText(image, buf, {16, 108}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 230, 255), 1, cv::LINE_AA);
  }

  cv::putText(image, "tracking index: " + std::to_string(target_index_),
    {16, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(80, 190, 255), 2, cv::LINE_AA);
  char pose_text[160];
  std::snprintf(pose_text, sizeof(pose_text), "robot map: x=%.2f y=%.2f yaw=%.1f deg",
    robot_pose.x, robot_pose.y, robot_pose.yaw * 180.0 / M_PI);
  cv::putText(image, pose_text, {16, 56}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
  char cmd_text[160];
  std::snprintf(cmd_text, sizeof(cmd_text), "cmd vel: x=%.3f y=%.3f wz=%.3f",
    last_cmd_vel_.linear.x, last_cmd_vel_.linear.y, last_cmd_vel_.angular.z);
  cv::putText(image, cmd_text, {16, 82}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(120, 230, 255), 1, cv::LINE_AA);
  cv::putText(image, "top = robot forward, red = current target",
    {16, sz - 18}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 200, 210), 1, cv::LINE_AA);

  try {
    cv::imshow("d1_local_planner_debug", image);
    cv::waitKey(1);
  } catch (const cv::Exception & ex) {
    debug_view_disabled_ = true;
    ROS_WARN("D1LocalPlanner: Disable OpenCV tracking debug view: %s", ex.what());
  }
}

cv::Point D1LocalPlanner::projectPlanPoint(
  const RobotPose2D & rp,
  const geometry_msgs::PoseStamped & pose,
  const cv::Point & center, double ppm) const
{
  const double dx = pose.pose.position.x - rp.x, dy = pose.pose.position.y - rp.y;
  const double cy = std::cos(rp.yaw), sy = std::sin(rp.yaw);
  const double base_x = cy * dx + sy * dy, base_y = -sy * dx + cy * dy;
  return cv::Point(
    static_cast<int>(std::round(center.x - base_y * ppm)),
    static_cast<int>(std::round(center.y - base_x * ppm)));
}

bool D1LocalPlanner::drawFinalGoalYaw(
  cv::Mat & image, const RobotPose2D & rp,
  const cv::Point & center, double ppm, double & yaw_error) const
{
  if (global_plan_.empty()) return false;
  const auto & fp = global_plan_.back();
  const cv::Point fp_px = projectPlanPoint(rp, fp, center, ppm);
  const double goal_yaw = tf2::getYaw(fp.pose.orientation);
  yaw_error = normalizeAngle(goal_yaw - rp.yaw);
  const double arrow_len = std::max(26.0, ppm * 0.35);
  const cv::Point arrow_end(
    static_cast<int>(std::round(fp_px.x - std::sin(yaw_error) * arrow_len)),
    static_cast<int>(std::round(fp_px.y - std::cos(yaw_error) * arrow_len)));
  cv::circle(image, fp_px, 10, cv::Scalar(0, 230, 255), 2, cv::LINE_AA);
  cv::arrowedLine(image, fp_px, arrow_end, cv::Scalar(0, 230, 255), 2, cv::LINE_AA, 0, 0.30);
  return true;
}

std::vector<std::string> D1LocalPlanner::splitCsv(const std::string & text)
{
  std::vector<std::string> parts; std::string cur;
  for (const char ch : text) {
    if (ch == ',') { const auto t = trim(cur); if (!t.empty()) parts.push_back(t); cur.clear(); }
    else cur.push_back(ch);
  }
  const auto t = trim(cur); if (!t.empty()) parts.push_back(t);
  return parts;
}

std::string D1LocalPlanner::trim(const std::string & text)
{
  std::size_t first = 0;
  while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first]))) ++first;
  std::size_t last = text.size();
  while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1]))) --last;
  return text.substr(first, last - first);
}

} // namespace octo_planner
