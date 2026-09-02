#include "octo_planner/d1_debug_visualizer.h"
#include <tf2/utils.h>

namespace octo_planner
{

D1DebugVisualizer::D1DebugVisualizer()
: window_name_("d1_tracking_debug")
{
}

void D1DebugVisualizer::initialize(ros::NodeHandle & nh, const DebugVisualizerParams & params, const std::string & window_name)
{
  params_ = params;
  window_name_ = window_name;
  marker_pub_ = nh.advertise<visualization_msgs::Marker>(params_.tracking_marker_topic, 1, /*latch=*/true);
}

void D1DebugVisualizer::publishTrackingPointMarker(const std::vector<geometry_msgs::PoseStamped> & plan, int target_index)
{
  if (plan.empty() || target_index < 0 || target_index >= static_cast<int>(plan.size())) {
    clearTrackingPointMarker();
    return;
  }
  visualization_msgs::Marker marker;
  marker.header.frame_id = params_.map_frame;
  marker.header.stamp    = ros::Time::now();
  marker.ns     = "d1_tracking_point";
  marker.id     = 0;
  marker.type   = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose   = plan[static_cast<std::size_t>(target_index)].pose;
  marker.scale.x = marker.scale.y = marker.scale.z = params_.tracking_marker_scale;
  marker.color.r = 0.1f; marker.color.g = 0.65f; marker.color.b = 1.0f; marker.color.a = 0.95f;
  marker_pub_.publish(marker);
}

void D1DebugVisualizer::clearTrackingPointMarker()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = params_.map_frame;
  marker.header.stamp    = ros::Time::now();
  marker.ns     = "d1_tracking_point";
  marker.id     = 0;
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub_.publish(marker);
}

void D1DebugVisualizer::renderDebugView(
  const RobotPose2D & robot_pose,
  const std::vector<geometry_msgs::PoseStamped> & plan,
  int target_index,
  const geometry_msgs::Twist & current_cmd_vel,
  double goal_pos_tol,
  double goal_yaw_tol,
  bool pos_ok,
  bool yaw_ok,
  bool pose_adjusting)
{
  if (!params_.enable_debug_view || debug_view_disabled_ || plan.empty()) return;

  try {
    const int sz = std::max(240, params_.debug_view_size_px);
    const double ppm = std::max(10.0, params_.debug_ppm);
    const cv::Point center(sz / 2, sz / 2);
    cv::Mat image(sz, sz, CV_8UC3, cv::Scalar(18, 24, 28));

    cv::line(image, {center.x, 0}, {center.x, sz}, cv::Scalar(48, 64, 70), 1);
    cv::line(image, {0, center.y}, {sz, center.y}, cv::Scalar(48, 64, 70), 1);
    cv::arrowedLine(image, center, {center.x, center.y - 58}, cv::Scalar(230, 230, 230), 2, cv::LINE_AA, 0, 0.25);
    cv::circle(image, center, 8, cv::Scalar(230, 230, 230), -1, cv::LINE_AA);
    cv::putText(image, "robot +X", {center.x + 10, center.y - 62},
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);

    std::vector<cv::Point> proj;
    proj.reserve(plan.size());
    for (const auto & pose : plan)
      proj.push_back(projectPlanPoint(robot_pose, pose, center, ppm));

    for (std::size_t i = 1; i < proj.size(); ++i)
      cv::line(image, proj[i - 1], proj[i], cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
    for (const auto & pt : proj)
      cv::circle(image, pt, 3, cv::Scalar(90, 210, 90), -1, cv::LINE_AA);

    if (target_index >= 0 && target_index < static_cast<int>(proj.size())) {
      cv::circle(image, proj[target_index], 12, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
      cv::circle(image, proj[target_index],  4, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    }

    double final_yaw_error = 0.0;
    if (drawFinalGoalYaw(image, robot_pose, plan, center, ppm, final_yaw_error)) {
      const auto & goal_p = plan.back().pose.position;
      const double pos_error = std::hypot(goal_p.x - robot_pose.x, goal_p.y - robot_pose.y);
      char buf[160];
      std::snprintf(buf, sizeof(buf), "goal err: pos=%.3fm yaw=%.1f deg (dx=%.2f dy=%.2f)",
                    pos_error, final_yaw_error * 180.0 / M_PI,
                    goal_p.x - robot_pose.x, goal_p.y - robot_pose.y);
      cv::putText(image, buf, {16, 108}, cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(0, 230, 255), 1, cv::LINE_AA);
    }

    cv::putText(image, "tracking index: " + std::to_string(target_index),
      {16, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(80, 190, 255), 2, cv::LINE_AA);
    char pose_text[160];
    std::snprintf(pose_text, sizeof(pose_text), "robot map: x=%.2f y=%.2f yaw=%.1f deg",
      robot_pose.x, robot_pose.y, robot_pose.yaw * 180.0 / M_PI);
    cv::putText(image, pose_text, {16, 56}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    char cmd_text[160];
    std::snprintf(cmd_text, sizeof(cmd_text), "cmd vel: x=%.3f y=%.3f wz=%.3f",
      current_cmd_vel.linear.x, current_cmd_vel.linear.y, current_cmd_vel.angular.z);
    cv::putText(image, cmd_text, {16, 82}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(120, 230, 255), 1, cv::LINE_AA);

    char tol_text[160];
    std::snprintf(tol_text, sizeof(tol_text), "thresholds: pos_tol=%.2fm yaw_tol=%.2frad (%.1fdeg)",
      goal_pos_tol, goal_yaw_tol, goal_yaw_tol * 180.0 / M_PI);
    cv::putText(image, tol_text, {16, 132}, cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 200, 100), 1, cv::LINE_AA);

    char status_text[160];
    std::snprintf(status_text, sizeof(status_text), "status: pos_ok=%d yaw_ok=%d adjusting=%d",
      pos_ok ? 1 : 0, yaw_ok ? 1 : 0, pose_adjusting ? 1 : 0);
    cv::putText(image, status_text, {16, 154}, cv::FONT_HERSHEY_SIMPLEX, 0.50,
      (pos_ok && yaw_ok) ? cv::Scalar(100, 255, 100) : cv::Scalar(100, 150, 255), 1, cv::LINE_AA);

    cv::putText(image, "top = robot forward, red = current target",
      {16, sz - 18}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 200, 210), 1, cv::LINE_AA);

    cv::imshow(window_name_, image);
    cv::waitKey(1);
  } catch (const cv::Exception & ex) {
    debug_view_disabled_ = true;
    ROS_WARN("Disable OpenCV tracking debug view [%s]: %s", window_name_.c_str(), ex.what());
  }
}

cv::Point D1DebugVisualizer::projectPlanPoint(
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

bool D1DebugVisualizer::drawFinalGoalYaw(
  cv::Mat & image,
  const RobotPose2D & rp,
  const std::vector<geometry_msgs::PoseStamped> & plan,
  const cv::Point & center, double ppm, double & yaw_error) const
{
  if (plan.empty()) return false;
  const auto & fp = plan.back();
  const cv::Point fp_px = projectPlanPoint(rp, fp, center, ppm);
  const double goal_yaw = tf2::getYaw(fp.pose.orientation);
  yaw_error = D1ControlUtils::normalizeAngle(goal_yaw - rp.yaw);
  const double arrow_len = std::max(26.0, ppm * 0.35);
  const cv::Point arrow_end(
    static_cast<int>(std::round(fp_px.x - std::sin(yaw_error) * arrow_len)),
    static_cast<int>(std::round(fp_px.y - std::cos(yaw_error) * arrow_len)));
  cv::circle(image, fp_px, 10, cv::Scalar(0, 230, 255), 2, cv::LINE_AA);
  cv::arrowedLine(image, fp_px, arrow_end, cv::Scalar(0, 230, 255), 2, cv::LINE_AA, 0, 0.30);
  return true;
}

} // namespace octo_planner
