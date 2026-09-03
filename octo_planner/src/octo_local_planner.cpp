#include "octo_planner/octo_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <limits>

PLUGINLIB_EXPORT_CLASS(octo_planner::OctoLocalPlanner, nav_core::BaseLocalPlanner)

namespace octo_planner
{

OctoLocalPlanner::OctoLocalPlanner()
: tf_buffer_(nullptr),
  costmap_ros_(nullptr),
  initialized_(false),
  map_ready_(false),
  map_changed_(true),
  last_local_rebuild_time_(0),
  worker_running_(false),
  target_index_(0),
  pose_adjusting_(false),
  goal_reached_(true)
{
}

OctoLocalPlanner::~OctoLocalPlanner()
{
}

void OctoLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("OctoLocalPlanner has already been initialized, doing nothing.");
    return;
  }

  ROS_INFO("OctoLocalPlanner: Initializing 3D local planner [%s]...", name.c_str());

  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;

  ros::NodeHandle private_nh("~/" + name);

  // Frames & offsets
  private_nh.param<std::string>("map_frame",                    map_frame_,                    "map");
  private_nh.param<std::string>("robot_center_offset_frame",    robot_center_offset_frame_,    "odin1_base_link");
  private_nh.param<double>     ("robot_center_offset_x",        robot_center_offset_x_,        -0.18);
  private_nh.param<double>     ("robot_center_offset_y",        robot_center_offset_y_,         0.0);
  private_nh.param<double>     ("robot_center_offset_z",        robot_center_offset_z_,         0.0);

  // Controller parameters
  private_nh.param<double>     ("lookahead_distance",           lookahead_distance_,            0.20);
  private_nh.param<double>     ("tracking_point_reached_xy_tolerance", tracking_xy_tol_,        0.20);
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

  // 3D Planner Parameters
  private_nh.param<double>("robot_radius", robot_radius_, 0.20);
  private_nh.param<bool>("require_ground_support", require_ground_support_, true);
  private_nh.param<bool>("strict_direct_ground_support", strict_direct_ground_support_, false);
  private_nh.param<int>("ground_support_xy_radius_cells", ground_support_xy_radius_cells_, 1);
  private_nh.param<int>("ground_support_depth_cells", ground_support_depth_cells_, 2);
  private_nh.param<int>("max_step_height_cells", max_step_height_cells_, 1);
  private_nh.param<int>("robot_clearance_height_cells", robot_clearance_height_cells_, 0);
  private_nh.param<int>("snap_search_radius_cells", snap_search_radius_cells_, 8);

  planner_.setRobotRadius(robot_radius_);
  planner_.setRequireGroundSupport(require_ground_support_);
  planner_.setStrictDirectGroundSupport(strict_direct_ground_support_);
  planner_.setGroundSupportXYRadiusCells(ground_support_xy_radius_cells_);
  planner_.setGroundSupportDepthCells(ground_support_depth_cells_);
  planner_.setMaxStepHeightCells(max_step_height_cells_);
  planner_.setRobotClearanceHeightCells(robot_clearance_height_cells_);
  planner_.setSnapSearchRadiusCells(snap_search_radius_cells_);

  // Elastic Band Parameters
  ElasticBandParams eb_params;
  private_nh.param<int>("eb_iterations", eb_params.iterations, 40);
  private_nh.param<double>("eb_w_smooth", eb_params.w_smooth, 1.0);
  private_nh.param<double>("eb_w_obstacle", eb_params.w_obstacle, 0.8);
  private_nh.param<double>("eb_w_tangent", eb_params.w_tangent, 0.0); // Disabled by default to prevent chattering
  private_nh.param<double>("eb_w_ground", eb_params.w_ground, 0.4);
  private_nh.param<double>("eb_safe_distance", eb_params.safe_distance, 0.35); // Narrow passage friendly
  private_nh.param<double>("eb_learning_rate", eb_params.learning_rate, 0.03); // Stable learning rate
  eb_params.robot_radius = robot_radius_;
  eb_params.require_ground_support = require_ground_support_;
  eb_params.strict_direct_ground_support = strict_direct_ground_support_;
  eb_params.ground_support_xy_radius_cells = ground_support_xy_radius_cells_;
  eb_params.ground_support_depth_cells = ground_support_depth_cells_;
  eb_params.snap_search_radius_cells = snap_search_radius_cells_;
  elastic_band_.setParams(eb_params);

  // Velocity Smoother Parameters
  VelocitySmootherParams smoother_params;
  smoother_params.max_linear_speed = max_linear_speed_;
  smoother_params.max_lateral_speed = max_lateral_speed_;
  smoother_params.max_angular_speed = max_angular_speed_;
  private_nh.param<double>("max_linear_acc",           smoother_params.max_linear_acc,           0.80);
  private_nh.param<double>("max_lateral_acc",          smoother_params.max_lateral_acc,          0.40);
  private_nh.param<double>("max_angular_acc",          smoother_params.max_angular_acc,          1.20);
  private_nh.param<bool>  ("enable_lateral_decoupling", smoother_params.enable_lateral_decoupling, true);
  smoother_params.linear_deadband = linear_deadband_;
  smoother_params.lateral_deadband = lateral_deadband_;
  smoother_params.angular_deadband = angular_deadband_;
  velocity_smoother_.setParams(smoother_params);

  // Visualizer initialization
  VisualizerParams vis_params;
  vis_params.map_frame = map_frame_;
  private_nh.param<double>("tracking_point_marker_scale", vis_params.tracking_marker_scale, 0.28);
  private_nh.param<bool>("enable_tracking_debug_view", vis_params.enable_debug_view, true);
  private_nh.param<int>("tracking_debug_view_size_px", vis_params.debug_view_size_px, 640);
  private_nh.param<double>("tracking_debug_view_pixels_per_meter", vis_params.debug_ppm, 80.0);
  private_nh.param<double>("tracking_debug_view_frequency", vis_params.debug_view_frequency, 10.0);
  visualizer_.initialize(private_nh, vis_params);

  // Subscribe to octomap
  std::string octomap_topic;
  private_nh.param<std::string>("octomap_topic", octomap_topic, "/octomap_local");
  ros::NodeHandle nh;
  octomap_sub_ = nh.subscribe(octomap_topic, 1, &OctoLocalPlanner::onOctomap, this);

  status_pub_ = nh.advertise<std_msgs::String>("/move_base/status_text", 1, true);

  initialized_ = true;
  ROS_INFO("OctoLocalPlanner initialized with async OcTree processing & modularized architecture, sub to [%s]", octomap_topic.c_str());
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
    planner_.rebuildPreblockedCells(); // Rebuild preblocked cells upon map arrival
    map_ready_ = true;
    map_changed_ = true;
  }
  else
  {
    ROS_WARN_THROTTLE(2.0, "OctoLocalPlanner: OcTree conversion failed.");
  }
  worker_running_ = false;
}

bool OctoLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_) return false;
  std::lock_guard<std::recursive_mutex> lock(planner_mutex_);

  velocity_smoother_.reset();
  last_control_time_ = ros::Time(0);

  if (plan.empty())
  {
    global_plan_.clear();
    optimized_local_plan_.clear();
    prev_optimized_local_plan_.clear();
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = true;
    visualizer_.clearMarkers();
    return true;
  }

  global_plan_ = plan;
  prev_optimized_local_plan_.clear();
  target_index_ = findInitialTargetIndex3D();
  pose_adjusting_ = false;
  goal_reached_ = false;

  visualizer_.publishTrackingPointMarker(global_plan_[target_index_]);
  ROS_INFO("OctoLocalPlanner: New plan set with %zu poses.", global_plan_.size());
  return true;
}

bool OctoLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_) return false;
  std::lock_guard<std::recursive_mutex> lock(planner_mutex_);

  if (global_plan_.empty()) return false;

  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return false;

  ros::Time now = ros::Time::now();
  double dt = 0.05;
  if (!last_control_time_.isZero())
  {
    dt = (now - last_control_time_).toSec();
    if (dt <= 1.0e-4 || dt > 0.5) dt = 0.05;
  }
  last_control_time_ = now;

  // Final goal adjustment
  if (pose_adjusting_)
  {
    geometry_msgs::PoseStamped final_pose_base;
    if (!transformToBase(global_plan_.back(), final_pose_base)) return false;

    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = final_pose_base.pose.position.x * linear_gain_;
    raw_cmd.linear.y = enable_lateral_motion_ 
                       ? final_pose_base.pose.position.y * lateral_gain_
                       : 0.0;

    double final_yaw_error = 0.0;
    if (align_final_yaw_)
    {
      if (!computeFinalYawErrorXY(global_plan_.back(), final_yaw_error)) return false;
      raw_cmd.angular.z = final_yaw_error * final_yaw_gain_;
    }

    const bool pos_ok = std::hypot(final_pose_base.pose.position.x, final_pose_base.pose.position.y) < goal_pos_tol_;
    const bool yaw_ok = !align_final_yaw_ || std::abs(final_yaw_error) < goal_yaw_tol_;

    if (pos_ok && yaw_ok)
    {
      goal_reached_ = true;
      global_plan_.clear();
      optimized_local_plan_.clear();
      prev_optimized_local_plan_.clear();
      velocity_smoother_.reset();
      cmd_vel = geometry_msgs::Twist();
      visualizer_.clearMarkers();
      return true;
    }

    cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
    return true;
  }

  // Extract local band
  int nearest_idx = findInitialTargetIndex3D();
  std::vector<geometry_msgs::PoseStamped> local_band;
  geometry_msgs::PoseStamped current_robot_pose_stamped;
  current_robot_pose_stamped.header.frame_id = map_frame_;
  current_robot_pose_stamped.header.stamp = ros::Time::now();
  current_robot_pose_stamped.pose.position.x = robot_pose.x;
  current_robot_pose_stamped.pose.position.y = robot_pose.y;
  current_robot_pose_stamped.pose.position.z = robot_pose.z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, robot_pose.yaw);
  current_robot_pose_stamped.pose.orientation = tf2::toMsg(q);
  local_band.push_back(current_robot_pose_stamped);

  double accumulated_dist = 0.0;
  int idx = nearest_idx;
  while (idx < static_cast<int>(global_plan_.size()) && accumulated_dist < 1.8 && local_band.size() < 15)
  {
    local_band.push_back(global_plan_[idx]);
    if (idx > nearest_idx)
    {
      double dx = global_plan_[idx].pose.position.x - global_plan_[idx - 1].pose.position.x;
      double dy = global_plan_[idx].pose.position.y - global_plan_[idx - 1].pose.position.y;
      double dz = global_plan_[idx].pose.position.z - global_plan_[idx - 1].pose.position.z;
      accumulated_dist += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    idx++;
  }

  // Optimize local band
  if (local_band.size() >= 3 && map_ready_)
  {
    if (map_changed_ && (now - last_local_rebuild_time_).toSec() >= 0.25)
    {
      planner_.rebuildPreblockedCells();
      map_changed_ = false;
      last_local_rebuild_time_ = now;

      bool need_vis = (visualizer_.getTraversablePub().getNumSubscribers() > 0 ||
                       visualizer_.getPreblockedPub().getNumSubscribers() > 0 ||
                       visualizer_.getRiskCostPub().getNumSubscribers() > 0);
      if (need_vis) {
        visualizer_.publishCellSetMarker(planner_.getTraversableCells(), visualizer_.getTraversablePub(), "traversable_cells", 0.20F, 0.95F, 0.55F, 0.55F, planner_, robot_pose.x, robot_pose.y, robot_pose.yaw);
        visualizer_.publishCellSetMarker(planner_.getPreblockedCells(), visualizer_.getPreblockedPub(), "preblocked_cells", 0.15F, 0.35F, 1.0F, 0.95F, planner_, robot_pose.x, robot_pose.y, robot_pose.yaw);
        visualizer_.publishRiskCostCloud(planner_, robot_pose.x, robot_pose.y, robot_pose.yaw);
      }
    }

    elastic_band_.optimize(local_band, planner_);

    // Temporal inter-frame low-pass filter (EMA)
    if (!prev_optimized_local_plan_.empty() && prev_optimized_local_plan_.size() == local_band.size())
    {
      double d_start = std::hypot(local_band[0].pose.position.x - prev_optimized_local_plan_[0].pose.position.x,
                                  local_band[0].pose.position.y - prev_optimized_local_plan_[0].pose.position.y);
      if (d_start < 0.30)
      {
        double alpha = 0.35; // 35% new optimized, 65% previous frame
        for (size_t i = 1; i < local_band.size() - 1; ++i)
        {
          local_band[i].pose.position.x = alpha * local_band[i].pose.position.x + (1.0 - alpha) * prev_optimized_local_plan_[i].pose.position.x;
          local_band[i].pose.position.y = alpha * local_band[i].pose.position.y + (1.0 - alpha) * prev_optimized_local_plan_[i].pose.position.y;
          local_band[i].pose.position.z = alpha * local_band[i].pose.position.z + (1.0 - alpha) * prev_optimized_local_plan_[i].pose.position.z;
        }
      }
    }

    // Spatial 3-point moving average smoothing
    if (local_band.size() >= 3)
    {
      std::vector<geometry_msgs::PoseStamped> smoothed_band = local_band;
      for (size_t i = 1; i < local_band.size() - 1; ++i)
      {
        smoothed_band[i].pose.position.x = 0.25 * local_band[i-1].pose.position.x + 0.50 * local_band[i].pose.position.x + 0.25 * local_band[i+1].pose.position.x;
        smoothed_band[i].pose.position.y = 0.25 * local_band[i-1].pose.position.y + 0.50 * local_band[i].pose.position.y + 0.25 * local_band[i+1].pose.position.y;
        smoothed_band[i].pose.position.z = 0.25 * local_band[i-1].pose.position.z + 0.50 * local_band[i].pose.position.z + 0.25 * local_band[i+1].pose.position.z;
      }
      local_band = smoothed_band;
    }

    prev_optimized_local_plan_ = local_band;
    optimized_local_plan_ = local_band;
    visualizer_.publishLocalBandMarkers(optimized_local_plan_);
  }
  else
  {
    prev_optimized_local_plan_.clear();
    optimized_local_plan_ = local_band;
    if (!map_ready_)
    {
      ROS_WARN_THROTTLE(2.0, "OctoLocalPlanner: OcTree map is not ready! Skipping Elastic Band optimization.");
    }
  }

  // Select lookahead tracking target
  TrackingTarget target;
  int tracking_idx = 1;
  for (size_t i = 1; i < optimized_local_plan_.size(); ++i)
  {
    double dx = optimized_local_plan_[i].pose.position.x - robot_pose.x;
    double dy = optimized_local_plan_[i].pose.position.y - robot_pose.y;
    tracking_idx = i;
    if (std::hypot(dx, dy) >= lookahead_distance_) break;
  }

  target_index_ = std::min(static_cast<int>(global_plan_.size()) - 1, nearest_idx + tracking_idx - 1);

  geometry_msgs::PoseStamped target_pose_base;
  if (!transformToBase(optimized_local_plan_[tracking_idx], target_pose_base)) return false;

  target.base_x = target_pose_base.pose.position.x;
  target.base_y = target_pose_base.pose.position.y;

  if (target_index_ == static_cast<int>(global_plan_.size()) - 1 && 
      std::hypot(target.base_x, target.base_y) < tracking_xy_tol_)
  {
    pose_adjusting_ = true;
    cmd_vel = geometry_msgs::Twist();
    return true;
  }

  // Control command calculation with acceleration smoothing via D1VelocitySmoother
  geometry_msgs::Twist raw_cmd;
  const double heading_error = std::atan2(target.base_y, std::max(0.05, target.base_x));
  const double heading_factor = std::max(0.2, std::cos(heading_error));
  raw_cmd.linear.x  = target.base_x * linear_gain_ * heading_factor;
  raw_cmd.linear.y  = enable_lateral_motion_ ? target.base_y * lateral_gain_ : 0.0;
  raw_cmd.angular.z = heading_error * heading_gain_ + target.base_y * cross_track_angular_gain_;

  cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);

  // Emergency stop check
  auto octree = planner_.getOctree();
  if (octree)
  {
    double check_dist = robot_radius_ + 0.15;
    bool emergency_stop = false;
    for (double fwd = 0.05; fwd <= check_dist; fwd += 0.08)
    {
      for (double lat = -robot_radius_ * 0.6; lat <= robot_radius_ * 0.6; lat += 0.08)
      {
        double mx = robot_pose.x + std::cos(robot_pose.yaw) * fwd - std::sin(robot_pose.yaw) * lat;
        double my = robot_pose.y + std::sin(robot_pose.yaw) * fwd + std::cos(robot_pose.yaw) * lat;
        octomap::point3d check_p(static_cast<float>(mx), static_cast<float>(my), static_cast<float>(robot_pose.z));
        const octomap::OcTreeNode* node = octree->search(check_p);
        if (node && octree->isNodeOccupied(node)) { emergency_stop = true; break; }
      }
      if (emergency_stop) break;
    }
    if (emergency_stop) {
      ROS_WARN_THROTTLE(1.0, "OctoLocalPlanner: Obstacle detected right in front of robot! Stopping.");
      velocity_smoother_.reset();
      cmd_vel = geometry_msgs::Twist();
    }
  }

  visualizer_.publishTrackingPointMarker(global_plan_[target_index_]);
  return true;
}

int OctoLocalPlanner::findInitialTargetIndex3D()
{
  if (global_plan_.empty()) return 0;
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return 0;

  int start_idx = 0;
  int end_idx = static_cast<int>(global_plan_.size());
  if (target_index_ > 0 && target_index_ < end_idx) {
    start_idx = std::max(0, target_index_ - 30);
    end_idx = std::min(static_cast<int>(global_plan_.size()), target_index_ + 100);
  }

  int nearest = start_idx; double nearest_sq = std::numeric_limits<double>::max();
  for (int i = start_idx; i < end_idx; ++i) {
    const auto & p = global_plan_[i].pose.position;
    const double sq = (p.x - robot_pose.x)*(p.x - robot_pose.x) + (p.y - robot_pose.y)*(p.y - robot_pose.y) + (p.z - robot_pose.z)*(p.z - robot_pose.z);
    if (sq < nearest_sq) { nearest_sq = sq; nearest = i; }
  }
  return nearest;
}

bool OctoLocalPlanner::lookupRobotPose2D(RobotPose2D & robot_pose)
{
  if (!costmap_ros_) return false;
  std::string base_frame = costmap_ros_->getBaseFrameID();
  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame, ros::Time(0), ros::Duration(0.01));
    robot_pose.x   = tf.transform.translation.x;
    robot_pose.y   = tf.transform.translation.y;
    robot_pose.z   = tf.transform.translation.z;
    robot_pose.yaw = tf2::getYaw(tf.transform.rotation);
    applyRobotCenterOffset(base_frame, robot_pose);
    return true;
  } catch (...) {
    return false;
  }
}

bool OctoLocalPlanner::computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error) {
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return false;
  geometry_msgs::PoseStamped final_pose = final_pose_in;
  if (final_pose.header.frame_id.empty()) final_pose.header.frame_id = map_frame_;
  final_pose.header.stamp = ros::Time(0);
  try {
    if (final_pose.header.frame_id != map_frame_)
      tf_buffer_->transform(final_pose, final_pose, map_frame_, ros::Duration(0.05));
  } catch (...) { return false; }
  yaw_error = normalizeAngle(tf2::getYaw(final_pose.pose.orientation) - robot_pose.yaw);
  return true;
}

bool OctoLocalPlanner::transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out) {
  if (!costmap_ros_) return false;
  std::string base_frame = costmap_ros_->getBaseFrameID();
  geometry_msgs::PoseStamped stamped = pose_in;
  if (stamped.header.frame_id.empty()) stamped.header.frame_id = map_frame_;
  stamped.header.stamp = ros::Time(0);
  try {
    tf_buffer_->transform(stamped, pose_out, base_frame, ros::Duration(0.01));
    applyRobotCenterOffsetToRelativePose(base_frame, pose_out);
    return true;
  } catch (...) { return false; }
}

bool OctoLocalPlanner::isGoalReached() { return goal_reached_; }

void OctoLocalPlanner::publishStatus(const std::string& status) {
  last_status_ = status;
  std_msgs::String msg;
  msg.data = status;
  status_pub_.publish(msg);
}

} // namespace octo_planner
