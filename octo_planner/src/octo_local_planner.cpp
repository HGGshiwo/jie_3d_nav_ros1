#include "octo_planner/octo_local_planner.h"
#include <visualization_msgs/MarkerArray.h>
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
: tf_buffer_(nullptr), costmap_ros_(nullptr), initialized_(false), map_ready_(false),
  map_changed_(true), last_local_rebuild_time_(0), worker_running_(false),
  target_index_(0), pose_adjusting_(false), goal_reached_(true) {}

OctoLocalPlanner::~OctoLocalPlanner() {}

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
  private_nh.param<bool>       ("enable_emergency_stop_check", enable_emergency_stop_check_, true);
  private_nh.param<int>        ("emergency_stop_min_occupied_voxels", emergency_stop_min_occupied_voxels_, 3);
  private_nh.param<double>     ("robot_body_height",            robot_body_height_,             0.26);

  // 3D Planner Parameters
  private_nh.param<double>("robot_radius", robot_radius_, 0.20);
  private_nh.param<bool>("require_ground_support", require_ground_support_, true);
  private_nh.param<bool>("strict_direct_ground_support", strict_direct_ground_support_, false);
  private_nh.param<int>("ground_support_xy_radius_cells", ground_support_xy_radius_cells_, 1);
  private_nh.param<int>("ground_support_depth_cells", ground_support_depth_cells_, 2);
  double max_step_height_m = 0.30;
  private_nh.param<double>("max_step_height_m", max_step_height_m, 0.30);
  private_nh.param<double>("max_step_height", max_step_height_m, max_step_height_m);
  private_nh.param<int>("max_step_height_cells", max_step_height_cells_, 1);
  private_nh.param<int>("robot_clearance_height_cells", robot_clearance_height_cells_, 0);
  private_nh.param<int>("snap_search_radius_cells", snap_search_radius_cells_, 8);

  planner_.setRobotRadius(robot_radius_);
  planner_.setRequireGroundSupport(require_ground_support_);
  planner_.setStrictDirectGroundSupport(strict_direct_ground_support_);
  planner_.setGroundSupportXYRadiusCells(ground_support_xy_radius_cells_);
  planner_.setGroundSupportDepthCells(ground_support_depth_cells_);
  planner_.setMaxStepHeightM(max_step_height_m);
  if (private_nh.hasParam("max_step_height_cells") && !private_nh.hasParam("max_step_height_m") && !private_nh.hasParam("max_step_height")) {
    planner_.setMaxStepHeightCells(max_step_height_cells_);
  }
  double robot_height_m = 0.60;
  private_nh.param<double>("robot_height_m", robot_height_m, 0.60);
  planner_.setRobotHeightM(robot_height_m);
  double heuristic_weight = 1.20;
  private_nh.param<double>("heuristic_weight", heuristic_weight, 1.20);
  planner_.setHeuristicWeight(heuristic_weight);
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
  emergency_stop_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("emergency_stop_markers", 1, /*latch=*/true);
  local_plan_pub_ = nh.advertise<nav_msgs::Path>("/move_base/local_plan", 1);

  // TEB Optimizer & Footprint Parameters (Open-Closed Principle: extensible footprint shapes)
  private_nh.param<bool>("use_teb_optimizer", use_teb_optimizer_, true);
  private_nh.param<double>("weight_forbidden_zone", weight_forbidden_zone_, 2.0);
  private_nh.param<double>("footprint_front_offset", footprint_front_offset_, 0.25);
  private_nh.param<double>("footprint_front_radius", footprint_front_radius_, 0.20);
  private_nh.param<double>("footprint_rear_offset", footprint_rear_offset_, 0.25);
  private_nh.param<double>("footprint_rear_radius", footprint_rear_radius_, 0.20);

  footprint_model_ = std::make_shared<TwoSpheresGradientFootprint>(
    footprint_front_offset_, footprint_front_radius_,
    footprint_rear_offset_, footprint_rear_radius_
  );

  // Configure TEB optimal planner parameters
  teb_config_.map_frame = map_frame_;
  teb_config_.robot.max_vel_x = max_linear_speed_;
  teb_config_.robot.max_vel_x_backwards = max_linear_speed_ * 0.5;
  teb_config_.robot.max_vel_y = enable_lateral_motion_ ? max_lateral_speed_ : 0.0;
  teb_config_.robot.max_vel_theta = max_angular_speed_;
  teb_config_.robot.acc_lim_x = smoother_params.max_linear_acc;
  teb_config_.robot.acc_lim_y = enable_lateral_motion_ ? smoother_params.max_lateral_acc : 0.0;
  teb_config_.robot.acc_lim_theta = smoother_params.max_angular_acc;
  teb_config_.trajectory.dt_ref = 0.20;
  teb_config_.trajectory.min_samples = 3;
  teb_config_.trajectory.max_samples = 25;
  teb_config_.optim.no_inner_iterations = 4;
  teb_config_.optim.no_outer_iterations = 3;
  teb_config_.optim.weight_obstacle = 0.0; // Disabled native point obstacles (Method 2)

  teb_planner_ = std::make_unique<OctoTebOptimalPlanner>(teb_config_);
  teb_planner_->initialize(teb_config_);

  initialized_ = true;
  ROS_INFO("OctoLocalPlanner initialized with async OcTree processing & modularized architecture, sub to [%s]", octomap_topic.c_str());
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
    if (local_plan_pub_.getNumSubscribers() > 0)
    {
      nav_msgs::Path empty_path;
      empty_path.header.frame_id = map_frame_;
      empty_path.header.stamp = ros::Time::now();
      local_plan_pub_.publish(empty_path);
    }
    return true;
  }

  global_plan_ = plan;
  prev_optimized_local_plan_.clear();
  target_index_ = findInitialTargetIndex3D();
  pose_adjusting_ = false;
  goal_reached_ = false;

  visualizer_.publishTrackingPointMarker(global_plan_[target_index_]);
  ROS_INFO("OctoLocalPlanner: New plan set with %zu poses, target_idx=%d.", global_plan_.size(), target_index_);
  return true;
}

bool OctoLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  ROS_INFO_THROTTLE(1.0, "[OctoLocalPlanner] computeVelocityCommands CALLED! plan=%zu, init=%d", global_plan_.size(), initialized_);

  if (!initialized_)
  {
    ROS_WARN_THROTTLE(2.0, "[OctoLocalPlanner] computeVelocityCommands failed: not initialized!");
    return false;
  }
  std::lock_guard<std::recursive_mutex> lock(planner_mutex_);

  if (global_plan_.empty())
  {
    ROS_WARN_THROTTLE(2.0, "[OctoLocalPlanner] computeVelocityCommands failed: global_plan is empty!");
    return false;
  }

  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose))
  {
    ROS_WARN_THROTTLE(2.0, "[OctoLocalPlanner] computeVelocityCommands failed: lookupRobotPose2D returned false!");
    return false;
  }

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
      if (local_plan_pub_.getNumSubscribers() > 0)
      {
        nav_msgs::Path empty_path;
        empty_path.header.frame_id = map_frame_;
        empty_path.header.stamp = ros::Time::now();
        local_plan_pub_.publish(empty_path);
      }
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

  // Check if current local band segment is blocked by obstacles; if so, run 3D A* local re-planning
  if (local_band.size() >= 2 && map_ready_) {
    bool is_segment_blocked = false;
    for (size_t i = 0; i + 1 < local_band.size(); ++i) {
      const auto & p1 = local_band[i].pose.position;
      const auto & p2 = local_band[i + 1].pose.position;
      octomap::point3d pt1(static_cast<float>(p1.x), static_cast<float>(p1.y), static_cast<float>(p1.z));
      octomap::point3d pt2(static_cast<float>(p2.x), static_cast<float>(p2.y), static_cast<float>(p2.z));
      if (!planner_.isLineTraversable(pt1, pt2)) {
        is_segment_blocked = true;
        break;
      }
    }

    if (is_segment_blocked) {
      const auto & p_start = local_band.front().pose.position;
      const auto & p_goal  = local_band.back().pose.position;
      std::vector<GridIndex> path_cells;
      std::string error_msg;
      if (planner_.plan(p_start, p_goal, path_cells, error_msg)) {
        auto replanned = planner_.generateSmoothPath(path_cells, local_band.front(), local_band.back(), true);
        if (replanned.size() >= 2) {
          local_band = replanned;
          ROS_INFO_THROTTLE(1.0, "OctoLocalPlanner: Obstacle detected on local segment. Local 3D A* detour generated with %zu nodes.", local_band.size());
        }
      }
    }
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

    visualizer_.publishRawLocalBandMarkers(local_band);

    if (use_teb_optimizer_ && teb_planner_)
    {
      forbidden_field_.updateField(planner_, robot_pose.x, robot_pose.y);
      std::vector<geometry_msgs::PoseStamped> teb_band;
      if (teb_planner_->planWithForbiddenZone(local_band, &last_cmd_vel_, false,
                                             forbidden_field_, *footprint_model_, weight_forbidden_zone_))
      {
        double cur_ground_z = robot_pose.z;
        double base_z_offset = 0.0;
        if (forbidden_field_.getTerrainHeight(robot_pose.x, robot_pose.y, cur_ground_z))
        {
          base_z_offset = robot_pose.z - cur_ground_z;
        }
        teb_planner_->getOptimizedTrajectory3D(forbidden_field_, teb_band, base_z_offset);
        if (teb_band.size() >= 2)
        {
          local_band = teb_band;
        }
      }
      else
      {
        elastic_band_.optimize(local_band, planner_);
      }
    }
    else
    {
      elastic_band_.optimize(local_band, planner_);
    }

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

    prev_optimized_local_plan_ = local_band;
    optimized_local_plan_ = local_band;
    visualizer_.publishLocalBandMarkers(optimized_local_plan_);

    if (local_plan_pub_.getNumSubscribers() > 0)
    {
      nav_msgs::Path local_path_msg;
      local_path_msg.header.frame_id = map_frame_;
      local_path_msg.header.stamp = ros::Time::now();
      local_path_msg.poses = optimized_local_plan_;
      local_plan_pub_.publish(local_path_msg);
    }
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

  // Emergency stop check (delegated to tracking module)
  if (checkEmergencyStop(robot_pose, cmd_vel))
  {
    return true;
  }

  visualizer_.publishTrackingPointMarker(global_plan_[target_index_]);
  ROS_INFO_THROTTLE(1.0, "[OctoLocalPlanner] Active: Band=%zu | Cmd: vx=%.2f, vy=%.2f, wz=%.2f",
                    optimized_local_plan_.size(), cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  return true;
}

} // namespace octo_planner
