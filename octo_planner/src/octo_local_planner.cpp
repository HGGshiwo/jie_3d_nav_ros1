#include "octo_planner/octo_local_planner.h"
#include "octo_planner/local_planner_profiler.h"
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
  if (initialized_) {
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
  private_nh.param<double>     ("goal_z_tolerance",             goal_z_tol_,                    0.25);
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
  private_nh.param<int>("robot_clearance_height_cells", robot_clearance_height_cells_, 0);
  private_nh.param<int>("snap_search_radius_cells", snap_search_radius_cells_, 8);

  double robot_height_m = 0.60;
  private_nh.param<double>("robot_height_m", robot_height_m, 0.60);
  private_nh.param<double>("robot_height", robot_height_m, robot_height_m);
  double heuristic_weight = 1.20;
  private_nh.param<double>("heuristic_weight", heuristic_weight, 1.20);

  bool enable_preblocked_costmap = true;
  private_nh.param<bool>("enable_preblocked_costmap", enable_preblocked_costmap, true);
  int preblocked_costmap_radius_cells = 3;
  private_nh.param<int>("preblocked_costmap_radius_cells", preblocked_costmap_radius_cells, 3);
  double preblocked_costmap_weight = 1.5;
  private_nh.param<double>("preblocked_costmap_weight", preblocked_costmap_weight, 1.5);
  bool lowest_traversable_only = false;
  private_nh.param<bool>("lowest_traversable_only", lowest_traversable_only, false);
  bool enable_path_shortcut = true;
  private_nh.param<bool>("enable_path_shortcut", enable_path_shortcut, true);
  bool enable_path_smoothing = true;
  private_nh.param<bool>("enable_path_smoothing", enable_path_smoothing, true);
  double path_interpolation_resolution = 0.05;
  private_nh.param<double>("path_interpolation_resolution", path_interpolation_resolution, 0.05);
  double corner_fillet_radius = 0.30;
  private_nh.param<double>("corner_fillet_radius", corner_fillet_radius, 0.30);
  bool enable_continuous_yaw = true;
  private_nh.param<bool>("enable_continuous_yaw", enable_continuous_yaw, true);
  int yaw_smoothing_window = 5;
  private_nh.param<int>("yaw_smoothing_window", yaw_smoothing_window, 5);
  local_max_iterations_ = 2000;
  private_nh.param<int>("local_max_iterations", local_max_iterations_, 2000);

  auto configCore = [&](OctoPlannerCore& core) {
    core.setRobotRadius(robot_radius_);
    core.setRequireGroundSupport(require_ground_support_);
    core.setStrictDirectGroundSupport(strict_direct_ground_support_);
    core.setGroundSupportXYRadiusCells(ground_support_xy_radius_cells_);
    core.setGroundSupportDepthCells(ground_support_depth_cells_);
    core.setMaxStepHeightM(max_step_height_m);
    core.setRobotHeightM(robot_height_m);
    core.setHeuristicWeight(heuristic_weight);
    core.setRobotClearanceHeightCells(robot_clearance_height_cells_);
    core.setSnapSearchRadiusCells(snap_search_radius_cells_);
    core.setEnablePreblockedCostmap(enable_preblocked_costmap);
    core.setPreblockedCostmapRadiusCells(preblocked_costmap_radius_cells);
    core.setPreblockedCostmapWeight(preblocked_costmap_weight);
    core.setLowestTraversableOnly(lowest_traversable_only);
    core.setEnablePathShortcut(enable_path_shortcut);
    core.setEnablePathSmoothing(enable_path_smoothing);
    core.setPathInterpolationResolution(path_interpolation_resolution);
    core.setCornerFilletRadius(corner_fillet_radius);
    core.setEnableContinuousYaw(enable_continuous_yaw);
    core.setYawSmoothingWindow(yaw_smoothing_window);
    core.setMaxIterations(local_max_iterations_);
  };
  configCore(planner_);
  configCore(bg_planner_);
  // Elastic Band Parameters
  ElasticBandParams eb_params;
  private_nh.param<int>("eb_iterations", eb_params.iterations, 40);
  private_nh.param<double>("eb_w_smooth", eb_params.w_smooth, 1.0);
  private_nh.param<double>("eb_w_obstacle", eb_params.w_obstacle, 0.8);
  private_nh.param<double>("eb_w_tangent", eb_params.w_tangent, 0.0);
  private_nh.param<double>("eb_w_ground", eb_params.w_ground, 0.4);
  private_nh.param<double>("eb_safe_distance", eb_params.safe_distance, 0.35);
  private_nh.param<double>("eb_learning_rate", eb_params.learning_rate, 0.03);
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
  private_nh.param<double>("local_planner_horizon", local_planner_horizon_, 2.20);
  private_nh.param<double>("teb_planning_horizon",  teb_planning_horizon_,  1.00);

  // Subscribe to octomap
  std::string octomap_topic;
  private_nh.param<std::string>("octomap_topic", octomap_topic, "/octomap_local");
  ros::NodeHandle nh;
  octomap_sub_ = nh.subscribe(octomap_topic, 1, &OctoLocalPlanner::onOctomap, this);
  status_pub_ = nh.advertise<std_msgs::String>("/move_base/status_text", 1, true);
  emergency_stop_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("emergency_stop_markers", 1, /*latch=*/true);
  local_plan_pub_ = nh.advertise<nav_msgs::Path>("/move_base/local_plan", 1);
  local_astar_plan_pub_ = nh.advertise<nav_msgs::Path>("/move_base/local_astar_plan", 1);
  // TEB Optimizer & Footprint Parameters
  private_nh.param<bool>("use_teb_optimizer", use_teb_optimizer_, true);
  private_nh.param<double>("weight_forbidden_zone", weight_forbidden_zone_, 2.0);
  double forbidden_zone_resolution = 0.10;
  private_nh.param<double>("forbidden_zone_resolution", forbidden_zone_resolution, 0.10);
  ForbiddenZoneParams fz_params;
  fz_params.resolution = forbidden_zone_resolution;
  private_nh.param<double>("exterior_gradient_alpha", fz_params.exterior_gradient_alpha, 3.0);
  private_nh.param<double>("exterior_cost_base", fz_params.exterior_cost_base, 2.0);
  forbidden_field_.setParams(fz_params);

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
  teb_config_.trajectory.global_plan_overwrite_orientation = true;
  teb_config_.optim.no_inner_iterations = 4;
  teb_config_.optim.no_outer_iterations = 3;
  teb_config_.optim.weight_obstacle = 0.0; // Disabled native point obstacles

  teb_planner_ = std::make_unique<OctoTebOptimalPlanner>(teb_config_);
  teb_planner_->initialize(teb_config_);

  initialized_ = true;

  ros::NodeHandle move_base_nh("~");
  double mb_ctrl_freq = 20.0;
  if (move_base_nh.getParam("controller_frequency", mb_ctrl_freq))
  {
    rate_monitor_.expected_controller_freq = mb_ctrl_freq;
    ROS_INFO("[OctoLocalPlanner] Detected move_base controller_frequency: %.2f Hz", mb_ctrl_freq);
    if (mb_ctrl_freq <= 2.0)
    {
      ROS_WARN("[OctoLocalPlanner] >>> CRITICAL WARNING: move_base controller_frequency is set to %.2f Hz! Check your move_base launch / yaml configuration! <<<", mb_ctrl_freq);
    }
  }

  ROS_INFO("OctoLocalPlanner initialized with async OcTree processing & modularized architecture, sub to [%s]", octomap_topic.c_str());
}

void OctoLocalPlanner::resetPlanState()
{
  global_plan_.clear();
  optimized_local_plan_.clear();
  prev_optimized_local_plan_.clear();
  last_valid_detour_.clear();
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
  if (local_astar_plan_pub_.getNumSubscribers() > 0)
  {
    nav_msgs::Path empty_path;
    empty_path.header.frame_id = map_frame_;
    empty_path.header.stamp = ros::Time::now();
    local_astar_plan_pub_.publish(empty_path);
  }
}

bool OctoLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_) return false;
  std::lock_guard<std::recursive_mutex> lock(planner_mutex_);

  velocity_smoother_.reset();
  last_control_time_ = ros::Time(0);

  if (plan.empty())
  {
    resetPlanState();
    return true;
  }

  global_plan_ = plan;
  prev_optimized_local_plan_.clear();
  // A new plan starts at the robot's current position, so the inherited index
  // from the previous plan points into an unrelated region of the new plan.
  // Reset the seed so the projection search starts from the plan head.
  target_index_ = 0;
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

  LocalPlannerProfiler prof(&rate_monitor_);
  if (!initialized_) {
    prof.fail_reason = PlannerFailReason::NOT_INITIALIZED;
    ROS_WARN_THROTTLE(2.0, "[OctoLocalPlanner] computeVelocityCommands failed: not initialized!");
    return false;
  }
  auto t_pre_lock = LocalPlannerProfiler::Clock::now();
  std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  prof.lock_ms = LocalPlannerProfiler::elapsedMs(t_pre_lock);

  if (global_plan_.empty()) {
    prof.fail_reason = PlannerFailReason::EMPTY_PLAN;
    ROS_WARN_THROTTLE(2.0, "[OctoLocalPlanner] computeVelocityCommands failed: global_plan is empty!");
    return false;
  }

  RobotPose2D robot_pose;
  auto t_pre_tf = LocalPlannerProfiler::Clock::now();
  bool tf_ok = lookupRobotPose2D(robot_pose);
  prof.tf_ms = LocalPlannerProfiler::elapsedMs(t_pre_tf);
  if (!tf_ok)
  {
    prof.fail_reason = PlannerFailReason::TF_LOOKUP_ROBOT;
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

  // Early goal proximity check
  const auto & goal_pos = global_plan_.back().pose.position;
  const double dist_xy = std::hypot(goal_pos.x - robot_pose.x, goal_pos.y - robot_pose.y);
  const double robot_ground_z = robot_pose.z - robot_body_height_;
  const double dist_z = std::min(std::abs(goal_pos.z - robot_ground_z), std::abs(goal_pos.z - robot_pose.z));
  const bool near_goal_pos = (dist_xy < goal_pos_tol_) && (dist_z < goal_z_tol_);

  if (!pose_adjusting_ && (near_goal_pos || 
      (dist_xy < tracking_xy_tol_ && dist_z < goal_z_tol_ && target_index_ >= static_cast<int>(global_plan_.size()) - 3)))
  {
    pose_adjusting_ = true;
  }

  // Final goal adjustment
  if (pose_adjusting_)
  {
    geometry_msgs::PoseStamped final_pose_base;
    if (!transformToBase(global_plan_.back(), final_pose_base)) {
      prof.fail_reason = PlannerFailReason::TF_TRANSFORM_BASE;
      return false;
    }

    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = final_pose_base.pose.position.x * linear_gain_;
    raw_cmd.linear.y = enable_lateral_motion_ 
                       ? final_pose_base.pose.position.y * lateral_gain_
                       : 0.0;

    double final_yaw_error = 0.0;
    if (align_final_yaw_)
    {
      if (!computeFinalYawErrorXY(global_plan_.back(), final_yaw_error)) {
        prof.fail_reason = PlannerFailReason::TF_YAW_ERROR;
        return false;
      }
      raw_cmd.angular.z = final_yaw_error * final_yaw_gain_;
    }

    const double rel_dist_xy = std::hypot(final_pose_base.pose.position.x, final_pose_base.pose.position.y);
    const double rel_dist_z = std::min(std::abs(final_pose_base.pose.position.z), std::abs(final_pose_base.pose.position.z + robot_body_height_));
    const bool pos_ok = (rel_dist_xy < goal_pos_tol_) && (rel_dist_z < goal_z_tol_);
    const bool yaw_ok = !align_final_yaw_ || std::abs(final_yaw_error) < goal_yaw_tol_;

    if (pos_ok && yaw_ok)
    {
      resetPlanState();
      velocity_smoother_.reset();
      cmd_vel = geometry_msgs::Twist();
      return true;
    }

    cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
    return true;
  }

  // 1. Macro local band extraction along global plan up to local_planner_horizon_ (e.g. 2.2m)
  std::vector<geometry_msgs::PoseStamped> local_astar_band = extractLocalBand(robot_pose, local_planner_horizon_);

  // Large heading deviation check: If robot is facing away from path (> 60 deg / 1.05 rad),
  // front perception sensors cannot observe the path yet. Prioritize rotating in place to bring
  // the path into forward sensor view before invoking 3D A* into the blind rear zone!
  if (local_astar_band.size() >= 2)
  {
    size_t look_idx = 1;
    for (size_t i = 1; i < local_astar_band.size(); ++i)
    {
      look_idx = i;
      double dx = local_astar_band[i].pose.position.x - robot_pose.x;
      double dy = local_astar_band[i].pose.position.y - robot_pose.y;
      if (std::hypot(dx, dy) >= 0.40) break;
    }
    geometry_msgs::PoseStamped look_base;
    if (transformToBase(local_astar_band[look_idx], look_base))
    {
      double heading_to_path = std::atan2(look_base.pose.position.y, look_base.pose.position.x);
      if (std::abs(heading_to_path) > 1.05)
      {
        publishLocalAStarPlan(local_astar_band);
        geometry_msgs::Twist raw_cmd;
        raw_cmd.linear.x = 0.0;
        raw_cmd.linear.y = 0.0;
        raw_cmd.angular.z = heading_to_path * heading_gain_;
        cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
        last_cmd_vel_ = cmd_vel;
        ROS_INFO_THROTTLE(1.0, "[OctoLocalPlanner] Facing away from path (yaw_err=%.1f deg). Aligning in place before 3D A*.",
                          heading_to_path * 180.0 / M_PI);
        return true;
      }
    }
  }

  // 2. Local 3D A* obstacle detection & macro detour replanning
  auto t_pre_detour = LocalPlannerProfiler::Clock::now();
  local_astar_band = checkAndReplanAStarDetour(local_astar_band);
  prof.detour_ms = LocalPlannerProfiler::elapsedMs(t_pre_detour);

  // 3. Publish macro A* plan to /move_base/local_astar_plan for debugging & web UI visualization
  publishLocalAStarPlan(local_astar_band);

  if (local_astar_band.size() < 2)
  {
    prof.fail_reason = PlannerFailReason::ASTAR_DETOUR_EMPTY;
    velocity_smoother_.reset();
    cmd_vel = geometry_msgs::Twist();
    last_cmd_vel_ = cmd_vel;
    if (local_plan_pub_.getNumSubscribers() > 0)
    {
      nav_msgs::Path empty_path;
      empty_path.header.frame_id = map_frame_;
      empty_path.header.stamp = ros::Time::now();
      local_plan_pub_.publish(empty_path);
    }
    ROS_WARN_THROTTLE(1.0, "[OctoLocalPlanner] 3D A* detour failed! Halting robot to avoid obstacle collision.");
    return false;
  }

  // 4. Micro local band clipping for TEB optimization (e.g. 1.0m planning right in front)
  std::vector<geometry_msgs::PoseStamped> local_band = clipTrajectoryByDistance(local_astar_band, teb_planning_horizon_);

  // Optimize local band
  if (local_band.size() >= 3 && map_ready_)
  {
    if (map_changed_ && (now - last_local_rebuild_time_).toSec() >= 0.25)
    {
      map_changed_ = false;
      last_local_rebuild_time_ = now;

      auto t_pre_vis = LocalPlannerProfiler::Clock::now();
      bool need_vis = (visualizer_.getTraversablePub().getNumSubscribers() > 0 ||
                       visualizer_.getPreblockedPub().getNumSubscribers() > 0 ||
                       visualizer_.getRiskCostPub().getNumSubscribers() > 0);
      if (need_vis) {
        visualizer_.publishCellSetMarker(planner_.getTraversableCells(), visualizer_.getTraversablePub(), "traversable_cells", 0.20F, 0.95F, 0.55F, 0.55F, planner_, robot_pose.x, robot_pose.y, robot_pose.yaw);
        visualizer_.publishCellSetMarker(planner_.getPreblockedCells(), visualizer_.getPreblockedPub(), "preblocked_cells", 0.15F, 0.35F, 1.0F, 0.95F, planner_, robot_pose.x, robot_pose.y, robot_pose.yaw);
        visualizer_.publishRiskCostCloud(planner_, robot_pose.x, robot_pose.y, robot_pose.yaw);
      }
      prof.vis_ms += LocalPlannerProfiler::elapsedMs(t_pre_vis);
    }

    visualizer_.publishRawLocalBandMarkers(local_band);

    bool teb_success = false;
    if (use_teb_optimizer_ && teb_planner_)
    {
      auto t_pre_field = LocalPlannerProfiler::Clock::now();
      forbidden_field_.updateField(planner_, robot_pose.x, robot_pose.y);
      prof.field_ms = LocalPlannerProfiler::elapsedMs(t_pre_field);

      auto t_pre_teb = LocalPlannerProfiler::Clock::now();
      std::vector<geometry_msgs::PoseStamped> teb_band;
      bool is_near_final_goal = false;
      if (!global_plan_.empty())
      {
        const auto& final_p = global_plan_.back().pose.position;
        const auto& curr_p = local_band.back().pose.position;
        if (std::hypot(final_p.x - curr_p.x, final_p.y - curr_p.y) < 0.30)
        {
          is_near_final_goal = true;
        }
      }
      bool free_goal_vel = !is_near_final_goal;
      if (teb_planner_->planWithForbiddenZone(local_band, &last_cmd_vel_, free_goal_vel,
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
          teb_success = true;
        }
      }
      else
      {
        elastic_band_.optimize(local_band, planner_);
      }
      prof.teb_ms = LocalPlannerProfiler::elapsedMs(t_pre_teb);
    }
    else
    {
      auto t_pre_teb = LocalPlannerProfiler::Clock::now();
      elastic_band_.optimize(local_band, planner_);
      prof.teb_ms = LocalPlannerProfiler::elapsedMs(t_pre_teb);
    }

    optimized_local_plan_ = local_band;
    prev_optimized_local_plan_ = local_band;
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

  // Check goal proximity & final tracking point
  if (near_goal_pos || 
      (dist_xy < tracking_xy_tol_ && dist_z < goal_z_tol_ && target_index_ >= static_cast<int>(global_plan_.size()) - 3)) {
    pose_adjusting_ = true;
    cmd_vel = geometry_msgs::Twist();
    return true;
  }

  // Output control velocity: directly use TEB's time-optimal velocity command
  bool teb_vel_valid = false;
  double teb_vx = 0.0, teb_vy = 0.0, teb_wz = 0.0;
  if (use_teb_optimizer_ && teb_planner_)
  {
    if (teb_planner_->getVelocityCommand(teb_vx, teb_vy, teb_wz, 1))
    {
      teb_vel_valid = true;
    }
  }

  if (!teb_vel_valid)
  {
    prof.fail_reason = PlannerFailReason::TEB_NO_FEASIBLE_VEL;
    ROS_WARN_THROTTLE(1.0, "[OctoLocalPlanner] TEB failed to compute feasible velocity! Stopping robot.");
    velocity_smoother_.reset();
    cmd_vel = geometry_msgs::Twist();
    last_cmd_vel_ = cmd_vel;
    return false;
  }

  geometry_msgs::Twist raw_cmd;
  raw_cmd.linear.x = teb_vx;
  raw_cmd.linear.y = enable_lateral_motion_ ? teb_vy : 0.0;
  raw_cmd.angular.z = teb_wz;

  cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
  last_cmd_vel_ = cmd_vel;

  // Emergency stop check (delegated to tracking module)
  auto t_pre_emg = LocalPlannerProfiler::Clock::now();
  bool emg_triggered = checkEmergencyStop(robot_pose, cmd_vel);
  prof.emg_ms = LocalPlannerProfiler::elapsedMs(t_pre_emg);

  prof.logSummary();

  if (emg_triggered)
  {
    last_cmd_vel_ = cmd_vel;
    return true;
  }

  visualizer_.publishTrackingPointMarker(global_plan_[target_index_]);
  ROS_INFO_THROTTLE(1.0, "[OctoLocalPlanner] Active: Band=%zu | Cmd: vx=%.2f, vy=%.2f, wz=%.2f",
                    optimized_local_plan_.size(), cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  return true;
}

} // namespace octo_planner
