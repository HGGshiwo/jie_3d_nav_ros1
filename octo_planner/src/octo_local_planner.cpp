#include "octo_planner/octo_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <geometry_msgs/Point.h>

// Register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(octo_planner::OctoLocalPlanner, nav_core::BaseLocalPlanner)

namespace octo_planner
{

OctoLocalPlanner::OctoLocalPlanner()
: tf_buffer_(nullptr),
  costmap_ros_(nullptr),
  initialized_(false),
  map_ready_(false),
  target_index_(0),
  pose_adjusting_(false),
  goal_reached_(true),
  debug_view_disabled_(false)
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

  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;

  ros::NodeHandle private_nh("~/" + name);

  // Read frames
  private_nh.param<std::string>("map_frame",                    map_frame_,                    "map");
  private_nh.param<std::string>("robot_center_offset_frame",    robot_center_offset_frame_,    "odin1_base_link");
  private_nh.param<double>     ("robot_center_offset_x",        robot_center_offset_x_,        -0.18);
  private_nh.param<double>     ("robot_center_offset_y",        robot_center_offset_y_,         0.0);
  private_nh.param<double>     ("robot_center_offset_z",        robot_center_offset_z_,         0.0);

  // Read controller parameters
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
  private_nh.param<int>("eb_iterations", eb_iterations_, 30);
  private_nh.param<double>("eb_w_smooth", eb_w_smooth_, 0.3);
  private_nh.param<double>("eb_w_obstacle", eb_w_obstacle_, 0.5);
  private_nh.param<double>("eb_w_ground", eb_w_ground_, 0.4);
  private_nh.param<double>("eb_safe_distance", eb_safe_distance_, 0.30);
  private_nh.param<double>("eb_learning_rate", eb_learning_rate_, 0.1);

  // Subscribe to the local Octomap
  std::string octomap_topic;
  private_nh.param<std::string>("octomap_topic", octomap_topic, "/octomap_local");
  
  ros::NodeHandle nh;
  octomap_sub_ = nh.subscribe(octomap_topic, 1, &OctoLocalPlanner::onOctomap, this);

  // Advertise markers
  std::string tracking_marker_topic;
  private_nh.param<std::string>("tracking_point_marker_topic",  tracking_marker_topic,        "tracking_point_marker");
  marker_pub_ = private_nh.advertise<visualization_msgs::Marker>(tracking_marker_topic, 1, /*latch=*/true);
  
  band_marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("optimized_local_band", 1, /*latch=*/true);

  // Initialize status publisher
  status_pub_ = nh.advertise<std_msgs::String>("/move_base/status_text", 1, true);

  if (enable_debug_view_)
  {
    const double debug_period = 1.0 / std::max(1.0, debug_view_frequency_);
    debug_view_timer_ = nh_.createTimer(ros::Duration(debug_period), &OctoLocalPlanner::renderTrackingDebugView, this);
  }

  initialized_ = true;
  ROS_INFO("OctoLocalPlanner initialized successfully under name: %s, sub to: %s", name.c_str(), octomap_topic.c_str());
}

void OctoLocalPlanner::onOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
{
  std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
  if (!octree)
  {
    ROS_ERROR("OctoLocalPlanner: Failed to convert OctoMap message to OcTree.");
    return;
  }
  planner_.setOctree(octree);
  map_ready_ = true;
}

bool OctoLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("OctoLocalPlanner is not initialized! Call initialize first.");
    return false;
  }

  if (plan.empty())
  {
    std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
    global_plan_.clear();
    optimized_local_plan_.clear();
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = true;
    clearMarkers();
    return true;
  }

  std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  global_plan_ = plan;
  target_index_ = findInitialTargetIndex3D();
  pose_adjusting_ = false;
  goal_reached_ = false;

  publishTrackingPointMarker();
  ROS_INFO("OctoLocalPlanner: Plan set with %zu points. Initial target index: %d", global_plan_.size(), target_index_);
  return true;
}

bool OctoLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("OctoLocalPlanner is not initialized!");
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(planner_mutex_);

  if (global_plan_.empty())
  {
    ROS_WARN_THROTTLE(2.0, "OctoLocalPlanner: Global plan is empty.");
    publishStatus("Local planner warning: Global plan is empty.");
    return false;
  }

  // Get current robot pose
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose))
  {
    publishStatus("Local planner failed: TF lookup robot pose failed.");
    return false;
  }

  // Check if final goal reached
  if (pose_adjusting_)
  {
    geometry_msgs::PoseStamped final_pose_base;
    if (!transformToBase(global_plan_.back(), final_pose_base))
    {
      publishStatus("Local planner failed: TF transform final pose failed.");
      return false;
    }

    // final yaw tracking
    cmd_vel.linear.x = clamp(final_pose_base.pose.position.x * linear_gain_, -max_linear_speed_, max_linear_speed_);
    cmd_vel.linear.y = enable_lateral_motion_ 
                       ? clamp(final_pose_base.pose.position.y * lateral_gain_, -max_lateral_speed_, max_lateral_speed_)
                       : 0.0;
    cmd_vel.linear.x = applyDeadband(cmd_vel.linear.x, linear_deadband_);
    cmd_vel.linear.y = applyDeadband(cmd_vel.linear.y, lateral_deadband_);

    double final_yaw_error = 0.0;
    if (align_final_yaw_)
    {
      if (!computeFinalYawErrorXY(global_plan_.back(), final_yaw_error)) return false;
      cmd_vel.angular.z = clamp(final_yaw_error * final_yaw_gain_, -max_angular_speed_, max_angular_speed_);
      cmd_vel.angular.z = applyDeadband(cmd_vel.angular.z, angular_deadband_);
    }

    const bool pos_ok = std::hypot(final_pose_base.pose.position.x, final_pose_base.pose.position.y) < goal_pos_tol_;
    const bool yaw_ok = !align_final_yaw_ || std::abs(final_yaw_error) < goal_yaw_tol_;

    if (pos_ok && yaw_ok)
    {
      goal_reached_ = true;
      global_plan_.clear();
      optimized_local_plan_.clear();
      cmd_vel = geometry_msgs::Twist();
      clearMarkers();
      ROS_INFO("OctoLocalPlanner: Goal reached.");
    }
    return true;
  }

  // Extract local plan segment to optimize
  int nearest_idx = findInitialTargetIndex3D();
  
  // Create a local band starting from current robot pose
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

  // Extract up to 1.8 meters or 15 points
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

  // Run 3D Elastic Band Optimization
  if (local_band.size() >= 3 && map_ready_)
  {
    optimizeElasticBand(local_band);
    optimized_local_plan_ = local_band;
    publishLocalBandMarkers(optimized_local_plan_);
  }
  else
  {
    optimized_local_plan_ = local_band;
  }

  // Select lookahead tracking target on optimized_local_plan_
  TrackingTarget target;
  int tracking_idx = 1;
  double lookahead_dist = lookahead_distance_;
  for (size_t i = 1; i < optimized_local_plan_.size(); ++i)
  {
    double dx = optimized_local_plan_[i].pose.position.x - robot_pose.x;
    double dy = optimized_local_plan_[i].pose.position.y - robot_pose.y;
    double d = std::hypot(dx, dy);
    tracking_idx = i;
    if (d >= lookahead_dist)
    {
      break;
    }
  }

  target_index_ = nearest_idx + tracking_idx - 1;
  if (target_index_ >= static_cast<int>(global_plan_.size()))
  {
    target_index_ = global_plan_.size() - 1;
  }

  // Convert lookahead point to base frame
  geometry_msgs::PoseStamped target_pose_base;
  if (!transformToBase(optimized_local_plan_[tracking_idx], target_pose_base))
  {
    return false;
  }
  target.base_x = target_pose_base.pose.position.x;
  target.base_y = target_pose_base.pose.position.y;

  // Check if final waypoint of global plan is reached
  if (target_index_ == static_cast<int>(global_plan_.size()) - 1 && 
      std::hypot(target.base_x, target.base_y) < tracking_xy_tol_)
  {
    pose_adjusting_ = true;
    ROS_INFO("OctoLocalPlanner: Final tracking point reached. Switching to final yaw adjustment.");
    cmd_vel = geometry_msgs::Twist();
    return true;
  }

  // Proportional Control Command
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

  last_cmd_vel_ = cmd_vel;
  publishTrackingPointMarker();
  
  publishStatus("Local planner: Tracking path.");
  return true;
}

void OctoLocalPlanner::optimizeElasticBand(std::vector<geometry_msgs::PoseStamped>& band)
{
  if (band.size() < 3) return;

  ros::WallTime start_time = ros::WallTime::now();

  // 1. Calculate bounding box of the local band
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();
  double max_z = -std::numeric_limits<double>::max();
  for (const auto& pose : band)
  {
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double z = pose.pose.position.z;
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    min_z = std::min(min_z, z);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
    max_z = std::max(max_z, z);
  }

  // Expand bounding box by safety distance plus a buffer margin
  double margin = eb_safe_distance_ + 0.05;
  octomap::point3d min_pt(min_x - margin, min_y - margin, min_z - margin);
  octomap::point3d max_pt(max_x + margin, max_y + margin, max_z + margin);

  // 2. Cache all occupied voxels inside this bounding box
  std::vector<octomap::point3d> local_obstacles;
  auto octree = planner_.getOctree();
  if (octree)
  {
    for (auto it = octree->begin_leafs_bbx(min_pt, max_pt), end = octree->end_leafs_bbx(); it != end; ++it)
    {
      if (octree->isNodeOccupied(*it))
      {
        local_obstacles.push_back(it.getCoordinate());
      }
    }
  }

  // 3. Perform Elastic Band Optimization
  for (int iter = 0; iter < eb_iterations_; ++iter)
  {
    for (size_t i = 1; i < band.size() - 1; ++i)
    {
      octomap::point3d P_i(band[i].pose.position.x, band[i].pose.position.y, band[i].pose.position.z);
      octomap::point3d P_prev(band[i-1].pose.position.x, band[i-1].pose.position.y, band[i-1].pose.position.z);
      octomap::point3d P_next(band[i+1].pose.position.x, band[i+1].pose.position.y, band[i+1].pose.position.z);

      // Smoothness / Spring contraction force
      octomap::point3d F_smooth = P_prev + P_next - P_i * 2.0;

      // Obstacle repulsive force using cached local_obstacles
      double distance = 0.0;
      octomap::point3d gradient(0, 0, 0);
      octomap::point3d F_obs(0, 0, 0);
      if (getDistanceAndGradient(P_i, local_obstacles, distance, gradient))
      {
        if (distance < eb_safe_distance_)
        {
          F_obs = gradient * (eb_safe_distance_ - distance);
        }
      }

      // Ground support vertical force (snapping to nearest traversable voxel)
      octomap::point3d F_ground(0, 0, 0);
      GridIndex cell_idx = planner_.worldToGrid(P_i.x(), P_i.y(), P_i.z());
      GridIndex snapped;
      if (planner_.findNearestFreeCell(cell_idx, robot_radius_, snap_search_radius_cells_,
                                       require_ground_support_, strict_direct_ground_support_,
                                       ground_support_xy_radius_cells_, ground_support_depth_cells_, snapped))
      {
        octomap::point3d ground_pos = planner_.gridToWorld(snapped);
        F_ground = ground_pos - P_i;
      }

      // Combined Total Force
      octomap::point3d F_total = F_smooth * eb_w_smooth_ + F_obs * eb_w_obstacle_ + F_ground * eb_w_ground_;

      // Apply update
      P_i = P_i + F_total * eb_learning_rate_;

      band[i].pose.position.x = P_i.x();
      band[i].pose.position.y = P_i.y();
      band[i].pose.position.z = P_i.z();
    }
  }

  double dt = (ros::WallTime::now() - start_time).toSec() * 1000.0;
  ROS_INFO_THROTTLE(2.0, "OctoLocalPlanner: Optimized local band (size=%zu, obs_cached=%zu) in %.3f ms", 
                    band.size(), local_obstacles.size(), dt);
}

bool OctoLocalPlanner::getDistanceAndGradient(const octomap::point3d& p, const std::vector<octomap::point3d>& obstacles, double& distance, octomap::point3d& gradient)
{
  if (obstacles.empty()) return false;

  double min_dist_sq = std::numeric_limits<double>::max();
  octomap::point3d nearest_obs;
  bool found = false;

  // Query cached obstacles in O(N) simple arithmetic vector checks
  for (const auto& obs : obstacles)
  {
    double dist_sq = (p - obs).norm_sq();
    if (dist_sq < min_dist_sq)
    {
      min_dist_sq = dist_sq;
      nearest_obs = obs;
      found = true;
    }
  }

  if (found)
  {
    distance = std::sqrt(min_dist_sq);
    if (distance > 1e-4)
    {
      gradient = (p - nearest_obs) * (1.0 / distance);
    }
    else
    {
      gradient = octomap::point3d(0, 0, 1.0);
    }
    return true;
  }
  return false;
}

bool OctoLocalPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("OctoLocalPlanner is not initialized!");
    return false;
  }
  return goal_reached_;
}

bool OctoLocalPlanner::isFinalTrackingPointReached(const TrackingTarget & target) const
{
  if (global_plan_.empty() || target_index_ != static_cast<int>(global_plan_.size()) - 1)
    return false;
  return std::hypot(target.base_x, target.base_y) < tracking_xy_tol_;
}

int OctoLocalPlanner::findInitialTargetIndex3D()
{
  if (global_plan_.empty()) return 0;
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) {
    ROS_WARN("OctoLocalPlanner: Failed to get robot pose. Start tracking from path index 0."); return 0;
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

bool OctoLocalPlanner::selectTrackingTarget(TrackingTarget & target)
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
  } catch (const tf2::TransformException & ex) {
    ROS_WARN_THROTTLE(2.0, "OctoLocalPlanner: Lookup robot pose from %s to %s failed: %s",
      map_frame_.c_str(), base_frame.c_str(), ex.what());
    publishStatus("Local planner failed: TF lookup robot pose failed.");
    return false;
  }
}

double OctoLocalPlanner::xyDistanceToPlanPoint(const RobotPose2D & rp, int idx) const
{
  const auto & p = global_plan_[static_cast<std::size_t>(idx)].pose.position;
  return std::hypot(p.x - rp.x, p.y - rp.y);
}

bool OctoLocalPlanner::computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error)
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
    ROS_WARN_THROTTLE(2.0, "OctoLocalPlanner: Transform final pose yaw %s -> %s failed: %s",
      final_pose.header.frame_id.c_str(), map_frame_.c_str(), ex.what());
    return false;
  }
  yaw_error = normalizeAngle(tf2::getYaw(final_pose.pose.orientation) - robot_pose.yaw);
  return true;
}

bool OctoLocalPlanner::transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out)
{
  if (!costmap_ros_) return false;
  std::string base_frame = costmap_ros_->getBaseFrameID();

  geometry_msgs::PoseStamped stamped = pose_in;
  if (stamped.header.frame_id.empty()) stamped.header.frame_id = map_frame_;
  stamped.header.stamp = ros::Time(0);
  try {
    tf_buffer_->transform(stamped, pose_out, base_frame, ros::Duration(0.01));
    applyRobotCenterOffsetToRelativePose(base_frame, pose_out);
    return true;
  } catch (const tf2::TransformException & ex) {
    ROS_WARN_THROTTLE(2.0, "OctoLocalPlanner: Transform %s -> %s failed: %s",
      stamped.header.frame_id.c_str(), base_frame.c_str(), ex.what());
    publishStatus("Local planner failed: TF transform failed.");
    return false;
  }
}

bool OctoLocalPlanner::shouldApplyRobotCenterOffset(const std::string & frame) const
{ return frame == robot_center_offset_frame_; }

void OctoLocalPlanner::applyRobotCenterOffset(const std::string & frame, RobotPose2D & rp) const
{
  if (!shouldApplyRobotCenterOffset(frame)) return;
  const double cy = std::cos(rp.yaw), sy = std::sin(rp.yaw);
  rp.x += cy * robot_center_offset_x_ - sy * robot_center_offset_y_;
  rp.y += sy * robot_center_offset_x_ + cy * robot_center_offset_y_;
  rp.z += robot_center_offset_z_;
}

void OctoLocalPlanner::applyRobotCenterOffsetToRelativePose(const std::string & frame, geometry_msgs::PoseStamped & pose) const
{
  if (!shouldApplyRobotCenterOffset(frame)) return;
  pose.pose.position.x -= robot_center_offset_x_;
  pose.pose.position.y -= robot_center_offset_y_;
  pose.pose.position.z -= robot_center_offset_z_;
}

void OctoLocalPlanner::publishTrackingPointMarker()
{
  if (global_plan_.empty() || target_index_ >= static_cast<int>(global_plan_.size())) return;
  visualization_msgs::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp    = ros::Time::now();
  marker.ns     = "octo_tracking_point";
  marker.id     = 0;
  marker.type   = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose   = global_plan_[target_index_].pose;
  marker.scale.x = marker.scale.y = marker.scale.z = tracking_marker_scale_;
  marker.color.r = 1.0f; marker.color.g = 0.5f; marker.color.b = 0.0f; marker.color.a = 0.95f; // Orange
  marker_pub_.publish(marker);
}

void OctoLocalPlanner::publishLocalBandMarkers(const std::vector<geometry_msgs::PoseStamped>& band)
{
  if (band.empty()) return;
  visualization_msgs::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp    = ros::Time::now();
  marker.ns     = "optimized_local_band";
  marker.id     = 1;
  marker.type   = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  
  marker.scale.x = marker.scale.y = marker.scale.z = 0.12; // Small spheres
  marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.5f; marker.color.a = 0.90f; // Mint green
  
  for (const auto& pose : band)
  {
    marker.points.push_back(pose.pose.position);
  }
  band_marker_pub_.publish(marker);
}

void OctoLocalPlanner::clearMarkers()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp    = ros::Time::now();
  marker.action = visualization_msgs::Marker::DELETE;
  
  marker.ns = "octo_tracking_point";
  marker.id = 0;
  marker_pub_.publish(marker);

  marker.ns = "optimized_local_band";
  marker.id = 1;
  band_marker_pub_.publish(marker);
}

void OctoLocalPlanner::renderTrackingDebugView(const ros::TimerEvent &)
{
  try { renderTrackingDebugViewImpl(); }
  catch (const cv::Exception & ex) {
    ROS_WARN_THROTTLE(2.0, "OctoLocalPlanner OpenCV debug view exception: %s", ex.what());
  }
}

void OctoLocalPlanner::renderTrackingDebugViewImpl()
{
  if (!enable_debug_view_ || debug_view_disabled_ || global_plan_.empty()) return;
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return;

  const int sz = std::max(240, debug_view_size_px_);
  const double ppm = std::max(10.0, debug_ppm_);
  const cv::Point center(sz / 2, sz / 2);
  cv::Mat image(sz, sz, CV_8UC3, cv::Scalar(24, 20, 18));

  // Axes
  cv::line(image, {center.x, 0}, {center.x, sz}, cv::Scalar(60, 50, 48), 1);
  cv::line(image, {0, center.y}, {sz, center.y}, cv::Scalar(60, 50, 48), 1);
  cv::arrowedLine(image, center, {center.x, center.y - 58}, cv::Scalar(230, 230, 230), 2, cv::LINE_AA, 0, 0.25);
  cv::circle(image, center, 8, cv::Scalar(230, 230, 230), -1, cv::LINE_AA);
  cv::putText(image, "robot +X", {center.x + 10, center.y - 62},
    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);

  // Plot global plan (grey)
  std::vector<cv::Point> proj_global;
  for (const auto & pose : global_plan_)
    proj_global.push_back(projectPlanPoint(robot_pose, pose, center, ppm));

  for (size_t i = 1; i < proj_global.size(); ++i)
    cv::line(image, proj_global[i - 1], proj_global[i], cv::Scalar(90, 90, 90), 1, cv::LINE_AA);

  // Plot optimized local plan (green)
  std::vector<cv::Point> proj_opt;
  for (const auto & pose : optimized_local_plan_)
    proj_opt.push_back(projectPlanPoint(robot_pose, pose, center, ppm));

  for (size_t i = 1; i < proj_opt.size(); ++i)
    cv::line(image, proj_opt[i - 1], proj_opt[i], cv::Scalar(80, 240, 120), 2, cv::LINE_AA);
  for (const auto & pt : proj_opt)
    cv::circle(image, pt, 4, cv::Scalar(80, 240, 120), -1, cv::LINE_AA);

  // Target point (orange circle)
  if (target_index_ >= 0 && target_index_ < static_cast<int>(proj_global.size())) {
    cv::circle(image, proj_global[target_index_], 12, cv::Scalar(0, 140, 255), 2, cv::LINE_AA);
    cv::circle(image, proj_global[target_index_],  4, cv::Scalar(0, 140, 255), -1, cv::LINE_AA);
  }

  // Texts
  cv::putText(image, "Octo 3D Local Planner", {16, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(80, 190, 255), 2, cv::LINE_AA);
  
  char pose_text[160];
  std::snprintf(pose_text, sizeof(pose_text), "robot map: x=%.2f y=%.2f z=%.2f yaw=%.1f deg",
    robot_pose.x, robot_pose.y, robot_pose.z, robot_pose.yaw * 180.0 / M_PI);
  cv::putText(image, pose_text, {16, 56}, cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
  
  char cmd_text[160];
  std::snprintf(cmd_text, sizeof(cmd_text), "cmd vel: x=%.3f y=%.3f wz=%.3f",
    last_cmd_vel_.linear.x, last_cmd_vel_.linear.y, last_cmd_vel_.angular.z);
  cv::putText(image, cmd_text, {16, 82}, cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(120, 230, 255), 1, cv::LINE_AA);

  try {
    cv::imshow("octo_3d_local_planner_debug", image);
    cv::waitKey(1);
  } catch (const cv::Exception & ex) {
    debug_view_disabled_ = true;
    ROS_WARN("Disable OctoLocalPlanner OpenCV debug view: %s", ex.what());
  }
}

cv::Point OctoLocalPlanner::projectPlanPoint(
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

bool OctoLocalPlanner::drawFinalGoalYaw(
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


void OctoLocalPlanner::publishStatus(const std::string& status)
{
  if (status != last_status_)
  {
    last_status_ = status;
    std_msgs::String msg;
    msg.data = status;
    status_pub_.publish(msg);
  }
}

} // namespace octo_planner
