#include "octo_planner/jie_path_node.h"

JiePathNode::JiePathNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
: nh_(nh), pnh_(pnh),
  map_ready_(false),
  has_start_(false),
  has_goal_(false),
  has_goal_pose_(false),
  planning_in_progress_(false),
  plan_seq_(0),
  last_success_seq_(0),
  last_octomap_hash_(0)
{
  // Parameters
  pnh_.param<std::string>("octomap_topic", octomap_topic_, "/octomap");
  pnh_.param<std::string>("start_topic", start_topic_, "/start_point");
  pnh_.param<std::string>("goal_topic", goal_topic_, "/goal_point");
  pnh_.param<std::string>("goal_pose_topic", goal_pose_topic_, "/goal_pose");
  pnh_.param<std::string>("path_topic", path_topic_, "/planned_path");
  pnh_.param<std::string>("path_marker_topic", path_marker_topic_, "/planned_path_marker");
  pnh_.param<std::string>("preblocked_marker_topic", preblocked_marker_topic_, "/preblocked_cells_markers");
  pnh_.param<std::string>("external_preblocked_marker_topic", external_preblocked_marker_topic_, "/edited_preblocked_cells_markers");
  pnh_.param<std::string>("edited_occupied_marker_topic", edited_occupied_marker_topic_, "/edited_occupied_markers");
  pnh_.param<std::string>("traversable_marker_topic", traversable_marker_topic_, "/traversable_cells_markers");
  pnh_.param<std::string>("risk_cost_topic", risk_cost_topic_, "/risk_cost_cells");
  pnh_.param<std::string>("frame_id", frame_id_, "map");
  
  double robot_radius;
  int max_iterations, snap_search_radius_cells;
  bool require_ground_support, strict_direct_ground_support;
  int ground_support_xy_radius_cells, ground_support_depth_cells;
  int max_step_height_cells, robot_clearance_height_cells;
  bool enable_preblocked_costmap;
  int preblocked_costmap_radius_cells;
  double preblocked_costmap_weight;
  bool lowest_traversable_only;

  pnh_.param<double>("robot_radius", robot_radius, 0.20);
  pnh_.param<int>("max_iterations", max_iterations, 250000);
  pnh_.param<int>("snap_search_radius_cells", snap_search_radius_cells, 8);
  pnh_.param<bool>("require_ground_support", require_ground_support, true);
  pnh_.param<bool>("strict_direct_ground_support", strict_direct_ground_support, true);
  pnh_.param<int>("ground_support_xy_radius_cells", ground_support_xy_radius_cells, 1);
  pnh_.param<int>("ground_support_depth_cells", ground_support_depth_cells, 2);
  pnh_.param<int>("max_step_height_cells", max_step_height_cells, 1);
  pnh_.param<int>("robot_clearance_height_cells", robot_clearance_height_cells, 0);
  pnh_.param<bool>("enable_preblocked_costmap", enable_preblocked_costmap_, true);
  enable_preblocked_costmap = enable_preblocked_costmap_;
  pnh_.param<int>("preblocked_costmap_radius_cells", preblocked_costmap_radius_cells, 3);
  pnh_.param<double>("preblocked_costmap_weight", preblocked_costmap_weight, 1.5);
  pnh_.param<bool>("lowest_traversable_only", lowest_traversable_only, false);
  pnh_.param<std::string>("map_id", map_id_, "navigation_map");
  pnh_.param<std::string>("source_world_file", source_world_file_, "");

  // Set parameters on the core planner
  planner_.setRobotRadius(robot_radius);
  planner_.setMaxIterations(max_iterations);
  planner_.setSnapSearchRadiusCells(snap_search_radius_cells);
  planner_.setRequireGroundSupport(require_ground_support);
  planner_.setStrictDirectGroundSupport(strict_direct_ground_support);
  planner_.setGroundSupportXYRadiusCells(ground_support_xy_radius_cells);
  planner_.setGroundSupportDepthCells(ground_support_depth_cells);
  planner_.setMaxStepHeightCells(max_step_height_cells);
  planner_.setRobotClearanceHeightCells(robot_clearance_height_cells);
  planner_.setEnablePreblockedCostmap(enable_preblocked_costmap);
  planner_.setPreblockedCostmapRadiusCells(preblocked_costmap_radius_cells);
  planner_.setPreblockedCostmapWeight(preblocked_costmap_weight);
  planner_.setLowestTraversableOnly(lowest_traversable_only);

  // Subscribers
  octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &JiePathNode::onOctomap, this);
  start_sub_   = nh_.subscribe(start_topic_, 1, &JiePathNode::onStart, this);
  goal_sub_    = nh_.subscribe(goal_topic_, 1, &JiePathNode::onGoal, this);
  goal_pose_sub_ = nh_.subscribe(goal_pose_topic_, 1, &JiePathNode::onGoalPose, this);
  external_preblocked_sub_ = nh_.subscribe(
    external_preblocked_marker_topic_, 1, &JiePathNode::onExternalPreblockedMarker, this);
  edited_occupied_sub_ = nh_.subscribe(
    edited_occupied_marker_topic_, 1, &JiePathNode::onEditedOccupiedMarker, this);

  // Publishers
  path_pub_             = nh_.advertise<nav_msgs::Path>(path_topic_, 1, /*latch=*/true);
  path_marker_pub_      = nh_.advertise<visualization_msgs::Marker>(path_marker_topic_, 1, true);
  octomap_pub_          = nh_.advertise<octomap_msgs::Octomap>(octomap_topic_, 1, true);
  preblocked_marker_pub_= nh_.advertise<visualization_msgs::Marker>(preblocked_marker_topic_, 1, true);
  traversable_marker_pub_= nh_.advertise<visualization_msgs::Marker>(traversable_marker_topic_, 1, true);
  risk_cost_pub_        = nh_.advertise<sensor_msgs::PointCloud2>(risk_cost_topic_, 1, true);

  // Services
  get_meta_srv_ = pnh_.advertiseService("get_meta", &JiePathNode::handleGetMapMeta, this);
  export_snapshot_srv_ = pnh_.advertiseService("export_snapshot", &JiePathNode::handleExportSnapshot, this);
  query_cell_debug_srv_ = pnh_.advertiseService("query_cell_debug_info", &JiePathNode::handleQueryCellDebugInfo, this);

  ROS_INFO(
    "jie_path_node started. octomap=%s start=%s goal=%s path=%s "
    "preblocked_marker=%s edited_occupied=%s meta_service=~/get_meta export_service=~/export_snapshot query_debug_service=~/query_cell_debug_info",
    octomap_topic_.c_str(), start_topic_.c_str(), goal_topic_.c_str(), path_topic_.c_str(),
    preblocked_marker_topic_.c_str(), edited_occupied_marker_topic_.c_str());
}

void JiePathNode::fillBounds(geometry_msgs::Point & min_bound, geometry_msgs::Point & max_bound) const
{
  auto octree = planner_.getOctree();
  if (!octree) return;
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree->getMetricMin(min_x, min_y, min_z);
  octree->getMetricMax(max_x, max_y, max_z);
  min_bound.x = min_x; min_bound.y = min_y; min_bound.z = min_z;
  max_bound.x = max_x; max_bound.y = max_y; max_bound.z = max_z;
}

uint64_t JiePathNode::hashOctomapData(const std::vector<int8_t> & data)
{
  std::uint64_t h = 1469598103934665603ULL;
  for (const auto v : data) {
    h ^= static_cast<std::uint8_t>(v);
    h *= 1099511628211ULL;
  }
  return h;
}

double JiePathNode::markerResolution(const visualization_msgs::Marker & msg)
{
  const double sx = msg.scale.x > 0.0 ? msg.scale.x : 0.0;
  const double sy = msg.scale.y > 0.0 ? msg.scale.y : 0.0;
  const double sz = msg.scale.z > 0.0 ? msg.scale.z : 0.0;
  if (sx <= 0.0 && sy <= 0.0 && sz <= 0.0) return 0.0;
  if (sx > 0.0) return sx;
  if (sy > 0.0) return sy;
  return sz;
}

void JiePathNode::publishCurrentOctomap()
{
  auto octree = planner_.getOctree();
  if (!octree) return;
  octomap_msgs::Octomap msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  if (!octomap_msgs::fullMapToMsg(*octree, msg)) {
    ROS_WARN("Failed to publish edited OctoMap message.");
    return;
  }
  octomap_pub_.publish(msg);
  last_octomap_hash_ = hashOctomapData(msg.data);
}

void JiePathNode::tryPlan()
{
  if (!map_ready_ || !has_start_ || !has_goal_) return;

  if (planning_in_progress_) {
    ROS_INFO("New planning request received. Cancelling active search...");
    planner_.setCancelFlag(true);
    while (planning_in_progress_) {
      ros::Duration(0.005).sleep();
    }
  }

  planner_.setCancelFlag(false);
  planning_in_progress_ = true;
  ++plan_seq_;
  const bool ok = planAndPublish();
  planning_in_progress_ = false;
  if (!ok) {
    ROS_WARN_THROTTLE(2.0, "\033[1;31mA* planning failed or was cancelled.\033[0m");
  } else {
    last_success_seq_ = plan_seq_;
  }
}

bool JiePathNode::planAndPublish()
{
  const uint64_t this_plan_seq = plan_seq_;
  std::vector<octo_planner::GridIndex> cells;
  std::string error_msg;

  bool ok = planner_.plan(start_point_.point, goal_point_.point, cells, error_msg);
  if (!ok) {
    ROS_ERROR_THROTTLE(2.0, "Planning failed: %s", error_msg.c_str());
    return false;
  }

  if (this_plan_seq == plan_seq_) {
    publishPath(cells, frame_id_);
    ROS_INFO("\033[1;32mA* path found. waypoints=%zu\033[0m", cells.size());
    return true;
  }
  return false;
}

void JiePathNode::publishPath(const std::vector<octo_planner::GridIndex> & cells, const std::string & frame_id)
{
  nav_msgs::Path path_msg;
  path_msg.header.stamp = now();
  path_msg.header.frame_id = frame_id;
  path_msg.poses.reserve(cells.size());

  visualization_msgs::Marker m;
  m.header = path_msg.header;
  m.ns = "jie_path";
  m.id = 0;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.32;
  m.color.r = 0.1F; m.color.g = 0.95F; m.color.b = 0.95F; m.color.a = 1.0F;
  m.pose.orientation.w = 1.0;

  for (std::size_t i = 0; i < cells.size(); ++i) {
    const auto p = planner_.gridToWorld(cells[i]);
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.position.z = p.z();
    pose.pose.orientation.w = 1.0;
    if (has_goal_pose_ && i + 1 == cells.size())
      pose.pose.orientation = goal_pose_.pose.orientation;
    path_msg.poses.push_back(pose);

    geometry_msgs::Point q;
    q.x = p.x(); q.y = p.y(); q.z = p.z();
    m.points.push_back(q);
  }

  path_pub_.publish(path_msg);
  path_marker_pub_.publish(m);
}

void JiePathNode::publishCellSetMarker(
  const std::unordered_set<octo_planner::GridIndex, octo_planner::GridIndexHash> & cells,
  ros::Publisher & publisher,
  const std::string & ns,
  float r_color, float g_color, float b_color, float a_color) const
{
  auto octree = planner_.getOctree();
  if (!octree) return;
  visualization_msgs::Marker marker;
  marker.header.stamp = now();
  marker.header.frame_id = frame_id_;
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  const double resolution = octree->getResolution();
  marker.scale.x = resolution;
  marker.scale.y = resolution;
  marker.scale.z = resolution;
  marker.color.r = r_color;
  marker.color.g = g_color;
  marker.color.b = b_color;
  marker.color.a = a_color;
  marker.points.reserve(cells.size());
  for (const auto & c : cells) {
    const auto p = planner_.gridToWorld(c);
    geometry_msgs::Point q;
    q.x = p.x(); q.y = p.y(); q.z = p.z();
    marker.points.push_back(q);
  }
  publisher.publish(marker);
}

void JiePathNode::publishPreblockedCellsMarker()
{
  publishCellSetMarker(planner_.getPreblockedCells(), preblocked_marker_pub_,
    "preblocked_cells", 0.15F, 0.35F, 1.0F, 0.95F);
}

void JiePathNode::publishRiskCostCloud() const
{
  const auto costmap = planner_.getPreblockedCostmap();
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = now();
  cloud_msg.header.frame_id = frame_id_;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::PointField::FLOAT32,
    "y", 1, sensor_msgs::PointField::FLOAT32,
    "z", 1, sensor_msgs::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(costmap.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(cloud_msg, "intensity");

  for (const auto & entry : costmap) {
    const auto p = planner_.gridToWorld(entry.first);
    *iter_x = p.x(); *iter_y = p.y(); *iter_z = p.z();
    *iter_i = static_cast<float>(entry.second);
    ++iter_x; ++iter_y; ++iter_z; ++iter_i;
  }
  risk_cost_pub_.publish(cloud_msg);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "jie_path_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  JiePathNode node(nh, pnh);
  ros::AsyncSpinner spinner(4); // 4 threads to handle callbacks concurrently
  spinner.start();
  ros::waitForShutdown();
  return 0;
}