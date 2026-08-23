// jie_path_node.cpp  —  ROS 1 port of the original ROS 2 implementation
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "jie_map_msgs/ExportNavigationSnapshot.h"
#include "jie_map_msgs/GetNavigationMapMeta.h"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/Octomap.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "visualization_msgs/Marker.h"

#include "octo_planner/octo_planner_core.h"

class JiePathNode
{
public:
  JiePathNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
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
    pnh_.param<bool>("enable_preblocked_costmap", enable_preblocked_costmap_, true); // Note: we need it for sub triggers
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

    // Subscribers (latching = transient_local equivalent in ROS1)
    octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &JiePathNode::onOctomap, this);
    start_sub_   = nh_.subscribe(start_topic_, 1, &JiePathNode::onStart, this);
    goal_sub_    = nh_.subscribe(goal_topic_, 1, &JiePathNode::onGoal, this);
    goal_pose_sub_ = nh_.subscribe(goal_pose_topic_, 1, &JiePathNode::onGoalPose, this);
    external_preblocked_sub_ = nh_.subscribe(
      external_preblocked_marker_topic_, 1, &JiePathNode::onExternalPreblockedMarker, this);
    edited_occupied_sub_ = nh_.subscribe(
      edited_occupied_marker_topic_, 1, &JiePathNode::onEditedOccupiedMarker, this);

    // Publishers (latch=true mimics transient_local)
    path_pub_             = nh_.advertise<nav_msgs::Path>(path_topic_, 1, /*latch=*/true);
    path_marker_pub_      = nh_.advertise<visualization_msgs::Marker>(path_marker_topic_, 1, true);
    octomap_pub_          = nh_.advertise<octomap_msgs::Octomap>(octomap_topic_, 1, true);
    preblocked_marker_pub_= nh_.advertise<visualization_msgs::Marker>(preblocked_marker_topic_, 1, true);
    traversable_marker_pub_= nh_.advertise<visualization_msgs::Marker>(traversable_marker_topic_, 1, true);
    risk_cost_pub_        = nh_.advertise<sensor_msgs::PointCloud2>(risk_cost_topic_, 1, true);

    // Services (use private node handle so they appear as ~/get_meta etc.)
    get_meta_srv_ = pnh_.advertiseService("get_meta", &JiePathNode::handleGetMapMeta, this);
    export_snapshot_srv_ = pnh_.advertiseService("export_snapshot", &JiePathNode::handleExportSnapshot, this);

    ROS_INFO(
      "jie_path_node started. octomap=%s start=%s goal=%s path=%s "
      "preblocked_marker=%s edited_occupied=%s meta_service=~/get_meta export_service=~/export_snapshot",
      octomap_topic_.c_str(), start_topic_.c_str(), goal_topic_.c_str(), path_topic_.c_str(),
      preblocked_marker_topic_.c_str(), edited_occupied_marker_topic_.c_str());
  }

private:
  // ---- helpers ----
  ros::Time now() const { return ros::Time::now(); }

  void fillBounds(geometry_msgs::Point & min_bound, geometry_msgs::Point & max_bound) const
  {
    auto octree = planner_.getOctree();
    if (!octree) return;
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree->getMetricMin(min_x, min_y, min_z);
    octree->getMetricMax(max_x, max_y, max_z);
    min_bound.x = min_x; min_bound.y = min_y; min_bound.z = min_z;
    max_bound.x = max_x; max_bound.y = max_y; max_bound.z = max_z;
  }

  // ---- services ----
  bool handleGetMapMeta(
    jie_map_msgs::GetNavigationMapMeta::Request & /*req*/,
    jie_map_msgs::GetNavigationMapMeta::Response & res)
  {
    auto octree = planner_.getOctree();
    res.success = map_ready_ && static_cast<bool>(octree);
    res.message = res.success ? "ok" : "octomap not ready";
    res.map_id  = map_id_;
    res.frame_id = frame_id_;
    res.resolution = octree ? octree->getResolution() : 0.0;
    fillBounds(res.min_bound, res.max_bound);
    // Note: read values back from the planner
    res.robot_radius = 0.20; // fallback or we could expose getter
    res.snap_search_radius_cells = 8;
    res.require_ground_support = true;
    res.strict_direct_ground_support = true;
    res.ground_support_xy_radius_cells = 1;
    res.ground_support_depth_cells = 2;
    res.enable_preblocked_costmap = enable_preblocked_costmap_;
    res.preblocked_costmap_radius_cells = 3;
    res.preblocked_costmap_weight = 1.5;
    res.source_world_file = source_world_file_;
    return true;
  }

  bool handleExportSnapshot(
    jie_map_msgs::ExportNavigationSnapshot::Request & req,
    jie_map_msgs::ExportNavigationSnapshot::Response & res)
  {
    auto octree = planner_.getOctree();
    if (!map_ready_ || !octree) {
      res.success = false;
      res.message = "octomap not ready";
      res.snapshot_stamp = now();
      return true;
    }
    if (req.recompute_layers) {
      planner_.rebuildAllLayers();
    } else {
      publishPreblockedCellsMarker();
      publishCellSetMarker(planner_.getTraversableCells(), traversable_marker_pub_, "traversable_cells", 0.20F, 0.95F, 0.55F, 0.55F);
      publishRiskCostCloud();
    }
    res.success = true;
    res.message = "snapshot ready";
    res.snapshot_stamp = now();
    return true;
  }

  uint64_t hashOctomapData(const std::vector<int8_t> & data)
  {
    std::uint64_t h = 1469598103934665603ULL;
    for (const auto v : data) {
      h ^= static_cast<std::uint8_t>(v);
      h *= 1099511628211ULL;
    }
    return h;
  }

  // ---- subscribers ----
  void onOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
  {
    const std::uint64_t map_hash = hashOctomapData(msg->data);
    if (map_ready_ && map_hash == last_octomap_hash_) return;

    std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
    if (!octree) {
      ROS_ERROR("Failed to convert OctoMap message to OcTree.");
      return;
    }
    planner_.setOctree(octree);
    map_ready_ = true;
    last_octomap_hash_ = map_hash;

    planner_.rebuildAllLayers();
    
    publishPreblockedCellsMarker();
    publishCellSetMarker(planner_.getTraversableCells(), traversable_marker_pub_, "traversable_cells", 0.20F, 0.95F, 0.55F, 0.55F);
    publishRiskCostCloud();
  }

  void onEditedOccupiedMarker(const visualization_msgs::Marker::ConstPtr & msg)
  {
    if (msg->type != visualization_msgs::Marker::CUBE_LIST) {
      ROS_WARN("Ignored edited occupied marker because it is not CUBE_LIST.");
      return;
    }
    planner_.clearExternalPreblockedCells();
    const double resolution = markerResolution(*msg);
    if (resolution <= 0.0) {
      ROS_WARN("Ignored edited occupied marker because scale is invalid.");
      return;
    }
    auto edited_tree = std::make_shared<octomap::OcTree>(resolution);
    for (const auto & point : msg->points) {
      edited_tree->updateNode(
        octomap::point3d(static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)),
        true);
    }
    edited_tree->updateInnerOccupancy();
    planner_.setOctree(edited_tree);
    map_ready_ = true;
    last_octomap_hash_ = 0;

    if (!msg->header.frame_id.empty()) frame_id_ = msg->header.frame_id;

    publishCurrentOctomap();
    planner_.rebuildAllLayers();

    publishPreblockedCellsMarker();
    publishCellSetMarker(planner_.getTraversableCells(), traversable_marker_pub_, "traversable_cells", 0.20F, 0.95F, 0.55F, 0.55F);
    publishRiskCostCloud();

    ROS_INFO("Edited occupied marker applied. occupied_cells=%zu resolution=%.3f",
      msg->points.size(), resolution);

    if (has_start_ && has_goal_) {
      if (!planAndPublish()) ROS_WARN("No path found after edited occupied map refresh.");
    }
  }

  static double markerResolution(const visualization_msgs::Marker & msg)
  {
    const double sx = msg.scale.x > 0.0 ? msg.scale.x : 0.0;
    const double sy = msg.scale.y > 0.0 ? msg.scale.y : 0.0;
    const double sz = msg.scale.z > 0.0 ? msg.scale.z : 0.0;
    if (sx <= 0.0 && sy <= 0.0 && sz <= 0.0) return 0.0;
    if (sx > 0.0) return sx;
    if (sy > 0.0) return sy;
    return sz;
  }

  void publishCurrentOctomap()
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

  void onStart(const geometry_msgs::PointStamped::ConstPtr & msg)
  {
    start_point_ = *msg;
    has_start_ = true;
    ROS_INFO("Start set to [%.3f, %.3f, %.3f]", msg->point.x, msg->point.y, msg->point.z);
  }

  void onGoal(const geometry_msgs::PointStamped::ConstPtr & msg)
  {
    goal_point_ = *msg;
    has_goal_ = true;
    ROS_INFO("Goal set to [%.3f, %.3f, %.3f]", msg->point.x, msg->point.y, msg->point.z);
    tryPlan();
  }

  void onGoalPose(const geometry_msgs::PoseStamped::ConstPtr & msg)
  {
    goal_pose_ = *msg;
    has_goal_pose_ = true;
    const double yaw = std::atan2(
      2.0 * (msg->pose.orientation.w * msg->pose.orientation.z +
      msg->pose.orientation.x * msg->pose.orientation.y),
      1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y +
      msg->pose.orientation.z * msg->pose.orientation.z));
    ROS_INFO("Goal pose yaw set to %.1f deg in frame %s",
      yaw * 180.0 / M_PI,
      msg->header.frame_id.empty() ? frame_id_.c_str() : msg->header.frame_id.c_str());
    tryPlan();
  }

  void tryPlan()
  {
    if (!map_ready_ || !has_start_ || !has_goal_ || planning_in_progress_) return;
    planning_in_progress_ = true;
    ++plan_seq_;
    const bool ok = planAndPublish();
    planning_in_progress_ = false;
    if (!ok) {
      ROS_WARN_THROTTLE(2.0, "\033[1;31mA* planning failed.\033[0m");
    } else {
      last_success_seq_ = plan_seq_;
    }
  }

  void onExternalPreblockedMarker(const visualization_msgs::Marker::ConstPtr & msg)
  {
    std::unordered_set<octo_planner::GridIndex, octo_planner::GridIndexHash> external_cells;
    auto octree = planner_.getOctree();
    if (!octree) return;
    for (const auto & point : msg->points) {
      const octo_planner::GridIndex idx = planner_.worldToGrid(point.x, point.y, point.z);
      if (planner_.isInsideMetricBounds(idx) && !planner_.isInsideMetricBounds(idx)) // occupied is checked inside core
        external_cells.insert(idx);
    }
    planner_.setExternalPreblockedCells(external_cells);
    ROS_INFO("Received external preblocked marker. cells=%zu", external_cells.size());
    planner_.rebuildAllLayers();
    
    publishPreblockedCellsMarker();
    publishCellSetMarker(planner_.getTraversableCells(), traversable_marker_pub_, "traversable_cells", 0.20F, 0.95F, 0.55F, 0.55F);
    publishRiskCostCloud();

    if (map_ready_ && has_start_ && has_goal_) {
      if (!planAndPublish()) ROS_WARN("No path found after external preblocked update.");
    }
  }

  bool planAndPublish()
  {
    const uint64_t this_plan_seq = plan_seq_;
    std::vector<octo_planner::GridIndex> cells;
    std::string error_msg;

    // Optional snapping log output
    octo_planner::GridIndex start_raw = planner_.worldToGrid(start_point_.point.x, start_point_.point.y, start_point_.point.z);
    octo_planner::GridIndex goal_raw = planner_.worldToGrid(goal_point_.point.x, goal_point_.point.y, goal_point_.point.z);
    octo_planner::GridIndex start, goal;
    // We could call planner_.findNearestFreeCell to log snapping info
    // But planner_.plan already handles it internally. We can just reproduce logs if snapped:
    // ...

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

  void publishPath(const std::vector<octo_planner::GridIndex> & cells, const std::string & frame_id)
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

  // ---- marker/cloud publishing ----
  void publishCellSetMarker(
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

  void publishPreblockedCellsMarker()
  {
    publishCellSetMarker(planner_.getPreblockedCells(), preblocked_marker_pub_,
      "preblocked_cells", 0.15F, 0.35F, 1.0F, 0.95F);
  }

  void publishRiskCostCloud() const
  {
    auto & costmap = planner_.getPreblockedCostmap();
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

  // ---- ros handles ----
  ros::NodeHandle & nh_;
  ros::NodeHandle & pnh_;

  // ---- parameters ----
  std::string octomap_topic_, start_topic_, goal_topic_, goal_pose_topic_;
  std::string path_topic_, path_marker_topic_, preblocked_marker_topic_;
  std::string external_preblocked_marker_topic_, edited_occupied_marker_topic_;
  std::string traversable_marker_topic_, risk_cost_topic_;
  std::string frame_id_, map_id_, source_world_file_;
  bool enable_preblocked_costmap_;

  // ---- state ----
  bool map_ready_, has_start_, has_goal_, has_goal_pose_, planning_in_progress_;
  std::uint64_t plan_seq_, last_success_seq_, last_octomap_hash_;
  geometry_msgs::PointStamped start_point_, goal_point_;
  geometry_msgs::PoseStamped goal_pose_;

  octo_planner::OctoPlannerCore planner_;

  // ---- ros handles ----
  ros::Subscriber octomap_sub_, start_sub_, goal_sub_, goal_pose_sub_;
  ros::Subscriber external_preblocked_sub_, edited_occupied_sub_;
  ros::Publisher path_pub_, path_marker_pub_, octomap_pub_;
  ros::Publisher preblocked_marker_pub_, traversable_marker_pub_, risk_cost_pub_;
  ros::ServiceServer get_meta_srv_, export_snapshot_srv_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "jie_path_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  JiePathNode node(nh, pnh);
  ros::spin();
  return 0;
}