#ifndef JIE_PATH_NODE_H
#define JIE_PATH_NODE_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "visualization_msgs/Marker.h"

#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/Octomap.h"

#include "octo_planner/octo_planner_core.h"
#include "jie_map_msgs/ExportNavigationSnapshot.h"
#include "jie_map_msgs/GetNavigationMapMeta.h"
#include "jie_map_msgs/QueryCellDebugInfo.h"

class JiePathNode
{
public:
  JiePathNode(ros::NodeHandle & nh, ros::NodeHandle & pnh);
  ~JiePathNode() = default;

private:
  // ---- helpers ----
  ros::Time now() const { return ros::Time::now(); }
  void fillBounds(geometry_msgs::Point & min_bound, geometry_msgs::Point & max_bound) const;
  uint64_t hashOctomapData(const std::vector<int8_t> & data);
  static double markerResolution(const visualization_msgs::Marker & msg);
  void publishCurrentOctomap();
  void tryPlan();
  bool planAndPublish();
  void publishPath(const std::vector<octo_planner::GridIndex> & cells, const std::string & frame_id);
  void publishCellSetMarker(
    const std::unordered_set<octo_planner::GridIndex, octo_planner::GridIndexHash> & cells,
    ros::Publisher & publisher,
    const std::string & ns,
    float r_color, float g_color, float b_color, float a_color) const;
  void publishPreblockedCellsMarker();
  void publishRiskCostCloud() const;

  // ---- subscribers ----
  void onOctomap(const octomap_msgs::Octomap::ConstPtr & msg);
  void onEditedOccupiedMarker(const visualization_msgs::Marker::ConstPtr & msg);
  void onStart(const geometry_msgs::PointStamped::ConstPtr & msg);
  void onGoal(const geometry_msgs::PointStamped::ConstPtr & msg);
  void onGoalPose(const geometry_msgs::PoseStamped::ConstPtr & msg);
  void onExternalPreblockedMarker(const visualization_msgs::Marker::ConstPtr & msg);

  // ---- services ----
  bool handleGetMapMeta(
    jie_map_msgs::GetNavigationMapMeta::Request & req,
    jie_map_msgs::GetNavigationMapMeta::Response & res);
  bool handleExportSnapshot(
    jie_map_msgs::ExportNavigationSnapshot::Request & req,
    jie_map_msgs::ExportNavigationSnapshot::Response & res);
  bool handleQueryCellDebugInfo(
    jie_map_msgs::QueryCellDebugInfo::Request & req,
    jie_map_msgs::QueryCellDebugInfo::Response & res);

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
  ros::NodeHandle & nh_;
  ros::NodeHandle & pnh_;
  ros::Subscriber octomap_sub_, start_sub_, goal_sub_, goal_pose_sub_;
  ros::Subscriber external_preblocked_sub_, edited_occupied_sub_;
  ros::Publisher path_pub_, path_marker_pub_, octomap_pub_;
  ros::Publisher preblocked_marker_pub_, traversable_marker_pub_, risk_cost_pub_;
  ros::ServiceServer get_meta_srv_, export_snapshot_srv_, query_cell_debug_srv_;
};

#endif // JIE_PATH_NODE_H
