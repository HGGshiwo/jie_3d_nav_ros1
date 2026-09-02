#include "octo_planner/octo_local_visualizer.h"
#include <sensor_msgs/point_cloud2_iterator.h>

namespace octo_planner
{

OctoLocalVisualizer::OctoLocalVisualizer()
{
}

void OctoLocalVisualizer::initialize(ros::NodeHandle & private_nh, const VisualizerParams & params)
{
  params_ = params;

  std::string tracking_marker_topic;
  private_nh.param<std::string>("tracking_point_marker_topic", tracking_marker_topic, "tracking_point_marker");
  marker_pub_ = private_nh.advertise<visualization_msgs::Marker>(tracking_marker_topic, 1, true);
  band_marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("optimized_local_band", 1, true);
  traversable_marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("traversable_cells", 1, true);
  preblocked_marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("preblocked_cells", 1, true);
  risk_cost_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("risk_cost_cloud", 1, true);
}

void OctoLocalVisualizer::publishTrackingPointMarker(const geometry_msgs::PoseStamped & pose)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = params_.map_frame;
  marker.header.stamp    = ros::Time::now();
  marker.ns     = "octo_tracking_point";
  marker.id     = 0;
  marker.type   = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose   = pose.pose;
  marker.scale.x = marker.scale.y = marker.scale.z = params_.tracking_marker_scale;
  marker.color.r = 1.0f; marker.color.g = 0.5f; marker.color.b = 0.0f; marker.color.a = 0.95f;
  marker_pub_.publish(marker);
}

void OctoLocalVisualizer::publishLocalBandMarkers(const std::vector<geometry_msgs::PoseStamped> & band)
{
  if (band.empty()) return;
  visualization_msgs::Marker marker;
  marker.header.frame_id = params_.map_frame;
  marker.header.stamp    = ros::Time::now();
  marker.ns     = "optimized_local_band";
  marker.id     = 1;
  marker.type   = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.12;
  marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.5f; marker.color.a = 0.90f;

  for (const auto & pose : band) {
    marker.points.push_back(pose.pose.position);
  }
  band_marker_pub_.publish(marker);
}

void OctoLocalVisualizer::publishCellSetMarker(
  const std::unordered_set<octo_planner::GridIndex, octo_planner::GridIndexHash> & cells,
  ros::Publisher & publisher,
  const std::string & ns,
  float r_color, float g_color, float b_color, float a_color,
  const OctoPlannerCore & planner) const
{
  auto octree = planner.getOctree();
  if (!octree) return;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = params_.map_frame;
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
    const auto p = planner.gridToWorld(c);
    geometry_msgs::Point q;
    q.x = p.x(); q.y = p.y(); q.z = p.z();
    marker.points.push_back(q);
  }
  publisher.publish(marker);
}

void OctoLocalVisualizer::publishRiskCostCloud(const OctoPlannerCore & planner) const
{
  const auto costmap = planner.getPreblockedCostmap();
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = params_.map_frame;

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
    const auto p = planner.gridToWorld(entry.first);
    *iter_x = p.x(); *iter_y = p.y(); *iter_z = p.z();
    *iter_i = static_cast<float>(entry.second);
    ++iter_x; ++iter_y; ++iter_z; ++iter_i;
  }
  risk_cost_pub_.publish(cloud_msg);
}

void OctoLocalVisualizer::clearMarkers()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = params_.map_frame;
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_pub_.publish(marker);
  band_marker_pub_.publish(marker);
}

} // namespace octo_planner
