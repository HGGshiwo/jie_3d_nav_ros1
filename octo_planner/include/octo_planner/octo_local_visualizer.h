#ifndef OCTO_LOCAL_VISUALIZER_H
#define OCTO_LOCAL_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <unordered_set>

#include "octo_planner/octo_planner_core.h"

namespace octo_planner
{

struct VisualizerParams
{
  std::string map_frame = "map";
  double tracking_marker_scale = 0.28;
  bool enable_debug_view = true;
  int debug_view_size_px = 640;
  double debug_ppm = 80.0;
  double debug_view_frequency = 10.0;
  double roi_radius_xy = 3.5; // Expanded forward ROI radius
  double roi_forward_shift = 1.5; // Forward shift along robot heading
};

class OctoLocalVisualizer
{
public:
  OctoLocalVisualizer();
  ~OctoLocalVisualizer() = default;

  void initialize(ros::NodeHandle & private_nh, const VisualizerParams & params);

  void publishTrackingPointMarker(const geometry_msgs::PoseStamped & pose);
  void publishLocalBandMarkers(const std::vector<geometry_msgs::PoseStamped> & band);
  void publishRawLocalBandMarkers(const std::vector<geometry_msgs::PoseStamped> & band);
  void publishCellSetMarker(
    const std::unordered_set<octo_planner::GridIndex, octo_planner::GridIndexHash> & cells,
    ros::Publisher & publisher,
    const std::string & ns,
    float r_color, float g_color, float b_color, float a_color,
    const OctoPlannerCore & planner,
    double robot_x = 0.0, double robot_y = 0.0, double robot_yaw = 0.0) const;
  void publishRiskCostCloud(const OctoPlannerCore & planner, double robot_x = 0.0, double robot_y = 0.0, double robot_yaw = 0.0) const;
  void clearMarkers();

  ros::Publisher & getTraversablePub() { return traversable_marker_pub_; }
  ros::Publisher & getPreblockedPub() { return preblocked_marker_pub_; }
  ros::Publisher & getRiskCostPub() { return risk_cost_pub_; }

private:
  VisualizerParams params_;

  ros::Publisher marker_pub_;
  ros::Publisher band_marker_pub_;
  ros::Publisher raw_band_marker_pub_;
  ros::Publisher traversable_marker_pub_;
  ros::Publisher preblocked_marker_pub_;
  ros::Publisher risk_cost_pub_;
  ros::Publisher debug_image_pub_;
};

} // namespace octo_planner

#endif // OCTO_LOCAL_VISUALIZER_H
