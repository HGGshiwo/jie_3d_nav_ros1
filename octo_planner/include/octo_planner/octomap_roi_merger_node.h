#ifndef OCTOMAP_ROI_MERGER_NODE_H
#define OCTOMAP_ROI_MERGER_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include "jie_map_msgs/QueryCellDebugInfo.h"
#include <mutex>
#include <string>

namespace octo_planner
{

class OctomapROIMergerNode
{
public:
  OctomapROIMergerNode(ros::NodeHandle & nh, ros::NodeHandle & pnh);
  ~OctomapROIMergerNode() = default;

private:
  void onGlobalOctomap(const octomap_msgs::Octomap::ConstPtr & msg);
  void onLocalOctomap(const octomap_msgs::Octomap::ConstPtr & msg);
  void onOdom(const nav_msgs::Odometry::ConstPtr & msg);
  bool getLatestRobotPose(double & rx, double & ry, double & rz);
  void processAndPublishFusedMap();
  bool handleQueryCellDebugInfo(
    jie_map_msgs::QueryCellDebugInfo::Request & req,
    jie_map_msgs::QueryCellDebugInfo::Response & res);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber global_octomap_sub_;
  ros::Subscriber local_octomap_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher fused_octomap_pub_;
  ros::ServiceServer query_cell_debug_srv_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex pose_mutex_;
  bool has_odom_pose_;
  double robot_x_, robot_y_, robot_z_;
  std::string odom_frame_;

  std::mutex map_mutex_;
  octomap_msgs::Octomap::ConstPtr latest_global_msg_;
  octomap_msgs::Octomap::ConstPtr latest_local_msg_;
  std::shared_ptr<octomap::OcTree> latest_fused_tree_;
  std::shared_ptr<octomap::OcTree> latest_global_tree_;
  std::shared_ptr<octomap::OcTree> latest_local_tree_;

  // Parameters
  std::string global_octomap_topic_;
  std::string local_octomap_topic_;
  std::string fused_octomap_topic_;
  std::string odom_topic_;
  std::string target_frame_;
  double crop_radius_xy_;
  double crop_height_above_;
  double crop_height_below_;
};

} // namespace octo_planner

#endif // OCTOMAP_ROI_MERGER_NODE_H
