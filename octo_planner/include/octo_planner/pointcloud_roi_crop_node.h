#ifndef POINTCLOUD_ROI_CROP_NODE_H
#define POINTCLOUD_ROI_CROP_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <string>

namespace octo_planner
{

class PointCloudROICropNode
{
public:
  PointCloudROICropNode(ros::NodeHandle & nh, ros::NodeHandle & pnh);
  ~PointCloudROICropNode() = default;

private:
  void onPointCloud(const sensor_msgs::PointCloud2::ConstPtr & msg);
  void onOdom(const nav_msgs::Odometry::ConstPtr & msg);
  bool getLatestRobotPose(double & rx, double & ry, double & rz, std::string & robot_frame);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher cloud_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex pose_mutex_;
  bool has_odom_pose_;
  double robot_x_, robot_y_, robot_z_;
  std::string odom_frame_;

  // Parameters
  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::string odom_topic_;
  std::string target_frame_;
  double crop_radius_xy_;
  double crop_height_above_;
  double crop_height_below_;
};

} // namespace octo_planner

#endif // POINTCLOUD_ROI_CROP_NODE_H
