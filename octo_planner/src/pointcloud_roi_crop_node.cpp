#include "octo_planner/pointcloud_roi_crop_node.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

namespace octo_planner
{

PointCloudROICropNode::PointCloudROICropNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
: nh_(nh),
  pnh_(pnh),
  tf_listener_(tf_buffer_),
  has_odom_pose_(false),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_z_(0.0)
{
  pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, "/cloud_in");
  pnh_.param<std::string>("output_cloud_topic", output_cloud_topic_, "/cloud_cropped");
  pnh_.param<std::string>("odom_topic", odom_topic_, "/loc_base");
  pnh_.param<std::string>("target_frame", target_frame_, "map");
  pnh_.param<double>("crop_radius_xy", crop_radius_xy_, 3.0);
  pnh_.param<double>("crop_height_above", crop_height_above_, 2.0);
  pnh_.param<double>("crop_height_below", crop_height_below_, 1.0);

  odom_sub_ = nh_.subscribe(odom_topic_, 1, &PointCloudROICropNode::onOdom, this);
  cloud_sub_ = nh_.subscribe(input_cloud_topic_, 1, &PointCloudROICropNode::onPointCloud, this);
  cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);

  ROS_INFO("PointCloudROICropNode initialized: in=%s, out=%s, odom=%s, radius=%.1fm",
           input_cloud_topic_.c_str(), output_cloud_topic_.c_str(), odom_topic_.c_str(), crop_radius_xy_);
}

void PointCloudROICropNode::onOdom(const nav_msgs::Odometry::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_z_ = msg->pose.pose.position.z;
  odom_frame_ = msg->header.frame_id.empty() ? target_frame_ : msg->header.frame_id;
  has_odom_pose_ = true;
}

bool PointCloudROICropNode::getLatestRobotPose(double & rx, double & ry, double & rz, std::string & robot_frame)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (has_odom_pose_) {
    rx = robot_x_;
    ry = robot_y_;
    rz = robot_z_;
    robot_frame = odom_frame_;
    return true;
  }
  return false;
}

void PointCloudROICropNode::onPointCloud(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  if (cloud_pub_.getNumSubscribers() == 0) {
    return;
  }

  double rx = 0.0, ry = 0.0, rz = 0.0;
  std::string odom_frame;
  if (!getLatestRobotPose(rx, ry, rz, odom_frame)) {
    // Fallback: try TF lookup if odom topic hasn't arrived
    try {
      geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(msg->header.frame_id, "base_link", ros::Time(0), ros::Duration(0.01));
      rx = tf.transform.translation.x;
      ry = tf.transform.translation.y;
      rz = tf.transform.translation.z;
    } catch (...) {
      // If no pose available yet, pass through or skip
      cloud_pub_.publish(*msg);
      return;
    }
  }

  // Create cropped point cloud output
  sensor_msgs::PointCloud2 output_msg;
  output_msg.header = msg->header;
  output_msg.height = 1;
  output_msg.is_dense = false;
  output_msg.is_bigendian = msg->is_bigendian;

  sensor_msgs::PointCloud2Modifier modifier(output_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2ConstIterator<float> iter_in_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_in_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_in_z(*msg, "z");

  // Pre-allocate temporary vectors for cropped points
  std::vector<float> valid_x, valid_y, valid_z;
  valid_x.reserve(msg->width * msg->height / 4);
  valid_y.reserve(msg->width * msg->height / 4);
  valid_z.reserve(msg->width * msg->height / 4);

  const double min_x = rx - crop_radius_xy_;
  const double max_x = rx + crop_radius_xy_;
  const double min_y = ry - crop_radius_xy_;
  const double max_y = ry + crop_radius_xy_;
  const double min_z = rz - crop_height_below_;
  const double max_z = rz + crop_height_above_;

  for (; iter_in_x != iter_in_x.end(); ++iter_in_x, ++iter_in_y, ++iter_in_z) {
    const float px = *iter_in_x;
    const float py = *iter_in_y;
    const float pz = *iter_in_z;

    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;

    if (px >= min_x && px <= max_x &&
        py >= min_y && py <= max_y &&
        pz >= min_z && pz <= max_z)
    {
      valid_x.push_back(px);
      valid_y.push_back(py);
      valid_z.push_back(pz);
    }
  }

  modifier.resize(valid_x.size());
  sensor_msgs::PointCloud2Iterator<float> iter_out_x(output_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_out_y(output_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_out_z(output_msg, "z");

  for (size_t i = 0; i < valid_x.size(); ++i, ++iter_out_x, ++iter_out_y, ++iter_out_z) {
    *iter_out_x = valid_x[i];
    *iter_out_y = valid_y[i];
    *iter_out_z = valid_z[i];
  }

  cloud_pub_.publish(output_msg);
}

} // namespace octo_planner

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "pointcloud_roi_crop_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  octo_planner::PointCloudROICropNode node(nh, pnh);
  ros::spin();
  return 0;
}
