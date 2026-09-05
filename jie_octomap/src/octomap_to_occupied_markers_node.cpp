#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class OctomapToOccupiedMarkersNode
{
public:
  OctomapToOccupiedMarkersNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  {
    std::string octomap_topic, marker_topic;
    pnh.param<std::string>("octomap_topic", octomap_topic, "/octomap");
    pnh.param<std::string>("marker_topic", marker_topic, "/octomap_occupied_markers");
    pnh.param<std::string>("frame_id", frame_id_, "map");

    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(marker_topic, 1, /*latch=*/true);

    octomap_sub_ = nh.subscribe(octomap_topic, 1,
                                &OctomapToOccupiedMarkersNode::onOctomap, this);

    ROS_INFO("octomap_to_occupied_markers started. octomap=%s marker_array=%s",
             octomap_topic.c_str(), marker_topic.c_str());
  }

private:
  void onOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> tree_ptr(octomap_msgs::msgToMap(*msg));
    if (!tree_ptr) {
      ROS_ERROR("Failed to decode octomap message.");
      return;
    }
    auto * oc_tree = dynamic_cast<octomap::OcTree *>(tree_ptr.get());
    if (!oc_tree) {
      ROS_ERROR("Decoded map is not octomap::OcTree.");
      return;
    }
    oc_tree->prune();

    const std::string frame = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;

    // Group leaf node coordinates by actual leaf node size
    std::map<double, std::vector<geometry_msgs::Point>> size_map;

    for (auto it = oc_tree->begin_leafs(); it != oc_tree->end_leafs(); ++it) {
      if (!oc_tree->isNodeOccupied(*it)) continue;

      geometry_msgs::Point p;
      p.x = it.getX();
      p.y = it.getY();
      p.z = it.getZ();

      size_map[it.getSize()].push_back(p);
    }

    // Build MarkerArray: each leaf size group has its own Marker with scale matching actual node size
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    size_t total_leaves = 0;

    for (const auto & entry : size_map) {
      const double node_size = entry.first;
      const auto & pts = entry.second;
      total_leaves += pts.size();

      visualization_msgs::Marker m;
      m.header.stamp = msg->header.stamp;
      m.header.frame_id = frame;
      m.ns = "occupied_voxels";
      m.id = marker_id++;
      m.type = visualization_msgs::Marker::CUBE_LIST;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = node_size;
      m.scale.y = node_size;
      m.scale.z = node_size;
      m.color.r = 0.95f;
      m.color.g = 0.45f;
      m.color.b = 0.15f;
      m.color.a = 0.95f;
      m.points = pts;

      marker_array.markers.push_back(m);
    }

    marker_pub_.publish(marker_array);

    ROS_INFO_THROTTLE(3.0, "Published occupied markers: %zu OctoMap leaf nodes across %zu size groups (MarkerArray)",
                      total_leaves, size_map.size());
  }

  ros::Subscriber octomap_sub_;
  ros::Publisher marker_pub_;
  std::string frame_id_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "octomap_to_occupied_markers");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  OctomapToOccupiedMarkersNode node(nh, pnh);
  ros::spin();
  return 0;
}