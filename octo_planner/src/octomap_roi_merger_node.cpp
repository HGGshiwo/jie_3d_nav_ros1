#include "octo_planner/octomap_roi_merger_node.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

namespace octo_planner
{

namespace
{
octomap::OcTreeNode* ensureNodeExpandedToMaxDepth(octomap::OcTree* tree, const octomap::OcTreeKey& key) {
  if (!tree) return nullptr;
  octomap::OcTreeNode* cur = tree->getRoot();
  if (!cur) return nullptr;

  const unsigned int max_depth = tree->getTreeDepth();

  for (unsigned int d = max_depth; d > 0; --d) {
    if (!tree->nodeHasChildren(cur)) {
      if (tree->isNodeOccupied(cur)) {
        return cur;
      }
      tree->expandNode(cur);
    }

    unsigned int pos = 0;
    if (key[0] & (1 << (d - 1))) pos |= 1;
    if (key[1] & (1 << (d - 1))) pos |= 2;
    if (key[2] & (1 << (d - 1))) pos |= 4;

    if (!tree->nodeChildExists(cur, pos)) {
      return nullptr;
    }

    cur = tree->getNodeChild(cur, pos);
  }

  return cur;
}
} // namespace

OctomapROIMergerNode::OctomapROIMergerNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
: nh_(nh),
  pnh_(pnh),
  tf_listener_(tf_buffer_),
  has_odom_pose_(false),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_z_(0.0)
{
  pnh_.param<std::string>("global_octomap_topic", global_octomap_topic_, "/octomap_global");
  pnh_.param<std::string>("local_octomap_topic", local_octomap_topic_, "/octomap_local");
  pnh_.param<std::string>("fused_octomap_topic", fused_octomap_topic_, "/octomap_fused");
  pnh_.param<std::string>("odom_topic", odom_topic_, "/loc_base");
  pnh_.param<std::string>("target_frame", target_frame_, "map");
  pnh_.param<double>("crop_radius_xy", crop_radius_xy_, 3.0);
  pnh_.param<double>("crop_height_above", crop_height_above_, 2.0);
  pnh_.param<double>("crop_height_below", crop_height_below_, 1.0);

  odom_sub_ = nh_.subscribe(odom_topic_, 1, &OctomapROIMergerNode::onOdom, this);
  global_octomap_sub_ = nh_.subscribe(global_octomap_topic_, 1, &OctomapROIMergerNode::onGlobalOctomap, this);
  local_octomap_sub_ = nh_.subscribe(local_octomap_topic_, 1, &OctomapROIMergerNode::onLocalOctomap, this);

  fused_octomap_pub_ = pnh_.advertise<octomap_msgs::Octomap>(fused_octomap_topic_, 1, /*latch=*/true);
  query_cell_debug_srv_ = pnh_.advertiseService("query_cell_debug_info", &OctomapROIMergerNode::handleQueryCellDebugInfo, this);

  ROS_INFO("OctomapROIMergerNode initialized: global=%s, local=%s, fused=%s, odom=%s",
           global_octomap_topic_.c_str(), local_octomap_topic_.c_str(),
           fused_octomap_topic_.c_str(), odom_topic_.c_str());
}

void OctomapROIMergerNode::onOdom(const nav_msgs::Odometry::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_z_ = msg->pose.pose.position.z;
  odom_frame_ = msg->header.frame_id.empty() ? target_frame_ : msg->header.frame_id;
  has_odom_pose_ = true;
}

bool OctomapROIMergerNode::getLatestRobotPose(double & rx, double & ry, double & rz)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (has_odom_pose_) {
    rx = robot_x_;
    ry = robot_y_;
    rz = robot_z_;
    return true;
  }
  return false;
}

void OctomapROIMergerNode::onGlobalOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_global_msg_ = msg;
    if (msg) {
      octomap::AbstractOcTree* abs_tree = octomap_msgs::msgToMap(*msg);
      if (abs_tree) {
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abs_tree);
        if (octree) {
          latest_global_tree_.reset(octree);
        } else {
          delete abs_tree;
        }
      }
    }
  }
  processAndPublishFusedMap();
}

void OctomapROIMergerNode::onLocalOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_local_msg_ = msg;
    if (msg) {
      octomap::AbstractOcTree* abs_tree = octomap_msgs::msgToMap(*msg);
      if (abs_tree) {
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abs_tree);
        if (octree) {
          latest_local_tree_.reset(octree);
        } else {
          delete abs_tree;
        }
      }
    }
  }
  processAndPublishFusedMap();
}

void OctomapROIMergerNode::processAndPublishFusedMap()
{
  if (fused_octomap_pub_.getNumSubscribers() == 0) {
    return;
  }

  octomap_msgs::Octomap::ConstPtr global_msg, local_msg;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    global_msg = latest_global_msg_;
    local_msg = latest_local_msg_;
  }

  if (!global_msg && !local_msg) {
    return;
  }

  double rx = 0.0, ry = 0.0, rz = 0.0;
  bool got_pose = getLatestRobotPose(rx, ry, rz);
  if (!got_pose) {
    // If no odom, try TF lookup from map -> base_link
    try {
      geometry_msgs::TransformStamped tf_stamped = tf_buffer_.lookupTransform(
          target_frame_, "base_link", ros::Time(0), ros::Duration(0.01));
      rx = tf_stamped.transform.translation.x;
      ry = tf_stamped.transform.translation.y;
      rz = tf_stamped.transform.translation.z;
      got_pose = true;
    } catch (...) {
      // Fallback pose at origin
      rx = 0.0; ry = 0.0; rz = 0.0;
    }
  }

  // Create base fused OcTree starting from local OcTree (or empty tree if local unavailable)
  std::unique_ptr<octomap::OcTree> fused_tree;

  if (local_msg) {
    octomap::AbstractOcTree* abs_tree = octomap_msgs::msgToMap(*local_msg);
    if (abs_tree) {
      octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abs_tree);
      if (octree) {
        fused_tree.reset(octree);
      } else {
        delete abs_tree;
      }
    }
  }

  double resolution = 0.1;
  if (!fused_tree && global_msg) {
    resolution = global_msg->resolution > 0 ? global_msg->resolution : 0.1;
    fused_tree = std::make_unique<octomap::OcTree>(resolution);
  } else if (!fused_tree) {
    fused_tree = std::make_unique<octomap::OcTree>(resolution);
  }

  // If global map is available, crop ROI and merge into fused OcTree
  if (global_msg) {
    octomap::AbstractOcTree* abs_global = octomap_msgs::msgToMap(*global_msg);
    if (abs_global) {
      octomap::OcTree* global_octree = dynamic_cast<octomap::OcTree*>(abs_global);
      if (global_octree) {
        // Compute ROI bounding box: -1.0m behind to +(crop_radius_xy_ + 1.5)m in front
        const double min_x = rx - 1.0;
        const double max_x = rx + (crop_radius_xy_ + 1.5);
        const double min_y = ry - crop_radius_xy_;
        const double max_y = ry + crop_radius_xy_;
        const double min_z = rz - crop_height_below_;
        const double max_z = rz + crop_height_above_;

        octomap::point3d min_pt(min_x, min_y, min_z);
        octomap::point3d max_pt(max_x, max_y, max_z);

        const double target_res = fused_tree->getResolution();
        const double half_target_res = target_res * 0.5;

        for (octomap::OcTree::leaf_bbx_iterator it = global_octree->begin_leafs_bbx(min_pt, max_pt),
                                                end = global_octree->end_leafs_bbx();
             it != end; ++it)
        {
          if (global_octree->isNodeOccupied(*it)) {
            const float global_log_odds = it->getLogOdds();
            const double node_size = it.getSize();
            const octomap::point3d center = it.getCoordinate();

            if (node_size <= target_res + 1e-6) {
              octomap::OcTreeKey key;
              octomap::OcTreeNode* node = nullptr;
              if (fused_tree->coordToKeyChecked(center, key)) {
                node = ensureNodeExpandedToMaxDepth(fused_tree.get(), key);
              }
              if (!node) {
                node = fused_tree->updateNode(center, 0.0f, /*lazy_eval=*/true);
              }
              if (node && node->getLogOdds() < global_log_odds) {
                node->setLogOdds(global_log_odds);
              }
            } else {
              // Integer Key-Bound Mapping algorithm:
              // Compute exact spatial bounding box [cell_min, cell_max] for global leaf node
              const double half_size = node_size * 0.5;
              octomap::point3d cell_min(center.x() - half_size + half_target_res,
                                        center.y() - half_size + half_target_res,
                                        center.z() - half_size + half_target_res);
              octomap::point3d cell_max(center.x() + half_size - half_target_res,
                                        center.y() + half_size - half_target_res,
                                        center.z() + half_size - half_target_res);

              octomap::OcTreeKey min_key, max_key;
              if (fused_tree->coordToKeyChecked(cell_min, min_key) &&
                  fused_tree->coordToKeyChecked(cell_max, max_key))
              {
                for (auto kx = min_key[0]; kx <= max_key[0]; ++kx) {
                  for (auto ky = min_key[1]; ky <= max_key[1]; ++ky) {
                    for (auto kz = min_key[2]; kz <= max_key[2]; ++kz) {
                      octomap::OcTreeKey key(kx, ky, kz);
                      octomap::OcTreeNode* node = ensureNodeExpandedToMaxDepth(fused_tree.get(), key);
                      if (!node) {
                        node = fused_tree->updateNode(key, 0.0f, /*lazy_eval=*/true);
                      }
                      if (node && node->getLogOdds() < global_log_odds) {
                        node->setLogOdds(global_log_odds);
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
      delete abs_global;
    }
  }

  fused_tree->updateInnerOccupancy();
  fused_tree->prune();

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_fused_tree_ = std::move(fused_tree);
  }

  // Publish fused octomap
  octomap_msgs::Octomap fused_msg;
  fused_msg.header.stamp = ros::Time::now();
  fused_msg.header.frame_id = target_frame_;

  if (latest_fused_tree_ && octomap_msgs::binaryMapToMsg(*latest_fused_tree_, fused_msg)) {
    fused_octomap_pub_.publish(fused_msg);
  }
}

bool OctomapROIMergerNode::handleQueryCellDebugInfo(
  jie_map_msgs::QueryCellDebugInfo::Request & req,
  jie_map_msgs::QueryCellDebugInfo::Response & res)
{
  std::shared_ptr<octomap::OcTree> tree;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (req.layer_name == "occupied" && latest_global_tree_) {
      tree = latest_global_tree_;
    } else if (req.layer_name == "local" && latest_local_tree_) {
      tree = latest_local_tree_;
    } else {
      tree = latest_fused_tree_ ? latest_fused_tree_ : latest_global_tree_;
    }
  }

  if (!tree) {
    res.success = false;
    res.message = "octomap memory not ready";
    return true;
  }

  const octomap::point3d p(static_cast<float>(req.x), static_cast<float>(req.y), static_cast<float>(req.z));
  const octomap::OcTreeNode * node = tree->search(p);

  const double res_val = tree->getResolution();
  res.grid_x = static_cast<int32_t>(std::floor(req.x / res_val));
  res.grid_y = static_cast<int32_t>(std::floor(req.y / res_val));
  res.grid_z = static_cast<int32_t>(std::floor(req.z / res_val));

  res.is_unknown = (node == nullptr);
  res.is_occupied = (node && tree->isNodeOccupied(node));

  if (node) {
    double size = res_val;
    for (octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(p, p), end = tree->end_leafs_bbx(); it != end; ++it) {
      size = it.getSize();
      break;
    }
    const bool occ = tree->isNodeOccupied(node);
    const float log_odds = node->getLogOdds();

    std::ostringstream ss;
    ss << "占据状态=" << (occ ? "占据(Occupied)" : "空闲(Free)")
       << ", 体素尺寸=" << std::fixed << std::setprecision(2) << size << "m"
       << ", LogOdds=" << std::setprecision(3) << log_odds
       << ", 探测点=[" << std::setprecision(3) << p.x() << "," << p.y() << "," << p.z() << "]";
    res.node_source_info = ss.str();
  } else {
    std::ostringstream ss;
    ss << "未知/未映射区域 (Unknown/Unmapped Space), 坐标点 ["
       << std::fixed << std::setprecision(3) << p.x() << "," << p.y() << "," << p.z() << "] 未在八叉树内存中分配节点";
    res.node_source_info = ss.str();
  }

  res.success = true;
  res.message = "query ok";
  return true;
}

} // namespace octo_planner

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "octomap_roi_merger_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  octo_planner::OctomapROIMergerNode node(nh, pnh);
  ros::spin();
  return 0;
}
