#include "octo_planner/jie_path_node.h"

void JiePathNode::onOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
{
  static bool printed = false;
  if (!printed) {
    ROS_INFO("Active map source in use: Octomap Topic [%s]", octomap_topic_.c_str());
    printed = true;
  }

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

void JiePathNode::onOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  static bool printed = false;
  if (!printed) {
    ROS_INFO("Active map source in use: Map Topic [%s]", map_topic_.c_str());
    printed = true;
  }

  const double grid_resolution = msg->info.resolution;
  if (grid_resolution <= 0.0 || octomap_resolution_ <= 0.0) {
    ROS_ERROR("Grid and OctoMap resolutions must be positive.");
    return;
  }

  struct XYKey {
    int x;
    int y;
    bool operator==(const XYKey & o) const { return x == o.x && y == o.y; }
  };
  struct XYKeyHash {
    std::size_t operator()(const XYKey & k) const {
      return std::hash<int>{}(k.x) ^ (std::hash<int>{}(k.y) << 1);
    }
  };

  std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(octomap_resolution_);
  const double wall_height = std::max(octomap_resolution_, wall_height_m_);
  const int height_cells = std::max(1, static_cast<int>(std::ceil(wall_height / octomap_resolution_)));

  const auto & origin = msg->info.origin.position;
  const std::size_t width = msg->info.width;
  const std::size_t height = msg->info.height;

  std::unordered_set<XYKey, XYKeyHash> known_cells;
  std::unordered_set<XYKey, XYKeyHash> occupied_cells;

  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      const std::size_t index = y * width + x;
      if (index >= msg->data.size()) continue;
      const int8_t value = msg->data[index];
      if (value < 0) continue;

      const double world_x = origin.x + (static_cast<double>(x) + 0.5) * grid_resolution;
      const double world_y = origin.y + (static_cast<double>(y) + 0.5) * grid_resolution;
      const int grid_x = static_cast<int>(std::floor(world_x / octomap_resolution_));
      const int grid_y = static_cast<int>(std::floor(world_y / octomap_resolution_));
      const XYKey key{grid_x, grid_y};
      known_cells.insert(key);
      if (value >= occupied_threshold_) occupied_cells.insert(key);
    }
  }

  for (const auto & key : known_cells) {
    const double wx = (static_cast<double>(key.x) + 0.5) * octomap_resolution_;
    const double wy = (static_cast<double>(key.y) + 0.5) * octomap_resolution_;
    octree->updateNode(wx, wy, floor_z_m_ + 0.5 * octomap_resolution_, true);
  }
  for (const auto & key : occupied_cells) {
    const double wx = (static_cast<double>(key.x) + 0.5) * octomap_resolution_;
    const double wy = (static_cast<double>(key.y) + 0.5) * octomap_resolution_;
    for (int z = 1; z <= height_cells; ++z) {
      const double wz = floor_z_m_ + (static_cast<double>(z) + 0.5) * octomap_resolution_;
      octree->updateNode(wx, wy, wz, true);
    }
  }
  octree->updateInnerOccupancy();

  planner_.setOctree(octree);
  map_ready_ = true;
  last_octomap_hash_ = 0;

  if (!msg->header.frame_id.empty()) frame_id_ = msg->header.frame_id;

  planner_.rebuildAllLayers();
  
  publishPreblockedCellsMarker();
  publishCellSetMarker(planner_.getTraversableCells(), traversable_marker_pub_, "traversable_cells", 0.20F, 0.95F, 0.55F, 0.55F);
  publishRiskCostCloud();

  if (has_start_ && has_goal_) {
    tryPlan();
  }
}

void JiePathNode::onEditedOccupiedMarker(const visualization_msgs::Marker::ConstPtr & msg)
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

void JiePathNode::onStart(const geometry_msgs::PointStamped::ConstPtr & msg)
{
  start_point_ = *msg;
  has_start_ = true;
  ROS_INFO("Start set to [%.3f, %.3f, %.3f]", msg->point.x, msg->point.y, msg->point.z);
}

void JiePathNode::onGoal(const geometry_msgs::PointStamped::ConstPtr & msg)
{
  goal_point_ = *msg;
  has_goal_ = true;
  ROS_INFO("Goal set to [%.3f, %.3f, %.3f]", msg->point.x, msg->point.y, msg->point.z);
  tryPlan();
}

void JiePathNode::onGoalPose(const geometry_msgs::PoseStamped::ConstPtr & msg)
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

void JiePathNode::onExternalPreblockedMarker(const visualization_msgs::Marker::ConstPtr & msg)
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
