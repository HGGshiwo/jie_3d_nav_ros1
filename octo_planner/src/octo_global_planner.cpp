#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#include "octo_planner/octo_planner_core.h"

namespace octo_planner
{

class OctoGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
  OctoGlobalPlanner()
  : initialized_(false),
    map_ready_(false)
  {
  }

  virtual ~OctoGlobalPlanner() = default;

  virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override
  {
    if (initialized_)
    {
      ROS_WARN("OctoGlobalPlanner has already been initialized, doing nothing.");
      return;
    }

    ros::NodeHandle private_nh("~/" + name);
    
    private_nh.param<std::string>("local_map_path", local_map_path_, "");
    private_nh.param<std::string>("map_topic", map_topic_, "");
    private_nh.param<std::string>("octomap_topic", octomap_topic_, "/octomap");
    
    double robot_radius;
    int max_iterations, snap_search_radius_cells;
    bool require_ground_support, strict_direct_ground_support;
    int ground_support_xy_radius_cells, ground_support_depth_cells;
    int max_step_height_cells, robot_clearance_height_cells;
    bool enable_preblocked_costmap;
    int preblocked_costmap_radius_cells;
    double preblocked_costmap_weight;
    bool lowest_traversable_only;

    private_nh.param<double>("robot_radius", robot_radius, 0.20);
    private_nh.param<int>("max_iterations", max_iterations, 250000);
    private_nh.param<int>("snap_search_radius_cells", snap_search_radius_cells, 8);
    private_nh.param<bool>("require_ground_support", require_ground_support, true);
    private_nh.param<bool>("strict_direct_ground_support", strict_direct_ground_support, true);
    private_nh.param<int>("ground_support_xy_radius_cells", ground_support_xy_radius_cells, 1);
    private_nh.param<int>("ground_support_depth_cells", ground_support_depth_cells, 2);
    private_nh.param<int>("max_step_height_cells", max_step_height_cells, 1);
    private_nh.param<int>("robot_clearance_height_cells", robot_clearance_height_cells, 0);
    private_nh.param<bool>("enable_preblocked_costmap", enable_preblocked_costmap, true);
    private_nh.param<int>("preblocked_costmap_radius_cells", preblocked_costmap_radius_cells, 3);
    private_nh.param<double>("preblocked_costmap_weight", preblocked_costmap_weight, 1.5);
    private_nh.param<bool>("lowest_traversable_only", lowest_traversable_only, false);

    private_nh.param<double>("octomap_resolution", octomap_resolution_, 0.2);
    private_nh.param<double>("wall_height_m", wall_height_m_, 1.0);
    private_nh.param<double>("floor_z_m", floor_z_m_, 0.0);
    private_nh.param<int>("occupied_threshold", occupied_threshold_, 50);

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

    // Map initialization with Priority Check & Fallback
    bool source_found = false;

    // Priority 1: Local Map File
    if (!local_map_path_.empty()) {
      ROS_INFO("OctoGlobalPlanner: Attempting to load local map file: %s", local_map_path_.c_str());
      std::shared_ptr<octomap::OcTree> octree;
      if (local_map_path_.length() > 3 && local_map_path_.substr(local_map_path_.length() - 3) == ".bt") {
        octree = std::make_shared<octomap::OcTree>(octomap_resolution_);
        if (octree->readBinary(local_map_path_)) {
          source_found = true;
        }
      } else {
        octomap::AbstractOcTree* abstract_tree = octomap::AbstractOcTree::read(local_map_path_);
        if (abstract_tree) {
          octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(abstract_tree));
          if (octree) {
            source_found = true;
          } else {
            delete abstract_tree;
          }
        }
      }

      if (source_found) {
        planner_.setOctree(octree);
        planner_.rebuildAllLayers();
        map_ready_ = true;
        ROS_INFO("OctoGlobalPlanner: Active map source in use: Local Map File [%s]", local_map_path_.c_str());
      } else {
        ROS_ERROR("OctoGlobalPlanner: Failed to load local map file '%s'. Downgrading to next priority...", local_map_path_.c_str());
      }
    }

    // Priority 2: Map Topic (OccupancyGrid)
    ros::NodeHandle nh;
    if (!source_found && !map_topic_.empty()) {
      map_sub_ = nh.subscribe(map_topic_, 1, &OctoGlobalPlanner::onOccupancyGrid, this);
      ROS_INFO("OctoGlobalPlanner: Map source configured: Map Topic [%s]", map_topic_.c_str());
      source_found = true;
    }

    // Priority 3: Octomap Topic (Octomap)
    if (!source_found && !octomap_topic_.empty()) {
      octomap_sub_ = nh.subscribe(octomap_topic_, 1, &OctoGlobalPlanner::onOctomap, this);
      ROS_INFO("OctoGlobalPlanner: Map source configured: Octomap Topic [%s]", octomap_topic_.c_str());
      source_found = true;
    }

    if (!source_found) {
      ROS_ERROR("OctoGlobalPlanner: No map source configured! Please set local_map_path, map_topic, or octomap_topic.");
    }

    // Advertise the full global plan topic globally under move_base namespace (~plan resolves to /move_base/plan)
    ros::NodeHandle move_base_nh("~");
    plan_pub_ = move_base_nh.advertise<nav_msgs::Path>("plan", 1, true);

    // Initialize status publisher
    status_pub_ = nh.advertise<std_msgs::String>("/move_base/status_text", 1, true);

    initialized_ = true;
    if (map_ready_) {
      ROS_INFO("OctoGlobalPlanner initialized successfully with pre-loaded map.");
    } else {
      ROS_INFO("OctoGlobalPlanner initialized successfully. Waiting for map message...");
    }
  }

  virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan) override
  {
    if (!initialized_)
    {
      ROS_ERROR("OctoGlobalPlanner is not initialized. Please call initialize() first.");
      return false;
    }

    if (!map_ready_)
    {
      ROS_WARN_THROTTLE(2.0, "OctoGlobalPlanner cannot plan because Octomap is not ready.");
      publishStatus("Global planner failed: Octomap is not ready.");
      return false;
    }

    std::vector<GridIndex> cells;
    std::string error_msg;
    bool ok = planner_.plan(start.pose.position, goal.pose.position, cells, error_msg);
    if (!ok)
    {
      ROS_WARN_THROTTLE(2.0, "OctoGlobalPlanner failed to find a plan: %s", error_msg.c_str());
      publishStatus("Global planner failed: " + error_msg);
      return false;
    }

    plan.clear();
    plan.reserve(cells.size());
    ros::Time plan_time = ros::Time::now();

    for (size_t i = 0; i < cells.size(); ++i)
    {
      const auto p = planner_.gridToWorld(cells[i]);
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = start.header.frame_id;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation.w = 1.0;
      
      // Preserve original orientation at the start/goal if applicable
      if (i == 0)
      {
        pose.pose.orientation = start.pose.orientation;
      }
      else if (i + 1 == cells.size())
      {
        pose.pose.orientation = goal.pose.orientation;
      }
      
      plan.push_back(pose);
    }

    // Manually publish the full plan
    nav_msgs::Path path_msg;
    path_msg.header.stamp = plan_time;
    path_msg.header.frame_id = start.header.frame_id;
    path_msg.poses = plan;
    plan_pub_.publish(path_msg);

    publishStatus("Global planner: Path found successfully.");
    ROS_INFO_THROTTLE(1.0, "OctoGlobalPlanner: Path found with %zu points. Published to /move_base/plan", plan.size());
    return true;
  }

private:
  void onOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
  {
    static bool printed = false;
    if (!printed) {
      ROS_INFO("OctoGlobalPlanner: Active map source in use: Octomap Topic [%s]", octomap_topic_.c_str());
      printed = true;
    }

    std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
    if (!octree)
    {
      ROS_ERROR("OctoGlobalPlanner: Failed to convert OctoMap message to OcTree.");
      return;
    }
    planner_.setOctree(octree);
    planner_.rebuildAllLayers();
    map_ready_ = true;
  }

  void onOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr & msg)
  {
    static bool printed = false;
    if (!printed) {
      ROS_INFO("OctoGlobalPlanner: Active map source in use: Map Topic [%s]", map_topic_.c_str());
      printed = true;
    }

    const double grid_resolution = msg->info.resolution;
    if (grid_resolution <= 0.0 || octomap_resolution_ <= 0.0) {
      ROS_ERROR("OctoGlobalPlanner: Grid and OctoMap resolutions must be positive.");
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
    planner_.rebuildAllLayers();
    map_ready_ = true;
  }

  void publishStatus(const std::string& status)
  {
    if (status != last_status_)
    {
      last_status_ = status;
      std_msgs::String msg;
      msg.data = status;
      status_pub_.publish(msg);
    }
  }

  bool initialized_;
  bool map_ready_;
  
  std::string local_map_path_;
  std::string map_topic_;
  std::string octomap_topic_;
  double octomap_resolution_;
  double wall_height_m_;
  double floor_z_m_;
  int occupied_threshold_;

  ros::Subscriber octomap_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher plan_pub_;
  ros::Publisher status_pub_;
  std::string last_status_;
  OctoPlannerCore planner_;
};

} // namespace octo_planner

PLUGINLIB_EXPORT_CLASS(octo_planner::OctoGlobalPlanner, nav_core::BaseGlobalPlanner)
