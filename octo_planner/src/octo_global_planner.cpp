#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <pluginlib/class_list_macros.h>

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
    
    std::string octomap_topic;
    private_nh.param<std::string>("octomap_topic", octomap_topic, "/octomap");
    
    double robot_radius;
    int max_iterations, snap_search_radius_cells;
    bool require_ground_support, strict_direct_ground_support;
    int ground_support_xy_radius_cells, ground_support_depth_cells;
    int max_step_height_cells, robot_clearance_height_cells;
    bool enable_preblocked_costmap;
    int preblocked_costmap_radius_cells;
    double preblocked_costmap_weight;
    bool lowest_traversable_only;
    bool enable_path_shortcut, enable_path_smoothing, enable_continuous_yaw;
    double path_interpolation_resolution, corner_fillet_radius;
    int yaw_smoothing_window;

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
    private_nh.param<bool>("enable_path_shortcut", enable_path_shortcut, true);
    private_nh.param<bool>("enable_path_smoothing", enable_path_smoothing, true);
    private_nh.param<double>("path_interpolation_resolution", path_interpolation_resolution, 0.05);
    private_nh.param<double>("corner_fillet_radius", corner_fillet_radius, 0.30);
    private_nh.param<bool>("enable_continuous_yaw", enable_continuous_yaw, true);
    private_nh.param<int>("yaw_smoothing_window", yaw_smoothing_window, 5);

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
    planner_.setEnablePathShortcut(enable_path_shortcut);
    planner_.setEnablePathSmoothing(enable_path_smoothing);
    planner_.setPathInterpolationResolution(path_interpolation_resolution);
    planner_.setCornerFilletRadius(corner_fillet_radius);
    planner_.setEnableContinuousYaw(enable_continuous_yaw);
    planner_.setYawSmoothingWindow(yaw_smoothing_window);

    // Subscribe to Octomap
    ros::NodeHandle nh;
    octomap_sub_ = nh.subscribe(octomap_topic, 1, &OctoGlobalPlanner::onOctomap, this);

    // Advertise the full global plan topic globally under move_base namespace (~plan resolves to /move_base/plan)
    ros::NodeHandle move_base_nh("~");
    plan_pub_ = move_base_nh.advertise<nav_msgs::Path>("plan", 1, true);

    initialized_ = true;
    ROS_INFO("OctoGlobalPlanner initialized successfully. Subscribed to octomap on: %s", octomap_topic.c_str());
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
      return false;
    }

    std::vector<GridIndex> cells;
    std::string error_msg;
    bool ok = planner_.plan(start.pose.position, goal.pose.position, cells, error_msg);
    if (!ok)
    {
      ROS_WARN_THROTTLE(2.0, "OctoGlobalPlanner failed to find a plan: %s", error_msg.c_str());
      return false;
    }

    // Generate smooth, interpolated path with continuous yaw
    plan = planner_.generateSmoothPath(cells, start, goal, true);

    // Manually publish the full plan
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = start.header.frame_id;
    path_msg.poses = plan;
    plan_pub_.publish(path_msg);

    ROS_INFO_THROTTLE(1.0, "OctoGlobalPlanner: Smooth path generated with %zu points (raw waypoints: %zu). Published to /move_base/plan", plan.size(), cells.size());
    return true;
  }

private:
  void onOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
  {
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

  bool initialized_;
  bool map_ready_;
  ros::Subscriber octomap_sub_;
  ros::Publisher plan_pub_;
  OctoPlannerCore planner_;
};

} // namespace octo_planner

PLUGINLIB_EXPORT_CLASS(octo_planner::OctoGlobalPlanner, nav_core::BaseGlobalPlanner)
