#ifndef OCTO_PLANNER_CORE_H
#define OCTO_PLANNER_CORE_H

#include <vector>
#include <atomic>
#include <mutex>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <queue>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstddef>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap/OcTree.h>

namespace octo_planner
{

struct GridIndex
{
  int x;
  int y;
  int z;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & k) const
  {
    std::size_t seed = 0;
    seed ^= std::hash<int>{}(k.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>{}(k.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>{}(k.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct QueueNode
{
  GridIndex idx;
  double f;
  double g;
};

struct QueueNodeCompare
{
  bool operator()(const QueueNode & a, const QueueNode & b) const
  {
    return a.f > b.f;
  }
};

struct CellDebugDetails
{
  int grid_x = 0;
  int grid_y = 0;
  int grid_z = 0;
  bool is_occupied = false;
  bool is_unknown = false;
  bool has_ground_support = false;
  bool is_preblocked = false;
  std::string preblocked_reason = "none";
  bool has_vertical_collision = false;
  bool has_horizontal_collision = false;
  bool has_below_preblocked_failure = false;
  double preblocked_cost = 0.0;
  double risk_cost = 0.0;
  bool is_candidate = false;
  bool is_traversable = false;
};

class OctoPlannerCore
{
public:
  OctoPlannerCore();
  ~OctoPlannerCore() = default;

  // Setters/Getters for parameters
  void setRobotRadius(double r) { robot_radius_ = r; }
  void setMaxIterations(int val) { max_iterations_ = val; }
  void setSnapSearchRadiusCells(int val) { snap_search_radius_cells_ = val; }
  void setCancelFlag(bool val) { cancel_ = val; }
  bool isCancelled() const { return cancel_; }
  void setRequireGroundSupport(bool val) { require_ground_support_ = val; }
  void setStrictDirectGroundSupport(bool val) { strict_direct_ground_support_ = val; }
  void setGroundSupportXYRadiusCells(int val) { ground_support_xy_radius_cells_ = val; }
  void setGroundSupportDepthCells(int val) { ground_support_depth_cells_ = val; }
  void setMaxStepHeightCells(int val) { max_step_height_cells_ = val; }
  void setRobotClearanceHeightCells(int val) { robot_clearance_height_cells_ = val; }
  void setEnablePreblockedCostmap(bool val) { enable_preblocked_costmap_ = val; }
  void setPreblockedCostmapRadiusCells(int val) { preblocked_costmap_radius_cells_ = val; }
  void setPreblockedCostmapWeight(double val) { preblocked_costmap_weight_ = val; }
  void setLowestTraversableOnly(bool val) { lowest_traversable_only_ = val; }
  void setEnablePathShortcut(bool val) { enable_path_shortcut_ = val; }
  bool getEnablePathShortcut() const { return enable_path_shortcut_; }
  void setEnablePathSmoothing(bool val) { enable_path_smoothing_ = val; }
  bool getEnablePathSmoothing() const { return enable_path_smoothing_; }
  void setPathInterpolationResolution(double val) { path_interpolation_resolution_ = val; }
  double getPathInterpolationResolution() const { return path_interpolation_resolution_; }
  void setCornerFilletRadius(double val) { corner_fillet_radius_ = val; }
  double getCornerFilletRadius() const { return corner_fillet_radius_; }
  void setEnableContinuousYaw(bool val) { enable_continuous_yaw_ = val; }
  bool getEnableContinuousYaw() const { return enable_continuous_yaw_; }
  void setYawSmoothingWindow(int val) { yaw_smoothing_window_ = val; }
  int getYawSmoothingWindow() const { return yaw_smoothing_window_; }

  // Octomap management
  bool setOctree(const std::shared_ptr<octomap::OcTree>& octree);
  std::shared_ptr<octomap::OcTree> getOctree() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return octree_;
  }

  // External constraints
  void setExternalPreblockedCells(const std::unordered_set<GridIndex, GridIndexHash>& cells);
  void clearExternalPreblockedCells() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    external_preblocked_cells_.clear();
  }

  // Layer rebuilds
  void rebuildPreblockedCells();
  void rebuildDerivedLayers();
  void rebuildPreblockedCostmap();
  void rebuildAllLayers();

  // Getters for layers
  std::unordered_set<GridIndex, GridIndexHash> getPreblockedCells() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return preblocked_cells_;
  }
  std::unordered_set<GridIndex, GridIndexHash> getExternalPreblockedCells() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return external_preblocked_cells_;
  }
  std::unordered_set<GridIndex, GridIndexHash> getTraversableCells() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return traversable_cells_;
  }
  std::unordered_map<GridIndex, double, GridIndexHash> getPreblockedCostmap() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return preblocked_costmap_;
  }

  // Coordinates converters
  GridIndex worldToGrid(double x, double y, double z) const;
  octomap::point3d gridToWorld(const GridIndex & idx) const;

  // Planning interface
  bool plan(const geometry_msgs::Point& start_pt, 
            const geometry_msgs::Point& goal_pt, 
            std::vector<GridIndex>& path_cells,
            std::string & error_msg);

  // Path optimization and post-processing
  bool isLineTraversable(const GridIndex & from, const GridIndex & to) const;
  bool isLineTraversable(const octomap::point3d & from, const octomap::point3d & to) const;
  std::vector<GridIndex> shortcutPath(const std::vector<GridIndex> & raw_path) const;
  std::vector<geometry_msgs::PoseStamped> generateSmoothPath(
    const std::vector<GridIndex> & cells,
    const geometry_msgs::PoseStamped & start_pose,
    const geometry_msgs::PoseStamped & goal_pose,
    bool has_goal_pose = false) const;

  bool isInsideMetricBounds(const GridIndex & idx) const;
  bool isCellTraversable(const GridIndex & idx, double robot_radius,
                        bool require_ground_support, bool strict, int xy_r, int depth) const;
  bool findNearestFreeCell(const GridIndex & seed, double robot_radius, int radius_cells,
                           bool require_ground_support, bool strict, int xy_r, int depth, GridIndex & out) const;
  bool queryCellDebugInfo(const GridIndex & idx, CellDebugDetails & details) const;

private:
  double euclidean(const GridIndex & a, const GridIndex & b) const;
  bool isDiagonalTransitionValid(const GridIndex & from, const GridIndex & to) const;
  bool hasGroundSupport(const GridIndex & idx, bool strict, int xy_r, int depth) const;
  bool isOccupiedCell(const GridIndex & idx) const;
  bool hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const;
  bool hasSameLevelNeighborWithOccupiedBelow(const GridIndex & idx) const;
  bool hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const;
  double getPreblockedCost(const GridIndex & idx) const;
  std::vector<GridIndex> makeDirections() const;
  std::string getPreblockedReason(const GridIndex & idx) const;

  // Parameters
  double robot_radius_;
  int max_iterations_;
  int snap_search_radius_cells_;
  bool require_ground_support_;
  bool strict_direct_ground_support_;
  int ground_support_xy_radius_cells_;
  int ground_support_depth_cells_;
  int max_step_height_cells_;
  int robot_clearance_height_cells_;
  bool enable_preblocked_costmap_;
  int preblocked_costmap_radius_cells_;
  double preblocked_costmap_weight_;
  bool lowest_traversable_only_;
  bool enable_path_shortcut_;
  bool enable_path_smoothing_;
  double path_interpolation_resolution_;
  double corner_fillet_radius_;
  bool enable_continuous_yaw_;
  int yaw_smoothing_window_;

  // State
  std::shared_ptr<octomap::OcTree> octree_;
  GridIndex min_idx_, max_idx_;
  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::unordered_set<GridIndex, GridIndexHash> candidates_;
  std::unordered_set<GridIndex, GridIndexHash> preblocked_cells_;
  std::unordered_set<GridIndex, GridIndexHash> external_preblocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> preblocked_costmap_;
  std::atomic<bool> cancel_;
  mutable std::recursive_mutex mutex_;
};

} // namespace octo_planner

#endif // OCTO_PLANNER_CORE_H
