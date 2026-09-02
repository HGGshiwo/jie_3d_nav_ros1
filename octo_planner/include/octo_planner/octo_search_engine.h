#ifndef OCTO_SEARCH_ENGINE_H
#define OCTO_SEARCH_ENGINE_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <string>
#include <atomic>
#include <memory>
#include <algorithm>
#include <mutex>
#include <geometry_msgs/Point.h>

#include "octo_planner/octo_planner_core.h"

namespace octo_planner
{

struct SearchQueueNode
{
  GridIndex idx;
  double f;
  double g;
  double h;
};

struct SearchQueueNodeCompare
{
  bool operator()(const SearchQueueNode & a, const SearchQueueNode & b) const
  {
    if (std::abs(a.f - b.f) > 1e-6)
    {
      return a.f > b.f; // Min-heap by f value
    }
    return a.h > b.h; // Tie-breaker: prefer node closer to target (smaller h)
  }
};

class OctoPlannerCore;

class OctoSearchEngine
{
public:
  OctoSearchEngine();
  ~OctoSearchEngine() = default;

  // Parameter Setters/Getters
  void setMaxIterations(int val) { max_iterations_ = val; }
  int getMaxIterations() const { return max_iterations_; }

  void setHeuristicWeight(double val) { heuristic_weight_ = std::max(1.0, val); }
  double getHeuristicWeight() const { return heuristic_weight_; }

  void setSearchFromGoal(bool val) { search_from_goal_ = val; }
  bool getSearchFromGoal() const { return search_from_goal_; }

  void setEnableGoalCache(bool val) { enable_goal_cache_ = val; }
  bool getEnableGoalCache() const { return enable_goal_cache_; }

  // Clear goal search cache when map or environment changes
  void invalidateCache();

  // Primary search interface
  bool plan(const OctoPlannerCore & planner,
            const GridIndex & start_cell,
            const GridIndex & goal_cell,
            bool enable_preblocked_costmap,
            double preblocked_costmap_weight,
            int max_step_height_cells,
            const std::atomic<bool> & cancel_flag,
            std::vector<GridIndex> & path_cells,
            std::string & error_msg);

private:
  double euclidean(const GridIndex & a, const GridIndex & b) const;
  std::vector<GridIndex> makeDirections(int max_step_height_cells) const;

  // Core search parameters
  int max_iterations_;
  double heuristic_weight_;
  bool search_from_goal_;
  bool enable_goal_cache_;

  // Cache state for Goal-to-Start search
  bool cache_valid_;
  GridIndex cached_root_goal_;
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> cached_came_from_;
  std::unordered_map<GridIndex, double, GridIndexHash> cached_g_score_;
  std::unordered_set<GridIndex, GridIndexHash> cached_closed_set_;
};

} // namespace octo_planner

#endif // OCTO_SEARCH_ENGINE_H
