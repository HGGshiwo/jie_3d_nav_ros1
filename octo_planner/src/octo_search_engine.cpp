#include "octo_planner/octo_search_engine.h"
#include "octo_planner/octo_planner_core.h"

namespace octo_planner
{

OctoSearchEngine::OctoSearchEngine()
: max_iterations_(250000),
  heuristic_weight_(1.15),
  search_from_goal_(true),
  enable_goal_cache_(true),
  cache_valid_(false),
  cached_root_goal_{0, 0, 0}
{
}

void OctoSearchEngine::invalidateCache()
{
  cache_valid_ = false;
  cached_root_goal_ = GridIndex{0, 0, 0};
  cached_came_from_.clear();
  cached_g_score_.clear();
  cached_closed_set_.clear();
}

double OctoSearchEngine::euclidean(const GridIndex & a, const GridIndex & b) const
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<GridIndex> OctoSearchEngine::makeDirections(int max_step_height_cells) const
{
  std::vector<GridIndex> dirs;
  const int h = std::max(0, max_step_height_cells);
  dirs.reserve(9 * (2 * h + 1) - 1);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -h; dz <= h; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) continue;
        dirs.push_back(GridIndex{dx, dy, dz});
      }
    }
  }
  return dirs;
}

bool OctoSearchEngine::plan(const OctoPlannerCore & planner,
                           const GridIndex & start_cell,
                           const GridIndex & goal_cell,
                           bool enable_preblocked_costmap,
                           double preblocked_costmap_weight,
                           int max_step_height_cells,
                           const std::atomic<bool> & cancel_flag,
                           std::vector<GridIndex> & path_cells,
                           std::string & error_msg)
{
  path_cells.clear();

  if (start_cell == goal_cell) {
    path_cells.push_back(start_cell);
    return true;
  }

  const std::vector<GridIndex> directions = makeDirections(max_step_height_cells);
  const auto & traversable_cells = planner.getTraversableCells();

  // Mode 1: Search from Goal to Start (Backward A*)
  if (search_from_goal_)
  {
    // A. Check Goal Search Tree Cache
    if (enable_goal_cache_ && cache_valid_ && cached_root_goal_ == goal_cell)
    {
      if (cached_came_from_.find(start_cell) != cached_came_from_.end())
      {
        GridIndex curr = start_cell;
        path_cells.push_back(curr);
        while (curr == goal_cell ? false : true)
        {
          auto it = cached_came_from_.find(curr);
          if (it == cached_came_from_.end()) break;
          curr = it->second;
          path_cells.push_back(curr);
        }
        if (!path_cells.empty() && path_cells.back() == goal_cell) {
          return true; // Zero-latency cache hit!
        }
        path_cells.clear();
      }
    }

    // B. Re-initialize search tree from Goal if root goal changed or cache invalid
    if (!cache_valid_ || cached_root_goal_ != goal_cell || !enable_goal_cache_)
    {
      invalidateCache();
      cache_valid_ = true;
      cached_root_goal_ = goal_cell;
    }

    std::priority_queue<SearchQueueNode, std::vector<SearchQueueNode>, SearchQueueNodeCompare> open_set;

    // Seed open_set from current cache or start fresh at goal_cell
    if (cached_g_score_.empty()) {
      cached_g_score_[goal_cell] = 0.0;
      double h0 = euclidean(goal_cell, start_cell);
      open_set.push(SearchQueueNode{goal_cell, h0 * heuristic_weight_, 0.0, h0});
    } else {
      // Re-populate priority queue with un-closed frontier nodes, re-evaluating h towards new start_cell
      for (const auto & entry : cached_g_score_) {
        if (cached_closed_set_.find(entry.first) == cached_closed_set_.end()) {
          double h_val = euclidean(entry.first, start_cell);
          open_set.push(SearchQueueNode{entry.first, entry.second + h_val * heuristic_weight_, entry.second, h_val});
        }
      }
    }

    int iters = 0;
    while (!open_set.empty() && iters < max_iterations_)
    {
      if (cancel_flag) {
        error_msg = "Planning cancelled by a new request.";
        return false;
      }

      const SearchQueueNode current = open_set.top();
      open_set.pop();
      ++iters;

      if (cached_closed_set_.find(current.idx) != cached_closed_set_.end()) continue;
      cached_closed_set_.insert(current.idx);

      if (current.idx == start_cell)
      {
        // Reconstruct path from Start to Goal (cached_came_from_ maps child->parent towards Goal)
        GridIndex curr = start_cell;
        path_cells.push_back(curr);
        while (curr == goal_cell ? false : true)
        {
          auto it = cached_came_from_.find(curr);
          if (it == cached_came_from_.end()) break;
          curr = it->second;
          path_cells.push_back(curr);
        }
        return true;
      }

      for (const auto & d : directions)
      {
        GridIndex nbr{current.idx.x + d.x, current.idx.y + d.y, current.idx.z + d.z};
        if (cached_closed_set_.find(nbr) != cached_closed_set_.end()) continue;
        if (traversable_cells.find(nbr) == traversable_cells.end()) continue;

        double tentative_g = current.g + euclidean(current.idx, nbr);
        if (enable_preblocked_costmap) {
          tentative_g += preblocked_costmap_weight * planner.getPreblockedCost(nbr);
        }

        auto g_it = cached_g_score_.find(nbr);
        if (g_it == cached_g_score_.end() || tentative_g < g_it->second)
        {
          cached_came_from_[nbr] = current.idx;
          cached_g_score_[nbr] = tentative_g;
          double h_val = euclidean(nbr, start_cell);
          open_set.push(SearchQueueNode{nbr, tentative_g + h_val * heuristic_weight_, tentative_g, h_val});
        }
      }
    }

    if (iters >= max_iterations_) {
      error_msg = "A* planning timed out (reached max_iterations limit).";
    } else {
      error_msg = "No traversable path exists (open_set became empty / graph disconnected).";
    }
    return false;
  }

  // Mode 2: Forward A* (Start to Goal)
  std::priority_queue<SearchQueueNode, std::vector<SearchQueueNode>, SearchQueueNodeCompare> open_set;
  std::unordered_map<GridIndex, double, GridIndexHash> g_score;
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
  std::unordered_set<GridIndex, GridIndexHash> closed_set;

  g_score[start_cell] = 0.0;
  double h0 = euclidean(start_cell, goal_cell);
  open_set.push(SearchQueueNode{start_cell, h0 * heuristic_weight_, 0.0, h0});
  int iters = 0;

  while (!open_set.empty() && iters < max_iterations_)
  {
    if (cancel_flag) {
      error_msg = "Planning cancelled by a new request.";
      return false;
    }

    const SearchQueueNode current = open_set.top();
    open_set.pop();
    ++iters;

    if (closed_set.find(current.idx) != closed_set.end()) continue;
    closed_set.insert(current.idx);

    if (current.idx == goal_cell)
    {
      GridIndex curr = goal_cell;
      path_cells.push_back(curr);
      while (came_from.find(curr) != came_from.end()) {
        curr = came_from.at(curr);
        path_cells.push_back(curr);
      }
      std::reverse(path_cells.begin(), path_cells.end());
      return true;
    }

    for (const auto & d : directions)
    {
      GridIndex nbr{current.idx.x + d.x, current.idx.y + d.y, current.idx.z + d.z};
      if (closed_set.find(nbr) != closed_set.end()) continue;
      if (traversable_cells.find(nbr) == traversable_cells.end()) continue;

      double tentative_g = current.g + euclidean(current.idx, nbr);
      if (enable_preblocked_costmap) {
        tentative_g += preblocked_costmap_weight * planner.getPreblockedCost(nbr);
      }

      auto g_it = g_score.find(nbr);
      if (g_it == g_score.end() || tentative_g < g_it->second)
      {
        came_from[nbr] = current.idx;
        g_score[nbr] = tentative_g;
        double h_val = euclidean(nbr, goal_cell);
        open_set.push(SearchQueueNode{nbr, tentative_g + h_val * heuristic_weight_, tentative_g, h_val});
      }
    }
  }

  if (iters >= max_iterations_) {
    error_msg = "A* planning timed out (reached max_iterations limit).";
  } else {
    error_msg = "No traversable path exists (open_set became empty / graph disconnected).";
  }
  return false;
}

} // namespace octo_planner
