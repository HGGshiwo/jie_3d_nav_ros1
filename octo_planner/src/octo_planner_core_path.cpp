#include "octo_planner/octo_planner_core.h"
#include <cmath>
#include <algorithm>
#include <vector>

namespace octo_planner
{

std::vector<GridIndex> OctoPlannerCore::makeDirections() const
{
  std::vector<GridIndex> dirs;
  dirs.reserve(9 * (2 * max_step_height_cells_ + 1) - 1);
  for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy)
      for (int dz = -max_step_height_cells_; dz <= max_step_height_cells_; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) continue;
        dirs.push_back(GridIndex{dx, dy, dz});
      }
  return dirs;
}

static std::vector<GridIndex> reconstructPath(
  const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
  GridIndex current)
{
  std::vector<GridIndex> path;
  path.push_back(current);
  while (came_from.find(current) != came_from.end()) {
    current = came_from.at(current);
    path.push_back(current);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

bool OctoPlannerCore::plan(const geometry_msgs::Point& start_pt, 
                           const geometry_msgs::Point& goal_pt, 
                           std::vector<GridIndex>& path_cells,
                           std::string & error_msg,
                           double start_yaw)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  path_cells.clear();
  if (!octree_) {
    error_msg = "OctoMap is not ready/loaded.";
    return false;
  }

  const GridIndex start = worldToGrid(start_pt.x, start_pt.y, start_pt.z);
  const GridIndex goal = worldToGrid(goal_pt.x, goal_pt.y, goal_pt.z);

  GridIndex valid_start = start;
  GridIndex valid_goal = goal;

  const bool is_descending = (goal_pt.z < start_pt.z - 0.05);
  const double fwd_yaw = !std::isnan(start_yaw) ? start_yaw : std::atan2(goal_pt.y - start_pt.y, goal_pt.x - start_pt.x);

  if (!findNearestFreeCell(start, robot_radius_, snap_search_radius_cells_,
        require_ground_support_, strict_direct_ground_support_,
        ground_support_xy_radius_cells_, ground_support_depth_cells_, valid_start,
        is_descending, fwd_yaw, true)) {
    error_msg = "Start cell is invalid/occupied and no free cell found nearby.";
    return false;
  }

  if (!findNearestFreeCell(goal, robot_radius_, snap_search_radius_cells_,
        require_ground_support_, strict_direct_ground_support_,
        ground_support_xy_radius_cells_, ground_support_depth_cells_, valid_goal,
        is_descending)) {
    error_msg = "Goal cell is invalid/occupied and no free cell found nearby.";
    return false;
  }

  if (valid_start == valid_goal) {
    path_cells.push_back(valid_start);
    return true;
  }

  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
  std::unordered_map<GridIndex, double, GridIndexHash> g_score;
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
  std::unordered_set<GridIndex, GridIndexHash> closed_set;

  g_score.reserve(8192);
  came_from.reserve(8192);
  closed_set.reserve(8192);

  const double initial_h = euclidean(valid_start, valid_goal);
  g_score[valid_start] = 0.0;
  open_set.push(QueueNode{valid_start, heuristic_weight_ * initial_h, 0.0});

  // 8 个水平移动方向
  static const int h_dirs[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };
  const int max_step = std::max(1, max_step_height_cells_);

  // 用于平局打破（Tie-breaking）的起终点基准向量
  const double dx_sg = static_cast<double>(valid_start.x - valid_goal.x);
  const double dy_sg = static_cast<double>(valid_start.y - valid_goal.y);

  int iters = 0;
  while (!open_set.empty() && iters < max_iterations_) {
    if (cancel_) {
      error_msg = "Planning cancelled by a new request.";
      return false;
    }

    const QueueNode current = open_set.top();
    open_set.pop();
    ++iters;

    if (closed_set.find(current.idx) != closed_set.end()) continue;
    closed_set.insert(current.idx);

    if (current.idx == valid_goal) {
      path_cells = reconstructPath(came_from, current.idx);
      return true;
    }

    for (int d = 0; d < 8; ++d) {
      const int dx = h_dirs[d][0];
      const int dy = h_dirs[d][1];

      GridIndex nbr;
      bool found_nbr = false;

      const bool prefer_down = (current.idx.z > valid_goal.z);

      if (!traversable_cells_.empty()) {
        // 智能地表跟随：从当前高度放射状探查最贴近的地表网格，下坡/下楼优先向下探测(-dz)，上坡优先向上(+dz)
        for (int dz_mag = 0; dz_mag <= max_step; ++dz_mag) {
          const int dz_cands[2] = {prefer_down ? -dz_mag : dz_mag, prefer_down ? dz_mag : -dz_mag};
          const int num_c = (dz_mag == 0) ? 1 : 2;
          for (int k = 0; k < num_c; ++k) {
            const int dz = dz_cands[k];
            const GridIndex cand{current.idx.x + dx, current.idx.y + dy, current.idx.z + dz};
            if (closed_set.find(cand) != closed_set.end()) continue;
            if (traversable_cells_.find(cand) != traversable_cells_.end()) {
              nbr = cand;
              found_nbr = true;
              break;
            }
          }
          if (found_nbr) break;
        }
      } else {
        // 动态局部规划模式：同样下楼梯优先向下探测
        for (int dz_mag = 0; dz_mag <= max_step; ++dz_mag) {
          const int dz_cands[2] = {prefer_down ? -dz_mag : dz_mag, prefer_down ? dz_mag : -dz_mag};
          const int num_c = (dz_mag == 0) ? 1 : 2;
          for (int k = 0; k < num_c; ++k) {
            const int dz = dz_cands[k];
            const GridIndex cand{current.idx.x + dx, current.idx.y + dy, current.idx.z + dz};
            if (closed_set.find(cand) != closed_set.end()) continue;
            if (isCellTraversable(cand, robot_radius_, require_ground_support_,
                                  strict_direct_ground_support_, ground_support_xy_radius_cells_,
                                  ground_support_depth_cells_)) {
              nbr = cand;
              found_nbr = true;
              break;
            }
          }
          if (found_nbr) break;
        }
      }

      if (!found_nbr) continue;

      double tentative_g = current.g + euclidean(current.idx, nbr);
      if (enable_preblocked_costmap_)
        tentative_g += preblocked_costmap_weight_ * getPreblockedCost(nbr);

      auto g_it = g_score.find(nbr);
      if (g_it == g_score.end() || tentative_g < g_it->second) {
        came_from[nbr] = current.idx;
        g_score[nbr] = tentative_g;

        const double h = euclidean(nbr, valid_goal);
        // Tie-breaking: 优先沿起终点连线直行，抑制同心圆泛洪
        const double dx_ng = static_cast<double>(nbr.x - valid_goal.x);
        const double dy_ng = static_cast<double>(nbr.y - valid_goal.y);
        const double cross = std::abs(dx_ng * dy_sg - dy_ng * dx_sg);
        const double f_priority = tentative_g + heuristic_weight_ * h + cross * 0.0005;

        open_set.push(QueueNode{nbr, f_priority, tentative_g});
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

bool OctoPlannerCore::isLineTraversable(const GridIndex & from, const GridIndex & to) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!octree_) return false;
  if (!isInsideMetricBounds(from) || !isInsideMetricBounds(to)) return false;

  const auto p1 = gridToWorld(from);
  const auto p2 = gridToWorld(to);
  const double dist = std::sqrt(
    (p2.x() - p1.x()) * (p2.x() - p1.x()) +
    (p2.y() - p1.y()) * (p2.y() - p1.y()) +
    (p2.z() - p1.z()) * (p2.z() - p1.z()));

  const double res = octree_->getResolution();
  const double step_size = std::max(0.01, res * 0.4);
  const int num_steps = std::max(1, static_cast<int>(std::ceil(dist / step_size)));

  GridIndex last_cell = from;

  for (int i = 0; i <= num_steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(num_steps);
    const double x = p1.x() + t * (p2.x() - p1.x());
    const double y = p1.y() + t * (p2.y() - p1.y());
    const double z = p1.z() + t * (p2.z() - p1.z());

    const GridIndex cell = worldToGrid(x, y, z);
    if (!isInsideMetricBounds(cell)) return false;

    // Check traversability: if in traversable_cells_ or passes isCellTraversable
    if (traversable_cells_.find(cell) == traversable_cells_.end()) {
      if (!isCellTraversable(cell, robot_radius_, require_ground_support_,
                             strict_direct_ground_support_, ground_support_xy_radius_cells_,
                             ground_support_depth_cells_)) {
        return false;
      }
    }

    if (preblocked_cells_.find(cell) != preblocked_cells_.end()) {
      return false;
    }

    if (cell.x != last_cell.x || cell.y != last_cell.y || cell.z != last_cell.z) {
      if (std::abs(cell.z - last_cell.z) > max_step_height_cells_) {
        return false;
      }
      if (!isDiagonalTransitionValid(last_cell, cell)) {
        return false;
      }
      last_cell = cell;
    }
  }

  return true;
}

bool OctoPlannerCore::isLineTraversable(const octomap::point3d & from, const octomap::point3d & to) const
{
  return isLineTraversable(worldToGrid(from.x(), from.y(), from.z()),
                           worldToGrid(to.x(), to.y(), to.z()));
}

} // namespace octo_planner

