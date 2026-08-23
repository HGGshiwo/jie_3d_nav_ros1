#include "octo_planner/octo_planner_core.h"

namespace octo_planner
{

OctoPlannerCore::OctoPlannerCore()
: robot_radius_(0.20),
  max_iterations_(250000),
  snap_search_radius_cells_(8),
  require_ground_support_(true),
  strict_direct_ground_support_(true),
  ground_support_xy_radius_cells_(1),
  ground_support_depth_cells_(2),
  max_step_height_cells_(1),
  robot_clearance_height_cells_(0),
  enable_preblocked_costmap_(true),
  preblocked_costmap_radius_cells_(3),
  preblocked_costmap_weight_(1.5),
  lowest_traversable_only_(false),
  min_idx_{0, 0, 0},
  max_idx_{0, 0, 0}
{
}

bool OctoPlannerCore::setOctree(const std::shared_ptr<octomap::OcTree>& octree)
{
  octree_ = octree;
  if (!octree_) return false;
  
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);
  min_idx_ = worldToGrid(min_x, min_y, min_z);
  max_idx_ = worldToGrid(max_x, max_y, max_z);
  return true;
}

void OctoPlannerCore::setExternalPreblockedCells(const std::unordered_set<GridIndex, GridIndexHash>& cells)
{
  external_preblocked_cells_ = cells;
}

void OctoPlannerCore::rebuildAllLayers()
{
  rebuildPreblockedCells();
  rebuildDerivedLayers();
  rebuildPreblockedCostmap();
}

GridIndex OctoPlannerCore::worldToGrid(double x, double y, double z) const
{
  if (!octree_) return GridIndex{0, 0, 0};
  const double r = octree_->getResolution();
  return GridIndex{
    static_cast<int>(std::floor(x / r)),
    static_cast<int>(std::floor(y / r)),
    static_cast<int>(std::floor(z / r))};
}

octomap::point3d OctoPlannerCore::gridToWorld(const GridIndex & idx) const
{
  if (!octree_) return octomap::point3d(0.f, 0.f, 0.f);
  const double r = octree_->getResolution();
  return octomap::point3d(
    static_cast<float>((static_cast<double>(idx.x) + 0.5) * r),
    static_cast<float>((static_cast<double>(idx.y) + 0.5) * r),
    static_cast<float>((static_cast<double>(idx.z) + 0.5) * r));
}

bool OctoPlannerCore::isInsideMetricBounds(const GridIndex & idx) const
{
  return idx.x >= min_idx_.x && idx.x <= max_idx_.x &&
         idx.y >= min_idx_.y && idx.y <= max_idx_.y &&
         idx.z >= min_idx_.z && idx.z <= max_idx_.z;
}

double OctoPlannerCore::euclidean(const GridIndex & a, const GridIndex & b) const
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool OctoPlannerCore::hasGroundSupport(const GridIndex & idx, bool strict, int xy_r, int depth) const
{
  if (!octree_) return false;
  if (strict) {
    GridIndex below{idx.x, idx.y, idx.z - 1};
    if (!isInsideMetricBounds(below)) return false;
    const auto p = gridToWorld(below);
    const octomap::OcTreeNode * node = octree_->search(p);
    return node && octree_->isNodeOccupied(node);
  }
  for (int dz = 1; dz <= std::max(1, depth); ++dz) {
    for (int dx = -xy_r; dx <= xy_r; ++dx) {
      for (int dy = -xy_r; dy <= xy_r; ++dy) {
        GridIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
        if (!isInsideMetricBounds(below)) continue;
        const auto p = gridToWorld(below);
        const octomap::OcTreeNode * node = octree_->search(p);
        if (node && octree_->isNodeOccupied(node)) return true;
      }
    }
  }
  return false;
}

bool OctoPlannerCore::isOccupiedCell(const GridIndex & idx) const
{
  if (!octree_) return false;
  if (!isInsideMetricBounds(idx)) return false;
  const auto p = gridToWorld(idx);
  const octomap::OcTreeNode * node = octree_->search(p);
  return node && octree_->isNodeOccupied(node);
}

bool OctoPlannerCore::hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const
{
  for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
      if (!isInsideMetricBounds(n)) continue;
      if (!isOccupiedCell(n)) return true;
    }
  return false;
}

bool OctoPlannerCore::hasSameLevelNeighborWithOccupiedBelow(const GridIndex & idx) const
{
  for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
      if (!isInsideMetricBounds(n)) continue;
      const GridIndex n_below{n.x, n.y, n.z - 1};
      if (!isInsideMetricBounds(n_below)) continue;
      if (isOccupiedCell(n_below)) return true;
    }
  return false;
}

bool OctoPlannerCore::hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const
{
  for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
      if (!isInsideMetricBounds(n)) continue;
      const GridIndex n_above1{n.x, n.y, n.z + 1};
      if (!isInsideMetricBounds(n_above1)) continue;
      if (isOccupiedCell(n_above1)) return true;
    }
  return false;
}

void OctoPlannerCore::rebuildPreblockedCells()
{
  preblocked_cells_.clear();
  if (!octree_) return;

  std::unordered_set<GridIndex, GridIndexHash> candidates;
  for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
    if (!octree_->isNodeOccupied(*it)) continue;
    const GridIndex occ = worldToGrid(it.getX(), it.getY(), it.getZ());
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        candidates.insert(GridIndex{occ.x + dx, occ.y + dy, occ.z});
      }
  }

  for (const auto & c : candidates) {
    if (!isInsideMetricBounds(c) || isOccupiedCell(c)) continue;
    const GridIndex below0{c.x, c.y, c.z - 1};
    const bool below0_occ = isInsideMetricBounds(below0) && isOccupiedCell(below0);
    if (below0_occ && hasSameLevelNeighborWithOccupiedAbove(c)) {
      preblocked_cells_.insert(c);
      continue;
    }
    const GridIndex above1{c.x, c.y, c.z + 1};
    const bool above1_occ = isInsideMetricBounds(above1) && isOccupiedCell(above1);
    if (!hasNonOccupiedNeighborSameLevel(c)) continue;
    if (above1_occ) continue;
    const GridIndex below1{c.x, c.y, c.z - 1};
    if (!isInsideMetricBounds(below1)) continue;
    if (!isOccupiedCell(below1)) preblocked_cells_.insert(c);
  }

  for (const auto & c : external_preblocked_cells_) {
    if (isInsideMetricBounds(c) && !isOccupiedCell(c))
      preblocked_cells_.insert(c);
  }
}

void OctoPlannerCore::rebuildPreblockedCostmap()
{
  preblocked_costmap_.clear();
  if (!octree_ || !enable_preblocked_costmap_) return;

  const int radius_cells = std::max(1, preblocked_costmap_radius_cells_);
  const double denom = static_cast<double>(radius_cells) + 1.0;

  std::vector<std::pair<GridIndex, double>> valid_offsets;
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) continue;
        double d = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
        if (d > static_cast<double>(radius_cells)) continue;
        double cst = std::max(0.0, (denom - d) / denom);
        valid_offsets.push_back({{dx, dy, dz}, cst});
      }
    }
  }

  for (const auto & t : traversable_cells_) {
    double max_cst = 0.0;
    for (const auto & off : valid_offsets) {
      GridIndex c{t.x + off.first.x, t.y + off.first.y, t.z + off.first.z};
      if (preblocked_cells_.find(c) != preblocked_cells_.end()) {
        if (off.second > max_cst) {
          max_cst = off.second;
        }
      }
    }
    if (max_cst > 0.0) {
      preblocked_costmap_[t] = max_cst;
    }
  }
}

double OctoPlannerCore::getPreblockedCost(const GridIndex & idx) const
{
  const auto it = preblocked_costmap_.find(idx);
  return it == preblocked_costmap_.end() ? 0.0 : it->second;
}

void OctoPlannerCore::rebuildDerivedLayers()
{
  traversable_cells_.clear();
  if (!octree_) return;

  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);
  const GridIndex min_idx = worldToGrid(min_x, min_y, min_z);
  const GridIndex max_idx = worldToGrid(max_x, max_y, max_z);

  if (require_ground_support_) {
    std::unordered_set<GridIndex, GridIndexHash> candidates;
    const int xy_r = ground_support_xy_radius_cells_;
    const int depth = ground_support_depth_cells_;

    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) continue;
      const GridIndex occ = worldToGrid(it.getX(), it.getY(), it.getZ());
      
      for (int dz = 1; dz <= std::max(1, depth); ++dz) {
        for (int dx = -xy_r; dx <= xy_r; ++dx) {
          for (int dy = -xy_r; dy <= xy_r; ++dy) {
            const GridIndex candidate{occ.x + dx, occ.y + dy, occ.z + dz};
            if (isInsideMetricBounds(candidate) && !isOccupiedCell(candidate)) {
              candidates.insert(candidate);
            }
          }
        }
      }
    }

    for (const auto & idx : candidates) {
      if (isCellTraversable(idx, robot_radius_, require_ground_support_,
            strict_direct_ground_support_, ground_support_xy_radius_cells_,
            ground_support_depth_cells_))
      {
        traversable_cells_.insert(idx);
      }
    }
  } else {
    for (int x = min_idx.x; x <= max_idx.x; ++x)
      for (int y = min_idx.y; y <= max_idx.y; ++y)
        for (int z = min_idx.z; z <= max_idx.z; ++z) {
          const GridIndex idx{x, y, z};
          if (!isInsideMetricBounds(idx) || isOccupiedCell(idx)) continue;
          if (isCellTraversable(idx, robot_radius_, require_ground_support_,
                strict_direct_ground_support_, ground_support_xy_radius_cells_,
                ground_support_depth_cells_))
          {
            traversable_cells_.insert(idx);
            if (lowest_traversable_only_) break;
          }
        }
  }
}

bool OctoPlannerCore::isCellTraversable(const GridIndex & idx, double robot_radius,
  bool require_ground_support, bool strict, int xy_r, int depth) const
{
  if (!octree_) return false;
  if (!isInsideMetricBounds(idx)) return false;
  if (require_ground_support && !hasGroundSupport(idx, strict, xy_r, depth)) return false;

  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_->getMetricMin(min_x, min_y, min_z);
  const int min_z_idx = static_cast<int>(std::floor(min_z / octree_->getResolution()));

  for (int z = idx.z - 1; z >= min_z_idx; --z) {
    const GridIndex below_idx{idx.x, idx.y, z};
    if (isOccupiedCell(below_idx)) break;
    if (preblocked_cells_.find(below_idx) != preblocked_cells_.end()) return false;
  }

  const octomap::point3d center = gridToWorld(idx);
  const double r = octree_->getResolution();
  const int n = std::max(1, static_cast<int>(std::ceil(robot_radius / r)));
  const double radius_sq = robot_radius * robot_radius;

  for (int dx = -n; dx <= n; ++dx)
    for (int dy = -n; dy <= n; ++dy)
      for (int dz = robot_clearance_height_cells_; dz <= n; ++dz) {
        const double dist_sq =
          (dx * r) * (dx * r) + (dy * r) * (dy * r) + (dz * r) * (dz * r);
        if (dist_sq > radius_sq) continue;
        const octomap::point3d p(
          center.x() + static_cast<float>(dx * r),
          center.y() + static_cast<float>(dy * r),
          center.z() + static_cast<float>(dz * r));
        const GridIndex nearby_idx = worldToGrid(p.x(), p.y(), p.z());
        if (preblocked_cells_.find(nearby_idx) != preblocked_cells_.end()) return false;
        const octomap::OcTreeNode * node = octree_->search(p);
        if (node && octree_->isNodeOccupied(node)) return false;
      }
  return true;
}

bool OctoPlannerCore::findNearestFreeCell(const GridIndex & seed, double robot_radius, int radius_cells,
  bool require_ground_support, bool strict, int xy_r, int depth, GridIndex & out) const
{
  if (isCellTraversable(seed, robot_radius, require_ground_support, strict, xy_r, depth)) {
    out = seed; return true;
  }
  for (int r = 1; r <= radius_cells; ++r)
    for (int dz = 0; dz <= r; ++dz)
      for (int dx = -r; dx <= r; ++dx)
        for (int dy = -r; dy <= r; ++dy) {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) continue;
          GridIndex c1{seed.x + dx, seed.y + dy, seed.z + dz};
          if (isCellTraversable(c1, robot_radius, require_ground_support, strict, xy_r, depth)) {
            out = c1; return true;
          }
          if (dz > 0) {
            GridIndex c2{seed.x + dx, seed.y + dy, seed.z - dz};
            if (isCellTraversable(c2, robot_radius, require_ground_support, strict, xy_r, depth)) {
              out = c2; return true;
            }
          }
        }
  return false;
}

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
                           std::string & error_msg)
{
  path_cells.clear();
  if (!octree_) {
    error_msg = "OctoMap is not ready/loaded.";
    return false;
  }
  const GridIndex start_raw = worldToGrid(start_pt.x, start_pt.y, start_pt.z);
  const GridIndex goal_raw = worldToGrid(goal_pt.x, goal_pt.y, goal_pt.z);

  GridIndex start = start_raw, goal = goal_raw;
  if (!findNearestFreeCell(start_raw, robot_radius_, snap_search_radius_cells_,
        require_ground_support_, strict_direct_ground_support_,
        ground_support_xy_radius_cells_, ground_support_depth_cells_, start))
  {
    error_msg = "Start is occupied/out of map and no nearby free cell found.";
    return false;
  }
  if (!findNearestFreeCell(goal_raw, robot_radius_, snap_search_radius_cells_,
        require_ground_support_, strict_direct_ground_support_,
        ground_support_xy_radius_cells_, ground_support_depth_cells_, goal))
  {
    error_msg = "Goal is occupied/out of map and no nearby free cell found.";
    return false;
  }

  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
  std::unordered_map<GridIndex, double, GridIndexHash> g_score;
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
  std::unordered_set<GridIndex, GridIndexHash> closed_set;

  g_score[start] = 0.0;
  open_set.push(QueueNode{start, euclidean(start, goal), 0.0});
  const std::vector<GridIndex> directions = makeDirections();
  int iters = 0;

  while (!open_set.empty() && iters < max_iterations_) {
    const QueueNode current = open_set.top();
    open_set.pop();
    ++iters;
    if (closed_set.find(current.idx) != closed_set.end()) continue;
    closed_set.insert(current.idx);

    if (current.idx == goal) {
      path_cells = reconstructPath(came_from, current.idx);
      return true;
    }

    for (const auto & d : directions) {
      GridIndex nbr{current.idx.x + d.x, current.idx.y + d.y, current.idx.z + d.z};
      if (closed_set.find(nbr) != closed_set.end()) continue;
      if (!isCellTraversable(nbr, robot_radius_, require_ground_support_,
            strict_direct_ground_support_, ground_support_xy_radius_cells_,
            ground_support_depth_cells_))
        continue;

      double tentative_g = current.g + euclidean(current.idx, nbr);
      if (enable_preblocked_costmap_)
        tentative_g += preblocked_costmap_weight_ * getPreblockedCost(nbr);

      auto g_it = g_score.find(nbr);
      if (g_it == g_score.end() || tentative_g < g_it->second) {
        came_from[nbr] = current.idx;
        g_score[nbr] = tentative_g;
        open_set.push(QueueNode{nbr, tentative_g + euclidean(nbr, goal), tentative_g});
      }
    }
  }

  error_msg = "A* planning failed or timed out (iterations limit reached).";
  return false;
}

} // namespace octo_planner
