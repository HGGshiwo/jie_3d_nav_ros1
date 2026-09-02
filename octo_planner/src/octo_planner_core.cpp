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
  enable_path_shortcut_(true),
  enable_path_smoothing_(true),
  path_interpolation_resolution_(0.05),
  corner_fillet_radius_(0.30),
  enable_continuous_yaw_(true),
  yaw_smoothing_window_(5),
  min_idx_{0, 0, 0},
  max_idx_{0, 0, 0},
  cancel_(false)
{
}

bool OctoPlannerCore::setOctree(const std::shared_ptr<octomap::OcTree>& octree)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
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
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  external_preblocked_cells_ = cells;
}

void OctoPlannerCore::rebuildAllLayers()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
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
  std::lock_guard<std::recursive_mutex> lock(mutex_);
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

bool OctoPlannerCore::isDiagonalTransitionValid(const GridIndex & from, const GridIndex & to) const
{
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  const int dz = to.z - from.z;

  // 1. Horizontal diagonal movement check: prevent cutting around wall corners or squeezing through corners
  if (dx != 0 && dy != 0) {
    const GridIndex cell_x1{from.x + dx, from.y, from.z};
    const GridIndex cell_y1{from.x, from.y + dy, from.z};
    if (isOccupiedCell(cell_x1) || isOccupiedCell(cell_y1)) {
      return false;
    }

    if (dz != 0) {
      const GridIndex cell_x2{from.x + dx, from.y, to.z};
      const GridIndex cell_y2{from.x, from.y + dy, to.z};
      if (isOccupiedCell(cell_x2) || isOccupiedCell(cell_y2)) {
        return false;
      }
    }
  }

  // 2. Vertical step transition check: ensure clearance between height levels
  if (dz != 0) {
    const int z_min = std::min(from.z, to.z);
    const int z_max = std::max(from.z, to.z);
    for (int z = z_min; z <= z_max; ++z) {
      if (isOccupiedCell(GridIndex{from.x, from.y, z}) || isOccupiedCell(GridIndex{to.x, to.y, z})) {
        return false;
      }
    }
  }

  return true;
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
  candidates_.clear();
  if (!octree_) return;

  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);
  const GridIndex min_idx = worldToGrid(min_x, min_y, min_z);
  const GridIndex max_idx = worldToGrid(max_x, max_y, max_z);

  if (require_ground_support_) {
    const int xy_r = ground_support_xy_radius_cells_;
    const int depth = ground_support_depth_cells_;
    const double res = octree_->getResolution();

    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) continue;
      
      const double x = it.getX();
      const double y = it.getY();
      const double z = it.getZ();
      const double size = it.getSize();
      
      // Calculate how many minimal grid units this leaf spans (K)
      const int K = static_cast<int>(std::round(size / res));
      
      // Locate the bottom-left grid index on the top surface of this leaf
      const GridIndex grid_min = worldToGrid(
        x - size / 2.0 + res / 2.0,
        y - size / 2.0 + res / 2.0,
        z + size / 2.0 - res / 2.0
      );
      
      // Sweep the K x K grid cells on the top surface
      for (int dx = 0; dx < K; ++dx) {
        for (int dy = 0; dy < K; ++dy) {
          const GridIndex top_surface_cell{grid_min.x + dx, grid_min.y + dy, grid_min.z};
          
          for (int dz = 1; dz <= std::max(1, depth); ++dz) {
            for (int g_dx = -xy_r; g_dx <= xy_r; ++g_dx) {
              for (int g_dy = -xy_r; g_dy <= xy_r; ++g_dy) {
                const GridIndex candidate{top_surface_cell.x + g_dx, top_surface_cell.y + g_dy, top_surface_cell.z + dz};
                if (isInsideMetricBounds(candidate) && !isOccupiedCell(candidate)) {
                  candidates_.insert(candidate);
                }
              }
            }
          }
        }
      }
    }

    for (const auto & idx : candidates_) {
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
  std::lock_guard<std::recursive_mutex> lock(mutex_);
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
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (traversable_cells_.find(seed) != traversable_cells_.end()) {
    out = seed; return true;
  }
  for (int r = 1; r <= radius_cells; ++r)
    for (int dz = 0; dz <= r; ++dz)
      for (int dx = -r; dx <= r; ++dx)
        for (int dy = -r; dy <= r; ++dy) {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) continue;
          GridIndex c1{seed.x + dx, seed.y + dy, seed.z + dz};
          if (traversable_cells_.find(c1) != traversable_cells_.end()) {
            out = c1; return true;
          }
          if (dz > 0) {
            GridIndex c2{seed.x + dx, seed.y + dy, seed.z - dz};
            if (traversable_cells_.find(c2) != traversable_cells_.end()) {
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
  std::lock_guard<std::recursive_mutex> lock(mutex_);
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
    if (cancel_) {
      error_msg = "Planning cancelled by a new request.";
      return false;
    }
    const QueueNode current = open_set.top();
    open_set.pop();
    ++iters;
    if (closed_set.find(current.idx) != closed_set.end()) continue;
    closed_set.insert(current.idx);

    if (current.idx == goal) {
      path_cells = reconstructPath(came_from, current.idx);
      if (enable_path_shortcut_) {
        path_cells = shortcutPath(path_cells);
      }
      return true;
    }

    for (const auto & d : directions) {
      GridIndex nbr{current.idx.x + d.x, current.idx.y + d.y, current.idx.z + d.z};
      if (closed_set.find(nbr) != closed_set.end()) continue;
      if (traversable_cells_.find(nbr) == traversable_cells_.end())
        continue;
      if (!isDiagonalTransitionValid(current.idx, nbr))
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

std::vector<GridIndex> OctoPlannerCore::shortcutPath(const std::vector<GridIndex> & raw_path) const
{
  if (raw_path.size() <= 2) {
    return raw_path;
  }

  // 1. Forward greedy shortcut pass
  std::vector<GridIndex> fwd_path;
  fwd_path.reserve(raw_path.size());
  fwd_path.push_back(raw_path.front());

  size_t curr = 0;
  while (curr < raw_path.size() - 1) {
    size_t furthest = curr + 1;
    for (size_t next = raw_path.size() - 1; next > curr + 1; --next) {
      if (isLineTraversable(raw_path[curr], raw_path[next])) {
        furthest = next;
        break;
      }
    }
    fwd_path.push_back(raw_path[furthest]);
    curr = furthest;
  }

  // 2. Backward greedy shortcut pass
  std::vector<GridIndex> bwd_path;
  bwd_path.reserve(raw_path.size());
  bwd_path.push_back(raw_path.back());

  int bwd_curr = static_cast<int>(raw_path.size()) - 1;
  while (bwd_curr > 0) {
    int furthest = bwd_curr - 1;
    for (int next = 0; next < bwd_curr - 1; ++next) {
      if (isLineTraversable(raw_path[bwd_curr], raw_path[next])) {
        furthest = next;
        break;
      }
    }
    bwd_path.push_back(raw_path[furthest]);
    bwd_curr = furthest;
  }
  std::reverse(bwd_path.begin(), bwd_path.end());

  // 3. Compute total metric length and pick the shorter one
  const auto calcLength = [this](const std::vector<GridIndex> & p) {
    double len = 0.0;
    for (size_t i = 0; i + 1 < p.size(); ++i) {
      const auto p1 = gridToWorld(p[i]);
      const auto p2 = gridToWorld(p[i + 1]);
      len += std::sqrt(
        (p2.x() - p1.x()) * (p2.x() - p1.x()) +
        (p2.y() - p1.y()) * (p2.y() - p1.y()) +
        (p2.z() - p1.z()) * (p2.z() - p1.z()));
    }
    return len;
  };

  return (calcLength(bwd_path) < calcLength(fwd_path)) ? bwd_path : fwd_path;
}

std::vector<geometry_msgs::PoseStamped> OctoPlannerCore::generateSmoothPath(
  const std::vector<GridIndex> & cells,
  const geometry_msgs::PoseStamped & start_pose,
  const geometry_msgs::PoseStamped & goal_pose,
  bool has_goal_pose) const
{
  std::vector<geometry_msgs::PoseStamped> plan;
  if (cells.empty()) return plan;

  ros::Time plan_time = ros::Time::now();
  const std::string frame_id = start_pose.header.frame_id.empty() ? "map" : start_pose.header.frame_id;

  if (cells.size() == 1 || !enable_path_smoothing_) {
    for (size_t i = 0; i < cells.size(); ++i) {
      const auto p = gridToWorld(cells[i]);
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = frame_id;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation.w = 1.0;
      if (i == 0) pose.pose.orientation = start_pose.pose.orientation;
      else if (i + 1 == cells.size() && has_goal_pose) pose.pose.orientation = goal_pose.pose.orientation;
      plan.push_back(pose);
    }
    return plan;
  }

  // Convert cells to metric 3D waypoints
  std::vector<octomap::point3d> waypoints;
  waypoints.reserve(cells.size());
  for (const auto & c : cells) {
    waypoints.push_back(gridToWorld(c));
  }

  // Generate piecewise curve segments with Bézier corner fillets
  std::vector<octomap::point3d> raw_curve_points;
  const size_t K = waypoints.size();

  if (K == 2) {
    const auto & p0 = waypoints[0];
    const auto & p1 = waypoints[1];
    const double seg_len = std::sqrt(
      (p1.x() - p0.x()) * (p1.x() - p0.x()) +
      (p1.y() - p0.y()) * (p1.y() - p0.y()) +
      (p1.z() - p0.z()) * (p1.z() - p0.z()));
    const double step_res = std::max(0.01, path_interpolation_resolution_);
    const int num_samples = std::max(1, static_cast<int>(std::ceil(seg_len / step_res)));
    for (int s = 0; s <= num_samples; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(num_samples);
      raw_curve_points.push_back(octomap::point3d(
        p0.x() + t * (p1.x() - p0.x()),
        p0.y() + t * (p1.y() - p0.y()),
        p0.z() + t * (p1.z() - p0.z())));
    }
  } else {
    // 1. Calculate segment directions and lengths
    std::vector<octomap::point3d> dir(K - 1);
    std::vector<double> seg_len(K - 1);
    for (size_t i = 0; i < K - 1; ++i) {
      const double dx = waypoints[i + 1].x() - waypoints[i].x();
      const double dy = waypoints[i + 1].y() - waypoints[i].y();
      const double dz = waypoints[i + 1].z() - waypoints[i].z();
      seg_len[i] = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (seg_len[i] > 1e-6) {
        dir[i] = octomap::point3d(dx / seg_len[i], dy / seg_len[i], dz / seg_len[i]);
      } else {
        dir[i] = octomap::point3d(1.0, 0.0, 0.0);
      }
    }

    // 2. Determine fillet control points at internal corners
    std::vector<octomap::point3d> fillet_start(K);
    std::vector<octomap::point3d> fillet_end(K);
    std::vector<bool> has_fillet(K, false);

    for (size_t i = 1; i < K - 1; ++i) {
      const double dot = dir[i - 1].x() * dir[i].x() + dir[i - 1].y() * dir[i].y() + dir[i - 1].z() * dir[i].z();
      if (dot > 0.999) {
        has_fillet[i] = false;
        continue;
      }
      double d = std::min({corner_fillet_radius_, 0.45 * seg_len[i - 1], 0.45 * seg_len[i]});
      if (d < 0.05) {
        has_fillet[i] = false;
        continue;
      }

      octomap::point3d p_start(
        waypoints[i].x() - d * dir[i - 1].x(),
        waypoints[i].y() - d * dir[i - 1].y(),
        waypoints[i].z() - d * dir[i - 1].z());
      octomap::point3d p_end(
        waypoints[i].x() + d * dir[i].x(),
        waypoints[i].y() + d * dir[i].y(),
        waypoints[i].z() + d * dir[i].z());

      octomap::point3d mid(
        0.25 * p_start.x() + 0.5 * waypoints[i].x() + 0.25 * p_end.x(),
        0.25 * p_start.y() + 0.5 * waypoints[i].y() + 0.25 * p_end.y(),
        0.25 * p_start.z() + 0.5 * waypoints[i].z() + 0.25 * p_end.z());
      const GridIndex mid_g = worldToGrid(mid.x(), mid.y(), mid.z());
      if (isInsideMetricBounds(mid_g) && !isOccupiedCell(mid_g) &&
          preblocked_cells_.find(mid_g) == preblocked_cells_.end()) {
        fillet_start[i] = p_start;
        fillet_end[i] = p_end;
        has_fillet[i] = true;
      } else {
        has_fillet[i] = false;
      }
    }

    // 3. Assemble trajectory by sampling straight segments and Bézier arcs
    const double step_res = std::max(0.01, path_interpolation_resolution_);
    raw_curve_points.push_back(waypoints[0]);

    for (size_t i = 0; i < K - 1; ++i) {
      const octomap::point3d seg_start = (i == 0 || !has_fillet[i]) ? waypoints[i] : fillet_end[i];
      const octomap::point3d seg_end = (!has_fillet[i + 1]) ? waypoints[i + 1] : fillet_start[i + 1];

      const double l_dx = seg_end.x() - seg_start.x();
      const double l_dy = seg_end.y() - seg_start.y();
      const double l_dz = seg_end.z() - seg_start.z();
      const double l_dist = std::sqrt(l_dx * l_dx + l_dy * l_dy + l_dz * l_dz);
      const int l_samples = std::max(1, static_cast<int>(std::ceil(l_dist / step_res)));

      for (int s = 1; s <= l_samples; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(l_samples);
        raw_curve_points.push_back(octomap::point3d(
          seg_start.x() + t * l_dx,
          seg_start.y() + t * l_dy,
          seg_start.z() + t * l_dz));
      }

      if (i + 1 < K - 1 && has_fillet[i + 1]) {
        const auto & p_start = fillet_start[i + 1];
        const auto & p_ctrl = waypoints[i + 1];
        const auto & p_end = fillet_end[i + 1];
        const double arc_approx_len = corner_fillet_radius_ * 1.5;
        const int arc_samples = std::max(3, static_cast<int>(std::ceil(arc_approx_len / step_res)));

        for (int s = 1; s <= arc_samples; ++s) {
          const double u = static_cast<double>(s) / static_cast<double>(arc_samples);
          const double u_inv = 1.0 - u;
          const double bx = u_inv * u_inv * p_start.x() + 2.0 * u_inv * u * p_ctrl.x() + u * u * p_end.x();
          const double by = u_inv * u_inv * p_start.y() + 2.0 * u_inv * u * p_ctrl.y() + u * u * p_end.y();
          const double bz = u_inv * u_inv * p_start.z() + 2.0 * u_inv * u * p_ctrl.z() + u * u * p_end.z();
          raw_curve_points.push_back(octomap::point3d(bx, by, bz));
        }
      }
    }
  }

  if (raw_curve_points.empty()) return plan;

  // 4. Equidistant arc-length resampling
  std::vector<octomap::point3d> resampled_points;
  resampled_points.push_back(raw_curve_points.front());
  const double desired_ds = std::max(0.01, path_interpolation_resolution_);
  double accumulated_dist = 0.0;

  for (size_t i = 0; i + 1 < raw_curve_points.size(); ++i) {
    const auto & pA = raw_curve_points[i];
    const auto & pB = raw_curve_points[i + 1];
    const double dAB = std::sqrt(
      (pB.x() - pA.x()) * (pB.x() - pA.x()) +
      (pB.y() - pA.y()) * (pB.y() - pA.y()) +
      (pB.z() - pA.z()) * (pB.z() - pA.z()));
    if (dAB < 1e-6) continue;

    double curr_dist = 0.0;
    while (accumulated_dist + (dAB - curr_dist) >= desired_ds) {
      const double remain = desired_ds - accumulated_dist;
      curr_dist += remain;
      const double t = curr_dist / dAB;
      resampled_points.push_back(octomap::point3d(
        pA.x() + t * (pB.x() - pA.x()),
        pA.y() + t * (pB.y() - pA.y()),
        pA.z() + t * (pB.z() - pA.z())));
      accumulated_dist = 0.0;
    }
    accumulated_dist += (dAB - curr_dist);
  }

  if (std::sqrt(
        (resampled_points.back().x() - raw_curve_points.back().x()) * (resampled_points.back().x() - raw_curve_points.back().x()) +
        (resampled_points.back().y() - raw_curve_points.back().y()) * (resampled_points.back().y() - raw_curve_points.back().y()) +
        (resampled_points.back().z() - raw_curve_points.back().z()) * (resampled_points.back().z() - raw_curve_points.back().z())) > 1e-3) {
    resampled_points.push_back(raw_curve_points.back());
  }

  const size_t N_pts = resampled_points.size();
  if (N_pts == 0) return plan;

  // 5. Compute continuous smooth yaw profile
  std::vector<double> yaws(N_pts, 0.0);
  if (enable_continuous_yaw_) {
    for (size_t i = 0; i < N_pts; ++i) {
      if (i < N_pts - 1) {
        const double dx = resampled_points[i + 1].x() - resampled_points[i].x();
        const double dy = resampled_points[i + 1].y() - resampled_points[i].y();
        if (std::hypot(dx, dy) > 1e-4) {
          yaws[i] = std::atan2(dy, dx);
        } else {
          yaws[i] = (i > 0) ? yaws[i - 1] : 0.0;
        }
      } else {
        yaws[i] = (i > 0) ? yaws[i - 1] : 0.0;
      }
    }

    double goal_yaw = 0.0;
    if (has_goal_pose) {
      const auto & q = goal_pose.pose.orientation;
      goal_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      yaws[N_pts - 1] = goal_yaw;
    }

    for (size_t i = 1; i < N_pts; ++i) {
      double diff = yaws[i] - yaws[i - 1];
      while (diff > M_PI) { yaws[i] -= 2.0 * M_PI; diff = yaws[i] - yaws[i - 1]; }
      while (diff < -M_PI) { yaws[i] += 2.0 * M_PI; diff = yaws[i] - yaws[i - 1]; }
    }

    if (has_goal_pose && N_pts > 1) {
      const size_t blend_count = std::min(static_cast<size_t>(10), N_pts);
      const size_t start_blend = N_pts - blend_count;
      double diff_goal = goal_yaw - yaws[start_blend];
      while (diff_goal > M_PI) diff_goal -= 2.0 * M_PI;
      while (diff_goal < -M_PI) diff_goal += 2.0 * M_PI;

      for (size_t i = start_blend; i < N_pts; ++i) {
        const double frac = static_cast<double>(i - start_blend) / static_cast<double>(blend_count - 1);
        yaws[i] = yaws[start_blend] + frac * diff_goal;
      }
    }

    const int win = std::max(1, yaw_smoothing_window_);
    if (win > 1 && N_pts > static_cast<size_t>(win)) {
      std::vector<double> smooth_yaws = yaws;
      const int half_w = win / 2;
      for (size_t i = 1; i + 1 < N_pts; ++i) {
        double sum = 0.0;
        int count = 0;
        for (int w = -half_w; w <= half_w; ++w) {
          int idx = static_cast<int>(i) + w;
          if (idx >= 0 && idx < static_cast<int>(N_pts)) {
            sum += yaws[idx];
            ++count;
          }
        }
        smooth_yaws[i] = sum / count;
      }
      yaws = smooth_yaws;
    }
  }

  // 6. Build PoseStamped vector
  plan.reserve(N_pts);
  for (size_t i = 0; i < N_pts; ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = resampled_points[i].x();
    pose.pose.position.y = resampled_points[i].y();
    pose.pose.position.z = resampled_points[i].z();

    if (enable_continuous_yaw_) {
      const double half_yaw = yaws[i] * 0.5;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(half_yaw);
      pose.pose.orientation.w = std::cos(half_yaw);
    } else {
      pose.pose.orientation.w = 1.0;
    }

    if (i == 0 && !start_pose.header.frame_id.empty() && start_pose.pose.orientation.w != 0.0) {
      pose.pose.orientation = start_pose.pose.orientation;
    }
    if (i + 1 == N_pts && has_goal_pose) {
      pose.pose.orientation = goal_pose.pose.orientation;
    }

    plan.push_back(pose);
  }

  return plan;
}

bool OctoPlannerCore::queryCellDebugInfo(const GridIndex & idx, CellDebugDetails & details) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!octree_) return false;
  if (!isInsideMetricBounds(idx)) {
    return false;
  }

  const auto p = gridToWorld(idx);
  const octomap::OcTreeNode * node = octree_->search(p);
  details.is_unknown = (node == nullptr);
  details.is_occupied = (node && octree_->isNodeOccupied(node));
  
  if (preblocked_cells_.find(idx) != preblocked_cells_.end()) {
    details.is_preblocked = true;
    details.preblocked_reason = getPreblockedReason(idx);
  } else {
    details.is_preblocked = false;
    details.preblocked_reason = "none";
  }
  
  details.has_ground_support = hasGroundSupport(idx, strict_direct_ground_support_, 
                                                ground_support_xy_radius_cells_, 
                                                ground_support_depth_cells_);
  
  bool has_below_preblocked_failure = false;
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_->getMetricMin(min_x, min_y, min_z);
  const int min_z_idx = static_cast<int>(std::floor(min_z / octree_->getResolution()));

  for (int z = idx.z - 1; z >= min_z_idx; --z) {
    const GridIndex below_idx{idx.x, idx.y, z};
    if (isOccupiedCell(below_idx)) break;
    if (preblocked_cells_.find(below_idx) != preblocked_cells_.end()) {
      has_below_preblocked_failure = true;
      break;
    }
  }
  details.has_below_preblocked_failure = has_below_preblocked_failure;
  
  const octomap::point3d center = gridToWorld(idx);
  const double r = octree_->getResolution();
  const int n = std::max(1, static_cast<int>(std::ceil(robot_radius_ / r)));
  const double radius_sq = robot_radius_ * robot_radius_;

  for (int dx = -n; dx <= n; ++dx) {
    for (int dy = -n; dy <= n; ++dy) {
      for (int dz = robot_clearance_height_cells_; dz <= n; ++dz) {
        const double dist_sq =
          (dx * r) * (dx * r) + (dy * r) * (dy * r) + (dz * r) * (dz * r);
        if (dist_sq > radius_sq) continue;
        const octomap::point3d p(
          center.x() + static_cast<float>(dx * r),
          center.y() + static_cast<float>(dy * r),
          center.z() + static_cast<float>(dz * r));
        const GridIndex nearby_idx = worldToGrid(p.x(), p.y(), p.z());
        
        if (preblocked_cells_.find(nearby_idx) != preblocked_cells_.end()) {
          details.has_horizontal_collision = true;
        }
        
        const octomap::OcTreeNode * node = octree_->search(p);
        if (node && octree_->isNodeOccupied(node)) {
          if (dx == 0 && dy == 0 && dz > 0) {
            details.has_vertical_collision = true;
          } else {
            details.has_horizontal_collision = true;
          }
        }
      }
    }
  }
  
  details.preblocked_cost = getPreblockedCost(idx);
  details.risk_cost = details.preblocked_cost;
  
  details.is_candidate = (candidates_.find(idx) != candidates_.end());
  details.is_traversable = (traversable_cells_.find(idx) != traversable_cells_.end());
  
  return true;
}

std::string OctoPlannerCore::getPreblockedReason(const GridIndex & idx) const
{
  if (external_preblocked_cells_.find(idx) != external_preblocked_cells_.end()) {
    return "manual";
  }
  if (preblocked_cells_.find(idx) == preblocked_cells_.end()) {
    return "none";
  }
  
  const GridIndex below{idx.x, idx.y, idx.z - 1};
  bool below_occ = isInsideMetricBounds(below) && isOccupiedCell(below);
  if (below_occ && hasSameLevelNeighborWithOccupiedAbove(idx)) {
    return "step_or_obstacle_edge";
  }
  
  if (!below_occ) {
    return "cliff_or_suspended";
  }
  
  return "unknown";
}

} // namespace octo_planner
