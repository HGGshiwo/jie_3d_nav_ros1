#include "octo_planner/octo_planner_core.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

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
  max_step_height_m_(0.30),
  max_step_height_cells_(1),
  robot_height_m_(0.60),
  robot_height_cells_(0),
  robot_clearance_height_cells_(0),
  heuristic_weight_(1.20),
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
  updateStepHeightCells();
  return true;
}

void OctoPlannerCore::setExternalPreblockedCells(const std::unordered_set<GridIndex, GridIndexHash>& cells)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  external_preblocked_cells_ = cells;
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
  const int step_limit = std::max(1, max_step_height_cells_);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
      if (!isInsideMetricBounds(n)) continue;
      // 只有当邻域障碍物高于跨越能力（max_step_height_cells_）时，才算作不可逾越的正障碍物/墙体
      for (int dz = step_limit; dz <= step_limit + 4; ++dz) {
        const GridIndex n_above{n.x, n.y, idx.z + dz};
        if (isInsideMetricBounds(n_above) && isOccupiedCell(n_above)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool OctoPlannerCore::isCellTraversable(const GridIndex & idx, double robot_radius,
  bool require_ground_support, bool strict, int xy_r, int depth) const
{
  if (!octree_) return false;
  if (!isInsideMetricBounds(idx)) return false;

  // 1. 若当前体素已被判定为禁行区（包含悬崖边缘禁行区），直接判定不可通行
  if (preblocked_cells_.find(idx) != preblocked_cells_.end()) {
    return false;
  }

  // 2. 地面支撑检查
  if (require_ground_support && !hasGroundSupport(idx, strict, xy_r, depth)) return false;

  // 3. 向下扫描到指定深度，若遇到禁行栅格或悬空禁行则拒绝通行
  const int max_depth = std::max(1, depth);
  const int min_z_check = idx.z - max_depth;

  for (int z = idx.z - 1; z >= min_z_check; --z) {
    const GridIndex below_idx{idx.x, idx.y, z};
    if (isInsideMetricBounds(below_idx)) {
      if (preblocked_cells_.find(below_idx) != preblocked_cells_.end()) return false;
      if (isOccupiedCell(below_idx)) break; // 实体地面，合法退出
    }
  }

  // 4. 车体水平与垂直躯干柱状体防碰撞检测（分层几何模型：避开腿部跨越区，仅在机身躯干层做实体碰撞检测）
  const octomap::point3d center = gridToWorld(idx);
  const double r = octree_->getResolution();
  const int n_xy = std::max(1, static_cast<int>(std::ceil(robot_radius / r)));
  const double radius_sq = robot_radius * robot_radius;
  const int dz_min = std::max(robot_clearance_height_cells_, max_step_height_cells_);
  const int dz_max = std::max(dz_min, robot_height_cells_ > 0 ? robot_height_cells_ : n_xy);

  for (int dx = -n_xy; dx <= n_xy; ++dx) {
    for (int dy = -n_xy; dy <= n_xy; ++dy) {
      const double dist_xy_sq = (dx * r) * (dx * r) + (dy * r) * (dy * r);
      if (dist_xy_sq > radius_sq) continue;

      for (int dz = dz_min; dz <= dz_max; ++dz) {
        const octomap::point3d p(
          center.x() + static_cast<float>(dx * r),
          center.y() + static_cast<float>(dy * r),
          center.z() + static_cast<float>(dz * r));
        const octomap::OcTreeNode * node = octree_->search(p);
        if (node && octree_->isNodeOccupied(node)) return false;
      }
    }
  }
  return true;
}

bool OctoPlannerCore::findNearestFreeCell(const GridIndex & seed, double robot_radius, int radius_cells,
  bool require_ground_support, bool strict, int xy_r, int depth, GridIndex & out,
  bool prefer_downward, double forward_yaw, bool enforce_forward) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  const bool has_heading = !std::isnan(forward_yaw);
  const double cos_yaw = has_heading ? std::cos(forward_yaw) : 1.0;
  const double sin_yaw = has_heading ? std::sin(forward_yaw) : 0.0;

  auto searchLoop = [&](bool only_forward) -> bool {
    // A. Use cached traversability set if populated (for global planner compatibility)
    if (!traversable_cells_.empty())
    {
      if (!only_forward || !has_heading) {
        if (traversable_cells_.find(seed) != traversable_cells_.end()) {
          out = seed; return true;
        }
      }
      for (int r = 1; r <= radius_cells; ++r)
        for (int dz_mag = 0; dz_mag <= r; ++dz_mag)
          for (int dx = -r; dx <= r; ++dx)
            for (int dy = -r; dy <= r; ++dy) {
              if (std::max({std::abs(dx), std::abs(dy), dz_mag}) != r) continue;
              if (only_forward && has_heading) {
                double fwd = dx * cos_yaw + dy * sin_yaw;
                if (fwd < -0.01) continue; // Strictly exclude cells behind robot heading
              }
              const int dz_cands[2] = {prefer_downward ? -dz_mag : dz_mag, prefer_downward ? dz_mag : -dz_mag};
              const int num_dz = (dz_mag == 0) ? 1 : 2;
              for (int k = 0; k < num_dz; ++k) {
                const int dz = dz_cands[k];
                GridIndex c{seed.x + dx, seed.y + dy, seed.z + dz};
                if (traversable_cells_.find(c) != traversable_cells_.end()) {
                  out = c; return true;
                }
              }
            }
      return false;
    }

    // B. Fallback to on-the-fly checks if traversability set is empty (local planner mode)
    if (!only_forward || !has_heading) {
      if (isCellTraversable(seed, robot_radius, require_ground_support, strict, xy_r, depth))
      {
        out = seed; return true;
      }
    }
    for (int r = 1; r <= radius_cells; ++r)
      for (int dz_mag = 0; dz_mag <= r; ++dz_mag)
        for (int dx = -r; dx <= r; ++dx)
          for (int dy = -r; dy <= r; ++dy) {
            if (std::max({std::abs(dx), std::abs(dy), dz_mag}) != r) continue;
            if (only_forward && has_heading) {
              double fwd = dx * cos_yaw + dy * sin_yaw;
              if (fwd < -0.01) continue; // Strictly exclude cells behind robot heading
            }
            const int dz_cands[2] = {prefer_downward ? -dz_mag : dz_mag, prefer_downward ? dz_mag : -dz_mag};
            const int num_dz = (dz_mag == 0) ? 1 : 2;
            for (int k = 0; k < num_dz; ++k) {
              const int dz = dz_cands[k];
              GridIndex c{seed.x + dx, seed.y + dy, seed.z + dz};
              if (isCellTraversable(c, robot_radius, require_ground_support, strict, xy_r, depth)) {
                out = c; return true;
              }
            }
          }

    // Progressive relaxation: if full robot_radius cannot find any cell in local window,
    // gracefully relax radius to find a traversable escape cell
    if (robot_radius > 0.08) {
      if (findNearestFreeCell(seed, robot_radius * 0.5, radius_cells, require_ground_support, strict, xy_r, depth, out,
                              prefer_downward, forward_yaw, only_forward)) {
        return true;
      }
    }

    return false;
  };

  if (enforce_forward && has_heading)
  {
    if (searchLoop(true)) return true;
    return searchLoop(false);
  }

  return searchLoop(false);
}

bool OctoPlannerCore::queryCellDebugInfo(const GridIndex & idx, CellDebugDetails & details) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  details.grid_x = idx.x;
  details.grid_y = idx.y;
  details.grid_z = idx.z;
  
  if (!octree_ || !isInsideMetricBounds(idx)) {
    return false;
  }
  
  const octomap::point3d p = gridToWorld(idx);
  const octomap::OcTreeNode * node = octree_->search(p);
  
  if (node) {
    details.is_occupied = octree_->isNodeOccupied(node);
    details.is_unknown = false;
    double size = octree_->getResolution();
    for (octomap::OcTree::leaf_bbx_iterator it = octree_->begin_leafs_bbx(p, p), end = octree_->end_leafs_bbx(); it != end; ++it) {
      size = it.getSize();
      break;
    }
    const bool occ = octree_->isNodeOccupied(node);
    const float log_odds = node->getLogOdds();
    
    std::ostringstream ss;
    ss << "占据状态=" << (occ ? "占据(Occupied)" : "空闲(Free)")
       << ", 体素尺寸=" << std::fixed << std::setprecision(2) << size << "m"
       << ", LogOdds=" << std::setprecision(3) << log_odds
       << ", 探测点=[" << std::setprecision(3) << p.x() << "," << p.y() << "," << p.z() << "]";
    details.node_source_info = ss.str();
  } else {
    details.node_source_info = "未知/未映射区域 (Unknown/Unmapped Space), 坐标点 [" + 
                               std::to_string(p.x()) + "," + std::to_string(p.y()) + "," + std::to_string(p.z()) + "] 未在八叉树内存中分配节点";
  }
  
  if (preblocked_cells_.find(idx) != preblocked_cells_.end()) {
    details.is_preblocked = true;
    details.preblocked_reason = getPreblockedReason(idx);
  } else {
    details.is_preblocked = false;
    details.preblocked_reason = "none";
  }
  
  details.has_ground_support = hasGroundSupport(idx, strict_direct_ground_support_, ground_support_xy_radius_cells_, ground_support_depth_cells_);
  
  bool has_below_preblocked_failure = false;
  const int max_depth = std::max(1, ground_support_depth_cells_);
  const int min_z_check = idx.z - max_depth;
  for (int z = idx.z - 1; z >= min_z_check; --z) {
    const GridIndex below_idx{idx.x, idx.y, z};
    if (isInsideMetricBounds(below_idx)) {
      if (isOccupiedCell(below_idx)) break;
      if (preblocked_cells_.find(below_idx) != preblocked_cells_.end()) {
        has_below_preblocked_failure = true;
      }
    }
  }
  details.has_below_preblocked_failure = has_below_preblocked_failure;
  
  details.has_vertical_collision = false;
  details.has_horizontal_collision = false;
  const double r = octree_->getResolution();
  const int n_xy = std::max(1, static_cast<int>(std::ceil(robot_radius_ / r)));
  const double radius_sq = robot_radius_ * robot_radius_;
  const int dz_min = std::max(robot_clearance_height_cells_, max_step_height_cells_);
  const int dz_max = std::max(dz_min, robot_height_cells_ > 0 ? robot_height_cells_ : n_xy);

  for (int dx = -n_xy; dx <= n_xy; ++dx) {
    for (int dy = -n_xy; dy <= n_xy; ++dy) {
      const double dist_xy_sq = (dx * r) * (dx * r) + (dy * r) * (dy * r);
      if (dist_xy_sq > radius_sq) continue;

      for (int dz = dz_min; dz <= dz_max; ++dz) {
        const octomap::point3d q(
          p.x() + static_cast<float>(dx * r),
          p.y() + static_cast<float>(dy * r),
          p.z() + static_cast<float>(dz * r));
        const octomap::OcTreeNode * nearby_node = octree_->search(q);
        if (nearby_node && octree_->isNodeOccupied(nearby_node)) {
          if (dx == 0 && dy == 0) {
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
  if (cliff_cells_.find(idx) != cliff_cells_.end()) {
    return "cliff_edge";
  }
  
  const GridIndex below{idx.x, idx.y, idx.z - 1};
  bool below_occ = isInsideMetricBounds(below) && isOccupiedCell(below);
  if (below_occ && hasSameLevelNeighborWithOccupiedAbove(idx)) {
    return "step_or_obstacle_edge";
  }
  
  if (!below_occ) {
    return "cliff_or_suspended";
  }
  
  return "obstacle_edge";
}

} // namespace octo_planner
