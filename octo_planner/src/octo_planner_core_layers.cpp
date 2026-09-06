#include "octo_planner/octo_planner_core.h"
#include <cmath>
#include <algorithm>
#include <vector>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace octo_planner
{

void OctoPlannerCore::rebuildAllLayers()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  rebuildPreblockedCells();
  rebuildDerivedLayers();
  rebuildPreblockedCostmap();
}

void OctoPlannerCore::rebuildPreblockedCells()
{
  preblocked_cells_.clear();
  cliff_cells_.clear();
  if (!octree_) return;

  updateStepHeightCells();
  const double res = octree_->getResolution();
  const int max_step = std::max(1, max_step_height_cells_);
  const int robot_r_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_ / res)));

  // 1. 收集所有实心地表顶层栅格（top_surface_cells）和传统障碍物边缘候选集（candidates）
  std::unordered_set<GridIndex, GridIndexHash> candidates;
  std::vector<GridIndex> top_surface_cells;

  for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
    if (!octree_->isNodeOccupied(*it)) continue;

    const double x = it.getX();
    const double y = it.getY();
    const double z = it.getZ();
    const double size = it.getSize();
    const int K = std::max(1, static_cast<int>(std::round(size / res)));

    const GridIndex grid_min = worldToGrid(
      x - size / 2.0 + res / 2.0,
      y - size / 2.0 + res / 2.0,
      z + size / 2.0 - res / 2.0
    );

    for (int dx = 0; dx < K; ++dx) {
      for (int dy = 0; dy < K; ++dy) {
        const GridIndex top_surf{grid_min.x + dx, grid_min.y + dy, grid_min.z};
        const GridIndex top_above{top_surf.x, top_surf.y, top_surf.z + 1};

        // 如果上方体素非占用，则该点为地表表面点
        if (!isOccupiedCell(top_above)) {
          top_surface_cells.push_back(top_surf);
        }

        // 8邻域障碍物候选点
        for (int c_dx = -1; c_dx <= 1; ++c_dx) {
          for (int c_dy = -1; c_dy <= 1; ++c_dy) {
            if (c_dx == 0 && c_dy == 0) continue;
            candidates.insert(GridIndex{top_surf.x + c_dx, top_surf.y + c_dy, top_surf.z});
          }
        }
      }
    }
  }

  // 2. 传统正障碍物边缘检测（保留台阶与墙壁边缘检测）
  std::vector<GridIndex> candidate_vec(candidates.begin(), candidates.end());
#ifdef _OPENMP
  #pragma omp parallel
  {
    std::vector<GridIndex> local_preblocked;
    #pragma omp for schedule(dynamic, 1000)
    for (size_t i = 0; i < candidate_vec.size(); ++i) {
      const auto & c = candidate_vec[i];
      if (!isInsideMetricBounds(c) || isOccupiedCell(c)) continue;
      const GridIndex below0{c.x, c.y, c.z - 1};
      const bool below0_occ = isInsideMetricBounds(below0) && isOccupiedCell(below0);
      if (below0_occ && hasSameLevelNeighborWithOccupiedAbove(c)) {
        local_preblocked.push_back(c);
        continue;
      }
      const GridIndex above1{c.x, c.y, c.z + 1};
      const bool above1_occ = isInsideMetricBounds(above1) && isOccupiedCell(above1);
      if (!hasNonOccupiedNeighborSameLevel(c)) continue;
      if (above1_occ) continue;

      // 检查下方在跨越容差范围内是否有支撑地面，避免将低矮台阶上方的合法空间误判为悬空
      bool has_ground_in_step = false;
      for (int bz = 1; bz <= max_step; ++bz) {
        const GridIndex b_cell{c.x, c.y, c.z - bz};
        if (isInsideMetricBounds(b_cell) && isOccupiedCell(b_cell)) {
          has_ground_in_step = true;
          break;
        }
      }
      if (!has_ground_in_step) local_preblocked.push_back(c);
    }
    #pragma omp critical
    {
      preblocked_cells_.insert(local_preblocked.begin(), local_preblocked.end());
    }
  }
#else
  for (const auto & c : candidate_vec) {
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

    bool has_ground_in_step = false;
    for (int bz = 1; bz <= max_step; ++bz) {
      const GridIndex b_cell{c.x, c.y, c.z - bz};
      if (isInsideMetricBounds(b_cell) && isOccupiedCell(b_cell)) {
        has_ground_in_step = true;
        break;
      }
    }
    if (!has_ground_in_step) preblocked_cells_.insert(c);
  }
#endif

  // 3. 悬崖边缘负障碍检测（精准单点检测，不向内侧连坐膨胀）
  for (const auto & surf : top_surface_cells) {
    bool is_cliff_brink = false;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;

        // A. 双向地表容差检查：在 [-max_step, +max_step] 内是否有连续地表或台阶
        bool has_ground_in_step_range = false;
        for (int dz = -max_step; dz <= max_step; ++dz) {
          GridIndex n_check{surf.x + dx, surf.y + dy, surf.z + dz};
          if (isInsideMetricBounds(n_check) && isOccupiedCell(n_check)) {
            has_ground_in_step_range = true;
            break;
          }
        }
        if (has_ground_in_step_range) {
          // 邻域地表连续（平地、缓坡或允许跨越的台阶），非悬崖方向
          continue;
        }

        // B. 排除上方正障碍物（如墙壁、柱子）：若上方有障碍物占据，属于正障碍物而非负悬崖
        bool is_wall_or_obstacle = false;
        const int clearance_check_top = max_step + std::max(1, robot_clearance_height_cells_);
        for (int dz = max_step + 1; dz <= clearance_check_top; ++dz) {
          GridIndex n_wall{surf.x + dx, surf.y + dy, surf.z + dz};
          if (isInsideMetricBounds(n_wall) && isOccupiedCell(n_wall)) {
            is_wall_or_obstacle = true;
            break;
          }
        }
        if (is_wall_or_obstacle) {
          continue;
        }

        // C. 确认为真正的外侧悬空跌落区域
        is_cliff_brink = true;

        // 仅标记外侧悬空空气点，阻止向悬空深坑探出
        for (int dz = 0; dz <= ground_support_depth_cells_; ++dz) {
          GridIndex void_cell{surf.x + dx, surf.y + dy, surf.z + dz};
          if (isInsideMetricBounds(void_cell) && !isOccupiedCell(void_cell)) {
            preblocked_cells_.insert(void_cell);
            cliff_cells_.insert(void_cell);
          }
        }
      }
    }

    // 若该地表点自身临近悬崖，仅将其正上方的站立行走层标记为禁行，不向内侧任何其他网格连坐膨胀
    if (is_cliff_brink) {
      for (int w_dz = 1; w_dz <= ground_support_depth_cells_; ++w_dz) {
        GridIndex brink_walk{surf.x, surf.y, surf.z + w_dz};
        if (isInsideMetricBounds(brink_walk) && !isOccupiedCell(brink_walk)) {
          preblocked_cells_.insert(brink_walk);
          cliff_cells_.insert(brink_walk);
        }
      }
    }
  }

  // 4. 外部手动标记的禁行栅格
  for (const auto & c : external_preblocked_cells_) {
    if (isInsideMetricBounds(c) && !isOccupiedCell(c)) {
      preblocked_cells_.insert(c);
    }
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
      
      const int K = static_cast<int>(std::round(size / res));
      
      const GridIndex grid_min = worldToGrid(
        x - size / 2.0 + res / 2.0,
        y - size / 2.0 + res / 2.0,
        z + size / 2.0 - res / 2.0
      );
      
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

    std::vector<GridIndex> candidate_vec(candidates_.begin(), candidates_.end());
#ifdef _OPENMP
    #pragma omp parallel
    {
      std::vector<GridIndex> local_traversable;
      #pragma omp for schedule(dynamic, 1000)
      for (size_t i = 0; i < candidate_vec.size(); ++i) {
        if (isCellTraversable(candidate_vec[i], robot_radius_, require_ground_support_,
              strict_direct_ground_support_, ground_support_xy_radius_cells_,
              ground_support_depth_cells_))
        {
          local_traversable.push_back(candidate_vec[i]);
        }
      }
      #pragma omp critical
      {
        traversable_cells_.insert(local_traversable.begin(), local_traversable.end());
      }
    }
#else
    for (const auto & idx : candidate_vec) {
      if (isCellTraversable(idx, robot_radius_, require_ground_support_,
            strict_direct_ground_support_, ground_support_xy_radius_cells_,
            ground_support_depth_cells_))
      {
        traversable_cells_.insert(idx);
      }
    }
#endif
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

} // namespace octo_planner
