#include "octo_planner/forbidden_zone_field.h"
#include <algorithm>
#include <cmath>
#include <queue>

namespace octo_planner
{

ForbiddenZoneField::ForbiddenZoneField()
{
}

void ForbiddenZoneField::updateField(const OctoPlannerCore& core, double center_x, double center_y)
{
  const double res = params_.resolution;
  const double half_w = params_.local_window_radius;

  origin_x_ = center_x - half_w;
  origin_y_ = center_y - half_w;
  grid_width_ = static_cast<int>(std::ceil((2.0 * half_w) / res)) + 1;
  grid_height_ = static_cast<int>(std::ceil((2.0 * half_w) / res)) + 1;

  const int total_cells = grid_width_ * grid_height_;
  cost_grid_.assign(total_cells, 0.0f);
  height_grid_.assign(total_cells, 0.0f);
  traversable_mask_.assign(total_cells, false);
  pullback_dir_x_.assign(total_cells, 0.0f);
  pullback_dir_y_.assign(total_cells, 0.0f);

  const auto& preblocked = core.getPreblockedCells();
  const auto& traversable = core.getTraversableCells();

  // 1. First pass: Project 3D traversable surface and preblocked layers onto local 2D grid
  for (const auto& t_cell : traversable)
  {
    octomap::point3d pt = core.gridToWorld(t_cell);
    int gx = static_cast<int>(std::round((pt.x() - origin_x_) / res));
    int gy = static_cast<int>(std::round((pt.y() - origin_y_) / res));

    if (isInsideGrid(gx, gy))
    {
      int idx = toIndex(gx, gy);
      traversable_mask_[idx] = true;
      height_grid_[idx] = pt.z();

      // Query core's preblocked cost field
      double cst = core.getPreblockedCost(t_cell);
      if (preblocked.find(t_cell) != preblocked.end())
      {
        cst = std::max(cst, 1.0);
      }
      cost_grid_[idx] = static_cast<float>(std::max(static_cast<double>(cost_grid_[idx]), cst));
    }
  }

  // Also stamp preblocked cells within local bounding window
  for (const auto& p_cell : preblocked)
  {
    octomap::point3d pt = core.gridToWorld(p_cell);
    int gx = static_cast<int>(std::round((pt.x() - origin_x_) / res));
    int gy = static_cast<int>(std::round((pt.y() - origin_y_) / res));

    if (isInsideGrid(gx, gy))
    {
      int idx = toIndex(gx, gy);
      cost_grid_[idx] = 1.0f;
    }
  }

  // 2. Second pass: Breadth-First Search (BFS) distance transform for Exterior Void (Defect 4)
  // Compute distance to nearest safe traversable cell and pull-back vector
  std::vector<int> dist_cells(total_cells, std::numeric_limits<int>::max());
  std::vector<int> nearest_safe(total_cells, -1);
  std::queue<int> q;

  for (int idx = 0; idx < total_cells; ++idx)
  {
    if (traversable_mask_[idx] && cost_grid_[idx] < 0.8f)
    {
      dist_cells[idx] = 0;
      nearest_safe[idx] = idx;
      q.push(idx);
    }
  }

  const int dxs[4] = {1, -1, 0, 0};
  const int dys[4] = {0, 0, 1, -1};

  while (!q.empty())
  {
    int curr = q.front();
    q.pop();
    int cx = curr % grid_width_;
    int cy = curr / grid_width_;
    int cur_d = dist_cells[curr];

    for (int k = 0; k < 4; ++k)
    {
      int nx = cx + dxs[k];
      int ny = cy + dys[k];
      if (isInsideGrid(nx, ny))
      {
        int n_idx = toIndex(nx, ny);
        if (dist_cells[n_idx] > cur_d + 1)
        {
          dist_cells[n_idx] = cur_d + 1;
          nearest_safe[n_idx] = nearest_safe[curr];
          q.push(n_idx);
        }
      }
    }
  }

  // Fill saturated cost and pull-back gradient for non-traversable / exterior cells
  for (int gy = 0; gy < grid_height_; ++gy)
  {
    for (int gx = 0; gx < grid_width_; ++gx)
    {
      int idx = toIndex(gx, gy);
      if (!traversable_mask_[idx])
      {
        int safe_idx = nearest_safe[idx];
        if (safe_idx >= 0)
        {
          int sx = safe_idx % grid_width_;
          int sy = safe_idx / grid_width_;
          double dx_m = (sx - gx) * res;
          double dy_m = (sy - gy) * res;
          double dist = std::hypot(dx_m, dy_m);

          // Saturated cost plus linear extrapolation
          cost_grid_[idx] = static_cast<float>(params_.exterior_cost_base + params_.exterior_gradient_alpha * dist);
          height_grid_[idx] = height_grid_[safe_idx];

          if (dist > 1e-4)
          {
            // Vector pointing towards safe ground
            pullback_dir_x_[idx] = static_cast<float>(dx_m / dist);
            pullback_dir_y_[idx] = static_cast<float>(dy_m / dist);
          }
        }
        else
        {
          cost_grid_[idx] = static_cast<float>(params_.exterior_cost_base);
        }
      }
    }
  }

  initialized_ = true;
}

bool ForbiddenZoneField::evaluateCostAndGradient(double x, double y, double& cost, Eigen::Vector2d& grad) const
{
  if (!initialized_)
  {
    cost = 0.0;
    grad.setZero();
    return false;
  }

  const double res = params_.resolution;
  const double inv_res = 1.0 / res;
  const double uf = (x - origin_x_) * inv_res;
  const double vf = (y - origin_y_) * inv_res;

  const int i = static_cast<int>(std::floor(uf));
  const int j = static_cast<int>(std::floor(vf));

  if (i < 0 || i >= grid_width_ - 1 || j < 0 || j >= grid_height_ - 1)
  {
    // Out of local window bounds: saturate with pull-back towards window center
    double cx = origin_x_ + 0.5 * grid_width_ * res;
    double cy = origin_y_ + 0.5 * grid_height_ * res;
    double dir_x = cx - x;
    double dir_y = cy - y;
    double norm = std::hypot(dir_x, dir_y);
    cost = params_.exterior_cost_base + 1.0;
    if (norm > 1e-4)
    {
      // Gradient dC/dx points in direction of increasing cost (away from center)
      grad = Eigen::Vector2d(-dir_x / norm, -dir_y / norm);
    }
    else
    {
      grad.setZero();
    }
    return false;
  }

  const double du = uf - static_cast<double>(i);
  const double dv = vf - static_cast<double>(j);

  // 4 neighbouring cell costs
  const double c00 = cost_grid_[toIndex(i,     j)];
  const double c10 = cost_grid_[toIndex(i + 1, j)];
  const double c01 = cost_grid_[toIndex(i,     j + 1)];
  const double c11 = cost_grid_[toIndex(i + 1, j + 1)];

  // Defect 2: Bilinear interpolation
  cost = (1.0 - du) * (1.0 - dv) * c00 +
         du * (1.0 - dv) * c10 +
         (1.0 - du) * dv * c01 +
         du * dv * c11;

  // Defect 2: Exact analytical derivatives
  const double dc_dx = inv_res * ((1.0 - dv) * (c10 - c00) + dv * (c11 - c01));
  const double dc_dy = inv_res * ((1.0 - du) * (c01 - c00) + du * (c11 - c10));

  grad.x() = dc_dx;
  grad.y() = dc_dy;

  // Defect 4: Add explicit pull-back vector if evaluating on exterior void
  int center_cell = toIndex(std::min(grid_width_ - 1, std::max(0, static_cast<int>(std::round(uf)))),
                            std::min(grid_height_ - 1, std::max(0, static_cast<int>(std::round(vf)))));
  if (!traversable_mask_[center_cell])
  {
    double pb_x = pullback_dir_x_[center_cell];
    double pb_y = pullback_dir_y_[center_cell];
    if (std::hypot(pb_x, pb_y) > 1e-4)
    {
      // Gradient dC/dx points away from safe ground (increasing cost direction)
      grad.x() -= params_.exterior_gradient_alpha * pb_x;
      grad.y() -= params_.exterior_gradient_alpha * pb_y;
    }
  }

  return true;
}

bool ForbiddenZoneField::getTerrainHeight(double x, double y, double& z) const
{
  if (!initialized_) return false;

  const double res = params_.resolution;
  const int i = static_cast<int>(std::round((x - origin_x_) / res));
  const int j = static_cast<int>(std::round((y - origin_y_) / res));

  if (!isInsideGrid(i, j)) return false;

  z = height_grid_[toIndex(i, j)];
  return true;
}

} // namespace octo_planner
