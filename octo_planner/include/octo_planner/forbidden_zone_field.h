#ifndef OCTO_PLANNER_FORBIDDEN_ZONE_FIELD_H_
#define OCTO_PLANNER_FORBIDDEN_ZONE_FIELD_H_

#include <vector>
#include <memory>
#include <limits>
#include <cmath>
#include <Eigen/Core>
#include "octo_planner/octo_planner_core.h"

namespace octo_planner
{

/**
 * @struct ForbiddenZoneParams
 * @brief Parameters for continuous forbidden zone cost and gradient calculation.
 */
struct ForbiddenZoneParams
{
  double resolution = 0.05;         ///< 2D grid resolution (meters)
  double local_window_radius = 4.0; ///< Half-width of local evaluation field window (meters)
  double exterior_cost_base = 1.0;  ///< Saturated cost for cells outside traversable terrain / cliff
  double exterior_gradient_alpha = 1.5; ///< Pull-back gradient strength for exterior/void cells
  double clearance_margin = 0.10;   ///< Additional safety margin (meters)
};

/**
 * @class ForbiddenZoneField
 * @brief Thread-safe continuous 2D potential field with analytical bilinear gradients
 * extracted from OctoPlannerCore 3D layers.
 * 
 * Specifically addresses:
 * - Defect 2: Bilinear interpolation and exact analytical spatial derivatives.
 * - Defect 4: Saturated penalty and explicit pull-back gradient for points outside traversable domain.
 * - Defect 5: Decoupled 2D manifold gradient avoiding stair step vertical spikes.
 */
class ForbiddenZoneField
{
public:
  ForbiddenZoneField();
  ~ForbiddenZoneField() = default;

  void setParams(const ForbiddenZoneParams& params) { params_ = params; }
  const ForbiddenZoneParams& getParams() const { return params_; }

  /**
   * @brief Rebuild the local dense cost and gradient field from OctoPlannerCore.
   * @param core OctoPlannerCore providing 3D preblocked, cliff, and traversable layers.
   * @param center_x Center of the local window in world coordinates (e.g. robot x).
   * @param center_y Center of the local window in world coordinates (e.g. robot y).
   */
  void updateField(const OctoPlannerCore& core, double center_x, double center_y);

  /**
   * @brief Evaluates continuous cost and analytical gradient at a given world position.
   * @param x World X coordinate (m)
   * @param y World Y coordinate (m)
   * @param[out] cost Continuous cost in [0, inf)
   * @param[out] grad Analytical gradient dC/dx, dC/dy pointing towards increasing cost
   * @return true if evaluation succeeded within valid bounding window
   */
  bool evaluateCostAndGradient(double x, double y, double& cost, Eigen::Vector2d& grad) const;

  /**
   * @brief Get terrain height z at given (x, y) coordinates from the updated field.
   */
  bool getTerrainHeight(double x, double y, double& z) const;

private:
  ForbiddenZoneParams params_;

  // 2D Local Dense Grid bounds
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  int grid_width_ = 0;
  int grid_height_ = 0;

  // Dense 2D buffers (Row-major: idx = y * width + x)
  std::vector<float> cost_grid_;
  std::vector<float> height_grid_;
  std::vector<bool>  traversable_mask_;
  std::vector<float> pullback_dir_x_;
  std::vector<float> pullback_dir_y_;
  bool initialized_ = false;

  inline int toIndex(int gx, int gy) const
  {
    return gy * grid_width_ + gx;
  }

  inline bool isInsideGrid(int gx, int gy) const
  {
    return gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_;
  }
};

} // namespace octo_planner

#endif // OCTO_PLANNER_FORBIDDEN_ZONE_FIELD_H_
