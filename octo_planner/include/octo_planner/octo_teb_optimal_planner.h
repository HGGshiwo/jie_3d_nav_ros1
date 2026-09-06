#ifndef OCTO_PLANNER_OCTO_TEB_OPTIMAL_PLANNER_H_
#define OCTO_PLANNER_OCTO_TEB_OPTIMAL_PLANNER_H_

#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/teb_config.h>
#include "octo_planner/forbidden_zone_field.h"
#include "octo_planner/gradient_footprint_model.h"
#include "octo_planner/edge_forbidden_zone.h"

namespace octo_planner
{

/**
 * @class OctoTebOptimalPlanner
 * @brief Subclass of TebOptimalPlanner that extends TEB with continuous forbidden zone gradient optimization.
 * 
 * Implements Method 2: References teb_local_planner as an external package without modifying TEB source code.
 * Disables native point obstacle edges (weight_obstacle=0) and injects EdgeForbiddenZoneDisk edges
 * on every unfixed trajectory pose vertex.
 */
class OctoTebOptimalPlanner : public teb_local_planner::TebOptimalPlanner
{
public:
  OctoTebOptimalPlanner();
  OctoTebOptimalPlanner(const teb_local_planner::TebConfig& cfg);
  virtual ~OctoTebOptimalPlanner() = default;

  /**
   * @brief Custom TEB optimization loop replacing native obstacles with continuous forbidden zone field.
   * @param iterations_innerloop Number of inner solver iterations per outer loop.
   * @param iterations_outerloop Number of outer loops (re-sizing + re-weighting).
   * @param field Continuous 2D/3D potential field computed from OctoPlannerCore.
   * @param footprint Model providing body-fixed control disks for arbitrary robot envelopes (OCP).
   * @param weight_forbidden Weight factor for the forbidden zone penalty edge.
   * @return true if optimization succeeded.
   */
  bool optimizeTEBWithForbiddenZone(int iterations_innerloop,
                                    int iterations_outerloop,
                                    const ForbiddenZoneField& field,
                                    const BaseGradientFootprintModel& footprint,
                                    double weight_forbidden);

  /**
   * @brief Plan a trajectory from an initial plan and optimize using the forbidden zone field.
   */
  bool planWithForbiddenZone(const std::vector<geometry_msgs::PoseStamped>& initial_plan,
                             const geometry_msgs::Twist* start_vel,
                             bool free_goal_vel,
                             const ForbiddenZoneField& field,
                             const BaseGradientFootprintModel& footprint,
                             double weight_forbidden);

  /**
   * @brief Initialize TEB optimal planner with empty obstacle container to prevent null-deref in AddTEBVertices.
   */
  void initialize(const teb_local_planner::TebConfig& cfg);

  /**
   * @brief Retrieve the optimized trajectory with 3D terrain height z re-attached.
   * @param field Field providing terrain elevation z(x, y).
   * @param[out] path_3d Resulting 3D trajectory (x, y, z, yaw).
   * @param base_z_offset Height offset between robot base link and terrain ground.
   */
  void getOptimizedTrajectory3D(const ForbiddenZoneField& field,
                                std::vector<geometry_msgs::PoseStamped>& path_3d,
                                double base_z_offset = 0.0) const;

private:
  teb_local_planner::ObstContainer dummy_obstacles_;
};

} // namespace octo_planner

#endif // OCTO_PLANNER_OCTO_TEB_OPTIMAL_PLANNER_H_
