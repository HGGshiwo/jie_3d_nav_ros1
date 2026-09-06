#ifndef OCTO_PLANNER_EDGE_FORBIDDEN_ZONE_H_
#define OCTO_PLANNER_EDGE_FORBIDDEN_ZONE_H_

#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/teb_config.h>
#include "octo_planner/forbidden_zone_field.h"
#include "octo_planner/gradient_footprint_model.h"

namespace octo_planner
{

/**
 * @class EdgeForbiddenZoneDisk
 * @brief G2O Unary Edge enforcing continuous forbidden zone and cliff potential field avoidance
 * for an individual footprint control disk of the robot body.
 *
 * Implements:
 * - Defect 1: 1st-order residual e = C, yielding standard quadratic penalty in g2o: 0.5 * Omega * C^2.
 * - Defect 2: Analytical Jacobian computed from bilinear gradient field.
 * - Defect 3: Non-zero rotational torque Jacobian for theta via rigid-body chain rule.
 */
class EdgeForbiddenZoneDisk : public teb_local_planner::BaseTebUnaryEdge<1, const ForbiddenZoneField*, teb_local_planner::VertexPose>
{
public:
  EdgeForbiddenZoneDisk();
  explicit EdgeForbiddenZoneDisk(const FootprintControlDisk& disk);

  void setDisk(const FootprintControlDisk& disk) { disk_ = disk; }
  const FootprintControlDisk& getDisk() const { return disk_; }

  void setParameters(const teb_local_planner::TebConfig& cfg, const ForbiddenZoneField* field)
  {
    setTebConfig(cfg);
    _measurement = field;
  }

  /**
   * @brief Compute the scalar cost error for this control disk.
   */
  void computeError() override;

  /**
   * @brief Analytical Jacobian matrix of error with respect to robot state (x, y, theta).
   */
  void linearizeOplus() override;

private:
  FootprintControlDisk disk_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace octo_planner

#endif // OCTO_PLANNER_EDGE_FORBIDDEN_ZONE_H_
