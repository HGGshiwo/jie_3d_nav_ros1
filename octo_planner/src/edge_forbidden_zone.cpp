#include "octo_planner/edge_forbidden_zone.h"
#include <ros/assert.h>
#include <cmath>

namespace octo_planner
{

EdgeForbiddenZoneDisk::EdgeForbiddenZoneDisk()
{
  _measurement = nullptr;
}

EdgeForbiddenZoneDisk::EdgeForbiddenZoneDisk(const FootprintControlDisk& disk)
  : disk_(disk)
{
  _measurement = nullptr;
}

void EdgeForbiddenZoneDisk::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setParameters() on EdgeForbiddenZoneDisk");
  const teb_local_planner::VertexPose* bandpt = static_cast<const teb_local_planner::VertexPose*>(_vertices[0]);

  const double x = bandpt->x();
  const double y = bandpt->y();
  const double theta = bandpt->theta();

  const double cos_th = std::cos(theta);
  const double sin_th = std::sin(theta);

  // Global world position of this control disk
  const double px = x + cos_th * disk_.dx - sin_th * disk_.dy;
  const double py = y + sin_th * disk_.dx + cos_th * disk_.dy;

  double cost = 0.0;
  Eigen::Vector2d grad;
  _measurement->evaluateCostAndGradient(px, py, cost, grad);

  // Defect 1: 1st-order linear cost residual, yielding standard quadratic penalty in g2o:
  // V_obs = 1/2 * Omega * e^2 = 1/2 * Omega * C^2
  _error[0] = (cost > 0.0) ? cost : 0.0;

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeForbiddenZoneDisk::computeError() _error[0]=%f\n", _error[0]);
}

void EdgeForbiddenZoneDisk::linearizeOplus()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setParameters() on EdgeForbiddenZoneDisk");
  const teb_local_planner::VertexPose* bandpt = static_cast<const teb_local_planner::VertexPose*>(_vertices[0]);

  const double x = bandpt->x();
  const double y = bandpt->y();
  const double theta = bandpt->theta();

  const double cos_th = std::cos(theta);
  const double sin_th = std::sin(theta);

  // Global world position of this control disk
  const double px = x + cos_th * disk_.dx - sin_th * disk_.dy;
  const double py = y + sin_th * disk_.dx + cos_th * disk_.dy;

  double cost = 0.0;
  Eigen::Vector2d grad(0.0, 0.0);
  _measurement->evaluateCostAndGradient(px, py, cost, grad);

  if (cost <= 0.0)
  {
    _jacobianOplusXi.setZero();
    return;
  }

  // Defect 2 & Defect 3: Analytical chain rule Jacobian
  // e = C(p(x, y, theta))
  // de/dx = dC/dpx
  // de/dy = dC/dpy
  // de/dtheta = dC/dpx * (-dx*sin_th - dy*cos_th) + dC/dpy * (dx*cos_th - dy*sin_th)
  const double dpx_dth = -disk_.dx * sin_th - disk_.dy * cos_th;
  const double dpy_dth =  disk_.dx * cos_th - disk_.dy * sin_th;

  _jacobianOplusXi(0, 0) = grad.x();
  _jacobianOplusXi(0, 1) = grad.y();
  _jacobianOplusXi(0, 2) = grad.x() * dpx_dth + grad.y() * dpy_dth;
}

} // namespace octo_planner
