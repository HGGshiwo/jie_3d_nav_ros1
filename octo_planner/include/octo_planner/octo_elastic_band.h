#ifndef OCTO_ELASTIC_BAND_H
#define OCTO_ELASTIC_BAND_H

#include <vector>
#include <memory>
#include <geometry_msgs/PoseStamped.h>
#include <octomap/OcTree.h>

#include "octo_planner/octo_planner_core.h"

namespace octo_planner
{

struct ElasticBandParams
{
  int iterations = 40;
  double w_smooth = 1.0;
  double w_obstacle = 0.8;
  double w_tangent = 0.0; // Sideways tangential bypass force weight (disabled to prevent sign chattering)
  double w_ground = 0.4;
  double safe_distance = 0.35; // Safe obstacle clearance (narrow passage friendly)
  double learning_rate = 0.03; // Stable learning rate to prevent overshoot
  double robot_radius = 0.20;
  bool require_ground_support = true;
  bool strict_direct_ground_support = false;
  int ground_support_xy_radius_cells = 1;
  int ground_support_depth_cells = 2;
  int snap_search_radius_cells = 8;
};

class OctoElasticBand
{
public:
  OctoElasticBand();
  ~OctoElasticBand() = default;

  void setParams(const ElasticBandParams & params) { params_ = params; }
  ElasticBandParams getParams() const { return params_; }

  // Optimize local elastic band path
  void optimize(std::vector<geometry_msgs::PoseStamped> & band,
                const OctoPlannerCore & planner);

private:
  bool getObstacleForce(const octomap::point3d & p,
                        const std::vector<octomap::point3d> & obstacles,
                        octomap::point3d & f_obs);

  ElasticBandParams params_;
};

} // namespace octo_planner

#endif // OCTO_ELASTIC_BAND_H
