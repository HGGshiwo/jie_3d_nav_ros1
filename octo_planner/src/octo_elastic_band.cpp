#include "octo_planner/octo_elastic_band.h"
#include <ros/ros.h>
#include <cmath>
#include <limits>
#include <algorithm>

namespace octo_planner
{

OctoElasticBand::OctoElasticBand()
{
}

bool OctoElasticBand::getDistanceAndGradient(const octomap::point3d & p,
                                            const std::vector<octomap::point3d> & obstacles,
                                            double & distance,
                                            octomap::point3d & gradient)
{
  if (obstacles.empty()) return false;

  double min_dist_sq = std::numeric_limits<double>::max();
  double min_dist = std::numeric_limits<double>::max();
  octomap::point3d nearest_obs;
  bool found = false;

  for (const auto & obs : obstacles)
  {
    double dx = p.x() - obs.x();
    if (std::abs(dx) >= min_dist) continue;
    double dy = p.y() - obs.y();
    if (std::abs(dy) >= min_dist) continue;
    double dz = p.z() - obs.z();
    if (std::abs(dz) >= min_dist) continue;

    double dist_sq = dx * dx + dy * dy + dz * dz;
    if (dist_sq < min_dist_sq)
    {
      min_dist_sq = dist_sq;
      min_dist = std::sqrt(min_dist_sq);
      nearest_obs = obs;
      found = true;
    }
  }

  if (found)
  {
    distance = min_dist;
    octomap::point3d diff = p - nearest_obs;
    if (distance > 1e-6)
    {
      gradient = diff * (1.0 / distance);
    }
    else
    {
      gradient = octomap::point3d(0, 0, 1);
    }
    return true;
  }
  return false;
}

void OctoElasticBand::optimize(std::vector<geometry_msgs::PoseStamped> & band,
                               const OctoPlannerCore & planner)
{
  if (band.size() < 3) return;

  ros::WallTime start_time = ros::WallTime::now();

  // 1. Calculate bounding box of the local band
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();
  double max_z = -std::numeric_limits<double>::max();
  for (const auto & pose : band)
  {
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double z = pose.pose.position.z;
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    min_z = std::min(min_z, z);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
    max_z = std::max(max_z, z);
  }

  double margin = params_.safe_distance + 0.05;
  octomap::point3d min_pt(min_x - margin, min_y - margin, min_z - margin);
  octomap::point3d max_pt(max_x + margin, max_y + margin, max_z + margin);

  // 2. Cache local obstacles inside bounding box
  std::vector<octomap::point3d> local_obstacles;
  auto octree = planner.getOctree();
  if (octree)
  {
    for (auto it = octree->begin_leafs_bbx(min_pt, max_pt), end = octree->end_leafs_bbx(); it != end; ++it)
    {
      if (octree->isNodeOccupied(*it))
      {
        local_obstacles.push_back(it.getCoordinate());
      }
    }
  }

  // Preblocked cells within bounding box
  const auto & preblocked = planner.getPreblockedCells();
  for (const auto & c : preblocked)
  {
    octomap::point3d p_world = planner.gridToWorld(c);
    if (p_world.x() >= min_pt.x() && p_world.x() <= max_pt.x() &&
        p_world.y() >= min_pt.y() && p_world.y() <= max_pt.y() &&
        p_world.z() >= min_pt.z() && p_world.z() <= max_pt.z())
    {
      local_obstacles.push_back(p_world);
    }
  }

  // 3. Ground support cache
  std::vector<octomap::point3d> ground_snapped_positions(band.size());
  std::vector<bool> ground_snapped_valid(band.size(), false);

  for (size_t i = 1; i < band.size() - 1; ++i)
  {
    octomap::point3d P_i(band[i].pose.position.x, band[i].pose.position.y, band[i].pose.position.z);
    GridIndex cell_idx = planner.worldToGrid(P_i.x(), P_i.y(), P_i.z());
    GridIndex snapped;
    int search_radius = std::min(4, params_.snap_search_radius_cells);

    bool found_snap = false;
    if (!planner.getTraversableCells().empty())
    {
      found_snap = planner.findNearestFreeCell(cell_idx, params_.robot_radius, search_radius,
                                                params_.require_ground_support, params_.strict_direct_ground_support,
                                                params_.ground_support_xy_radius_cells, params_.ground_support_depth_cells, snapped);
    }
    else
    {
      for (int dz = 0; dz <= search_radius; ++dz)
      {
        GridIndex below{cell_idx.x, cell_idx.y, cell_idx.z - dz};
        if (planner.isOccupiedCell(below))
        {
          snapped = GridIndex{cell_idx.x, cell_idx.y, below.z + 1};
          found_snap = true;
          break;
        }
      }
    }

    if (found_snap)
    {
      ground_snapped_positions[i] = planner.gridToWorld(snapped);
      ground_snapped_valid[i] = true;
    }
  }

  // 4. Elastic band optimization loop
  for (int iter = 0; iter < params_.iterations; ++iter)
  {
    for (size_t i = 1; i < band.size() - 1; ++i)
    {
      octomap::point3d P_i(band[i].pose.position.x, band[i].pose.position.y, band[i].pose.position.z);
      octomap::point3d P_prev(band[i-1].pose.position.x, band[i-1].pose.position.y, band[i-1].pose.position.z);
      octomap::point3d P_next(band[i+1].pose.position.x, band[i+1].pose.position.y, band[i+1].pose.position.z);

      octomap::point3d F_smooth = P_prev + P_next - P_i * 2.0;

      double distance = 0.0;
      octomap::point3d gradient(0, 0, 0);
      octomap::point3d F_obs(0, 0, 0);
      if (getDistanceAndGradient(P_i, local_obstacles, distance, gradient))
      {
        if (distance < params_.safe_distance)
        {
          F_obs = gradient * (params_.safe_distance - distance);
        }
      }

      octomap::point3d F_ground(0, 0, 0);
      if (ground_snapped_valid[i])
      {
        F_ground = ground_snapped_positions[i] - P_i;
      }

      octomap::point3d F_total = F_smooth * params_.w_smooth + F_obs * params_.w_obstacle + F_ground * params_.w_ground;

      P_i = P_i + F_total * params_.learning_rate;

      band[i].pose.position.x = P_i.x();
      band[i].pose.position.y = P_i.y();
      band[i].pose.position.z = P_i.z();
    }
  }

  double dt = (ros::WallTime::now() - start_time).toSec() * 1000.0;
  ROS_INFO_THROTTLE(2.0, "OctoElasticBand: Optimized local band (size=%zu, obs_cached=%zu) in %.3f ms",
                    band.size(), local_obstacles.size(), dt);
}

} // namespace octo_planner
