#include "octo_planner/octo_elastic_band.h"
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <limits>
#include <algorithm>

namespace octo_planner
{

OctoElasticBand::OctoElasticBand()
{
}

void OctoElasticBand::resampleBand(std::vector<geometry_msgs::PoseStamped> & band, double min_dist, double max_dist)
{
  if (band.size() < 2) return;

  std::vector<geometry_msgs::PoseStamped> resampled;
  resampled.reserve(band.size());
  resampled.push_back(band.front());

  for (size_t i = 0; i < band.size() - 1; ++i)
  {
    const auto & p1 = resampled.back().pose.position;
    const auto & p2 = band[i + 1].pose.position;
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < min_dist && (i + 1) < band.size() - 1)
    {
      continue; // Skip sticking/redundant node (unless it is goal pose)
    }

    if (dist > max_dist)
    {
      int num_segments = static_cast<int>(std::ceil(dist / 0.10));
      for (int k = 1; k < num_segments; ++k)
      {
        double t = static_cast<double>(k) / num_segments;
        geometry_msgs::PoseStamped interp = band[i + 1];
        interp.pose.position.x = p1.x + t * dx;
        interp.pose.position.y = p1.y + t * dy;
        interp.pose.position.z = p1.z + t * dz;
        resampled.push_back(interp);
      }
    }

    resampled.push_back(band[i + 1]);
  }

  // Update tangent orientation for all poses
  for (size_t i = 0; i < resampled.size(); ++i)
  {
    double yaw = 0.0;
    if (i + 1 < resampled.size())
    {
      double dx = resampled[i + 1].pose.position.x - resampled[i].pose.position.x;
      double dy = resampled[i + 1].pose.position.y - resampled[i].pose.position.y;
      yaw = std::atan2(dy, dx);
    }
    else if (i > 0)
    {
      double dx = resampled[i].pose.position.x - resampled[i - 1].pose.position.x;
      double dy = resampled[i].pose.position.y - resampled[i - 1].pose.position.y;
      yaw = std::atan2(dy, dx);
    }
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    resampled[i].pose.orientation = tf2::toMsg(q);
  }

  band = resampled;
}

bool OctoElasticBand::getObstacleForce(const octomap::point3d & p,
                                      const std::vector<octomap::point3d> & obstacles,
                                      octomap::point3d & f_obs)
{
  f_obs = octomap::point3d(0, 0, 0);
  if (obstacles.empty()) return false;

  bool has_near = false;
  for (const auto & obs : obstacles)
  {
    double dx = p.x() - obs.x();
    if (std::abs(dx) >= params_.safe_distance) continue;
    double dy = p.y() - obs.y();
    if (std::abs(dy) >= params_.safe_distance) continue;
    double dz = p.z() - obs.z();
    if (std::abs(dz) >= params_.safe_distance) continue;

    double d_sq = dx * dx + dy * dy + dz * dz;
    double dist = std::sqrt(d_sq);
    if (dist < params_.safe_distance && dist > 1e-4)
    {
      double d_xy = std::hypot(dx, dy);
      if (d_xy < 1e-4) continue;

      // Smooth quadratic repulsion force term with cap to prevent oscillation in narrow passages
      double term = (1.0 / std::max(0.08, dist)) - (1.0 / params_.safe_distance);
      double f_mag = std::min(2.5, term * term);

      // Continuous 2D horizontal repulsion vector
      f_obs.x() += (dx / d_xy) * f_mag;
      f_obs.y() += (dy / d_xy) * f_mag;
      has_near = true;
    }
  }

  return has_near;
}

void OctoElasticBand::optimize(std::vector<geometry_msgs::PoseStamped> & band,
                               const OctoPlannerCore & planner)
{
  resampleBand(band, 0.05, 0.18);
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

  double margin = params_.safe_distance + 0.20;
  octomap::point3d min_pt(min_x - margin, min_y - margin, min_z - margin);
  octomap::point3d max_pt(max_x + margin, max_y + margin, max_z + margin);

  // 2. Cache ground support for each path node first
  std::vector<octomap::point3d> ground_snapped_positions(band.size());
  std::vector<bool> ground_snapped_valid(band.size(), false);

  for (size_t i = 1; i < band.size() - 1; ++i)
  {
    octomap::point3d P_i(band[i].pose.position.x, band[i].pose.position.y, band[i].pose.position.z);
    GridIndex cell_idx = planner.worldToGrid(P_i.x(), P_i.y(), P_i.z());
    GridIndex snapped;
    int search_radius = std::min(4, params_.snap_search_radius_cells);

    // Reuse OctoPlannerCore's findNearestFreeCell (which checks isCellTraversable on-the-fly)
    bool found_snap = planner.findNearestFreeCell(cell_idx, params_.robot_radius, search_radius,
                                                   params_.require_ground_support, params_.strict_direct_ground_support,
                                                   params_.ground_support_xy_radius_cells, params_.ground_support_depth_cells, snapped);
    if (found_snap)
    {
      ground_snapped_positions[i] = planner.gridToWorld(snapped);
      ground_snapped_valid[i] = true;
    }
  }

  // 3. Cache body-level obstacles (filtering out pure ground voxels below local robot clearance)
  std::vector<octomap::point3d> local_obstacles;
  auto octree = planner.getOctree();
  if (octree)
  {
    for (auto it = octree->begin_leafs_bbx(min_pt, max_pt), end = octree->end_leafs_bbx(); it != end; ++it)
    {
      if (octree->isNodeOccupied(*it))
      {
        octomap::point3d pt = it.getCoordinate();
        bool is_ground_voxel = false;
        double min_d2 = std::numeric_limits<double>::max();
        double local_gz = 0.0;
        bool found_ground_ref = false;

        for (size_t i = 1; i < band.size() - 1; ++i) {
          if (ground_snapped_valid[i]) {
            double dx = pt.x() - band[i].pose.position.x;
            double dy = pt.y() - band[i].pose.position.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < min_d2) {
              min_d2 = d2;
              local_gz = ground_snapped_positions[i].z();
              found_ground_ref = true;
            }
          }
        }

        // Only voxels strictly below floor surface level are ground.
        // Body-level obstacles sit above floor level.
        if (found_ground_ref && pt.z() <= local_gz + 0.08) {
          is_ground_voxel = true;
        }

        if (!is_ground_voxel) {
          local_obstacles.push_back(pt);
        }
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

  // 4. Synchronous Jacobi-style Elastic Band Optimization with Displacement Clamping
  std::vector<octomap::point3d> displacements(band.size(), octomap::point3d(0, 0, 0));
  for (int iter = 0; iter < params_.iterations; ++iter)
  {
    for (size_t i = 1; i < band.size() - 1; ++i)
    {
      octomap::point3d P_i(band[i].pose.position.x, band[i].pose.position.y, band[i].pose.position.z);
      octomap::point3d P_prev(band[i-1].pose.position.x, band[i-1].pose.position.y, band[i-1].pose.position.z);
      octomap::point3d P_next(band[i+1].pose.position.x, band[i+1].pose.position.y, band[i+1].pose.position.z);

      // Smoothness Force (contraction towards neighbors)
      octomap::point3d F_smooth = P_prev + P_next - P_i * 2.0;

      // Obstacle Force (continuous sum from all obstacles in safe range)
      octomap::point3d F_obs(0, 0, 0);
      getObstacleForce(P_i, local_obstacles, F_obs);

      // Ground Force
      octomap::point3d F_ground(0, 0, 0);
      if (ground_snapped_valid[i])
      {
        F_ground = octomap::point3d(0, 0, ground_snapped_positions[i].z() - P_i.z());
      }

      octomap::point3d F_total = F_smooth * params_.w_smooth + 
                                 F_obs * params_.w_obstacle + 
                                 F_ground * params_.w_ground;

      octomap::point3d delta = F_total * params_.learning_rate;

      // Clamp maximum single-step displacement to 0.04m to prevent overshoot/divergence
      double max_step = 0.04;
      double d_len = std::hypot(delta.x(), delta.y());
      if (d_len > max_step) {
        delta.x() = (delta.x() / d_len) * max_step;
        delta.y() = (delta.y() / d_len) * max_step;
      }
      if (std::abs(delta.z()) > max_step) {
        delta.z() = (delta.z() > 0 ? max_step : -max_step);
      }

      displacements[i] = delta;
    }

    // Synchronous update for all intermediate nodes
    for (size_t i = 1; i < band.size() - 1; ++i) {
      band[i].pose.position.x += displacements[i].x();
      band[i].pose.position.y += displacements[i].y();
      band[i].pose.position.z += displacements[i].z();
    }
  }

  double dt = (ros::WallTime::now() - start_time).toSec() * 1000.0;
  ROS_INFO_THROTTLE(2.0, "OctoElasticBand: Optimized local band (size=%zu, body_obs=%zu) with continuous forces in %.3f ms",
                    band.size(), local_obstacles.size(), dt);
}

} // namespace octo_planner
