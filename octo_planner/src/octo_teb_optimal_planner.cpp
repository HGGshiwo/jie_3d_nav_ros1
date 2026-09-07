#include "octo_planner/octo_teb_optimal_planner.h"
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>

namespace octo_planner
{

OctoTebOptimalPlanner::OctoTebOptimalPlanner()
  : teb_local_planner::TebOptimalPlanner()
{
  obstacles_ = &dummy_obstacles_;
}

OctoTebOptimalPlanner::OctoTebOptimalPlanner(const teb_local_planner::TebConfig& cfg)
  : teb_local_planner::TebOptimalPlanner(cfg, &dummy_obstacles_)
{
}

void OctoTebOptimalPlanner::initialize(const teb_local_planner::TebConfig& cfg)
{
  teb_local_planner::TebOptimalPlanner::initialize(cfg, &dummy_obstacles_);
}

bool OctoTebOptimalPlanner::optimizeTEBWithForbiddenZone(int iterations_innerloop,
                                                        int iterations_outerloop,
                                                        const ForbiddenZoneField& field,
                                                        const BaseGradientFootprintModel& footprint,
                                                        double weight_forbidden)
{
  if (!cfg_->optim.optimization_activate)
    return false;

  auto t_start_opt = std::chrono::steady_clock::now();
  double t_build_ms = 0.0;
  double t_solve_ms = 0.0;

  bool success = false;
  optimized_ = false;
  double weight_multiplier = 1.0;
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;

  // Method 2: Ensure TEB native point obstacle edge weight is 0
  const_cast<teb_local_planner::TebConfig*>(cfg_)->optim.weight_obstacle = 0.0;

  const auto& disks = footprint.getDisks();

  for (int iter = 0; iter < iterations_outerloop; ++iter)
  {
    if (cfg_->trajectory.teb_autosize)
    {
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis,
                      cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);
    }

    auto t0 = std::chrono::steady_clock::now();
    // 1. Build all native kinematic, velocity, acceleration, and time-optimality edges
    success = buildGraph(weight_multiplier);
    if (!success)
    {
      clearGraph();
      return false;
    }

    // 2. Inject EdgeForbiddenZoneDisk on every unfixed trajectory pose vertex
    Eigen::Matrix<double, 1, 1> information;
    for (int i = 1; i < teb_.sizePoses() - 1; ++i)
    {
      for (const auto& disk : disks)
      {
        information.fill(weight_forbidden * disk.weight * weight_multiplier);
        EdgeForbiddenZoneDisk* edge = new EdgeForbiddenZoneDisk(disk);
        edge->setVertex(0, teb_.PoseVertex(i));
        edge->setInformation(information);
        edge->setParameters(*cfg_, &field);
        optimizer_->addEdge(edge);
      }
    }

    // Synchronize G2O hyper-graph active edges and Hessian block indexes
    optimizer_->initializeOptimization();
    auto t1 = std::chrono::steady_clock::now();
    t_build_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();

    // 3. Run G2O Levenberg-Marquardt solver
    auto t2 = std::chrono::steady_clock::now();
    success = optimizeGraph(iterations_innerloop, false);
    auto t3 = std::chrono::steady_clock::now();
    t_solve_ms += std::chrono::duration<double, std::milli>(t3 - t2).count();

    if (!success)
    {
      clearGraph();
      return false;
    }
    optimized_ = true;

    clearGraph();
    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  double total_teb_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start_opt).count();
  ROS_INFO_THROTTLE(1.0, "[TEB-Detail] Total: %5.1f ms (Build: %4.1f ms, Solve: %4.1f ms) | Mode: %s | Poses: %zu",
                    total_teb_ms, t_build_ms, t_solve_ms, (is_warm_start_ ? "Warm" : "Cold"), teb_.sizePoses());

  return true;
}

bool OctoTebOptimalPlanner::planWithForbiddenZone(const std::vector<geometry_msgs::PoseStamped>& initial_plan,
                                                 const geometry_msgs::Twist* start_vel,
                                                 bool free_goal_vel,
                                                 const ForbiddenZoneField& field,
                                                 const BaseGradientFootprintModel& footprint,
                                                 double weight_forbidden)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (initial_plan.empty())
  {
    ROS_WARN("OctoTebOptimalPlanner: initial_plan is empty.");
    return false;
  }

  if (!teb_.isInit())
  {
    teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta,
                              cfg_->trajectory.global_plan_overwrite_orientation,
                              cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    is_warm_start_ = false;
  }
  else
  {
    teb_local_planner::PoseSE2 start_(initial_plan.front().pose);
    teb_local_planner::PoseSE2 goal_(initial_plan.back().pose);
    if (teb_.sizePoses() > 0 &&
        (goal_.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist &&
        std::abs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular)
    {
      teb_.updateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples);
      is_warm_start_ = true;
    }
    else
    {
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta,
                                cfg_->trajectory.global_plan_overwrite_orientation,
                                cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
      is_warm_start_ = false;
    }
  }

  if (start_vel)
    setVelocityStart(*start_vel);

  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true;

  return optimizeTEBWithForbiddenZone(cfg_->optim.no_inner_iterations,
                                      cfg_->optim.no_outer_iterations,
                                      field,
                                      footprint,
                                      weight_forbidden);
}

void OctoTebOptimalPlanner::getOptimizedTrajectory3D(const ForbiddenZoneField& field,
                                                    std::vector<geometry_msgs::PoseStamped>& path_3d,
                                                    double base_z_offset) const
{
  path_3d.clear();
  path_3d.reserve(teb_.sizePoses());

  for (int i = 0; i < teb_.sizePoses(); ++i)
  {
    const auto& pose_se2 = teb_.Pose(i);
    geometry_msgs::PoseStamped p;
    p.header.frame_id = cfg_->map_frame;
    p.header.stamp = ros::Time::now();
    p.pose.position.x = pose_se2.x();
    p.pose.position.y = pose_se2.y();

    double z = 0.0;
    if (field.getTerrainHeight(pose_se2.x(), pose_se2.y(), z))
    {
      p.pose.position.z = z + base_z_offset;
    }
    else if (i > 0)
    {
      p.pose.position.z = path_3d.back().pose.position.z;
    }
    else
    {
      p.pose.position.z = base_z_offset;
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose_se2.theta());
    p.pose.orientation = tf2::toMsg(q);

    path_3d.push_back(p);
  }
}

} // namespace octo_planner
