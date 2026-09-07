#ifndef OCTO_PLANNER_LOCAL_PLANNER_PROFILER_H_
#define OCTO_PLANNER_LOCAL_PLANNER_PROFILER_H_

#include <ros/ros.h>
#include <chrono>

namespace octo_planner
{

struct LocalPlannerProfiler
{
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  TimePoint t_start;
  double lock_ms = 0.0;
  double detour_ms = 0.0;
  double rebuild_ms = 0.0;
  double field_ms = 0.0;
  double teb_ms = 0.0;
  double vis_ms = 0.0;
  double emg_ms = 0.0;

  LocalPlannerProfiler() : t_start(Clock::now()) {}

  static double elapsedMs(const TimePoint& start)
  {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  }

  void logSummary() const
  {
    double total_ms = elapsedMs(t_start);
    ROS_INFO_THROTTLE(1.0,
      "[OctoPerf] Total: %5.1f ms | Lock: %4.1f | Rebuild: %4.1f | Detour(A*): %4.1f | Field: %4.1f | TEB: %5.1f | Vis: %4.1f | Emg: %4.1f",
      total_ms, lock_ms, rebuild_ms, detour_ms, field_ms, teb_ms, vis_ms, emg_ms);
  }
};

} // namespace octo_planner

#endif // OCTO_PLANNER_LOCAL_PLANNER_PROFILER_H_
