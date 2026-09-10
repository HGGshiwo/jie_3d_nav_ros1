#ifndef OCTO_PLANNER_LOCAL_PLANNER_PROFILER_H_
#define OCTO_PLANNER_LOCAL_PLANNER_PROFILER_H_

#include <ros/ros.h>
#include <chrono>
#include <string>
#include <algorithm>

namespace octo_planner
{

enum class PlannerFailReason
{
  NONE = 0,
  NOT_INITIALIZED,
  EMPTY_PLAN,
  TF_LOOKUP_ROBOT,
  TF_TRANSFORM_BASE,
  TF_YAW_ERROR,
  ASTAR_DETOUR_EMPTY,
  TEB_NO_FEASIBLE_VEL
};

inline const char* getPlannerFailReasonStr(PlannerFailReason r)
{
  switch (r)
  {
    case PlannerFailReason::NONE: return "NONE";
    case PlannerFailReason::NOT_INITIALIZED: return "NOT_INITIALIZED";
    case PlannerFailReason::EMPTY_PLAN: return "EMPTY_PLAN";
    case PlannerFailReason::TF_LOOKUP_ROBOT: return "TF_LOOKUP_ROBOT";
    case PlannerFailReason::TF_TRANSFORM_BASE: return "TF_TRANSFORM_BASE";
    case PlannerFailReason::TF_YAW_ERROR: return "TF_YAW_ERROR";
    case PlannerFailReason::ASTAR_DETOUR_EMPTY: return "ASTAR_DETOUR_EMPTY";
    case PlannerFailReason::TEB_NO_FEASIBLE_VEL: return "TEB_NO_FEASIBLE_VEL";
    default: return "UNKNOWN";
  }
}

struct PlannerRateMonitor
{
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  TimePoint window_start = Clock::now();
  double expected_controller_freq = 20.0;

  int total_calls = 0;
  int success_calls = 0;
  int fail_not_init = 0;
  int fail_empty_plan = 0;
  int fail_tf = 0;
  int fail_astar = 0;
  int fail_teb = 0;

  double sum_total_ms = 0.0;
  double max_total_ms = 0.0;
  double sum_lock_ms = 0.0;
  double sum_tf_ms = 0.0;
  double sum_detour_ms = 0.0;
  double sum_field_ms = 0.0;
  double sum_teb_ms = 0.0;
  double sum_vis_ms = 0.0;
  double sum_emg_ms = 0.0;

  void recordCall(PlannerFailReason fail_reason,
                  double total_ms, double lock_ms, double tf_ms,
                  double detour_ms, double field_ms, double teb_ms,
                  double vis_ms, double emg_ms)
  {
    total_calls++;
    if (fail_reason == PlannerFailReason::NONE)
    {
      success_calls++;
    }
    else
    {
      switch (fail_reason)
      {
        case PlannerFailReason::NOT_INITIALIZED: fail_not_init++; break;
        case PlannerFailReason::EMPTY_PLAN:      fail_empty_plan++; break;
        case PlannerFailReason::TF_LOOKUP_ROBOT:
        case PlannerFailReason::TF_TRANSFORM_BASE:
        case PlannerFailReason::TF_YAW_ERROR:    fail_tf++; break;
        case PlannerFailReason::ASTAR_DETOUR_EMPTY: fail_astar++; break;
        case PlannerFailReason::TEB_NO_FEASIBLE_VEL: fail_teb++; break;
        default: break;
      }
    }

    sum_total_ms += total_ms;
    if (total_ms > max_total_ms) max_total_ms = total_ms;
    sum_lock_ms += lock_ms;
    sum_tf_ms += tf_ms;
    sum_detour_ms += detour_ms;
    sum_field_ms += field_ms;
    sum_teb_ms += teb_ms;
    sum_vis_ms += vis_ms;
    sum_emg_ms += emg_ms;

    auto now = Clock::now();
    double window_duration = std::chrono::duration<double>(now - window_start).count();
    if (window_duration >= 1.0)
    {
      double call_rate = total_calls / window_duration;
      double pub_rate = success_calls / window_duration;
      int n = std::max(1, total_calls);
      double avg_total = sum_total_ms / n;
      double avg_lock = sum_lock_ms / n;
      double avg_tf = sum_tf_ms / n;
      double avg_detour = sum_detour_ms / n;
      double avg_field = sum_field_ms / n;
      double avg_teb = sum_teb_ms / n;
      double avg_vis = sum_vis_ms / n;
      double avg_emg = sum_emg_ms / n;

      if (call_rate < 3.0)
      {
        ROS_WARN("[OctoPlanner-Diag] [LOW_CALL_RATE] move_base calling rate is ONLY %.1f Hz (%d calls in %.2fs)! "
                 "Planner avg time is %.2f ms (Max: %.2f ms). "
                 "Root cause is OUTSIDE local planner! (Check move_base 'controller_frequency' param or costmap thread delay)",
                 call_rate, total_calls, window_duration, avg_total, max_total_ms);
      }
      else if (total_calls - success_calls > 0)
      {
        ROS_WARN("[OctoPlanner-Diag] [HIGH_REJECTION] Calls: %.1f Hz (%d) | cmd_vel: %.1f Hz (%d) | "
                 "Fails: TF=%d, A*=%d, TEB=%d, EmptyPlan=%d | AvgTime: %.1f ms (Max: %.1f ms)",
                 call_rate, total_calls, pub_rate, success_calls,
                 fail_tf, fail_astar, fail_teb, fail_empty_plan,
                 avg_total, max_total_ms);
      }
      else
      {
        ROS_INFO("[OctoPlanner-Diag] Rate: move_base=%.1f Hz | cmd_vel=%.1f Hz | Avg: %.1f ms (Max: %.1f ms) "
                 "[Lock:%.1f, TF:%.1f, A*:%.1f, Field:%.1f, TEB:%.1f, Vis:%.1f, Emg:%.1f]",
                 call_rate, pub_rate, avg_total, max_total_ms,
                 avg_lock, avg_tf, avg_detour, avg_field, avg_teb, avg_vis, avg_emg);
      }

      // Reset window
      window_start = now;
      total_calls = 0;
      success_calls = 0;
      fail_not_init = 0;
      fail_empty_plan = 0;
      fail_tf = 0;
      fail_astar = 0;
      fail_teb = 0;
      sum_total_ms = 0.0;
      max_total_ms = 0.0;
      sum_lock_ms = 0.0;
      sum_tf_ms = 0.0;
      sum_detour_ms = 0.0;
      sum_field_ms = 0.0;
      sum_teb_ms = 0.0;
      sum_vis_ms = 0.0;
      sum_emg_ms = 0.0;
    }
  }
};

struct LocalPlannerProfiler
{
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  TimePoint t_start;
  double lock_ms = 0.0;
  double tf_ms = 0.0;
  double detour_ms = 0.0;
  double rebuild_ms = 0.0;
  double field_ms = 0.0;
  double teb_ms = 0.0;
  double vis_ms = 0.0;
  double emg_ms = 0.0;
  PlannerFailReason fail_reason = PlannerFailReason::NONE;
  PlannerRateMonitor* monitor = nullptr;
  bool recorded = false;

  explicit LocalPlannerProfiler(PlannerRateMonitor* m = nullptr)
  : t_start(Clock::now()), monitor(m)
  {
  }

  ~LocalPlannerProfiler()
  {
    record();
  }

  static double elapsedMs(const TimePoint& start)
  {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  }

  void record()
  {
    if (recorded) return;
    recorded = true;
    double total_ms = elapsedMs(t_start);
    if (monitor)
    {
      monitor->recordCall(fail_reason, total_ms, lock_ms, tf_ms, detour_ms, field_ms, teb_ms, vis_ms, emg_ms);
    }
  }

  void logSummary() const
  {
    double total_ms = elapsedMs(t_start);
    ROS_INFO_THROTTLE(1.0,
      "[OctoPerf] Total: %5.1f ms | Lock: %4.1f | TF: %4.1f | Detour(A*): %4.1f | Field: %4.1f | TEB: %5.1f | Vis: %4.1f | Emg: %4.1f",
      total_ms, lock_ms, tf_ms, detour_ms, field_ms, teb_ms, vis_ms, emg_ms);
  }
};

} // namespace octo_planner

#endif // OCTO_PLANNER_LOCAL_PLANNER_PROFILER_H_
