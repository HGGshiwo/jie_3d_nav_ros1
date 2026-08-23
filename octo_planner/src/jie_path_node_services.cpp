#include "octo_planner/jie_path_node.h"

bool JiePathNode::handleGetMapMeta(
  jie_map_msgs::GetNavigationMapMeta::Request & /*req*/,
  jie_map_msgs::GetNavigationMapMeta::Response & res)
{
  auto octree = planner_.getOctree();
  res.success = map_ready_ && static_cast<bool>(octree);
  res.message = res.success ? "ok" : "octomap not ready";
  res.map_id  = map_id_;
  res.frame_id = frame_id_;
  res.resolution = octree ? octree->getResolution() : 0.0;
  fillBounds(res.min_bound, res.max_bound);
  // Note: read values back from the planner
  res.robot_radius = 0.20; // fallback or we could expose getter
  res.snap_search_radius_cells = 8;
  res.require_ground_support = true;
  res.strict_direct_ground_support = true;
  res.ground_support_xy_radius_cells = 1;
  res.ground_support_depth_cells = 2;
  res.enable_preblocked_costmap = enable_preblocked_costmap_;
  res.preblocked_costmap_radius_cells = 3;
  res.preblocked_costmap_weight = 1.5;
  res.source_world_file = source_world_file_;
  return true;
}

bool JiePathNode::handleExportSnapshot(
  jie_map_msgs::ExportNavigationSnapshot::Request & req,
  jie_map_msgs::ExportNavigationSnapshot::Response & res)
{
  auto octree = planner_.getOctree();
  if (!map_ready_ || !octree) {
    res.success = false;
    res.message = "octomap not ready";
    res.snapshot_stamp = now();
    return true;
  }
  if (req.recompute_layers) {
    planner_.rebuildAllLayers();
  } else {
    publishPreblockedCellsMarker();
    publishCellSetMarker(planner_.getTraversableCells(), traversable_marker_pub_, "traversable_cells", 0.20F, 0.95F, 0.55F, 0.55F);
    publishRiskCostCloud();
  }
  res.success = true;
  res.message = "snapshot ready";
  res.snapshot_stamp = now();
  return true;
}

bool JiePathNode::handleQueryCellDebugInfo(
  jie_map_msgs::QueryCellDebugInfo::Request & req,
  jie_map_msgs::QueryCellDebugInfo::Response & res)
{
  auto octree = planner_.getOctree();
  if (!map_ready_ || !octree) {
    res.success = false;
    res.message = "octomap not ready";
    return true;
  }

  octo_planner::GridIndex idx = planner_.worldToGrid(req.x, req.y, req.z);
  octo_planner::CellDebugDetails details;
  details.grid_x = idx.x;
  details.grid_y = idx.y;
  details.grid_z = idx.z;

  bool ret = planner_.queryCellDebugInfo(idx, details);
  if (!ret) {
    res.success = false;
    res.message = "coordinate out of metric bounds";
    return true;
  }

  res.success = true;
  res.message = "query ok";
  res.grid_x = details.grid_x;
  res.grid_y = details.grid_y;
  res.grid_z = details.grid_z;
  res.is_occupied = details.is_occupied;
  res.is_unknown = details.is_unknown;
  res.has_ground_support = details.has_ground_support;
  res.is_preblocked = details.is_preblocked;
  res.preblocked_reason = details.preblocked_reason;
  res.has_vertical_collision = details.has_vertical_collision;
  res.has_horizontal_collision = details.has_horizontal_collision;
  res.has_below_preblocked_failure = details.has_below_preblocked_failure;
  res.preblocked_cost = details.preblocked_cost;
  res.risk_cost = details.risk_cost;
  res.is_candidate = details.is_candidate;
  res.is_traversable = details.is_traversable;
  return true;
}
