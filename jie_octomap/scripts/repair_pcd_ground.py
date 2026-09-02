#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
repair_pcd_ground.py
Ground & Stairs PCD Inpainting & Repair Tool using 3D KD-Tree and Local Surface Fitting.
"""

import os
import sys
import time
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

def repair_point_cloud(
    input_pcd_path: str,
    output_pcd_path: str,
    grid_res: float = 0.04,
    search_radius: float = 0.25,
    step_height_tol: float = 0.08,
    stat_nb_neighbors: int = 20,
    stat_std_ratio: float = 2.0
):
    start_time = time.time()
    input_path = Path(input_pcd_path)
    output_path = Path(output_pcd_path)

    if not input_path.exists():
        print(f"[ERROR] 输入 PCD 文件不存在: {input_path}")
        sys.exit(1)

    print(f"==================================================")
    print(f"  PCD 地面与楼梯 3D KD-Tree 修复工具启动")
    print(f"==================================================")
    print(f"• 输入点云: {input_path}")
    print(f"• 输出点云: {output_path}")
    print(f"• 修复网格分辨率: {grid_res} m")
    print(f"• 局部搜索半径: {search_radius} m")
    print(f"• 台阶高度公差: {step_height_tol} m")

    # 1. 加载点云
    pcd = o3d.io.read_point_cloud(str(input_path))
    raw_points_count = len(pcd.points)
    print(f"\n[1/5] 读取原始点云成功，总点数: {raw_points_count:,}")

    # 2. 统计离群点滤波去噪
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=stat_nb_neighbors, std_ratio=stat_std_ratio)
    denoised_pts = np.asarray(cl.points)
    outliers_removed = raw_points_count - len(denoised_pts)
    print(f"[2/5] 统计滤波完成，剔除浮空散点噪点: {outliers_removed:,} 个 ({outliers_removed/raw_points_count*100:.2f}%)")

    # 3. 法向量估计与地面/踏步面分离
    cl.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=30))
    normals = np.asarray(cl.normals)

    # 踏步面/平坦地面判定：法向量偏向垂直 (|Nz| > 0.80)
    is_ground = np.abs(normals[:, 2]) > 0.80
    ground_pts = denoised_pts[is_ground]
    obstacle_pts = denoised_pts[~is_ground]
    print(f"[3/5] 表面法向量分析完成:")
    print(f"      • 提取地面/楼梯踏步面点数: {len(ground_pts):,}")
    print(f"      • 提取墙体/立面障碍物点数: {len(obstacle_pts):,}")

    # 4. 基于 3D KD-Tree 进行空洞检测与分层台阶修补
    print(f"[4/5] 正在构建 3D KD-Tree 索引并进行多层地面/楼梯空洞补全...")
    kdtree_2d = cKDTree(ground_pts[:, :2])
    
    x_min, y_min = ground_pts[:, 0].min(), ground_pts[:, 1].min()
    x_max, y_max = ground_pts[:, 0].max(), ground_pts[:, 1].max()

    gx, gy = np.meshgrid(
        np.arange(x_min, x_max, grid_res),
        np.arange(y_min, y_max, grid_res)
    )
    grid_xy = np.vstack([gx.ravel(), gy.ravel()]).T
    total_grid_cells = len(grid_xy)
    print(f"      • 扫描水平候选网格数: {total_grid_cells:,}")

    inpainted_pts = []
    # 批量查询半径内的邻居索引
    indices_list = kdtree_2d.query_ball_point(grid_xy, r=search_radius)

    for idx, (xy, neighbor_indices) in enumerate(zip(grid_xy, indices_list)):
        if len(neighbor_indices) < 4:
            continue

        local_pts = ground_pts[neighbor_indices]

        # 检查是否已经是密集点区域（若已有极近点，则跳过不需要补洞）
        dists_sq = (local_pts[:, 0] - xy[0])**2 + (local_pts[:, 1] - xy[1])**2
        if np.min(dists_sq) < (grid_res * 0.5)**2:
            continue

        # 聚类高度层（适配单层平地与多层台阶）
        z_median = np.median(local_pts[:, 2])
        same_layer_pts = local_pts[np.abs(local_pts[:, 2] - z_median) <= step_height_tol]

        if len(same_layer_pts) >= 4:
            # 局部平面 SVD 拟合
            centroid = np.mean(same_layer_pts, axis=0)
            diff = same_layer_pts - centroid
            cov = diff.T @ diff
            _, _, vh = np.linalg.svd(cov)
            normal = vh[2]

            # 确保局部拟合平面也是水平/微缓坡面
            if np.abs(normal[2]) > 0.75:
                # 平面方程计算该点的精确 Z 坐标
                z_fit = centroid[2] - (normal[0]*(xy[0]-centroid[0]) + normal[1]*(xy[1]-centroid[1])) / normal[2]
                inpainted_pts.append([xy[0], xy[1], z_fit])

    inpainted_pts = np.array(inpainted_pts) if len(inpainted_pts) > 0 else np.empty((0, 3))
    print(f"      • 空洞修补完成！成功补足地面/台阶有效点: {len(inpainted_pts):,} 个")

    # 5. 合并并保存修复后的点云
    output_path.parent.mkdir(parents=True, exist_ok=True)
    final_pts = np.vstack([obstacle_pts, ground_pts, inpainted_pts])
    
    repaired_pcd = o3d.geometry.PointCloud()
    repaired_pcd.points = o3d.utility.Vector3dVector(final_pts)
    o3d.io.write_point_cloud(str(output_path), repaired_pcd, write_ascii=False)
    
    elapsed = time.time() - start_time
    print(f"[5/5] 输出点云写入完毕: {output_path}")
    print(f"      • 最终总点数: {len(final_pts):,} (较原始点云净增加 {len(final_pts)-raw_points_count:+,} 个高质量支撑点)")
    print(f"• 总耗时: {elapsed:.2f} 秒")
    print(f"==================================================")
    return True

if __name__ == "__main__":
    pkg_dir = Path("/home/user/catkin_ws/src/jie_3d_nav_ros1-main/jie_octomap")
    in_file = pkg_dir / "pcd" / "map-segment.pcd"
    out_file = pkg_dir / "pcd" / "map-segment_repaired.pcd"

    repair_point_cloud(str(in_file), str(out_file))
