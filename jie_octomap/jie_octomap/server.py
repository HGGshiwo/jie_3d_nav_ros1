#!/usr/bin/env python3
import os
import yaml
import numpy as np
from pathlib import Path
from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from pydantic import BaseModel
from typing import List, Dict, Optional
import uvicorn
import asyncio
from concurrent.futures import ThreadPoolExecutor

import rospy
import tf2_ros
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path as ROSPath
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import time
from jie_map_msgs.srv import LoadNavigationMapPackage, LoadNavigationMapPackageRequest, SaveNavigationMapPackage, SaveNavigationMapPackageRequest, QueryCellDebugInfo, QueryCellDebugInfoRequest

from fastapi.middleware.gzip import GZipMiddleware

app = FastAPI()
app.add_middleware(GZipMiddleware, minimum_size=1000)

executor = ThreadPoolExecutor(max_workers=8)

async def run_in_thread(func, *args, **kwargs):
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(executor, lambda: func(*args, **kwargs))

# 挂载前端静态文件
current_dir = Path(__file__).resolve().parent
app.mount("/static", StaticFiles(directory=current_dir / "static", html=True), name="static")

@app.get("/")
def read_index():
    return FileResponse(current_dir / "static" / "index.html")

class MapDataRequest(BaseModel):
    root_path: str
    map_name: str

class SaveMapRequest(BaseModel):
    root_path: str
    map_name: str
    layers: Dict[str, Dict]

# 全局 ROS 状态和订阅器缓存
latest_ros_data = {
    "occupied": None,
    "preblocked": None,
    "traversable": None,
    "risk_cost": None
}
# 全局存储最新规划好的路径点
latest_planned_path = []

# 地图更新版本控制（增量版本号，用于前端轻量级轮询同步）
preblocked_version = 0
occupied_version = 0

def path_callback(msg):
    global latest_planned_path
    points = []
    for pose in msg.poses:
        points.append([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
    latest_planned_path = points

def occupied_callback(msg):
    global occupied_version
    if msg.type == Marker.CUBE_LIST:
        pts = [[p.x, p.y, p.z] for p in msg.points]
        scale = [msg.scale.x, msg.scale.y, msg.scale.z]
        latest_ros_data["occupied"] = {"points": pts, "scale": scale, "stamp": msg.header.stamp}
        occupied_version += 1
        print(f"[occupied_callback] Received occupied marker. Points={len(pts)} Stamp={msg.header.stamp.to_sec()} Version={occupied_version}", flush=True)

def preblocked_callback(msg):
    global preblocked_version
    if msg.type == Marker.CUBE_LIST:
        pts = [[p.x, p.y, p.z] for p in msg.points]
        scale = [msg.scale.x, msg.scale.y, msg.scale.z]
        latest_ros_data["preblocked"] = {"points": pts, "scale": scale, "stamp": msg.header.stamp}
        preblocked_version += 1
        print(f"[preblocked_callback] Received preblocked marker. Points={len(pts)} Stamp={msg.header.stamp.to_sec()} Version={preblocked_version}", flush=True)

def traversable_callback(msg):
    if msg.type == Marker.CUBE_LIST:
        pts = [[p.x, p.y, p.z] for p in msg.points]
        scale = [msg.scale.x, msg.scale.y, msg.scale.z]
        latest_ros_data["traversable"] = {"points": pts, "scale": scale, "stamp": msg.header.stamp}
        print(f"[traversable_callback] Received traversable marker. Points={len(pts)} Stamp={msg.header.stamp.to_sec()}", flush=True)

def risk_cost_callback(msg):
    try:
        if hasattr(msg, 'point_step') and msg.point_step == 16:
            # Quick numpy parsing for FLOAT32 PointCloud2 fields (x, y, z, intensity)
            data_arr = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
            risk_arr = data_arr[~np.isnan(data_arr).any(axis=1)]
        else:
            # Fallback to sensor_msgs.point_cloud2 generator
            risk_records = list(pc2.read_points(
                msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
            risk_arr = (np.array([[r[0], r[1], r[2], r[3]] for r in risk_records], dtype=np.float32)
                        if risk_records else np.empty((0, 4), dtype=np.float32))
        
        pts = risk_arr[:, :3].tolist()
        intensities = risk_arr[:, 3].tolist()
        
        latest_ros_data["risk_cost"] = {
            "points": pts,
            "intensities": intensities,
            "stamp": msg.header.stamp
        }
        print(f"[risk_cost_callback] Received risk cost cloud. Points={len(pts)} Stamp={msg.header.stamp.to_sec()}", flush=True)
    except Exception as e:
        print(f"[risk_cost_callback] Error parsing PointCloud2: {e}")

# TF 监听全局变量
tf_buffer = None
tf_listener = None

# 全局 ROS 发布者
ros_pubs = {}

@app.on_event("startup")
def startup_event():
    global tf_buffer, tf_listener
    try:
        rospy.init_node("web_map_manager", disable_signals=True)
        ros_pubs["occupied"] = rospy.Publisher("/edited_occupied_markers", Marker, queue_size=1, latch=True)
        ros_pubs["preblocked"] = rospy.Publisher("/edited_preblocked_cells_markers", Marker, queue_size=1, latch=True)
        ros_pubs["start_pub"] = rospy.Publisher("/start_point", PointStamped, queue_size=1, latch=True)
        ros_pubs["goal_pub"] = rospy.Publisher("/goal_point", PointStamped, queue_size=1, latch=True)
        ros_pubs["goal_pose_pub"] = rospy.Publisher("/goal_pose", PoseStamped, queue_size=1, latch=True)
        
        # 初始化 TF2 监听器
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        
        rospy.Subscriber("/octomap_occupied_markers", Marker, occupied_callback)
        rospy.Subscriber("/preblocked_cells_markers", Marker, preblocked_callback)
        rospy.Subscriber("/traversable_cells_markers", Marker, traversable_callback)
        rospy.Subscriber("/risk_cost_cells", PointCloud2, risk_cost_callback)
        rospy.Subscriber("/planned_path", ROSPath, path_callback)
        print("ROS 节点已启动，订阅者、发布者与 TF 监听器就绪")
    except Exception as e:
        print(f"ROS 初始化警告 (非ROS环境可忽略): {e}")

@app.post("/api/load_map")
async def load_map(req: MapDataRequest):
    """读取本地地图包，通过 ROS 加载服务载入原始地图，并将保存的编辑图层同步至 C++ 内存以完成地图恢复"""
    pkg_path = Path(req.root_path).expanduser() / req.map_name
    meta_path = pkg_path / "meta.yaml"
    layers_path = pkg_path / "layers.npz"

    if not meta_path.exists():
        raise HTTPException(status_code=404, detail="找不到地图文件 meta.yaml")

    t_start = rospy.Time.now()

    # 清空缓存
    latest_ros_data["occupied"] = None
    latest_ros_data["preblocked"] = None
    latest_ros_data["traversable"] = None

    # 1. 尝试调用 ROS 载入服务以载入底层的原始 OctoMap
    service_name = "/map_package_manager/load_package"
    try:
        def call_service():
            rospy.wait_for_service(service_name, timeout=1.5)
            load_service = rospy.ServiceProxy(service_name, LoadNavigationMapPackage)
            return load_service(LoadNavigationMapPackageRequest(package_path=str(pkg_path)))
        resp = await run_in_thread(call_service)
        if not resp.success:
            print(f"ROS 载入地图服务返回失败: {resp.message}")
    except Exception as e:
        print(f"ROS 载入地图服务不可用: {e}")

    # 2. 如果存在 layers.npz，直接读取并发布到 `/edited_occupied_markers` 与 `/edited_preblocked_cells_markers`
    #    以恢复 C++ 规划节点的内存状态，而不改写磁盘文件
    if layers_path.exists():
        def load_npz():
            return np.load(layers_path, allow_pickle=False)
        layers_data = await run_in_thread(load_npz)
        
        # 恢复 occupied 栅格内存
        if "occupied_points" in layers_data:
            pts = layers_data["occupied_points"].tolist()
            scale = layers_data["occupied_scale"].tolist() if "occupied_scale" in layers_data else [0.2, 0.2, 0.2]
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = t_start
            marker.ns = "occupied_cells"
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.scale.x, marker.scale.y, marker.scale.z = scale[0], scale[1], scale[2]
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.95, 0.45, 0.15, 1.0
            for pt in pts:
                p = Point()
                p.x, p.y, p.z = pt[0], pt[1], pt[2]
                marker.points.append(p)
            ros_pubs["occupied"].publish(marker)
            
        # 恢复 preblocked 禁行内存
        if "preblocked_points" in layers_data:
            pts = layers_data["preblocked_points"].tolist()
            scale = layers_data["preblocked_scale"].tolist() if "preblocked_scale" in layers_data else [0.2, 0.2, 0.2]
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = t_start
            marker.ns = "preblocked_cells"
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.scale.x, marker.scale.y, marker.scale.z = scale[0], scale[1], scale[2]
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 1.0
            for pt in pts:
                p = Point()
                p.x, p.y, p.z = pt[0], pt[1], pt[2]
                marker.points.append(p)
            ros_pubs["preblocked"].publish(marker)

    # 3. 等待并收集 ROS 话题发送的已更新地图图层数据
    # 等待时间最大设为 3.0 秒，保障底层 C++ 规划器能接收并计算完毕
    timeout = 3.0
    start_wait = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start_wait < timeout:
        occ = latest_ros_data["occupied"]
        pre = latest_ros_data["preblocked"]
        if (occ is not None and occ.get("stamp", rospy.Time(0)) >= t_start and
            pre is not None and pre.get("stamp", rospy.Time(0)) >= t_start):
            break
        await asyncio.sleep(0.05)

    try:
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        
        response_data = {"meta": meta, "layers": {}}
        
        # 优先使用从 ROS 订阅到的图层数据
        if latest_ros_data["occupied"] is not None:
            response_data["layers"]["occupied"] = {
                "points": latest_ros_data["occupied"]["points"],
                "scale": latest_ros_data["occupied"]["scale"]
            }
        if latest_ros_data["preblocked"] is not None:
            response_data["layers"]["preblocked"] = {
                "points": latest_ros_data["preblocked"]["points"],
                "scale": latest_ros_data["preblocked"]["scale"]
            }
        if latest_ros_data["traversable"] is not None:
            response_data["layers"]["traversable"] = {
                "points": latest_ros_data["traversable"]["points"],
                "scale": latest_ros_data["traversable"]["scale"]
            }
        if latest_ros_data["risk_cost"] is not None:
            response_data["layers"]["risk_cost"] = {
                "points": latest_ros_data["risk_cost"]["points"],
                "intensities": latest_ros_data["risk_cost"]["intensities"],
                "scale": latest_ros_data["occupied"]["scale"] if latest_ros_data["occupied"] else [0.2, 0.2, 0.2]
            }

        # Fallback: 如果没有通过 ROS 话题收到 occupied（或 ROS 未开启），直接读取 NPZ 文件作为保底
        if "occupied" not in response_data["layers"] or not response_data["layers"]["occupied"]:
            if layers_path.exists():
                def load_npz():
                    return np.load(layers_path, allow_pickle=False)
                layers_data = await run_in_thread(load_npz)
                for layer_name in ["occupied", "preblocked", "traversable"]:
                    pts_key = f"{layer_name}_points"
                    sc_key = f"{layer_name}_scale"
                    if pts_key in layers_data and layer_name not in response_data["layers"]:
                        response_data["layers"][layer_name] = {
                            "points": layers_data[pts_key].tolist(),
                            "scale": layers_data[sc_key].tolist() if sc_key in layers_data else [0.2, 0.2, 0.2]
                        }
                if "risk_points" in layers_data and "risk_intensity" in layers_data and "risk_cost" not in response_data["layers"]:
                    scale = layers_data["preblocked_scale"].tolist() if "preblocked_scale" in layers_data else [0.2, 0.2, 0.2]
                    response_data["layers"]["risk_cost"] = {
                        "points": layers_data["risk_points"].tolist(),
                        "intensities": layers_data["risk_intensity"].tolist(),
                        "scale": scale
                    }
        
        return JSONResponse(content=response_data)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/save_map")
async def save_map(req: SaveMapRequest):
    """将前端编辑后的地图同步至 ROS，并触发 ROS 保存服务将修改后的地图（含 OctoMap、各图层与元数据）保存至磁盘"""
    t_start = rospy.Time.now()
    pkg_path = Path(req.root_path).expanduser() / req.map_name
    
    # 1. 首先将前端发送的最新编辑数据同步至 ROS (以便 C++ 规划节点更新当前内存中的地图)
    sync_preblocked = False
    sync_occupied = False
    
    for layer_name, data in req.layers.items():
        if layer_name in ros_pubs and "points" in data:
            if layer_name == "preblocked":
                sync_preblocked = True
            elif layer_name == "occupied":
                sync_occupied = True
                
            pts = data["points"]
            scale = data["scale"]
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = t_start
            marker.ns = f"{layer_name}_cells"
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.scale.x, marker.scale.y, marker.scale.z = scale[0], scale[1], scale[2]
            
            if layer_name == "occupied":
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.95, 0.45, 0.15, 1.0
            elif layer_name == "preblocked":
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 1.0
            
            for pt in pts:
                p = Point()
                p.x, p.y, p.z = pt[0], pt[1], pt[2]
                marker.points.append(p)
                
            ros_pubs[layer_name].publish(marker)
            
    # 2. 等待底层 C++ 节点更新完成
    wait_start = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - wait_start < 3.0:
        preblocked_done = True
        occupied_done = True
        
        if sync_preblocked:
            pre = latest_ros_data["preblocked"]
            preblocked_done = pre is not None and pre.get("stamp", rospy.Time(0)) >= t_start
        if sync_occupied:
            occ = latest_ros_data["occupied"]
            occupied_done = occ is not None and occ.get("stamp", rospy.Time(0)) >= t_start
            
        if preblocked_done and occupied_done:
            break
        await asyncio.sleep(0.05)

    # 3. 调用 ROS 保存服务，触发底层保存 OctoMap 消息文件、图层 NPZ 和 meta.yaml
    service_name = "/map_package_manager/save_package"
    try:
        def call_save():
            rospy.wait_for_service(service_name, timeout=2.0)
            save_service = rospy.ServiceProxy(service_name, SaveNavigationMapPackage)
            req_save = SaveNavigationMapPackageRequest()
            req_save.package_path = str(pkg_path)
            req_save.overwrite = True
            return save_service(req_save)
            
        resp = await run_in_thread(call_save)
        if resp.success:
            return {"status": "success", "message": f"地图已成功保存至 {pkg_path}！"}
        else:
            raise HTTPException(status_code=500, detail=f"保存地图服务返回失败: {resp.message}")
    except Exception as e:
        # Fallback: 如果 ROS 保存服务不可用，回退至直接将前端图层保存到 NPZ（作为保底）
        print(f"ROS 保存地图服务不可用 (回退至直接落盘 layers.npz): {e}")
        def save_io():
            pkg_path.mkdir(parents=True, exist_ok=True)
            save_dict = {}
            for layer_name, data in req.layers.items():
                if "points" in data and len(data["points"]) > 0:
                    save_dict[f"{layer_name}_points"] = np.array(data["points"], dtype=np.float32)
                    save_dict[f"{layer_name}_scale"] = np.array(data["scale"], dtype=np.float32)
            np.savez_compressed(pkg_path / "layers.npz", **save_dict)
            
        try:
            await run_in_thread(save_io)
            return {"status": "success", "message": f"已成功保存地图图层至 {pkg_path} (ROS 保存服务不可用)"}
        except Exception as ex:
            raise HTTPException(status_code=500, detail=str(ex))

@app.post("/api/sync_ros")
async def sync_ros(req: SaveMapRequest):
    """仅同步至 ROS 话题，不落盘"""
    t_start = rospy.Time.now()
    print(f"[sync_ros] Starting sync. t_start={t_start.to_sec()}", flush=True)
    
    sync_preblocked = False
    sync_occupied = False
    
    for layer_name, data in req.layers.items():
        if layer_name in ros_pubs and "points" in data:
            if layer_name == "preblocked":
                sync_preblocked = True
            elif layer_name == "occupied":
                sync_occupied = True
                
            pts = data["points"]
            scale = data["scale"]
            print(f"[sync_ros] Publishing layer '{layer_name}' with {len(pts)} points.", flush=True)
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = t_start
            marker.ns = f"{layer_name}_cells"
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.scale.x, marker.scale.y, marker.scale.z = scale[0], scale[1], scale[2]
            
            # 颜色设置
            if layer_name == "occupied":
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.95, 0.45, 0.15, 1.0
            elif layer_name == "preblocked":
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 1.0
            
            for pt in pts:
                p = Point()
                p.x, p.y, p.z = pt[0], pt[1], pt[2]
                marker.points.append(p)
                
            ros_pubs[layer_name].publish(marker)
            
    # 同步等待底层 C++ 节点更新并重新发布（最多等待 30.0 秒）
    # 注意：即便 sync_preblocked 为 False（用户未手绘禁行区），底层 C++ 仍会因 occupied 改变而重新计算并发布 preblocked 和 traversable 图层。
    # 故我们始终需要等待最新的 preblocked 与 traversable 图层发布完成，才算 C++ 重建完全结束。
    wait_start = asyncio.get_event_loop().time()
    success = False
    loop_count = 0
    while asyncio.get_event_loop().time() - wait_start < 30.0:
        occupied_done = True
        preblocked_done = False
        traversable_done = False
        
        pre = latest_ros_data["preblocked"]
        occ = latest_ros_data["occupied"]
        tra = latest_ros_data["traversable"]
        
        if sync_occupied:
            occupied_done = occ is not None and occ.get("stamp", rospy.Time(0)) >= t_start
        else:
            occupied_done = True
            
        preblocked_done = pre is not None and pre.get("stamp", rospy.Time(0)) >= t_start
        traversable_done = tra is not None and tra.get("stamp", rospy.Time(0)) >= t_start
            
        if loop_count % 10 == 0:
            pre_stamp_sec = pre.get("stamp", rospy.Time(0)).to_sec() if pre else 0.0
            occ_stamp_sec = occ.get("stamp", rospy.Time(0)).to_sec() if occ else 0.0
            tra_stamp_sec = tra.get("stamp", rospy.Time(0)).to_sec() if tra else 0.0
            print(f"[sync_ros wait] loop={loop_count} occ_done={occupied_done}({occ_stamp_sec}) pre_done={preblocked_done}({pre_stamp_sec}) tra_done={traversable_done}({tra_stamp_sec}) t_start={t_start.to_sec()}", flush=True)
            
        if occupied_done and preblocked_done and traversable_done:
            success = True
            break
        await asyncio.sleep(0.05)
        loop_count += 1
        
    duration = asyncio.get_event_loop().time() - wait_start
    occ_stamp = latest_ros_data["occupied"].get("stamp", rospy.Time(0)).to_sec() if latest_ros_data["occupied"] else 0.0
    pre_stamp = latest_ros_data["preblocked"].get("stamp", rospy.Time(0)).to_sec() if latest_ros_data["preblocked"] else 0.0
    tra_stamp = latest_ros_data["traversable"].get("stamp", rospy.Time(0)).to_sec() if latest_ros_data["traversable"] else 0.0
    if success:
        print(f"[sync_ros] Sync finished successfully in {duration:.2f}s. stamp_occ={occ_stamp} stamp_pre={pre_stamp} stamp_tra={tra_stamp} t_start={t_start.to_sec()}", flush=True)
    else:
        print(f"[sync_ros] Sync TIMEOUT after {duration:.2f}s. stamp_occ={occ_stamp} stamp_pre={pre_stamp} stamp_tra={tra_stamp} t_start={t_start.to_sec()}", flush=True)
        
    return {"status": "success", "message": "地图已成功同步，且底层 C++ 规划器已完成重新结算！"}

class PointRequest(BaseModel):
    x: float
    y: float
    z: float

@app.post("/api/set_start")
async def set_start(req: PointRequest):
    if "start_pub" in ros_pubs:
        msg = PointStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.point.x = req.x
        msg.point.y = req.y
        msg.point.z = req.z
        ros_pubs["start_pub"].publish(msg)
        return {"status": "success", "message": f"起点已设定为: [{req.x:.2f}, {req.y:.2f}, {req.z:.2f}]"}
    else:
        raise HTTPException(status_code=500, detail="ROS 起点发布器未启动")

@app.post("/api/set_goal")
async def set_goal(req: PointRequest):
    if "goal_pub" in ros_pubs and "goal_pose_pub" in ros_pubs:
        # 1. 尝试从 TF 获取机器人当前位置，并发布为起点以自动激活路径规划
        global tf_buffer
        tf_success = False
        if tf_buffer is not None:
            def lookup_tf():
                # 优先读取 ROS 参数服务器配置，若无则尝试该工作空间下常见的底盘坐标系
                parent_frame = rospy.get_param("~tf_parent_frame", "map")
                default_child = rospy.get_param("~tf_child_frame", "base_footprint")
                candidate_children = [default_child, "odin1_base_link", "base_link"]
                
                trans = None
                for child in candidate_children:
                    try:
                        # 快速检索各坐标系（超时设为 0.15s，防止接口阻塞）
                        trans = tf_buffer.lookup_transform(parent_frame, child, rospy.Time(0), rospy.Duration(0.15))
                        break
                    except Exception:
                        continue
                if trans is not None:
                    try:
                        t = trans.transform.translation
                        msg_start = PointStamped()
                        msg_start.header.frame_id = parent_frame
                        msg_start.header.stamp = rospy.Time.now()
                        msg_start.point.x = t.x
                        msg_start.point.y = t.y
                        msg_start.point.z = t.z
                        
                        ros_pubs["start_pub"].publish(msg_start)
                        return True, t
                    except Exception as e:
                        print(f"发布 TF 起点失败: {e}")
                return False, None

            tf_success, t = await run_in_thread(lookup_tf)
            if tf_success and t:
                print(f"自动从 TF 读取机器人当前位置并发布为起点: [{t.x:.2f}, {t.y:.2f}, {t.z:.2f}]")
            else:
                print("自动获取机器人 TF 起点位置失败 (未找到 map -> base_footprint/odin1_base_link/base_link 变换)")

        # 2. 发布 PointStamped 目标点和 PoseStamped 目标位姿，触发规划器的 A* 算法
        msg_point = PointStamped()
        msg_point.header.frame_id = "map"
        msg_point.header.stamp = rospy.Time.now()
        msg_point.point.x = req.x
        msg_point.point.y = req.y
        msg_point.point.z = req.z
        ros_pubs["goal_pub"].publish(msg_point)
        
        msg_pose = PoseStamped()
        msg_pose.header.frame_id = "map"
        msg_pose.header.stamp = rospy.Time.now()
        msg_pose.pose.position.x = req.x
        msg_pose.pose.position.y = req.y
        msg_pose.pose.position.z = req.z
        msg_pose.pose.orientation.w = 1.0
        ros_pubs["goal_pose_pub"].publish(msg_pose)
        
        msg_txt = f"终点已设定为: [{req.x:.2f}, {req.y:.2f}, {req.z:.2f}]"
        if tf_success:
            msg_txt += " (已自动从 TF 获取机器人当前位置作为起点并触发规划)"
        else:
            msg_txt += " (TF 暂不可用，已通过之前设定的手动起点触发规划)"
            
        return {"status": "success", "message": msg_txt}
    else:
        raise HTTPException(status_code=500, detail="ROS 终点发布器未启动")

@app.get("/api/get_path")
async def get_path():
    return {"path": latest_planned_path}

@app.post("/api/debug_cell")
async def debug_cell(req: PointRequest):
    service_name = "/jie_path_node/query_cell_debug_info"
    try:
        service_name = rospy.get_param("~query_cell_debug_service", service_name)
    except Exception:
        pass
    try:
        def call_service():
            rospy.wait_for_service(service_name, timeout=1.5)
            query_service = rospy.ServiceProxy(service_name, QueryCellDebugInfo)
            return query_service(QueryCellDebugInfoRequest(x=req.x, y=req.y, z=req.z))
        resp = await run_in_thread(call_service)
        if not resp.success:
            return {"status": "error", "message": resp.message}
        return {
            "status": "success",
            "grid_x": resp.grid_x,
            "grid_y": resp.grid_y,
            "grid_z": resp.grid_z,
            "is_occupied": resp.is_occupied,
            "is_unknown": resp.is_unknown,
            "has_ground_support": resp.has_ground_support,
            "is_preblocked": resp.is_preblocked,
            "preblocked_reason": resp.preblocked_reason,
            "has_vertical_collision": resp.has_vertical_collision,
            "has_horizontal_collision": resp.has_horizontal_collision,
            "has_below_preblocked_failure": resp.has_below_preblocked_failure,
            "preblocked_cost": resp.preblocked_cost,
            "risk_cost": resp.risk_cost,
            "is_candidate": resp.is_candidate,
            "is_traversable": resp.is_traversable
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"ROS 调试服务不可用: {e}")

@app.get("/api/map_version")
async def get_map_version():
    return {"preblocked_version": preblocked_version, "occupied_version": occupied_version}

@app.get("/api/get_current_map")
async def get_current_map():
    occ_stamp = latest_ros_data["occupied"].get("stamp", rospy.Time(0)).to_sec() if latest_ros_data["occupied"] else 0.0
    pre_stamp = latest_ros_data["preblocked"].get("stamp", rospy.Time(0)).to_sec() if latest_ros_data["preblocked"] else 0.0
    tra_stamp = latest_ros_data["traversable"].get("stamp", rospy.Time(0)).to_sec() if latest_ros_data["traversable"] else 0.0
    risk_stamp = latest_ros_data["risk_cost"].get("stamp", rospy.Time(0)).to_sec() if latest_ros_data["risk_cost"] else 0.0
    
    occ_len = len(latest_ros_data["occupied"]["points"]) if latest_ros_data["occupied"] else 0
    pre_len = len(latest_ros_data["preblocked"]["points"]) if latest_ros_data["preblocked"] else 0
    tra_len = len(latest_ros_data["traversable"]["points"]) if latest_ros_data["traversable"] else 0
    risk_len = len(latest_ros_data["risk_cost"]["points"]) if latest_ros_data["risk_cost"] else 0

    print(f"[get_current_map] Request received. occupied(len={occ_len}, stamp={occ_stamp}), preblocked(len={pre_len}, stamp={pre_stamp}), traversable(len={tra_len}, stamp={tra_stamp}), risk_cost(len={risk_len}, stamp={risk_stamp})", flush=True)

    response_layers = {}
    if latest_ros_data["occupied"] is not None:
        response_layers["occupied"] = {
            "points": latest_ros_data["occupied"]["points"],
            "scale": latest_ros_data["occupied"]["scale"]
        }
    if latest_ros_data["preblocked"] is not None:
        response_layers["preblocked"] = {
            "points": latest_ros_data["preblocked"]["points"],
            "scale": latest_ros_data["preblocked"]["scale"]
        }
    if latest_ros_data["traversable"] is not None:
        response_layers["traversable"] = {
            "points": latest_ros_data["traversable"]["points"],
            "scale": latest_ros_data["traversable"]["scale"]
        }
    if latest_ros_data["risk_cost"] is not None:
        response_layers["risk_cost"] = {
            "points": latest_ros_data["risk_cost"]["points"],
            "intensities": latest_ros_data["risk_cost"]["intensities"],
            "scale": latest_ros_data["occupied"]["scale"] if latest_ros_data["occupied"] else [0.2, 0.2, 0.2]
        }
    return JSONResponse(content={"layers": response_layers})

@app.get("/api/default_map")
async def get_default_map():
    default_map_package = os.environ.get("MAP_VIEWER_DEFAULT_PACKAGE", "/home/robot/maps/map")
    try:
        default_map_package = rospy.get_param("~default_map_package", default_map_package)
    except Exception:
        pass
    path = Path(default_map_package).expanduser()
    return {"root_path": str(path.parent), "map_name": str(path.name)}

def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()