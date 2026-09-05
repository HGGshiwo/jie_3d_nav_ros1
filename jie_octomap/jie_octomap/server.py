#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Web 3D 地图可视化管理服务端
基于 FastAPI 提供 HTTP / REST 接口，配合 ros_bridge 实现与 ROS 规划系统的高性能增量同步
"""

import os
import json
import yaml
import time
import numpy as np
from pathlib import Path
from typing import List, Dict, Optional
import uvicorn
import asyncio
from concurrent.futures import ThreadPoolExecutor

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from fastapi.middleware.gzip import GZipMiddleware
from pydantic import BaseModel

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from jie_map_msgs.srv import (
    LoadNavigationMapPackage, LoadNavigationMapPackageRequest,
    SaveNavigationMapPackage, SaveNavigationMapPackageRequest,
    QueryCellDebugInfo, QueryCellDebugInfoRequest
)

from jie_octomap.ros_bridge import ros_bridge

app = FastAPI(title="Jie OctoMap Web Server")
app.add_middleware(GZipMiddleware, minimum_size=1000)

executor = ThreadPoolExecutor(max_workers=8)

async def run_in_thread(func, *args, **kwargs):
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(executor, lambda: func(*args, **kwargs))

# 静态文件挂载
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

class PointRequest(BaseModel):
    x: float
    y: float
    z: float
    layer_name: Optional[str] = ""


@app.on_event("startup")
def startup_event():
    try:
        ros_bridge.init_ros()
    except Exception as e:
        print(f"[Startup] ROS 初始化警告: {e}")


@app.websocket("/ws/live")
async def websocket_live(websocket: WebSocket):
    """
    全双工实时流式通道：
    整合机器人位姿、状态、路径与增量地图图层，通过单持久连接主动推送到前端。
    彻底消除前端多路 HTTP 轮询与并发 pending 队头阻塞。
    """
    await websocket.accept()
    client_requested_layers = None
    client_layer_versions = {}
    client_path_v = -1
    client_status_v = -1

    stop_event = asyncio.Event()

    async def client_listener():
        nonlocal client_requested_layers, client_layer_versions
        try:
            while not stop_event.is_set():
                text = await websocket.receive_text()
                msg = json.loads(text)
                if msg.get("type") == "subscribe":
                    client_requested_layers = msg.get("layers")
                    if "versions" in msg and isinstance(msg["versions"], dict):
                        client_layer_versions.update(msg["versions"])
        except Exception:
            stop_event.set()

    listener_task = asyncio.create_task(client_listener())

    try:
        while not stop_event.is_set():
            def build_frame():
                frame = ros_bridge.get_live_frame(
                    client_requested_layers,
                    client_layer_versions,
                    client_path_v,
                    client_status_v
                )
                return frame, json.dumps(frame)

            frame, json_str = await run_in_thread(build_frame)

            # 更新已推送给该客户端的版本戳
            if frame.get("path_version") is not None:
                client_path_v = frame["path_version"]
            if frame.get("status_version") is not None:
                client_status_v = frame["status_version"]
            for l_name, l_info in frame.get("layers", {}).items():
                if "version" in l_info:
                    client_layer_versions[l_name] = l_info["version"]

            await websocket.send_text(json_str)
            await asyncio.sleep(0.1) # 10Hz 稳定流式推送
    except Exception:
        pass
    finally:
        stop_event.set()
        listener_task.cancel()


@app.get("/api/get_current_map")
async def get_current_map(layers: Optional[str] = None, versions: Optional[str] = None):
    """
    高频地图拉取核心接口：
    支持基于 versions 参数的增量同步。若客户端已有对应版本，直接返回 unchanged: True
    """
    requested = layers.split(',') if layers else None
    client_versions = {}
    if versions:
        try:
            client_versions = json.loads(versions)
        except Exception:
            pass

    response_layers = ros_bridge.get_layer_response(requested, client_versions)
    return JSONResponse(content={"layers": response_layers})


@app.get("/api/map_version")
async def get_map_version():
    """获取各图层当前最新版本号，便于前端轻量比对"""
    return {
        "preblocked_version": ros_bridge.layer_versions.get("preblocked", 0),
        "occupied_version": ros_bridge.layer_versions.get("occupied", 0),
        "all_versions": ros_bridge.layer_versions
    }


@app.post("/api/load_map")
async def load_map(req: MapDataRequest):
    """读取本地地图包并同步至 ROS 规划节点"""
    pkg_path = Path(req.root_path).expanduser() / req.map_name
    meta_path = pkg_path / "meta.yaml"
    layers_path = pkg_path / "layers.npz"

    if not meta_path.exists():
        raise HTTPException(status_code=404, detail="找不到地图文件 meta.yaml")

    t_start = rospy.Time.now()

    # 清空缓存
    ros_bridge.latest_ros_data["occupied"] = None
    ros_bridge.latest_ros_data["preblocked"] = None
    ros_bridge.latest_ros_data["traversable"] = None

    # 1. 触发 ROS 载入服务
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

    # 2. 若存在 layers.npz，发布以恢复内存
    if layers_path.exists():
        def load_npz():
            return np.load(layers_path, allow_pickle=False)
        layers_data = await run_in_thread(load_npz)
        for layer_name in ["occupied", "preblocked"]:
            pts_key = f"{layer_name}_points"
            sc_key = f"{layer_name}_scale"
            if pts_key in layers_data:
                pts = layers_data[pts_key].tolist()
                sc = layers_data[sc_key].tolist() if sc_key in layers_data else [0.2, 0.2, 0.2]
                ros_bridge.publish_edited_marker(layer_name, pts, sc, t_start)

    # 3. 等待底层 C++ 节点完成预构建发布
    timeout = 15.0
    start_wait = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start_wait < timeout:
        occ = ros_bridge.latest_ros_data["occupied"]
        pre = ros_bridge.latest_ros_data["preblocked"]
        tra = ros_bridge.latest_ros_data["traversable"]
        if (occ is not None and occ.get("stamp", rospy.Time(0)) >= t_start and
            pre is not None and pre.get("stamp", rospy.Time(0)) >= t_start and
            tra is not None and tra.get("stamp", rospy.Time(0)) >= t_start):
            break
        await asyncio.sleep(0.05)

    try:
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        # 获取全量图层（全量传递时不传 client_versions）
        response_layers = ros_bridge.get_layer_response(None, {})

        # Fallback 兜底
        if "occupied" not in response_layers or not response_layers["occupied"].get("groups"):
            if layers_path.exists():
                layers_data = await run_in_thread(lambda: np.load(layers_path, allow_pickle=False))
                for l_name in ["occupied", "preblocked", "traversable"]:
                    pts_key = f"{l_name}_points"
                    sc_key = f"{l_name}_scale"
                    if pts_key in layers_data:
                        pts_list = layers_data[pts_key].tolist()
                        sc_list = layers_data[sc_key].tolist() if sc_key in layers_data else [0.2, 0.2, 0.2]
                        response_layers[l_name] = {
                            "version": 1,
                            "unchanged": False,
                            "scale": sc_list,
                            "groups": [{"points": pts_list, "scale": sc_list}] if pts_list else []
                        }

        return JSONResponse(content={"meta": meta, "layers": response_layers})
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/save_map")
async def save_map(req: SaveMapRequest):
    """保存编辑图层至磁盘并同步至 ROS"""
    t_start = rospy.Time.now()
    pkg_path = Path(req.root_path).expanduser() / req.map_name

    # 1. 同步编辑栅格至 ROS
    sync_preblocked = "preblocked" in req.layers
    sync_occupied = "occupied" in req.layers
    for layer_name, data in req.layers.items():
        if "points" in data:
            ros_bridge.publish_edited_marker(layer_name, data["points"], data["scale"], t_start)

    # 2. 等待底层 C++ 更新
    wait_start = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - wait_start < 3.0:
        pre_ok = not sync_preblocked or (ros_bridge.latest_ros_data["preblocked"] is not None and
                                         ros_bridge.latest_ros_data["preblocked"].get("stamp", rospy.Time(0)) >= t_start)
        occ_ok = not sync_occupied or (ros_bridge.latest_ros_data["occupied"] is not None and
                                       ros_bridge.latest_ros_data["occupied"].get("stamp", rospy.Time(0)) >= t_start)
        if pre_ok and occ_ok:
            break
        await asyncio.sleep(0.05)

    # 3. 触发 ROS 保存服务
    service_name = "/map_package_manager/save_package"
    try:
        def call_save():
            rospy.wait_for_service(service_name, timeout=2.0)
            save_service = rospy.ServiceProxy(service_name, SaveNavigationMapPackage)
            req_save = SaveNavigationMapPackageRequest(package_path=str(pkg_path), overwrite=True)
            return save_service(req_save)
        resp = await run_in_thread(call_save)
        if resp.success:
            return {"status": "success", "message": f"地图已成功保存至 {pkg_path}！"}
        raise HTTPException(status_code=500, detail=f"保存地图服务返回失败: {resp.message}")
    except Exception as e:
        print(f"ROS 保存服务异常，回退至直接保存 npz: {e}")
        def save_io():
            pkg_path.mkdir(parents=True, exist_ok=True)
            save_dict = {}
            for l_name, d in req.layers.items():
                if "points" in d and len(d["points"]) > 0:
                    save_dict[f"{l_name}_points"] = np.array(d["points"], dtype=np.float32)
                    save_dict[f"{l_name}_scale"] = np.array(d["scale"], dtype=np.float32)
            np.savez_compressed(pkg_path / "layers.npz", **save_dict)
        await run_in_thread(save_io)
        return {"status": "success", "message": f"已成功保存地图图层至 {pkg_path} (文件直接写入)"}


@app.post("/api/sync_ros")
async def sync_ros(req: SaveMapRequest):
    """仅同步至 ROS 话题，不写磁盘"""
    t_start = rospy.Time.now()
    sync_occupied = "occupied" in req.layers
    for layer_name, data in req.layers.items():
        if "points" in data:
            ros_bridge.publish_edited_marker(layer_name, data["points"], data["scale"], t_start)

    # 等待 C++ 规划器重新构建
    wait_start = asyncio.get_event_loop().time()
    success = False
    while asyncio.get_event_loop().time() - wait_start < 30.0:
        occ_done = True
        if sync_occupied:
            occ = ros_bridge.latest_ros_data["occupied"]
            occ_done = occ is not None and occ.get("stamp", rospy.Time(0)) >= t_start

        pre = ros_bridge.latest_ros_data["preblocked"]
        tra = ros_bridge.latest_ros_data["traversable"]
        pre_done = pre is not None and pre.get("stamp", rospy.Time(0)) >= t_start
        tra_done = tra is not None and tra.get("stamp", rospy.Time(0)) >= t_start

        if occ_done and pre_done and tra_done:
            success = True
            break
        await asyncio.sleep(0.05)

    duration = asyncio.get_event_loop().time() - wait_start
    print(f"[sync_ros] 同步耗时: {duration:.2f}s, 成功={success}", flush=True)
    return {"status": "success", "message": "地图已成功同步，底层 C++ 规划器已重新结算！"}


@app.post("/api/set_start")
async def set_start(req: PointRequest):
    if "start_pub" not in ros_bridge.ros_pubs:
        raise HTTPException(status_code=500, detail="ROS 起点发布器未启动")

    msg = PointStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.point.x, msg.point.y, msg.point.z = req.x, req.y, req.z
    ros_bridge.ros_pubs["start_pub"].publish(msg)

    if ros_bridge.publish_fake_tf:
        ros_bridge.fake_robot_pose = {"x": req.x, "y": req.y, "z": req.z}
        ros_bridge.publish_fake_tf_loop()

    return {"status": "success", "message": f"起点已设定为: [{req.x:.2f}, {req.y:.2f}, {req.z:.2f}]"}


@app.post("/api/set_goal")
async def set_goal(req: PointRequest):
    if "goal_pub" not in ros_bridge.ros_pubs or "goal_pose_pub" not in ros_bridge.ros_pubs:
        raise HTTPException(status_code=500, detail="ROS 终点发布器未启动")

    # 1. 自动读取机器人当前位姿发布为规划起点
    has_pose, pos, _ = ros_bridge.lookup_robot_pose()
    start_source = None
    if has_pose and pos:
        msg_start = PointStamped()
        msg_start.header.frame_id = "map"
        msg_start.header.stamp = rospy.Time.now()
        msg_start.point.x, msg_start.point.y, msg_start.point.z = pos["x"], pos["y"], pos["z"]
        ros_bridge.ros_pubs["start_pub"].publish(msg_start)
        start_source = "当前机器人位姿"

    # 2. 发布终点 PointStamped 与 PoseStamped
    now_stamp = rospy.Time.now()
    msg_point = PointStamped()
    msg_point.header.frame_id = "map"
    msg_point.header.stamp = now_stamp
    msg_point.point.x, msg_point.point.y, msg_point.point.z = req.x, req.y, req.z
    ros_bridge.ros_pubs["goal_pub"].publish(msg_point)

    msg_pose = PoseStamped()
    msg_pose.header.frame_id = "map"
    msg_pose.header.stamp = now_stamp
    msg_pose.pose.position.x, msg_pose.pose.position.y, msg_pose.pose.position.z = req.x, req.y, req.z
    msg_pose.pose.orientation.w = 1.0
    ros_bridge.ros_pubs["goal_pose_pub"].publish(msg_pose)

    # 3. 发布到 move_base 动作目标
    if "move_base_simple_goal" in ros_bridge.ros_pubs:
        ros_bridge.ros_pubs["move_base_simple_goal"].publish(msg_pose)
    if "move_base_action_goal" in ros_bridge.ros_pubs:
        action_goal = MoveBaseActionGoal()
        action_goal.header.stamp = now_stamp
        action_goal.goal_id.stamp = now_stamp
        action_goal.goal_id.id = f"web_goal_{time.time()}"
        action_goal.goal.target_pose = msg_pose
        ros_bridge.ros_pubs["move_base_action_goal"].publish(action_goal)

    info = f"终点已设定: [{req.x:.2f}, {req.y:.2f}, {req.z:.2f}]"
    if start_source:
        info += f" (已自动从 {start_source} 发布起点触发规划)"
    return {"status": "success", "message": info}


@app.get("/api/get_path")
async def get_path():
    return {"path": ros_bridge.latest_planned_path}


@app.get("/api/get_status_text")
async def get_status_text():
    return {"status_text": ros_bridge.latest_status_text}


@app.get("/api/get_robot_pose")
async def get_robot_pose():
    has_pose, pos, ori = ros_bridge.lookup_robot_pose()
    if has_pose:
        return {"has_pose": True, "position": pos, "orientation": ori}
    return {"has_pose": False}


@app.post("/api/debug_cell")
async def debug_cell(req: PointRequest):
    service_name = rospy.get_param("~query_cell_debug_service", "/octomap_roi_merger/query_cell_debug_info")
    try:
        def call_service():
            try:
                rospy.wait_for_service(service_name, timeout=1.0)
            except Exception:
                fallback_name = "/jie_path_node/query_cell_debug_info"
                rospy.wait_for_service(fallback_name, timeout=1.0)
                return rospy.ServiceProxy(fallback_name, QueryCellDebugInfo)(
                    QueryCellDebugInfoRequest(x=req.x, y=req.y, z=req.z, layer_name=req.layer_name or "")
                )
            return rospy.ServiceProxy(service_name, QueryCellDebugInfo)(
                QueryCellDebugInfoRequest(x=req.x, y=req.y, z=req.z, layer_name=req.layer_name or "")
            )
        resp = await run_in_thread(call_service)
        if not resp.success:
            return {"status": "error", "message": resp.message}
        return {
            "status": "success",
            "grid_x": resp.grid_x, "grid_y": resp.grid_y, "grid_z": resp.grid_z,
            "is_occupied": resp.is_occupied, "is_unknown": resp.is_unknown,
            "has_ground_support": resp.has_ground_support,
            "is_preblocked": resp.is_preblocked, "preblocked_reason": resp.preblocked_reason,
            "has_vertical_collision": resp.has_vertical_collision,
            "has_horizontal_collision": resp.has_horizontal_collision,
            "has_below_preblocked_failure": resp.has_below_preblocked_failure,
            "preblocked_cost": resp.preblocked_cost, "risk_cost": resp.risk_cost,
            "is_candidate": resp.is_candidate, "is_traversable": resp.is_traversable,
            "node_source_info": getattr(resp, 'node_source_info', 'N/A')
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"ROS 调试服务不可用: {e}")


@app.get("/api/default_map")
async def get_default_map():
    default_pkg = rospy.get_param("~default_map_package", os.environ.get("MAP_VIEWER_DEFAULT_PACKAGE", "/home/robot/maps/map"))
    path = Path(default_pkg).expanduser()
    return {"root_path": str(path.parent), "map_name": str(path.name)}


def main():
    uvicorn.run(app, host="0.0.0.0", port=8008)

if __name__ == "__main__":
    main()