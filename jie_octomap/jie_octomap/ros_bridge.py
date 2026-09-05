#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 通信与状态管理桥接模块 (ROS Bridge)
负责：
1. ROS 节点初始化、TF 监听与伪 TF 高频广播
2. 所有地图图层与状态话题的订阅与高效缓存
3. 图层增量版本号 (Layer Versions) 维护，供前端实现轻量差量同步
4. 起点、终点、目标位姿以及编辑栅格话题的发布
"""

import os
import time
import numpy as np
from typing import Dict, List, Optional, Any, Tuple

import rospy
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Path as ROSPath, Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String


def parse_marker_or_array(msg: Any) -> Dict[str, Any]:
    """
    高效解析 ROS Marker 或 MarkerArray 消息：
    提取各 Marker 的 cube 组，记录其点集与分辨率 scale
    """
    groups = []
    stamp = getattr(msg, 'header', None).stamp if hasattr(msg, 'header') else None
    
    if isinstance(msg, MarkerArray):
        for m in msg.markers:
            if m.type == Marker.CUBE_LIST and m.action == Marker.ADD:
                pts = [[p.x, p.y, p.z] for p in m.points]
                sc = [m.scale.x, m.scale.y, m.scale.z] if m.scale.x > 0 else [0.1, 0.1, 0.1]
                if pts:
                    groups.append({"points": pts, "scale": sc})
                if stamp is None and hasattr(m, 'header'):
                    stamp = m.header.stamp
    elif isinstance(msg, Marker):
        if msg.type == Marker.CUBE_LIST and msg.action == Marker.ADD:
            pts = [[p.x, p.y, p.z] for p in msg.points]
            sc = [msg.scale.x, msg.scale.y, msg.scale.z] if msg.scale.x > 0 else [0.1, 0.1, 0.1]
            if pts:
                groups.append({"points": pts, "scale": sc})
            if stamp is None and hasattr(msg, 'header'):
                stamp = msg.header.stamp

    first_scale = groups[0]["scale"] if groups else [0.1, 0.1, 0.1]

    return {
        "groups": groups,
        "scale": first_scale,
        "stamp": stamp if stamp is not None else rospy.Time(0)
    }


class RosBridge:
    def __init__(self):
        self.latest_ros_data: Dict[str, Optional[Dict[str, Any]]] = {
            "occupied": None,
            "preblocked": None,
            "traversable": None,
            "risk_cost": None,
            "local_octomap": None,
            "fused_octomap": None,
            "emergency_stop_free": None,
            "emergency_stop_occupied": None
        }
        # 各图层独立版本号（递增整数），用于前端零拷贝增量检测
        self.layer_versions: Dict[str, int] = {k: 0 for k in self.latest_ros_data}
        
        self.latest_planned_path: List[List[float]] = []
        self.path_version = 0
        self.latest_odom_pose = None
        self.latest_status_text = "就绪"
        self.status_version = 0
        
        # TF 监听与广播
        self.tf_buffer = None
        self.tf_listener = None
        self.tf_broadcaster = None
        self.publish_fake_tf = False
        self.fake_robot_pose = None
        
        # ROS 发布者
        self.ros_pubs: Dict[str, rospy.Publisher] = {}

    def init_ros(self):
        """初始化 ROS 节点、订阅器与发布者"""
        rospy.init_node("web_map_manager", disable_signals=True)
        
        # 1. 初始化发布者
        self.ros_pubs["occupied"] = rospy.Publisher("/edited_occupied_markers", Marker, queue_size=1, latch=True)
        self.ros_pubs["preblocked"] = rospy.Publisher("/edited_preblocked_cells_markers", Marker, queue_size=1, latch=True)
        self.ros_pubs["start_pub"] = rospy.Publisher("/start_point", PointStamped, queue_size=1, latch=True)
        self.ros_pubs["goal_pub"] = rospy.Publisher("/goal_point", PointStamped, queue_size=1, latch=True)
        self.ros_pubs["goal_pose_pub"] = rospy.Publisher("/goal_pose", PoseStamped, queue_size=1, latch=True)
        self.ros_pubs["move_base_simple_goal"] = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
        self.ros_pubs["move_base_action_goal"] = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1, latch=True)

        # 2. 获取话题参数
        occupied_topic = rospy.get_param("~occupied_marker_topic", "/octomap_occupied_markers")
        preblocked_topic = rospy.get_param("~preblocked_marker_topic", "/move_base/preblocked_cells")
        traversable_topic = rospy.get_param("~traversable_marker_topic", "/move_base/traversable_cells")
        risk_cost_topic = rospy.get_param("~risk_cost_topic", "/move_base/risk_cost_cloud")
        octomap_local_topic = rospy.get_param("~octomap_local_topic", "/octomap_local_markers")
        octomap_fused_topic = rospy.get_param("~octomap_fused_topic", "/octomap_fused_markers")
        emergency_stop_topic = rospy.get_param("~emergency_stop_topic", "/move_base/OctoLocalPlanner/emergency_stop_markers")
        path_topic = rospy.get_param("~path_topic", "/move_base/plan")
        odom_topic = rospy.get_param("~odom_topic", "/loc_base")
        status_text_topic = rospy.get_param("~status_text_topic", "/move_base/status_text")
        self.publish_fake_tf = rospy.get_param("~publish_fake_tf", False)

        # 3. 初始化 TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        if self.publish_fake_tf:
            if self.fake_robot_pose is None:
                self.fake_robot_pose = {"x": 0.0, "y": 0.0, "z": 0.0}
            rospy.Timer(rospy.Duration(0.05), self.publish_fake_tf_loop)

        # 4. 注册订阅者
        rospy.Subscriber(occupied_topic, MarkerArray, self._occupied_callback)
        rospy.Subscriber(preblocked_topic, Marker, self._preblocked_callback)
        rospy.Subscriber(traversable_topic, Marker, self._traversable_callback)
        rospy.Subscriber(risk_cost_topic, PointCloud2, self._risk_cost_callback)
        rospy.Subscriber(octomap_local_topic, MarkerArray, self._local_octomap_callback)
        rospy.Subscriber(octomap_fused_topic, MarkerArray, self._fused_octomap_callback)
        rospy.Subscriber(emergency_stop_topic, MarkerArray, self._emergency_stop_callback)
        rospy.Subscriber(path_topic, ROSPath, self._path_callback)
        rospy.Subscriber(odom_topic, Odometry, self._odom_callback)
        rospy.Subscriber(status_text_topic, String, self._status_text_callback)
        
        print(f"[RosBridge] ROS 节点已启动，监听话题就绪 (publish_fake_tf={self.publish_fake_tf})", flush=True)

    # ------------------ 内部回调 ------------------
    def _status_text_callback(self, msg: String):
        if self.latest_status_text != msg.data:
            self.latest_status_text = msg.data
            self.status_version += 1

    def _odom_callback(self, msg: Odometry):
        self.latest_odom_pose = msg.pose.pose

    def _path_callback(self, msg: ROSPath):
        self.latest_planned_path = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
        self.path_version += 1

    def _update_layer(self, name: str, parsed_data: Dict[str, Any]):
        self.latest_ros_data[name] = parsed_data
        self.layer_versions[name] += 1

    def _occupied_callback(self, msg: MarkerArray):
        parsed = parse_marker_or_array(msg)
        if parsed["groups"]:
            self._update_layer("occupied", parsed)

    def _preblocked_callback(self, msg: Marker):
        parsed = parse_marker_or_array(msg)
        if parsed["groups"]:
            self._update_layer("preblocked", parsed)

    def _traversable_callback(self, msg: Marker):
        parsed = parse_marker_or_array(msg)
        if parsed["groups"]:
            self._update_layer("traversable", parsed)

    def _local_octomap_callback(self, msg: MarkerArray):
        parsed = parse_marker_or_array(msg)
        if parsed["groups"]:
            self._update_layer("local_octomap", parsed)

    def _fused_octomap_callback(self, msg: MarkerArray):
        parsed = parse_marker_or_array(msg)
        if parsed["groups"]:
            self._update_layer("fused_octomap", parsed)

    def _emergency_stop_callback(self, msg: MarkerArray):
        free_pts, occ_pts = [], []
        free_scale, occ_scale = [0.06, 0.06, 0.06], [0.10, 0.10, 0.10]
        stamp = msg.markers[0].header.stamp if msg.markers else rospy.Time.now()
        for m in msg.markers:
            if m.action != Marker.ADD:
                continue
            pts = [[p.x, p.y, p.z] for p in m.points]
            sc = [m.scale.x, m.scale.y, m.scale.z] if m.scale.x > 0 else [0.06, 0.06, 0.06]
            if m.ns == "emergency_stop_free" or m.id == 0:
                free_pts.extend(pts)
                free_scale = sc
            elif m.ns == "emergency_stop_occupied" or m.id == 1:
                occ_pts.extend(pts)
                occ_scale = sc

        if free_pts:
            self._update_layer("emergency_stop_free", {
                "groups": [{"points": free_pts, "scale": free_scale}],
                "scale": free_scale,
                "stamp": stamp
            })
        if occ_pts:
            self._update_layer("emergency_stop_occupied", {
                "groups": [{"points": occ_pts, "scale": occ_scale}],
                "scale": occ_scale,
                "stamp": stamp
            })

    def _risk_cost_callback(self, msg: PointCloud2):
        try:
            if hasattr(msg, 'point_step') and msg.point_step == 16:
                data_arr = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
                risk_arr = data_arr[~np.isnan(data_arr).any(axis=1)]
            else:
                risk_records = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
                risk_arr = (np.array([[r[0], r[1], r[2], r[3]] for r in risk_records], dtype=np.float32)
                            if risk_records else np.empty((0, 4), dtype=np.float32))

            pts = risk_arr[:, :3].tolist()
            intensities = risk_arr[:, 3].tolist()
            self._update_layer("risk_cost", {
                "points": pts,
                "intensities": intensities,
                "scale": [0.2, 0.2, 0.2],
                "groups": [{"points": pts, "scale": [0.2, 0.2, 0.2]}],
                "stamp": msg.header.stamp
            })
        except Exception as e:
            print(f"[risk_cost_callback] Error: {e}")

    # ------------------ 伪 TF 广播 ------------------
    def publish_fake_tf_loop(self, event=None):
        if not self.publish_fake_tf or self.tf_broadcaster is None or self.fake_robot_pose is None:
            return
        try:
            now_stamp = rospy.Time.now()
            child_frame = rospy.get_param("~tf_child_frame", "base_footprint")
            
            t_map_odom = TransformStamped()
            t_map_odom.header.stamp = now_stamp
            t_map_odom.header.frame_id = "map"
            t_map_odom.child_frame_id = "odom"
            t_map_odom.transform.rotation.w = 1.0

            t_odom_base = TransformStamped()
            t_odom_base.header.stamp = now_stamp
            t_odom_base.header.frame_id = "odom"
            t_odom_base.child_frame_id = child_frame
            t_odom_base.transform.translation.x = self.fake_robot_pose["x"]
            t_odom_base.transform.translation.y = self.fake_robot_pose["y"]
            t_odom_base.transform.translation.z = self.fake_robot_pose["z"]
            t_odom_base.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform([t_map_odom, t_odom_base])
        except Exception:
            pass

    # ------------------ 状态与数据提供接口 ------------------
    def get_layer_response(self, requested_layers: Optional[List[str]], client_versions: Dict[str, int]) -> Dict[str, Any]:
        """
        核心优化：增量返回图层数据
        - 若 client_version >= 当前版本，返回 {"version": v, "unchanged": True}
        - 消除 points 与 groups 的双重冗余，大幅缩减 JSON 大小与带宽
        """
        all_layers = [
            "occupied", "preblocked", "traversable", "risk_cost",
            "local_octomap", "fused_octomap", "emergency_stop_free", "emergency_stop_occupied"
        ]
        res = {}
        for name in all_layers:
            if requested_layers is not None and name not in requested_layers:
                continue
            
            current_v = self.layer_versions.get(name, 0)
            client_v = client_versions.get(name, -1)
            
            # 若客户端版本一致且已有数据，返回 unchanged 标记，免去整个点云阵列序列化传输
            if client_v >= 0 and client_v == current_v:
                res[name] = {"version": current_v, "unchanged": True}
                continue
                
            data = self.latest_ros_data.get(name)
            if data is None:
                continue

            if name == "risk_cost":
                res[name] = {
                    "version": current_v,
                    "unchanged": False,
                    "scale": data.get("scale", [0.2, 0.2, 0.2]),
                    "points": data.get("points", []),
                    "intensities": data.get("intensities", [])
                }
            else:
                # 剔除冗余的 points 字段，仅下发 groups（groups 中已包含 points 与 scale）
                # groups 为空时兜底回退 points
                groups = data.get("groups", [])
                res[name] = {
                    "version": current_v,
                    "unchanged": False,
                    "scale": data.get("scale", [0.2, 0.2, 0.2]),
                    "groups": groups
                }
        return res

    def lookup_robot_pose(self) -> Tuple[bool, Optional[Dict], Optional[Dict]]:
        """获取当前机器人位姿，优先 Odom，其次 TF"""
        if self.latest_odom_pose is not None:
            p = self.latest_odom_pose.position
            o = self.latest_odom_pose.orientation
            return True, {"x": p.x, "y": p.y, "z": p.z}, {"x": o.x, "y": o.y, "z": o.z, "w": o.w}
        
        if self.tf_buffer is not None:
            try:
                parent_frame = rospy.get_param("~tf_parent_frame", "map")
                candidates = [rospy.get_param("~tf_child_frame", "base_footprint"), "odin1_base_link", "base_link"]
                for child in candidates:
                    try:
                        trans = self.tf_buffer.lookup_transform(parent_frame, child, rospy.Time(0), rospy.Duration(0.05))
                        t = trans.transform.translation
                        r = trans.transform.rotation
                        return True, {"x": t.x, "y": t.y, "z": t.z}, {"x": r.x, "y": r.y, "z": r.z, "w": r.w}
                    except Exception:
                        continue
            except Exception:
                pass
        return False, None, None

    def publish_edited_marker(self, layer_name: str, pts: List[List[float]], scale: List[float], stamp: rospy.Time):
        """发布编辑后的图层 Marker 到 ROS"""
        if layer_name not in self.ros_pubs:
            return
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
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

        self.ros_pubs[layer_name].publish(marker)

    def get_live_frame(self, requested_layers: Optional[List[str]], client_versions: Dict[str, int], client_path_v: int = -1, client_status_v: int = -1) -> Dict[str, Any]:
        """为 WebSocket 构造聚合帧：整合位姿、路径、状态与增量图层"""
        has_pose, pos, ori = self.lookup_robot_pose()
        layers_data = self.get_layer_response(requested_layers, client_versions)

        # 仅在发生变化时才携带完整 path，避免每帧重复发送大数组
        path_data = self.latest_planned_path if (client_path_v < 0 or client_path_v != self.path_version) else None
        status_data = self.latest_status_text if (client_status_v < 0 or client_status_v != self.status_version) else None

        return {
            "pose": {"position": pos, "orientation": ori} if has_pose else None,
            "status_text": status_data,
            "status_version": self.status_version,
            "path": path_data,
            "path_version": self.path_version,
            "layers": layers_data
        }


# 全局单例
ros_bridge = RosBridge()
