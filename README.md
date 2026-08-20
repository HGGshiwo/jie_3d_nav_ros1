# jie_3d_nav_ros1

一套基于 ROS 1 Noetic 的 3D 导航 system，通过 Web 界面交互。本项目基于原 ROS 2 版本 [6-robot/jie_3d_nav](https://github.com/6-robot/jie_3d_nav) 迁移完成。本系统已在智元科技 D1 机器狗以及留形科技 Odin 1 空间定位模组上测试通过。

## 依赖包安装 (Ubuntu 20.04 ROS 1 Noetic 版本)

```bash
pip3 install vtk open3d
```

字体安装（可选）
```
sudo apt-get update
sudo apt-get install -y fonts-wqy-zenhei fonts-wqy-microhei 
```

## 功能包构成

本目录包含三个 ROS 1 功能包：

- **`jie_map_msgs`**：地图包保存、加载、导出等自定义服务（srv）接口。
- **`jie_octomap`**：OctoMap 管理包，负责多种地图格式导入、地图包保存/加载、OctoMap 可视化 and 编辑。
- **`octo_planner`**：基于 OctoMap 的 3D 路径规划、路径跟踪控制和 Web 测试/导航 launch。

---

## 系统架构与数据流 (ASCII Art)

```text
+-----------------------------------------------------------------------------------+
| 1. 传感器与硬件输入 (Sensors & Hardware)                                           |
|   [Odin 1 空间定位模组]   [3D/2D 激光雷达 (LiDAR)]     [IMU 惯导]     [底盘轮速里程计] |
+-----------+----------------------+--------------------+--------------------+------+
            |                      |                    |                    |
            v                      v                    v                    v
+-----------------------------------------------------------------------------------+
| 2. 全局定位方案 (Localization Options)                                             |
|                                                                                   |
|  [方案 A: Odin 1 重定位]    [方案 B: 3D LiDAR SLAM]      [方案 C: 2D 栅格定位]     |
|   (odin_ros_driver)         (如 Fast-LIO / hdl)          (如 Gmapping / AMCL)     |
|    加载重定位 .bin 地图      加载先验 .pcd 点云地图       输出 /map (OccupancyGrid)|
+-----------+----------------------+--------------------+------------+--------------+
            |                      |                                 |
            +----------------------+--- TF (map -> odom -> base_link) |
            |                                                        |
            |                                                        v
            |                                        +------------------------------+
            |                                        | 3. 地图转换与导入 (Mapping)  |
            |                                        |                              |
            |    +---------------------+             | [occupancy_grid_to_octomap]  |
            |    | 离线点云文件 (.pcd) | --> [pcd_to] |                              |
            |    +---------------------+             | (接收 /map 动态拉伸转换为     |
            |                                        |  3D OctoMap 栅格地图)        |
            |    +---------------------+             +--------------+---------------+
            |    | Gazebo场景 (.world) | --> [world]                |
            |    +---------------------+                            |
            |                                                       v
            |                                            发布话题 /octomap (Octomap)
            |                                                       |
            |                                                       v
            |                                        +------------------------------+
            |                                        | 4. 导航规划核心 (Planning)   |
            |                                        |                              |
            |                                        |  [jie_path_node (A*规划器)]  |
            |                                        +--------------+---------------+
            |                                                       |
            |                                                       v 发布话题 /planned_path
            |                                                       | (nav_msgs/Path)
            v                                                       v
+-----------+-------------------------------------------------------+---------------+
| 5. 底盘运动控制 (Control)                                                         |
|                                                                                   |
|   [d1_controller (前瞻纯跟踪控制器)] <------------------- (遥控话题: /web_cmd_vel) |
+-----------------------------------+-----------------------------------------------+
                                    |
                                    v 发布底盘控制话题 /cmd_vel (geometry_msgs/Twist)
                            [移动机器狗/机器人底盘]
```

---

## 功能概览

- 将 PCD 点云地图导入为 OctoMap。
- 将 ROS 1 2D 栅格地图（`OccupancyGrid`）导入为 3D OctoMap。
- 将 Gazebo `.world` / `.sdf` 场景转换为 OctoMap。
- 保存、加载 OctoMap 地图包。
- 使用 Qt/VTK GUI 查看和编辑 OctoMap 栅格。
- 使用 Web 页面查看 OctoMap、选择起点/终点并进行路径规划。
- 提供面向安装了留形科技 Odin 1 的 智元 D1 机器狗的导航入口和独立网页测试入口。

---

## 介绍视频

- **Bilibili**：[【开源】基于ROS2 of 3D 导航系统](https://www.bilibili.com/video/BV1jgR9BmELw) *(注：视频为原 ROS 2 版本演示)*
- **YouTube**：[【开源】基于ROS2 of 3D 导航系统](https://www.youtube.com/watch?v=CepO90mzIeI) *(注：视频为原 ROS 2 版本演示)*

---

## 目录结构

```text
jie_3d_nav_ros1/
├── jie_map_msgs/        # 自定义 srv 接口
├── jie_octomap/         # OctoMap 导入、管理、编辑、Web/GUI 工具
├── octo_planner/        # 3D 路径规划、控制器、导航 launch
├── worlds/              # 示例 Gazebo world
└── install_deps_noetic.sh
```

---

## 环境要求

- Ubuntu 20.04
- ROS 1 Noetic
- Catkin (支持 `catkin_make` 或 `catkin build`)
- OctoMap / `octomap_msgs`
- OpenCV
- Open3D C++ 开发库
- PyQt5, VTK, NumPy, Pillow, PyYAML
- **可选**：`rosbridge_server`，用于 Web 页面通过 WebSocket 访问 ROS 1

> [!NOTE]
> 基础编译不需要以下两个包：
> - `d1_bringup`
> - `d1_description`
>
> **注意**：完整智元科技 D1 机器狗导航入口 `octo_planner/launch/nav.launch` 仍然会在运行时使用 `d1_bringup` 和 `d1_description`，因为它会启动 `d1_core` 并读取智元科技 D1 机器狗的 URDF 模型。

---

## 安装依赖

可以使用仓库内脚本安装 ROS 1 常用依赖：

```bash
cd ~/catkin_ws/src/jie_3d_nav_ros1
bash install_deps_noetic.sh
```

如果 CMake 找不到 Open3D，需要额外安装 Open3D C++ 开发库，并确保 `Open3DConfig.cmake` 能被 CMake 找到，例如通过环境变量 `Open3D_DIR` 或在 `CMakeLists.txt` 中指定 `CMAKE_PREFIX_PATH`。

---

## 编译

从 ROS 1 工作空间（Workspace）根目录进行编译：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash

# 使用 catkin_make 编译
catkin_make -DCMAKE_BUILD_TYPE=Release

# 或者使用 catkin build
# catkin build jie_map_msgs jie_octomap octo_planner

source devel/setup.bash
```

如果源码目录移动过导致 CMake 缓存冲突，可以清理后重编：

```bash
catkin_make clean
# 或者直接删除 build 和 devel 目录重编
rm -rf build/ devel/
catkin_make
```

---

## 地图导入

### 1. 导入 PCD 点云地图

```bash
roslaunch jie_octomap import_pcd_map.launch
```

该 launch 会启动：
- `pcd_to_octomap_node`
- `octomap_to_occupied_markers_node`
- `map_package_manager`
- `pcd_map_import_gui`
- `octo_planner/jie_path_node`

### 2. 导入 ROS 2D 栅格地图

```bash
roslaunch jie_octomap import_ros_map.launch
```

该 launch 会启动：
- `occupancy_grid_to_octomap_node`
- `octomap_to_occupied_markers_node`
- `map_package_manager`
- `ros_map_import_gui`
- `octo_planner/jie_path_node`

### 3. 导入 Gazebo World / SDF

```bash
# 启动 Gazebo 物理环境
gazebo worlds/field.world
```

加载包内示例 world 时，推荐使用 `world_name` 参数：

```bash
roslaunch jie_octomap import_gazebo_world.launch world_name:=hello_gazebo.world
```

加载外部 world 文件时，请使用绝对路径：

```bash
roslaunch jie_octomap import_gazebo_world.launch world_file:=/absolute/path/to/map.world
```

如果同时传入 `world_file` 和 `world_name`，优先使用 `world_file`。

`jie_octomap/worlds/` 目录内提供了两个示例 world 文件，并会随 `jie_octomap` 包安装到 share 路径：
- `2_storey.world`：双层建筑/楼层示例。
- `field.world`：场地示例。

加载双层建筑示例：

```bash
roslaunch jie_octomap import_gazebo_world.launch world_name:=2_storey.world
```

加载场地示例：

```bash
roslaunch jie_octomap import_gazebo_world.launch world_name:=hello_gazebo.world
```

该 launch 会启动：
- `world_to_octomap_node`
- `world_selector_gui.py`
- `map_package_manager`
- `octo_planner/jie_path_node`

---

## 地图管理与编辑

OctoMap 管理和编辑主入口：

```bash
roslaunch jie_octomap map_manager.launch
```

该 launch 会启动：
- `map_package_manager`
- `octomap_to_occupied_markers_node`
- `map_viewer_gui`
- 可选启动 `octo_planner/jie_path_node`

`map_viewer_gui` 支持以下功能：
- 打开地图包 / 刷新地图 / 保存地图
- 查看占据、禁行、可通行、风险代价图层
- 编辑栅格状态：`occupied`、`preblocked`、`traversable`、`clear`
- 在地图上直观选择起点、终点、导航目标

---

## Web 可视化

### 1. 加载地图并启动 Web 页面

```bash
roslaunch jie_octomap web_octomap.launch map_package:=/home/ubuntu/catkin_3dnavi/hello_gazebo launch_rosbridge:=true
```

常用参数：
- `map_package`：已保存的地图包目录路径。
- `http_port`：静态 Web 服务端口，默认 `8080`。
- `launch_rosbridge`：是否启动 ROS 1 的 `rosbridge_websocket`。
- `launch_map_gui`：是否同时启动 Qt 保存/加载窗口。

浏览器访问地址：
- `http://localhost:8080`
- `http://<机器人IP>:8080`

### 2. 坐标系与话题规范

#### 坐标变换树（TF Tree）
启动 Gazebo 环境 and 机器人后，请确保系统发布如下 TF 树以在网页端自动显示定位：
`map` -> `odom` -> `base_link`

#### 导航及控制话题
- **路径话题**：规划器向网页和底层发布 `/planned_path` (类型：`nav_msgs/Path`)，作为全局路径，通常需要后接一个局部路径规划器/跟踪器。
- **网页控制话题**：`/web_cmd_vel` (类型：`geometry_msgs/Twist`) 是网页摇杆的手动遥控控制话题。

---

### Web 功能测试

```bash
roslaunch octo_planner web_test.launch
```

`web_test.launch` 用于测试网页访问、地图显示、Web 起终点选择、路径规划和基础控制链路。该 launch 已去除对 `d1_bringup` 和 `d1_description` 硬件依赖，会使用一个最小化的 `base_link` URDF 启动 ROS 1 的 `robot_state_publisher`。

启动前同样需要根据实际环境配置参数文件：
`octo_planner/config/nav_params.yaml`

至少需要部署好以下路径：
- `relocalization_bin_file`：重定位使用的 `.bin` 地图文件。
- `map_package_dir`：已经保存好的 OctoMap 地图包目录。

---

## 智元科技 D1 机器狗完整导航

完整机器人导航入口（连接真实硬件与动力学控制）：

```bash
roslaunch octo_planner nav.launch
```

该 launch 面向智元科技 D1 机器狗实际导航，并结合留形科技 Odin 1 空间定位模组相关驱动流程，会启动或使用：
- `d1_bringup/d1_core`
- `d1_description/urdf/d1.urdf`
- `odin_ros_driver`
- `octo_planner/jie_path_node`
- `octo_planner/d1_controller`
- `jie_octomap/map_package_manager`
- Web viewer 和 `rosbridge_websocket`

运行前需要根据实际环境修改参数文件：
`octo_planner/config/nav_params.yaml`

重点字段说明：
- `relocalization_bin_file`
- `map_package_dir`
- `relocalization_pcd_file`
- `show_rviz`
- `show_map_gui`
- `publish_d1_odom`
- `use_static_odom_to_base`

同时需要确认留形科技 Odin 1 空间定位模组的 ROS 1 驱动配置：
`odin_ros_driver/config/control_command.yaml`

将其中的 `custom_map_mode` 设置为 `2`，即 **Relocalization mode（重定位模式）**。

`octo_planner/config/nav_params.yaml` 中至少需要配置好：
- `relocalization_bin_file`：重定位使用的 `.bin` 地图文件。
- `map_package_dir`：已经保存好的 OctoMap 地图包目录。

如果需要使用 RViz 观察定位与点云对齐效果，还需要部署：
- `relocalization_pcd_file`：用于 RViz 显示的 `.pcd` 点云地图文件。

---

## 其他 Launch 工具

```bash
roslaunch jie_octomap octomap_test.launch
roslaunch jie_octomap octomap_open3d.launch
roslaunch jie_octomap odin1_slam.launch
roslaunch jie_octomap odin1_loc.launch
```

> [!NOTE]
> 其中 `odin1_slam.launch` and `odin1_loc.launch` 面向留形科技 Odin 1 空间定位模组流程，运行时需要确保 ROS 1 环境中已有 `odin_ros_driver`，并可选使用 `odin_costmap` 插件配置。 