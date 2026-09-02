cmake_minimum_required(VERSION 3.0.2)
project(octo_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  jie_map_msgs
  octomap_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  nav_core
  pluginlib
  costmap_2d
)

find_package(OCTOMAP REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES octo_planner_core
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    jie_map_msgs
    octomap_msgs
    visualization_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    nav_core
    pluginlib
    costmap_2d
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ---- octo_planner_core ----
add_library(octo_planner_core src/octo_planner_core.cpp)
target_link_libraries(octo_planner_core
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(octo_planner_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- jie_path_node ----
add_executable(jie_path_node
  src/jie_path_node.cpp
  src/jie_path_node_callbacks.cpp
  src/jie_path_node_services.cpp
)
target_link_libraries(jie_path_node
  octo_planner_core
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(jie_path_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- octo_global_planner_plugin ----
add_library(octo_global_planner_plugin src/octo_global_planner.cpp)
target_link_libraries(octo_global_planner_plugin
  octo_planner_core
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(octo_global_planner_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- d1_controller ----
add_executable(d1_controller src/d1_controller.cpp)
target_link_libraries(d1_controller
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(d1_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- d1_local_planner_plugin ----
add_library(d1_local_planner_plugin src/d1_local_planner.cpp)
target_link_libraries(d1_local_planner_plugin
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(d1_local_planner_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- test_map_to_odom_tf_node ----
add_executable(test_map_to_odom_tf_node src/test_map_to_odom_tf_node.cpp)
target_link_libraries(test_map_to_odom_tf_node ${catkin_LIBRARIES})
add_dependencies(test_map_to_odom_tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS
  octo_planner_core
  octo_global_planner_plugin
  d1_local_planner_plugin
  jie_path_node
  d1_controller
  test_map_to_odom_tf_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)