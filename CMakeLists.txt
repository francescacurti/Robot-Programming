cmake_minimum_required(VERSION 3.16)
project(dynamic_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Nodo principale
add_executable(dynamic_planner_node
  src/dynamic_planner.cpp
  src/astar_planner.cpp
  src/laser_processor.cpp
  src/distance_map.cpp
)

target_include_directories(dynamic_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dynamic_planner_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS dynamic_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Nodo fake_laser_publisher
add_executable(laser_publisher 
  src/laser_publisher.cpp
)

target_include_directories(laser_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(laser_publisher 
  rclcpp 
  sensor_msgs
  tf2
  tf2_ros
  geometry_msgs
)

install(TARGETS
  laser_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
