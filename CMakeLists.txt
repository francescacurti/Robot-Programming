cmake_minimum_required(VERSION 3.16)
project(dynamic_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


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

ament_package()