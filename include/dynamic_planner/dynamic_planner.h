#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "dynamic_planner/laser_processor.h"
#include "dynamic_planner/distance_map.h"
#include "dynamic_planner/astar_planner.h"

class DynamicPlannerNode : public rclcpp::Node
{
public:
  DynamicPlannerNode();

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void initGlobalMap();
  void updateGlobalMap(const nav_msgs::msg::OccupancyGrid &local_map);
  void publishPath();

  int map_width;
  int map_height;
  float resolution;

  int start_x;
  int start_y;
  int goal_x;
  int goal_y;

  LaserProcessor laser_processor;
  DistanceMap distance_map;
  AStarPlanner astar_planner;

  nav_msgs::msg::OccupancyGrid global_map;
  std::vector<double> distance_map_data;
  nav_msgs::msg::OccupancyGrid last_local_map;

  rclcpp::Time last_update_time;
  std::chrono::milliseconds update_period;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pub;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
};
