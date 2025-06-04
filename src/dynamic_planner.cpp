#include "dynamic_planner/dynamic_planner.h"
#include "dynamic_planner/distance_map.h"
#include <algorithm>

DynamicPlannerNode::DynamicPlannerNode()
    : Node("dynamic_planner_node"),
      map_width(150),
      map_height(150),
      resolution(0.1),
      laser_processor(map_width, map_height, resolution),
      distance_map(map_width, map_height, resolution),
      astar_planner(map_width, map_height),
      update_period(std::chrono::milliseconds(500))
{
    initGlobalMap();

    start_x = static_cast<int>((0.0 - global_map.info.origin.position.x) / resolution);
    start_y = static_cast<int>((0.0 - global_map.info.origin.position.y) / resolution);

    goal_x = static_cast<int>((4.0 - global_map.info.origin.position.x) / resolution);
    goal_y = static_cast<int>((4.0 - global_map.info.origin.position.y) / resolution);

    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&DynamicPlannerNode::laserCallback, this, std::placeholders::_1));
    goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&DynamicPlannerNode::goalCallback, this, std::placeholders::_1));

    map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    marker_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/marker_pose", 10);
    last_update_time = this->get_clock()->now();
}

void DynamicPlannerNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    auto current_time = this->get_clock()->now();
    auto time_diff = current_time - last_update_time;

    if (time_diff >= rclcpp::Duration(update_period))
    {
        last_local_map = laser_processor.processLaserScan(scan_msg);
        last_update_time = current_time;
    }

    updateGlobalMap(last_local_map);
    distance_map_data = distance_map.computeDistanceMap(global_map);
    global_map.header.stamp = current_time;
    map_pub->publish(global_map);
    publishPath();
}

void DynamicPlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
{
    goal_x = static_cast<int>((goal_msg->pose.position.x - global_map.info.origin.position.x) / resolution);
    goal_y = static_cast<int>((goal_msg->pose.position.y - global_map.info.origin.position.y) / resolution);

    marker_pub->publish(*goal_msg);
    publishPath();
}

void DynamicPlannerNode::initGlobalMap()
{
    global_map.header.frame_id = "map";
    global_map.info.width = map_width;
    global_map.info.height = map_height;
    global_map.info.resolution = resolution;
    global_map.info.origin.position.x = -(map_width * resolution) / 2.0;
    global_map.info.origin.position.y = -(map_height * resolution) / 2.0;
    global_map.info.origin.position.z = 0.0;
    global_map.info.origin.orientation.w = 1.0;
    global_map.data.assign(map_width * map_height, -1);
    RCLCPP_INFO(this->get_logger(), "Map initialized");
}

void DynamicPlannerNode::updateGlobalMap(const nav_msgs::msg::OccupancyGrid &local_map)
{
    int offset_x = static_cast<int>((local_map.info.origin.position.x - global_map.info.origin.position.x) / resolution);
    int offset_y = static_cast<int>((local_map.info.origin.position.y - global_map.info.origin.position.y) / resolution);

    for (int y = 0; y < local_map.info.height; ++y)
    {
        for (int x = 0; x < local_map.info.width; ++x)
        {
            int local_idx = y * local_map.info.width + x;
            int global_x = x + offset_x;
            int global_y = y + offset_y;

            if (global_x >= 0 && global_x < map_width && global_y >= 0 && global_y < map_height)
            {
                int global_idx = global_y * map_width + global_x;
                int value = local_map.data[local_idx];
                if (value != -1)
                {
                    global_map.data[global_idx] = value;
                }
            }
        }
    }
}

void DynamicPlannerNode::publishPath()
{
    auto path = astar_planner.planPath(start_x, start_y, goal_x, goal_y, global_map, distance_map_data);
    nav_msgs::msg::Path planned_path;
    planned_path.header.frame_id = "map";
    planned_path.header.stamp = this->get_clock()->now();

    for (auto[x, y] : path)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = planned_path.header;
        pose.pose.position.x = x * resolution + global_map.info.origin.position.x;
        pose.pose.position.y = y * resolution + global_map.info.origin.position.y;
        pose.pose.orientation.w = 1.0;
        planned_path.poses.push_back(pose);
    }

    path_pub->publish(planned_path);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
