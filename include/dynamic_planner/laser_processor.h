#pragma once

#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>  

class LaserProcessor
{
public:
    LaserProcessor(int width, int height, double resolution);

    nav_msgs::msg::OccupancyGrid processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan);

private:
    int laser_width;
    int laser_height;
    double laser_resolution;
};

