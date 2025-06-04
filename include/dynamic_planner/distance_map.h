#pragma once

#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>

class DistanceMap
{
public:
    DistanceMap(int width, int height, double resolution);

    std::vector<double> computeDistanceMap(const nav_msgs::msg::OccupancyGrid &occupancy_map);

private:
    int dist_width;
    int dist_height;
    double dist_resolution;
};

