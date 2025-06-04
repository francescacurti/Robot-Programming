#include "dynamic_planner/laser_processor.h"
#include <cmath>

LaserProcessor::LaserProcessor(int map_width, int map_height, double map_resolution)
    : laser_width(map_width), laser_height(map_height), laser_resolution(map_resolution) {}

nav_msgs::msg::OccupancyGrid LaserProcessor::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
{
    nav_msgs::msg::OccupancyGrid local_grid;
    local_grid.header.frame_id = "map";
    local_grid.info.width = laser_width;
    local_grid.info.height = laser_height;
    local_grid.info.resolution = laser_resolution;
    local_grid.info.origin.position.x = -laser_width * laser_resolution / 2.0;
    local_grid.info.origin.position.y = -laser_height * laser_resolution / 2.0;
    local_grid.info.origin.position.z = 0.0;
    local_grid.info.origin.orientation.w = 1.0;
    local_grid.data.assign(laser_width * laser_height, 0);

    int center_x = laser_width / 2;
    int center_y = laser_height / 2;

    float angle = scan->angle_min;

    for (auto range : scan->ranges)
    {
        if (std::isfinite(range) && range >= scan->range_min && range < scan->range_max)
        {
            int map_x = static_cast<int>(std::round((range * std::cos(angle)) / laser_resolution)) + center_x;
            int map_y = static_cast<int>(std::round((range * std::sin(angle)) / laser_resolution)) + center_y;

            if (map_x >= 0 && map_x < laser_width && map_y >= 0 && map_y < laser_height)
            {
                local_grid.data[map_y * laser_width + map_x] = 100;
            }
        }
        angle += scan->angle_increment;
    }

    return local_grid;
}
