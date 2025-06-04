#pragma once

#include <vector>
#include <utility>
#include "nav_msgs/msg/occupancy_grid.hpp"

class AStarPlanner
{
public:
    AStarPlanner(int width, int height);

    // Restituisce il path come vettore di (x,y) dati start e goal,
    // occupacy grid e la distance map per costi variabili.
    std::vector<std::pair<int,int>> planPath(
        int start_x, int start_y,
        int goal_x, int goal_y,
        const nav_msgs::msg::OccupancyGrid &map,
        const std::vector<double> &distance_map_data);

private:
    int astar_width;
    int astar_height;
    std::vector<double> global_distance_map;
    std::vector<double> computeDijkstraHeuristic(int goal_x, int goal_y);
};
