#include "dynamic_planner/distance_map.h"
#include <queue>
#include <cmath>
#include <limits>

DistanceMap::DistanceMap(int width, int height, double resolution)
    : dist_width(width), dist_height(height), dist_resolution(resolution)
{}

std::vector<double> DistanceMap::computeDistanceMap(const nav_msgs::msg::OccupancyGrid &map)
{
    constexpr double MAX_DIST = std::numeric_limits<double>::max();
    std::vector<double> distance_map(dist_width * dist_height, MAX_DIST);
    std::queue<std::pair<int,int>> queue;

    // Inizializza la coda con le celle occupate (ostacoli) impostando distanza 0
    for (int y = 0; y < dist_height; ++y)
    {
        for (int x = 0; x < dist_width; ++x)
        {
            int index = y * dist_width + x;
            if (map.data[index] >= 100)
            {
                distance_map[index] = 0.0;
                queue.push({x, y});
            }
        }
    }

    // Direzioni 8 connesse (ortogonali + diagonali)
    const std::vector<std::pair<int,int>> neighbors{{1,0}, {-1,0}, {0,1}, {0,-1},{1,1}, {-1,1}, {1,-1}, {-1,-1}};

    while (!queue.empty())
    {
        std::pair<int,int> current = queue.front();
        queue.pop();

        int x = current.first;
        int y = current.second;
        int curr_index = y * dist_width + x;

        for (int i = 0; i < (int)neighbors.size(); i++)
        {
            int dir_x = neighbors[i].first;
            int dir_y = neighbors[i].second;
            int neighbor_x = x + dir_x;
            int neighbor_y = y + dir_y;

            if (neighbor_x < 0 || neighbor_x >= dist_width || neighbor_y < 0 || neighbor_y >= dist_height)
                continue;

            int neighbor_index = neighbor_y * dist_width + neighbor_x;

            double step_cost;
            if (dir_x != 0 && dir_y != 0){
                step_cost = std::sqrt(2.0) * dist_resolution;
            }
            else{
                step_cost = dist_resolution;
            }
            double tentative_dist = distance_map[curr_index] + step_cost;

            if (tentative_dist < distance_map[neighbor_index]){
                distance_map[neighbor_index] = tentative_dist;
                queue.push({neighbor_x, neighbor_y});
            }
        }
    }

    return distance_map;
}
