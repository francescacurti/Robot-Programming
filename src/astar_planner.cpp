#include "dynamic_planner/astar_planner.h"
#include <queue>
#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>

AStarPlanner::AStarPlanner(int width, int height)
    : astar_width(width), astar_height(height)
{}

std::vector<double> AStarPlanner::computeDijkstraHeuristic(int goal_x, int goal_y)
{
    struct Cell{
        int x, y;
        double cost;
        bool operator>(const Cell &other) const { return cost > other.cost; } 
    };

    std::vector<double> heuristic(astar_width * astar_height, std::numeric_limits<double>::max());
    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> frontier;

    int goal_idx = goal_y * astar_width + goal_x;
    heuristic[goal_idx] = 0.0;
    frontier.push({goal_x, goal_y, 0.0});

    const std::vector<std::pair<int, int>> moves{{1, 0}, {-1, 0}, {0, 1}, {0, -1},{1, 1}, {-1, 1}, {1, -1}, {-1, -1}};

    while (!frontier.empty()){
        Cell current = frontier.top();
        frontier.pop();

        int curr_index = current.y * astar_width + current.x;

        for (auto[dir_x,dir_y]:moves){
            int neighbor_x = current.x + dir_x;
            int neighbor_y = current.y + dir_y;
            if (neighbor_x < 0 || neighbor_x >= astar_width || neighbor_y < 0 || neighbor_y >= astar_height)
                continue;

            int neighbor_idx = neighbor_y * astar_width + neighbor_x;
            double step_cost;
            if (dir_x != 0 && dir_y != 0){
                step_cost = std::sqrt(2.0);
            }else{
                step_cost = 1.0;
            }
            double new_cost = heuristic[curr_index] + step_cost;

            if (new_cost < heuristic[neighbor_idx])
            {
                heuristic[neighbor_idx] = new_cost;
                frontier.push({neighbor_x, neighbor_y, new_cost});
            }
        }
    }
    return heuristic;
}

std::vector<std::pair<int,int>> AStarPlanner::planPath(
    int start_x, int start_y,
    int goal_x, int goal_y,
    const nav_msgs::msg::OccupancyGrid &map,
    const std::vector<double> &distance_map)
{
    if (global_distance_map.empty()){
        global_distance_map = distance_map;
    }
    else{
        for (int i = 0; i < (int)global_distance_map.size(); ++i){
            global_distance_map[i] = std::min(global_distance_map[i], distance_map[i]);
        }
    }

    struct Node{
        int x, y;
        double cost;
        double priority;
        bool operator>(const Node &other) const { return priority > other.priority; }//un nodo è maggiore di un altro se la sua priorità è maggiore
    };

    // Calcoliamo l’euristica Dijkstra dal goal
    auto heuristic_map = computeDijkstraHeuristic(goal_x, goal_y);

    auto heuristic = [&](int x, int y) -> double {
        return heuristic_map[y * astar_width + x];
    };

    std::vector<double> cost_so_far(astar_width * astar_height, std::numeric_limits<double>::max());
    std::vector<int> parent_x(astar_width * astar_height, -1);
    std::vector<int> parent_y(astar_width * astar_height, -1);
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;

    int start_idx = start_y * astar_width + start_x;
    cost_so_far[start_idx] = 0.0;
    open_list.push({start_x, start_y, 0.0, heuristic(start_x, start_y)});

    const std::vector<std::pair<int,int>> directions{{1,0}, {-1,0}, {0,1}, {0,-1},{1,1}, {-1,1}, {1,-1}, {-1,-1}};

    while (!open_list.empty())
    {
        Node current = open_list.top();
        open_list.pop();

        if (current.x == goal_x && current.y == goal_y)
        {
            std::vector<std::pair<int,int>> path;
            int cx = current.x, cy = current.y;

            while (cx != start_x || cy != start_y)
            {
                path.emplace_back(cx, cy);
                int idx = cy * astar_width + cx;
                cx = parent_x[idx];
                cy = parent_y[idx];
            }

            path.emplace_back(start_x, start_y);
            std::reverse(path.begin(), path.end());
            return path;
        }

        int current_idx = current.y * astar_width + current.x;

        for (auto[dir_x, dir_y]:directions)
        {
            int neighbor_x = current.x + dir_x;
            int neighbor_y = current.y + dir_y;

            if (neighbor_x < 0 || neighbor_x >= astar_width || neighbor_y < 0 || neighbor_y >= astar_height)
                continue;

            int neighbor_idx = neighbor_y * astar_width + neighbor_x;

            if (map.data[neighbor_idx] >= 100){
                continue;
            }

            double move_cost; 
            if(dir_x != 0 && dir_y != 0){
                move_cost = std::sqrt(2);
            }else{
                move_cost = 1.0;
            }

            double safety_distance = 1.0; 
            double dist = global_distance_map[neighbor_idx];
            double extra_cost = 0.0;

            if (dist < safety_distance){
                double penalty = 10.0;
                extra_cost = (safety_distance - dist) / safety_distance * penalty;
            }

            double new_cost = cost_so_far[current_idx] + move_cost + extra_cost;

            if (new_cost < cost_so_far[neighbor_idx]){
                cost_so_far[neighbor_idx] = new_cost;
                parent_x[neighbor_idx] = current.x;
                parent_y[neighbor_idx] = current.y;
                double priority = new_cost + heuristic(neighbor_x, neighbor_y);
                open_list.push({neighbor_x, neighbor_y, new_cost, priority});
            }
        }
    }
    return {};
}