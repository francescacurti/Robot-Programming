# Robot-Programming
This is the repository for the Robot Programming exam academic year 2024-2025

4 .cpp files uploaded:
  1. dynamic_planner: it is the code for the node that initializes and updates the global map with dynamic obstacles, computes the distance map, plans a path using the A* algorithm, and publishes the resulting path.
  2. astar_planner: it is the code for implementing the A* algorithm used in dynamic_planner.cpp
  3. laser_processor: it is the code that turns a LaserScan into a 2D map, marking the spots hit by valid laser beams as occupied
  4. distance_map:it is the code that calculates a distance map from obstacles in a grid map, using a breadth-first search algorithm
and the respectively .h files

First commit: upload of the file that implement the dynamic planner. No problem in building process

Next commit: a laser publisher in order to test the dynamic planner
