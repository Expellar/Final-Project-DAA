# Final-Project-DAA Improved Dijkstra Algorithm for Mobile Robot Path Planning and Obstacle Avoidance
# Project Overview
In this project, we aim to demonstrate how the Improved Dijkstra’s Algorithm can be applied to robotic navigation in dynamic environments. The algorithm builds upon the traditional Dijkstra’s Algorithm by incorporating real-time obstacle detection and path recalculations. The simulation highlights the algorithm’s potential for use in autonomous robots, where adaptability is crucial.

This repository contains the implementation of the Improved Dijkstra’s Algorithm, designed for dynamic pathfinding in environments with obstacles. The project includes:

A C++ program to compute the shortest path in a graph while dynamically handling obstacles.
A simulation in CoppeliaSim, showcasing the algorithm in action with a mobile robot navigating a grid environment.
- **ImprovedDijkstraAlgo**: The improved dijkstra's algorithm our team has developed
- **Dijkstra_Improvement.ttt**: The implementation of the algorithm in a simulated virtual robot, in a simulated environment. Using the program CopelliaSim.

## Features
# C++ Implementation
- **Graph Representation**: The graph is represented using an adjacency list.
- **Shortest Path Calculation**: The program calculates the shortest path between a source and a destination node using Dijkstra's algorithm.
- **Dynamic Obstacle Detection**: As you traverse the calculated path, you can specify if an edge is blocked. If an obstacle is detected, the program recalculates the path from the current position.

# CoppeliaSim Integration
- **8x8 Grid Simulation**: The environment is modeled as an 8x8 grid, with each cell representing a physical area.
- **Obstacle Representation**: Cuboids are used to simulate obstacles, dynamically updating the cost matrix.
- **Path Following**: The robot follows the computed path using a PID controller for smooth and precise movement.
- **Dynamic Recalculations**: Obstacles trigger real-time path adjustments, ensuring the robot avoids blocked paths.

## Prerequisites
# For C++ Program
C++ compiler supporting C++11 or later.

# For CoppeliaSim
Install CoppeliaSim from Coppelia Robotics.
Use the provided .ttt file for the simulation setup.
  
## CHow to Run
# C++ Program
Use the following command to compile the program:

```bash
g++ -o improved_dijkstra ImprovedDijkstraAlgo.cpp
./improved_dijkstra

Input
The program starts with a precomputed path.
During traversal, you’ll be prompted to confirm if an edge is blocked (y/n).
Enter y if there is an obstacle on the edge, or n if there is no obstacle.

Output
The recalculated path, total distance, and execution time.

```
# CopelliaSim
- Open CoppeliaSim and load the provided .ttt file.
- Start the simulation to observe the robot navigating the 8x8 grid.
- The robot:
    Detects obstacles dynamically using the cost matrix.
    Recalculates paths using the Improved Dijkstra’s Algorithm.
    Adjusts its movements using a PID controller.

## Code Structure
# C++ Implementation
- Graph Class: 
    **addEdge(u, v, w)**: Adds an edge between nodes u and v with weight w.
    **removeEdge(u, v)**: Removes the edge between nodes u and v.
    **shortestPath(src, dest)**: Computes the shortest path using Dijkstra's Algorithm.
    **displayGraph()**: Displays the adjacency list for visualization.
- Main Function: 
    Initializes the graph, computes the shortest path, and dynamically handles obstacles.

## Results
# Performance:
    The robot successfully navigated the grid while avoiding obstacles.
    Real-time path recalculations ensured efficient navigation.

## Conclusion
This project demonstrates the practicality of the Improved Dijkstra’s Algorithm in dynamic environments. By combining precomputed paths and real-time recalculations, the algorithm bridges the gap between theoretical graph algorithms and real-world robotic applications. This project is still not perfect as there are many things that able to be improved, but we hope that in the future this project would be able to reach the stage where it could be implemented into physical robots for autonomous navigation in a more complex and real world scenarios.

Special thanks to Coppelia Robotics for providing an excellent simulation.
https://github.com/Expellar/Final-Project-DAA

This project provides an accessible and interactive way for students and educators to learn about robotics, pathfinding algorithms, 
and obstacle avoidance. The DIY mobile robot, along with the enhanced Dijkstra algorithm, demonstrates key concepts in robotics and
offers a platform for hands-on learning in STEAM education.