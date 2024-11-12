# Final-Project-DAA
Improved Dijkstra Algorithm for Mobile Robot Path Planning and Obstacle Avoidance

# Graph Shortest Path with Obstacle Detection

This C++ program represents a graph, finds the shortest path between two nodes, and can dynamically detect obstacles on the path, recalculating if an edge is blocked.

## Features

- **Graph Representation**: The graph is represented using an adjacency list.
- **Shortest Path Calculation**: The program calculates the shortest path between a source and a destination node using Dijkstra's algorithm.
- **Dynamic Obstacle Detection**: As you traverse the calculated path, you can specify if an edge is blocked. If an obstacle is detected, the program recalculates the path from the current position.


# Prerequisites

- C++ compiler supporting C++11 or later.
  
# Compilation

Use the following command to compile the program:

```bash
g++ -o graph_shortest_path graph_shortest_path.cpp

Input
The program initially defines a graph with vertices and edges.
After computing the shortest path, it will prompt you to confirm if there are obstacles on specific edges along the path.
Enter y if there is an obstacle on the edge, or n if there is no obstacle.

Output
The program will print the calculated path from the source to the destination.
If an obstacle is detected, the program recalculates the path and tries to find an alternate route.

Classes and Functions
    Graph Class: Represents the graph using an adjacency list.
    addEdge(int u, int v, int w): Adds an undirected edge between nodes u and v with weight w.
    removeEdge(int u, int v): Removes the edge between nodes u and v.
    shortestPath(int src, int dest): Uses Dijkstra’s algorithm to find the shortest path from the source src to the destination dest.
    main() Function: Sets up the graph, calculates the shortest path, and handles user input for obstacle detection.

Code Structure
    Graph Initialization: A graph with 9 vertices is created, and edges with weights are added.
    Shortest Path Calculation: The initial shortest path from src to dest is computed.
    Obstacle Detection Loop: Each edge along the path is checked for obstacles, with recalculations if an edge is blocked.
    Customization

To modify the graph:
    Adjust the number of vertices in the main function.
    Add or remove edges using the addEdge function.
    Set different source and destination nodes.

https://github.com/Expellar/Final-Project-DAA

This project provides an accessible and interactive way for students and educators to learn about robotics, pathfinding algorithms, 
and obstacle avoidance. The DIY mobile robot, along with the enhanced Dijkstra algorithm, demonstrates key concepts in robotics and
offers a platform for hands-on learning in STEAM education.