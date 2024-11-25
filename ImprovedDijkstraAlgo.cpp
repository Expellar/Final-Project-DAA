#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <utility>
#include <algorithm> // Include this for the reverse function
#include <chrono>    // For measuring execution time
#include <iomanip>   // For formatting output

using namespace std;
using namespace chrono; // For timing

// Define INF as a large value to represent infinity
constexpr int INF = 0x3f3f3f3f;

// iPair ==> Integer Pair
using iPair = pair<int, int>;

// Class representing a graph using adjacency list representation
class Graph {
    int V; // Number of vertices
    vector<list<iPair>> adj; // Adjacency list

public:
    explicit Graph(int V); // Constructor

    void addEdge(int u, int v, int w); // Function to add an edge
    vector<int> shortestPath(int src, int dest); // Find shortest path from src to dest
    void removeEdge(int u, int v); // Function to remove an edge
    void displayGraph(); // Display the adjacency list as a table

    // Getter for the adjacency list
    const vector<list<iPair>>& getAdj() const { return adj; }
};

// Constructor to allocate memory for the adjacency list
Graph::Graph(int V) : V(V), adj(V) {}

// Function to add an edge to the graph
void Graph::addEdge(int u, int v, int w) {
    adj[u].emplace_back(v, w);
    adj[v].emplace_back(u, w); // Since the graph is undirected
}

// Function to remove an edge (simulating an obstacle)
void Graph::removeEdge(int u, int v) {
    adj[u].remove_if([v](const iPair& edge) { return edge.first == v; });
    adj[v].remove_if([u](const iPair& edge) { return edge.first == u; });
}

// Function to find shortest path from src to dest
vector<int> Graph::shortestPath(int src, int dest) {
    // Priority queue to store vertices being processed
    priority_queue<iPair, vector<iPair>, greater<>> pq;

    // Vector to store distances and initialize all distances as INF
    vector<int> dist(V, INF);
    vector<int> parent(V, -1); // To store the path
    dist[src] = 0;
    pq.emplace(0, src);

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (u == dest) break; // Stop if we reached the destination

        // Process each neighbor of u
        for (const auto& [v, weight] : adj[u]) {
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                pq.emplace(dist[v], v);
            }
        }
    }

    // Backtrack from dest to src to get the path
    vector<int> path;
    for (int v = dest; v != -1; v = parent[v])
        path.push_back(v);
    reverse(path.begin(), path.end()); // Use reverse to get the path from src to dest

    return path;
}

// Function to display the graph as an adjacency list
void Graph::displayGraph() {
    cout << "\nFinal Graph (Adjacency List):\n";
    cout << setw(5) << "Node" << " -> Connected Nodes (Weight)\n";
    for (int i = 0; i < V; ++i) {
        cout << setw(5) << i << " -> ";
        for (const auto& [v, w] : adj[i]) {
            cout << "(" << v << ", " << w << ") ";
        }
        cout << "\n";
    }
}

int main() {
    int V = 9; // Number of vertices
    Graph g(V);

    // Edges added to the graph
    g.addEdge(0, 1, 4);
    g.addEdge(0, 7, 8);
    g.addEdge(1, 2, 8);
    g.addEdge(1, 7, 11);
    g.addEdge(2, 3, 7);
    g.addEdge(2, 8, 2);
    g.addEdge(2, 5, 4);
    g.addEdge(3, 4, 9);
    g.addEdge(3, 5, 14);
    g.addEdge(4, 5, 10);
    g.addEdge(5, 6, 2);
    g.addEdge(6, 7, 1);
    g.addEdge(6, 8, 6);
    g.addEdge(7, 8, 7);

    int src = 0, dest = 4;

    // Start timing
    auto start = high_resolution_clock::now();

    // Find initial shortest path
    vector<int> path = g.shortestPath(src, dest);

    // Display the original path
    cout << "\n--- Original Path (Before Any Obstacles) ---\n";
    for (int node : path)
        cout << node << " ";
    cout << "\n";

    vector<int> finalFullPath; // To store the full path including recalculated segments
    int totalDistance = 0;

    // Traverse the path with obstacle detection
cout << "Traversing the path with obstacle detection:\n";
size_t i = 0; // Initialize index for traversal
while (i < path.size() - 1) {
    int u = path[i];
    int v = path[i + 1];

    cout << "Checking edge from " << u << " to " << v << ". Is there an obstacle? (y/n): ";
    char response;
    cin >> response;

    if (response == 'y' || response == 'Y') {
        cout << "Obstacle detected. Recalculating path...\n";
        g.removeEdge(u, v);
        path = g.shortestPath(u, dest); // Recalculate path from current node

        // Check if the path is valid
        if (path.size() <= 1) {
            cout << "No available path to destination.\n";
            return 0;
        }

        // Output the updated path
        cout << "Updated Path: ";
        for (int node : path)
            cout << node << " ";
        cout << "\n";

        // Restart traversal from the new path's beginning
        i = 0;
        continue;
    } else {
        finalFullPath.push_back(u); // Add the current node to the full path
        for (const auto& [nextNode, weight] : g.getAdj()[u]) {
            if (nextNode == v) {
                totalDistance += weight;
                break;
            }
        }
        ++i; // Move to the next edge
    }
}

// Add the final destination node to the full path
finalFullPath.push_back(dest);



    // End timing
    auto end = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end - start);

    // Print the full path from beginning to end
    cout << "\n--- Final Path to Destination ---\n";
    for (int node : finalFullPath)
        cout << node << " ";
    cout << "\n";

    // Output total distance
    cout << "\nTotal Distance: " << totalDistance << "\n";

    // Output execution time
    cout << "Execution Time: " << duration.count() << " ms\n";

    // Display final graph
    g.displayGraph();

    return 0;
}
