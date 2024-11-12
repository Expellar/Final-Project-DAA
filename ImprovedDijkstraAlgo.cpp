#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <utility>
#include <algorithm> // Include this for the reverse function

using namespace std;

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

// Driver code
int main() {
    int V = 9; // Number of vertices
    Graph g(V);

    // Add edges to the graph
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

    // Find initial shortest path
    vector<int> path = g.shortestPath(src, dest);

    // Traverse the path with obstacle detection
    cout << "Traversing the path with obstacle detection:\n";
    for (size_t i = 0; i < path.size() - 1; ++i) {
        int u = path[i];
        int v = path[i + 1];

        cout << "Checking edge from " << u << " to " << v << ". Is there an obstacle? (y/n): ";
        char response;
        cin >> response;

        if (response == 'y' || response == 'Y') {
            cout << "Obstacle detected. Recalculating path...\n";
            g.removeEdge(u, v);
            path = g.shortestPath(u, dest); // Recalculate path from current node
            i = -1; // Restart the traversal from the updated path
            if (path.empty()) {
                cout << "No available path to destination.\n";
                return 0;
            }
        }
    }

    // If traversal is successful, print the path
    cout << "Path to destination:\n";
    for (int node : path)
        cout << node << " ";
    cout << "\n";

    return 0;
}
