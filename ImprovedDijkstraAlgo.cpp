#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <utility>
#include <unordered_set>

using namespace std;

// Define INF as a large value to represent infinity
constexpr int INF = 0x3f3f3f3f;

// iPair ==> Integer Pair
using iPair = pair<int, int>;

// Class representing a graph using adjacency list representation
class Graph {
    int V; // Number of vertices
    vector<list<iPair>> adj; // Adjacency list
    unordered_set<int> blockedNodes; // Set of blocked nodes

public:
    explicit Graph(int V); // Constructor

    void addEdge(int u, int v, int w); // Function to add an edge
    void blockNode(int node); // Function to block a node (simulate obstacle detection)
    void unblockNode(int node); // Function to unblock a node
    void dynamicShortestPath(int src, int target); // Dynamic path with obstacle detection
};

// Constructor to allocate memory for the adjacency list
Graph::Graph(int V) : V(V), adj(V) {}

// Function to add an edge to the graph
void Graph::addEdge(int u, int v, int w) {
    adj[u].emplace_back(v, w);
    adj[v].emplace_back(u, w); // Since the graph is undirected
}

// Function to block a node (simulate detecting an obstacle)
void Graph::blockNode(int node) {
    blockedNodes.insert(node);
}

// Function to unblock a node
void Graph::unblockNode(int node) {
    blockedNodes.erase(node);
}

// Dynamic path planning with obstacle detection
void Graph::dynamicShortestPath(int src, int target) {
    vector<int> dist(V, INF); // Vector to store distances
    priority_queue<iPair, vector<iPair>, greater<>> pq; // Priority queue for processing

    dist[src] = 0;
    pq.emplace(0, src);

    // Process the priority queue
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        // If we reached the target node, stop the search
        if (u == target) {
            cout << "Reached target node " << target << " with distance " << dist[u] << '\n';
            return;
        }

        // Skip processing if the current node is blocked
        if (blockedNodes.count(u)) continue;

        // Iterate through all adjacent vertices of the current vertex
        for (const auto& [v, weight] : adj[u]) {
            // Skip blocked nodes
            if (blockedNodes.count(v)) continue;

            // If a shorter path to v is found
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.emplace(dist[v], v);
            }
        }

        // Simulate obstacle detection
        if (rand() % 5 == 0) { // Randomly detect an obstacle
            int obstacleNode = u + 1; // Let's assume next node is blocked
            if (obstacleNode < V) {
                cout << "Detected obstacle at node " << obstacleNode << '\n';
                blockNode(obstacleNode);

                // Restart the path calculation from the current node
                dist.assign(V, INF);
                dist[u] = 0;
                while (!pq.empty()) pq.pop();
                pq.emplace(0, u);
            }
        }
    }

    // If the target is unreachable
    cout << "Target " << target << " is unreachable due to obstacles.\n";
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

    // Call the dynamicShortestPath function
    g.dynamicShortestPath(0, 8); // Start from node 0, target node 8

    return 0;
}
